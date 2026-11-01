/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/*
 * USB CDC-ACM device on the STM32H743 USB2 OTG_FS core (Synopsys DWC2),
 * register level, slave mode (no DMA), no external USB stack.
 *
 * Endpoints: EP0 control, EP1 OUT/IN bulk 64 bytes (data), EP2 IN interrupt
 * 8 bytes (CDC notifications, declared but never used).
 *
 * All endpoint and FIFO access happens either in the interrupt handler or
 * with interrupts disabled, so the core never sees interleaved FIFO writes.
 *
 * C++ only because this port's vector table uses C++ handler names.
 */

#include "usb_cdc.h"
#include "peripherals/gpio.h"
#include "interfaces/delays.h"
#include "stm32h7xx.h"
#include <string.h>
#include <errno.h>

#define USB_BASE USB2_OTG_FS_PERIPH_BASE
#define CORE USB2_OTG_FS
#define DEV ((USB_OTG_DeviceTypeDef *)(USB_BASE + USB_OTG_DEVICE_BASE))
#define INEP(i)                                                        \
    ((USB_OTG_INEndpointTypeDef *)(USB_BASE + USB_OTG_IN_ENDPOINT_BASE \
                                   + ((i) * 0x20)))
#define OUTEP(i)                                                         \
    ((USB_OTG_OUTEndpointTypeDef *)(USB_BASE + USB_OTG_OUT_ENDPOINT_BASE \
                                    + ((i) * 0x20)))
#ifndef FIFO // a host test supplies its own
#define FIFO(i) \
    (*(volatile uint32_t *)(USB_BASE + USB_OTG_FIFO_BASE + ((i) * 0x1000)))
#endif
#define PCGCCTL (*(volatile uint32_t *)(USB_BASE + USB_OTG_PCGCCTL_BASE))

#define EP0_SIZE 64
#define DATA_SIZE 64
#define NOTIF_SIZE 8

// FIFO RAM layout, in 32-bit words (4 KiB = 1024 words available)
#define RX_FIFO_WORDS 128
#define TX0_FIFO_WORDS 64
#define TX1_FIFO_WORDS 128
#define TX2_FIFO_WORDS 16

#define TX_RING_SIZE 2048 // powers of two
#define RX_RING_SIZE 256

#define USB_VID 0x0483
#define USB_PID 0x5740

// GRXSTSP packet status values
#define PKT_OUT_DATA 2
#define PKT_SETUP_DATA 6

// Interrupt flags handled by the driver
#define GINT_MASK                                        \
    (USB_OTG_GINTMSK_USBRST | USB_OTG_GINTMSK_ENUMDNEM   \
     | USB_OTG_GINTMSK_RXFLVLM | USB_OTG_GINTMSK_OEPINT  \
     | USB_OTG_GINTMSK_IEPINT | USB_OTG_GINTMSK_USBSUSPM \
     | USB_OTG_GINTMSK_WUIM)

enum Ep0State {
    EP0_IDLE,     // waiting for SETUP
    EP0_DATA_IN,  // sending the data stage
    EP0_DATA_OUT, // receiving the data stage (SET_LINE_CODING)
    EP0_STATUS    // status stage in progress
};

static const uint8_t devDesc[] = {
    18,
    1,    // bLength, DEVICE
    0x00,
    0x02, // USB 2.0
    0x02,
    0x00,
    0x00, // class CDC, no subclass, no protocol
    EP0_SIZE,
    USB_VID & 0xFF,
    USB_VID >> 8,
    USB_PID & 0xFF,
    USB_PID >> 8,
    0x00,
    0x01, // bcdDevice 1.00
    1,
    2,
    3, // manufacturer, product, serial strings
    1  // one configuration
};

static const uint8_t cfgDesc[] = {
    9,          2,    67,   0,          // CONFIGURATION, total length 67
    2,          1,    0,                // 2 interfaces, value 1, no string
    0xC0,       50,                     // self powered, 100 mA

    9,          4,    0,    0,    1,    // INTERFACE 0, 1 endpoint
    0x02,       0x02, 0x01, 0,          // CDC, ACM, AT commands

    5,          0x24, 0x00, 0x10, 0x01, // header, CDC 1.10
    5,          0x24, 0x01, 0x00, 0x01, // call management, data interface 1
    4,          0x24, 0x02, 0x02, // ACM: line coding and control line state
    5,          0x24, 0x06, 0x00, 0x01,             // union: master 0, slave 1

    7,          5,    0x82, 0x03,                   // ENDPOINT 2 IN, interrupt
    NOTIF_SIZE, 0,    16,                           // 8 bytes, every 16 ms

    9,          4,    1,    0,    2,                // INTERFACE 1, 2 endpoints
    0x0A,       0x00, 0x00, 0,                      // CDC data

    7,          5,    0x01, 0x02,                   // ENDPOINT 1 OUT, bulk
    DATA_SIZE,  0,    0,    7,    5,    0x81, 0x02, // ENDPOINT 1 IN, bulk
    DATA_SIZE,  0,    0
};

static const char *const strings[] = { "OpenRTX", "OpenRTX rtxlink" };

static uint8_t ep0Buf[128];
static uint8_t setup[8];
static enum Ep0State ep0State;
static uint8_t lineCoding[7] = { 0x00, 0xC2, 0x01, 0x00, 0, 0, 8 };

static volatile bool configured;
static volatile bool dtr;
static volatile bool suspended;
static volatile bool initialized;

static uint8_t txRing[TX_RING_SIZE];
static volatile uint32_t txHead; // written by thread code
static volatile uint32_t txTail; // written by the interrupt handler
static volatile bool txBusy;
static volatile bool txZlp;      // last packet was full-size

static uint8_t rxRing[RX_RING_SIZE];
static volatile uint32_t rxHead; // written by the interrupt handler
static volatile uint32_t rxTail; // written by thread code
static volatile bool rxArmed;

static inline uint32_t irqLock()
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static inline void irqUnlock(uint32_t key)
{
    __set_PRIMASK(key);
}

static bool waitBits(volatile uint32_t *reg, uint32_t mask, uint32_t value,
                     uint32_t tries = 2000000)
{
    for (uint32_t i = 0; i < tries; i++) {
        if (((*reg) & mask) == value)
            return true;
    }

    return false;
}

// FIFO helpers -------------------------------------------------------------

static void fifoWrite(uint8_t ep, const uint8_t *data, uint32_t len)
{
    for (uint32_t i = 0; i < len; i += 4) {
        uint32_t word = 0;
        memcpy(&word, &data[i], ((len - i) >= 4) ? 4 : (len - i));
        FIFO(ep) = word;
    }
}

static void fifoRead(uint8_t *data, uint32_t len)
{
    for (uint32_t i = 0; i < len; i += 4) {
        uint32_t word = FIFO(0);
        memcpy(&data[i], &word, ((len - i) >= 4) ? 4 : (len - i));
    }
}

static void fifoDiscard(uint32_t len)
{
    for (uint32_t i = 0; i < len; i += 4) {
        uint32_t word = FIFO(0);
        (void)word;
    }
}

// Endpoint 0 ---------------------------------------------------------------

static void ep0ArmOut(uint32_t len)
{
    OUTEP(0)->DOEPTSIZ = (3u << USB_OTG_DOEPTSIZ_STUPCNT_Pos)
                       | (1u << USB_OTG_DOEPTSIZ_PKTCNT_Pos) | len;
    OUTEP(0)->DOEPCTL |= USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA;
}

static void ep0Send(const uint8_t *data, uint32_t len)
{
    uint16_t wLength = setup[6] | (setup[7] << 8);
    if (len > wLength)
        len = wLength;

    if (len > 127)
        len = 127;

    // One transfer of up to 127 bytes; the core splits it into packets
    uint32_t pkts = (len == 0) ? 1 : ((len + EP0_SIZE - 1) / EP0_SIZE);
    INEP(0)->DIEPTSIZ = (pkts << USB_OTG_DIEPTSIZ_PKTCNT_Pos) | len;
    INEP(0)->DIEPCTL |= USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA;
    fifoWrite(0, data, len);

    ep0State = (len > 0) ? EP0_DATA_IN : EP0_STATUS;

    // Accept the host's status-stage ZLP (or the next SETUP)
    ep0ArmOut(EP0_SIZE);
}

static inline void ep0Ack()
{
    ep0Send(NULL, 0);
}

static void ep0Stall()
{
    INEP(0)->DIEPCTL |= USB_OTG_DIEPCTL_STALL;
    OUTEP(0)->DOEPCTL |= USB_OTG_DOEPCTL_STALL;
    ep0State = EP0_IDLE;
    ep0ArmOut(EP0_SIZE);
}

static uint32_t stringDesc(uint8_t index)
{
    uint32_t len = 0;

    if (index == 0) {
        ep0Buf[2] = 0x09; // English (US)
        ep0Buf[3] = 0x04;
        len = 4;
    } else if (index <= 2) {
        const char *s = strings[index - 1];
        len = 2;
        while (*s != '\0') {
            ep0Buf[len++] = (uint8_t)*s++;
            ep0Buf[len++] = 0;
        }
    } else if (index == 3) {
        // Serial number: the 96-bit unique ID as 24 hex digits
        static const char hex[] = "0123456789ABCDEF";
        const uint8_t *uid = (const uint8_t *)UID_BASE;
        len = 2;
        for (int i = 11; i >= 0; i--) {
            ep0Buf[len++] = hex[uid[i] >> 4];
            ep0Buf[len++] = 0;
            ep0Buf[len++] = hex[uid[i] & 0x0F];
            ep0Buf[len++] = 0;
        }
    } else {
        return 0;
    }

    ep0Buf[0] = len;
    ep0Buf[1] = 3; // STRING
    return len;
}

// Data endpoints -----------------------------------------------------------

static void rxArm()
{
    if ((RX_RING_SIZE - (rxHead - rxTail)) < DATA_SIZE) {
        rxArmed = false; // read() re-arms once there is room
        return;
    }

    OUTEP(1)->DOEPTSIZ = (1u << USB_OTG_DOEPTSIZ_PKTCNT_Pos) | DATA_SIZE;
    OUTEP(1)->DOEPCTL |= USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA;
    rxArmed = true;
}

/** Start the next bulk IN packet if possible. Interrupts must be disabled. */
static void txKick()
{
    if (!configured || !dtr || suspended || txBusy)
        return;

    uint32_t len = txHead - txTail;
    if (len == 0) {
        if (!txZlp)
            return;

        txZlp = false; // end the transfer after a full-size packet
    }

    if (len > DATA_SIZE)
        len = DATA_SIZE;

    INEP(1)->DIEPTSIZ = (1u << USB_OTG_DIEPTSIZ_PKTCNT_Pos) | len;
    INEP(1)->DIEPCTL |= USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA;

    for (uint32_t i = 0; i < len; i += 4) {
        uint32_t word = 0;
        for (uint32_t j = 0; (j < 4) && ((i + j) < len); j++) {
            uint8_t b = txRing[(txTail + i + j) & (TX_RING_SIZE - 1)];
            word |= ((uint32_t)b) << (8 * j);
        }

        FIFO(1) = word;
    }

    txTail = txTail + len;
    txBusy = true;
    txZlp = (len == DATA_SIZE);
}

static void configureEndpoints()
{
    INEP(1)->DIEPCTL = USB_OTG_DIEPCTL_USBAEP
                     | (2u << USB_OTG_DIEPCTL_EPTYP_Pos) // bulk
                     | (1u << USB_OTG_DIEPCTL_TXFNUM_Pos)
                     | USB_OTG_DIEPCTL_SD0PID_SEVNFRM | DATA_SIZE;

    INEP(2)->DIEPCTL = USB_OTG_DIEPCTL_USBAEP
                     | (3u << USB_OTG_DIEPCTL_EPTYP_Pos) // interrupt
                     | (2u << USB_OTG_DIEPCTL_TXFNUM_Pos)
                     | USB_OTG_DIEPCTL_SD0PID_SEVNFRM | NOTIF_SIZE;

    OUTEP(1)->DOEPCTL = USB_OTG_DOEPCTL_USBAEP
                      | (2u << USB_OTG_DOEPCTL_EPTYP_Pos) // bulk
                      | USB_OTG_DOEPCTL_SD0PID_SEVNFRM | DATA_SIZE;

    DEV->DAINTMSK |= (1u << 1) | (1u << 2) | (1u << (16 + 1));

    txBusy = false;
    txZlp = false;
    rxArm();
}

static void deconfigureEndpoints()
{
    // A packet may still be armed on the bulk IN endpoint
    if (INEP(1)->DIEPCTL & USB_OTG_DIEPCTL_EPENA) {
        INEP(1)->DIEPCTL |= USB_OTG_DIEPCTL_EPDIS | USB_OTG_DIEPCTL_SNAK;
        waitBits(&INEP(1)->DIEPINT, USB_OTG_DIEPINT_EPDISD,
                 USB_OTG_DIEPINT_EPDISD, 10000);
        INEP(1)->DIEPINT = USB_OTG_DIEPINT_EPDISD;
    }

    INEP(1)->DIEPCTL = 0;
    INEP(2)->DIEPCTL = 0;
    OUTEP(1)->DOEPCTL = 0;
    DEV->DAINTMSK &= ~((1u << 1) | (1u << 2) | (1u << (16 + 1)));
    configured = false;
    dtr = false;
    txBusy = false;
}

// Control requests ---------------------------------------------------------

static void handleSetup()
{
    uint8_t bmRequestType = setup[0];
    uint8_t bRequest = setup[1];
    uint16_t wValue = setup[2] | (setup[3] << 8);
    uint16_t wLength = setup[6] | (setup[7] << 8);

    if ((bmRequestType & 0x60) == 0x20) // class request
    {
        switch (bRequest) {
            case 0x20: // SET_LINE_CODING, 7 bytes follow
                if (wLength != sizeof(lineCoding))
                    break;

                ep0State = EP0_DATA_OUT;
                ep0ArmOut(sizeof(lineCoding));
                return;

            case 0x21: // GET_LINE_CODING
                memcpy(ep0Buf, lineCoding, sizeof(lineCoding));
                ep0Send(ep0Buf, sizeof(lineCoding));
                return;

            case 0x22: // SET_CONTROL_LINE_STATE
                dtr = (wValue & 0x01) != 0;
                if (dtr == false)
                    txZlp = false;
                ep0Ack();
                return;

            case 0x23: // SEND_BREAK
                ep0Ack();
                return;

            default:
                break;
        }

        ep0Stall();
        return;
    }

    if ((bmRequestType & 0x60) != 0x00) // vendor or reserved
    {
        ep0Stall();
        return;
    }

    switch (bRequest) {
        case 0x00: // GET_STATUS
            ep0Buf[0] = ((bmRequestType & 0x1F) == 0) ? 0x01 : 0x00;
            ep0Buf[1] = 0;
            ep0Send(ep0Buf, 2);
            return;

        case 0x01: // CLEAR_FEATURE
            // ENDPOINT_HALT on a data endpoint: the host resets its data
            // toggle, so must we, or the next packet is silently dropped
            if (((bmRequestType & 0x1F) == 0x02) && (wValue == 0)) {
                uint8_t ep = setup[4] & 0x0F;
                if ((ep == 1) || (ep == 2)) {
                    if (setup[4] & 0x80) {
                        INEP(ep)->DIEPCTL &= ~USB_OTG_DIEPCTL_STALL;
                        INEP(ep)->DIEPCTL |= USB_OTG_DIEPCTL_SD0PID_SEVNFRM;
                    } else {
                        OUTEP(ep)->DOEPCTL &= ~USB_OTG_DOEPCTL_STALL;
                        OUTEP(ep)->DOEPCTL |= USB_OTG_DOEPCTL_SD0PID_SEVNFRM;
                    }
                }
            }

            ep0Ack();
            return;

        case 0x03: // SET_FEATURE
            ep0Ack();
            return;

        case 0x05: // SET_ADDRESS: the DWC2 core takes it before the status
            DEV->DCFG = (DEV->DCFG & ~USB_OTG_DCFG_DAD)
                      | ((wValue & 0x7F) << USB_OTG_DCFG_DAD_Pos);
            ep0Ack();
            return;

        case 0x06: // GET_DESCRIPTOR
        {
            uint8_t type = wValue >> 8;
            uint8_t index = wValue & 0xFF;

            if (type == 1) {
                memcpy(ep0Buf, devDesc, sizeof(devDesc));
                ep0Send(ep0Buf, sizeof(devDesc));
                return;
            }

            if (type == 2) {
                memcpy(ep0Buf, cfgDesc, sizeof(cfgDesc));
                ep0Send(ep0Buf, sizeof(cfgDesc));
                return;
            }

            if (type == 3) {
                uint32_t len = stringDesc(index);
                if (len > 0) {
                    ep0Send(ep0Buf, len);
                    return;
                }
            }

            break; // device qualifier and others: full-speed only
        }

        case 0x08: // GET_CONFIGURATION
            ep0Buf[0] = configured ? 1 : 0;
            ep0Send(ep0Buf, 1);
            return;

        case 0x09: // SET_CONFIGURATION
            if (wValue == 1) {
                configureEndpoints();
                configured = true;
            } else {
                deconfigureEndpoints();
            }

            ep0Ack();
            return;

        case 0x0A: // GET_INTERFACE
            ep0Buf[0] = 0;
            ep0Send(ep0Buf, 1);
            return;

        case 0x0B: // SET_INTERFACE
            ep0Ack();
            return;

        default:
            break;
    }

    ep0Stall();
}

// Interrupt handling -------------------------------------------------------

static void flushTxFifos()
{
    CORE->GRSTCTL = USB_OTG_GRSTCTL_TXFFLSH
                  | (0x10u << USB_OTG_GRSTCTL_TXFNUM_Pos); // all
    waitBits(&CORE->GRSTCTL, USB_OTG_GRSTCTL_TXFFLSH, 0);
}

static void busReset()
{
    flushTxFifos();

    for (int i = 0; i < 4; i++) {
        INEP(i)->DIEPINT = 0xFB7F;
        INEP(i)->DIEPCTL &= ~USB_OTG_DIEPCTL_STALL;
        OUTEP(i)->DOEPINT = 0xFB7F;
        OUTEP(i)->DOEPCTL &= ~USB_OTG_DOEPCTL_STALL;
        OUTEP(i)->DOEPCTL |= USB_OTG_DOEPCTL_SNAK;
    }

    deconfigureEndpoints();
    suspended = false;

    DEV->DAINTMSK = (1u << 0) | (1u << 16);
    DEV->DOEPMSK = USB_OTG_DOEPMSK_STUPM | USB_OTG_DOEPMSK_XFRCM;
    DEV->DIEPMSK = USB_OTG_DIEPMSK_XFRCM;
    DEV->DCFG &= ~USB_OTG_DCFG_DAD;

    ep0State = EP0_IDLE;
    ep0ArmOut(EP0_SIZE);
}

static void rxFifoLevel()
{
    uint32_t status = CORE->GRXSTSP;
    uint8_t ep = status & USB_OTG_GRXSTSP_EPNUM;
    uint32_t count = (status & USB_OTG_GRXSTSP_BCNT)
                  >> USB_OTG_GRXSTSP_BCNT_Pos;
    uint32_t pkt = (status & USB_OTG_GRXSTSP_PKTSTS)
                >> USB_OTG_GRXSTSP_PKTSTS_Pos;

    if ((pkt == PKT_SETUP_DATA) && (count == sizeof(setup))) {
        fifoRead(setup, sizeof(setup));
        return;
    }

    if (pkt != PKT_OUT_DATA)
        return; // completion entries carry no data

    if (ep == 0) {
        if (count <= sizeof(ep0Buf))
            fifoRead(ep0Buf, count);
        else
            fifoDiscard(count);

        if ((ep0State == EP0_DATA_OUT) && (count == sizeof(lineCoding)))
            memcpy(lineCoding, ep0Buf, sizeof(lineCoding));

        return;
    }

    if (ep == 1) {
        // Room for a full packet was checked when the endpoint was armed
        for (uint32_t i = 0; i < count; i += 4) {
            uint32_t word = FIFO(0);
            for (uint32_t j = 0; (j < 4) && ((i + j) < count); j++) {
                rxRing[rxHead & (RX_RING_SIZE - 1)] = (uint8_t)(word
                                                                >> (8 * j));
                rxHead = rxHead + 1;
            }
        }

        return;
    }

    fifoDiscard(count);
}

static void outEndpoints()
{
    uint32_t pending = (DEV->DAINT & DEV->DAINTMSK) >> 16;

    if (pending & (1u << 0)) {
        uint32_t flags = OUTEP(0)->DOEPINT;
        OUTEP(0)->DOEPINT = flags;

        if (flags & USB_OTG_DOEPINT_XFRC) {
            if (ep0State == EP0_DATA_OUT)
                ep0Ack();            // line coding stored, send status
            else if (ep0State == EP0_DATA_IN)
                ep0State = EP0_IDLE; // host's status ZLP received
        }

        if (flags & USB_OTG_DOEPINT_STUP) {
            ep0State = EP0_IDLE;
            handleSetup();
        }
    }

    if (pending & (1u << 1)) {
        uint32_t flags = OUTEP(1)->DOEPINT;
        OUTEP(1)->DOEPINT = flags;

        if (flags & USB_OTG_DOEPINT_XFRC)
            rxArm();
    }
}

static void inEndpoints()
{
    uint32_t pending = DEV->DAINT & DEV->DAINTMSK & 0xFFFF;

    if (pending & (1u << 0)) {
        uint32_t flags = INEP(0)->DIEPINT;
        INEP(0)->DIEPINT = flags;

        if ((flags & USB_OTG_DIEPINT_XFRC) && (ep0State == EP0_STATUS))
            ep0State = EP0_IDLE;
    }

    if (pending & (1u << 1)) {
        uint32_t flags = INEP(1)->DIEPINT;
        INEP(1)->DIEPINT = flags;

        if (flags & USB_OTG_DIEPINT_XFRC) {
            txBusy = false;
            txKick();
        }
    }

    if (pending & (1u << 2))
        INEP(2)->DIEPINT = INEP(2)->DIEPINT;
}

void OTG_FS_IRQHandler()
{
    uint32_t status = CORE->GINTSTS & CORE->GINTMSK;

    if (status & USB_OTG_GINTSTS_USBRST) {
        CORE->GINTSTS = USB_OTG_GINTSTS_USBRST;
        busReset();
    }

    if (status & USB_OTG_GINTSTS_ENUMDNE) {
        CORE->GINTSTS = USB_OTG_GINTSTS_ENUMDNE;
        INEP(0)->DIEPCTL &= ~USB_OTG_DIEPCTL_MPSIZ; // 64 bytes
        DEV->DCTL |= USB_OTG_DCTL_CGINAK;
    }

    while (CORE->GINTSTS & USB_OTG_GINTSTS_RXFLVL)
        rxFifoLevel();

    if (status & USB_OTG_GINTSTS_OEPINT)
        outEndpoints();

    if (status & USB_OTG_GINTSTS_IEPINT)
        inEndpoints();

    if (status & USB_OTG_GINTSTS_USBSUSP) {
        CORE->GINTSTS = USB_OTG_GINTSTS_USBSUSP;
        suspended = true; // also what a cable unplug looks like
    }

    if (status & USB_OTG_GINTSTS_WKUINT) {
        CORE->GINTSTS = USB_OTG_GINTSTS_WKUINT;
        suspended = false;
        txKick();
    }
}

// Character device ---------------------------------------------------------

static int usbcdc_init(const struct chardev *dev)
{
    (void)dev;

    if (initialized)
        return 0;

    // 48 MHz kernel clock: PLL3, HSE 25 MHz / 5 = 5 MHz, * 96 = 480 MHz,
    // Q = 10. PLL3 is not used anywhere else on this port.
    RCC->CR &= ~RCC_CR_PLL3ON;
    waitBits(&RCC->CR, RCC_CR_PLL3RDY, 0);
    RCC->PLLCKSELR = (RCC->PLLCKSELR & ~RCC_PLLCKSELR_DIVM3)
                   | (5u << RCC_PLLCKSELR_DIVM3_Pos);
    RCC->PLL3DIVR = ((10u - 1) << RCC_PLL3DIVR_Q3_Pos)
                  | ((96u - 1) << RCC_PLL3DIVR_N3_Pos);
    RCC->PLLCFGR = (RCC->PLLCFGR
                    & ~(RCC_PLLCFGR_PLL3RGE | RCC_PLLCFGR_PLL3VCOSEL
                        | RCC_PLLCFGR_PLL3FRACEN))
                 | RCC_PLLCFGR_PLL3RGE_2 // 4..8 MHz input
                 | RCC_PLLCFGR_DIVQ3EN;  // wide VCO range
    RCC->CR |= RCC_CR_PLL3ON;
    if (!waitBits(&RCC->CR, RCC_CR_PLL3RDY, RCC_CR_PLL3RDY))
        return -ETIMEDOUT;

    RCC->D2CCIP2R = (RCC->D2CCIP2R & ~RCC_D2CCIP2R_USBSEL)
                  | RCC_D2CCIP2R_USBSEL_1; // pll3_q_ck

    // USB transceiver supply detector
    PWR->CR3 |= PWR_CR3_USB33DEN;
    if (!waitBits(&PWR->CR3, PWR_CR3_USB33RDY, PWR_CR3_USB33RDY))
        return -ETIMEDOUT;

    gpio_setMode(GPIOA, 11, ALTERNATE | ALTERNATE_FUNC(10));
    gpio_setMode(GPIOA, 12, ALTERNATE | ALTERNATE_FUNC(10));

    // Reset the peripheral: the boot loader may have used it
    RCC->AHB1ENR |= RCC_AHB1ENR_USB2OTGFSEN;
    RCC->AHB1RSTR |= RCC_AHB1RSTR_USB2OTGFSRST;
    __DSB();
    RCC->AHB1RSTR &= ~RCC_AHB1RSTR_USB2OTGFSRST;
    __DSB();

    // Core soft reset, embedded full-speed PHY
    CORE->GUSBCFG |= USB_OTG_GUSBCFG_PHYSEL;
    if (!waitBits(&CORE->GRSTCTL, USB_OTG_GRSTCTL_AHBIDL,
                  USB_OTG_GRSTCTL_AHBIDL))
        return -ETIMEDOUT;

    CORE->GRSTCTL |= USB_OTG_GRSTCTL_CSRST;
    if (!waitBits(&CORE->GRSTCTL, USB_OTG_GRSTCTL_CSRST, 0))
        return -ETIMEDOUT;

    CORE->GCCFG |= USB_OTG_GCCFG_PWRDWN; // transceiver on

    // Device mode; TRDT = 6 for an AHB clock above 32 MHz
    CORE->GUSBCFG = (CORE->GUSBCFG
                     & ~(USB_OTG_GUSBCFG_FHMOD | USB_OTG_GUSBCFG_TRDT))
                  | USB_OTG_GUSBCFG_FDMOD | (6u << USB_OTG_GUSBCFG_TRDT_Pos);
    delayMs(30); // mode change takes up to 25 ms

    // No VBUS sensing: force the B-session valid
    CORE->GCCFG &= ~USB_OTG_GCCFG_VBDEN;
    CORE->GOTGCTL |= USB_OTG_GOTGCTL_BVALOEN | USB_OTG_GOTGCTL_BVALOVAL;

    PCGCCTL = 0;
    DEV->DCTL |= USB_OTG_DCTL_SDIS; // stay detached while setting up
    DEV->DCFG |= USB_OTG_DCFG_DSPD; // full speed, embedded PHY

    CORE->GRXFSIZ = RX_FIFO_WORDS;
    CORE->DIEPTXF0_HNPTXFSIZ = (TX0_FIFO_WORDS << 16) | RX_FIFO_WORDS;
    CORE->DIEPTXF[0] = (TX1_FIFO_WORDS << 16)
                     | (RX_FIFO_WORDS + TX0_FIFO_WORDS);
    CORE->DIEPTXF[1] = (TX2_FIFO_WORDS << 16)
                     | (RX_FIFO_WORDS + TX0_FIFO_WORDS + TX1_FIFO_WORDS);

    flushTxFifos();
    CORE->GRSTCTL = USB_OTG_GRSTCTL_RXFFLSH;
    waitBits(&CORE->GRSTCTL, USB_OTG_GRSTCTL_RXFFLSH, 0);

    DEV->DIEPMSK = 0;
    DEV->DOEPMSK = 0;
    DEV->DAINTMSK = 0;
    for (int i = 0; i < 4; i++) {
        INEP(i)->DIEPCTL = 0;
        INEP(i)->DIEPTSIZ = 0;
        INEP(i)->DIEPINT = 0xFB7F;
        OUTEP(i)->DOEPCTL = 0;
        OUTEP(i)->DOEPTSIZ = 0;
        OUTEP(i)->DOEPINT = 0xFB7F;
    }

    CORE->GINTSTS = 0xFFFFFFFF;
    CORE->GINTMSK = GINT_MASK;
    CORE->GAHBCFG |= USB_OTG_GAHBCFG_GINT;

    NVIC_ClearPendingIRQ(OTG_FS_IRQn);
    NVIC_SetPriority(OTG_FS_IRQn, 14);
    NVIC_EnableIRQ(OTG_FS_IRQn);

    // Attach: the host sees D+ go high and starts enumeration
    delayMs(5);
    DEV->DCTL &= ~USB_OTG_DCTL_SDIS;

    initialized = true;
    return 0;
}

static int usbcdc_terminate(const struct chardev *dev)
{
    (void)dev;

    if (!initialized)
        return 0;

    DEV->DCTL |= USB_OTG_DCTL_SDIS;
    NVIC_DisableIRQ(OTG_FS_IRQn);
    RCC->AHB1ENR &= ~RCC_AHB1ENR_USB2OTGFSEN;
    configured = false;
    dtr = false;
    initialized = false;

    return 0;
}

static ssize_t usbcdc_read(const struct chardev *dev, void *data,
                           const size_t len)
{
    (void)dev;

    uint8_t *out = (uint8_t *)data;
    size_t n = 0;

    uint32_t key = irqLock();
    while ((n < len) && (rxTail != rxHead)) {
        out[n++] = rxRing[rxTail & (RX_RING_SIZE - 1)];
        rxTail = rxTail + 1;
    }

    if (configured && !rxArmed)
        rxArm();
    irqUnlock(key);

    return n;
}

static ssize_t usbcdc_write(const struct chardev *dev, const void *data,
                            const size_t len)
{
    (void)dev;

    if (!usbcdc_connected())
        return -ENOTCONN;

    const uint8_t *in = (const uint8_t *)data;

    uint32_t key = irqLock();
    if ((TX_RING_SIZE - (txHead - txTail)) < len) {
        irqUnlock(key);
        return -EAGAIN;
    }

    for (size_t i = 0; i < len; i++)
        txRing[(txHead + i) & (TX_RING_SIZE - 1)] = in[i];

    txHead = txHead + len;
    txKick();
    irqUnlock(key);

    return len;
}

static int usbcdc_ioctl(const struct chardev *dev, const int cmd,
                        const void *arg)
{
    (void)dev;
    (void)cmd;
    (void)arg;

    return -ENOTSUP;
}

bool usbcdc_connected()
{
    return initialized && configured && dtr && !suspended;
}

// Positional initializers: designated ones are C++20
static const struct chardev_api usbcdc_api = { usbcdc_init, usbcdc_terminate,
                                               usbcdc_read, usbcdc_write,
                                               usbcdc_ioctl };

const struct chardev usb_cdc_chardev = { NULL, // config
                                         NULL, // data
                                         &usbcdc_api };
