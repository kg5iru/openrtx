/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/*
 * Composite USB device on the STM32H743 USB2 OTG_FS core (Synopsys DWC2),
 * register level, slave mode (no DMA), no external USB stack:
 *
 * - two CDC-ACM serial ports: the system log (rtxlink) and CAT control;
 * - a USB Audio Class 1.0 sound card, 48 kHz mono 16 bit both ways
 *   (usb_audio.h).
 *
 * Interfaces and endpoints:
 *   0, 1  log port    EP2 IN interrupt (notifications, unused), EP1 bulk
 *   2, 3  CAT port    EP4 IN interrupt (notifications, unused), EP3 bulk
 *   4     audio control (two feature units: volume and mute)
 *   5     audio to the radio    (alt 1: EP5 OUT isochronous, adaptive)
 *   6     audio from the radio  (alt 1: EP5 IN isochronous, asynchronous)
 *
 * All endpoint and FIFO access happens either in the interrupt handler or
 * with interrupts disabled, so the core never sees interleaved FIFO writes.
 *
 * C++ only because this port's vector table uses C++ handler names.
 */

#include "usb_cdc.h"
#include "usb_audio.h"
#include "peripherals/gpio.h"
#include "interfaces/delays.h"
#include "stm32h7xx.h"
#include <string.h>
#include <errno.h>
#include <math.h>

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
#define NUM_EPS 6 // EP0 .. EP5

#define AUDIO_EP 5
#define AUDIO_MPS 100 // bytes: up to 50 samples per frame
#define AUDIO_RATE 48000

// FIFO RAM layout, in 32-bit words (4 KiB = 1024 words available)
#define RX_FIFO_WORDS 256
#define TX0_FIFO_WORDS 64
#define TX1_FIFO_WORDS 128 // log port data
#define TX2_FIFO_WORDS 16
#define TX3_FIFO_WORDS 32  // CAT port data
#define TX4_FIFO_WORDS 16
#define TX5_FIFO_WORDS 64  // audio: two packets

#define LOG_TX_RING 2048   // powers of two
#define LOG_RX_RING 256
#define CAT_TX_RING 512
#define CAT_RX_RING 256

#define USB_VID 0x0483
#define USB_PID 0x5740

// GRXSTSP packet status values
#define PKT_OUT_DATA 2
#define PKT_SETUP_DATA 6

// Interrupt flags handled by the driver (SOF is added while audio streams)
#define GINT_MASK                                        \
    (USB_OTG_GINTMSK_USBRST | USB_OTG_GINTMSK_ENUMDNEM   \
     | USB_OTG_GINTMSK_RXFLVLM | USB_OTG_GINTMSK_OEPINT  \
     | USB_OTG_GINTMSK_IEPINT | USB_OTG_GINTMSK_USBSUSPM \
     | USB_OTG_GINTMSK_WUIM | USB_OTG_GINTMSK_IISOIXFRM  \
     | USB_OTG_GINTMSK_PXFRM_IISOOXFRM)

enum Ep0State {
    EP0_IDLE,     // waiting for SETUP
    EP0_DATA_IN,  // sending the data stage
    EP0_DATA_OUT, // receiving the data stage
    EP0_STATUS    // status stage in progress
};

// Descriptors --------------------------------------------------------------

static const uint8_t devDesc[] = {
    18,
    1,    // bLength, DEVICE
    0x00,
    0x02, // USB 2.0
    0xEF,
    0x02,
    0x01, // miscellaneous, common class, interface association
    EP0_SIZE,
    USB_VID & 0xFF,
    USB_VID >> 8,
    USB_PID & 0xFF,
    USB_PID >> 8,
    0x00,
    0x02, // bcdDevice 2.00: composite (1.00 was the log port alone)
    1,
    2,
    3, // manufacturer, product, serial strings
    1  // one configuration
};

// String indexes
#define STR_LOG 4
#define STR_CAT 5
#define STR_AUDIO 6

/*
 * One CDC-ACM function: interface association, communication interface with
 * its notification endpoint, data interface with the bulk endpoints.
 */
#define CDC_FUNCTION_LEN 66
#define CDC_FUNCTION(commIf, notifEp, dataEp, str)                            \
    8, 11, (commIf), 2, 0x02, 0x02, 0x01, (str),       /* IAD */              \
        9, 4, (commIf), 0, 1, 0x02, 0x02, 0x01, (str), /* comm interface */   \
        5, 0x24, 0x00, 0x10, 0x01,                     /* header, CDC 1.10 */ \
        5, 0x24, 0x01, 0x00, (commIf) + 1,             /* call management */  \
        4, 0x24, 0x02, 0x02,                           /* ACM */              \
        5, 0x24, 0x06, (commIf), (commIf) + 1,         /* union */            \
        7, 5, 0x80 | (notifEp), 0x03, NOTIF_SIZE, 0, 16, 9, 4, (commIf) + 1,  \
        0, 2, 0x0A, 0x00, 0x00, 0,                     /* data interface */   \
        7, 5, (dataEp), 0x02, DATA_SIZE, 0, 0, 7, 5, 0x80 | (dataEp), 0x02,   \
        DATA_SIZE, 0, 0

// Audio function: entity IDs of the audio control topology
#define AC_IT_USB 1 // input terminal: USB stream from the computer
#define AC_FU_TX 2  // feature unit: transmit volume and mute
#define AC_OT_TX 3  // output terminal: to the transmitter ("speaker")
#define AC_IT_RX 4  // input terminal: from the receiver ("microphone")
#define AC_FU_RX 5  // feature unit: receive volume and mute
#define AC_OT_USB 6 // output terminal: USB stream to the computer
#define AC_CLASS_LEN 70
#define AS_LEN 52
#define AUDIO_FUNCTION_LEN (8 + 9 + AC_CLASS_LEN + (2 * AS_LEN))

#define AS_INTERFACE(intf, terminal, epAddr, epAttr)                          \
    9, 4, (intf), 0, 0, 0x01, 0x02, 0x00, 0,       /* alt 0: no endpoint */   \
        9, 4, (intf), 1, 1, 0x01, 0x02, 0x00, 0,   /* alt 1: streaming */     \
        7, 0x24, 0x01, (terminal), 1, 0x01, 0x00,  /* general: PCM */         \
        11, 0x24, 0x02, 0x01, 1, 2, 16, 1,         /* type I, mono, 16 bit */ \
        AUDIO_RATE & 0xFF, (AUDIO_RATE >> 8) & 0xFF, AUDIO_RATE >> 16, 9, 5,  \
        (epAddr), (epAttr), AUDIO_MPS, 0, 1, 0, 0, /* endpoint */             \
        7, 0x25, 0x01, 0x00, 0, 0, 0               /* class endpoint */

#define CFG_LEN (9 + (2 * CDC_FUNCTION_LEN) + AUDIO_FUNCTION_LEN)

// clang-format off
static const uint8_t cfgDesc[] = {
    9, 2, CFG_LEN & 0xFF, CFG_LEN >> 8, // CONFIGURATION
    7, 1, 0,                            // 7 interfaces, value 1, no string
    0xC0, 50,                           // self powered, 100 mA

    CDC_FUNCTION(0, 2, 1, STR_LOG),
    CDC_FUNCTION(2, 4, 3, STR_CAT),

    // Audio: interface association, then audio control interface 4
    8, 11, 4, 3, 0x01, 0x00, 0x00, STR_AUDIO,
    9, 4, 4, 0, 0, 0x01, 0x01, 0x00, STR_AUDIO,
    10, 0x24, 0x01, 0x00, 0x01, AC_CLASS_LEN, 0, 2, 5, 6, // header, ADC 1.0
    // Computer -> radio: USB stream, feature unit, "speaker"
    12, 0x24, 0x02, AC_IT_USB, 0x01, 0x01, 0, 1, 0x00, 0x00, 0, 0,
    9, 0x24, 0x06, AC_FU_TX, AC_IT_USB, 1, 0x03, 0x00, 0, // mute, volume
    9, 0x24, 0x03, AC_OT_TX, 0x01, 0x03, 0, AC_FU_TX, 0,
    // Radio -> computer: "microphone", feature unit, USB stream
    12, 0x24, 0x02, AC_IT_RX, 0x01, 0x02, 0, 1, 0x00, 0x00, 0, 0,
    9, 0x24, 0x06, AC_FU_RX, AC_IT_RX, 1, 0x03, 0x00, 0,
    9, 0x24, 0x03, AC_OT_USB, 0x01, 0x01, 0, AC_FU_RX, 0,

    AS_INTERFACE(5, AC_IT_USB, AUDIO_EP, 0x09),        // OUT, adaptive
    AS_INTERFACE(6, AC_OT_USB, 0x80 | AUDIO_EP, 0x05), // IN, asynchronous
};
// clang-format on

static_assert(sizeof(cfgDesc) == CFG_LEN, "configuration descriptor length");

static const char *const strings[] = { "OpenRTX", "OpenRTX radio",
                                       "OpenRTX log", "OpenRTX CAT",
                                       "OpenRTX audio" };

// Serial ports ---------------------------------------------------------------

struct Port {
    uint8_t commIf;  // communication interface number
    uint8_t dataEp;  // bulk endpoint number
    uint8_t notifEp; // interrupt endpoint number
    bool needDtr;    // output only while the computer has the port open
    uint8_t *txRing;
    uint32_t txSize;
    uint8_t *rxRing;
    uint32_t rxSize;
    volatile uint32_t txHead; // written by thread code
    volatile uint32_t txTail; // written by the interrupt handler
    volatile bool txBusy;
    volatile bool txZlp;      // last packet was full-size
    volatile uint32_t rxHead; // written by the interrupt handler
    volatile uint32_t rxTail; // written by thread code
    volatile bool rxArmed;
    volatile bool dtr;
    uint8_t lineCoding[7];
};

static uint8_t logTx[LOG_TX_RING], logRx[LOG_RX_RING];
static uint8_t catTx[CAT_TX_RING], catRx[CAT_RX_RING];

// clang-format off
static Port ports[2] = {
    { 0, 1, 2, true, logTx, LOG_TX_RING, logRx, LOG_RX_RING,
      0, 0, false, false, 0, 0, false, false,
      { 0x00, 0xC2, 0x01, 0x00, 0, 0, 8 } },  // 115200 8N1
    { 2, 3, 4, false, catTx, CAT_TX_RING, catRx, CAT_RX_RING,
      0, 0, false, false, 0, 0, false, false,
      { 0x80, 0x25, 0x00, 0x00, 0, 0, 8 } },  // 9600 8N1
};
// clang-format on

static uint8_t ep0Buf[128];
static uint8_t setup[8];
static enum Ep0State ep0State;
static const uint8_t *ep0TxPtr; // rest of the IN data stage
static uint32_t ep0TxLeft;
static bool ep0TxZlp;           // end the data stage with a ZLP
static uint32_t ep0RxLen;       // bytes of the OUT data stage received

static volatile bool configured;
static volatile bool suspended;
static volatile bool initialized;

// Audio ----------------------------------------------------------------------

#define TX_RING 4096        // samples, computer -> radio
#define RX_RING 4096        // samples, radio -> computer
#define RX_TARGET 1920      // receive buffer level kept, 40 ms
#define TX_TARGET 1920      // transmit buffer level kept, 40 ms

#define VOL_MIN (-40 * 256) // feature unit volume, 1/256 dB
#define VOL_MAX (12 * 256)
#define VOL_RES 256

struct Feature {
    volatile bool mute;
    volatile int16_t volume;
};

static Feature fuTx = { false, 0 };
static Feature fuRx = { false, 0 };

static volatile uint8_t altTx; // interface 5
static volatile uint8_t altRx; // interface 6

static int16_t txRing[TX_RING];
static volatile uint32_t txRingHead; // interrupt handler
static volatile uint32_t txRingTail; // thread code
static int16_t rxRing[RX_RING];
static volatile uint32_t rxRingHead; // thread code
static volatile uint32_t rxRingTail; // interrupt handler
static bool rxPriming = true;        // sending silence until the buffer fills
static int32_t rxAvg;                // smoothed buffer level, x 64
static volatile bool hostPtt;

static uint32_t txOverruns; // diagnostics
static uint32_t rxUnderruns;

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

static void flushTxFifo(uint32_t num)
{
    CORE->GRSTCTL = USB_OTG_GRSTCTL_TXFFLSH
                  | (num << USB_OTG_GRSTCTL_TXFNUM_Pos);
    waitBits(&CORE->GRSTCTL, USB_OTG_GRSTCTL_TXFFLSH, 0);
}

static inline uint32_t frameOdd()
{
    return (DEV->DSTS >> USB_OTG_DSTS_FNSOF_Pos) & 1;
}

// Endpoint 0 ---------------------------------------------------------------

static void ep0ArmOut(uint32_t len)
{
    OUTEP(0)->DOEPTSIZ = (3u << USB_OTG_DOEPTSIZ_STUPCNT_Pos)
                       | (1u << USB_OTG_DOEPTSIZ_PKTCNT_Pos) | len;
    OUTEP(0)->DOEPCTL |= USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA;
}

// One packet of the IN data stage (or its ZLP)
static void ep0SendPacket()
{
    uint32_t len = (ep0TxLeft > EP0_SIZE) ? EP0_SIZE : ep0TxLeft;

    INEP(0)->DIEPTSIZ = (1u << USB_OTG_DIEPTSIZ_PKTCNT_Pos) | len;
    INEP(0)->DIEPCTL |= USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA;
    fifoWrite(0, ep0TxPtr, len);

    ep0TxPtr += len;
    ep0TxLeft -= len;
}

static void ep0Send(const uint8_t *data, uint32_t len)
{
    uint16_t wLength = setup[6] | (setup[7] << 8);
    if (len > wLength)
        len = wLength;

    // A data stage shorter than asked, ending on a full packet, needs a ZLP
    ep0TxZlp = (len > 0) && (len < wLength) && ((len % EP0_SIZE) == 0);
    ep0TxPtr = data;
    ep0TxLeft = len;
    ep0SendPacket();

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

static void ep0Receive(uint32_t len)
{
    ep0RxLen = 0;
    ep0State = EP0_DATA_OUT;
    ep0ArmOut(len);
}

static uint32_t stringDesc(uint8_t index)
{
    uint32_t len = 0;
    const uint32_t numStrings = sizeof(strings) / sizeof(strings[0]);

    if (index == 0) {
        ep0Buf[2] = 0x09; // English (US)
        ep0Buf[3] = 0x04;
        len = 4;
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
    } else if ((index <= 2)
               || ((index >= STR_LOG) && (index <= numStrings + 1))) {
        const char *s = strings[(index <= 2) ? (index - 1) : (index - 2)];
        len = 2;
        while (*s != '\0') {
            ep0Buf[len++] = (uint8_t)*s++;
            ep0Buf[len++] = 0;
        }
    } else {
        return 0;
    }

    ep0Buf[0] = len;
    ep0Buf[1] = 3; // STRING
    return len;
}

// Serial port endpoints ------------------------------------------------------

static void rxArm(Port *p)
{
    if ((p->rxSize - (p->rxHead - p->rxTail)) < DATA_SIZE) {
        p->rxArmed = false; // read() re-arms once there is room
        return;
    }

    OUTEP(p->dataEp)->DOEPTSIZ = (1u << USB_OTG_DOEPTSIZ_PKTCNT_Pos)
                               | DATA_SIZE;
    OUTEP(p->dataEp)->DOEPCTL |= USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA;
    p->rxArmed = true;
}

static bool portOpen(const Port *p)
{
    return configured && !suspended && (p->dtr || !p->needDtr);
}

/** Start the next bulk IN packet if possible. Interrupts must be disabled. */
static void txKick(Port *p)
{
    if (!portOpen(p) || p->txBusy)
        return;

    uint32_t len = p->txHead - p->txTail;
    if (len == 0) {
        if (!p->txZlp)
            return;

        p->txZlp = false; // end the transfer after a full-size packet
    }

    if (len > DATA_SIZE)
        len = DATA_SIZE;

    uint8_t ep = p->dataEp;
    INEP(ep)->DIEPTSIZ = (1u << USB_OTG_DIEPTSIZ_PKTCNT_Pos) | len;
    INEP(ep)->DIEPCTL |= USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA;

    for (uint32_t i = 0; i < len; i += 4) {
        uint32_t word = 0;
        for (uint32_t j = 0; (j < 4) && ((i + j) < len); j++) {
            uint8_t b = p->txRing[(p->txTail + i + j) & (p->txSize - 1)];
            word |= ((uint32_t)b) << (8 * j);
        }

        FIFO(ep) = word;
    }

    p->txTail = p->txTail + len;
    p->txBusy = true;
    p->txZlp = (len == DATA_SIZE);
}

static void configurePort(Port *p)
{
    INEP(p->dataEp)->DIEPCTL =
        USB_OTG_DIEPCTL_USBAEP | (2u << USB_OTG_DIEPCTL_EPTYP_Pos) // bulk
        | ((uint32_t)p->dataEp << USB_OTG_DIEPCTL_TXFNUM_Pos)
        | USB_OTG_DIEPCTL_SD0PID_SEVNFRM | DATA_SIZE;

    INEP(p->notifEp)->DIEPCTL =
        USB_OTG_DIEPCTL_USBAEP | (3u << USB_OTG_DIEPCTL_EPTYP_Pos) // interrupt
        | ((uint32_t)p->notifEp << USB_OTG_DIEPCTL_TXFNUM_Pos)
        | USB_OTG_DIEPCTL_SD0PID_SEVNFRM | NOTIF_SIZE;

    OUTEP(p->dataEp)->DOEPCTL = USB_OTG_DOEPCTL_USBAEP
                              | (2u << USB_OTG_DOEPCTL_EPTYP_Pos) // bulk
                              | USB_OTG_DOEPCTL_SD0PID_SEVNFRM | DATA_SIZE;

    DEV->DAINTMSK |= (1u << p->dataEp) | (1u << p->notifEp)
                   | (1u << (16 + p->dataEp));

    p->txBusy = false;
    p->txZlp = false;
    rxArm(p);
}

static void deconfigurePort(Port *p)
{
    uint8_t ep = p->dataEp;

    // A packet may still be armed on the bulk IN endpoint
    if (INEP(ep)->DIEPCTL & USB_OTG_DIEPCTL_EPENA) {
        INEP(ep)->DIEPCTL |= USB_OTG_DIEPCTL_EPDIS | USB_OTG_DIEPCTL_SNAK;
        waitBits(&INEP(ep)->DIEPINT, USB_OTG_DIEPINT_EPDISD,
                 USB_OTG_DIEPINT_EPDISD, 10000);
        INEP(ep)->DIEPINT = USB_OTG_DIEPINT_EPDISD;
    }

    INEP(ep)->DIEPCTL = 0;
    INEP(p->notifEp)->DIEPCTL = 0;
    OUTEP(ep)->DOEPCTL = 0;
    DEV->DAINTMSK &= ~((1u << ep) | (1u << p->notifEp) | (1u << (16 + ep)));
    p->dtr = false;
    p->txBusy = false;
}

static Port *portOfInterface(uint8_t intf)
{
    for (Port &p : ports) {
        if (p.commIf == intf)
            return &p;
    }

    return NULL;
}

// Audio endpoints ------------------------------------------------------------

static void updateSofMask()
{
    if ((altTx != 0) || (altRx != 0))
        CORE->GINTMSK |= USB_OTG_GINTMSK_SOFM;
    else
        CORE->GINTMSK &= ~USB_OTG_GINTMSK_SOFM;
}

/*
 * Next receive packet (radio -> computer), for the next frame: 48 samples,
 * or 47/49 to keep the buffer level near RX_TARGET whatever the drift
 * between the radio's clock and the computer's.  Silence while priming:
 * until the radio has written RX_TARGET samples, and again after the buffer
 * ran dry.
 */
static void audioRxSend()
{
    uint32_t fill = rxRingHead - rxRingTail;
    uint32_t n = AUDIO_RATE / 1000;

    if (rxPriming && (fill >= RX_TARGET)) {
        rxPriming = false;
        rxAvg = (int32_t)fill * 64;
    }

    if (!rxPriming) {
        rxAvg += (int32_t)fill - (rxAvg / 64);
        int32_t avg = rxAvg / 64;
        if (avg > (RX_TARGET + 240))
            n += 1;
        else if (avg < (RX_TARGET - 240))
            n -= 1;

        if (fill < n) {
            rxPriming = true; // ran dry
            rxUnderruns++;
        }
    }

    INEP(AUDIO_EP)->DIEPTSIZ = (1u << USB_OTG_DIEPTSIZ_MULCNT_Pos)
                             | (1u << USB_OTG_DIEPTSIZ_PKTCNT_Pos) | (2 * n);
    INEP(AUDIO_EP)->DIEPCTL |= (frameOdd() ? USB_OTG_DIEPCTL_SD0PID_SEVNFRM :
                                             USB_OTG_DIEPCTL_SODDFRM)
                             | USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA;

    for (uint32_t i = 0; i < n; i += 2) {
        uint32_t word = 0;
        if (!rxPriming) {
            uint16_t s0 = (uint16_t)rxRing[rxRingTail & (RX_RING - 1)];
            uint16_t s1 = 0;
            if ((i + 1) < n)
                s1 = (uint16_t)rxRing[(rxRingTail + 1) & (RX_RING - 1)];
            word = s0 | ((uint32_t)s1 << 16);
            rxRingTail = rxRingTail + (((i + 1) < n) ? 2 : 1);
        }

        FIFO(AUDIO_EP) = word;
    }
}

// Arm the transmit endpoint (computer -> radio) for this frame or the next
static void audioTxArm(bool thisFrame)
{
    uint32_t odd = frameOdd();
    if (!thisFrame)
        odd ^= 1;

    OUTEP(AUDIO_EP)->DOEPTSIZ = (1u << USB_OTG_DOEPTSIZ_PKTCNT_Pos) | AUDIO_MPS;
    OUTEP(AUDIO_EP)->DOEPCTL |= (odd ? USB_OTG_DOEPCTL_SODDFRM :
                                       USB_OTG_DOEPCTL_SD0PID_SEVNFRM)
                              | USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA;
}

static void audioRxStart()
{
    INEP(AUDIO_EP)->DIEPCTL = USB_OTG_DIEPCTL_USBAEP
                            | (1u << USB_OTG_DIEPCTL_EPTYP_Pos) // isochronous
                            | ((uint32_t)AUDIO_EP << USB_OTG_DIEPCTL_TXFNUM_Pos)
                            | AUDIO_MPS;
    DEV->DAINTMSK |= (1u << AUDIO_EP);
    flushTxFifo(AUDIO_EP);
    rxPriming = true;
    audioRxSend();
}

static void audioRxStop()
{
    if (INEP(AUDIO_EP)->DIEPCTL & USB_OTG_DIEPCTL_EPENA) {
        INEP(AUDIO_EP)->DIEPCTL |= USB_OTG_DIEPCTL_EPDIS | USB_OTG_DIEPCTL_SNAK;
        waitBits(&INEP(AUDIO_EP)->DIEPINT, USB_OTG_DIEPINT_EPDISD,
                 USB_OTG_DIEPINT_EPDISD, 10000);
        INEP(AUDIO_EP)->DIEPINT = USB_OTG_DIEPINT_EPDISD;
    }

    INEP(AUDIO_EP)->DIEPCTL = 0;
    DEV->DAINTMSK &= ~(1u << AUDIO_EP);
    flushTxFifo(AUDIO_EP);
}

static void audioTxStart()
{
    OUTEP(AUDIO_EP)->DOEPCTL = USB_OTG_DOEPCTL_USBAEP
                             | (1u << USB_OTG_DOEPCTL_EPTYP_Pos) // isochronous
                             | AUDIO_MPS;
    DEV->DAINTMSK |= (1u << (16 + AUDIO_EP));
    audioTxArm(false);
}

static void audioTxStop()
{
    // An enabled OUT endpoint is disabled under the global OUT NAK
    if (OUTEP(AUDIO_EP)->DOEPCTL & USB_OTG_DOEPCTL_EPENA) {
        DEV->DCTL |= USB_OTG_DCTL_SGONAK;
        waitBits(&CORE->GINTSTS, USB_OTG_GINTSTS_BOUTNAKEFF,
                 USB_OTG_GINTSTS_BOUTNAKEFF, 10000);
        OUTEP(AUDIO_EP)->DOEPCTL |= USB_OTG_DOEPCTL_EPDIS
                                  | USB_OTG_DOEPCTL_SNAK;
        waitBits(&OUTEP(AUDIO_EP)->DOEPINT, USB_OTG_DOEPINT_EPDISD,
                 USB_OTG_DOEPINT_EPDISD, 10000);
        OUTEP(AUDIO_EP)->DOEPINT = USB_OTG_DOEPINT_EPDISD;
        DEV->DCTL |= USB_OTG_DCTL_CGONAK;
    }

    OUTEP(AUDIO_EP)->DOEPCTL = 0;
    DEV->DAINTMSK &= ~(1u << (16 + AUDIO_EP));
}

static void setAudioAlt(uint8_t intf, uint8_t alt)
{
    if (intf == 5) {
        if ((alt != 0) && (altTx == 0))
            audioTxStart();
        else if ((alt == 0) && (altTx != 0))
            audioTxStop();
        altTx = alt;
    } else {
        if ((alt != 0) && (altRx == 0))
            audioRxStart();
        else if ((alt == 0) && (altRx != 0))
            audioRxStop();
        altRx = alt;
    }

    updateSofMask();
}

static void audioStop()
{
    if (altTx != 0)
        audioTxStop();
    if (altRx != 0)
        audioRxStop();
    altTx = 0;
    altRx = 0;
    hostPtt = false;
    updateSofMask();
}

// Received audio packet from the computer, still in the RX FIFO
static void audioTxData(uint32_t count)
{
    uint32_t samples = count / 2;
    uint32_t room = TX_RING - (txRingHead - txRingTail);

    if ((altTx == 0) || (samples > room) || (count & 1)) {
        if (altTx != 0)
            txOverruns++;
        fifoDiscard(count);
        return;
    }

    uint32_t head = txRingHead;
    for (uint32_t i = 0; i < samples; i += 2) {
        uint32_t word = FIFO(0);
        txRing[head & (TX_RING - 1)] = (int16_t)(word & 0xFFFF);
        head++;
        if ((i + 1) < samples) {
            txRing[head & (TX_RING - 1)] = (int16_t)(word >> 16);
            head++;
        }
    }

    txRingHead = head;
}

// Start of frame: keep the isochronous endpoints going
static void startOfFrame()
{
    if (altTx != 0) {
        if (OUTEP(AUDIO_EP)->DOEPCTL & USB_OTG_DOEPCTL_EPENA) {
            // A frame without data (or one dropped) leaves the endpoint
            // waiting for the wrong parity: follow the current frame
            OUTEP(AUDIO_EP)->DOEPCTL |= frameOdd() ?
                                            USB_OTG_DOEPCTL_SODDFRM :
                                            USB_OTG_DOEPCTL_SD0PID_SEVNFRM;
        } else {
            audioTxArm(true);
        }
    }

    if ((altRx != 0) && !(INEP(AUDIO_EP)->DIEPCTL & USB_OTG_DIEPCTL_EPENA))
        audioRxSend();
}

// The receive packet armed for the frame just ended was not taken
static void incompleteRx()
{
    if (altRx == 0)
        return;

    uint32_t ctl = INEP(AUDIO_EP)->DIEPCTL;
    uint32_t parity = (ctl & USB_OTG_DIEPCTL_EONUM_DPID) ? 1 : 0;
    if ((ctl & USB_OTG_DIEPCTL_EPENA) && (parity == frameOdd())) {
        // Keep the data in the FIFO, retry in the next frame
        INEP(AUDIO_EP)->DIEPCTL = ctl
                                | (parity ? USB_OTG_DIEPCTL_SD0PID_SEVNFRM :
                                            USB_OTG_DIEPCTL_SODDFRM);
    }
}

// Audio class requests -------------------------------------------------------

static Feature *featureOf(uint8_t entity)
{
    if (entity == AC_FU_TX)
        return &fuTx;
    if (entity == AC_FU_RX)
        return &fuRx;
    return NULL;
}

static void putLe16(uint8_t *buf, int16_t v)
{
    buf[0] = (uint8_t)(v & 0xFF);
    buf[1] = (uint8_t)(((uint16_t)v) >> 8);
}

static bool audioRequest(uint8_t bmRequestType, uint8_t bRequest,
                         uint16_t wValue, uint16_t wIndex, uint16_t wLength)
{
    uint8_t cs = wValue >> 8;

    // Endpoint: sampling frequency (only one, 48 kHz)
    if ((bmRequestType & 0x1F) == 0x02) {
        if ((wIndex & 0x0F) != AUDIO_EP || cs != 0x01)
            return false;
        if (bRequest == 0x01) { // SET_CUR
            ep0Receive(wLength);
            return true;
        }
        if (bRequest == 0x81) { // GET_CUR
            ep0Buf[0] = AUDIO_RATE & 0xFF;
            ep0Buf[1] = (AUDIO_RATE >> 8) & 0xFF;
            ep0Buf[2] = AUDIO_RATE >> 16;
            ep0Send(ep0Buf, 3);
            return true;
        }
        return false;
    }

    // Feature units: master channel mute (1 byte) and volume (2 bytes)
    Feature *fu = featureOf(wIndex >> 8);
    if ((fu == NULL) || ((wValue & 0xFF) != 0))
        return false;

    if (bRequest == 0x01) { // SET_CUR
        if ((cs != 0x01) && (cs != 0x02))
            return false;
        ep0Receive(wLength);
        return true;
    }

    if (cs == 0x01) {
        if (bRequest != 0x81)
            return false;
        ep0Buf[0] = fu->mute ? 1 : 0;
        ep0Send(ep0Buf, 1);
        return true;
    }

    if (cs != 0x02)
        return false;

    switch (bRequest) {
        case 0x81: // GET_CUR
            putLe16(ep0Buf, fu->volume);
            break;
        case 0x82: // GET_MIN
            putLe16(ep0Buf, VOL_MIN);
            break;
        case 0x83: // GET_MAX
            putLe16(ep0Buf, VOL_MAX);
            break;
        case 0x84: // GET_RES
            putLe16(ep0Buf, VOL_RES);
            break;
        default:
            return false;
    }

    ep0Send(ep0Buf, 2);
    return true;
}

// Data stage of a class request received
static void outDataStage()
{
    uint8_t bmRequestType = setup[0];
    uint8_t bRequest = setup[1];
    uint8_t cs = setup[3];
    uint16_t wIndex = setup[4] | (setup[5] << 8);

    if (((bmRequestType & 0x1F) == 0x01) && (bRequest == 0x20)) {
        // SET_LINE_CODING
        Port *p = portOfInterface(wIndex & 0xFF);
        if ((p != NULL) && (ep0RxLen == sizeof(p->lineCoding)))
            memcpy(p->lineCoding, ep0Buf, sizeof(p->lineCoding));
    } else if (((bmRequestType & 0x1F) == 0x01) && (bRequest == 0x01)) {
        // Feature unit SET_CUR
        Feature *fu = featureOf(wIndex >> 8);
        if ((fu != NULL) && (cs == 0x01) && (ep0RxLen >= 1)) {
            fu->mute = (ep0Buf[0] != 0);
        } else if ((fu != NULL) && (cs == 0x02) && (ep0RxLen >= 2)) {
            int16_t v = (int16_t)(ep0Buf[0] | (ep0Buf[1] << 8));
            if (v < VOL_MIN)
                v = VOL_MIN;
            if (v > VOL_MAX)
                v = VOL_MAX;
            fu->volume = v;
        }
    }
    // Endpoint SET_CUR (sampling frequency): only 48 kHz, nothing to do

    ep0Ack();
}

// Control requests ---------------------------------------------------------

static void classRequest(uint8_t bmRequestType, uint8_t bRequest,
                         uint16_t wValue, uint16_t wIndex, uint16_t wLength)
{
    uint8_t intf = wIndex & 0xFF;
    Port *p = NULL;
    if ((bmRequestType & 0x1F) == 0x01)
        p = portOfInterface(intf);

    if (p != NULL) {
        switch (bRequest) {
            case 0x20: // SET_LINE_CODING, 7 bytes follow
                if (wLength != sizeof(p->lineCoding))
                    break;
                ep0Receive(sizeof(p->lineCoding));
                return;

            case 0x21: // GET_LINE_CODING
                memcpy(ep0Buf, p->lineCoding, sizeof(p->lineCoding));
                ep0Send(ep0Buf, sizeof(p->lineCoding));
                return;

            case 0x22: // SET_CONTROL_LINE_STATE
                p->dtr = (wValue & 0x01) != 0;
                if (!p->dtr)
                    p->txZlp = false;
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

    bool audio = (((bmRequestType & 0x1F) == 0x01) && (intf == 4))
              || ((bmRequestType & 0x1F) == 0x02);
    if (audio && audioRequest(bmRequestType, bRequest, wValue, wIndex, wLength))
        return;

    ep0Stall();
}

static void configure()
{
    for (Port &p : ports)
        configurePort(&p);
    configured = true;
}

static void deconfigure()
{
    audioStop();
    for (Port &p : ports)
        deconfigurePort(&p);
    configured = false;
}

static void handleSetup()
{
    uint8_t bmRequestType = setup[0];
    uint8_t bRequest = setup[1];
    uint16_t wValue = setup[2] | (setup[3] << 8);
    uint16_t wIndex = setup[4] | (setup[5] << 8);
    uint16_t wLength = setup[6] | (setup[7] << 8);

    if ((bmRequestType & 0x60) == 0x20) { // class request
        classRequest(bmRequestType, bRequest, wValue, wIndex, wLength);
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
                uint8_t ep = wIndex & 0x0F;
                if ((ep >= 1) && (ep <= 4)) {
                    if (wIndex & 0x80) {
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
                ep0Send(devDesc, sizeof(devDesc));
                return;
            }

            if (type == 2) {
                ep0Send(cfgDesc, sizeof(cfgDesc));
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
                if (!configured)
                    configure();
            } else {
                deconfigure();
            }

            ep0Ack();
            return;

        case 0x0A: // GET_INTERFACE
        {
            uint8_t intf = wIndex & 0xFF;
            if (intf > 6)
                break;
            ep0Buf[0] = (intf == 5) ? altTx : ((intf == 6) ? altRx : 0);
            ep0Send(ep0Buf, 1);
            return;
        }

        case 0x0B: // SET_INTERFACE
        {
            uint8_t intf = wIndex & 0xFF;
            if ((intf == 5) || (intf == 6)) {
                if (wValue > 1)
                    break;
                setAudioAlt(intf, wValue);
            } else if ((intf > 6) || (wValue != 0)) {
                break;
            }

            ep0Ack();
            return;
        }

        default:
            break;
    }

    ep0Stall();
}

// Interrupt handling -------------------------------------------------------

static void busReset()
{
    flushTxFifo(0x10); // all

    for (int i = 0; i < NUM_EPS; i++) {
        INEP(i)->DIEPINT = 0xFB7F;
        INEP(i)->DIEPCTL &= ~USB_OTG_DIEPCTL_STALL;
        OUTEP(i)->DOEPINT = 0xFB7F;
        OUTEP(i)->DOEPCTL &= ~USB_OTG_DOEPCTL_STALL;
        OUTEP(i)->DOEPCTL |= USB_OTG_DOEPCTL_SNAK;
    }

    deconfigure();
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
        if ((ep0RxLen + count) <= sizeof(ep0Buf)) {
            fifoRead(&ep0Buf[ep0RxLen], count);
            ep0RxLen += count;
        } else {
            fifoDiscard(count);
        }

        return;
    }

    if (ep == AUDIO_EP) {
        audioTxData(count);
        return;
    }

    for (Port &p : ports) {
        if (ep != p.dataEp)
            continue;

        // Room for a full packet was checked when the endpoint was armed
        for (uint32_t i = 0; i < count; i += 4) {
            uint32_t word = FIFO(0);
            for (uint32_t j = 0; (j < 4) && ((i + j) < count); j++) {
                p.rxRing[p.rxHead & (p.rxSize - 1)] = (uint8_t)(word
                                                                >> (8 * j));
                p.rxHead = p.rxHead + 1;
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
                outDataStage();      // data stored, send status
            else if (ep0State == EP0_DATA_IN)
                ep0State = EP0_IDLE; // host's status ZLP received
        }

        if (flags & USB_OTG_DOEPINT_STUP) {
            ep0State = EP0_IDLE;
            handleSetup();
        }
    }

    for (Port &p : ports) {
        if (pending & (1u << p.dataEp)) {
            uint32_t flags = OUTEP(p.dataEp)->DOEPINT;
            OUTEP(p.dataEp)->DOEPINT = flags;

            if (flags & USB_OTG_DOEPINT_XFRC)
                rxArm(&p);
        }
    }

    if (pending & (1u << AUDIO_EP)) {
        uint32_t flags = OUTEP(AUDIO_EP)->DOEPINT;
        OUTEP(AUDIO_EP)->DOEPINT = flags;

        if ((flags & USB_OTG_DOEPINT_XFRC) && (altTx != 0))
            audioTxArm(false);
    }
}

static void inEndpoints()
{
    uint32_t pending = DEV->DAINT & DEV->DAINTMSK & 0xFFFF;

    if (pending & (1u << 0)) {
        uint32_t flags = INEP(0)->DIEPINT;
        INEP(0)->DIEPINT = flags;

        if (flags & USB_OTG_DIEPINT_XFRC) {
            if ((ep0State == EP0_DATA_IN) && ((ep0TxLeft > 0) || ep0TxZlp)) {
                if (ep0TxLeft == 0)
                    ep0TxZlp = false;
                ep0SendPacket();
            } else if (ep0State == EP0_STATUS) {
                ep0State = EP0_IDLE;
            }
        }
    }

    for (Port &p : ports) {
        if (pending & (1u << p.dataEp)) {
            uint32_t flags = INEP(p.dataEp)->DIEPINT;
            INEP(p.dataEp)->DIEPINT = flags;

            if (flags & USB_OTG_DIEPINT_XFRC) {
                p.txBusy = false;
                txKick(&p);
            }
        }

        if (pending & (1u << p.notifEp))
            INEP(p.notifEp)->DIEPINT = INEP(p.notifEp)->DIEPINT;
    }

    if (pending & (1u << AUDIO_EP)) {
        uint32_t flags = INEP(AUDIO_EP)->DIEPINT;
        INEP(AUDIO_EP)->DIEPINT = flags;

        if ((flags & USB_OTG_DIEPINT_XFRC) && (altRx != 0))
            audioRxSend();
    }
}

static void linkDown()
{
    suspended = true; // also what a cable unplug looks like
    hostPtt = false;
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

    if (status & USB_OTG_GINTSTS_SOF) {
        CORE->GINTSTS = USB_OTG_GINTSTS_SOF;
        startOfFrame();
    }

    while (CORE->GINTSTS & USB_OTG_GINTSTS_RXFLVL)
        rxFifoLevel();

    if (status & USB_OTG_GINTSTS_OEPINT)
        outEndpoints();

    if (status & USB_OTG_GINTSTS_IEPINT)
        inEndpoints();

    if (status & USB_OTG_GINTSTS_IISOIXFR) {
        CORE->GINTSTS = USB_OTG_GINTSTS_IISOIXFR;
        incompleteRx();
    }

    if (status & USB_OTG_GINTSTS_PXFR_INCOMPISOOUT) {
        // A transmit packet missed: the next start of frame re-aims
        CORE->GINTSTS = USB_OTG_GINTSTS_PXFR_INCOMPISOOUT;
    }

    if (status & USB_OTG_GINTSTS_USBSUSP) {
        CORE->GINTSTS = USB_OTG_GINTSTS_USBSUSP;
        linkDown();
    }

    if (status & USB_OTG_GINTSTS_WKUINT) {
        CORE->GINTSTS = USB_OTG_GINTSTS_WKUINT;
        suspended = false;
        for (Port &p : ports)
            txKick(&p);
    }
}

// Initialisation -------------------------------------------------------------

static int usbInit()
{
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

    // Keep the core clocked in sleep (the idle thread runs WFI), but not
    // the ULPI clock: there is no external PHY, and with it left enabled
    // the core stops answering the host while the CPU sleeps.
    RCC->AHB1LPENR = (RCC->AHB1LPENR & ~RCC_AHB1LPENR_USB2OTGFSULPILPEN)
                   | RCC_AHB1LPENR_USB2OTGFSLPEN;

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

    // FIFO RAM: receive FIFO, then one transmit FIFO per IN endpoint
    static const uint16_t txWords[NUM_EPS] = { TX0_FIFO_WORDS, TX1_FIFO_WORDS,
                                               TX2_FIFO_WORDS, TX3_FIFO_WORDS,
                                               TX4_FIFO_WORDS, TX5_FIFO_WORDS };
    uint32_t start = RX_FIFO_WORDS;
    CORE->GRXFSIZ = RX_FIFO_WORDS;
    CORE->DIEPTXF0_HNPTXFSIZ = (txWords[0] << 16) | start;
    start += txWords[0];
    for (int i = 1; i < NUM_EPS; i++) {
        CORE->DIEPTXF[i - 1] = ((uint32_t)txWords[i] << 16) | start;
        start += txWords[i];
    }

    flushTxFifo(0x10);
    CORE->GRSTCTL = USB_OTG_GRSTCTL_RXFFLSH;
    waitBits(&CORE->GRSTCTL, USB_OTG_GRSTCTL_RXFFLSH, 0);

    DEV->DIEPMSK = 0;
    DEV->DOEPMSK = 0;
    DEV->DAINTMSK = 0;
    for (int i = 0; i < NUM_EPS; i++) {
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

// Character devices ----------------------------------------------------------

static Port *portOf(const struct chardev *dev)
{
    return (Port *)dev->data;
}

static int usbcdc_init(const struct chardev *dev)
{
    (void)dev;
    return usbInit();
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
    for (Port &p : ports)
        p.dtr = false;
    altTx = 0;
    altRx = 0;
    hostPtt = false;
    initialized = false;

    return 0;
}

static ssize_t usbcdc_read(const struct chardev *dev, void *data,
                           const size_t len)
{
    Port *p = portOf(dev);
    uint8_t *out = (uint8_t *)data;
    size_t n = 0;

    uint32_t key = irqLock();
    while ((n < len) && (p->rxTail != p->rxHead)) {
        out[n++] = p->rxRing[p->rxTail & (p->rxSize - 1)];
        p->rxTail = p->rxTail + 1;
    }

    if (configured && !p->rxArmed)
        rxArm(p);
    irqUnlock(key);

    return n;
}

static ssize_t usbcdc_write(const struct chardev *dev, const void *data,
                            const size_t len)
{
    Port *p = portOf(dev);

    if (!initialized || !portOpen(p))
        return -ENOTCONN;

    const uint8_t *in = (const uint8_t *)data;

    uint32_t key = irqLock();
    if ((p->txSize - (p->txHead - p->txTail)) < len) {
        irqUnlock(key);
        return -EAGAIN;
    }

    for (size_t i = 0; i < len; i++)
        p->txRing[(p->txHead + i) & (p->txSize - 1)] = in[i];

    p->txHead = p->txHead + len;
    txKick(p);
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
    return initialized && portOpen(&ports[0]);
}

bool usbcat_connected()
{
    return initialized && portOpen(&ports[1]);
}

// Positional initializers: designated ones are C++20
static const struct chardev_api usbcdc_api = { usbcdc_init, usbcdc_terminate,
                                               usbcdc_read, usbcdc_write,
                                               usbcdc_ioctl };

const struct chardev usb_cdc_chardev = { NULL, // config
                                         &ports[0], &usbcdc_api };

const struct chardev usb_cat_chardev = { NULL, // config
                                         &ports[1], &usbcdc_api };

// Audio API ------------------------------------------------------------------

/*
 * Linear gain of a feature unit, recomputed when its volume changes (thread
 * code only: powf is not for the interrupt handler).
 */
struct GainCache {
    int16_t volume;
    float gain;
};

static float gainOf(const Feature *fu, GainCache *cache)
{
    if (fu->mute)
        return 0.0f;

    int16_t v = fu->volume;
    if (v != cache->volume) {
        cache->gain = powf(10.0f, ((float)v / 256.0f) / 20.0f);
        cache->volume = v;
    }

    return cache->gain;
}

static inline int16_t saturate(float v)
{
    if (v > 32767.0f)
        return 32767;
    if (v < -32768.0f)
        return -32768;
    return (int16_t)v;
}

static GainCache rxGain = { 0, 1.0f };
static GainCache txGain = { 0, 1.0f };

bool usbAudio_rxStreaming()
{
    return initialized && configured && !suspended && (altRx != 0);
}

void usbAudio_write(const int16_t *samples, size_t n)
{
    if (!usbAudio_rxStreaming())
        return;

    float g = gainOf(&fuRx, &rxGain);
    uint32_t room = RX_RING - (rxRingHead - rxRingTail);
    if (n > room)
        n = room; // overflow: the rest is dropped

    uint32_t head = rxRingHead;
    for (size_t i = 0; i < n; i++)
        rxRing[(head + i) & (RX_RING - 1)] = saturate(samples[i] * g);

    rxRingHead = head + n;
}

bool usbAudio_txStreaming()
{
    return initialized && configured && !suspended && (altTx != 0);
}

size_t usbAudio_available()
{
    return txRingHead - txRingTail;
}

size_t usbAudio_read(int16_t *samples, size_t n)
{
    float g = gainOf(&fuTx, &txGain);
    uint32_t avail = txRingHead - txRingTail;
    if (n > avail)
        n = avail;

    uint32_t tail = txRingTail;
    for (size_t i = 0; i < n; i++)
        samples[i] = saturate(txRing[(tail + i) & (TX_RING - 1)] * g);

    txRingTail = tail + n;
    return n;
}

/*
 * Resampler for a consumer paced by the radio's clock: linear interpolation
 * at a rate 1 +- 0.2% steered by the buffer level, so that the computer's
 * clock and the radio's can differ by a few hundred ppm.  Silence while the
 * buffer fills up to TX_TARGET (at the start, and after running dry).
 */
static bool txPriming = true;
static float txFrac = 0.0f;

void usbAudio_readResampled(int16_t *samples, size_t n)
{
    float g = gainOf(&fuTx, &txGain);
    uint32_t fill = txRingHead - txRingTail;

    if (txPriming && (fill >= TX_TARGET)) {
        txPriming = false;
        txFrac = 0.0f;
    }

    if (txPriming) {
        memset(samples, 0, n * sizeof(int16_t));
        return;
    }

    float err = ((float)fill - (float)TX_TARGET) / (float)TX_TARGET;
    if (err > 1.0f)
        err = 1.0f;
    if (err < -1.0f)
        err = -1.0f;
    float step = 1.0f + (0.002f * err);

    uint32_t tail = txRingTail;
    for (size_t i = 0; i < n; i++) {
        if ((txRingHead - tail) < 2) {
            // Ran dry: silence until the buffer fills again
            txPriming = true;
            memset(&samples[i], 0, (n - i) * sizeof(int16_t));
            break;
        }

        float a = txRing[tail & (TX_RING - 1)];
        float b = txRing[(tail + 1) & (TX_RING - 1)];
        samples[i] = saturate((a + ((b - a) * txFrac)) * g);

        txFrac += step;
        while (txFrac >= 1.0f) {
            txFrac -= 1.0f;
            tail++;
        }
    }

    txRingTail = tail;
}

void usbAudio_flushTx()
{
    txRingTail = txRingHead;
    txPriming = true;
}

void usbAudio_setPtt(bool on)
{
    hostPtt = on && initialized && configured && !suspended;
}

bool usbAudio_ptt()
{
    return hostPtt;
}
