/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/*
 * Host test of the STM32H7 USB CDC driver's protocol logic. The driver
 * source is included directly so its static state is reachable; the OTG
 * registers live in a plain memory block, and the data FIFOs are replaced by
 * a proxy that records writes and serves queued reads.
 *
 * What this covers: request decoding, descriptors, SET_ADDRESS,
 * SET_CONFIGURATION and endpoint setup, the SET_LINE_CODING data stage,
 * DTR gating, bulk IN chunking and the terminating ZLP, bulk OUT into the
 * receive ring with NAK flow control, CLEAR_FEATURE(ENDPOINT_HALT) toggle
 * reset, STALL on unsupported requests, bus reset.
 *
 * What it cannot cover: clocks, the PHY, timing, and how the real core
 * sequences interrupts. That needs the radio.
 */

#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <string>
#include <vector>

// Register and FIFO model --------------------------------------------------

alignas(8) static uint8_t usbMem[0x1000];
static const uint8_t fakeUid[12] = { 0x11, 0x22, 0x33, 0x44, 0x55, 0x66,
                                     0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC };

static std::vector<uint32_t> inFifo[4]; // words written per IN endpoint
static std::deque<uint32_t> rxFifo;     // words the core would hand over

struct FifoPort {
    int ep;

    FifoPort &operator=(uint32_t word)
    {
        inFifo[ep].push_back(word);
        return *this;
    }

    operator uint32_t() const
    {
        assert(!rxFifo.empty());
        uint32_t w = rxFifo.front();
        rxFifo.pop_front();
        return w;
    }
};

#define FIFO(i) (FifoPort{ (i) })

#include "stm32h7xx.h"
#undef USB2_OTG_FS_PERIPH_BASE
#define USB2_OTG_FS_PERIPH_BASE ((uintptr_t)usbMem)
#undef UID_BASE
#define UID_BASE ((uintptr_t)fakeUid)

// Stubs for what init() calls (init itself is not exercised here)
extern "C" void gpio_setMode(const void *, const uint8_t, const uint16_t)
{
}
extern "C" void delayMs(unsigned int)
{
}

#include "usb_cdc.cpp"

// Helpers ------------------------------------------------------------------

static std::vector<uint8_t> takeIn(int ep)
{
    uint32_t len = INEP(ep)->DIEPTSIZ & USB_OTG_DIEPTSIZ_XFRSIZ;
    std::vector<uint8_t> out;
    for (uint32_t w : inFifo[ep])
        for (int j = 0; j < 4; j++)
            out.push_back((uint8_t)(w >> (8 * j)));
    out.resize(len);
    inFifo[ep].clear();
    return out;
}

static uint32_t pktCount(int ep)
{
    return (INEP(ep)->DIEPTSIZ & USB_OTG_DIEPTSIZ_PKTCNT)
        >> USB_OTG_DIEPTSIZ_PKTCNT_Pos;
}

static void pushRx(uint8_t ep, uint8_t pktsts, const uint8_t *data,
                   uint32_t len)
{
    CORE->GRXSTSP = ep | (len << USB_OTG_GRXSTSP_BCNT_Pos)
                  | ((uint32_t)pktsts << USB_OTG_GRXSTSP_PKTSTS_Pos);
    for (uint32_t i = 0; i < len; i += 4) {
        uint32_t w = 0;
        memcpy(&w, data + i, (len - i) >= 4 ? 4 : (len - i));
        rxFifo.push_back(w);
    }
    rxFifoLevel();
    assert(rxFifo.empty());
}

/** Full SETUP path: RX FIFO entry, then the STUP interrupt on OUT EP0. */
static void setupReq(uint8_t bm, uint8_t req, uint16_t value, uint16_t index,
                     uint16_t length)
{
    uint8_t pkt[8] = { bm,
                       req,
                       (uint8_t)value,
                       (uint8_t)(value >> 8),
                       (uint8_t)index,
                       (uint8_t)(index >> 8),
                       (uint8_t)length,
                       (uint8_t)(length >> 8) };

    INEP(0)->DIEPCTL = 0;
    OUTEP(0)->DOEPCTL = 0;
    inFifo[0].clear();

    pushRx(0, PKT_SETUP_DATA, pkt, 8);
    DEV->DAINT = 1u << 16;
    OUTEP(0)->DOEPINT = USB_OTG_DOEPINT_STUP;
    outEndpoints();
    OUTEP(0)->DOEPINT = 0;
    DEV->DAINT = 0;
}

static bool stalled()
{
    return (INEP(0)->DIEPCTL & USB_OTG_DIEPCTL_STALL)
        && (OUTEP(0)->DOEPCTL & USB_OTG_DOEPCTL_STALL);
}

static void inComplete(int ep)
{
    DEV->DAINT = 1u << ep;
    INEP(ep)->DIEPINT = USB_OTG_DIEPINT_XFRC;
    inEndpoints();
    INEP(ep)->DIEPINT = 0;
    DEV->DAINT = 0;
}

static void outComplete(int ep)
{
    DEV->DAINT = 1u << (16 + ep);
    OUTEP(ep)->DOEPINT = USB_OTG_DOEPINT_XFRC;
    outEndpoints();
    OUTEP(ep)->DOEPINT = 0;
    DEV->DAINT = 0;
}

static bool outArmed(int ep)
{
    return (OUTEP(ep)->DOEPCTL & USB_OTG_DOEPCTL_EPENA) != 0;
}

static unsigned checks = 0;
#define CHECK(x)   \
    do {           \
        assert(x); \
        checks++;  \
    } while (0)

int main()
{
    initialized = true;
    busReset();
    CHECK(!usbcdc_connected());
    CHECK(DEV->DAINTMSK == ((1u << 0) | (1u << 16)));
    CHECK(outArmed(0));

    // Device descriptor, first asked with wLength 64: 18 bytes, one packet
    setupReq(0x80, 6, 0x0100, 0, 64);
    std::vector<uint8_t> d = takeIn(0);
    CHECK(d.size() == 18);
    CHECK(memcmp(d.data(), devDesc, 18) == 0);
    CHECK(d[8] == 0x83 && d[9] == 0x04 && d[10] == 0x40 && d[11] == 0x57);
    CHECK(pktCount(0) == 1);
    CHECK(ep0State == EP0_DATA_IN);
    CHECK(outArmed(0)); // ready for the status ZLP
    outComplete(0);
    CHECK(ep0State == EP0_IDLE);

    // SET_ADDRESS applied before the status stage
    setupReq(0x00, 5, 0x2A, 0, 0);
    CHECK(((DEV->DCFG & USB_OTG_DCFG_DAD) >> USB_OTG_DCFG_DAD_Pos) == 0x2A);
    CHECK(takeIn(0).empty() && pktCount(0) == 1);
    inComplete(0);
    CHECK(ep0State == EP0_IDLE);

    // Configuration descriptor: header only, then all 67 bytes in 2 packets
    setupReq(0x80, 6, 0x0200, 0, 9);
    d = takeIn(0);
    CHECK(d.size() == 9 && d[2] == 67 && d[3] == 0);
    setupReq(0x80, 6, 0x0200, 0, 255);
    d = takeIn(0);
    CHECK(d.size() == 67);
    CHECK(memcmp(d.data(), cfgDesc, 67) == 0);
    CHECK(pktCount(0) == 2);
    {
        // Walk the descriptors: lengths must add up to wTotalLength
        size_t pos = 0;
        unsigned eps = 0;
        while (pos < d.size()) {
            CHECK(d[pos] >= 2);
            if (d[pos + 1] == 5)
                eps++;
            pos += d[pos];
        }
        CHECK(pos == 67 && eps == 3);
    }

    // Strings: language, manufacturer, product, serial from the UID
    setupReq(0x80, 6, 0x0300, 0, 255);
    d = takeIn(0);
    CHECK(d.size() == 4 && d[2] == 0x09 && d[3] == 0x04);
    setupReq(0x80, 6, 0x0302, 0x0409, 255);
    d = takeIn(0);
    CHECK(d[0] == d.size() && d[1] == 3);
    std::string product;
    for (size_t i = 2; i < d.size(); i += 2)
        product += (char)d[i];
    CHECK(product == "OpenRTX rtxlink");
    setupReq(0x80, 6, 0x0303, 0x0409, 255);
    d = takeIn(0);
    std::string serial;
    for (size_t i = 2; i < d.size(); i += 2)
        serial += (char)d[i];
    CHECK(serial == "CCBBAA998877665544332211");

    // Device qualifier: full-speed device, must STALL; next SETUP still works
    setupReq(0x80, 6, 0x0600, 0, 10);
    CHECK(stalled());
    setupReq(0x80, 6, 0x0304, 0, 255); // no such string
    CHECK(stalled());
    setupReq(0x40, 0x01, 0, 0, 0);     // vendor request
    CHECK(stalled());

    // SET_CONFIGURATION 1: data endpoints come up, OUT armed for 64 bytes
    setupReq(0x00, 9, 1, 0, 0);
    CHECK(configured);
    CHECK(INEP(1)->DIEPCTL & USB_OTG_DIEPCTL_USBAEP);
    CHECK(((INEP(1)->DIEPCTL & USB_OTG_DIEPCTL_EPTYP)
           >> USB_OTG_DIEPCTL_EPTYP_Pos)
          == 2);
    CHECK(((INEP(1)->DIEPCTL & USB_OTG_DIEPCTL_TXFNUM)
           >> USB_OTG_DIEPCTL_TXFNUM_Pos)
          == 1);
    CHECK((INEP(1)->DIEPCTL & USB_OTG_DIEPCTL_MPSIZ) == 64);
    CHECK(((INEP(2)->DIEPCTL & USB_OTG_DIEPCTL_EPTYP)
           >> USB_OTG_DIEPCTL_EPTYP_Pos)
          == 3);
    CHECK(outArmed(1));
    CHECK((OUTEP(1)->DOEPTSIZ & USB_OTG_DOEPTSIZ_XFRSIZ) == 64);
    setupReq(0x80, 8, 0, 0, 1);
    d = takeIn(0);
    CHECK(d.size() == 1 && d[0] == 1);

    // SET_LINE_CODING: data stage OUT, then status IN
    setupReq(0x21, 0x20, 0, 0, 7);
    CHECK(ep0State == EP0_DATA_OUT);
    CHECK((OUTEP(0)->DOEPTSIZ & USB_OTG_DOEPTSIZ_XFRSIZ) == 7);
    const uint8_t lc[7] = { 0x00, 0x10, 0x0E, 0x00, 0, 0, 8 }; // 921600
    pushRx(0, PKT_OUT_DATA, lc, 7);
    outComplete(0);
    CHECK(memcmp(lineCoding, lc, 7) == 0);
    CHECK(ep0State == EP0_STATUS && pktCount(0) == 1 && takeIn(0).empty());
    setupReq(0xA1, 0x21, 0, 0, 7);
    d = takeIn(0);
    CHECK(d.size() == 7 && memcmp(d.data(), lc, 7) == 0);

    // Nothing goes out before DTR
    const char msg[] = "hello";
    CHECK(usb_cdc_chardev.api->write(&usb_cdc_chardev, msg, 5) == -ENOTCONN);
    CHECK(inFifo[1].empty());

    setupReq(0x21, 0x22, 0x0003, 0, 0); // DTR + RTS
    CHECK(usbcdc_connected());

    // 150 bytes: 64 + 64 + 22, no ZLP after the short packet
    uint8_t big[150];
    for (int i = 0; i < 150; i++)
        big[i] = (uint8_t)(i * 7 + 1);
    CHECK(usb_cdc_chardev.api->write(&usb_cdc_chardev, big, 150) == 150);
    std::vector<uint8_t> all;
    d = takeIn(1);
    CHECK(d.size() == 64 && txBusy);
    all.insert(all.end(), d.begin(), d.end());
    inComplete(1);
    d = takeIn(1);
    CHECK(d.size() == 64);
    all.insert(all.end(), d.begin(), d.end());
    inComplete(1);
    d = takeIn(1);
    CHECK(d.size() == 22);
    all.insert(all.end(), d.begin(), d.end());
    CHECK(memcmp(all.data(), big, 150) == 0);
    inComplete(1);
    CHECK(!txBusy && inFifo[1].empty());

    // Exactly 64 bytes: full packet, then a ZLP to end the transfer
    CHECK(usb_cdc_chardev.api->write(&usb_cdc_chardev, big, 64) == 64);
    CHECK(takeIn(1).size() == 64);
    inComplete(1);
    CHECK(txBusy && pktCount(1) == 1 && takeIn(1).empty()); // the ZLP
    inComplete(1);
    CHECK(!txBusy);

    // All-or-nothing: a write larger than the free space queues nothing
    static uint8_t huge[TX_RING_SIZE + 1];
    CHECK(usb_cdc_chardev.api->write(&usb_cdc_chardev, huge, sizeof(huge))
          == -EAGAIN);
    CHECK(inFifo[1].empty() && !txBusy);

    // Ring wrap-around: push enough data through to wrap the indices
    for (int round = 0; round < 40; round++) {
        CHECK(usb_cdc_chardev.api->write(&usb_cdc_chardev, big, 100) == 100);
        all.clear();
        while (txBusy) {
            d = takeIn(1);
            all.insert(all.end(), d.begin(), d.end());
            inComplete(1);
        }
        CHECK(all.size() == 100 && memcmp(all.data(), big, 100) == 0);
    }

    // Bulk OUT: data lands in the ring and comes out of read()
    const uint8_t cmd[10] = { 0xC0, 0x04, 'L', 3, 1, 2, 3, 4, 5, 0xC0 };
    pushRx(1, PKT_OUT_DATA, cmd, 10);
    outComplete(1);
    CHECK(outArmed(1));
    uint8_t rbuf[64];
    CHECK(usb_cdc_chardev.api->read(&usb_cdc_chardev, rbuf, sizeof(rbuf))
          == 10);
    CHECK(memcmp(rbuf, cmd, 10) == 0);

    // Flow control: fill the 256-byte ring, the endpoint stays NAKed until
    // read() makes room for a full packet
    uint8_t pkt[64];
    memset(pkt, 0x5A, sizeof(pkt));
    for (int i = 0; i < 4; i++) {
        pushRx(1, PKT_OUT_DATA, pkt, 64);
        OUTEP(1)->DOEPCTL &= ~USB_OTG_DOEPCTL_EPENA; // core clears it
        outComplete(1);
    }
    CHECK(!outArmed(1) && !rxArmed);
    CHECK(usb_cdc_chardev.api->read(&usb_cdc_chardev, rbuf, 10) == 10);
    CHECK(!outArmed(1)); // 10 free bytes is not enough
    CHECK(usb_cdc_chardev.api->read(&usb_cdc_chardev, rbuf, 64) == 64);
    CHECK(outArmed(1) && rxArmed);

    // CLEAR_FEATURE(ENDPOINT_HALT) on 0x81 and 0x01 resets the data toggle
    INEP(1)->DIEPCTL &= ~USB_OTG_DIEPCTL_SD0PID_SEVNFRM;
    OUTEP(1)->DOEPCTL &= ~USB_OTG_DOEPCTL_SD0PID_SEVNFRM;
    setupReq(0x02, 1, 0, 0x81, 0);
    CHECK(INEP(1)->DIEPCTL & USB_OTG_DIEPCTL_SD0PID_SEVNFRM);
    setupReq(0x02, 1, 0, 0x01, 0);
    CHECK(OUTEP(1)->DOEPCTL & USB_OTG_DOEPCTL_SD0PID_SEVNFRM);

    // Suspend stops output without losing it; resume restarts it
    suspended = true;
    CHECK(!usbcdc_connected());
    CHECK(usb_cdc_chardev.api->write(&usb_cdc_chardev, msg, 5) == -ENOTCONN);
    suspended = false;

    // DTR drop: port closed, output held
    setupReq(0x21, 0x22, 0x0000, 0, 0);
    CHECK(!usbcdc_connected());

    // Bus reset: back to the default state
    setupReq(0x21, 0x22, 0x0001, 0, 0);
    CHECK(usbcdc_connected());
    busReset();
    CHECK(!configured && !dtr && !usbcdc_connected());
    CHECK((DEV->DCFG & USB_OTG_DCFG_DAD) == 0);
    CHECK(DEV->DAINTMSK == ((1u << 0) | (1u << 16)));

    printf("usb_cdc_test: %u checks passed\n", checks);
    return 0;
}
