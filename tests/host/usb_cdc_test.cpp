/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/*
 * Host test of the STM32H7 composite USB driver's protocol logic (two
 * CDC-ACM ports and a USB audio function). The driver source is included
 * directly so its static state is reachable; the OTG registers live in a
 * plain memory block, and the data FIFOs are replaced by a proxy that
 * records writes and serves queued reads.
 *
 * What this covers: request decoding, descriptors (walked and checked),
 * multi-packet control transfers, SET_ADDRESS, SET_CONFIGURATION and
 * endpoint setup, the SET_LINE_CODING data stage per port, DTR gating of the
 * log port only, bulk IN chunking and the terminating ZLP, bulk OUT into the
 * receive ring with NAK flow control, CLEAR_FEATURE(ENDPOINT_HALT) toggle
 * reset, STALL on unsupported requests, the audio alternate settings,
 * isochronous IN packet sizing and frame parity, isochronous OUT into the
 * transmit ring, the resampler, feature unit volume and mute, the transmit
 * request dropped on suspend and reset, bus reset.
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
#include <cmath>
#include <cstdlib>

// Register and FIFO model --------------------------------------------------

alignas(8) static uint8_t usbMem[0x1000];
static const uint8_t fakeUid[12] = { 0x11, 0x22, 0x33, 0x44, 0x55, 0x66,
                                     0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC };

static std::vector<uint32_t> inFifo[6]; // words written per IN endpoint
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

// Port shorthands
static Port &logPort = ports[0];
static Port &catPort = ports[1];

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

/** Whole IN data stage of a control transfer, packet by packet. */
static std::vector<uint8_t> takeEp0(unsigned *packets = NULL)
{
    std::vector<uint8_t> all;
    unsigned n = 0;
    for (;;) {
        std::vector<uint8_t> d = takeIn(0);
        all.insert(all.end(), d.begin(), d.end());
        n++;
        if ((ep0State == EP0_DATA_IN) && ((ep0TxLeft > 0) || ep0TxZlp))
            inComplete(0);
        else
            break;
    }
    if (packets != NULL)
        *packets = n;
    return all;
}

static void setFrame(uint32_t frame)
{
    DEV->DSTS = frame << USB_OTG_DSTS_FNSOF_Pos;
}

static void sof(uint32_t frame)
{
    setFrame(frame);
    startOfFrame();
}

/** Samples of the isochronous IN packet just armed. */
static std::vector<int16_t> takeAudioIn()
{
    std::vector<uint8_t> b = takeIn(AUDIO_EP);
    std::vector<int16_t> s(b.size() / 2);
    memcpy(s.data(), b.data(), s.size() * 2);
    return s;
}

static void pushAudioOut(const int16_t *s, uint32_t n)
{
    pushRx(AUDIO_EP, PKT_OUT_DATA, (const uint8_t *)s, 2 * n);
    OUTEP(AUDIO_EP)->DOEPCTL &= ~USB_OTG_DOEPCTL_EPENA; // core clears it
    outComplete(AUDIO_EP);
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
    CHECK(!usbcdc_connected() && !usbcat_connected());
    CHECK(DEV->DAINTMSK == ((1u << 0) | (1u << 16)));
    CHECK(outArmed(0));

    // Device descriptor, first asked with wLength 64: 18 bytes, one packet
    setupReq(0x80, 6, 0x0100, 0, 64);
    unsigned pkts;
    std::vector<uint8_t> d = takeEp0(&pkts);
    CHECK(d.size() == 18 && pkts == 1);
    CHECK(memcmp(d.data(), devDesc, 18) == 0);
    CHECK(d[4] == 0xEF && d[5] == 0x02 && d[6] == 0x01); // IAD device class
    CHECK(d[8] == 0x83 && d[9] == 0x04 && d[10] == 0x40 && d[11] == 0x57);
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

    // Configuration descriptor: header only, then all of it in 64-byte packets
    setupReq(0x80, 6, 0x0200, 0, 9);
    d = takeEp0();
    CHECK(d.size() == 9 && (d[2] | (d[3] << 8)) == CFG_LEN);
    CHECK(d[4] == 7); // interfaces
    setupReq(0x80, 6, 0x0200, 0, 0xFFFF);
    d = takeEp0(&pkts);
    CHECK(d.size() == CFG_LEN && pkts == (CFG_LEN + 63) / 64);
    CHECK(memcmp(d.data(), cfgDesc, CFG_LEN) == 0);
    {
        // Walk the descriptors: lengths add up, the counts are right, the
        // audio control header's total length covers its class descriptors
        size_t pos = 0;
        unsigned eps = 0, intfs = 0, iads = 0, acTotal = 0, acSeen = 0;
        bool inAc = false;
        while (pos < d.size()) {
            uint8_t len = d[pos], type = d[pos + 1];
            CHECK(len >= 2 && (pos + len) <= d.size());
            if (type == 5)
                eps++;
            if (type == 11)
                iads++;
            if (type == 4) {
                if (d[pos + 3] == 0)
                    intfs++; // alternate 0 only
                inAc = (d[pos + 5] == 0x01) && (d[pos + 6] == 0x01);
            } else if (inAc && (type == 0x24)) {
                if (d[pos + 2] == 0x01)
                    acTotal = d[pos + 5] | (d[pos + 6] << 8);
                acSeen += len;
            }
            pos += len;
        }
        CHECK(pos == CFG_LEN);
        CHECK(intfs == 7 && iads == 3 && eps == 8);
        CHECK(acTotal == AC_CLASS_LEN && acSeen == AC_CLASS_LEN);
    }

    // A 64-byte-multiple answer shorter than asked ends with a ZLP
    {
        static const uint8_t blob[128] = { 1 };
        setupReq(0x80, 6, 0x0200, 0, 200);
        ep0Send(blob, 128);
        d = takeEp0(&pkts);
        CHECK(d.size() == 128 && pkts == 3); // 64 + 64 + ZLP
    }

    // Strings: language, manufacturer, product, serial, function names
    setupReq(0x80, 6, 0x0300, 0, 255);
    d = takeEp0();
    CHECK(d.size() == 4 && d[2] == 0x09 && d[3] == 0x04);
    auto str = [&](uint8_t idx) {
        setupReq(0x80, 6, 0x0300 | idx, 0x0409, 255);
        std::vector<uint8_t> s = takeEp0();
        CHECK(s[0] == s.size() && s[1] == 3);
        std::string out;
        for (size_t i = 2; i < s.size(); i += 2)
            out += (char)s[i];
        return out;
    };
    CHECK(str(1) == "OpenRTX");
    CHECK(str(2) == "OpenRTX radio");
    CHECK(str(3) == "CCBBAA998877665544332211");
    CHECK(str(STR_LOG) == "OpenRTX log");
    CHECK(str(STR_CAT) == "OpenRTX CAT");
    CHECK(str(STR_AUDIO) == "OpenRTX audio");

    // Device qualifier: full-speed device, must STALL; next SETUP still works
    setupReq(0x80, 6, 0x0600, 0, 10);
    CHECK(stalled());
    setupReq(0x80, 6, 0x0307, 0, 255); // no such string
    CHECK(stalled());
    setupReq(0x40, 0x01, 0, 0, 0);     // vendor request
    CHECK(stalled());

    // SET_CONFIGURATION 1: both ports' endpoints come up, OUT armed
    setupReq(0x00, 9, 1, 0, 0);
    CHECK(configured);
    for (Port *p : { &logPort, &catPort }) {
        uint8_t ep = p->dataEp;
        CHECK(INEP(ep)->DIEPCTL & USB_OTG_DIEPCTL_USBAEP);
        CHECK(((INEP(ep)->DIEPCTL & USB_OTG_DIEPCTL_EPTYP)
               >> USB_OTG_DIEPCTL_EPTYP_Pos)
              == 2);
        CHECK(((INEP(ep)->DIEPCTL & USB_OTG_DIEPCTL_TXFNUM)
               >> USB_OTG_DIEPCTL_TXFNUM_Pos)
              == ep);
        CHECK((INEP(ep)->DIEPCTL & USB_OTG_DIEPCTL_MPSIZ) == 64);
        CHECK(((INEP(p->notifEp)->DIEPCTL & USB_OTG_DIEPCTL_EPTYP)
               >> USB_OTG_DIEPCTL_EPTYP_Pos)
              == 3);
        CHECK(outArmed(ep));
        CHECK((OUTEP(ep)->DOEPTSIZ & USB_OTG_DOEPTSIZ_XFRSIZ) == 64);
    }
    CHECK(logPort.dataEp == 1 && catPort.dataEp == 3);
    setupReq(0x80, 8, 0, 0, 1);
    d = takeEp0();
    CHECK(d.size() == 1 && d[0] == 1);

    // SET_LINE_CODING: data stage OUT, then status IN; per port
    setupReq(0x21, 0x20, 0, 0, 7);
    CHECK(ep0State == EP0_DATA_OUT);
    CHECK((OUTEP(0)->DOEPTSIZ & USB_OTG_DOEPTSIZ_XFRSIZ) == 7);
    const uint8_t lc[7] = { 0x00, 0x10, 0x0E, 0x00, 0, 0, 8 }; // 921600
    pushRx(0, PKT_OUT_DATA, lc, 7);
    outComplete(0);
    CHECK(memcmp(logPort.lineCoding, lc, 7) == 0);
    CHECK(ep0State == EP0_STATUS && pktCount(0) == 1 && takeIn(0).empty());
    setupReq(0xA1, 0x21, 0, 0, 7);
    d = takeEp0();
    CHECK(d.size() == 7 && memcmp(d.data(), lc, 7) == 0);
    const uint8_t lc2[7] = { 0x80, 0x25, 0x00, 0x00, 0, 0, 8 }; // 9600
    setupReq(0x21, 0x20, 0, 2, 7);
    pushRx(0, PKT_OUT_DATA, lc2, 7);
    outComplete(0);
    CHECK(memcmp(catPort.lineCoding, lc2, 7) == 0);
    CHECK(memcmp(logPort.lineCoding, lc, 7) == 0);
    setupReq(0x21, 0x20, 0, 1, 7); // a data interface: no such request
    CHECK(stalled());

    // Log port: nothing goes out before DTR.  CAT port: no DTR needed.
    const char msg[] = "hello";
    CHECK(usb_cdc_chardev.api->write(&usb_cdc_chardev, msg, 5) == -ENOTCONN);
    CHECK(inFifo[1].empty());
    CHECK(usbcat_connected());
    CHECK(usb_cat_chardev.api->write(&usb_cat_chardev, "FA;", 3) == 3);
    d = takeIn(3);
    CHECK(d.size() == 3 && memcmp(d.data(), "FA;", 3) == 0);
    inComplete(3);
    CHECK(!catPort.txBusy);

    setupReq(0x21, 0x22, 0x0003, 0, 0); // DTR + RTS on the log port
    CHECK(usbcdc_connected());
    CHECK(logPort.dtr && !catPort.dtr);

    // 150 bytes: 64 + 64 + 22, no ZLP after the short packet
    uint8_t big[150];
    for (int i = 0; i < 150; i++)
        big[i] = (uint8_t)(i * 7 + 1);
    CHECK(usb_cdc_chardev.api->write(&usb_cdc_chardev, big, 150) == 150);
    std::vector<uint8_t> all;
    d = takeIn(1);
    CHECK(d.size() == 64 && logPort.txBusy);
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
    CHECK(!logPort.txBusy && inFifo[1].empty());

    // Exactly 64 bytes: full packet, then a ZLP to end the transfer
    CHECK(usb_cdc_chardev.api->write(&usb_cdc_chardev, big, 64) == 64);
    CHECK(takeIn(1).size() == 64);
    inComplete(1);
    CHECK(logPort.txBusy && pktCount(1) == 1 && takeIn(1).empty()); // ZLP
    inComplete(1);
    CHECK(!logPort.txBusy);

    // All-or-nothing: a write larger than the free space queues nothing
    static uint8_t huge[LOG_TX_RING + 1];
    CHECK(usb_cdc_chardev.api->write(&usb_cdc_chardev, huge, sizeof(huge))
          == -EAGAIN);
    CHECK(usb_cat_chardev.api->write(&usb_cat_chardev, huge, CAT_TX_RING + 1)
          == -EAGAIN);
    CHECK(inFifo[1].empty() && !logPort.txBusy);

    // Ring wrap-around: push enough data through to wrap the indices
    for (int round = 0; round < 40; round++) {
        CHECK(usb_cdc_chardev.api->write(&usb_cdc_chardev, big, 100) == 100);
        all.clear();
        while (logPort.txBusy) {
            d = takeIn(1);
            all.insert(all.end(), d.begin(), d.end());
            inComplete(1);
        }
        CHECK(all.size() == 100 && memcmp(all.data(), big, 100) == 0);
    }

    // Bulk OUT: data lands in the right port's ring and comes out of read()
    const uint8_t cmd[10] = { 0xC0, 0x04, 'L', 3, 1, 2, 3, 4, 5, 0xC0 };
    pushRx(1, PKT_OUT_DATA, cmd, 10);
    outComplete(1);
    CHECK(outArmed(1));
    pushRx(3, PKT_OUT_DATA, (const uint8_t *)"IF;", 3);
    outComplete(3);
    uint8_t rbuf[64];
    CHECK(usb_cdc_chardev.api->read(&usb_cdc_chardev, rbuf, sizeof(rbuf))
          == 10);
    CHECK(memcmp(rbuf, cmd, 10) == 0);
    CHECK(usb_cat_chardev.api->read(&usb_cat_chardev, rbuf, sizeof(rbuf)) == 3);
    CHECK(memcmp(rbuf, "IF;", 3) == 0);

    // Flow control: fill the 256-byte ring, the endpoint stays NAKed until
    // read() makes room for a full packet
    uint8_t pkt[64];
    memset(pkt, 0x5A, sizeof(pkt));
    for (int i = 0; i < 4; i++) {
        pushRx(1, PKT_OUT_DATA, pkt, 64);
        OUTEP(1)->DOEPCTL &= ~USB_OTG_DOEPCTL_EPENA; // core clears it
        outComplete(1);
    }
    CHECK(!outArmed(1) && !logPort.rxArmed);
    CHECK(usb_cdc_chardev.api->read(&usb_cdc_chardev, rbuf, 10) == 10);
    CHECK(!outArmed(1)); // 10 free bytes is not enough
    CHECK(usb_cdc_chardev.api->read(&usb_cdc_chardev, rbuf, 64) == 64);
    CHECK(outArmed(1) && logPort.rxArmed);

    // CLEAR_FEATURE(ENDPOINT_HALT) on 0x83 and 0x03 resets the data toggle
    INEP(3)->DIEPCTL &= ~USB_OTG_DIEPCTL_SD0PID_SEVNFRM;
    OUTEP(3)->DOEPCTL &= ~USB_OTG_DOEPCTL_SD0PID_SEVNFRM;
    setupReq(0x02, 1, 0, 0x83, 0);
    CHECK(INEP(3)->DIEPCTL & USB_OTG_DIEPCTL_SD0PID_SEVNFRM);
    setupReq(0x02, 1, 0, 0x03, 0);
    CHECK(OUTEP(3)->DOEPCTL & USB_OTG_DOEPCTL_SD0PID_SEVNFRM);

    // Audio: nothing streams before the host selects alternate setting 1
    CHECK(!usbAudio_rxStreaming() && !usbAudio_txStreaming());
    CHECK(!(CORE->GINTMSK & USB_OTG_GINTMSK_SOFM));
    setupReq(0x81, 0x0A, 0, 6, 1); // GET_INTERFACE
    d = takeEp0();
    CHECK(d.size() == 1 && d[0] == 0);
    setupReq(0x01, 0x0B, 2, 6, 0); // no alternate setting 2
    CHECK(stalled());
    setupReq(0x01, 0x0B, 1, 3, 0); // nor 1 on a CDC interface
    CHECK(stalled());

    // Receive stream (radio -> computer): interface 6, alternate 1.  The
    // first packet is armed at once, 48 samples of silence, for the next
    // frame: frame 101 is odd, so the even frame
    setFrame(101);
    setupReq(0x01, 0x0B, 1, 6, 0);
    CHECK(usbAudio_rxStreaming() && (CORE->GINTMSK & USB_OTG_GINTMSK_SOFM));
    CHECK(INEP(AUDIO_EP)->DIEPCTL & USB_OTG_DIEPCTL_USBAEP);
    CHECK(((INEP(AUDIO_EP)->DIEPCTL & USB_OTG_DIEPCTL_EPTYP)
           >> USB_OTG_DIEPCTL_EPTYP_Pos)
          == 1);
    CHECK(INEP(AUDIO_EP)->DIEPCTL & USB_OTG_DIEPCTL_SD0PID_SEVNFRM);
    CHECK(!(INEP(AUDIO_EP)->DIEPCTL & USB_OTG_DIEPCTL_SODDFRM));
    CHECK(((INEP(AUDIO_EP)->DIEPTSIZ & USB_OTG_DIEPTSIZ_MULCNT)
           >> USB_OTG_DIEPTSIZ_MULCNT_Pos)
          == 1);
    std::vector<int16_t> a = takeAudioIn();
    CHECK(a.size() == 48);
    for (int16_t s : a)
        CHECK(s == 0);
    setupReq(0x81, 0x0A, 0, 6, 1);
    d = takeEp0();
    CHECK(d.size() == 1 && d[0] == 1);

    // The radio writes 30 ms: still priming (40 ms), silence goes out
    // Samples: a ramp, sample i = i & 0x7FFF
    size_t wr = 0;
    auto writeRamp = [&](size_t n) {
        std::vector<int16_t> blk(n);
        for (size_t i = 0; i < n; i++)
            blk[i] = (int16_t)((wr + i) & 0x7FFF);
        usbAudio_write(blk.data(), n);
        wr += n;
    };
    auto ramp = [](size_t i) { return (int16_t)(i & 0x7FFF); };
    writeRamp(1440);
    INEP(AUDIO_EP)->DIEPCTL &= ~(USB_OTG_DIEPCTL_SD0PID_SEVNFRM
                                 | USB_OTG_DIEPCTL_SODDFRM);
    setFrame(102);
    inComplete(AUDIO_EP);
    CHECK(INEP(AUDIO_EP)->DIEPCTL & USB_OTG_DIEPCTL_SODDFRM); // frame 103
    a = takeAudioIn();
    CHECK(a.size() == 48 && a[0] == 0 && a[47] == 0);

    // Past 40 ms the samples flow, in order, 47 to 49 per frame as the level
    // settles, then 48 per frame while the radio keeps pace (20 ms blocks,
    // as the radio writes them)
    writeRamp(960);
    std::vector<int16_t> got;
    unsigned not48 = 0;
    for (int f = 0; f < 2000; f++) {
        if ((f % 20) == 0)
            writeRamp(960);
        inComplete(AUDIO_EP);
        a = takeAudioIn();
        CHECK(a.size() >= 47 && a.size() <= 49);
        if ((f >= 1000) && (a.size() != 48))
            not48++;
        got.insert(got.end(), a.begin(), a.end());
    }
    for (size_t i = 0; i < got.size(); i++)
        CHECK(got[i] == ramp(i));
    CHECK(not48 == 0 && rxUnderruns == 0);

    // The radio running 1% fast: the device sends 49 now and then, and the
    // buffer stays bounded with no sample lost or repeated
    size_t sent = got.size();
    unsigned n49 = 0;
    for (int f = 0; f < 4000; f++) {
        if ((f % 20) == 0)
            writeRamp(970);
        inComplete(AUDIO_EP);
        a = takeAudioIn();
        CHECK(a.size() >= 47 && a.size() <= 49);
        if (a.size() == 49)
            n49++;
        for (size_t i = 0; i < a.size(); i++)
            CHECK(a[i] == ramp(sent + i));
        sent += a.size();
    }
    CHECK(n49 > 300 && rxUnderruns == 0);
    CHECK((rxRingHead - rxRingTail) < (RX_TARGET + 1500));

    // An IN packet the host did not take is retried in the next frame
    INEP(AUDIO_EP)->DIEPCTL |= USB_OTG_DIEPCTL_EPENA
                             | USB_OTG_DIEPCTL_EONUM_DPID; // odd frame
    INEP(AUDIO_EP)->DIEPCTL &= ~(USB_OTG_DIEPCTL_SD0PID_SEVNFRM
                                 | USB_OTG_DIEPCTL_SODDFRM);
    setFrame(301);
    incompleteRx();
    CHECK(INEP(AUDIO_EP)->DIEPCTL & USB_OTG_DIEPCTL_SD0PID_SEVNFRM);
    INEP(AUDIO_EP)->DIEPCTL &= ~(USB_OTG_DIEPCTL_EPENA
                                 | USB_OTG_DIEPCTL_EONUM_DPID
                                 | USB_OTG_DIEPCTL_SD0PID_SEVNFRM);
    takeIn(AUDIO_EP);

    // A frame with the endpoint idle (no XFRC seen) is re-armed at SOF
    sof(302);
    CHECK(INEP(AUDIO_EP)->DIEPCTL & USB_OTG_DIEPCTL_EPENA);
    a = takeAudioIn();
    CHECK(a.size() >= 47 && a.size() <= 49);

    // Receive volume: -6 dB halves the samples; mute silences them
    setupReq(0xA1, 0x82, 0x0200, (AC_FU_RX << 8) | 4, 2); // GET_MIN
    d = takeEp0();
    CHECK(d.size() == 2 && (int16_t)(d[0] | (d[1] << 8)) == VOL_MIN);
    setupReq(0xA1, 0x83, 0x0200, (AC_FU_RX << 8) | 4, 2); // GET_MAX
    d = takeEp0();
    CHECK((int16_t)(d[0] | (d[1] << 8)) == VOL_MAX);
    setupReq(0xA1, 0x84, 0x0200, (AC_FU_RX << 8) | 4, 2); // GET_RES
    d = takeEp0();
    CHECK((int16_t)(d[0] | (d[1] << 8)) == VOL_RES);
    const uint8_t minus6[2] = { 0x00, 0xFA }; // -6 dB
    setupReq(0x21, 0x01, 0x0200, (AC_FU_RX << 8) | 4, 2);
    CHECK(ep0State == EP0_DATA_OUT);
    pushRx(0, PKT_OUT_DATA, minus6, 2);
    outComplete(0);
    CHECK(fuRx.volume == -6 * 256 && ep0State == EP0_STATUS);
    setupReq(0xA1, 0x81, 0x0200, (AC_FU_RX << 8) | 4, 2); // GET_CUR
    d = takeEp0();
    CHECK(d.size() == 2 && d[0] == 0x00 && d[1] == 0xFA);
    {
        int16_t one[1] = { 10000 };
        uint32_t h = rxRingHead;
        usbAudio_write(one, 1);
        CHECK(rxRing[h & (RX_RING - 1)] > 5000
              && rxRing[h & (RX_RING - 1)] < 5030);
        const uint8_t on[1] = { 1 };
        setupReq(0x21, 0x01, 0x0100, (AC_FU_RX << 8) | 4, 1);
        pushRx(0, PKT_OUT_DATA, on, 1);
        outComplete(0);
        CHECK(fuRx.mute);
        h = rxRingHead;
        usbAudio_write(one, 1);
        CHECK(rxRing[h & (RX_RING - 1)] == 0);
        setupReq(0xA1, 0x81, 0x0100, (AC_FU_RX << 8) | 4, 1);
        d = takeEp0();
        CHECK(d.size() == 1 && d[0] == 1);
    }
    setupReq(0xA1, 0x81, 0x0300, (AC_FU_RX << 8) | 4, 1); // bass: none
    CHECK(stalled());
    setupReq(0xA1, 0x81, 0x0200, (3 << 8) | 4, 2); // a terminal: no controls
    CHECK(stalled());

    // Sampling frequency on the endpoint: 48 kHz, SET_CUR accepted
    setupReq(0xA2, 0x81, 0x0100, 0x85, 3);
    d = takeEp0();
    CHECK(d.size() == 3 && d[0] == 0x80 && d[1] == 0xBB && d[2] == 0x00);
    const uint8_t f48[3] = { 0x80, 0xBB, 0x00 };
    setupReq(0x22, 0x01, 0x0100, 0x05, 3);
    pushRx(0, PKT_OUT_DATA, f48, 3);
    outComplete(0);
    CHECK(ep0State == EP0_STATUS);

    // Transmit stream (computer -> radio): interface 5, alternate 1.  The
    // OUT endpoint is armed for the next frame, and re-aimed at each SOF
    setFrame(400);
    setupReq(0x01, 0x0B, 1, 5, 0);
    CHECK(usbAudio_txStreaming() && outArmed(AUDIO_EP));
    CHECK(OUTEP(AUDIO_EP)->DOEPCTL & USB_OTG_DOEPCTL_SODDFRM); // frame 401
    CHECK((OUTEP(AUDIO_EP)->DOEPTSIZ & USB_OTG_DOEPTSIZ_XFRSIZ) == AUDIO_MPS);
    OUTEP(AUDIO_EP)->DOEPCTL &= ~(USB_OTG_DOEPCTL_SODDFRM
                                  | USB_OTG_DOEPCTL_SD0PID_SEVNFRM);
    sof(402);
    CHECK(OUTEP(AUDIO_EP)->DOEPCTL & USB_OTG_DOEPCTL_SD0PID_SEVNFRM);

    // Host-paced read: the samples, in order
    usbAudio_flushTx();
    int16_t frame48[48];
    for (int f = 0; f < 10; f++) {
        for (int i = 0; i < 48; i++)
            frame48[i] = (int16_t)((f * 48) + i);
        pushAudioOut(frame48, 48);
        CHECK(outArmed(AUDIO_EP));
    }
    CHECK(usbAudio_available() == 480);
    int16_t rd[480];
    CHECK(usbAudio_read(rd, 480) == 480);
    for (int i = 0; i < 480; i++)
        CHECK(rd[i] == i);
    CHECK(usbAudio_read(rd, 10) == 0);

    // Radio-paced read: silence until 40 ms are in, then the samples follow
    // within the resampler's +-0.2%, and the level stays bounded with the
    // computer 0.1% fast
    usbAudio_flushTx();
    std::vector<int16_t> out;
    double phase = 0.0;
    for (int ms = 0; ms < 3000; ms++) {
        // The computer: 48 samples per frame, 49 every 1000th
        int16_t pk[50];
        int n = ((ms % 1000) == 999) ? 49 : 48;
        for (int i = 0; i < n; i++) {
            pk[i] = (int16_t)(10000.0 * sin(phase));
            phase += 2 * M_PI * 1000.0 / 48000.0;
        }
        pushAudioOut(pk, n);
        // The radio: 960 samples every 20 ms
        if ((ms % 20) == 19) {
            int16_t blk[960];
            usbAudio_readResampled(blk, 960);
            out.insert(out.end(), blk, blk + 960);
        }
    }
    CHECK(out[0] == 0 && out[959] == 0); // primed only after 40 ms
    uint32_t level = txRingHead - txRingTail;
    CHECK(level > 960 && level < 3500);
    CHECK(!txPriming);
    {
        // The output is a clean 1 kHz tone: no step anywhere
        int maxStep = 0;
        for (size_t i = 3000; i < out.size(); i++) {
            int step = abs(out[i] - out[i - 1]);
            if (step > maxStep)
                maxStep = step;
        }
        CHECK(maxStep < 1400); // 10000 * 2 pi / 48 = 1309 at most
    }

    // Transmit volume applies to what the radio reads
    setupReq(0x21, 0x01, 0x0200, (AC_FU_TX << 8) | 4, 2);
    pushRx(0, PKT_OUT_DATA, minus6, 2);
    outComplete(0);
    usbAudio_flushTx();
    for (int i = 0; i < 48; i++)
        frame48[i] = 20000;
    pushAudioOut(frame48, 48);
    CHECK(usbAudio_read(rd, 48) == 48 && rd[0] > 10000 && rd[0] < 10060);

    // A packet that does not fit is dropped whole
    usbAudio_flushTx();
    for (int f = 0; f < (TX_RING / 48) + 3; f++)
        pushAudioOut(frame48, 48);
    CHECK(usbAudio_available() <= TX_RING && txOverruns > 0);
    CHECK((usbAudio_available() % 48) == 0);

    // Transmit request: held while the link is up, dropped on suspend and
    // on reset, refused while suspended
    usbAudio_setPtt(true);
    CHECK(usbAudio_ptt());
    linkDown();
    CHECK(!usbAudio_ptt() && !usbcat_connected());
    usbAudio_setPtt(true);
    CHECK(!usbAudio_ptt());
    suspended = false;
    usbAudio_setPtt(true);
    CHECK(usbAudio_ptt());

    // Back to alternate 0: endpoints off, SOF interrupt off
    setupReq(0x01, 0x0B, 0, 6, 0);
    setupReq(0x01, 0x0B, 0, 5, 0);
    CHECK(!usbAudio_rxStreaming() && !usbAudio_txStreaming());
    CHECK(INEP(AUDIO_EP)->DIEPCTL == 0 && OUTEP(AUDIO_EP)->DOEPCTL == 0);
    CHECK(!(CORE->GINTMSK & USB_OTG_GINTMSK_SOFM));

    // Suspend stops output without losing it; resume restarts it
    suspended = true;
    CHECK(!usbcdc_connected());
    CHECK(usb_cdc_chardev.api->write(&usb_cdc_chardev, msg, 5) == -ENOTCONN);
    CHECK(usb_cat_chardev.api->write(&usb_cat_chardev, msg, 5) == -ENOTCONN);
    suspended = false;

    // DTR drop: log port closed, output held; the CAT port is unaffected
    setupReq(0x21, 0x22, 0x0000, 0, 0);
    CHECK(!usbcdc_connected() && usbcat_connected());

    // Bus reset: back to the default state, the transmit request dropped
    setupReq(0x21, 0x22, 0x0001, 0, 0);
    setupReq(0x01, 0x0B, 1, 6, 0);
    usbAudio_setPtt(true);
    CHECK(usbcdc_connected() && usbAudio_ptt());
    busReset();
    CHECK(!configured && !logPort.dtr && !usbcdc_connected());
    CHECK(!usbAudio_ptt() && !usbAudio_rxStreaming());
    CHECK((DEV->DCFG & USB_OTG_DCFG_DAD) == 0);
    CHECK(DEV->DAINTMSK == ((1u << 0) | (1u << 16)));

    printf("usb_cdc_test: %u checks passed\n", checks);
    return 0;
}
