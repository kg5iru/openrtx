/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/*
 * DMR slot state machine for CS7000-PLUS.
 *
 * STATUS: untested skeleton, RX-only. This is a first attempt at driving
 * the HR_C6000's Layer-2 DMR mode following the "Passive reception"
 * sequence documented in the HR_C6000 user manual, section 5.4.5. It has
 * never been run against real hardware or a real DMR signal -- treat every
 * register value and every design choice below as a hypothesis to verify
 * with a scope/logic analyzer, not a known-good implementation.
 *
 * Interrupt pins (checked against the schematic net names and the stock V9.00.96
 * firmware, which configures GPIOC pins 0, 2 and 3 as falling-edge interrupts
 * at 0x080887BA): TIME_SLOT_INTER = PC0 (EXTI0), RF_TX_INTER = PC2 (EXTI2),
 * SYS_INTER = PC3 (EXTI3).  PC1 is not an HR_C6000 interrupt.
 *
 * KNOWN SHORTCUTS taken here, to be fixed before this is more than a
 * bring-up aid:
 *
 *   1. All SPI register access happens directly inside the EXTI ISRs. The
 *      HR_C6000 interrupts are 3 clock cycles wide (~300ns) low pulses (see
 *      manual 5.1), and U_SPI accesses take a lot longer than that, so this
 *      relies on the flag simply being latched in EXTI->PR1 rather than the
 *      pin still being low when we get around to servicing it -- but doing
 *      blocking SPI transactions at interrupt priority is still not great
 *      practice, especially once TX and the vocoder link are added on top.
 *      Consider moving the actual register decisions to a lower-priority
 *      task woken from the ISR instead.
 *   2. No audio path yet: this only brings up frame-level RX synchronisation.
 *      There is no hardware AMBE chip on this board (see the note in
 *      dmrMode()); the software codec glue is in drivers/vocoder (built with
 *      -Dcs7000_ambe=true), but the V_SPI/I2S plumbing around it is not done.
 *   3. Colour code, group/source address matching, TX, and CACH/relay mode
 *      are all out of scope for this first pass.
 */

#include "interfaces/platform.h"
#include "peripherals/gpio.h"
#include "hwconfig.h"
#include "drivers/baseband/HR_C6000.h"
#include "drivers/baseband/dmr_CS7000.h"
#include "core/log.h"
#include "peripherals/spi.h"
#include "drivers/SPI/spi_stm32.h"

/*
 * Voice payload port (V_SPI).  The stock firmware reads the 27 voice bytes
 * of a burst through SPI1 (PB3 SCK, PB4 MISO, PB5 MOSI, PB6 CS; mode 0/CPHA=1,
 * MSB first) with the command bytes 0x83, 0x00 (0x08085C62), not from the
 * U_SPI RX RAM windows.  Clock rate is a conservative guess (stock MBR=6).
 */
static void vspiInit()
{
    gpio_setMode(V_SPI_SCK, ALTERNATE | ALTERNATE_FUNC(5));
    gpio_setMode(V_SPI_MISO, ALTERNATE | ALTERNATE_FUNC(5));
    gpio_setMode(V_SPI_MOSI, ALTERNATE | ALTERNATE_FUNC(5));
    gpio_setMode(V_SPI_CS, OUTPUT);
    gpio_setPin(V_SPI_CS);
    spiStm32_init(&v_spi, 1500000, SPI_FLAG_CPHA);
}

static void vspiReadVoice(uint8_t out[27])
{
    uint8_t tx[29] = { 0x83, 0x00 };
    uint8_t rx[29];
    gpio_clearPin(V_SPI_CS);
    spi_transfer(&v_spi, tx, rx, sizeof(tx));
    gpio_setPin(V_SPI_CS);
    for (int i = 0; i < 27; i++)
        out[i] = rx[2 + i];
}

// Local, hardcoded matches for first bring-up. TODO: pull from rtxStatus_t.
static constexpr uint8_t LOCAL_COLOUR_CODE = 0x01;

enum class DmrRxState {
    LISTENING,       // Waiting for a matching voice/data header (sys_inter)
    VOICE_HEADER_OK, // Header matched, waiting for next slot boundary
    VOICE_RX         // Steady-state: toggling RX on/off every other slot
};

static volatile DmrRxState rxState = DmrRxState::LISTENING;
// Voice bursts waiting for the audio thread: a small FIFO so that a burst
// arriving while the previous one is still being played is not lost.
static constexpr uint8_t VOICE_FIFO = 8;
static uint8_t voiceFifo[VOICE_FIFO][27];
static volatile uint8_t vHead = 0; // written by the ISR
static volatile uint8_t vTail = 0; // written by the audio thread
static volatile bool callActive = false;
static dmrCallInfo_t callInfo;
static volatile dmrDebug_t dbg;

/*
 * Name of interrupt handlers is mangled for C++ compatibility, following
 * the convention already used in chSelector_UV3x0.c.
 */

// DMR_TS_INT / TIME_SLOT_INTER (GPIOC,0 -> EXTI0): fires every slot (~30ms)
// once the chip has established (or is tracking) a time axis.
//
// The stock firmware only runs its state machine here.  The receive part of
// that (0x08075698) counts slots in which 0x5F[1:0] reports no sync, and
// after 13 of them re-arms the receiver:
//   0x41=0; 0x40=0x03; 0x41=0x20; 0x41=0x40; 0x40=0x43; 0x5F=0x30
static void endCall();

static uint8_t slotsSinceVoice = 0;

/*
 * Voice capture.
 *
 * v28's capture (call.log, 2026-11-01) settled what was wrong with the audio:
 * decoded offline, every burst read from the voice port was either clean (all
 * three AMBE+2 frames valid Golay codewords, no bit errors at all) or pure
 * noise (all three random), almost never in between.  So the demodulator was
 * fine; the bad half of the bursts were reads of whatever else the port held
 * (the hotspot's idle slot, mostly), taken because register 0x51 described a
 * voice burst when there was none.  0x51 does not track the bursts reliably
 * here.  Both V_SPI clock modes gave the same split, so the mode was not it
 * either.
 *
 * So the port is now read at every chip event, and the payload itself decides:
 * a burst is voice when the first Golay(24,12) word of each of its three frames
 * is a codeword with at most one bit flipped.  Random data passes that for one
 * frame 1.2% of the time, for all three about two in a million.  The silence
 * frame (B9 E8 81 52 61 73 00 2A 6B) passes, and so does all of the clean audio
 * in the capture.
 */

// Positions, in the 72 on-air bits of an AMBE+2 frame (MSB of byte 0 first),
// of the 24 bits of its first Golay word, as in MMDVMHost (AMBEFEC.cpp).
static const uint8_t golayA[24] = {
    0,  4,  8,  12, 16, 20, 24, 28, 32, 36, 40, 44,
    48, 52, 56, 60, 64, 68, 1,  5,  9,  13, 17, 21
};

static uint16_t golaySingle[23]; // syndromes of the 23 single-bit errors

static uint16_t golaySyndrome(uint32_t v) // v: 23-bit Golay(23,12) word
{
    for (int i = 22; i >= 11; i--)
        if ((v >> i) & 1)
            v ^= 0xC75u << (i - 11);
    return (uint16_t)v;
}

static void golayInit()
{
    for (int i = 0; i < 23; i++)
        golaySingle[i] = golaySyndrome(1u << i);
}

static bool frameIsVoice(const uint8_t *f)
{
    uint32_t a = 0;
    for (int i = 0; i < 24; i++) {
        uint8_t pos = golayA[i];
        a = (a << 1) | ((f[pos >> 3] >> (7 - (pos & 7))) & 1);
    }
    uint16_t syn = golaySyndrome(a >> 1); // drop the parity bit
    if (syn == 0)
        return true;
    for (int i = 0; i < 23; i++)
        if (syn == golaySingle[i])
            return true;
    return false;
}

static uint8_t lastVoice[27];
static bool haveLast = false;
static uint32_t lastVoiceSlot = 0;
static uint8_t repeatRun = 0;

// Read the voice port and queue its burst if it is a new voice burst.
// 'ev' says which interrupt this is (0 = slot, 1 = system), 'r51' and 'r82'
// are the status registers read at that event (0 for a slot event), logged
// alongside to see which events carry the bursts.
static void pollVoicePort(uint8_t ev, uint8_t r82, uint8_t r51)
{
    uint8_t buf[27];
    vspiReadVoice(buf);
    dbg.vReads++;

    int good = 0;
    for (int i = 0; i < 3; i++)
        if (frameIsVoice(&buf[9 * i]))
            good++;
    if (good == 0) {
        dbg.vGarbage++;
        return;
    }
    if (good < 3) {
        dbg.vPartial++;
        LOG_D("dmr", "voice port: %d of 3 frames valid, burst dropped", good);
        return;
    }

    bool same = haveLast;
    for (int i = 0; same && (i < 27); i++)
        if (buf[i] != lastVoice[i])
            same = false;

    if (same) {
        // The port holds a burst until something else is received, so the
        // same bytes are read again at later events.  A repeat is only taken
        // as a new burst (a run of identical silence frames) when at least
        // three slots have passed, a voice burst coming every two, never to
        // open a call, and for at most about three seconds.
        if (!callActive || ((dbg.irqSlot - lastVoiceSlot) < 3) ||
            (repeatRun >= 34)) {
            dbg.vDup++;
            return;
        }
        repeatRun++;
        dbg.vRepeat++;
    } else {
        repeatRun = 0;
    }

    for (int i = 0; i < 27; i++)
        lastVoice[i] = buf[i];
    haveLast = true;
    lastVoiceSlot = dbg.irqSlot;

    uint8_t nh = (vHead + 1) % VOICE_FIFO;
    if (nh == vTail) {
        dbg.vDropped++; // full: drop the oldest
        LOG_W("dmr", "voice FIFO full, oldest burst dropped");
        vTail = (vTail + 1) % VOICE_FIFO;
    }
    for (int i = 0; i < 27; i++)
        voiceFifo[vHead][i] = buf[i];
    vHead = nh;
    dbg.vAccepted++;

    LOG_D("dmr", "voice ok %s slot %lu 82=%02x 51=%02x%s",
          ev ? "sys" : "slot", (unsigned long) dbg.irqSlot, r82, r51,
          same ? " (repeat)" : "");
    LOG_HEX(LOG_LVL_DEBUG, "dmr", "voice", buf, 27);

    // Late entry: voice without a header seen (header missed, or the call
    // was ended early by the timeout) still opens the call, so its audio is
    // played; the IDs stay those of the last header.
    if (!callActive) {
        callActive = true;
        LOG_I("dmr", "call start: late entry, no header seen");
    }
    if (rxState != DmrRxState::VOICE_RX)
        C6000.writeCfgRegister(0x22, 0x26); // stock: voice path start
    rxState = DmrRxState::VOICE_RX;
    slotsSinceVoice = 0;
}

static void handleSlot()
{
    // A receive request covers a single slot: the chip receives that slot and
    // then falls idle unless a fresh request is queued before the next
    // boundary (OpenGD77 and the MD-UV3x0 DMR work both re-arm it every
    // slot).  0x50 = receive with the layer 2 access bit set.  Not doing this
    // left the receiver deaf after the first slot.
    //
    // The stock "13 slots without sync, re-arm everything" routine is not
    // used: run on an idle channel it restarts the receiver about every
    // 0.4 s, and it rewrote the sync enable register (0x5F) to 0x30.
    C6000.writeCfgRegister(0x41, 0x50);

    pollVoicePort(0, 0, 0);

    // End a call whose terminator was missed: no voice for 20 slots (600 ms).
    if (callActive && (++slotsSinceVoice >= 20)) {
        LOG_I("dmr", "call end: no voice for 20 slots");
        endCall();
    }
}

// DMR_SYS_INT / SYS_INTER (GPIOC,3 -> EXTI3): "something happened, go read
// the status registers to find out what."
//
// Layout taken from the stock handler (HOST V9.00.96, 0x08038E58) and the
// frame dispatcher it calls on event flag 0x08 (0x08039410):
//   0x82 = event flags, write-1-to-ack through 0x83.  Bit 3 = a burst has
//          been received.
//   0x5F = sync status, [1:0] != 0 means a sync pattern was found
//   0x51 = [1:0] burst class (1 = voice, 2 = data), bit 2 = invalid,
//          [7:4] = data type for data bursts (1 = voice LC header,
//          2 = terminator with LC, ...), [6:4] = voice frame number A..F
//   0x52 = [7:4] received colour code
// Link control is read from RX RAM address 0x00, 9 bytes: FLCO, FID,
// service options, dst[3], src[3] (24 bit big-endian).
// The stock firmware always acknowledges, even for events it ignores.
static uint32_t be24(const uint8_t *p)
{
    return (uint32_t(p[0]) << 16) | (uint32_t(p[1]) << 8) | p[2];
}

/*
 * A/B test, one call at a time.  v24 switched register sets from the
 * interrupt, and the first switch (with its module reset) left the receiver
 * deaf, so set A was the only one ever measured: 60 of 90 frames bad.  This
 * build keeps the registers as dmrMode() sets them and instead steps the
 * receive frequency, from the OpMode thread, by the offsets below (Hz): FM
 * shrugs off a few hundred Hz of error, 4FSK does not.  v25 (0, +-400,
 * +-900) found fewer bad frames the lower the frequency, best at -900 Hz.
 */
static const int32_t rxOffsets[DMR_VARIANTS] = { 0, 0, 0, 0, 0 };

static volatile uint8_t variant = 0;

int32_t dmr_variantOffset(uint8_t v)
{
    return rxOffsets[v % DMR_VARIANTS];
}

uint8_t dmr_variant()
{
    return variant;
}

static void endCall()
{
    // The voice FIFO is left to drain: a terminator ends the call after its
    // last voice burst, which is still waiting to be played.
    callActive = false;
    rxState = DmrRxState::LISTENING;
}

static void handleSys()
{
    uint8_t flags = C6000.readCfgRegister(0x82);
    // Flags first: reading the other status registers consumes the master
    // flag that announced them (order as in the stock firmware).
    uint8_t reg52 = C6000.readCfgRegister(0x52);
    uint8_t reg51 = C6000.readCfgRegister(0x51);
    uint8_t sync = C6000.readCfgRegister(0x5F);

    uint8_t rxCc = (reg52 >> 4) & 0x0F;
    // v28 logged this for every event (about 200 a second); now only for
    // the events flagged "data received" (0x82 bit 3).
    if (flags & 0x08)
        LOG_D("dmr", "sys 82=%02x 51=%02x 52=%02x 5f=%02x", flags, reg51,
              reg52, sync);
    dbg.flagsLast = flags;
    dbg.flagsOr |= flags;
    dbg.sync = sync;
    dbg.reg51 = reg51;
    dbg.cc = rxCc;
    uint8_t cls = reg51 & 0x03;
    // Bit 2 set = the chip's check failed.  v21 kept voice bursts with it set
    // (they are about half of all voice-class events): the codec's FEC errors
    // per frame doubled, so those are not good voice and are dropped again.
    bool valid = ((reg51 & 0x04) == 0) && (cls == 1 || cls == 2);
    if ((cls == 1) && (reg51 & 0x04)) {
        dbg.vFlagged++;
        LOG_D("dmr", "voice burst flagged invalid, dropped");
    }
    if (valid)
        dbg.bursts++;

    // Every system interrupt is treated as a frame event and judged by the
    // burst status alone, as the MD-UV3x0 DMR work does.  Gating on flag 0x08
    // (what the stock firmware's dispatcher keys on) never opened here: only
    // 0x02 and 0x04 were ever seen in 0x82.
    if (valid) {
        // No software colour code filter: the chip gates on register 0x1F, and
        // only repeaters (CACH) ever report a received code at all.
        dbg.v51 = reg51;
        if (cls == 2) {
            uint8_t type = reg51 >> 4;
            uint8_t lc[9];
            C6000.readRxBurst(0x00, lc, sizeof(lc));
            dbg.nData++;
            dbg.lastType = type;
            for (int i = 0; i < 9; i++)
                dbg.lc[i] = lc[i];
            LOG_HEX(LOG_LVL_DEBUG, "dmr", "data lc", lc, sizeof(lc));
            if (type == 1) // voice LC header
            {
                callInfo.flco = lc[0] & 0x3F;
                callInfo.dst = be24(&lc[3]);
                callInfo.src = be24(&lc[6]);
                // TODO: filter on configured contact / talkgroup.
                callActive = true;
                slotsSinceVoice = 0;
                dbg.headers++;
                rxState = DmrRxState::VOICE_HEADER_OK;
                LOG_I("dmr", "call start: flco %u src %lu dst %lu",
                      callInfo.flco, callInfo.src, callInfo.dst);
            } else if (type == 2) // terminator with LC
            {
                LOG_I("dmr", "call end: terminator");
                endCall();
            }
        } else {
            dbg.nVoice++;
        }
    }

    C6000.writeCfgRegister(0x83, flags);

    // Voice: judged by the payload, at every event (see pollVoicePort).
    pollVoicePort(1, flags, reg51);
}

// DMR_TX_INT / RF_TX_INTER (GPIOC,2 -> EXTI2): fires ahead of a TX slot
// boundary so the MCU can key the RF chain in time. Unused in this RX-only
// pass -- stubbed so the interrupt is at least acknowledged and doesn't
// hang the NVIC if it fires unexpectedly.
/*
 * Interrupt handling.  The status registers must be read at the instant of
 * the event: v19 did it from a worker thread and the readings already
 * described other bursts (no voice was ever queued).  The U_SPI bus is
 * protected by a pthread mutex, which an interrupt cannot take, so on this
 * target ScopedChipSelect skips the mutex in interrupt context and every
 * thread-side transaction masks the DMR interrupt lines instead (see
 * HR_Cx000.cpp).
 */
static volatile bool dmrRunning = false;
extern "C" void dmrSlotIrqImpl()
{
    EXTI->PR1 = EXTI_PR1_PR0;
    dbg.irqSlot++;
    if (dmrRunning)
        handleSlot();
}

extern "C" void dmrSysIrqImpl()
{
    EXTI->PR1 = EXTI_PR1_PR3;
    dbg.irqSys++;
    if (dmrRunning)
        handleSys();
}

void EXTI0_IRQHandler()
{
    dmrSlotIrqImpl();
}

void EXTI3_IRQHandler()
{
    dmrSysIrqImpl();
}

void EXTI2_IRQHandler()
{
    if ((EXTI->PR1 & EXTI_PR1_PR2) == 0)
        return;

    EXTI->PR1 = EXTI_PR1_PR2;

    // TODO: TX slot handling goes here once RX bring-up is solid.
}

void dmr_init()
{
    variant = 0;
    golayInit();
    haveLast = false;
    repeatRun = 0;
    dmrRunning = true;

    gpio_setMode(DMR_TS_INT, INPUT);
    gpio_setMode(DMR_SYS_INT, INPUT);
    gpio_setMode(DMR_TX_INT, INPUT);

    rxState = DmrRxState::LISTENING;
    vspiInit();

    /*
     * HR_C6000 interrupts are active-low pulses (manual, section 5.1), so
     * we trigger on the falling edge.
     *
     * NOTE: STM32H7's EXTI block is laid out very differently from the F4
     * used on MD-UV3x0 (IMR1/PR1/RTSR1/FTSR1 instead of plain IMR/PR/RTSR/
     * FTSR, plus a whole separate D3-domain pending/mask setup this driver
     * doesn't touch). There is no existing EXTI usage anywhere else in the
     * CS7000-PLUS/STM32H7xx tree to confirm this pattern against -- this is
     * the first EXTI consumer on this MCU port, so double check against the
     * reference manual (RM0433), not just by analogy with the F4 code.
     */
    EXTI->IMR1 |= EXTI_IMR1_IM0 | EXTI_IMR1_IM2 | EXTI_IMR1_IM3;
    EXTI->FTSR1 |= EXTI_FTSR1_TR0 | EXTI_FTSR1_TR2 | EXTI_FTSR1_TR3;
    EXTI->RTSR1 &= ~(EXTI_RTSR1_TR0 | EXTI_RTSR1_TR2 | EXTI_RTSR1_TR3);

    // SYSCFG sits on the APB4 bus of the H7 and its clock is off after reset;
    // without it the EXTI line routing below is silently ignored.
    RCC->APB4ENR |= RCC_APB4ENR_SYSCFGEN;
    (void)RCC->APB4ENR;

    // DMR_TS_INT/DMR_TX_INT/DMR_SYS_INT are on GPIOC pins 0, 2 and 3
    // (see pinmap.h) -> SYSCFG_EXTICR1, "_PC" selectors.
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PC | SYSCFG_EXTICR1_EXTI2_PC
                       | SYSCFG_EXTICR1_EXTI3_PC;

    NVIC_ClearPendingIRQ(EXTI0_IRQn);
    NVIC_ClearPendingIRQ(EXTI3_IRQn);
    NVIC_ClearPendingIRQ(EXTI2_IRQn);

    // TODO: priorities picked arbitrarily (higher urgency for the 30ms slot
    // tick than for the status-decode interrupt) -- revisit once there's a
    // scheduler-level view of what else is competing for CPU time during
    // an active call.
    // Same priority for all three so that one handler never interrupts
    // another in the middle of a U_SPI transaction.
    NVIC_SetPriority(EXTI0_IRQn, 11);
    NVIC_SetPriority(EXTI3_IRQn, 11);
    NVIC_SetPriority(EXTI2_IRQn, 11);

    NVIC_EnableIRQ(EXTI0_IRQn);
    NVIC_EnableIRQ(EXTI3_IRQn);
    NVIC_EnableIRQ(EXTI2_IRQn);
}

void dmr_terminate()
{
    dmrRunning = false;
    NVIC_DisableIRQ(EXTI0_IRQn);
    NVIC_DisableIRQ(EXTI3_IRQn);
    NVIC_DisableIRQ(EXTI2_IRQn);

    EXTI->IMR1 &= ~(EXTI_IMR1_IM0 | EXTI_IMR1_IM2 | EXTI_IMR1_IM3);
}

// Fetch the last received voice burst: 3 AMBE+2 frames of 9 bytes each, to be
// fed one at a time to ambe_cs7000_decode().  Returns false if none pending.
bool dmr_popVoiceBurst(uint8_t out[27])
{
    if (vTail == vHead)
        return false;
    uint8_t t = vTail;
    for (int i = 0; i < 27; i++)
        out[i] = voiceFifo[t][i];
    vTail = (t + 1) % VOICE_FIFO;
    return true;
}

unsigned dmr_voiceQueued()
{
    return (unsigned)((vHead + VOICE_FIFO - vTail) % VOICE_FIFO);
}

bool dmr_getCallInfo(dmrCallInfo_t *out)
{
    if (!callActive)
        return false;
    *out = callInfo;
    return true;
}

void dmr_getDebug(dmrDebug_t *out)
{
    out->irqSys = dbg.irqSys;
    out->irqSlot = dbg.irqSlot;
    out->bursts = dbg.bursts;
    out->headers = dbg.headers;
    out->sync = dbg.sync;
    out->reg51 = dbg.reg51;
    out->cc = dbg.cc;
    out->r40 = dbg.r40;
    out->r41 = dbg.r41;
    out->r81 = dbg.r81;
    out->r5f = dbg.r5f;
    out->r1f = dbg.r1f;
    out->r21 = dbg.r21;
    out->pins = dbg.pins;
    out->vDropped = dbg.vDropped;
    out->vFlagged = dbg.vFlagged;
    out->vDup = dbg.vDup;
    out->vMismatch = dbg.vMismatch;
    out->nVoice = dbg.nVoice;
    out->nData = dbg.nData;
    out->v51 = dbg.v51;
    out->lastType = dbg.lastType;
    for (int i = 0; i < 9; i++)
        out->lc[i] = dbg.lc[i];
    out->flagsOr = dbg.flagsOr;
    out->flagsLast = dbg.flagsLast;
    out->polls = dbg.polls;
    out->vReads = dbg.vReads;
    out->vAccepted = dbg.vAccepted;
    out->vGarbage = dbg.vGarbage;
    out->vPartial = dbg.vPartial;
    out->vRepeat = dbg.vRepeat;
}

void dmr_pollDiag()
{
    // The chip registers are reached through a bus without locking, so the
    // interrupt handlers must not run in the middle of a thread transaction.
    NVIC_DisableIRQ(EXTI0_IRQn);
    NVIC_DisableIRQ(EXTI2_IRQn);
    NVIC_DisableIRQ(EXTI3_IRQn);
    dbg.r40 = C6000.readCfgRegister(0x40);
    dbg.r41 = C6000.readCfgRegister(0x41);
    dbg.r81 = C6000.readCfgRegister(0x81);
    dbg.r5f = C6000.readCfgRegister(0x5F);
    dbg.r1f = C6000.readCfgRegister(0x1F);
    dbg.r21 = C6000.readCfgRegister(0x21);
    dbg.pins = (gpio_readPin(DMR_TS_INT) ? 1 : 0)
             | (gpio_readPin(DMR_TX_INT) ? 2 : 0)
             | (gpio_readPin(DMR_SYS_INT) ? 4 : 0);
    dbg.polls++;

    NVIC_EnableIRQ(EXTI0_IRQn);
    NVIC_EnableIRQ(EXTI2_IRQn);
    NVIC_EnableIRQ(EXTI3_IRQn);
}
