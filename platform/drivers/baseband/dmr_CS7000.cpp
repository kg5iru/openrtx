/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/*
 * DMR for the CS7000-PLUS: HR_C6000 interrupt handling, voice burst capture
 * and call tracking, and (bring-up, from v40) transmission of a voice call in
 * direct mode.
 *
 * Interrupt pins (schematic net names, and the stock V9.00.96 firmware, which
 * configures GPIOC pins 0, 2 and 3 as falling-edge interrupts at 0x080887BA):
 * TIME_SLOT_INTER = PC0 (EXTI0), RF_TX_INTER = PC2 (EXTI2), SYS_INTER = PC3
 * (EXTI3).  PC1 is not an HR_C6000 interrupt.
 *
 * The chip's status registers are only valid at the instant of the event, so
 * they are read inside the interrupt handlers.  U_SPI is shared with thread
 * code (audio FIFO, register setup): on this target ScopedChipSelect skips its
 * mutex in interrupt context and every thread-side transaction masks the DMR
 * interrupt lines instead (see HR_Cx000.cpp).
 *
 * Voice payload: three AMBE+2 frames of 9 bytes per burst, read through the
 * separate voice port (V_SPI, SPI1) as the stock firmware does.  Whether a
 * read holds a voice burst is decided from the payload itself (see
 * frameIsVoice), because register 0x51 does not reliably describe the burst at
 * the moment it is read (see the 2026-11-01 analysis in the project notes).
 */

#include "interfaces/platform.h"
#include "peripherals/gpio.h"
#include "hwconfig.h"
#include "drivers/baseband/HR_C6000.h"
#include "drivers/baseband/dmr_CS7000.h"
#include "drivers/baseband/dmr_audio_CS7000.h"
#include "drivers/baseband/dmr_diag_CS7000.h"
#include "core/log.h"
#include "peripherals/spi.h"
#include "drivers/SPI/spi_stm32.h"

/*
 * Voice payload port (V_SPI).  The stock firmware reads the 27 voice bytes of
 * a burst through SPI1 (PB3 SCK, PB4 MISO, PB5 MOSI, PB6 CS; CPOL 0, CPHA 1,
 * MSB first) with the command bytes 0x83, 0x00 (0x08085C62).
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

/*
 * AMBE+2 frame check.  A DMR voice frame (72 bits, on-air order) protects its
 * most significant bits with a Golay(24,12) word A and a Golay(23,12) word B,
 * B being scrambled with a sequence seeded by A's data bits (ETSI TS 102 361-1
 * and MMDVMHost AMBEFEC.cpp).  A frame is taken as voice when both words are
 * codewords with at most one bit error each.  The bursts read from the voice
 * port are either clean or random (the port also holds the chip's other
 * bursts, such as a hotspot's idle slot); random data passes for one frame
 * about once in 7000 tries, for the three frames of a burst practically never.
 * An all-zero read (no signal) fails on B.
 */
static const uint8_t golayA[24] = { 0,  4,  8,  12, 16, 20, 24, 28,
                                    32, 36, 40, 44, 48, 52, 56, 60,
                                    64, 68, 1,  5,  9,  13, 17, 21 };

static const uint8_t golayB[23] = { 25, 29, 33, 37, 41, 45, 49, 53,
                                    57, 61, 65, 69, 2,  6,  10, 14,
                                    18, 22, 26, 30, 34, 38, 42 };

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

// Correct at most one bit error in a Golay(23,12) word; false if more.
static bool golayCorrect(uint32_t *v)
{
    uint16_t syn = golaySyndrome(*v);
    if (syn == 0)
        return true;
    for (int i = 0; i < 23; i++) {
        if (syn == golaySingle[i]) {
            *v ^= 1u << i;
            return true;
        }
    }
    return false;
}

static uint32_t gatherBits(const uint8_t *f, const uint8_t *pos, int n)
{
    uint32_t v = 0;
    for (int i = 0; i < n; i++)
        v = (v << 1) | ((f[pos[i] >> 3] >> (7 - (pos[i] & 7))) & 1);
    return v;
}

static bool frameIsVoice(const uint8_t *f)
{
    uint32_t a = gatherBits(f, golayA, 24) >> 1; // parity bit not checked
    if (!golayCorrect(&a))
        return false;

    // Scrambling sequence for B, seeded by A's 12 data bits.
    uint32_t p = 16 * (a >> 11);
    uint32_t prn = 0;
    for (int i = 0; i < 23; i++) {
        p = (173 * p + 13849) & 0xFFFF;
        prn = (prn << 1) | (p >> 15);
    }

    uint32_t b = gatherBits(f, golayB, 23) ^ prn;
    return golayCorrect(&b);
}

static dmrCallInfo_t callInfo;
static volatile bool callActive = false;
static volatile bool dmrRunning = false;
static uint8_t slotsSinceVoice = 0;

static bool termSeen = false; // terminator read since the last voice burst
static uint8_t lastVoice[27];
static bool haveLast = false;
static bool voicePathOn = false;
static uint32_t callBursts = 0;
static uint32_t callDropped = 0;

/*
 * Talkgroup filter.  A call is muted (not played, not shown) when it is a
 * group call to another talkgroup, or when a talkgroup is set and the call's
 * header was missed (late entry: its talkgroup is unknown).
 */
static volatile uint32_t rxTalkgroup = 0;
static volatile bool tgMuted = false;

/*
 * Timeslot filter.  With a timeslot set, radio_CS7000.cpp puts the chip in
 * the stock repeater mode (reg 0x10 = 0x6C / 0x6D).  The chip still receives
 * both slots, but in that mode reg 0x52 bit 2 carries the CACH TDMA channel
 * (TC) bit: 0 on TS1, 1 on TS2 (v37 log, 2026-11-04: 14 of 14 voice bursts
 * of 7 calls on known slots; in the mode dmrMode() sets, 0x6A, the bit is
 * always 0).  The bit was read after 0x50 and 0x51, at the first voice
 * bursts, so it is read the same way here.  A call's slot is taken from its
 * first voice burst and the call is muted when it is the other slot; the
 * later bursts are checked too and disagreements are counted.
 */
static volatile uint8_t rxTimeslot = 0;
static volatile bool slotMuted = false;
static uint8_t callSlot = 0;
static uint32_t slotDisagree = 0;
// Sync kind of the call (see burstSyncKind)
enum : uint8_t { SYNC_UNKNOWN = 0, SYNC_BS, SYNC_MS };
static uint8_t callSync = SYNC_UNKNOWN;
static uint8_t syncTries = 0; // voice bursts looked at for the sync kind
static uint8_t syncRaw[2][6]; // windows of the last burst looked at

static inline bool callMuted()
{
    return tgMuted || slotMuted;
}

// Registers read at the last system interrupt, logged with each call
static uint8_t last52 = 0;
static uint8_t last57 = 0;

static void endCall(const char *why)
{
    if (callActive) {
        LOG_I("dmr", "call end (%s): %lu bursts, %lu frames lost", why,
              (unsigned long)callBursts, (unsigned long)callDropped);
        if (slotDisagree != 0)
            LOG_I("dmr", "slot bit disagreed with TS%u on %lu bursts", callSlot,
                  (unsigned long)slotDisagree);
    }
    // The voice FIFO is left to drain: the last bursts of the call are still
    // waiting to be played.
    callActive = false;
    tgMuted = false;
    slotMuted = false;
    callSlot = 0;
    slotDisagree = 0;
    callSync = SYNC_UNKNOWN;
    syncTries = 0;
    voicePathOn = false;
    termSeen = false;
}

/*
 * Timeslot of the voice burst just received, 1 or 2, from the CACH TC bit
 * (reg 0x52 bit 2); 0 when the chip is not in the mode that decodes it.
 */
/*
 * Direct mode has no timeslots: a call from a radio in direct mode (MS
 * sourced sync) carries no CACH, and the TC bit read from reg 0x52 means
 * nothing (v46 on air: an HT in direct mode was taken as TS1 and muted with
 * TS2 set, and the bit disagreed with itself on a quarter of the bursts).
 * The kind of sync is read from the RX RAM burst windows, where voice frame
 * A has its sync word at the start (v36 dumps: 75 5F D7 DF 75 F7, the BS
 * voice sync, at 0x30 or 0x60).  The first byte of a RAM read is sometimes
 * stale, so bytes 1..5 are compared.
 */

static uint8_t burstSyncKind()
{
    static const uint8_t bsVoice[5] = { 0x5F, 0xD7, 0xDF, 0x75, 0xF7 };
    static const uint8_t msVoice[5] = { 0x7D, 0x5D, 0xD5, 0x7D, 0xFD };
    static const uint8_t windows[2] = { 0x30, 0x60 };

    for (int w = 0; w < 2; w++) {
        C6000.readRxBurst(windows[w], syncRaw[w], 6);
        bool bs = true, ms = true;
        for (int i = 0; i < 5; i++) {
            if (syncRaw[w][1 + i] != bsVoice[i])
                bs = false;
            if (syncRaw[w][1 + i] != msVoice[i])
                ms = false;
        }
        if (bs)
            return SYNC_BS;
        if (ms)
            return SYNC_MS;
    }
    return SYNC_UNKNOWN;
}

static uint8_t burstSlot()
{
    if (rxTimeslot == 0)
        return 0;

    (void)C6000.readCfgRegister(0x50);
    (void)C6000.readCfgRegister(0x51);
    uint8_t r52 = C6000.readCfgRegister(0x52);
    return ((r52 & 0x04) != 0) ? 2 : 1;
}

/*
 * Read the voice port and queue its burst if it is a new voice burst.  Called
 * at every chip event: the port keeps a burst until the chip receives
 * something else, so the same bytes are read several times and only the first
 * read is kept.
 */
static void pollVoicePort()
{
    uint8_t buf[27];
    vspiReadVoice(buf);

    for (int i = 0; i < 3; i++)
        if (!frameIsVoice(&buf[9 * i]))
            return;

    bool same = haveLast;
    for (int i = 0; same && (i < 27); i++)
        if (buf[i] != lastVoice[i])
            same = false;
    if (same)
        return;

    for (int i = 0; i < 27; i++)
        lastVoice[i] = buf[i];
    haveLast = true;

    // Late entry: voice without a header seen still opens the call, so its
    // audio is played; the IDs stay those of the last header.  With a
    // talkgroup filter set, its talkgroup is unknown: muted until a header.
    if (!callActive) {
        callActive = true;
        tgMuted = (rxTalkgroup != 0);
        callBursts = 0;
        callDropped = 0;
        LOG_I("dmr", "call start: late entry%s",
              tgMuted ? ", muted (talkgroup unknown)" : "");
    }

    /*
     * Timeslot filter.  The slot is taken from the TC bit of the first
     * voice burst, but applied only once the call is known to come through
     * a repeater (BS sync); until then (frame A carries the sync, so at
     * most 6 bursts) its voice is held back.  A direct mode call is never
     * muted for its slot.  If no sync was recognised in 7 bursts, the TC
     * bit decides as before and the windows are logged.
     */
    uint8_t slot = burstSlot();
    if ((slot != 0) && (callSlot == 0))
        callSlot = slot;
    else if ((slot != 0) && (slot != callSlot))
        slotDisagree++;

    bool slotPending = false;
    if ((rxTimeslot != 0) && (callSync == SYNC_UNKNOWN)) {
        callSync = burstSyncKind();
        syncTries++;
        if (callSync == SYNC_BS) {
            slotMuted = (callSlot != rxTimeslot);
            LOG_I("dmr", "call on TS%u%s", callSlot,
                  slotMuted ? " (muted)" : "");
        } else if (callSync == SYNC_MS) {
            slotMuted = false;
            callSlot = 0;
            LOG_I("dmr", "call in direct mode: no timeslot filter");
        } else if (syncTries >= 7) {
            callSync = SYNC_BS;
            slotMuted = (callSlot != rxTimeslot);
            LOG_W("dmr", "call sync not recognised, taken as TS%u%s", callSlot,
                  slotMuted ? " (muted)" : "");
            LOG_HEX(LOG_LVL_WARN, "dmr", "rx ram 0x30", syncRaw[0], 6);
            LOG_HEX(LOG_LVL_WARN, "dmr", "rx ram 0x60", syncRaw[1], 6);
        } else {
            slotPending = true;
        }
    }
    if (callSync == SYNC_MS)
        callSlot = 0;

    slotsSinceVoice = 0;
    termSeen = false;
    if (callMuted() || slotPending) {
        callBursts++;
        return;
    }

    // The three AMBE+2 frames go to the playback queue (dmr_audio_CS7000)
    for (int i = 0; i < 3; i++)
        if (!dmrAudio_pushFrame(&buf[9 * i]))
            callDropped++;
    callBursts++;

    if (!voicePathOn) {
        C6000.writeCfgRegister(0x22, 0x26); // stock: voice path start
        voicePathOn = true;
    }
}

/*
 * Transmit bring-up diagnostics (see dmr_diag_CS7000.h).
 */
static bool diagReady = false;

void dmrDiag_init()
{
    if (diagReady)
        return;

    RCC->APB4ENR |= RCC_APB4ENR_RTCAPBEN;
    (void)RCC->APB4ENR;
    PWR->CR1 |= PWR_CR1_DBP;
    while ((PWR->CR1 & PWR_CR1_DBP) == 0)
        ;

    if (dmrDiag_get(DIAG_MAGIC) == DIAG_MAGIC_VALUE) {
        LOG_W("dmr",
              "last transmission did not finish: stage %lu, %lu RTX "
              "updates",
              (unsigned long)dmrDiag_get(DIAG_STAGE),
              (unsigned long)dmrDiag_get(DIAG_BEAT));
        LOG_W("dmr",
              "  slot irq %lu, sys irq %lu, tx irq %lu, in irq %lu, "
              "sys flags %02lx",
              (unsigned long)dmrDiag_get(DIAG_SLOT_IRQ),
              (unsigned long)dmrDiag_get(DIAG_SYS_IRQ),
              (unsigned long)dmrDiag_get(DIAG_TX_IRQ),
              (unsigned long)dmrDiag_get(DIAG_IN_IRQ),
              (unsigned long)dmrDiag_get(DIAG_SYS_FLAGS));
        uint32_t enc = dmrDiag_get(DIAG_ENC);
        LOG_W("dmr", "  encoder: %lu frames%s",
              (unsigned long)(enc & 0x7FFFFFFF),
              ((enc & 0x80000000u) != 0) ? ", stopped inside the codec" : "");
    }

    // Check that the registers hold a value (the backup domain is powered)
    dmrDiag_set(DIAG_MAGIC, 0x12345678);
    bool ok = (dmrDiag_get(DIAG_MAGIC) == 0x12345678);
    for (int r = 0; r < DIAG_COUNT; r++)
        dmrDiag_set((enum dmrDiagReg)r, 0);
    if (!ok)
        LOG_W("dmr", "backup registers not writable: no TX breadcrumbs");

    diagReady = true;
}

void dmrDiag_txStart()
{
    for (int r = 1; r < DIAG_COUNT; r++)
        dmrDiag_set((enum dmrDiagReg)r, 0);
    dmrDiag_set(DIAG_STAGE, STAGE_PTT);
    dmrDiag_set(DIAG_MAGIC, DIAG_MAGIC_VALUE);
}

void dmrDiag_txEnd()
{
    dmrDiag_set(DIAG_MAGIC, 0);
    dmrDiag_set(DIAG_STAGE, STAGE_IDLE);
}

static bool watchdogOn = false;

void dmrDiag_startWatchdog()
{
    if (watchdogOn)
        return;

    // LSI (32 kHz) / 64 = 500 Hz; 2500 counts = 5 s
    IWDG1->KR = 0xCCCC; // start
    IWDG1->KR = 0x5555; // unlock PR and RLR
    IWDG1->PR = 4;
    IWDG1->RLR = 2500;
    while (IWDG1->SR != 0)
        ;
    IWDG1->KR = 0xAAAA;
    watchdogOn = true;
    LOG_I("dmr", "watchdog started (5 s)");
}

void dmrDiag_kickWatchdog()
{
    if (watchdogOn)
        IWDG1->KR = 0xAAAA;
}

/*
 * Transmit, bring-up (v40): a group voice call in direct mode, built from the
 * burst primitives of the stock slot state machine (state 4 at 0x08077474,
 * see the project notes):
 * - the handler runs at every slot interrupt and alternates: one slot arms a
 *   burst (reg 0x41 = 0x80, then the payload, then reg 0x50 = burst type),
 *   the next writes reg 0x41 = 0x00;
 * - voice LC header: 9 byte link control written at address 0, reg 0x50 =
 *   0x10 (data type 1); after the first one, reg 0x22 = 0x86.  The stock
 *   code sends several, from a counter; 3 here;
 * - voice burst: 27 bytes over V_SPI, the link control again (the chip
 *   builds the embedded signalling from it), reg 0x50 = (n << 4) | 8 with n
 *   the frame A..F;
 * - terminator with LC: reg 0x50 = 0x20; in the slot after it, reg 0x22 =
 *   0x46, 0x21 = 0x03, 0x41 = 0x40 (back to receive).
 * The power amplifier stays keyed for the whole transmission, as the stock
 * firmware does (OpMode_DMR keys it).
 */
enum : uint8_t { TX_OFF = 0, TX_HEADER, TX_VOICE, TX_END, TX_DONE };

static constexpr uint8_t TX_HEADERS = 3;

static const uint8_t ambeSilence[9] = { 0xB9, 0xE8, 0x81, 0x52, 0x61,
                                        0x73, 0x00, 0x2A, 0x6B };

static volatile uint8_t txState = TX_OFF;
static volatile bool txStopReq = false;
static uint8_t txLc[9];
static bool txArmSlot = false; // this slot interrupt arms a burst
// Reg 0x10 as last written by radio_CS7000.cpp, put back after a
// transmission: reading it back does not give the written value (v45 read
// 0x80 for 0x6A, 0xEC for 0x6D)
static uint8_t slotModeReg = 0x6A;
// Slot interrupts while receiving, since the last TX start: tells whether
// the chip's slot timer was running when the PTT was pressed
static volatile uint32_t rxSlotIrqs = 0;
static volatile uint32_t rxSlotIrqsTotal = 0; // never reset
static uint8_t txHeaders = 0;
static uint8_t txVoiceN = 0;
static uint32_t txBursts = 0;
static uint32_t txUnderruns = 0; // voice frames sent as silence: none encoded
// Bring-up diagnostics: interrupts seen while transmitting.  The slot
// interrupt comes every 30 ms during a received call but faster and
// irregularly while the receiver searches; the TX handler alternates on it.
static volatile uint32_t txSlotIrqs = 0;
static volatile uint32_t txTxIrqs = 0;
// The first two voice bursts sent, logged at the end of the transmission
static uint8_t txFirstBursts[2][27];

static void txSendTerminator()
{
    dmrDiag_set(DIAG_STAGE, STAGE_TERM);
    C6000.writeCfgRegister(0x41, 0x80);
    C6000.writeTxBurst(0x00, txLc, sizeof(txLc));
    C6000.writeCfgRegister(0x50, 0x20);
    txState = TX_END;
}

// Take the next 3 encoded frames (silence if none) and write them to V_SPI
static void loadVoice()
{
    uint8_t voice[27];
    for (int i = 0; i < 3; i++) {
        if (!dmrAudio_popTxFrame(&voice[9 * i])) {
            for (int k = 0; k < 9; k++)
                voice[(9 * i) + k] = ambeSilence[k];
            txUnderruns++;
        }
    }

    if (txBursts < 2) {
        for (int i = 0; i < 27; i++)
            txFirstBursts[txBursts][i] = voice[i];
    }

    /*
     * The burst's voice goes to TX RAM 0x30..0x4A, the place the HR_C6000
     * manual gives for voice frames A..F (table 5.2), written in the same
     * slot as 0x41 = 0x80 and reg 0x50 (the "t2" preparation time).  The
     * stock firmware does this on its privacy path, with reg 0x56 bit 4 set
     * (state 4, 0x08077824..0x08077850).  Over V_SPI (v40..v44) the chip
     * took only some of the writes, and sent each block it took for 1 to
     * 200 bursts in a row (off-air captures).
     */
    C6000.writeTxBurst(0x30, voice, sizeof(voice));
}

static void handleTxSlot()
{
    txArmSlot = !txArmSlot;

    if (!txArmSlot) {
        if (txState == TX_END) {
            // The slot after the terminator: the stock end of transmission
            dmrDiag_set(DIAG_STAGE, STAGE_END);
            C6000.writeCfgRegister(0x22, 0x46);
            C6000.writeCfgRegister(0x21, 0x03);
            C6000.writeCfgRegister(0x41, 0x40);
            txState = TX_DONE;
        } else {
            C6000.writeCfgRegister(0x41, 0x00);
        }
        return;
    }

    switch (txState) {
        case TX_HEADER:
            dmrDiag_set(DIAG_STAGE, STAGE_HEADER);
            C6000.writeCfgRegister(0x41, 0x80);
            C6000.writeTxBurst(0x00, txLc, sizeof(txLc));
            C6000.writeCfgRegister(0x50, 0x10);
            if (txHeaders == TX_HEADERS)
                C6000.writeCfgRegister(0x22, 0x86); // stock: after the header
            if (--txHeaders == 0) {
                txState = TX_VOICE;
                txVoiceN = 0;
            }
            break;

        case TX_VOICE: {
            if (txStopReq) {
                txSendTerminator();
                break;
            }

            dmrDiag_set(DIAG_STAGE, STAGE_VOICE);
            C6000.writeCfgRegister(0x41, 0x80);
            loadVoice();
            C6000.writeTxBurst(0x00, txLc, sizeof(txLc));
            C6000.writeCfgRegister(0x50, (uint8_t)((txVoiceN << 4) | 0x08));
            txVoiceN = (txVoiceN + 1) % 6;
            txBursts++;
            break;
        }

        default:
            break;
    }
}

// DMR_TS_INT / TIME_SLOT_INTER (PC0 -> EXTI0): every slot (30 ms).
static void handleSlot()
{
    if (txState != TX_OFF) {
        txSlotIrqs++;
        dmrDiag_inc(DIAG_SLOT_IRQ);
        dmrDiag_set(DIAG_IN_IRQ, 1);
        if (txState != TX_DONE)
            handleTxSlot();
        dmrDiag_set(DIAG_IN_IRQ, 0);
        return;
    }

    rxSlotIrqs++;
    rxSlotIrqsTotal++;

    // A receive request covers a single slot: the chip receives that slot and
    // then falls idle unless a fresh request is queued before the next
    // boundary.  0x50 = receive with the layer 2 access bit set.
    C6000.writeCfgRegister(0x41, 0x50);

    pollVoicePort();

    // End a call whose terminator was missed: no voice for 20 slots (600 ms).
    if (callActive && (++slotsSinceVoice >= 20))
        endCall(termSeen ? "terminator" : "no voice");
}

static uint32_t be24(const uint8_t *p)
{
    return (uint32_t(p[0]) << 16) | (uint32_t(p[1]) << 8) | p[2];
}

/*
 * Reed-Solomon (12,9) check of a full link control (ETSI TS 102 361-1 B.3.6,
 * as in MMDVMHost RS129.cpp): GF(256) with polynomial 0x11D, generator roots
 * alpha^1..alpha^3.  The three parity bytes are sent XORed with a mask that
 * tells a voice LC header (0x96) from a terminator with LC (0x99).
 */
static uint8_t gfExp[512];
static uint8_t gfLog[256];

static void gfInit()
{
    uint16_t x = 1;
    for (int i = 0; i < 255; i++) {
        gfExp[i] = (uint8_t)x;
        gfLog[x] = (uint8_t)i;
        x <<= 1;
        if (x & 0x100)
            x ^= 0x11D;
    }
    for (int i = 255; i < 512; i++)
        gfExp[i] = gfExp[i - 255];
}

static uint8_t gfMul(uint8_t a, uint8_t b)
{
    if ((a == 0) || (b == 0))
        return 0;
    return gfExp[gfLog[a] + gfLog[b]];
}

// Parity bytes in the order they follow the 9 data bytes.
static void rs129Parity(const uint8_t *data, uint8_t out[3])
{
    static const uint8_t poly[3] = { 64, 56, 14 };
    uint8_t par[3] = { 0, 0, 0 };
    for (int i = 0; i < 9; i++) {
        uint8_t d = data[i] ^ par[2];
        par[2] = par[1] ^ gfMul(poly[2], d);
        par[1] = par[0] ^ gfMul(poly[1], d);
        par[0] = gfMul(poly[0], d);
    }
    out[0] = par[2];
    out[1] = par[1];
    out[2] = par[0];
}

static bool rs129Match(const uint8_t *lc, const uint8_t par[3], uint8_t mask)
{
    return ((lc[9] ^ mask) == par[0]) && ((lc[10] ^ mask) == par[1])
        && ((lc[11] ^ mask) == par[2]);
}

/*
 * Link control.  v29 and v30 showed no caller at all: the header was read
 * only when 0x51 said "voice LC header", and 0x51 is as unreliable for data
 * bursts as it was for voice.  The LC image in RX RAM (12 bytes at 0x00) is
 * now read at every system interrupt instead and judged by its own
 * Reed-Solomon parity, with the header or the terminator mask.  The chip keeps
 * the received parity bytes there (v31: callers and talkgroups shown
 * correctly, every header that passed matched the call heard).
 */
static uint8_t lcImage[12];
static bool lcHave = false;

static void pollLinkControl(uint8_t r82, uint8_t r51)
{
    uint8_t lc[12];
    C6000.readRxBurst(0x00, lc, sizeof(lc));

    bool same = lcHave;
    for (int i = 0; same && (i < 12); i++)
        if (lc[i] != lcImage[i])
            same = false;
    if (same)
        return;
    for (int i = 0; i < 12; i++)
        lcImage[i] = lc[i];
    lcHave = true;

    uint8_t par[3];
    rs129Parity(lc, par);
    bool header = rs129Match(lc, par, 0x96);
    bool term = rs129Match(lc, par, 0x99);
    bool plain = rs129Match(lc, par, 0x00);

    // v31: the chip keeps the received parity, and most changes of the image
    // are other bursts (2710 failed the check against 98 that passed), so
    // only the ones that pass are logged.
    if (header || term || plain) {
        LOG_HEX(LOG_LVL_DEBUG, "dmr", "lc", lc, sizeof(lc));
        LOG_D("dmr", "lc rs %s 82=%02x 51=%02x",
              header ? "header" : (term ? "term" : "plain"), r82, r51);
    }

    uint8_t flco = lc[0] & 0x3F;
    if ((header || plain) && ((flco == 0) || (flco == 3))) {
        uint32_t dst = be24(&lc[3]);
        uint32_t src = be24(&lc[6]);
        bool changed = !callActive || (flco != callInfo.flco)
                    || (dst != callInfo.dst) || (src != callInfo.src);
        callInfo.flco = flco;
        callInfo.dst = dst;
        callInfo.src = src;
        // Private calls always pass: there is no own-ID setting yet.
        uint32_t tg = rxTalkgroup;
        tgMuted = (flco == 0) && (tg != 0) && (dst != tg);
        if (changed)
            LOG_I("dmr", "call: flco %u src %lu dst %lu%s 52=%02x 57=%02x",
                  flco, (unsigned long)src, (unsigned long)dst,
                  tgMuted ? " (muted)" : "", last52, last57);
        if (!callActive) {
            callBursts = 0;
            callDropped = 0;
            callSlot = 0;
            slotMuted = false;
            slotDisagree = 0;
            callSync = SYNC_UNKNOWN;
            syncTries = 0;
        }
        callActive = true;
        slotsSinceVoice = 0;
    } else if (term && callActive) {
        // The call ends unless voice follows within three slots.
        if (slotsSinceVoice < 17)
            slotsSinceVoice = 17;
        termSeen = true;
    }
}

/*
 * DMR_SYS_INT / SYS_INTER (PC3 -> EXTI3).  Register layout from the stock
 * handler (0x08038E58) and its frame dispatcher (0x08039410):
 *   0x82 = event flags, write-1-to-ack through 0x83; read first, as reading
 *          the other status registers consumes the master flag
 *   0x52 = [7:4] received colour code (repeater/CACH only)
 *   0x51 = [1:0] burst class (1 = voice, 2 = data), bit 2 = check failed,
 *          [7:4] data type for data bursts (1 = voice LC header,
 *          2 = terminator with LC)
 *   0x5F = sync status
 * Link control is in RX RAM from address 0x00: FLCO, FID, service options,
 * dst[3], src[3] (24 bit big-endian), then 3 parity bytes.
 */
static void handleSys()
{
    uint8_t flags = C6000.readCfgRegister(0x82);

    // While transmitting, events are only acknowledged
    if (txState != TX_OFF) {
        dmrDiag_inc(DIAG_SYS_IRQ);
        dmrDiag_set(DIAG_SYS_FLAGS, flags);
        C6000.writeCfgRegister(0x83, flags);
        return;
    }

    last52 = C6000.readCfgRegister(0x52);
    uint8_t reg51 = C6000.readCfgRegister(0x51);
    (void)C6000.readCfgRegister(0x5F);
    last57 = C6000.readCfgRegister(0x57);

    pollLinkControl(flags, reg51);

    C6000.writeCfgRegister(0x83, flags);

    pollVoicePort();
}

/*
 * Interrupt entry points.  All three lines share one priority, so a handler
 * never interrupts another in the middle of a U_SPI transaction.
 */
extern "C" void dmrSlotIrqImpl()
{
    EXTI->PR1 = EXTI_PR1_PR0;
    if (dmrRunning)
        handleSlot();
}

extern "C" void dmrSysIrqImpl()
{
    EXTI->PR1 = EXTI_PR1_PR3;
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

// DMR_TX_INT / RF_TX_INTER (PC2 -> EXTI2): fires ahead of a TX slot boundary.
// Unused while receive only; acknowledged so that it cannot stay pending.
void EXTI2_IRQHandler()
{
    EXTI->PR1 = EXTI_PR1_PR2;
    if (txState != TX_OFF) {
        txTxIrqs++;
        dmrDiag_inc(DIAG_TX_IRQ);
    }
}

void dmr_init()
{
    gpio_setMode(DMR_TS_INT, INPUT);
    gpio_setMode(DMR_SYS_INT, INPUT);
    gpio_setMode(DMR_TX_INT, INPUT);

    golayInit();
    gfInit();
    vspiInit();
    dmrDiag_init();

    callActive = false;
    voicePathOn = false;
    haveLast = false;
    lcHave = false;
    slotsSinceVoice = 0;
    dmrRunning = true;

    // HR_C6000 interrupts are active-low pulses (manual, section 5.1):
    // trigger on the falling edge.  The H7 EXTI block uses the "1" registers.
    EXTI->IMR1 |= EXTI_IMR1_IM0 | EXTI_IMR1_IM2 | EXTI_IMR1_IM3;
    EXTI->FTSR1 |= EXTI_FTSR1_TR0 | EXTI_FTSR1_TR2 | EXTI_FTSR1_TR3;
    EXTI->RTSR1 &= ~(EXTI_RTSR1_TR0 | EXTI_RTSR1_TR2 | EXTI_RTSR1_TR3);

    // SYSCFG sits on the APB4 bus of the H7 and its clock is off after reset;
    // without it the EXTI line routing below is silently ignored.
    RCC->APB4ENR |= RCC_APB4ENR_SYSCFGEN;
    (void)RCC->APB4ENR;

    // DMR_TS_INT/DMR_TX_INT/DMR_SYS_INT are on GPIOC pins 0, 2 and 3.
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PC | SYSCFG_EXTICR1_EXTI2_PC
                       | SYSCFG_EXTICR1_EXTI3_PC;

    NVIC_ClearPendingIRQ(EXTI0_IRQn);
    NVIC_ClearPendingIRQ(EXTI3_IRQn);
    NVIC_ClearPendingIRQ(EXTI2_IRQn);

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
    callActive = false;
}

void dmr_startTx(uint32_t src, uint32_t dst)
{
    if (callActive)
        endCall("transmit");

    // Group voice call link control: FLCO 0, FID 0, no service options,
    // destination and source IDs big-endian (as read on receive)
    txLc[0] = 0x00;
    txLc[1] = 0x00;
    txLc[2] = 0x00;
    txLc[3] = (dst >> 16) & 0xFF;
    txLc[4] = (dst >> 8) & 0xFF;
    txLc[5] = dst & 0xFF;
    txLc[6] = (src >> 16) & 0xFF;
    txLc[7] = (src >> 8) & 0xFF;
    txLc[8] = src & 0xFF;

    txStopReq = false;
    txHeaders = TX_HEADERS;
    txVoiceN = 0;
    txBursts = 0;
    txUnderruns = 0;
    txArmSlot = false;
    txSlotIrqs = 0;
    txTxIrqs = 0;

    /*
     * HR_C6000 manual, 5.4.4/5.4.5 ("active sending", "voice
     * transmission"): on the send request reg 0x40 = 0xA3, active mode
     * (bit 5), in which the chip builds its own 30 ms time axis and starts
     * the slot interrupt.  0xC3 (v40..v44) is passive mode: the time axis
     * comes from a received signal, so the radio could only transmit right
     * after receiving a call.  The manual's example runs in layer 2
     * non-relay mode (reg 0x10 = 0x6A): the timeslot setting's 0x6C / 0x6D
     * is put back afterwards.  Reg 0x56 bit 4: voice from TX RAM 0x30 (see
     * loadVoice).
     */
    C6000.writeCfgRegister(0x10, 0x6A);
    C6000.writeCfgRegister(0x56, 0x10); // 0x00 otherwise (HR_C6000 init)
    C6000.writeCfgRegister(0x40, 0xA3);

    LOG_I("dmr",
          "tx start: src %lu dst %lu, %lu slot interrupts since the "
          "last one",
          (unsigned long)src, (unsigned long)dst, (unsigned long)rxSlotIrqs);
    rxSlotIrqs = 0;
    dmrDiag_set(DIAG_STAGE, STAGE_STARTED);
    txState = TX_HEADER; // last: the slot interrupt starts from here
}

uint32_t dmr_rxSlotIrqs()
{
    return rxSlotIrqsTotal;
}

uint32_t dmr_txSlotIrqs()
{
    return txSlotIrqs;
}

void dmr_kickTx()
{
    if ((txState != TX_HEADER) || (txSlotIrqs != 0))
        return;

    // Arm the first header from here, with the DMR interrupts held off so
    // that a slot interrupt cannot split the sequence
    NVIC_DisableIRQ(EXTI0_IRQn);
    NVIC_DisableIRQ(EXTI3_IRQn);
    C6000.writeCfgRegister(0x41, 0x80);
    C6000.writeTxBurst(0x00, txLc, sizeof(txLc));
    C6000.writeCfgRegister(0x50, 0x10);
    C6000.writeCfgRegister(0x22, 0x86);
    txArmSlot = true; // the next slot interrupt is the idle one
    txHeaders--;
    NVIC_EnableIRQ(EXTI3_IRQn);
    NVIC_EnableIRQ(EXTI0_IRQn);

    LOG_W("dmr", "no slot interrupt 300 ms after PTT: first header armed "
                 "from the RTX thread");
}

void dmr_stopTx()
{
    txStopReq = true;
}

bool dmr_txDone()
{
    return txState == TX_DONE;
}

void dmr_endTx(uint32_t elapsedMs)
{
    if (txState == TX_OFF)
        return;

    bool clean = (txState == TX_DONE);
    txState = TX_OFF;

    // Transmission off (manual: reg 0x40 = 0x03), then passive reception
    // with the steps dmrMode() starts it with
    C6000.writeCfgRegister(0x41, 0x00);
    C6000.writeCfgRegister(0x40, 0x03);
    C6000.writeCfgRegister(0x56, 0x00);
    C6000.writeCfgRegister(0x10, slotModeReg);
    C6000.writeCfgRegister(0x41, 0x20);
    C6000.writeCfgRegister(0x41, 0x00);
    C6000.writeCfgRegister(0x41, 0x40);
    C6000.writeCfgRegister(0x40, 0x43);
    haveLast = false;
    lcHave = false;

    LOG_I("dmr", "tx end%s: %lu voice bursts, %lu frames sent as silence",
          clean ? "" : " (aborted)", (unsigned long)txBursts,
          (unsigned long)txUnderruns);
    LOG_I("dmr", "tx: %lu ms, %lu slot interrupts, %lu TX interrupts",
          (unsigned long)elapsedMs, (unsigned long)txSlotIrqs,
          (unsigned long)txTxIrqs);
    LOG_I("dmr", "tx: reg 0x10 back to %02x", slotModeReg);
    if (txBursts >= 1)
        LOG_HEX(LOG_LVL_INFO, "dmr", "tx burst 0", txFirstBursts[0], 27);
    if (txBursts >= 2)
        LOG_HEX(LOG_LVL_INFO, "dmr", "tx burst 1", txFirstBursts[1], 27);
}

void dmr_setSlotModeReg(uint8_t value)
{
    slotModeReg = value;
}

void dmr_setRxTalkgroup(uint32_t tg)
{
    rxTalkgroup = tg;
}

void dmr_setRxTimeslot(uint8_t ts)
{
    rxTimeslot = (ts > 2) ? 0 : ts;
}

bool dmr_getCallInfo(dmrCallInfo_t *out)
{
    if (!callActive || callMuted())
        return false;
    *out = callInfo;
    out->slot = callSlot;
    return true;
}
