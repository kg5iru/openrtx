/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/*
 * DMR receive for the CS7000-PLUS: HR_C6000 interrupt handling, voice burst
 * capture and call tracking.  Receive only.
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

// Voice bursts waiting for the audio thread.
static constexpr uint8_t VOICE_FIFO = 8;
static uint8_t voiceFifo[VOICE_FIFO][27];
static volatile uint8_t vHead = 0; // written by the interrupts
static volatile uint8_t vTail = 0; // written by the audio thread

static uint8_t lastVoice[27];
static bool haveLast = false;
static bool voicePathOn = false;
static uint32_t callBursts = 0;
static uint32_t callDropped = 0;

static void endCall(const char *why)
{
    if (callActive)
        LOG_I("dmr", "call end (%s): %lu bursts, %lu lost", why,
              (unsigned long)callBursts, (unsigned long)callDropped);
    // The voice FIFO is left to drain: the last bursts of the call are still
    // waiting to be played.
    callActive = false;
    voicePathOn = false;
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
    // audio is played; the IDs stay those of the last header.
    if (!callActive) {
        callActive = true;
        callBursts = 0;
        callDropped = 0;
        LOG_I("dmr", "call start: late entry");
    }

    uint8_t nh = (vHead + 1) % VOICE_FIFO;
    if (nh == vTail) {
        vTail = (vTail + 1) % VOICE_FIFO; // full: drop the oldest
        callDropped++;
    }
    for (int i = 0; i < 27; i++)
        voiceFifo[vHead][i] = buf[i];
    vHead = nh;
    callBursts++;

    if (!voicePathOn) {
        C6000.writeCfgRegister(0x22, 0x26); // stock: voice path start
        voicePathOn = true;
    }
    slotsSinceVoice = 0;
}

// DMR_TS_INT / TIME_SLOT_INTER (PC0 -> EXTI0): every slot (30 ms).
static void handleSlot()
{
    // A receive request covers a single slot: the chip receives that slot and
    // then falls idle unless a fresh request is queued before the next
    // boundary.  0x50 = receive with the layer 2 access bit set.
    C6000.writeCfgRegister(0x41, 0x50);

    pollVoicePort();

    // End a call whose terminator was missed: no voice for 20 slots (600 ms).
    if (callActive && (++slotsSinceVoice >= 20))
        endCall("no voice");
}

static uint32_t be24(const uint8_t *p)
{
    return (uint32_t(p[0]) << 16) | (uint32_t(p[1]) << 8) | p[2];
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
 * Link control is read from RX RAM address 0x00, 9 bytes: FLCO, FID, service
 * options, dst[3], src[3] (24 bit big-endian).
 */
static void handleSys()
{
    uint8_t flags = C6000.readCfgRegister(0x82);
    (void)C6000.readCfgRegister(0x52);
    uint8_t reg51 = C6000.readCfgRegister(0x51);
    (void)C6000.readCfgRegister(0x5F);

    // Data bursts: 0x51 is not reliable on its own (it often reads as noise),
    // but the link control it leads to carries the chip's own check, and a
    // false header only re-reads the last LC.
    bool dataOk = ((reg51 & 0x04) == 0) && ((reg51 & 0x03) == 2);
    if (dataOk) {
        uint8_t type = reg51 >> 4;
        if (type == 1) // voice LC header
        {
            uint8_t lc[9];
            C6000.readRxBurst(0x00, lc, sizeof(lc));
            uint8_t flco = lc[0] & 0x3F;
            uint32_t dst = be24(&lc[3]);
            uint32_t src = be24(&lc[6]);
            bool changed = !callActive || (flco != callInfo.flco)
                        || (dst != callInfo.dst) || (src != callInfo.src);
            callInfo.flco = flco;
            callInfo.dst = dst;
            callInfo.src = src;
            // TODO: filter on configured talkgroup / contact.
            if (changed)
                LOG_I("dmr", "call start: flco %u src %lu dst %lu", flco,
                      (unsigned long)src, (unsigned long)dst);
            if (!callActive) {
                callBursts = 0;
                callDropped = 0;
            }
            callActive = true;
            slotsSinceVoice = 0;
        } else if ((type == 2) && callActive) // terminator with LC
        {
            // Many terminators read here are false (voice goes on right
            // after them), so a terminator only shortens the timeout: the
            // call ends unless voice follows within three slots.
            if (slotsSinceVoice < 17)
                slotsSinceVoice = 17;
        }
    }

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
}

void dmr_init()
{
    gpio_setMode(DMR_TS_INT, INPUT);
    gpio_setMode(DMR_SYS_INT, INPUT);
    gpio_setMode(DMR_TX_INT, INPUT);

    golayInit();
    vspiInit();

    callActive = false;
    voicePathOn = false;
    haveLast = false;
    slotsSinceVoice = 0;
    vTail = vHead;
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
