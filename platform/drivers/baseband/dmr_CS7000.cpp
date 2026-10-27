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
static uint8_t voiceBurst[27];
static volatile bool voiceBurstReady = false;
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

static constexpr uint8_t SYNC_LOSS_SLOTS = 13;
static uint8_t missedSlots = 0;

void EXTI0_IRQHandler()
{
    if ((EXTI->PR1 & EXTI_PR1_PR0) == 0)
        return;

    EXTI->PR1 = EXTI_PR1_PR0;
    dbg.irqSlot++;

    uint8_t sync = C6000.readCfgRegister(0x5F);
    if (sync & 0x03) {
        missedSlots = 0;
        return;
    }

    if (++missedSlots < SYNC_LOSS_SLOTS)
        return;

    missedSlots = 0;
    C6000.writeCfgRegister(0x41, 0x00);
    C6000.writeCfgRegister(0x40, 0x03);
    C6000.writeCfgRegister(0x41, 0x20);
    C6000.writeCfgRegister(0x41, 0x40);
    C6000.writeCfgRegister(0x40, 0x43);
    C6000.writeCfgRegister(0x5F, 0x30);
    endCall();
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

static void endCall()
{
    callActive = false;
    voiceBurstReady = false;
    rxState = DmrRxState::LISTENING;
}

void EXTI3_IRQHandler()
{
    if ((EXTI->PR1 & EXTI_PR1_PR3) == 0)
        return;

    EXTI->PR1 = EXTI_PR1_PR3;

    uint8_t flags = C6000.readCfgRegister(0x82);
    // Flags first: reading the other status registers consumes the master
    // flag that announced them (order as in the stock firmware).
    uint8_t reg52 = C6000.readCfgRegister(0x52);
    uint8_t reg51 = C6000.readCfgRegister(0x51);
    uint8_t sync = C6000.readCfgRegister(0x5F);

    uint8_t rxCc = (reg52 >> 4) & 0x0F;
    dbg.irqSys++;
    dbg.sync = sync;
    dbg.reg51 = reg51;
    dbg.cc = rxCc;
    if (flags & 0x08)
        dbg.bursts++;
    uint8_t cls = reg51 & 0x03;
    bool valid = ((reg51 & 0x04) == 0) && (cls == 1 || cls == 2);

    if ((flags & 0x08) && valid && (sync & 0x03)) {
        // No software colour code filter: the chip gates on register 0x1F, and
        // only repeaters (CACH) ever report a received code at all.
        if (cls == 2) {
            uint8_t type = reg51 >> 4;
            if (type == 1) // voice LC header
            {
                uint8_t lc[9];
                C6000.readRxBurst(0x00, lc, sizeof(lc));
                callInfo.flco = lc[0] & 0x3F;
                callInfo.dst = be24(&lc[3]);
                callInfo.src = be24(&lc[6]);
                // TODO: filter on configured contact / talkgroup.
                callActive = true;
                dbg.headers++;
                rxState = DmrRxState::VOICE_HEADER_OK;
            } else if (type == 2) // terminator with LC
            {
                endCall();
            }
        } else if (callActive) // voice burst
        {
            // Voice bytes come from V_SPI (see vspiReadVoice).  The RX RAM
            // windows (0x30/0x60) are not the codec input in the stock
            // firmware.
            vspiReadVoice(voiceBurst);

            if (rxState != DmrRxState::VOICE_RX)
                C6000.writeCfgRegister(0x22, 0x26); // stock: voice path start

            voiceBurstReady = true;
            rxState = DmrRxState::VOICE_RX;
        }
    }

    C6000.writeCfgRegister(0x83, flags);
}

// DMR_TX_INT / RF_TX_INTER (GPIOC,2 -> EXTI2): fires ahead of a TX slot
// boundary so the MCU can key the RF chain in time. Unused in this RX-only
// pass -- stubbed so the interrupt is at least acknowledged and doesn't
// hang the NVIC if it fires unexpectedly.
void EXTI2_IRQHandler()
{
    if ((EXTI->PR1 & EXTI_PR1_PR2) == 0)
        return;

    EXTI->PR1 = EXTI_PR1_PR2;

    // TODO: TX slot handling goes here once RX bring-up is solid.
}

void dmr_init()
{
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
    NVIC_SetPriority(EXTI0_IRQn, 10);
    NVIC_SetPriority(EXTI3_IRQn, 11);
    NVIC_SetPriority(EXTI2_IRQn, 10);

    NVIC_EnableIRQ(EXTI0_IRQn);
    NVIC_EnableIRQ(EXTI3_IRQn);
    NVIC_EnableIRQ(EXTI2_IRQn);
}

void dmr_terminate()
{
    NVIC_DisableIRQ(EXTI0_IRQn);
    NVIC_DisableIRQ(EXTI3_IRQn);
    NVIC_DisableIRQ(EXTI2_IRQn);

    EXTI->IMR1 &= ~(EXTI_IMR1_IM0 | EXTI_IMR1_IM2 | EXTI_IMR1_IM3);
}

// Fetch the last received voice burst: 3 AMBE+2 frames of 9 bytes each, to be
// fed one at a time to ambe_cs7000_decode().  Returns false if none pending.
bool dmr_popVoiceBurst(uint8_t out[27])
{
    if (!voiceBurstReady)
        return false;
    for (int i = 0; i < 27; i++)
        out[i] = voiceBurst[i];
    voiceBurstReady = false;
    return true;
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
}

void dmr_pollDiag()
{
    dbg.r40 = C6000.readCfgRegister(0x40);
    dbg.r41 = C6000.readCfgRegister(0x41);
    dbg.r81 = C6000.readCfgRegister(0x81);
    dbg.r5f = C6000.readCfgRegister(0x5F);
    dbg.r1f = C6000.readCfgRegister(0x1F);
    dbg.r21 = C6000.readCfgRegister(0x21);
    dbg.pins = (gpio_readPin(DMR_TS_INT) ? 1 : 0)
             | (gpio_readPin(DMR_TX_INT) ? 2 : 0)
             | (gpio_readPin(DMR_SYS_INT) ? 4 : 0);
}
