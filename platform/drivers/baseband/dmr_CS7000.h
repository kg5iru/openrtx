/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef DMR_CS7000_H
#define DMR_CS7000_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Configure GPIO/EXTI for the HR_C6000's three DMR-related interrupt lines
 * (DMR_TS_INT, DMR_SYS_INT, DMR_TX_INT) and enable the corresponding NVIC
 * IRQs. Does not touch the HR_C6000 itself -- call dmrMode() separately to
 * actually switch the chip into DMR operation.
 *
 * STATUS: untested, see dmr_CS7000.cpp header comment.
 */
void dmr_init();

/**
 * Disable the DMR-related EXTI interrupts configured by dmr_init().
 */
void dmr_terminate();

/** Link control of the call being received. */
typedef struct {
    uint8_t flco; ///< 0 = group voice, 3 = unit-to-unit voice
    uint32_t dst; ///< destination ID (talkgroup or radio)
    uint32_t src; ///< source radio ID
} dmrCallInfo_t;

/** Returns false when no call is being received. */
bool dmr_getCallInfo(dmrCallInfo_t *out);

/** Raw receive diagnostics, to see on screen what the HR_C6000 reports. */
typedef struct {
    uint32_t irqSys;  ///< SYS_INTER interrupts seen
    uint32_t irqSlot; ///< TIME_SLOT_INTER interrupts seen
    uint32_t bursts;  ///< SYS_INTER with the "burst received" flag
    uint32_t headers; ///< voice LC headers decoded
    uint8_t sync;     ///< last 0x5F
    uint8_t reg51;    ///< last 0x51
    uint8_t cc;       ///< last received colour code (0x52[7:4])
    uint8_t r40, r41, r81, r5f, r1f, r21; ///< live register readback
    uint8_t flagsOr;   ///< every SYS_INTER flag bit seen (0x82, ORed)
    uint8_t flagsLast; ///< 0x82 of the last SYS_INTER
    uint32_t polls;    ///< diagnostic polls done, a heartbeat
    uint32_t nVoice;   ///< valid voice-class bursts
    uint32_t nData;    ///< valid data-class bursts
    uint8_t v51;       ///< 0x51 of the last valid burst
    uint8_t lastType;  ///< data type of the last valid data burst
    uint8_t lc[9];     ///< RX RAM bytes read at the last valid data burst
    /// voice bursts overwritten before the audio thread took them
    uint32_t vDropped;
    uint32_t vFlagged; ///< voice bursts with 0x51 bit 2 set (kept)
    uint8_t pins;      ///< levels of PC0 (bit 0), PC2 (bit 1), PC3 (bit 2)
} dmrDebug_t;

void dmr_getDebug(dmrDebug_t *out);

/** Read the chip registers and interrupt pins for the diagnostics; thread context. */
void dmr_pollDiag();

/** Pop the last received voice burst (27 bytes = 3 AMBE+2 frames). */
bool dmr_popVoiceBurst(uint8_t out[27]);

/** Number of voice bursts waiting to be played. */
unsigned dmr_voiceQueued();

#ifdef __cplusplus
}
#endif

#endif /* DMR_CS7000_H */
