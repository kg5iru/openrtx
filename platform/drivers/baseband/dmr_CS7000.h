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
    uint8_t slot; ///< timeslot, 1 or 2; 0 when unknown (no timeslot set)
} dmrCallInfo_t;

/** Returns false when no call is being received. */
bool dmr_getCallInfo(dmrCallInfo_t *out);

/**
 * Talkgroup to receive: group calls to other talkgroups are muted and not
 * shown.  0 receives every call.  Private calls are always received.
 */
void dmr_setRxTalkgroup(uint32_t tg);

/**
 * Timeslot to receive, 1 or 2: calls on the other slot are muted and not
 * shown.  0 receives both.  The chip reports the slot only in the mode
 * radio_CS7000.cpp sets when a timeslot is selected.
 */
void dmr_setRxTimeslot(uint8_t ts);

/**
 * Start transmitting a group voice call in direct mode (bring-up): three
 * voice LC headers, then voice bursts from dmrAudio_popTxFrame(), until
 * dmr_stopTx().  The caller keys the RF chain before and starts the encoder.
 *
 * @param src: own radio ID.
 * @param dst: talkgroup.
 */
void dmr_startTx(uint32_t src, uint32_t dst);

/**
 * Tell the driver the value written to reg 0x10 (layer 2 mode / timeslot),
 * to put back after a transmission, which runs in non-relay mode.
 */
void dmr_setSlotModeReg(uint8_t value);

/** @return slot interrupts seen while receiving, since dmr_init(). */
uint32_t dmr_rxSlotIrqs();

/** @return slot interrupts seen since dmr_startTx(). */
uint32_t dmr_txSlotIrqs();

/**
 * Bring-up fallback: arm the first voice LC header from the calling thread,
 * for when no slot interrupt has come since dmr_startTx().
 */
void dmr_kickTx();

/** Request the end of the transmission: a terminator is sent next. */
void dmr_stopTx();

/** @return true once the terminator has been sent. */
bool dmr_txDone();

/**
 * Back to receive; to be called after dmr_txDone(), or to abort.
 *
 * @param elapsedMs: duration of the transmission, for the log.
 */
void dmr_endTx(uint32_t elapsedMs);

#ifdef __cplusplus
}
#endif

#endif /* DMR_CS7000_H */
