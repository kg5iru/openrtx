/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef DMR_CS7000_H
#define DMR_CS7000_H

#include <stdint.h>
#include <stdbool.h>

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

/** Pop the last received voice burst (27 bytes = 3 AMBE+2 frames). */
bool dmr_popVoiceBurst(uint8_t out[27]);

#endif /* DMR_CS7000_H */
