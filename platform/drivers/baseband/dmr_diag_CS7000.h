/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef DMR_DIAG_CS7000_H
#define DMR_DIAG_CS7000_H

#include <stdint.h>
#include "stm32h7xx.h"

/*
 * DMR transmit bring-up diagnostics for the CS7000-PLUS.
 *
 * Breadcrumbs are kept in the RTC backup registers, which survive a reset
 * (watchdog, fault or pin) as long as the backup domain stays powered: after
 * a lock-up the next start logs where the transmission was.  The independent
 * watchdog, started at the first DMR transmission, resets the radio (and so
 * unkeys the transmitter) if the RTX thread stops running for 5 s.
 */

#ifdef __cplusplus
extern "C" {
#endif

enum dmrDiagReg {
    DIAG_MAGIC = 0, ///< DIAG_MAGIC_VALUE while a transmission is under way
    DIAG_STAGE,     ///< Last transmit stage reached (enum dmrDiagStage)
    DIAG_SLOT_IRQ,  ///< Slot interrupts since the start of the transmission
    DIAG_SYS_IRQ,   ///< System interrupts since the start
    DIAG_TX_IRQ,    ///< RF_TX_INTER interrupts since the start
    DIAG_ENC,       ///< Frames encoded; bit 31 set while inside the codec
    DIAG_IN_IRQ,    ///< Interrupt being handled: 1 slot, 2 sys, 3 tx, 0 none
    DIAG_SYS_FLAGS, ///< Last reg 0x82 read while transmitting
    DIAG_BEAT,      ///< RTX thread updates since the start
    DIAG_COUNT
};

enum dmrDiagStage {
    STAGE_IDLE = 0,
    STAGE_PTT,       ///< PTT seen, about to start the encoder
    STAGE_ENCODER,   ///< Encoder thread created
    STAGE_KEYED,     ///< RF chain keyed
    STAGE_STARTED,   ///< dmr_startTx() done
    STAGE_HEADER,    ///< A voice LC header armed
    STAGE_VOICE,     ///< A voice burst armed
    STAGE_TERM,      ///< Terminator armed
    STAGE_END,       ///< Stock end sequence written
    STAGE_FINISHING, ///< OpMode stopping the transmission
};

#define DIAG_MAGIC_VALUE 0xD7A0C0DEu

static inline volatile uint32_t *dmrDiag_reg(enum dmrDiagReg r)
{
    return &RTC->BKP0R + (unsigned)r;
}

static inline void dmrDiag_set(enum dmrDiagReg r, uint32_t v)
{
    *dmrDiag_reg(r) = v;
}

static inline void dmrDiag_inc(enum dmrDiagReg r)
{
    *dmrDiag_reg(r) += 1;
}

static inline uint32_t dmrDiag_get(enum dmrDiagReg r)
{
    return *dmrDiag_reg(r);
}

/**
 * Enable access to the backup registers, and log the breadcrumbs of a
 * transmission that did not finish before the last reset.  Called once, at
 * the first DMR mode entry.
 */
void dmrDiag_init(void);

/** Start a transmission's breadcrumbs (counters cleared). */
void dmrDiag_txStart(void);

/** End of a transmission: breadcrumbs cleared. */
void dmrDiag_txEnd(void);

/** Start the independent watchdog (5 s), if not running yet. */
void dmrDiag_startWatchdog(void);

/** Reload the watchdog; harmless while it is not running. */
void dmrDiag_kickWatchdog(void);

#ifdef __cplusplus
}
#endif

#endif /* DMR_DIAG_CS7000_H */
