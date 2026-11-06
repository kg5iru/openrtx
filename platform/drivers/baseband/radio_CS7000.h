/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef RADIO_CS7000_H
#define RADIO_CS7000_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Audio source of the next FM transmission: the microphone (default), or the
 * MCU's DAC on the HR_C6000 line input (USB audio).  Pre-emphasis, band pass
 * filter, limiter and CTCSS apply either way.
 *
 * @param lineIn: true for the line input.
 */
void radio_setFmTxLineIn(const bool lineIn);

#ifdef __cplusplus
}
#endif

#endif /* RADIO_CS7000_H */
