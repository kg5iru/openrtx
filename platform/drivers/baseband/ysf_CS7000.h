/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef YSF_CS7000_H
#define YSF_CS7000_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Call being received in YSF mode, for the user interface. */
typedef struct {
    char src[11]; ///< Source callsign, trimmed ("" if not received yet)
    char dst[11]; ///< Destination callsign, trimmed
    uint8_t dgid; ///< Digital group ID from the FICH
    uint8_t dt;   ///< Data type: 0 V/D 1, 1 data, 2 V/D 2 (DN), 3 VW
} ysfCallInfo_t;

/**
 * Get the call being received.
 *
 * @param out: call information.
 * @return false when no call is being received.
 */
bool ysf_getCallInfo(ysfCallInfo_t *out);

#ifdef __cplusplus
}
#endif

#endif /* YSF_CS7000_H */
