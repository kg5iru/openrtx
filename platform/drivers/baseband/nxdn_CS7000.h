/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef NXDN_CS7000_H
#define NXDN_CS7000_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Call being received in NXDN mode, for the user interface. */
typedef struct {
    uint16_t src; ///< Source unit ID, 0 if not received yet
    uint16_t dst; ///< Destination group (or unit) ID
    bool group;   ///< Group call
    bool ids;     ///< src, dst and group are known
    uint8_t ran;  ///< Radio access number of the last SACCH
} nxdnCallInfo_t;

/**
 * Get the call being received.
 *
 * @param out: call information.
 * @return false when no call is being received.
 */
bool nxdn_getCallInfo(nxdnCallInfo_t *out);

#ifdef __cplusplus
}
#endif

#endif /* NXDN_CS7000_H */
