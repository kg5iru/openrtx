/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef P25_MODEM_CONSTANTS_H
#define P25_MODEM_CONSTANTS_H

#ifndef __cplusplus
#error This header is C++ only!
#endif

#include <cstdint>
#include <cstddef>
#include <array>

namespace P25
{

/*
 * P25 Phase 1 air interface constants (TIA-102).
 */
static constexpr size_t SYMBOL_RATE = 4800;
static constexpr size_t FRAME_BYTES = 216; ///< One LDU: 180 ms
static constexpr size_t FRAME_SYMBOLS = FRAME_BYTES * 4;

using frame_t = std::array<uint8_t, FRAME_BYTES>;

/*
 * Frame synchronization: P25 uses a single 48-bit frame sync (FS), the same
 * for every data unit; the DUID in the following NID separates frame types.
 * FS = 0x5575F5FF77FF.
 */
static constexpr size_t SYNCWORD_SYMBOLS = 24;

static constexpr std::array<int8_t, SYNCWORD_SYMBOLS> FS_SYNC_WORD = {
    +3, +3, +3, +3, +3, -3, +3, +3, -3, -3, +3, +3,
    -3, -3, -3, -3, +3, -3, +3, -3, -3, -3, -3, -3
};

static constexpr std::array<uint8_t, 6> FS_SYNC_BYTES = { 0x55, 0x75, 0xF5,
                                                          0xFF, 0x77, 0xFF };

/*
 * Maximum hamming distance, in bits, for the FS check.
 */
static constexpr uint8_t FS_MAX_DISTANCE = 4;

} /* P25 */

#endif /* P25_MODEM_CONSTANTS_H */
