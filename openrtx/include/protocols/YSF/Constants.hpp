/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef YSF_CONSTANTS_H
#define YSF_CONSTANTS_H

#ifndef __cplusplus
#error This header is C++ only!
#endif

#include <cstdint>
#include <cstddef>
#include <array>

/*
 * Yaesu System Fusion (YSF), C4FM: 4800 symbols/s 4FSK, one frame every
 * 100 ms of 480 symbols (960 bits):
 *
 *   sync    40 bits   D4 71 C9 63 4D
 *   FICH   200 bits   frame information channel (Golay, convolutional code,
 *                     CRC, interleaved)
 *   payload 720 bits  five blocks of 144 bits; the split of each block
 *                     depends on the frame and data types in the FICH
 *
 * Bit order, field layouts and codes follow MMDVMHost (YSFDefines.h,
 * YSFFICH.cpp, YSFPayload.cpp) and MMDVM (YSFDefines.h).  Dibit to symbol:
 * 01 = +3, 00 = +1, 10 = -1, 11 = -3, as in M17.
 */

namespace YSF
{

static constexpr size_t SYMBOL_RATE = 4800;
static constexpr size_t FRAME_BYTES = 120;
static constexpr size_t FRAME_SYMBOLS = FRAME_BYTES * 4;
static constexpr size_t SYNC_BYTES = 5;
static constexpr size_t SYNC_SYMBOLS = SYNC_BYTES * 4;
static constexpr size_t FICH_BYTES = 25;
static constexpr size_t PAYLOAD_BYTES = 90;
static constexpr size_t PAYLOAD_OFFSET = SYNC_BYTES + FICH_BYTES;
static constexpr size_t CALLSIGN_LENGTH = 10;

using frame_t = std::array<uint8_t, FRAME_BYTES>;

static constexpr std::array<uint8_t, SYNC_BYTES> SYNC_WORD = { 0xD4, 0x71, 0xC9,
                                                               0x63, 0x4D };

static constexpr std::array<int8_t, SYNC_SYMBOLS> SYNC_SYMBOLS_VALUES = {
    -3, +3, +3, +1, +3, -3, +1, +3, -3, +1,
    -1, +3, +3, -1, +3, -3, +3, +1, -3, +3
};

/** Frame information (FICH FI field) */
enum FrameInfo : uint8_t {
    FI_HEADER = 0,
    FI_COMMUNICATIONS = 1,
    FI_TERMINATOR = 2,
    FI_TEST = 3
};

/** Data type (FICH DT field) */
enum DataType : uint8_t {
    DT_VD_MODE1 = 0,     ///< Voice/data mode 1: half-rate voice, DMR FEC
    DT_DATA_FR_MODE = 1, ///< Data full rate, no voice
    DT_VD_MODE2 = 2,     ///< Voice/data mode 2 ("DN"): half-rate voice
    DT_VOICE_FR = 3      ///< Voice full rate ("VW"): IMBE
};

/** Call mode (FICH CM field) */
enum CallMode : uint8_t { CM_GROUP1 = 0, CM_GROUP2 = 1, CM_INDIVIDUAL = 3 };

/**
 * Data whitening sequence applied to the DCH/CSD contents and to the voice
 * channel of V/D mode 2.
 */
static constexpr std::array<uint8_t, 20> WHITENING = {
    0x93, 0xD7, 0x51, 0x21, 0x9C, 0x2F, 0x6C, 0xD0, 0xEF, 0x0F,
    0xF8, 0x3D, 0xF1, 0x73, 0x20, 0x94, 0xED, 0x1E, 0x7C, 0xD8
};

} /* YSF */

#endif /* YSF_CONSTANTS_H */
