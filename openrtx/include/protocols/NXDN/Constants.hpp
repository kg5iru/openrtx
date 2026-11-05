/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef NXDN_CONSTANTS_H
#define NXDN_CONSTANTS_H

#ifndef __cplusplus
#error This header is C++ only!
#endif

#include <cstdint>
#include <cstddef>
#include <array>

/*
 * NXDN, 4800 bit/s (NXDN48, 6.25 kHz channels): 2400 symbols/s 4FSK, outer
 * symbols at +-1050 Hz, one frame every 80 ms of 192 symbols (384 bits):
 *
 *   FSW      20 bits   frame sync word CD F5 9
 *   LICH     16 bits   link information: 8 bits, each sent as (bit, 1)
 *   SACCH    60 bits   slow associated control channel (26 bits of data,
 *                      CRC-6, convolutional code, punctured, interleaved)
 *   payload 288 bits   four AMBE+2 voice frames of 72 bits (the DMR
 *                      layout), or one or two FACCH1 of 144 bits in their
 *                      place (80 bits of data, CRC-12, convolutional code)
 *
 * Everything but the FSW is scrambled (SCRAMBLER below).  Bit order, field
 * layouts and codes follow MMDVMHost (NXDNDefines.h, NXDNLICH.cpp,
 * NXDNSACCH.cpp, NXDNFACCH1.cpp, NXDNControl.cpp).  Dibit to symbol: 01 =
 * +3, 00 = +1, 10 = -1, 11 = -3, as in M17 and YSF.
 */

namespace NXDN
{

static constexpr size_t SYMBOL_RATE = 2400;
static constexpr size_t FRAME_BYTES = 48;
static constexpr size_t FRAME_BITS = FRAME_BYTES * 8;
static constexpr size_t FRAME_SYMBOLS = FRAME_BITS / 2;
static constexpr size_t SYNC_SYMBOLS = 10;
static constexpr size_t SYNC_BITS = 2 * SYNC_SYMBOLS;

using frame_t = std::array<uint8_t, FRAME_BYTES>;

/// Frame sync word: 20 bits, the low nibble of the third byte is the LICH
static constexpr std::array<uint8_t, 3> SYNC_WORD = { 0xCD, 0xF5, 0x90 };
static constexpr std::array<uint8_t, 3> SYNC_MASK = { 0xFF, 0xFF, 0xF0 };

static constexpr std::array<int8_t, SYNC_SYMBOLS> SYNC_SYMBOLS_VALUES = {
    -3, +1, -3, +3, -3, -3, +3, +3, -1, +3
};

/// Bit offsets of the channels in a frame
static constexpr size_t LICH_OFFSET = SYNC_BITS;
static constexpr size_t SACCH_OFFSET = LICH_OFFSET + 16;
static constexpr size_t PAYLOAD_OFFSET = SACCH_OFFSET + 60;
static constexpr size_t FACCH1_BITS = 144;
static constexpr size_t VOICE_BYTE_OFFSET = PAYLOAD_OFFSET / 8;
static constexpr size_t AMBE_BYTES = 9;

/// Frame scrambler, XORed over the whole frame (zero on the FSW)
static constexpr std::array<uint8_t, FRAME_BYTES> SCRAMBLER = {
    0x00, 0x00, 0x00, 0x82, 0xA0, 0x88, 0x8A, 0x00, 0xA2, 0xA8, 0x82, 0x8A,
    0x82, 0x02, 0x20, 0x08, 0x8A, 0x20, 0xAA, 0xA2, 0x82, 0x08, 0x22, 0x8A,
    0xAA, 0x08, 0x28, 0x88, 0x28, 0x28, 0x00, 0x0A, 0x02, 0x82, 0x20, 0x28,
    0x82, 0x2A, 0xAA, 0x20, 0x22, 0x80, 0xA8, 0x8A, 0x08, 0xA0, 0xAA, 0x02
};

/** LICH radio channel type */
enum Rfct : uint8_t { RFCT_RCCH = 0, RFCT_RTCH = 1, RFCT_RDCH = 2 };

/** LICH functional channel type */
enum Fct : uint8_t {
    FCT_SACCH_NS = 0, ///< SACCH, not superframe: header, terminator
    FCT_UDCH = 1,     ///< user data channel
    FCT_SACCH_SS = 2, ///< SACCH, superframe: voice
    FCT_SACCH_IDLE = 3
};

/** LICH option: which halves of the payload a FACCH1 takes */
enum Steal : uint8_t {
    STEAL_FACCH = 0,    ///< both
    STEAL_FACCH1_1 = 1, ///< the first
    STEAL_FACCH1_2 = 2, ///< the second
    STEAL_NONE = 3      ///< none: four voice frames
};

/** LICH direction */
enum Direction : uint8_t { DIR_INBOUND = 0, DIR_OUTBOUND = 1 };

/** SACCH structure: part of a message spread over four frames */
enum Structure : uint8_t {
    SR_SINGLE = 0,
    SR_4_4 = 0,
    SR_3_4 = 1,
    SR_2_4 = 2,
    SR_1_4 = 3
};

/** Layer 3 message types */
enum MessageType : uint8_t {
    MSG_VCALL = 0x01,
    MSG_TX_REL = 0x08,
    MSG_IDLE = 0x10
};

} /* NXDN */

#endif /* NXDN_CONSTANTS_H */
