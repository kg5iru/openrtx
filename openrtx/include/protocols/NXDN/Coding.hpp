/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef NXDN_CODING_H
#define NXDN_CODING_H

#ifndef __cplusplus
#error This header is C++ only!
#endif

#include <cstdint>
#include <cstddef>
#include <array>
#include "protocols/NXDN/Constants.hpp"

/*
 * Channel coding of the NXDN frame: scrambler, LICH, SACCH and FACCH1.
 * Bits are numbered from the MSB of byte 0, as in MMDVMHost.  The decode
 * functions take a descrambled frame, the encode functions write into one
 * that is scrambled afterwards.
 */

namespace NXDN
{

static inline bool readBit(const uint8_t *buf, const size_t pos)
{
    return ((buf[pos >> 3] >> (7 - (pos & 7))) & 0x01) != 0;
}

static inline void writeBit(uint8_t *buf, const size_t pos, const bool bit)
{
    uint8_t mask = 0x80 >> (pos & 7);
    if (bit)
        buf[pos >> 3] |= mask;
    else
        buf[pos >> 3] &= ~mask;
}

/** SACCH contents: 26 bits (structure, RAN, 18 data bits), MSB first. */
using sacch_t = std::array<uint8_t, 4>;

/** FACCH1 contents: 80 bits, a layer 3 message. */
using facch1_t = std::array<uint8_t, 10>;

/**
 * CRC-6 of the SACCH (polynomial 0x27, initial value 0x3F).
 *
 * @param data: data bits, from the MSB of data[0].
 * @param bits: number of data bits.
 */
uint8_t crc6(const uint8_t *data, const size_t bits);

/** CRC-12 of the FACCH1 (polynomial 0x80F, initial value 0xFFF). */
uint16_t crc12(const uint8_t *data, const size_t bits);

/** XOR the frame with the scrambler sequence (scrambles and descrambles). */
void scramble(frame_t &frame);

/** Bit errors of the frame sync word at the start of a frame. */
uint8_t syncErrors(const frame_t &frame);

/** Write the frame sync word at the start of a frame. */
void encodeSync(frame_t &frame);

/** LICH byte from its fields, with its parity bit. */
uint8_t makeLich(const uint8_t rfct, const uint8_t fct, const uint8_t option,
                 const uint8_t direction);

static inline uint8_t lichRfct(const uint8_t lich)
{
    return (lich >> 6) & 0x03;
}

static inline uint8_t lichFct(const uint8_t lich)
{
    return (lich >> 4) & 0x03;
}

static inline uint8_t lichOption(const uint8_t lich)
{
    return (lich >> 2) & 0x03;
}

static inline uint8_t lichDirection(const uint8_t lich)
{
    return (lich >> 1) & 0x01;
}

/**
 * Read the LICH of a frame.
 *
 * @return true if its parity bit is right.
 */
bool decodeLich(const frame_t &frame, uint8_t &lich);

/** Write the LICH of a frame. */
void encodeLich(const uint8_t lich, frame_t &frame);

/** SACCH contents from the RAN (0..63), structure and 18 data bits. */
sacch_t makeSacch(const uint8_t ran, const uint8_t structure,
                  const uint8_t data[3]);

static inline uint8_t sacchRan(const sacch_t &sacch)
{
    return sacch[0] & 0x3F;
}

static inline uint8_t sacchStructure(const sacch_t &sacch)
{
    return (sacch[0] >> 6) & 0x03;
}

/** The 18 data bits of a SACCH, MSB first in data[0..2]. */
void sacchData(const sacch_t &sacch, uint8_t data[3]);

/**
 * Decode the SACCH of a frame.
 *
 * @return true if its CRC is right.
 */
bool decodeSacch(const frame_t &frame, sacch_t &sacch);

/** Encode a SACCH into a frame. */
void encodeSacch(const sacch_t &sacch, frame_t &frame);

/**
 * Decode a FACCH1.
 *
 * @param offset: bit offset in the frame, PAYLOAD_OFFSET for the first half
 * of the payload, PAYLOAD_OFFSET + FACCH1_BITS for the second.
 * @return true if its CRC is right.
 */
bool decodeFacch1(const frame_t &frame, const size_t offset, facch1_t &data);

/** Encode a FACCH1 into a frame, at a bit offset as decodeFacch1(). */
void encodeFacch1(const facch1_t &data, const size_t offset, frame_t &frame);

} /* NXDN */

#endif /* NXDN_CODING_H */
