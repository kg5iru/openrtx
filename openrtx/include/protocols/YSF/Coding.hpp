/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef YSF_CODING_H
#define YSF_CODING_H

#ifndef __cplusplus
#error This header is C++ only!
#endif

#include <cstdint>
#include <cstddef>

/*
 * Bit-level helpers and the small codes used by YSF.  Bits are numbered from
 * the MSB of byte 0, as in MMDVMHost.
 */

namespace YSF
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

/**
 * Dibit interleaver used by the FICH and the data channels: dibit i of the
 * coded stream goes to bit position 2 * (i / cols) + 40 * (i % cols) of the
 * channel, where cols is 5 (FICH, V/D mode 2 DCH) or 9 (CSD, V/D mode 1 DCH).
 */
static inline size_t dibitPosition(const size_t i, const size_t cols)
{
    return (2 * (i / cols)) + (40 * (i % cols));
}

/**
 * Bit interleaver of the V/D mode 2 voice channel (104 bits): bit i goes to
 * position (i / 26) + 4 * (i % 26).
 */
static inline size_t vchPosition(const size_t i)
{
    return (i / 26) + (4 * (i % 26));
}

/**
 * CRC-16 of the FICH and data channels: polynomial 0x1021, initial value 0,
 * result inverted, stored high byte first after the data.
 *
 * @param data: data bytes.
 * @param len: number of data bytes, CRC excluded.
 * @return the CRC.
 */
uint16_t crc16(const uint8_t *data, const size_t len);

/**
 * Check the CRC stored in the two bytes following the data.
 *
 * @param data: data bytes followed by the CRC.
 * @param len: number of data bytes, CRC excluded.
 * @return true if the CRC matches.
 */
bool checkCrc16(const uint8_t *data, const size_t len);

/**
 * Append the CRC to the data.
 *
 * @param data: data bytes, with two bytes of room after them.
 * @param len: number of data bytes, CRC excluded.
 */
void addCrc16(uint8_t *data, const size_t len);

/**
 * Rate 1/2, K = 5 convolutional encoder (G1 = 0x19, G2 = 0x17, the M17
 * code), with the dibit interleaver above: encode nBits of data starting from
 * the MSB of in[0] and write the 2 * nBits coded bits into the channel.
 *
 * @param in: data bits.
 * @param nBits: number of data bits, tail bits included.
 * @param channel: destination channel bits.
 * @param cols: interleaver columns.
 */
void convEncode(const uint8_t *in, const size_t nBits, uint8_t *channel,
                const size_t cols);

} /* YSF */

#endif /* YSF_CODING_H */
