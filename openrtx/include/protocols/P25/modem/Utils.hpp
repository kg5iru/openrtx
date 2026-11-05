/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef P25_MODEM_UTILS_H
#define P25_MODEM_UTILS_H

#ifndef __cplusplus
#error This header is C++ only!
#endif

#include <cstddef>
#include <cstdint>
#include <array>
#include <assert.h>

namespace P25
{

/**
 * Utility function allowing to retrieve the value of a single bit from an
 * array of bytes. Bits are counted scanning from left to right, thus bit
 * number zero is the leftmost bit of array[0].
 */
template <size_t N>
inline bool getBit(const std::array<uint8_t, N> &array, const size_t pos)
{
    size_t i = pos / 8;
    size_t j = pos % 8;
    return (array[i] >> (7 - j)) & 0x01;
}

/**
 * Utility function allowing to set the value of a single bit from an array
 * of bytes. Bits are counted scanning from left to right, thus bit number
 * zero is the leftmost bit of array[0].
 */
template <size_t N>
inline void setBit(std::array<uint8_t, N> &array, const size_t pos,
                   const bool bit)
{
    size_t i = pos / 8;
    size_t j = pos % 8;
    uint8_t mask = 1 << (7 - j);
    array[i] = (array[i] & ~mask) | (bit ? mask : 0x00);
}

/**
 * Compute the hamming distance between two bytes.
 */
static inline uint8_t hammingDistance(const uint8_t x, const uint8_t y)
{
    return __builtin_popcount(x ^ y);
}

/**
 * Compare the 48-bit frame sync at the beginning of a frame against the
 * expected FS bytes, based on total hamming distance over the six bytes.
 *
 * @param frame: byte array whose first six bytes are checked.
 * @param maxDist: maximum allowed hamming distance, in bits.
 * @return true if the frame starts with a valid FS.
 */
template <size_t N>
static bool checkFrameSync(const uint8_t *frame,
                           const std::array<uint8_t, N> &target,
                           const uint8_t maxDist)
{
    static_assert(N == 6, "P25 FS is 48 bits");
    uint16_t hd = 0;
    for (size_t i = 0; i < N; i++)
        hd += hammingDistance(frame[i], target[i]);

    return (hd <= maxDist);
}

/**
 * Utility function allowing to set the value of a symbol on an array
 * of bytes. TIA-102 dibit-to-deviation map: 01 = +3, 00 = +1, 10 = -1,
 * 11 = -3 (spec polarity; hardware inversion is a platform concern).
 */
template <size_t N>
inline void setSymbol(std::array<uint8_t, N> &array, const size_t pos,
                      const int8_t symbol)
{
    switch (symbol) {
        case +3:
            setBit<N>(array, 2 * pos, 0);
            setBit<N>(array, 2 * pos + 1, 1);
            break;
        case +1:
            setBit<N>(array, 2 * pos, 0);
            setBit<N>(array, 2 * pos + 1, 0);
            break;
        case -1:
            setBit<N>(array, 2 * pos, 1);
            setBit<N>(array, 2 * pos + 1, 0);
            break;
        case -3:
            setBit<N>(array, 2 * pos, 1);
            setBit<N>(array, 2 * pos + 1, 1);
            break;
        default:
            assert("Error: unknown P25 symbol!");
    }
}

/**
 * Utility function to encode a given byte of data into 4FSK symbols, most
 * significant dibit first.
 */
inline std::array<int8_t, 4> byteToSymbols(uint8_t value)
{
    std::array<int8_t, 4> symbols;
    for (size_t i = 0; i < 4; i++) {
        bool b1 = (value >> (7 - 2 * i)) & 0x01;
        bool b2 = (value >> (7 - 2 * i - 1)) & 0x01;

        if (!b1 && b2)
            symbols[i] = +3;
        if (!b1 && !b2)
            symbols[i] = +1;
        if (b1 && !b2)
            symbols[i] = -1;
        if (b1 && b2)
            symbols[i] = -3;
    }
    return symbols;
}

/**
 * Offset of the apex of the parabola through three equally spaced points
 * (a, b, c) relative to the middle one, in sample units. Returns zero when
 * the points do not describe a peak.
 */
static inline float parabolicPeakOffset(const int32_t a, const int32_t b,
                                        const int32_t c)
{
    float denom = static_cast<float>(a) - 2.0f * static_cast<float>(b)
                + static_cast<float>(c);
    if (denom >= 0.0f)
        return 0.0f;

    float mu = 0.5f * (static_cast<float>(a) - static_cast<float>(c)) / denom;
    if (mu > 0.5f)
        mu = 0.5f;
    if (mu < -0.5f)
        mu = -0.5f;
    return mu;
}

/**
 * Four-point Lagrange (cubic) interpolation of equally spaced samples
 * s[0..3] at fractional position f in [0, 1) between s[1] and s[2].
 */
static inline float cubicInterpolate(const int16_t s[4], const float f)
{
    float l0 = -f * (f - 1.0f) * (f - 2.0f) / 6.0f;
    float l1 = (f + 1.0f) * (f - 1.0f) * (f - 2.0f) / 2.0f;
    float l2 = -(f + 1.0f) * f * (f - 2.0f) / 2.0f;
    float l3 = (f + 1.0f) * f * (f - 1.0f) / 6.0f;
    return s[0] * l0 + s[1] * l1 + s[2] * l2 + s[3] * l3;
}

} /* P25 */

#endif /* P25_MODEM_UTILS_H */
