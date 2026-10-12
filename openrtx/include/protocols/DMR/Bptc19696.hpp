/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef DMR_BPTC_196_96_HPP
#define DMR_BPTC_196_96_HPP

#include <cstdint>

namespace DMR
{

/**
 * \brief BPTC(196,96), the "block product turbo code" used by every
 *        single-block DMR PDU carried in a data burst's 196 info bits
 *        (Voice LC Header, Terminator with LC, CSBK, Data Header, MBC
 *        Header, Unified Single Block Data).
 *
 * ETSI TS 102 361-1 Annex B.1.1. The 196 on-air bits are a fixed
 * bit-interleaved arrangement of a 13x15 matrix whose 11x9 submatrix
 * holds the 96 information bits, with the 13 rows each protected by a
 * Hamming (15,11,3) code and the 15 columns each protected by a
 * Hamming (13,9,3) code (three bits of the matrix are reserved
 * padding, not used). Decoding de-interleaves the bits back into the
 * matrix, corrects any single-bit error per row and per column, and
 * lifts out the 96 information bits; encoding does the reverse.
 *
 * The 196-entry bit permutation below is the standard's own table
 * (ETSI TS 102 361-1 Table B.3 and surrounding text); it is
 * reproduced here as plain data, cross-checked against the row/column
 * placement documented in the standard. The traversal/repair logic is
 * original.
 */
class Bptc19696
{
public:
    /**
     * \brief De-interleave 196 on-air bits, correct any single-bit
     *        error in each row/column, and extract the 96 information
     *        bits.
     * \param onAir 196 bits (one bit per byte, 0/1), as received.
     * \param info  output: 96 information bits.
     * \return false if a row or column carried more than one bit error
     *         (uncorrectable).
     */
    static bool decode(const uint8_t onAir[196], uint8_t info[96]);

    /**
     * \brief Build the 196 on-air bits (with Hamming row/column parity
     *        filled in and interleaved) from 96 information bits.
     */
    static void encode(const uint8_t info[96], uint8_t onAir[196]);

private:
    struct MapEntry
    {
        uint8_t interleaveIndex;    // position of this matrix cell in the on-air bit order
        uint8_t row;                // 1..13
        uint8_t col;                // 0..14
        bool reserved;              // padding bit, carries no data/parity
        bool hamming;                // row/column parity bit, not an information bit
    };

    static const MapEntry map[196];

    static void deinterleave(const uint8_t onAir[196], uint8_t matrix[13][15]);
    static void interleave(const uint8_t matrix[13][15], uint8_t onAir[196]);
};

}   // namespace DMR

#endif  // DMR_BPTC_196_96_HPP
