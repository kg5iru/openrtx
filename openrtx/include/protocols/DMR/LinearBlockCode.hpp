/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef DMR_LINEAR_BLOCK_CODE_HPP
#define DMR_LINEAR_BLOCK_CODE_HPP

#include <cstdint>

namespace DMR
{

/**
 * \brief Generic systematic binary linear block code, (N, K).
 *
 * All the short FEC codes used by the DMR air interface (Golay (20,8),
 * Quadratic Residue (16,7,6), Hamming (15,11,3) and Hamming (13,9,3)) are
 * systematic codes: a codeword is simply the K data bits followed by
 * (N - K) parity bits, each parity bit being the XOR of a fixed subset of
 * the data bits. That subset is exactly what the "generator matrix" in
 * ETSI TS 102 361-1 Annex B describes.
 *
 * Because the code is systematic, the whole encode/decode machinery
 * collapses to one small routine parameterised only by that subset
 * ("parity[k][p] == 1" iff data bit k contributes to parity bit p), which
 * is why a single template implements all four codes here instead of
 * four bespoke ones. Decoding corrects at most one bit, which is all any
 * of these codes guarantee (minimum Hamming distance 3 or 4, i.e. single
 * error correction only) via standard syndrome decoding: the syndrome of
 * a single-bit error always equals the corresponding column of the
 * parity-check matrix, so a flipped data bit k produces syndrome
 * parity[k][*], and a flipped parity bit p produces the unit vector e_p.
 *
 * This is a from-scratch implementation of the standard generic
 * algorithm for decoding a systematic linear block code; only the
 * parity-bit tables fed to it (in Golay2087.cpp, QR1676.cpp, ...) are
 * taken from the ETSI standard's own tables.
 */
template < unsigned int N, unsigned int K >
class LinearBlockCode
{
public:
    static constexpr unsigned int numParityBits = N - K;

    /**
     * \param parity parity[k] is a bitmask of (N-K) bits: bit p of
     *        parity[k] is 1 iff data bit k contributes to parity bit p.
     *        This is exactly row k of the standard's generator matrix,
     *        restricted to its parity columns, packed MSB-first.
     */
    constexpr explicit LinearBlockCode(const uint16_t *parity) : parity(parity) { }

    /**
     * \brief Encode K data bits (one bit per byte, 0 or 1) into an N-bit
     *        systematic codeword (data bits followed by parity bits).
     */
    void encode(const uint8_t *data, uint8_t *codeword) const
    {
        for (unsigned int k = 0; k < K; k++)
            codeword[k] = data[k];

        uint16_t acc = 0;
        for (unsigned int k = 0; k < K; k++)
        {
            if (data[k])
                acc ^= parity[k];
        }

        for (unsigned int p = 0; p < numParityBits; p++)
            codeword[K + p] = (acc >> (numParityBits - 1 - p)) & 1;
    }

    /**
     * \brief Check an N-bit codeword and correct a single bit error in
     *        place, if present. Returns false if the codeword carries
     *        more than one bit error (uncorrectable by this code).
     *        On success, writes out the K data bits.
     */
    bool decode(uint8_t *codeword, uint8_t *data) const
    {
        uint16_t syndrome = 0;
        for (unsigned int k = 0; k < K; k++)
        {
            if (codeword[k])
                syndrome ^= parity[k];
        }
        for (unsigned int p = 0; p < numParityBits; p++)
            syndrome ^= (uint16_t)(codeword[K + p] & 1) << (numParityBits - 1 - p);

        if (syndrome != 0)
        {
            int flip = -1;

            for (unsigned int k = 0; k < K; k++)
            {
                if (parity[k] == syndrome)
                {
                    flip = (int) k;
                    break;
                }
            }

            if (flip < 0)
            {
                // Single set bit in the syndrome means the error is in
                // one of the parity bits themselves.
                unsigned int popcount = 0;
                unsigned int bitIndex = 0;
                for (unsigned int p = 0; p < numParityBits; p++)
                {
                    if ((syndrome >> p) & 1)
                    {
                        popcount++;
                        bitIndex = p;
                    }
                }
                if (popcount == 1)
                    flip = (int)(K + (numParityBits - 1 - bitIndex));
            }

            if (flip < 0)
                return false;

            codeword[flip] ^= 1;
        }

        for (unsigned int k = 0; k < K; k++)
            data[k] = codeword[k];

        return true;
    }

private:
    const uint16_t *parity;
};

}   // namespace DMR

#endif  // DMR_LINEAR_BLOCK_CODE_HPP
