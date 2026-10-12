/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef DMR_REED_SOLOMON_12_9_4_HPP
#define DMR_REED_SOLOMON_12_9_4_HPP

#include <cstdint>

namespace DMR
{

/**
 * \brief Reed-Solomon (12,9,4) over GF(256), used to protect the Full
 *        Link Control / Terminator-with-LC payload.
 *
 * ETSI TS 102 361-1 Annex B.3.6. Unlike the other DMR FEC fields this
 * one is *not* a CRC: 9 bytes of data produce 3 bytes of systematic
 * RS parity via a textbook LFSR-style systematic RS encoder over
 * GF(256) with the standard's own generator polynomial
 * (64, 56, 14, 1). The resulting 3 parity bytes are then XORed with a
 * fixed per-burst-type mask before transmission (0x969696 for the
 * Voice LC Header, 0x999999 for the Terminator with LC).
 *
 * The GF(256) exponential/log tables and generator polynomial
 * coefficients are the standard's own data (see ReedSolomon1294.cpp);
 * the encode routine below is the generic systematic-RS-encoder
 * algorithm, not copied from any particular implementation.
 */
class ReedSolomon1294
{
public:
    /**
     * \brief Compute the masked 3-byte RS parity for 9 bytes of data.
     * \param data 9 bytes of information.
     * \param mask 3-byte XOR mask (0x969696 / 0x999999), big-endian.
     * \param parity output: 3 bytes of parity.
     */
    static void generate(const uint8_t data[9], uint32_t mask, uint8_t parity[3]);

    /**
     * \brief Verify a 12-byte (9 data + 3 parity) block against its
     *        expected mask. Does not attempt error correction (DMR
     *        Full LC is single-shot per burst with no retransmission
     *        at this layer; a mismatch just means the burst is
     *        discarded).
     */
    static bool check(const uint8_t block[12], uint32_t mask);

private:
    static uint8_t gfMultiply(uint8_t a, uint8_t b);

    // expTable is 512 entries: two concatenated copies of the 255-cycle
    // GF(256) exponential table (plus the trailing zero), which lets
    // gfMultiply() index it directly with logTable[a]+logTable[b]
    // (max 254+254=508) without a modulo.
    static const uint8_t expTable[512];
    static const uint8_t logTable[256];
    static const uint8_t polynomial[3];    // non-zero leading coefficients
};

}   // namespace DMR

#endif  // DMR_REED_SOLOMON_12_9_4_HPP
