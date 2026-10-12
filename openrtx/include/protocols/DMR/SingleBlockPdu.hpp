/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef DMR_SINGLE_BLOCK_PDU_HPP
#define DMR_SINGLE_BLOCK_PDU_HPP

#include <cstdint>
#include <cstring>
#include "Crc.hpp"

namespace DMR
{

/**
 * \brief Generic envelope shared by CSBK, Data Header, MBC Header and
 *        Unified Single Block Data: 96 bits (after BPTC(196,96)
 *        decoding) = 80 bits of PDU content + a 16-bit CRC-CCITT,
 *        masked per burst type (ETSI TS 102 361-1 clause 9.3 and
 *        Annex B.3.12).
 *
 * This only validates/builds the envelope; the 10-byte content's
 * internal fields (CSBK opcode and its many payloads, Data Header
 * fields, ...) are not parsed -- see docs/DMR_STATUS.md. A higher
 * layer that needs them can interpret \c content directly.
 */
class SingleBlockPdu
{
public:
    uint8_t content[10] = { };     // 80 bits

    static bool decode(const uint8_t block96[12], uint32_t mask, SingleBlockPdu &out)
    {
        uint8_t bits[80];
        unpackBits(block96, 10, bits);
        uint16_t expected = (uint16_t)((block96[10] << 8) | block96[11]);
        uint16_t got = crc16OnAir(bits, 80, mask);
        if (got != expected)
            return false;

        memcpy(out.content, block96, 10);
        return true;
    }

    void encode(uint8_t block96[12], uint32_t mask) const
    {
        memcpy(block96, content, 10);
        uint8_t bits[80];
        unpackBits(content, 10, bits);
        uint16_t crc = crc16OnAir(bits, 80, mask);
        block96[10] = (uint8_t)(crc >> 8);
        block96[11] = (uint8_t) crc;
    }
};

}   // namespace DMR

#endif  // DMR_SINGLE_BLOCK_PDU_HPP
