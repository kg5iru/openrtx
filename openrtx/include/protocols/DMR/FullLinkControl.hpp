/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef DMR_FULL_LINK_CONTROL_HPP
#define DMR_FULL_LINK_CONTROL_HPP

#include <cstdint>
#include <cstring>
#include "ReedSolomon1294.hpp"
#include "Crc.hpp"

namespace DMR
{

/**
 * \brief Full Link Control, the 96-bit payload (after BPTC(196,96)
 *        decoding) of a Voice LC Header or Terminator-with-LC burst.
 *
 * ETSI TS 102 361-1 clause 9.3.11 / Annex B.3.6. Laid out as 9 bytes
 * of content (1 byte: protect flag + reserved + 6-bit FLCO, 1 byte:
 * Feature Set ID, 7 bytes: FLCO-specific payload) followed by 3 bytes
 * of Reed-Solomon(12,9,4) parity, XOR-masked by burst type.
 *
 * Only the envelope (split/validate/build the 12 bytes) is
 * implemented; interpreting the 7-byte payload according to the FLCO
 * (Group Voice Channel User, Unit-to-Unit, Talker Alias, GPS info,
 * ...) is not -- see docs/DMR_STATUS.md.
 */
class FullLinkControl
{
public:
    bool    protectFlag = false;
    uint8_t flco        = 0;        // 6 bits
    uint8_t featureSetId = 0;
    uint8_t payload[7]  = { };      // FLCO-specific, opaque for now

    /**
     * \param block96 12 bytes (96 bits) as recovered from
     *        Bptc19696::decode() and then packed with packBits().
     * \param mask    CrcMask::VoiceLcHeader or CrcMask::TerminatorWithLc.
     */
    static bool decode(const uint8_t block96[12], uint32_t mask, FullLinkControl &out)
    {
        if (!ReedSolomon1294::check(block96, mask))
            return false;

        out.protectFlag  = (block96[0] & 0x80) != 0;
        out.flco         = block96[0] & 0x3F;
        out.featureSetId = block96[1];
        memcpy(out.payload, &block96[2], 7);
        return true;
    }

    void encode(uint8_t block96[12], uint32_t mask) const
    {
        block96[0] = (uint8_t)((protectFlag ? 0x80 : 0x00) | (flco & 0x3F));
        block96[1] = featureSetId;
        memcpy(&block96[2], payload, 7);

        uint8_t parity[3];
        ReedSolomon1294::generate(block96, mask, parity);
        memcpy(&block96[9], parity, 3);
    }
};

}   // namespace DMR

#endif  // DMR_FULL_LINK_CONTROL_HPP
