/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef DMR_CONSTANTS_HPP
#define DMR_CONSTANTS_HPP

#include <cstdint>

namespace DMR
{

/**
 * \brief DMR air-interface burst layout, ETSI TS 102 361-1 clause 9.1.
 *
 * A DMR physical-layer burst is 264 bits (33 bytes). For a voice or
 * data burst that carries a 196-bit BPTC-coded payload, the layout is
 * Info(98) + Sync-or-(EMB+embedded signalling)(48) + Info(98), plus a
 * trailing CACH (not part of the burst proper, present on repeater
 * timeslot boundaries). For a reverse/direct burst with a Slot Type
 * field, the 48-bit middle section is Slot Type(20) + reserved(28)
 * instead of Sync/EMB.
 */
namespace BurstLength
{
    constexpr unsigned int totalBits   = 264;
    constexpr unsigned int infoBits    = 98;      // each of the two halves
    constexpr unsigned int middleBits  = 48;      // sync, or EMB+embedded signalling
    constexpr unsigned int syncBits    = 48;
    constexpr unsigned int embBits     = 16;
    constexpr unsigned int embSigBits  = 32;
    constexpr unsigned int slotTypeBits = 20;
    constexpr unsigned int cachBits    = 24;
}

/**
 * \brief 48-bit DMR sync patterns, ETSI TS 102 361-1 Table 9.2.
 *
 * Reproduced as plain numeric data mandated by the standard (also
 * cross-referenced against MMDVM's DMRDefines.h, which tabulates the
 * same constants under GPLv2+). Each pattern both identifies which
 * side sent the burst (base station / mobile station) and whether it
 * is a voice or data burst; a burst with neither Slot Type nor a
 * recognisable sync in its middle 48 bits is a voice superframe
 * "embedded signalling" burst (B..E) instead, see Emb.hpp.
 */
namespace Sync
{
    constexpr uint64_t mask         = 0x0000FFFFFFFFFFFFULL;
    constexpr uint64_t msData       = 0x0000D5D7F77FD757ULL;
    constexpr uint64_t msVoice      = 0x00007F7D5DD57DFDULL;
    constexpr uint64_t bsData       = 0x0000DFF57D75DF5DULL;
    constexpr uint64_t bsVoice      = 0x0000755FD7DF75F7ULL;

    enum class Pattern
    {
        None,
        MsData,
        MsVoice,
        BsData,
        BsVoice,
    };

    /**
     * \brief Classify a 48-bit value against the four known sync
     *        patterns, allowing up to \p maxHammingDistance bit
     *        differences (DMR implementations typically tolerate a
     *        handful of bit errors here since the sync carries no FEC
     *        of its own).
     */
    inline Pattern classify(uint64_t bits48, unsigned int maxHammingDistance = 4)
    {
        const struct { uint64_t pattern; Pattern id; } table[] =
        {
            { msData,  Pattern::MsData  },
            { msVoice, Pattern::MsVoice },
            { bsData,  Pattern::BsData  },
            { bsVoice, Pattern::BsVoice },
        };

        for (const auto &entry : table)
        {
            uint64_t diff = (bits48 ^ entry.pattern) & mask;
            unsigned int distance = 0;
            while (diff)
            {
                distance += diff & 1;
                diff >>= 1;
            }
            if (distance <= maxHammingDistance)
                return entry.id;
        }

        return Pattern::None;
    }
}

/**
 * \brief Slot Type "data/payload type" field, ETSI TS 102 361-1 Table
 *        9.3. Carried, together with the 4-bit colour code, as the
 *        8 Golay(20,8)-protected info bits of a data burst's Slot
 *        Type field.
 */
enum class DataType : uint8_t
{
    PiHeader         = 0,
    VoiceLcHeader    = 1,
    TerminatorWithLc = 2,
    Csbk             = 3,
    MbcHeader        = 4,
    MbcContinuation  = 5,
    DataHeader       = 6,
    Rate12Data       = 7,
    Rate34Data       = 8,
    Idle             = 9,
    Rate1Data        = 10,
    // 11-15 reserved
};

}   // namespace DMR

#endif  // DMR_CONSTANTS_HPP
