/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef DMR_BURST_HPP
#define DMR_BURST_HPP

#include <cstdint>
#include <cstring>
#include "Constants.hpp"

namespace DMR
{

/**
 * \brief The three fixed-size pieces of one 264-bit DMR burst.
 *
 * ETSI TS 102 361-1 clause 9.1: Info(98) + middle(48) + Info(98). The
 * two Info halves concatenate (info1 || info2) into the 196 bits fed
 * to BPTC(196,96) for a data burst. The 48-bit middle section is
 * either a Sync pattern (burst A of a voice superframe, or any data
 * burst), a Slot Type field (data bursts only, 20 of the 48 bits,
 * remainder reserved), or EMB + embedded signalling (voice bursts
 * B..E). Which one it is has to be worked out by the caller: a data
 * burst's Slot Type is Golay-decoded first (SlotType::decode()), and
 * if that fails the 48 bits are compared against the known Sync
 * patterns (Sync::classify()); anything else is assumed to be EMB.
 *
 * CACH (the 3-byte Common Announcement CHannel some repeater/MS
 * timeslot layouts prepend to each burst) is not handled here -- it
 * sits outside the 264-bit burst and is relevant only to trunking,
 * which is out of scope for now (see docs/DMR_STATUS.md).
 */
struct Burst
{
    uint8_t info1[BurstLength::infoBits];
    uint8_t middle[BurstLength::middleBits];
    uint8_t info2[BurstLength::infoBits];

    /**
     * \param onAir 264 bits (one bit per byte, 0/1), MSB-first as
     *        received off the air.
     */
    static void split(const uint8_t onAir[BurstLength::totalBits], Burst &out)
    {
        memcpy(out.info1,  onAir, BurstLength::infoBits);
        memcpy(out.middle, onAir + BurstLength::infoBits, BurstLength::middleBits);
        memcpy(out.info2,  onAir + BurstLength::infoBits + BurstLength::middleBits,
               BurstLength::infoBits);
    }

    void join(uint8_t onAir[BurstLength::totalBits]) const
    {
        memcpy(onAir, info1, BurstLength::infoBits);
        memcpy(onAir + BurstLength::infoBits, middle, BurstLength::middleBits);
        memcpy(onAir + BurstLength::infoBits + BurstLength::middleBits, info2,
               BurstLength::infoBits);
    }

    /**
     * \brief The 196 bits (info1 || info2) that feed BPTC(196,96) for
     *        a data burst.
     */
    void infoBits196(uint8_t out[196]) const
    {
        memcpy(out, info1, BurstLength::infoBits);
        memcpy(out + BurstLength::infoBits, info2, BurstLength::infoBits);
    }

    void setInfoBits196(const uint8_t in[196])
    {
        memcpy(info1, in, BurstLength::infoBits);
        memcpy(info2, in + BurstLength::infoBits, BurstLength::infoBits);
    }

    /**
     * \brief Pack the 48 middle bits into a uint64_t (bit 0 of
     *        \c middle in the MSB of the low 48 bits), as expected by
     *        Sync::classify().
     */
    uint64_t middleAsUint64() const
    {
        uint64_t v = 0;
        for (unsigned int i = 0; i < BurstLength::middleBits; i++)
            v = (v << 1) | (middle[i] & 1);
        return v;
    }
};

}   // namespace DMR

#endif  // DMR_BURST_HPP
