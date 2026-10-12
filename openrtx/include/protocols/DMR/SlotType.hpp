/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef DMR_SLOT_TYPE_HPP
#define DMR_SLOT_TYPE_HPP

#include "Golay2087.hpp"
#include "Constants.hpp"

namespace DMR
{

/**
 * \brief The 20-bit Slot Type field present in the middle of every
 *        DMR *data* burst (not voice bursts, which carry Sync or
 *        embedded signalling there instead).
 *
 * ETSI TS 102 361-1 clause 9.1.5: 4-bit colour code + 4-bit data type
 * (see DataType in Constants.hpp), Golay (20,8,7) protected.
 */
struct SlotType
{
    uint8_t colorCode;  // 0-15
    DataType dataType;

    /**
     * \brief Decode and error-correct a 20-bit Slot Type field.
     * \return false if it carried more than a single bit error.
     */
    static bool decode(const uint8_t bits20[20], SlotType &out)
    {
        uint8_t codeword[20];
        for (unsigned int i = 0; i < 20; i++)
            codeword[i] = bits20[i];

        uint8_t info[8];
        if (!Golay2087::decode(codeword, info))
            return false;

        out.colorCode = (uint8_t)((info[0] << 3) | (info[1] << 2) | (info[2] << 1) | info[3]);
        out.dataType  = (DataType)((info[4] << 3) | (info[5] << 2) | (info[6] << 1) | info[7]);
        return true;
    }

    void encode(uint8_t bits20[20]) const
    {
        uint8_t info[8] =
        {
            (uint8_t)((colorCode >> 3) & 1), (uint8_t)((colorCode >> 2) & 1),
            (uint8_t)((colorCode >> 1) & 1), (uint8_t)(colorCode & 1),
            (uint8_t)(((uint8_t) dataType >> 3) & 1), (uint8_t)(((uint8_t) dataType >> 2) & 1),
            (uint8_t)(((uint8_t) dataType >> 1) & 1), (uint8_t)((uint8_t) dataType & 1),
        };
        Golay2087::encode(info, bits20);
    }
};

}   // namespace DMR

#endif  // DMR_SLOT_TYPE_HPP
