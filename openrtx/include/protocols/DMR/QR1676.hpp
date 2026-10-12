/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef DMR_QR_16_7_6_HPP
#define DMR_QR_16_7_6_HPP

#include "LinearBlockCode.hpp"

namespace DMR
{

/**
 * \brief Quadratic Residue (16,7,6) FEC, used by the DMR EMB field.
 *
 * ETSI TS 102 361-1 Annex B.3.2. 7 information bits (4-bit colour code +
 * 1-bit PI + 2-bit LCSS) protected by 9 parity bits, correcting any
 * single bit error. See Golay2087.hpp for the decoding approach.
 */
class QR1676
{
public:
    static void encode(const uint8_t data[7], uint8_t codeword[16])
    {
        code().encode(data, codeword);
    }

    static bool decode(uint8_t codeword[16], uint8_t data[7])
    {
        return code().decode(codeword, data);
    }

private:
    static const LinearBlockCode<16, 7>& code()
    {
        static const uint16_t parity[7] =
        {
            0b001001111,
            0b100011110,
            0b110110111,
            0b111100010,
            0b111001001,
            0b011100101,
            0b001110011,
        };
        static const LinearBlockCode<16, 7> instance(parity);
        return instance;
    }
};

}   // namespace DMR

#endif  // DMR_QR_16_7_6_HPP
