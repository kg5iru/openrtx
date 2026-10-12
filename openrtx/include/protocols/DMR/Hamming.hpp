/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef DMR_HAMMING_HPP
#define DMR_HAMMING_HPP

#include "LinearBlockCode.hpp"

namespace DMR
{

/**
 * \brief Hamming (15,11,3), the row code of BPTC(196,96).
 *
 * ETSI TS 102 361-1 Annex B.3.4.
 */
class Hamming15113
{
public:
    static void encode(const uint8_t data[11], uint8_t codeword[15])
    {
        code().encode(data, codeword);
    }

    static bool decode(uint8_t codeword[15], uint8_t data[11])
    {
        return code().decode(codeword, data);
    }

private:
    static const LinearBlockCode<15, 11>& code()
    {
        static const uint16_t parity[11] =
        {
            0b1001, 0b1101, 0b1111, 0b1110, 0b0111, 0b1010,
            0b0101, 0b1011, 0b1100, 0b0110, 0b0011,
        };
        static const LinearBlockCode<15, 11> instance(parity);
        return instance;
    }
};

/**
 * \brief Hamming (13,9,3), the column code of BPTC(196,96).
 *
 * ETSI TS 102 361-1 Annex B.3.4.
 */
class Hamming1393
{
public:
    static void encode(const uint8_t data[9], uint8_t codeword[13])
    {
        code().encode(data, codeword);
    }

    static bool decode(uint8_t codeword[13], uint8_t data[9])
    {
        return code().decode(codeword, data);
    }

private:
    static const LinearBlockCode<13, 9>& code()
    {
        static const uint16_t parity[9] =
        {
            0b1111, 0b1110, 0b0111, 0b1010, 0b0101,
            0b1011, 0b1100, 0b0110, 0b0011,
        };
        static const LinearBlockCode<13, 9> instance(parity);
        return instance;
    }
};

}   // namespace DMR

#endif  // DMR_HAMMING_HPP
