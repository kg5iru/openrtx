/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef DMR_GOLAY_20_8_7_HPP
#define DMR_GOLAY_20_8_7_HPP

#include "LinearBlockCode.hpp"

namespace DMR
{

/**
 * \brief Golay (20,8,7) FEC, used by the DMR Slot Type field.
 *
 * ETSI TS 102 361-1 Annex B.3.3. 8 information bits (4-bit colour code +
 * 4-bit data/payload type) protected by 12 parity bits, correcting any
 * single bit error. The parity table below is the systematic generator
 * matrix's parity half as printed in the standard (and cross-checked
 * against the equivalent matrix used by other open-source DMR
 * implementations); the decoding algorithm itself is the generic
 * syndrome decoder in LinearBlockCode.hpp.
 */
class Golay2087
{
public:
    static void encode(const uint8_t data[8], uint8_t codeword[20])
    {
        code().encode(data, codeword);
    }

    static bool decode(uint8_t codeword[20], uint8_t data[8])
    {
        return code().decode(codeword, data);
    }

private:
    // A function-local static keeps this class header-only under C++14
    // (no out-of-line definition needed for an ODR-used constexpr array).
    static const LinearBlockCode<20, 8>& code()
    {
        static const uint16_t parity[8] =
        {
            0b001111011010,
            0b110110011001,
            0b011011001101,
            0b001101100111,
            0b110111000110,
            0b101010010111,
            0b100100111110,
            0b100011101011,
        };
        static const LinearBlockCode<20, 8> instance(parity);
        return instance;
    }
};

}   // namespace DMR

#endif  // DMR_GOLAY_20_8_7_HPP
