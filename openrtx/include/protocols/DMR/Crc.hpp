/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef DMR_CRC_HPP
#define DMR_CRC_HPP

#include <cstdint>
#include <cstddef>

namespace DMR
{

/**
 * \brief Generic bit-at-a-time CRC, parameterised by width and
 *        polynomial, as used by several DMR PDU types.
 *
 * ETSI TS 102 361-1 Annex B.3 defines five different CRCs for DMR
 * (widths 7, 8, 9, 16 and 32 bits), each a plain, non-reflected CRC
 * with init value 0 and no final XOR: a standard textbook bit-serial
 * CRC register, run MSB-first over the input bits. This one small
 * routine, instantiated with each width/polynomial pair, implements
 * all five; see Crc.cpp for the DMR-specific widths/polynomials and
 * the per-PDU-type XOR masks.
 */
template < unsigned int WIDTH >
class Crc
{
public:
    constexpr explicit Crc(uint32_t polynomial) : polynomial(polynomial) { }

    /**
     * \param bits   input bits, one bit per byte (0 or 1), MSB-first
     *               bit order as transmitted over the air.
     * \param nBits  number of bits in \p bits.
     * \return the WIDTH-bit CRC remainder.
     */
    uint32_t compute(const uint8_t *bits, size_t nBits) const
    {
        uint32_t reg = 0;
        const uint32_t topBit = 1u << (WIDTH - 1);
        const uint32_t mask   = (WIDTH >= 32) ? 0xFFFFFFFFu : ((1u << WIDTH) - 1u);

        for (size_t i = 0; i < nBits; i++)
        {
            uint32_t bit = reg & topBit ? 1u : 0u;
            reg = (reg << 1) & mask;
            if (bit ^ (bits[i] & 1))
                reg ^= polynomial;
        }

        return reg & mask;
    }

private:
    uint32_t polynomial;
};

/**
 * \brief Helper to unpack bytes into a one-bit-per-byte buffer, MSB
 *        first, as expected by Crc<N>::compute() and the FEC codes.
 */
inline void unpackBits(const uint8_t *bytes, size_t nBytes, uint8_t *bits)
{
    for (size_t i = 0; i < nBytes; i++)
    {
        for (unsigned int b = 0; b < 8; b++)
            bits[i * 8 + b] = (bytes[i] >> (7 - b)) & 1;
    }
}

inline void packBits(const uint8_t *bits, size_t nBits, uint8_t *bytes)
{
    size_t nBytes = (nBits + 7) / 8;
    for (size_t i = 0; i < nBytes; i++)
        bytes[i] = 0;

    for (size_t i = 0; i < nBits; i++)
    {
        if (bits[i])
            bytes[i / 8] |= (uint8_t)(1u << (7 - (i % 8)));
    }
}

/**
 * \brief The ETSI CRCs actually used on the air interface.
 *
 * Polynomials from ETSI TS 102 361-1 Annex B.3.5-B.3.9, all plain
 * (non-reflected) bit-serial CRCs with a zero initial value. The
 * "on-air" value that actually appears in a PDU is, for CRC9 and
 * CRC16, the ones' complement of the raw register XORed with the
 * data-type mask (ETSI TS 102 361-1 Table B.3); CRC8 is used as-is
 * with no mask. CRC32 additionally needs its input byte-swapped
 * before feeding the register (used only for Rate 1/2/3/4 data
 * continuation blocks, which are out of scope for now -- see
 * docs/DMR_STATUS.md) and is exposed only as the raw primitive here.
 */
inline uint32_t crc7Raw(const uint8_t *bits, size_t nBits)
{
    static const Crc<7> c(0x27);
    return c.compute(bits, nBits);
}

inline uint32_t crc8Raw(const uint8_t *bits, size_t nBits)
{
    static const Crc<8> c(0x07);
    return c.compute(bits, nBits);
}

inline uint32_t crc9Raw(const uint8_t *bits, size_t nBits)
{
    static const Crc<9> c(0x059);
    return c.compute(bits, nBits);
}

inline uint32_t crc16Raw(const uint8_t *bits, size_t nBits)
{
    static const Crc<16> c(0x1021);
    return c.compute(bits, nBits);
}

inline uint32_t crc32Raw(const uint8_t *bits, size_t nBits)
{
    static const Crc<32> c(0x04C11DB7);
    return c.compute(bits, nBits);
}

/**
 * \brief On-air CRC16, i.e. what is actually transmitted/expected in a
 *        CSBK / Data Header / MBC Header / Unified Single Block Data
 *        PDU's trailing 16 bits: ~raw16(data) ^ mask.
 */
inline uint16_t crc16OnAir(const uint8_t *bits, size_t nBits, uint32_t mask)
{
    return (uint16_t)((~crc16Raw(bits, nBits) & 0xFFFFu) ^ mask);
}

/**
 * \brief On-air CRC9, as used by the PI Header: ~raw9(data) ^ mask.
 */
inline uint16_t crc9OnAir(const uint8_t *bits, size_t nBits, uint32_t mask)
{
    return (uint16_t)((~crc9Raw(bits, nBits) & 0x1FFu) ^ mask);
}

/**
 * \brief Per-PDU-type XOR masks applied to the computed CRC, ETSI TS
 *        102 361-1 Table B.3 "Data Type CRC Mask".
 */
namespace CrcMask
{
    constexpr uint32_t PiHeader                = 0x6969;
    constexpr uint32_t Csbk                    = 0xA5A5;
    constexpr uint32_t MbcHeader               = 0xAAAA;
    constexpr uint32_t DataHeader               = 0xCCCC;
    constexpr uint32_t UnifiedSingleBlockData  = 0x3333;
    constexpr uint32_t Rate12DataContinuation  = 0x0F0;
    constexpr uint32_t Rate34DataContinuation  = 0x1FF;
    constexpr uint32_t Rate1DataContinuation   = 0x10F;
    constexpr uint32_t ReverseChannel          = 0x7A;

    // Reed-Solomon(12,9,4) masks (Full LC / Terminator with LC), applied
    // to the 3-byte RS parity rather than to a CRC; kept here since the
    // standard tabulates them alongside the CRC masks.
    constexpr uint32_t VoiceLcHeader           = 0x969696;
    constexpr uint32_t TerminatorWithLc        = 0x999999;
}

}   // namespace DMR

#endif  // DMR_CRC_HPP
