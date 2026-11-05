/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <cstdlib>
#include <cstring>
#include "protocols/NXDN/Coding.hpp"

using namespace NXDN;

/*
 * Rate 1/2, K = 5 convolutional code of the SACCH and FACCH1: G1 = 1 + D^3 +
 * D^4, G2 = 1 + D + D^2 + D^4 (the M17 and YSF code), G1 sent first.
 */
static void convEncode(const uint8_t *in, const size_t nBits, uint8_t *out)
{
    uint8_t mem = 0; // bit 0 = newest input bit

    for (size_t i = 0; i < nBits; i++) {
        mem = ((mem << 1) | (readBit(in, i) ? 1 : 0)) & 0x1F;
        writeBit(out, 2 * i, (__builtin_popcount(mem & 0x19) & 1) != 0);
        writeBit(out, (2 * i) + 1, (__builtin_popcount(mem & 0x17) & 1) != 0);
    }
}

/*
 * Hard decision Viterbi decoder of that code, as MMDVMHost's
 * NXDNConvolution: symbols are 0 and 2 for received bits, 1 for punctured
 * (erased) ones.  The coded stream ends with four steps of zero input
 * beyond the tail, so that the chain back starts from state 0.
 */
static constexpr size_t MAX_STEPS = 100;

static void viterbiDecode(const uint8_t *sym, const size_t steps, uint8_t *out,
                          const size_t nBits)
{
    static constexpr uint8_t BRANCH1[8] = { 0, 0, 0, 0, 2, 2, 2, 2 };
    static constexpr uint8_t BRANCH2[8] = { 0, 2, 2, 0, 0, 2, 2, 0 };
    static constexpr uint16_t M = 4;

    uint16_t metrics[2][16];
    uint16_t decisions[MAX_STEPS];
    memset(metrics, 0, sizeof(metrics));

    uint16_t *oldM = metrics[0];
    uint16_t *newM = metrics[1];

    for (size_t s = 0; s < steps; s++) {
        uint8_t s0 = sym[2 * s];
        uint8_t s1 = sym[(2 * s) + 1];
        uint16_t d = 0;

        for (uint8_t i = 0; i < 8; i++) {
            uint8_t j = i * 2;
            uint16_t metric = std::abs(BRANCH1[i] - s0)
                            + std::abs(BRANCH2[i] - s1);

            uint16_t m0 = oldM[i] + metric;
            uint16_t m1 = oldM[i + 8] + (M - metric);
            bool d0 = (m0 >= m1);
            newM[j] = d0 ? m1 : m0;

            m0 = oldM[i] + (M - metric);
            m1 = oldM[i + 8] + metric;
            bool d1 = (m0 >= m1);
            newM[j + 1] = d1 ? m1 : m0;

            d |= (d0 ? (1 << j) : 0) | (d1 ? (1 << (j + 1)) : 0);
        }

        decisions[s] = d;
        uint16_t *tmp = oldM;
        oldM = newM;
        newM = tmp;
    }

    uint32_t state = 0;
    size_t s = steps;
    size_t n = nBits;
    while (n-- > 0) {
        s--;
        uint8_t bit = (decisions[s] >> (state >> 4)) & 1;
        state = (bit << 7) | (state >> 1);
        writeBit(out, n, bit != 0);
    }
}

uint8_t NXDN::crc6(const uint8_t *data, const size_t bits)
{
    uint8_t crc = 0x3F;
    for (size_t i = 0; i < bits; i++) {
        bool b = readBit(data, i) != ((crc & 0x20) != 0);
        crc <<= 1;
        if (b)
            crc ^= 0x27;
    }
    return crc & 0x3F;
}

uint16_t NXDN::crc12(const uint8_t *data, const size_t bits)
{
    uint16_t crc = 0x0FFF;
    for (size_t i = 0; i < bits; i++) {
        bool b = readBit(data, i) != ((crc & 0x0800) != 0);
        crc <<= 1;
        if (b)
            crc ^= 0x080F;
    }
    return crc & 0x0FFF;
}

void NXDN::scramble(frame_t &frame)
{
    for (size_t i = 0; i < FRAME_BYTES; i++)
        frame[i] ^= SCRAMBLER[i];
}

uint8_t NXDN::syncErrors(const frame_t &frame)
{
    uint8_t errs = 0;
    for (size_t i = 0; i < SYNC_WORD.size(); i++)
        errs += __builtin_popcount((frame[i] ^ SYNC_WORD[i]) & SYNC_MASK[i]);
    return errs;
}

void NXDN::encodeSync(frame_t &frame)
{
    for (size_t i = 0; i < SYNC_WORD.size(); i++)
        frame[i] = (frame[i] & ~SYNC_MASK[i]) | SYNC_WORD[i];
}

static bool lichParity(const uint8_t lich)
{
    // As MMDVMHost: set for the RDCH header/terminator and voice values
    uint8_t top = lich & 0xF0;
    return (top == 0x80) || (top == 0xB0);
}

uint8_t NXDN::makeLich(const uint8_t rfct, const uint8_t fct,
                       const uint8_t option, const uint8_t direction)
{
    uint8_t lich = ((rfct & 0x03) << 6) | ((fct & 0x03) << 4)
                 | ((option & 0x03) << 2) | ((direction & 0x01) << 1);
    return lich | (lichParity(lich) ? 1 : 0);
}

bool NXDN::decodeLich(const frame_t &frame, uint8_t &lich)
{
    // Each LICH bit is sent as a dibit (bit, 1): the second bit is not read
    lich = 0;
    for (size_t i = 0; i < 8; i++) {
        if (readBit(frame.data(), LICH_OFFSET + (2 * i)))
            lich |= 0x80 >> i;
    }

    return ((lich & 0x01) != 0) == lichParity(lich);
}

void NXDN::encodeLich(const uint8_t lich, frame_t &frame)
{
    for (size_t i = 0; i < 8; i++) {
        writeBit(frame.data(), LICH_OFFSET + (2 * i),
                 (lich & (0x80 >> i)) != 0);
        writeBit(frame.data(), LICH_OFFSET + (2 * i) + 1, true);
    }
}

sacch_t NXDN::makeSacch(const uint8_t ran, const uint8_t structure,
                        const uint8_t data[3])
{
    sacch_t s{};
    s[0] = ((structure & 0x03) << 6) | (ran & 0x3F);
    for (size_t i = 0; i < 18; i++)
        writeBit(s.data(), 8 + i, readBit(data, i));
    return s;
}

void NXDN::sacchData(const sacch_t &sacch, uint8_t data[3])
{
    memset(data, 0, 3);
    for (size_t i = 0; i < 18; i++)
        writeBit(data, i, readBit(sacch.data(), 8 + i));
}

/*
 * SACCH: 26 bits + CRC-6 + 4 tail bits, coded to 72 bits, every sixth one
 * (5, 11, .. 71) punctured, and the 60 left interleaved over 5 rows of 12:
 * bit i of the punctured stream goes to 5 * (i % 12) + i / 12.
 */
static inline size_t sacchPosition(const size_t i)
{
    return SACCH_OFFSET + (5 * (i % 12)) + (i / 12);
}

bool NXDN::decodeSacch(const frame_t &frame, sacch_t &sacch)
{
    uint8_t sym[80];
    size_t n = 0;
    for (size_t i = 0; i < 60; i++) {
        if ((n % 6) == 5)
            sym[n++] = 1;
        sym[n++] = readBit(frame.data(), sacchPosition(i)) ? 2 : 0;
    }
    sym[n++] = 1; // position 71, punctured
    while (n < sizeof(sym))
        sym[n++] = 0;

    uint8_t dec[5] = { 0 };
    viterbiDecode(sym, 40, dec, 36);

    for (size_t i = 0; i < sacch.size(); i++)
        sacch[i] = dec[i];
    sacch[3] &= 0xC0; // the 26 data bits only

    uint8_t crc = 0;
    for (size_t i = 0; i < 6; i++)
        crc = (crc << 1) | (readBit(dec, 26 + i) ? 1 : 0);

    return crc == crc6(dec, 26);
}

void NXDN::encodeSacch(const sacch_t &sacch, frame_t &frame)
{
    uint8_t data[5] = { 0 };
    for (size_t i = 0; i < 26; i++)
        writeBit(data, i, readBit(sacch.data(), i));
    uint8_t crc = crc6(data, 26);
    for (size_t i = 0; i < 6; i++)
        writeBit(data, 26 + i, (crc & (0x20 >> i)) != 0);

    uint8_t coded[9];
    convEncode(data, 36, coded);

    size_t n = 0;
    for (size_t i = 0; i < 72; i++) {
        if ((i % 6) == 5)
            continue;
        writeBit(frame.data(), sacchPosition(n), readBit(coded, i));
        n++;
    }
}

/*
 * FACCH1: 80 bits + CRC-12 + 4 tail bits, coded to 192 bits, every fourth
 * one from the second (1, 5, .. 189) punctured, and the 144 left
 * interleaved over 9 rows of 16: bit i goes to 9 * (i % 16) + i / 16.
 */
static inline size_t facch1Position(const size_t offset, const size_t i)
{
    return offset + (9 * (i % 16)) + (i / 16);
}

bool NXDN::decodeFacch1(const frame_t &frame, const size_t offset,
                        facch1_t &data)
{
    uint8_t sym[200];
    size_t n = 0;
    for (size_t i = 0; i < FACCH1_BITS; i++) {
        if ((n % 4) == 1)
            sym[n++] = 1;
        sym[n++] = readBit(frame.data(), facch1Position(offset, i)) ? 2 : 0;
    }
    while (n < sizeof(sym))
        sym[n++] = 0;

    uint8_t dec[12] = { 0 };
    viterbiDecode(sym, 100, dec, 96);

    for (size_t i = 0; i < data.size(); i++)
        data[i] = dec[i];

    uint16_t crc = 0;
    for (size_t i = 0; i < 12; i++)
        crc = (crc << 1) | (readBit(dec, 80 + i) ? 1 : 0);

    return crc == crc12(dec, 80);
}

void NXDN::encodeFacch1(const facch1_t &data, const size_t offset,
                        frame_t &frame)
{
    uint8_t in[12] = { 0 };
    memcpy(in, data.data(), data.size());
    uint16_t crc = crc12(in, 80);
    for (size_t i = 0; i < 12; i++)
        writeBit(in, 80 + i, (crc & (0x800 >> i)) != 0);

    uint8_t coded[24];
    convEncode(in, 96, coded);

    size_t n = 0;
    for (size_t i = 0; i < 192; i++) {
        if ((i % 4) == 1)
            continue;
        writeBit(frame.data(), facch1Position(offset, n), readBit(coded, i));
        n++;
    }
}
