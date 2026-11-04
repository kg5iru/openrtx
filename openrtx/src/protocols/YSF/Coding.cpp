/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "protocols/YSF/Coding.hpp"

uint16_t YSF::crc16(const uint8_t *data, const size_t len)
{
    uint16_t crc = 0;

    for (size_t i = 0; i < len; i++) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int b = 0; b < 8; b++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }

    return ~crc;
}

bool YSF::checkCrc16(const uint8_t *data, const size_t len)
{
    uint16_t crc = crc16(data, len);
    return (data[len] == (crc >> 8)) && (data[len + 1] == (crc & 0xFF));
}

void YSF::addCrc16(uint8_t *data, const size_t len)
{
    uint16_t crc = crc16(data, len);
    data[len] = crc >> 8;
    data[len + 1] = crc & 0xFF;
}

void YSF::convEncode(const uint8_t *in, const size_t nBits, uint8_t *channel,
                     const size_t cols)
{
    uint8_t mem = 0; // bit 0 = newest input bit

    for (size_t i = 0; i < nBits; i++) {
        mem = ((mem << 1) | (readBit(in, i) ? 1 : 0)) & 0x1F;
        bool g1 = (__builtin_popcount(mem & 0x19) & 1) != 0;
        bool g2 = (__builtin_popcount(mem & 0x17) & 1) != 0;

        size_t pos = dibitPosition(i, cols);
        writeBit(channel, pos, g1);
        writeBit(channel, pos + 1, g2);
    }
}
