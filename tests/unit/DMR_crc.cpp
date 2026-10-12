/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <catch2/catch_test_macros.hpp>
#include <cstdint>
#include <cstring>
#include "protocols/DMR/Crc.hpp"
#include "protocols/DMR/ReedSolomon1294.hpp"

using namespace DMR;

TEST_CASE("CRC16 on-air value is stable and mask-dependent", "[dmr][crc]")
{
    uint8_t bytes[10] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A};
    uint8_t bits[80];
    unpackBits(bytes, 10, bits);

    uint16_t csbk = crc16OnAir(bits, 80, CrcMask::Csbk);
    uint16_t hdr  = crc16OnAir(bits, 80, CrcMask::DataHeader);

    // same data, different mask => different on-air CRC, and each mask
    // is self-consistent (same input -> same output every time)
    REQUIRE(csbk != hdr);
    REQUIRE(crc16OnAir(bits, 80, CrcMask::Csbk) == csbk);
}

TEST_CASE("CRC16 on-air value detects single-byte corruption", "[dmr][crc]")
{
    uint8_t bytes[10] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A};
    uint8_t bits[80];
    unpackBits(bytes, 10, bits);
    uint16_t good = crc16OnAir(bits, 80, CrcMask::Csbk);

    bytes[4] ^= 0x01;
    unpackBits(bytes, 10, bits);
    uint16_t corrupted = crc16OnAir(bits, 80, CrcMask::Csbk);

    REQUIRE(good != corrupted);
}

TEST_CASE("Reed-Solomon(12,9,4) check() validates a block generate() produced", "[dmr][rs]")
{
    uint8_t data[9] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99};
    uint8_t parity[3];
    ReedSolomon1294::generate(data, CrcMask::VoiceLcHeader, parity);

    uint8_t block[12];
    memcpy(block, data, 9);
    memcpy(block + 9, parity, 3);

    REQUIRE(ReedSolomon1294::check(block, CrcMask::VoiceLcHeader));
    REQUIRE_FALSE(ReedSolomon1294::check(block, CrcMask::TerminatorWithLc));

    block[0] ^= 0xFF;
    REQUIRE_FALSE(ReedSolomon1294::check(block, CrcMask::VoiceLcHeader));
}
