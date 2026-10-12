/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <catch2/catch_test_macros.hpp>
#include <cstdint>
#include <cstring>
#include "protocols/DMR/Golay2087.hpp"
#include "protocols/DMR/QR1676.hpp"
#include "protocols/DMR/Hamming.hpp"

using namespace DMR;

TEST_CASE("Golay(20,8) encodes/decodes every data word without error", "[dmr][fec]")
{
    for (unsigned int v = 0; v < 256; v++)
    {
        uint8_t data[8], codeword[20], decoded[8];
        for (unsigned int i = 0; i < 8; i++)
            data[i] = (uint8_t)((v >> (7 - i)) & 1);

        Golay2087::encode(data, codeword);
        bool ok = Golay2087::decode(codeword, decoded);

        REQUIRE(ok);
        REQUIRE(memcmp(data, decoded, 8) == 0);
    }
}

TEST_CASE("Golay(20,8) corrects any single bit error", "[dmr][fec]")
{
    uint8_t data[8] = {1, 0, 1, 1, 0, 0, 1, 0};
    uint8_t codeword[20];
    Golay2087::encode(data, codeword);

    for (unsigned int bit = 0; bit < 20; bit++)
    {
        uint8_t corrupted[20];
        memcpy(corrupted, codeword, 20);
        corrupted[bit] ^= 1;

        uint8_t decoded[8];
        bool ok = Golay2087::decode(corrupted, decoded);

        INFO("bit flipped: " << bit);
        REQUIRE(ok);
        REQUIRE(memcmp(data, decoded, 8) == 0);
    }
}

TEST_CASE("QR(16,7,6) encodes/decodes every data word without error", "[dmr][fec]")
{
    for (unsigned int v = 0; v < 128; v++)
    {
        uint8_t data[7], codeword[16], decoded[7];
        for (unsigned int i = 0; i < 7; i++)
            data[i] = (uint8_t)((v >> (6 - i)) & 1);

        QR1676::encode(data, codeword);
        bool ok = QR1676::decode(codeword, decoded);

        REQUIRE(ok);
        REQUIRE(memcmp(data, decoded, 7) == 0);
    }
}

TEST_CASE("QR(16,7,6) corrects any single bit error", "[dmr][fec]")
{
    uint8_t data[7] = {1, 0, 1, 1, 0, 0, 1};
    uint8_t codeword[16];
    QR1676::encode(data, codeword);

    for (unsigned int bit = 0; bit < 16; bit++)
    {
        uint8_t corrupted[16];
        memcpy(corrupted, codeword, 16);
        corrupted[bit] ^= 1;

        uint8_t decoded[7];
        bool ok = QR1676::decode(corrupted, decoded);

        INFO("bit flipped: " << bit);
        REQUIRE(ok);
        REQUIRE(memcmp(data, decoded, 7) == 0);
    }
}

TEST_CASE("Hamming(15,11,3) corrects any single bit error", "[dmr][fec]")
{
    uint8_t data[11] = {1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0};
    uint8_t codeword[15];
    Hamming15113::encode(data, codeword);

    for (unsigned int bit = 0; bit < 15; bit++)
    {
        uint8_t corrupted[15];
        memcpy(corrupted, codeword, 15);
        corrupted[bit] ^= 1;

        uint8_t decoded[11];
        bool ok = Hamming15113::decode(corrupted, decoded);

        INFO("bit flipped: " << bit);
        REQUIRE(ok);
        REQUIRE(memcmp(data, decoded, 11) == 0);
    }
}

TEST_CASE("Hamming(13,9,3) corrects any single bit error", "[dmr][fec]")
{
    uint8_t data[9] = {1, 0, 1, 1, 0, 0, 1, 1, 0};
    uint8_t codeword[13];
    Hamming1393::encode(data, codeword);

    for (unsigned int bit = 0; bit < 13; bit++)
    {
        uint8_t corrupted[13];
        memcpy(corrupted, codeword, 13);
        corrupted[bit] ^= 1;

        uint8_t decoded[9];
        bool ok = Hamming1393::decode(corrupted, decoded);

        INFO("bit flipped: " << bit);
        REQUIRE(ok);
        REQUIRE(memcmp(data, decoded, 9) == 0);
    }
}
