/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <catch2/catch_test_macros.hpp>
#include <cstdint>
#include <cstring>
#include <random>
#include "protocols/DMR/Bptc19696.hpp"

using namespace DMR;
using namespace std;

static default_random_engine rng;

TEST_CASE("BPTC(196,96) round-trips random data with no errors", "[dmr][bptc]")
{
    uniform_int_distribution<int> bit(0, 1);

    for (int trial = 0; trial < 200; trial++)
    {
        uint8_t info[96], onAir[196], decoded[96];
        for (auto &b : info)
            b = (uint8_t) bit(rng);

        Bptc19696::encode(info, onAir);
        bool ok = Bptc19696::decode(onAir, decoded);

        REQUIRE(ok);
        REQUIRE(memcmp(info, decoded, 96) == 0);
    }
}

TEST_CASE("BPTC(196,96) corrects a single bit error at every position", "[dmr][bptc]")
{
    uint8_t info[96], onAir[196];
    for (unsigned int i = 0; i < 96; i++)
        info[i] = (uint8_t)(i % 3 == 0);

    Bptc19696::encode(info, onAir);

    for (unsigned int bit = 0; bit < 196; bit++)
    {
        uint8_t corrupted[196];
        memcpy(corrupted, onAir, 196);
        corrupted[bit] ^= 1;

        uint8_t decoded[96];
        bool ok = Bptc19696::decode(corrupted, decoded);

        INFO("bit flipped: " << bit);
        REQUIRE(ok);
        REQUIRE(memcmp(info, decoded, 96) == 0);
    }
}
