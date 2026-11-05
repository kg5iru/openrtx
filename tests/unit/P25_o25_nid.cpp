/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/*
 * o25 Network Identifier: BCH(63,16) t=11 encode/decode, raw frame layout.
 */

#include <catch2/catch_test_macros.hpp>
#include <cstdint>
#include <cstring>
#include <vector>
#include "protocols/P25/modem/Nid.hpp"

using namespace P25;

static uint32_t lcg(uint32_t &s)
{
    s = 0x41C64E6D * s + 0x3039;
    return s >> 16;
}

static const uint8_t duids[] = { Nid::DUID_HDU,  Nid::DUID_TDU,  Nid::DUID_LDU1,
                                 Nid::DUID_TSDU, Nid::DUID_LDU2, Nid::DUID_PDU,
                                 Nid::DUID_TDULC };

TEST_CASE("NID encode/decode round trip", "[p25][o25][nid]")
{
    const uint16_t nacs[] = { 0x000, 0x125, 0x293, 0x5A5, 0xF7E, 0xFFF };
    for (uint16_t nac : nacs) {
        for (uint8_t duid : duids) {
            uint8_t nid[8];
            Nid::encode(nac, duid, nid);

            // Even parity over the 64 bits
            int ones = 0;
            for (int i = 0; i < 8; i++)
                ones += __builtin_popcount(nid[i]);
            CHECK((ones & 1) == 0);

            uint16_t rnac = 0xFFFF;
            uint8_t rduid = 0xFF;
            REQUIRE(Nid::decode(nid, rnac, rduid) == true);
            CHECK(rnac == nac);
            CHECK(rduid == duid);
        }
    }
}

TEST_CASE("NID codeword matches the reference BCH encoder", "[p25][o25][nid]")
{
    /*
     * Golden vectors from the G4KLX/MMDVM CBCH encoder (Morelos-Zaragoza
     * LFSR implementation), BCH codeword bits 0..62; the parity bit is
     * cleared in the vectors and ignored in the comparison.
     */
    struct Golden {
        uint16_t nac;
        uint8_t duid;
        uint8_t bytes[8];
    };
    static const Golden golden[] = {
        { 0x293, 0x5, { 0x29, 0x35, 0x55, 0xEF, 0x2C, 0x65, 0x34, 0x36 } },
        { 0x125, 0x0, { 0x12, 0x50, 0xE7, 0xA2, 0xED, 0xF9, 0x83, 0x56 } },
        { 0xFFF, 0xF, { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE } },
        { 0x000, 0x3, { 0x00, 0x03, 0x36, 0x4C, 0x2F, 0x74, 0xEC, 0xAC } },
        { 0x5A5, 0xA, { 0x5A, 0x5A, 0x12, 0xCC, 0x18, 0x1C, 0x4F, 0x36 } },
    };

    for (const auto &g : golden) {
        uint8_t nid[8];
        Nid::encode(g.nac, g.duid, nid);
        INFO("nac " << g.nac << " duid " << (int)g.duid);
        for (int i = 0; i < 7; i++)
            CHECK(nid[i] == g.bytes[i]);
        CHECK((nid[7] & 0xFE) == g.bytes[7]);
    }
}

TEST_CASE("NID corrects up to eleven bit errors", "[p25][o25][nid]")
{
    uint32_t rng = 0xC0FFEE;
    for (int trial = 0; trial < 400; trial++) {
        uint16_t nac = lcg(rng) & 0xFFF;
        uint8_t duid = duids[lcg(rng) % 7];
        int errors = 1 + (lcg(rng) % 11);

        uint8_t nid[8];
        Nid::encode(nac, duid, nid);

        // Distinct error positions within the 63-bit codeword
        std::vector<int> pos;
        while ((int)pos.size() < errors) {
            int p = lcg(rng) % 63;
            bool dup = false;
            for (int q : pos)
                dup |= (q == p);
            if (!dup)
                pos.push_back(p);
        }
        for (int p : pos)
            nid[p / 8] ^= 0x80 >> (p % 8);

        uint16_t rnac;
        uint8_t rduid;
        INFO("trial " << trial << ": nac " << nac << " duid " << (int)duid
                      << " errors " << errors);
        REQUIRE(Nid::decode(nid, rnac, rduid) == true);
        CHECK(rnac == nac);
        CHECK(rduid == duid);
    }
}

TEST_CASE("NID rejects random words", "[p25][o25][nid]")
{
    uint32_t rng = 0xBADC0DE;
    int accepted = 0;
    for (int trial = 0; trial < 2000; trial++) {
        uint8_t nid[8];
        for (int i = 0; i < 8; i++)
            nid[i] = lcg(rng) & 0xFF;
        uint16_t nac;
        uint8_t duid;
        if (Nid::decode(nid, nac, duid))
            accepted++;
    }
    // 2^16 codewords, radius-11 spheres in 2^63 words: ~0.1% false accepts
    CHECK(accepted < 20);
}

TEST_CASE("NID raw frame layout skips the status symbol", "[p25][o25][nid]")
{
    uint8_t nid[8];
    Nid::encode(0x293, Nid::DUID_LDU1, nid);

    // Raw frame: FS (bits 0..47), NID bits 48..69, status symbol 70..71,
    // NID bits 72..113.
    uint8_t frame[15];
    memset(frame, 0, sizeof(frame));
    Nid::toFrame(nid, frame);

    // Status symbol position must be untouched (left zero) and the NID bits
    // must straddle it.
    CHECK((frame[8] & 0x03) == 0);

    // Mark the status symbol with the "inbound channel busy" dibit (11):
    // extraction must ignore it.
    frame[8] |= 0x03;

    uint8_t back[8];
    Nid::fromFrame(frame, back);
    for (int i = 0; i < 8; i++)
        CHECK(back[i] == nid[i]);
}
