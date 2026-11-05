/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/*
 * o25 modem framing: a P25 transmission is a sequence of data units of
 * different lengths (HDU 99 bytes, LDU 216, TDU 18 ...), each starting with
 * the frame sync and a NID whose DUID gives the length. The demodulator
 * must deliver every unit with its own length, byte-exact, at both
 * decimation phases.
 */

#include <catch2/catch_test_macros.hpp>
#include <cstdio>
#include <cstring>
#include <vector>
#include "protocols/P25/modem/Modulator.hpp"
#include "protocols/P25/modem/Demodulator.hpp"
#include "protocols/P25/modem/Constants.hpp"
#include "protocols/P25/modem/Nid.hpp"

using namespace P25;

struct Unit {
    uint8_t duid;
    std::vector<uint8_t> bytes;
};

static Unit makeUnit(uint32_t &rng, const uint16_t nac, const uint8_t duid)
{
    Unit u;
    u.duid = duid;
    u.bytes.resize(Nid::frameBytes(duid));
    for (auto &b : u.bytes) {
        rng = 0x41C64E6D * rng + 0x3039;
        b = static_cast<uint8_t>(rng >> 16);
    }
    for (size_t i = 0; i < FS_SYNC_BYTES.size(); i++)
        u.bytes[i] = FS_SYNC_BYTES[i];
    uint8_t nid[8];
    Nid::encode(nac, duid, nid);
    Nid::toFrame(nid, u.bytes.data());
    return u;
}

static void runFraming(const size_t decimationPhase)
{
    constexpr uint16_t NAC = 0x293;
    uint32_t rng = 0x5EED1234;

    std::vector<Unit> tx;
    tx.push_back(makeUnit(rng, NAC, Nid::DUID_HDU));
    tx.push_back(makeUnit(rng, NAC, Nid::DUID_LDU1));
    tx.push_back(makeUnit(rng, NAC, Nid::DUID_LDU2));
    tx.push_back(makeUnit(rng, NAC, Nid::DUID_LDU1));
    tx.push_back(makeUnit(rng, NAC, Nid::DUID_LDU2));
    tx.push_back(makeUnit(rng, NAC, Nid::DUID_TDU));

    std::vector<int16_t> wire;
    Modulator mod;
    mod.init();
    mod.setSampleSink(
        [](const int16_t *samples, size_t count, void *context) {
            auto &out = *static_cast<std::vector<int16_t> *>(context);
            out.insert(out.end(), samples, samples + count);
        },
        &wire);
    REQUIRE(mod.start());
    mod.sendPreamble();
    for (const auto &u : tx)
        mod.sendFrame(u.bytes.data(), u.bytes.size());
    mod.sendPreamble(); // drain the filter delay
    mod.stop();

    Demodulator dem;
    dem.init();
    std::vector<Unit> rx;
    for (size_t i = decimationPhase; i < wire.size(); i += 2) {
        if (dem.sample(wire[i], false)) {
            Unit u;
            u.duid = dem.getDuid();
            CHECK(dem.getNac() == NAC);
            const frame_t &f = dem.getFrame();
            u.bytes.assign(f.begin(), f.begin() + dem.getFrameLength());
            rx.push_back(u);
        }
    }

    INFO("decimation phase " << decimationPhase << ": " << rx.size()
                             << " units received of " << tx.size());
    REQUIRE(rx.size() == tx.size());
    for (size_t i = 0; i < tx.size(); i++) {
        INFO("unit " << i << " duid " << (int)tx[i].duid);
        CHECK(rx[i].duid == tx[i].duid);
        REQUIRE(rx[i].bytes.size() == tx[i].bytes.size());
        size_t bad = 0;
        for (size_t b = 0; b < tx[i].bytes.size(); b++)
            bad += (rx[i].bytes[b] != tx[i].bytes[b]);
        CHECK(bad == 0);
    }
}

TEST_CASE("o25 framing: HDU, LDUs and TDU, centres on-sample",
          "[p25][o25][framing]")
{
    runFraming(1);
}

TEST_CASE("o25 framing: HDU, LDUs and TDU, centres half a sample off",
          "[p25][o25][framing]")
{
    runFraming(0);
}

TEST_CASE("o25 framing: a unit with an undecodable NID drops the lock, the "
          "next unit is caught",
          "[p25][o25][framing]")
{
    constexpr uint16_t NAC = 0x125;
    uint32_t rng = 0xABCDEF01;

    Unit good1 = makeUnit(rng, NAC, Nid::DUID_LDU1);
    Unit bad = makeUnit(rng, NAC, Nid::DUID_LDU2);
    Unit good2 = makeUnit(rng, NAC, Nid::DUID_LDU1);

    // Destroy the NID of the middle unit: 20 bit errors, beyond BCH reach.
    for (int i = 0; i < 20; i++) {
        unsigned bit = 48 + (i * 3) + ((i * 3) >= 22 ? 2 : 0);
        bad.bytes[bit / 8] ^= 0x80 >> (bit % 8);
    }

    std::vector<int16_t> wire;
    Modulator mod;
    mod.init();
    mod.setSampleSink(
        [](const int16_t *samples, size_t count, void *context) {
            auto &out = *static_cast<std::vector<int16_t> *>(context);
            out.insert(out.end(), samples, samples + count);
        },
        &wire);
    REQUIRE(mod.start());
    mod.sendPreamble();
    mod.sendFrame(good1.bytes.data(), good1.bytes.size());
    mod.sendFrame(bad.bytes.data(), bad.bytes.size());
    mod.sendFrame(good2.bytes.data(), good2.bytes.size());
    mod.sendPreamble();
    mod.stop();

    Demodulator dem;
    dem.init();
    std::vector<Unit> rx;
    for (size_t i = 1; i < wire.size(); i += 2) {
        if (dem.sample(wire[i], false)) {
            Unit u;
            u.duid = dem.getDuid();
            const frame_t &f = dem.getFrame();
            u.bytes.assign(f.begin(), f.begin() + dem.getFrameLength());
            rx.push_back(u);
        }
    }

    REQUIRE(rx.size() == 2);
    CHECK(rx[0].bytes == good1.bytes);
    CHECK(rx[1].bytes == good2.bytes);
}
