/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/*
 * o25 modem loopback: P25::Modulator (48 kHz) -> 2:1 decimation ->
 * P25::Demodulator (24 kHz), byte-exact frame compare. Both decimation
 * phases are exercised: with the TX FIR delay of 65 samples the symbol
 * centres land on odd 48 kHz indices, so the even phase presents the
 * receiver with a half-sample (10% of a symbol) timing offset - the worst
 * case for a 5 samples/symbol sampler - while the odd phase is on-centre.
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

static frame_t makeFrame(uint32_t &rng, uint32_t index)
{
    frame_t f;
    for (size_t i = 0; i < FS_SYNC_BYTES.size(); i++)
        f[i] = FS_SYNC_BYTES[i];

    // Deterministic payload: LCG bytes, seeded by frame index so every
    // frame's content is unique and reproducible.
    rng = 0x41C64E6D * (rng ^ index) + 0x3039;
    for (size_t i = FS_SYNC_BYTES.size(); i < f.size(); i++) {
        rng = 0x41C64E6D * rng + 0x3039;
        f[i] = static_cast<uint8_t>(rng >> 16);
    }

    // Alternate LDU1/LDU2 NIDs: the demodulator frames on the DUID.
    uint8_t nid[Nid::BYTES];
    Nid::encode(0x293, (index & 1) ? Nid::DUID_LDU2 : Nid::DUID_LDU1, nid);
    Nid::toFrame(nid, f.data());
    return f;
}

static std::vector<int16_t> transmit(const std::vector<frame_t> &frames,
                                     const int8_t *pattern = nullptr,
                                     size_t patternLen = 0,
                                     size_t symbolCount = 0)
{
    std::vector<int16_t> wire;
    Modulator mod;
    mod.init();
    mod.invertPhase(false);
    mod.setSampleSink(
        [](const int16_t *samples, size_t count, void *context) {
            auto &out = *static_cast<std::vector<int16_t> *>(context);
            out.insert(out.end(), samples, samples + count);
        },
        &wire);
    REQUIRE(mod.start());
    if (pattern != nullptr)
        mod.sendPreamble(pattern, patternLen, symbolCount);
    else
        mod.sendPreamble();
    for (const auto &frame : frames)
        mod.sendFrame(frame);
    mod.sendPreamble(); // Drain the TX/RX filter delay after the last frame.
    mod.stop();
    return wire;
}

static void runLoopback(const size_t decimationPhase)
{
    constexpr size_t NUM_FRAMES = 12;
    uint32_t rng = 0x12345678;

    std::vector<frame_t> tx;
    for (size_t i = 0; i < NUM_FRAMES; i++)
        tx.push_back(makeFrame(rng, i));

    auto wire48k = transmit(tx);
    REQUIRE(wire48k.size() == (NUM_FRAMES + 2) * FRAME_SYMBOLS * 10);

    std::vector<int16_t> rx24k;
    rx24k.reserve(wire48k.size() / 2);
    for (size_t i = decimationPhase; i < wire48k.size(); i += 2)
        rx24k.push_back(wire48k[i]);

    Demodulator dem;
    dem.init();
    std::vector<frame_t> rx;
    bool locked = false;
    for (size_t i = 0; i < rx24k.size(); i++) {
        if (dem.sample(rx24k[i], false))
            rx.push_back(dem.getFrame());
        locked = dem.isLocked() || locked;
    }

    INFO("decimation phase " << decimationPhase << ": recovered " << rx.size()
                             << " of " << tx.size() << " frames");
    REQUIRE(locked == true);

    // Every transmitted frame must come back byte-exact (the preamble is
    // long enough for acquisition before the first frame).
    REQUIRE(rx.size() == tx.size());
    for (size_t i = 0; i < rx.size(); i++) {
        size_t bad = 0, first = 0;
        for (size_t b = 0; b < FRAME_BYTES; b++) {
            if (rx[i][b] != tx[i][b]) {
                if (bad == 0)
                    first = b;
                bad++;
            }
        }
        INFO("frame " << i << ": " << bad << " byte errors, first at byte "
                      << first);
        CHECK(bad == 0);
    }
}

static void runPreambleLoopback(const int8_t *pattern, size_t patternLen,
                                size_t symbolCount)
{
    constexpr size_t NUM_FRAMES = 4;
    uint32_t rng = 0x87654321;

    std::vector<frame_t> tx;
    for (size_t i = 0; i < NUM_FRAMES; i++)
        tx.push_back(makeFrame(rng, i));

    // The demodulator acquires on the frame sync, never on the preamble:
    // every option must decode, none may disturb the following frames.
    auto wire48k = transmit(tx, pattern, patternLen, symbolCount);

    std::vector<int16_t> rx24k;
    for (size_t i = 0; i < wire48k.size(); i += 2)
        rx24k.push_back(wire48k[i]);

    Demodulator dem;
    dem.init();
    std::vector<frame_t> rx;
    for (size_t i = 0; i < rx24k.size(); i++)
        if (dem.sample(rx24k[i], false))
            rx.push_back(dem.getFrame());

    REQUIRE(rx.size() == tx.size());
    for (size_t i = 0; i < rx.size(); i++)
        for (size_t b = 0; b < FRAME_BYTES; b++)
            if (rx[i][b] != tx[i][b]) {
                INFO("frame " << i << " byte " << b);
                REQUIRE(false);
            }
}

TEST_CASE("o25 modem loopback with Kenwood and sweep preambles", "[p25][o25]")
{
    runPreambleLoopback(Modulator::PREAMBLE_KENWOOD, 4, 255 * 4);
    runPreambleLoopback(Modulator::PREAMBLE_SWEEP, 4, 144 * 4);
    runPreambleLoopback(Modulator::PREAMBLE_MMDVM, 2, 864);
}

TEST_CASE("o25 modem loopback, symbol centres half a sample off", "[p25][o25]")
{
    runLoopback(0);
}

TEST_CASE("o25 modem loopback, symbol centres on-sample", "[p25][o25]")
{
    runLoopback(1);
}
