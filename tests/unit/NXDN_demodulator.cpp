/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <random>
#include <vector>

#include "protocols/NXDN/Coding.hpp"
#include "protocols/NXDN/Constants.hpp"
#include "protocols/NXDN/DSP.hpp"
#include "protocols/NXDN/Demodulator.hpp"

using namespace NXDN;

/*
 * Transmitter as the NXDN modulator works: random frames starting with the
 * sync word, dibits to +-1/+-3, shaped at 48 kHz by the transmit taps
 * (polyphase, 20 samples per symbol), then taken to 24 kHz by keeping every
 * other sample (the signal is below 1.5 kHz).
 */
struct Transmission {
    std::vector<frame_t> frames;
    std::vector<int16_t> samples;
};

static int8_t symbolOf(const frame_t &f, size_t i)
{
    uint8_t d = (f[i / 4] >> (6 - 2 * (i % 4))) & 3;
    switch (d) {
        case 1:
            return +3;
        case 0:
            return +1;
        case 2:
            return -1;
        default:
            return -3;
    }
}

static Transmission transmit(size_t nFrames, std::mt19937 &rng,
                             double outer = 1200.0, double noise = 0.0,
                             double dc = 0.0, bool invert = false,
                             size_t skipSamples = 0, double clockPpm = 0.0)
{
    Transmission tx;
    std::normal_distribution<double> gauss(0.0, 1.0);

    // 200 ms of carrier-only lead in, the frames, 200 ms of carrier
    std::vector<int8_t> syms(480, 0);
    for (size_t f = 0; f < nFrames; f++) {
        frame_t fr;
        for (auto &b : fr)
            b = rng();
        encodeSync(fr);
        tx.frames.push_back(fr);
        for (size_t s = 0; s < FRAME_SYMBOLS; s++)
            syms.push_back(symbolOf(fr, s));
    }
    syms.resize(syms.size() + 480, 0);

    std::vector<double> shaped;
    for (size_t i = 0; i < syms.size(); i++) {
        for (size_t p = 0; p < 20; p += 2) {
            double acc = 0.0;
            for (size_t k = 0; (k < 12) && (k <= i); k++)
                acc += syms[i - k] * tx_taps_48k[(20 * k) + p];
            shaped.push_back(acc / 3.0);
        }
    }

    double step = 1.0 + clockPpm * 1e-6;
    for (double t = skipSamples; t < shaped.size() - 1; t += step) {
        size_t i = static_cast<size_t>(t);
        double fr = t - i;
        double v = shaped[i] * (1.0 - fr) + shaped[i + 1] * fr;
        v = v * outer;
        if (invert)
            v = -v;
        v += dc + noise * gauss(rng);
        if (v > 32000)
            v = 32000;
        if (v < -32000)
            v = -32000;
        tx.samples.push_back(static_cast<int16_t>(v));
    }

    return tx;
}

struct RxResult {
    size_t frames;    ///< Frames matched to a transmitted one
    size_t exact;
    size_t bitErrors; ///< Over the matched frames
    size_t bits;
    size_t spurious;  ///< Frames matching none
    uint32_t realigns;
};

static size_t bestMatch(const frame_t &f, const Transmission &tx)
{
    size_t best = 9999;
    for (const auto &t : tx.frames) {
        size_t e = 0;
        for (size_t i = 0; i < FRAME_BYTES; i++)
            e += __builtin_popcount(f[i] ^ t[i]);
        if (e < best)
            best = e;
    }
    return best;
}

static RxResult receive(const Transmission &tx)
{
    Demodulator demod;
    RxResult r = { 0, 0, 0, 0, 0, 0 };

    for (auto s : tx.samples) {
        if (demod.sample(s) == false)
            continue;
        size_t best = bestMatch(demod.getFrame(), tx);
        if (best >= FRAME_BITS / 4) {
            r.spurious++;
            continue;
        }
        r.frames++;
        if (best == 0)
            r.exact++;
        r.bitErrors += best;
        r.bits += FRAME_BITS;
    }

    r.realigns = demod.realignCount();
    return r;
}

TEST_CASE("NXDN demodulator: clean signal, every frame bit-exact", "[nxdn]")
{
    std::mt19937 rng(10);
    auto tx = transmit(20, rng);
    auto r = receive(tx);
    REQUIRE(r.frames >= 19);
    // The first frame of a lock may have a bit error, while the symbol
    // timing settles
    REQUIRE(r.exact >= r.frames - 1);
    REQUIRE(r.spurious <= 4);
    REQUIRE(r.realigns == 0);
}

TEST_CASE("NXDN demodulator: inverted signal", "[nxdn]")
{
    std::mt19937 rng(11);
    auto tx = transmit(20, rng, 1200.0, 0.0, 0.0, true);
    auto r = receive(tx);
    REQUIRE(r.frames >= 19);
    REQUIRE(r.exact >= 19);
}

TEST_CASE("NXDN demodulator: late entry locks on the next sync", "[nxdn]")
{
    std::mt19937 rng(12);
    // Start about one frame and a half into the transmission
    auto tx = transmit(20, rng, 1200.0, 0.0, 0.0, false, 4800 + 2900, 0.0);
    auto r = receive(tx);
    REQUIRE(r.frames >= 18);
    REQUIRE(r.exact >= 18);
}

TEST_CASE("NXDN demodulator: DC offset, clock error and lower level", "[nxdn]")
{
    std::mt19937 rng(13);
    auto tx = transmit(30, rng, 600.0, 0.0, 400.0, false, 0, 100.0);
    auto r = receive(tx);
    REQUIRE(r.frames >= 29);
    REQUIRE(r.exact >= 29);
}

TEST_CASE("NXDN demodulator: noise alone gives few frames", "[nxdn]")
{
    std::mt19937 rng(14);
    std::normal_distribution<double> gauss(0.0, 1500.0);
    Demodulator demod;
    size_t frames = 0;
    for (size_t i = 0; i < 24000 * 20; i++) {
        double v = gauss(rng);
        if (v > 32000)
            v = 32000;
        if (v < -32000)
            v = -32000;
        if (demod.sample(static_cast<int16_t>(v))) {
            demod.getFrame();
            frames++;
        }
    }
    /*
     * The sync word is short (20 bits): noise forms one now and then (about
     * every 4 s when written), and the lock gives two frames before it sees
     * the next sync is missing.  The caller drops them on their LICH and
     * SACCH.
     */
    REQUIRE(frames <= 20);
}

TEST_CASE("NXDN demodulator: bit error rate in noise", "[nxdn]")
{
    // Outer level 1200, noise rms added before the receive filter
    const double levels[] = { 150.0, 250.0, 350.0 };
    const double maxBer[] = { 2e-3, 5e-3, 2e-2 };
    for (int i = 0; i < 3; i++) {
        std::mt19937 rng(20 + i);
        auto tx = transmit(60, rng, 1200.0, levels[i], 200.0, false, 0, 50.0);
        auto r = receive(tx);
        double ber = (r.bits > 0) ? double(r.bitErrors) / r.bits : 1.0;
        INFO("noise " << levels[i] << ": frames " << r.frames << ", exact "
                      << r.exact << ", BER " << ber << ", realigns "
                      << r.realigns);
        CHECK(r.frames >= 58);
        CHECK(ber <= maxBer[i]);
        CHECK(r.realigns == 0); // no false sync words found in the data
    }
}

TEST_CASE("NXDN demodulator: long call, no false realignment", "[nxdn]")
{
    std::mt19937 rng(40);
    auto tx = transmit(400, rng, 1200.0, 100.0, 0.0, false, 0, 30.0);
    auto r = receive(tx);
    INFO("frames " << r.frames << ", exact " << r.exact);
    REQUIRE(r.frames >= 398);
    REQUIRE(r.realigns == 0);
}

TEST_CASE("NXDN demodulator: realigns when the frame timing jumps", "[nxdn]")
{
    /*
     * A hotspot whose network stream runs dry restarts its frames: model it
     * by cutting 900 samples (about half a frame) out of the middle of
     * frame 10.  The lock misses the next sync, then moves to the new
     * frame boundary.
     */
    std::mt19937 rng(30);
    auto tx = transmit(30, rng);
    size_t cut = 4800 + (10 * FRAME_SYMBOLS * 10) + 1000;
    tx.samples.erase(tx.samples.begin() + cut, tx.samples.begin() + cut + 900);

    Demodulator demod;
    size_t frames = 0, exact = 0;
    for (auto s : tx.samples) {
        if (!demod.sample(s))
            continue;
        size_t best = bestMatch(demod.getFrame(), tx);
        if (best < FRAME_BITS / 4)
            frames++;
        if (best == 0)
            exact++;
    }

    INFO("frames " << frames << ", exact " << exact << ", realigns "
                   << demod.realignCount());
    REQUIRE(demod.realignCount() >= 1);
    REQUIRE(exact >= 27);
}
