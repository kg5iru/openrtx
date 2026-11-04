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

#include "protocols/YSF/Constants.hpp"
#include "protocols/YSF/Demodulator.hpp"

using namespace YSF;

/*
 * Reference C4FM transmitter, independent of the demodulator: random frames
 * starting with the sync word, dibits to +-1/+-3, impulses at 24 kHz shaped
 * by a root raised cosine (roll-off 0.2) computed here from its formula.
 */
static std::vector<float> rrcTaps(double alpha, int sps, int span)
{
    std::vector<float> h;
    double sum = 0.0;
    for (int i = -span * sps / 2; i <= span * sps / 2; i++) {
        double t = static_cast<double>(i) / sps, v;
        if (std::fabs(t) < 1e-9)
            v = 1.0 - alpha + 4.0 * alpha / M_PI;
        else if (std::fabs(std::fabs(t) - 1.0 / (4.0 * alpha)) < 1e-9)
            v = alpha / std::sqrt(2.0)
              * ((1.0 + 2.0 / M_PI) * std::sin(M_PI / (4.0 * alpha))
                 + (1.0 - 2.0 / M_PI) * std::cos(M_PI / (4.0 * alpha)));
        else
            v = (std::sin(M_PI * t * (1.0 - alpha))
                 + 4.0 * alpha * t * std::cos(M_PI * t * (1.0 + alpha)))
              / (M_PI * t * (1.0 - std::pow(4.0 * alpha * t, 2.0)));
        h.push_back(static_cast<float>(v));
        sum += v;
    }
    for (auto &v : h)
        v /= sum;
    return h;
}

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
                             double outer = 3000.0, double noise = 0.0,
                             double dc = 0.0, bool invert = false,
                             size_t skipSamples = 0, double clockPpm = 0.0)
{
    Transmission tx;
    auto taps = rrcTaps(0.2, 5, 16);
    std::normal_distribution<double> gauss(0.0, 1.0);

    std::vector<float> impulses;
    // 200 ms of carrier-only lead in (zero deviation)
    impulses.resize(4800, 0.0f);
    for (size_t f = 0; f < nFrames; f++) {
        frame_t fr;
        for (auto &b : fr)
            b = rng();
        memcpy(fr.data(), SYNC_WORD.data(), SYNC_BYTES);
        tx.frames.push_back(fr);
        for (size_t s = 0; s < FRAME_SYMBOLS; s++) {
            impulses.push_back(static_cast<float>(symbolOf(fr, s)));
            for (int k = 1; k < 5; k++)
                impulses.push_back(0.0f);
        }
    }
    impulses.resize(impulses.size() + 4800, 0.0f);

    // Pulse shaping; the gain makes the outer symbol peak at 'outer'
    std::vector<double> shaped(impulses.size(), 0.0);
    for (size_t n = 0; n < impulses.size(); n++) {
        if (impulses[n] == 0.0f)
            continue;
        for (size_t k = 0; k < taps.size(); k++) {
            size_t idx = n + k;
            if (idx < shaped.size())
                shaped[idx] += impulses[n] * taps[k];
        }
    }
    double peak = 3.0 * taps[taps.size() / 2];

    // Optional clock error: resample by linear interpolation
    double step = 1.0 + clockPpm * 1e-6;
    for (double t = skipSamples; t < shaped.size() - 1; t += step) {
        size_t i = static_cast<size_t>(t);
        double fr = t - i;
        double v = shaped[i] * (1.0 - fr) + shaped[i + 1] * fr;
        v = v * (outer / peak);
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
    size_t spurious;  ///< Frames matching none (after the signal ends)
};

// Match each demodulated frame against the transmitted one with the same
// payload start (frames are random, so a frame is found by its content)
static RxResult receive(const Transmission &tx)
{
    Demodulator demod;
    RxResult r = { 0, 0, 0, 0, 0 };

    for (auto s : tx.samples) {
        if (demod.sample(s) == false)
            continue;
        const frame_t &f = demod.getFrame();

        size_t best = 9999;
        for (const auto &t : tx.frames) {
            size_t e = 0;
            for (size_t i = 0; i < FRAME_BYTES; i++)
                e += __builtin_popcount(f[i] ^ t[i]);
            if (e < best)
                best = e;
        }
        // A frame far from all of them (a quarter of its bits or more) is
        // demodulated from the silence after the transmission
        if (best >= (FRAME_BYTES * 8) / 4) {
            r.spurious++;
            continue;
        }
        r.frames++;
        if (best == 0)
            r.exact++;
        r.bitErrors += best;
        r.bits += FRAME_BYTES * 8;
    }

    return r;
}

TEST_CASE("YSF demodulator: clean signal, every frame bit-exact", "[ysf]")
{
    std::mt19937 rng(10);
    auto tx = transmit(20, rng);
    auto r = receive(tx);
    REQUIRE(r.frames >= 19);
    REQUIRE(r.exact == r.frames);
    REQUIRE(r.spurious <= 4);
}

TEST_CASE("YSF demodulator: inverted signal", "[ysf]")
{
    std::mt19937 rng(11);
    auto tx = transmit(20, rng, 3000.0, 0.0, 0.0, true);
    auto r = receive(tx);
    REQUIRE(r.frames >= 19);
    REQUIRE(r.exact >= 19);
}

TEST_CASE("YSF demodulator: late entry locks on the next sync", "[ysf]")
{
    std::mt19937 rng(12);
    // Start about one frame into the transmission (after the 200 ms lead in)
    auto tx = transmit(20, rng, 3000.0, 0.0, 0.0, false, 4800 + 2300 * 1, 0.0);
    auto r = receive(tx);
    REQUIRE(r.frames >= 18);
    REQUIRE(r.exact >= 18);
}

TEST_CASE("YSF demodulator: DC offset, clock error and lower level", "[ysf]")
{
    std::mt19937 rng(13);
    auto tx = transmit(30, rng, 1500.0, 0.0, 400.0, false, 0, 100.0);
    auto r = receive(tx);
    REQUIRE(r.frames >= 28);
    REQUIRE(r.exact >= 28);
}

TEST_CASE("YSF demodulator: noise alone gives no frames", "[ysf]")
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
        if (demod.sample(static_cast<int16_t>(v)))
            frames++;
    }
    REQUIRE(frames == 0);
}

TEST_CASE("YSF demodulator: bit error rate in noise", "[ysf]")
{
    // Outer level 3000, noise rms added before the receive filter.  Measured
    // when written: BER 5e-4, 1.6e-3, 7.7e-3; the AMBE FEC copes below 1e-2.
    const double levels[] = { 300.0, 500.0, 700.0 };
    const double maxBer[] = { 2e-3, 5e-3, 2e-2 };
    for (int i = 0; i < 3; i++) {
        std::mt19937 rng(20 + i);
        auto tx = transmit(40, rng, 3000.0, levels[i], 200.0, false, 0, 50.0);
        auto r = receive(tx);
        double ber = (r.bits > 0) ? double(r.bitErrors) / r.bits : 1.0;
        INFO("noise " << levels[i] << ": frames " << r.frames << ", exact "
                      << r.exact << ", BER " << ber);
        CHECK(r.frames >= 38);
        CHECK(ber <= maxBer[i]);
    }
}
