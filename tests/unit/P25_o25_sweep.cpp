/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/*
 * o25 modem noise sweep (report tool, not a unit test): Modulator -> 2:1
 * decimation -> AWGN at 24 kHz -> Demodulator. Noise sigma is expressed
 * relative to the OUTER symbol level seen at the receiver input (the
 * steady +3 deviation), so results are comparable across pulse shapes
 * and against the MMDVM-lineage demodulator (clean to sigma/outer ~0.37,
 * cliff ~0.46 on the same metric, see references).
 *
 * Usage: p25_o25_sweep [timing_offset_48k_samples]
 *   timing offset 0 -> even 48 kHz samples, symbol centres between RX
 *   samples (worst case for a 5 sps sampler); 1 -> centres on-sample.
 */

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <vector>
#include <algorithm>
#include "protocols/P25/modem/Modulator.hpp"
#include "protocols/P25/modem/Demodulator.hpp"
#include "protocols/P25/modem/Constants.hpp"
#include "protocols/P25/modem/Nid.hpp"

using namespace P25;

static uint32_t lcg(uint32_t &s)
{
    s = 0x41C64E6D * s + 0x3039;
    return s >> 16;
}

static frame_t makeFrame(uint32_t &rng, uint32_t index)
{
    frame_t f;
    for (size_t i = 0; i < FS_SYNC_BYTES.size(); i++)
        f[i] = FS_SYNC_BYTES[i];
    rng = 0x41C64E6D * (rng ^ index) + 0x3039;
    for (size_t i = FS_SYNC_BYTES.size(); i < f.size(); i++)
        f[i] = static_cast<uint8_t>(lcg(rng));

    // Alternate LDU1/LDU2 NIDs: the demodulator frames on the DUID.
    uint8_t nid[Nid::BYTES];
    Nid::encode(0x293, (index & 1) ? Nid::DUID_LDU2 : Nid::DUID_LDU1, nid);
    Nid::toFrame(nid, f.data());
    return f;
}

int main(int argc, char **argv)
{
    setvbuf(stdout, NULL, _IONBF, 0);
    constexpr size_t NUM_FRAMES = 20;
    const size_t phase = (argc > 1) ? atoi(argv[1]) & 1 : 0;

    std::vector<frame_t> tx;
    uint32_t rng = 0xBEEF1234;
    for (size_t i = 0; i < NUM_FRAMES; i++)
        tx.push_back(makeFrame(rng, i));

    std::vector<int16_t> wire48k;
    Modulator mod;
    mod.init();
    mod.setSampleSink(
        [](const int16_t *s, size_t n, void *ctx) {
            auto &out = *static_cast<std::vector<int16_t> *>(ctx);
            out.insert(out.end(), s, s + n);
        },
        &wire48k);
    mod.start();
    mod.sendPreamble();
    for (const auto &f : tx)
        mod.sendFrame(f);
    mod.sendPreamble(); // drain filter delay
    mod.stop();

    std::vector<int16_t> clean;
    for (size_t i = phase; i < wire48k.size(); i += 2)
        clean.push_back(wire48k[i]);

    const double outer = Modulator::DEFAULT_OUTER_LEVEL;
    int16_t peak = 0;
    for (auto s : clean)
        peak = std::max(peak, static_cast<int16_t>(std::abs(s)));
    printf("wire: %zu samples @24k, peak %d, outer level %.0f, phase %zu\n",
           clean.size(), peak, outer, phase);

    printf("%8s %6s %8s %10s %10s\n", "sig/out", "seed", "frames", "byteErrs",
           "cleanRun");
    for (double k = 0.0; k <= 0.701; k += 0.05) {
        for (uint32_t seed = 1; seed <= 3; seed++) {
            std::vector<int16_t> noisy = clean;
            uint32_t rs = seed * 7919 + 13;
            double sigma = k * outer;
            for (size_t i = 0; i + 1 < noisy.size(); i += 2) {
                double u1 = (lcg(rs) + 0.5) / 65536.0;
                double u2 = (lcg(rs) + 0.5) / 65536.0;
                double r = std::sqrt(-2.0 * std::log(u1));
                double g1 = r * std::cos(2 * M_PI * u2) * sigma;
                double g2 = r * std::sin(2 * M_PI * u2) * sigma;
                auto sat = [](double v) {
                    return static_cast<int16_t>(
                        std::max(-32768.0, std::min(32767.0, std::round(v))));
                };
                noisy[i] = sat(noisy[i] + g1);
                noisy[i + 1] = sat(noisy[i + 1] + g2);
            }

            Demodulator dem;
            dem.init();
            std::vector<frame_t> rxf;
            for (size_t i = 0; i < noisy.size(); i++)
                if (dem.sample(noisy[i], false))
                    rxf.push_back(dem.getFrame());

            // Nearest-frame alignment: random 216-byte payloads make the
            // right match unambiguous (~2 vs ~210 byte errors).
            size_t byteErrs = 0, longest = 0, run = 0;
            for (const auto &rf : rxf) {
                size_t best = FRAME_BYTES + 1;
                for (const auto &tf : tx) {
                    size_t bad = 0;
                    for (size_t b = 0; b < FRAME_BYTES; b++)
                        bad += (rf[b] != tf[b]);
                    best = std::min(best, bad);
                }
                byteErrs += best;
                if (best == 0) {
                    run++;
                    longest = std::max(longest, run);
                } else {
                    run = 0;
                }
            }
            printf("%8.2f %6u %8zu %10zu %10zu\n", k, seed, rxf.size(),
                   byteErrs, longest);
        }
    }
    return 0;
}
