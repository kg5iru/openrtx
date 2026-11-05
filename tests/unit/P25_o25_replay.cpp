/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/*
 * Replay a 24 kHz s16le discriminator capture through the o25 demodulator
 * and report what it framed: data units by type, NACs seen, link control
 * (source, destination, LCF) from the LDU1s, and the IMBE FEC error count
 * per LDU, which measures symbol-decision quality on a real signal.
 *
 * Captures come from HackRF IQ via
 * ham-digital-modes/scripts/cs8_to_discriminator.py.
 *
 * usage: p25_o25_replay <baseband.s16> [--invert]
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <vector>
#include <map>
#include "protocols/P25/modem/Demodulator.hpp"
#include "protocols/P25/modem/Nid.hpp"
#include "protocols/P25/P25Data.h"
#include "protocols/P25/P25Audio.h"

using namespace P25;

static const char *duidName(uint8_t duid)
{
    switch (duid) {
        case Nid::DUID_HDU:
            return "HDU";
        case Nid::DUID_TDU:
            return "TDU";
        case Nid::DUID_LDU1:
            return "LDU1";
        case Nid::DUID_LDU2:
            return "LDU2";
        case Nid::DUID_TDULC:
            return "TDULC";
        default:
            return "?";
    }
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "usage: %s <baseband.s16> [--invert]\n", argv[0]);
        return 1;
    }

    bool invert = (argc > 2) && (strcmp(argv[2], "--invert") == 0);

    FILE *f = fopen(argv[1], "rb");
    if (f == NULL) {
        fprintf(stderr, "cannot open %s\n", argv[1]);
        return 1;
    }

    std::vector<int16_t> bb;
    int16_t buf[4096];
    size_t n;
    while ((n = fread(buf, sizeof(int16_t), 4096, f)) > 0)
        bb.insert(bb.end(), buf, buf + n);
    fclose(f);
    printf("%zu samples (%.2f s at 24 kHz), polarity %s\n", bb.size(),
           bb.size() / 24000.0, invert ? "inverted" : "normal");

    Demodulator dem;
    dem.init();
    CP25Data data;
    CP25Audio audio;

    std::map<uint8_t, unsigned> units;
    std::map<uint16_t, unsigned> nacs;
    unsigned ldus = 0;
    unsigned fecErrors = 0;
    unsigned worstLdu = 0;
    unsigned lcOk = 0;
    unsigned lcBad = 0;
    unsigned lastSrc = 0, lastDst = 0;
    unsigned lastLcf = 0;

    for (size_t i = 0; i < bb.size(); i++) {
        if (dem.sample(bb[i], invert) == false)
            continue;

        uint8_t duid = dem.getDuid();
        units[duid] += 1;
        nacs[dem.getNac()] += 1;

        const frame_t &frame = dem.getFrame();
        if ((duid == Nid::DUID_LDU1) || (duid == Nid::DUID_LDU2)) {
            uint8_t copy[216];
            memcpy(copy, frame.data(), 216);
            unsigned errs = audio.process(copy);
            fecErrors += errs;
            if (errs > worstLdu)
                worstLdu = errs;
            ldus += 1;

            if (duid == Nid::DUID_LDU1) {
                if (data.decodeLDU1(copy)) {
                    lcOk += 1;
                    lastSrc = data.getSrcId();
                    lastDst = data.getDstId();
                    lastLcf = data.getLCF();
                } else {
                    lcBad += 1;
                }
            }

            printf("  t=%7.3f s  %-5s nac $%03X  imbe fec errs %3u%s\n",
                   i / 24000.0, duidName(duid), dem.getNac(), errs,
                   (duid == Nid::DUID_LDU1) ?
                       (data.getSrcId() ? "  LC ok" : "  LC ?") :
                       "");
        } else {
            printf("  t=%7.3f s  %-5s nac $%03X\n", i / 24000.0, duidName(duid),
                   dem.getNac());
        }
    }

    printf("\nunits:");
    for (auto &u : units)
        printf(" %s x%u", duidName(u.first), u.second);
    printf("\nNACs:");
    for (auto &k : nacs)
        printf(" $%03X x%u", k.first, k.second);
    printf("\nLDUs %u, IMBE FEC errors total %u (%.2f per LDU, worst %u of "
           "%u bits)\n",
           ldus, fecErrors, ldus ? (double)fecErrors / ldus : 0.0, worstLdu,
           9 * 144);
    printf("LDU1 link control: %u decoded, %u failed; last src %u dst %u "
           "lcf 0x%02X\n",
           lcOk, lcBad, lastSrc, lastDst, lastLcf);

    return (ldus > 0) ? 0 : 2;
}
