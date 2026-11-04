/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <cstring>
#include "protocols/M17/Golay.hpp"
#include "protocols/YSF/Coding.hpp"
#include "protocols/YSF/FrameDecoder.hpp"

using namespace YSF;

/*
 * Bit positions, in the 72 bits of a DMR AMBE+2 frame, of fields A (24 bits,
 * Golay(24,12)), B (23 bits, Golay(23,12), scrambled) and C (25 bits).  As in
 * MMDVMHost's AMBEFEC.cpp (DMR_A/B/C_TABLE).
 */
static constexpr uint8_t DMR_A[24] = { 0,  4,  8,  12, 16, 20, 24, 28,
                                       32, 36, 40, 44, 48, 52, 56, 60,
                                       64, 68, 1,  5,  9,  13, 17, 21 };

static constexpr uint8_t DMR_B[23] = { 25, 29, 33, 37, 41, 45, 49, 53,
                                       57, 61, 65, 69, 2,  6,  10, 14,
                                       18, 22, 26, 30, 34, 38, 42 };

static constexpr uint8_t DMR_C[25] = { 46, 50, 54, 58, 62, 66, 70, 3,  7,
                                       11, 15, 19, 23, 27, 31, 35, 39, 43,
                                       47, 51, 55, 59, 63, 67, 71 };

bool FrameDecoder::decodeFich(const frame_t &frame, Fich &fich)
{
    const uint8_t *chan = frame.data() + SYNC_BYTES;

    std::array<uint8_t, FICH_BYTES> coded;
    coded.fill(0);
    for (size_t i = 0; i < 100; i++) {
        size_t n = dibitPosition(i, 5);
        writeBit(coded.data(), 2 * i, readBit(chan, n));
        writeBit(coded.data(), 2 * i + 1, readBit(chan, n + 1));
    }

    std::array<uint8_t, 12> dec;
    viterbi.decode(coded, dec);

    uint16_t w[4];
    for (size_t k = 0; k < 4; k++) {
        uint32_t code = (static_cast<uint32_t>(dec[3 * k]) << 16)
                      | (static_cast<uint32_t>(dec[3 * k + 1]) << 8)
                      | static_cast<uint32_t>(dec[3 * k + 2]);
        w[k] = M17::golay24_decode(code);
        if (w[k] == 0xFFFF)
            return false;
    }

    fich.raw[0] = (w[0] >> 4) & 0xFF;
    fich.raw[1] = ((w[0] << 4) & 0xF0) | ((w[1] >> 8) & 0x0F);
    fich.raw[2] = w[1] & 0xFF;
    fich.raw[3] = (w[2] >> 4) & 0xFF;
    fich.raw[4] = ((w[2] << 4) & 0xF0) | ((w[3] >> 8) & 0x0F);
    fich.raw[5] = w[3] & 0xFF;

    return checkCrc16(fich.raw.data(), 4);
}

/*
 * The data channels are spread over the five 144-bit payload blocks: in each
 * block, segBytes bytes starting segOffset bytes in.  They are gathered,
 * deinterleaved, Viterbi decoded, CRC checked and unwhitened.
 */
template <size_t CODED_BYTES, size_t OUT_BYTES>
bool FrameDecoder::decodeDataChannel(const frame_t &frame,
                                     const size_t segBytes,
                                     const size_t segOffset, const size_t cols,
                                     std::array<uint8_t, OUT_BYTES> &out)
{
    const uint8_t *payload = frame.data() + PAYLOAD_OFFSET;

    uint8_t gathered[CODED_BYTES];
    for (size_t s = 0; s < 5; s++)
        memcpy(&gathered[s * segBytes], &payload[(s * 18) + segOffset],
               segBytes);

    std::array<uint8_t, CODED_BYTES> coded;
    coded.fill(0);
    for (size_t i = 0; i < (CODED_BYTES * 4); i++) {
        size_t n = dibitPosition(i, cols);
        writeBit(coded.data(), 2 * i, readBit(gathered, n));
        writeBit(coded.data(), 2 * i + 1, readBit(gathered, n + 1));
    }

    viterbi.decode(coded, out);

    if (checkCrc16(out.data(), OUT_BYTES - 2) == false)
        return false;

    for (size_t i = 0; i < (OUT_BYTES - 2); i++)
        out[i] ^= WHITENING[i];

    return true;
}

bool FrameDecoder::decodeCsd(const frame_t &frame, const uint8_t csd,
                             std::array<uint8_t, 20> &out)
{
    std::array<uint8_t, 22> dec;
    bool ok = decodeDataChannel<45, 22>(frame, 9, (csd == 2) ? 9 : 0, 9, dec);
    memcpy(out.data(), dec.data(), out.size());
    return ok;
}

bool FrameDecoder::decodeVd2Dch(const frame_t &frame,
                                std::array<uint8_t, 10> &out)
{
    std::array<uint8_t, 12> dec;
    bool ok = decodeDataChannel<25, 12>(frame, 5, 0, 5, dec);
    memcpy(out.data(), dec.data(), out.size());
    return ok;
}

bool FrameDecoder::decodeVd1Dch(const frame_t &frame,
                                std::array<uint8_t, 20> &out)
{
    std::array<uint8_t, 22> dec;
    bool ok = decodeDataChannel<45, 22>(frame, 9, 0, 9, dec);
    memcpy(out.data(), dec.data(), out.size());
    return ok;
}

uint32_t YSF::ambePrng(const uint32_t a)
{
    uint32_t p = 16 * (a & 0xFFF);
    uint32_t out = 0;

    for (int i = 0; i < 23; i++) {
        p = ((173 * p) + 13849) & 0xFFFF;
        out = (out << 1) | (p >> 15);
    }

    return out;
}

void YSF::ambeFieldsToDmr(const uint32_t a, const uint32_t b, const uint32_t c,
                          uint8_t ambe[9])
{
    uint32_t fa = M17::golay24_encode(a & 0xFFF);
    uint32_t fb = (M17::golay24_encode(b & 0xFFF) >> 1) ^ ambePrng(a);

    memset(ambe, 0, 9);
    for (size_t i = 0; i < 24; i++)
        writeBit(ambe, DMR_A[i], (fa >> (23 - i)) & 1);
    for (size_t i = 0; i < 23; i++)
        writeBit(ambe, DMR_B[i], (fb >> (22 - i)) & 1);
    for (size_t i = 0; i < 25; i++)
        writeBit(ambe, DMR_C[i], (c >> (24 - i)) & 1);
}

bool YSF::dmrToAmbeFields(const uint8_t ambe[9], uint32_t &a, uint32_t &b,
                          uint32_t &c)
{
    uint32_t fa = 0, fb = 0;
    c = 0;
    for (size_t i = 0; i < 24; i++)
        fa = (fa << 1) | (readBit(ambe, DMR_A[i]) ? 1 : 0);
    for (size_t i = 0; i < 23; i++)
        fb = (fb << 1) | (readBit(ambe, DMR_B[i]) ? 1 : 0);
    for (size_t i = 0; i < 25; i++)
        c = (c << 1) | (readBit(ambe, DMR_C[i]) ? 1 : 0);

    uint16_t da = M17::golay24_decode(fa);
    if (da == 0xFFFF)
        return false;
    a = da;

    // B has no parity bit: try both, keep the decoding with fewer changes
    fb ^= ambePrng(a);
    uint32_t best = 0;
    int bestErr = 99;
    for (uint32_t par = 0; par < 2; par++) {
        uint32_t cw = (fb << 1) | par;
        uint16_t d = M17::golay24_decode(cw);
        if (d == 0xFFFF)
            continue;
        int err = __builtin_popcount((M17::golay24_encode(d) >> 1) ^ fb);
        if (err < bestErr) {
            bestErr = err;
            best = d;
        }
    }
    b = (bestErr == 99) ? (fb >> 11) : best;

    return true;
}

uint8_t YSF::vd2VoiceToDmr(const frame_t &frame, const size_t n,
                           uint8_t ambe[9])
{
    const uint8_t *payload = frame.data() + PAYLOAD_OFFSET;
    const size_t offset = (144 * n) + 40; // after the 40-bit DCH segment

    uint8_t vch[13] = { 0 };
    for (size_t i = 0; i < 104; i++)
        writeBit(vch, i, readBit(payload, offset + vchPosition(i)));

    for (size_t i = 0; i < 13; i++)
        vch[i] ^= WHITENING[i];

    // 27 bits sent three times each, majority vote
    uint32_t voted = 0;
    uint8_t fixed = 0;
    for (size_t i = 0; i < 27; i++) {
        int ones = (readBit(vch, 3 * i) ? 1 : 0)
                 + (readBit(vch, 3 * i + 1) ? 1 : 0)
                 + (readBit(vch, 3 * i + 2) ? 1 : 0);
        if ((ones == 1) || (ones == 2))
            fixed++;
        voted = (voted << 1) | ((ones >= 2) ? 1 : 0);
    }

    uint32_t a = (voted >> 15) & 0xFFF;
    uint32_t b = (voted >> 3) & 0xFFF;
    uint32_t c = voted & 0x07;
    for (size_t i = 0; i < 22; i++)
        c = (c << 1) | (readBit(vch, 81 + i) ? 1 : 0);

    ambeFieldsToDmr(a, b, c, ambe);
    return fixed;
}

void YSF::vd1VoiceToDmr(const frame_t &frame, const size_t n, uint8_t ambe[9])
{
    const uint8_t *payload = frame.data() + PAYLOAD_OFFSET;
    memcpy(ambe, &payload[(18 * n) + 9], 9);
}

void YSF::callsignToString(const uint8_t *in, char out[11])
{
    size_t len = 0;
    for (size_t i = 0; i < CALLSIGN_LENGTH; i++) {
        char ch = static_cast<char>(in[i]);
        out[i] = ((ch >= 0x20) && (ch < 0x7F)) ? ch : ' ';
        if (out[i] != ' ')
            len = i + 1;
    }
    out[len] = '\0';
}
