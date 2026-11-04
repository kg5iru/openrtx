/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cstdint>
#include <cstring>
#include <random>

#include "protocols/YSF/Constants.hpp"
#include "protocols/YSF/Coding.hpp"
#include "protocols/YSF/Fich.hpp"
#include "protocols/YSF/FrameDecoder.hpp"

using namespace YSF;

/*
 * Reference encoder, written independently of the decoder from the layouts
 * in MMDVMHost (YSFFICH.cpp, YSFPayload.cpp, AMBEFEC.cpp) and YSF2DMR
 * (ModeConv.cpp): table-free Golay by polynomial division, a shift register
 * convolutional encoder, explicit interleaver loops and a table-driven CRC.
 */
namespace ref
{

static void setBit(uint8_t *p, size_t i, bool b)
{
    if (b)
        p[i >> 3] |= (0x80 >> (i & 7));
    else
        p[i >> 3] &= ~(0x80 >> (i & 7));
}

static bool getBit(const uint8_t *p, size_t i)
{
    return (p[i >> 3] & (0x80 >> (i & 7))) != 0;
}

// Golay(23,12) by division with x^11+x^10+x^6+x^5+x^4+x^2+1
static uint32_t golay23(uint32_t data)
{
    uint32_t r = data << 11;
    for (int i = 22; i >= 11; i--)
        if (r & (1u << i))
            r ^= 0xC75u << (i - 11);
    return (data << 11) | r;
}

static uint32_t golay24(uint32_t data)
{
    uint32_t cw = golay23(data);
    return (cw << 1) | (__builtin_popcount(cw) & 1);
}

static uint16_t crcTable[256];

static void crcInit()
{
    for (int i = 0; i < 256; i++) {
        uint16_t c = i << 8;
        for (int b = 0; b < 8; b++)
            c = (c & 0x8000) ? ((c << 1) ^ 0x1021) : (c << 1);
        crcTable[i] = c;
    }
}

static void addCrc(uint8_t *d, size_t len) // len includes the 2 CRC bytes
{
    uint16_t crc = 0;
    for (size_t i = 0; i < len - 2; i++)
        crc = (crc << 8) ^ crcTable[(crc >> 8) ^ d[i]];
    crc = ~crc;
    d[len - 2] = crc >> 8;
    d[len - 1] = crc & 0xFF;
}

static void conv(const uint8_t *in, uint8_t *out, size_t nBits)
{
    uint8_t d1 = 0, d2 = 0, d3 = 0, d4 = 0;
    size_t k = 0;
    for (size_t i = 0; i < nBits; i++) {
        uint8_t d = getBit(in, i) ? 1 : 0;
        uint8_t g1 = (d + d3 + d4) & 1;
        uint8_t g2 = (d + d1 + d2 + d4) & 1;
        d4 = d3;
        d3 = d2;
        d2 = d1;
        d1 = d;
        setBit(out, k++, g1);
        setBit(out, k++, g2);
    }
}

// Dibit interleave: rows of 'cols' dibits, 20 rows
static void interleave(const uint8_t *conv, uint8_t *dst, size_t nDibits,
                       size_t cols)
{
    size_t j = 0;
    for (size_t row = 0; row < nDibits / cols; row++) {
        for (size_t col = 0; col < cols; col++) {
            size_t n = (row * 2) + (col * 40);
            setBit(dst, n, getBit(conv, j++));
            setBit(dst, n + 1, getBit(conv, j++));
        }
    }
}

static void encodeFich(const uint8_t raw4[4], uint8_t *frame)
{
    uint8_t f[6];
    memcpy(f, raw4, 4);
    addCrc(f, 6);

    uint32_t b0 = (f[0] << 4) | (f[1] >> 4);
    uint32_t b1 = ((f[1] & 0x0F) << 8) | f[2];
    uint32_t b2 = (f[3] << 4) | (f[4] >> 4);
    uint32_t b3 = ((f[4] & 0x0F) << 8) | f[5];

    uint8_t data[13] = { 0 };
    uint32_t c[4] = { golay24(b0), golay24(b1), golay24(b2), golay24(b3) };
    for (int k = 0; k < 4; k++) {
        data[3 * k] = c[k] >> 16;
        data[3 * k + 1] = c[k] >> 8;
        data[3 * k + 2] = c[k];
    }

    uint8_t convolved[25] = { 0 };
    conv(data, convolved, 100);
    interleave(convolved, frame + SYNC_BYTES, 100, 5);
}

// Whitened data + CRC, convolved, interleaved and scattered over the payload
static void encodeDataChannel(const uint8_t *text, size_t textLen,
                              uint8_t *frame, size_t segBytes, size_t segOffset)
{
    uint8_t buf[23] = { 0 };
    for (size_t i = 0; i < textLen; i++)
        buf[i] = text[i] ^ WHITENING[i];
    addCrc(buf, textLen + 2);

    size_t nBits = (textLen + 2) * 8 + 4; // + 4 tail bits
    size_t nDibits = nBits;
    uint8_t convolved[45] = { 0 };
    conv(buf, convolved, nBits);

    uint8_t gathered[45] = { 0 };
    interleave(convolved, gathered, nDibits, nDibits / 20);

    uint8_t *payload = frame + PAYLOAD_OFFSET;
    for (size_t s = 0; s < 5; s++)
        memcpy(&payload[(s * 18) + segOffset], &gathered[s * segBytes],
               segBytes);
}

static uint32_t prngTable(uint32_t a)
{
    // From YSF2DMR ModeConv.cpp PRNG_TABLE (first 48 entries), >> 1
    static const uint32_t T[48] = {
        0x42CC47, 0x19D6FE, 0x304729, 0x6B2CD0, 0x60BF47, 0x39650E, 0x7354F1,
        0xEACF60, 0x819C9F, 0xDE25CE, 0xD7B745, 0x8CC8B8, 0x8D592B, 0xF71257,
        0xBCA084, 0xA5B329, 0xEE6AFA, 0xF7D9A7, 0xBCC21C, 0x4712D9, 0x4F2922,
        0x14FA37, 0x5D43EC, 0x564115, 0x299A92, 0x20A9EB, 0x7B707D, 0x3BE3A4,
        0x20D95B, 0x6B085A, 0x5233A5, 0x99A474, 0xC0EDCB, 0xCB5F12, 0x918455,
        0xF897EC, 0xE32E3B, 0xAA7CC2, 0xB1E7C9, 0xFC561D, 0xA70DE6, 0x8DBE73,
        0xD4F608, 0x57658D, 0x0E5E56, 0x458DAB, 0x7E15B8, 0x376645
    };
    return T[a] >> 1;
}

static const uint8_t A_POS[24] = { 0,  4,  8,  12, 16, 20, 24, 28,
                                   32, 36, 40, 44, 48, 52, 56, 60,
                                   64, 68, 1,  5,  9,  13, 17, 21 };
static const uint8_t B_POS[23] = { 25, 29, 33, 37, 41, 45, 49, 53,
                                   57, 61, 65, 69, 2,  6,  10, 14,
                                   18, 22, 26, 30, 34, 38, 42 };
static const uint8_t C_POS[25] = { 46, 50, 54, 58, 62, 66, 70, 3,  7,
                                   11, 15, 19, 23, 27, 31, 35, 39, 43,
                                   47, 51, 55, 59, 63, 67, 71 };

// DMR-layout AMBE frame, scrambling sequence given by the caller
static void dmrFrame(uint32_t a, uint32_t b, uint32_t c, uint32_t prn,
                     uint8_t out[9])
{
    uint32_t fa = golay24(a);
    uint32_t fb = golay23(b) ^ prn;
    memset(out, 0, 9);
    for (int i = 0; i < 24; i++)
        setBit(out, A_POS[i], (fa >> (23 - i)) & 1);
    for (int i = 0; i < 23; i++)
        setBit(out, B_POS[i], (fb >> (22 - i)) & 1);
    for (int i = 0; i < 25; i++)
        setBit(out, C_POS[i], (c >> (24 - i)) & 1);
}

// V/D mode 2 voice channel n from the three voice fields (YSF2DMR layout)
static void encodeVd2Voice(uint32_t a, uint32_t b, uint32_t c, uint8_t *frame,
                           size_t n)
{
    uint8_t vch[13] = { 0 };
    for (int i = 0; i < 12; i++)
        for (int r = 0; r < 3; r++)
            setBit(vch, 3 * i + r, (a >> (11 - i)) & 1);
    for (int i = 0; i < 12; i++)
        for (int r = 0; r < 3; r++)
            setBit(vch, 3 * (i + 12) + r, (b >> (11 - i)) & 1);
    for (int i = 0; i < 3; i++)
        for (int r = 0; r < 3; r++)
            setBit(vch, 3 * (i + 24) + r, (c >> (24 - i)) & 1);
    for (int i = 0; i < 22; i++)
        setBit(vch, 81 + i, (c >> (21 - i)) & 1);
    setBit(vch, 103, 0);

    for (int i = 0; i < 13; i++)
        vch[i] ^= WHITENING[i];

    uint8_t *payload = frame + PAYLOAD_OFFSET;
    size_t offset = (144 * n) + 40;
    for (size_t i = 0; i < 104; i++) {
        size_t pos = (i / 26) + (4 * (i % 26));
        setBit(payload, offset + pos, getBit(vch, i));
    }
}

} /* ref */

static void flipBits(uint8_t *buf, size_t from, size_t len, size_t count,
                     std::mt19937 &rng)
{
    std::uniform_int_distribution<size_t> pick(from, from + len - 1);
    for (size_t i = 0; i < count; i++) {
        size_t p = pick(rng);
        buf[p >> 3] ^= (0x80 >> (p & 7));
    }
}

TEST_CASE("YSF CRC-16 matches the table-driven reference", "[ysf]")
{
    ref::crcInit();
    std::mt19937 rng(1);
    for (int t = 0; t < 200; t++) {
        uint8_t a[24], b[24];
        size_t len = 2 + (rng() % 20);
        for (size_t i = 0; i < len; i++)
            a[i] = b[i] = rng();
        ref::addCrc(a, len + 2);
        addCrc16(b, len);
        REQUIRE(a[len] == b[len]);
        REQUIRE(a[len + 1] == b[len + 1]);
        REQUIRE(checkCrc16(a, len));
        a[0] ^= 0x10;
        REQUIRE_FALSE(checkCrc16(a, len));
    }
}

TEST_CASE("YSF AMBE scrambling sequence matches the YSF2DMR table", "[ysf]")
{
    for (uint32_t a = 0; a < 48; a++)
        REQUIRE(ambePrng(a) == ref::prngTable(a));
}

TEST_CASE("YSF FICH decodes, with and without channel errors", "[ysf]")
{
    ref::crcInit();
    std::mt19937 rng(2);
    FrameDecoder dec;

    for (int t = 0; t < 300; t++) {
        frame_t frame;
        for (auto &b : frame)
            b = rng();
        uint8_t raw[4] = { (uint8_t)rng(), (uint8_t)rng(), (uint8_t)rng(),
                           (uint8_t)rng() };
        ref::encodeFich(raw, frame.data());

        // Up to 3 random errors over the 200 FICH bits
        flipBits(frame.data(), SYNC_BYTES * 8, 200, t % 4, rng);

        Fich f;
        REQUIRE(dec.decodeFich(frame, f));
        REQUIRE(memcmp(f.raw.data(), raw, 4) == 0);
    }

    // Field accessors against MMDVMHost's bit layout
    Fich f;
    f.setFi(FI_COMMUNICATIONS);
    f.setCm(CM_GROUP2);
    f.setBn(2);
    f.setBt(1);
    f.setFn(5);
    f.setFt(6);
    f.setMr(1);
    f.setDt(DT_VD_MODE2);
    f.setDgid(42);
    REQUIRE(f.raw[0] == ((1 << 6) | (1 << 2) | 2));
    REQUIRE(f.raw[1] == ((1 << 6) | (5 << 3) | 6));
    REQUIRE(f.raw[2] == ((1 << 3) | 2));
    REQUIRE(f.raw[3] == 42);
    REQUIRE(f.fi() == FI_COMMUNICATIONS);
    REQUIRE(f.fn() == 5);
    REQUIRE(f.dt() == DT_VD_MODE2);
    REQUIRE(f.dgid() == 42);
}

TEST_CASE("YSF FICH rejects heavy corruption", "[ysf]")
{
    ref::crcInit();
    std::mt19937 rng(3);
    FrameDecoder dec;
    int accepted = 0;

    for (int t = 0; t < 2000; t++) {
        frame_t frame;
        for (auto &b : frame)
            b = rng(); // noise only
        Fich f;
        if (dec.decodeFich(frame, f))
            accepted++;
    }

    // Four Golay words and a CRC-16: noise passes very rarely
    REQUIRE(accepted <= 1);
}

TEST_CASE("YSF V/D mode 2 data channel decodes callsigns", "[ysf]")
{
    ref::crcInit();
    std::mt19937 rng(4);
    FrameDecoder dec;

    for (int t = 0; t < 200; t++) {
        frame_t frame;
        for (auto &b : frame)
            b = rng();
        uint8_t text[10];
        for (auto &c : text)
            c = 0x20 + (rng() % 0x5F);
        ref::encodeDataChannel(text, 10, frame.data(), 5, 0);

        // A few errors in the DCH segments (first 40 bits of each block)
        for (int e = 0; e < (t % 3); e++) {
            size_t blk = rng() % 5;
            flipBits(frame.data(), (PAYLOAD_OFFSET * 8) + (blk * 144), 40, 1,
                     rng);
        }

        std::array<uint8_t, 10> out;
        REQUIRE(dec.decodeVd2Dch(frame, out));
        REQUIRE(memcmp(out.data(), text, 10) == 0);
    }

    frame_t frame;
    const char *call = "KG5IRU    ";
    for (auto &b : frame)
        b = 0;
    ref::encodeDataChannel((const uint8_t *)call, 10, frame.data(), 5, 0);
    std::array<uint8_t, 10> out;
    REQUIRE(dec.decodeVd2Dch(frame, out));
    char str[11];
    callsignToString(out.data(), str);
    REQUIRE(strcmp(str, "KG5IRU") == 0);
}

TEST_CASE("YSF header CSD1/CSD2 and V/D mode 1 data channel decode", "[ysf]")
{
    ref::crcInit();
    std::mt19937 rng(5);
    FrameDecoder dec;

    frame_t frame;
    for (auto &b : frame)
        b = rng();
    const char *csd1 = "ALL       KG5IRU    ";
    const char *csd2 = "W5ABC     N0CALL    ";
    ref::encodeDataChannel((const uint8_t *)csd1, 20, frame.data(), 9, 0);
    ref::encodeDataChannel((const uint8_t *)csd2, 20, frame.data(), 9, 9);
    flipBits(frame.data(), PAYLOAD_OFFSET * 8, 720, 3, rng);

    std::array<uint8_t, 20> out;
    REQUIRE(dec.decodeCsd(frame, 1, out));
    REQUIRE(memcmp(out.data(), csd1, 20) == 0);
    REQUIRE(dec.decodeCsd(frame, 2, out));
    REQUIRE(memcmp(out.data(), csd2, 20) == 0);

    // V/D mode 1 uses the CSD1 layout for its data channel
    REQUIRE(dec.decodeVd1Dch(frame, out));
    REQUIRE(memcmp(out.data(), csd1, 20) == 0);
}

TEST_CASE("YSF V/D mode 2 voice converts to DMR-layout AMBE+2", "[ysf]")
{
    std::mt19937 rng(6);

    for (int t = 0; t < 500; t++) {
        frame_t frame;
        for (auto &b : frame)
            b = rng();
        uint32_t a[5], b[5], c[5];
        for (int n = 0; n < 5; n++) {
            a[n] = rng() % 48; // within the reference PRNG table
            b[n] = rng() & 0xFFF;
            c[n] = rng() & 0x1FFFFFF;
            ref::encodeVd2Voice(a[n], b[n], c[n], frame.data(), n);
        }

        // One error per triplet in a few triplets: the vote fixes them
        size_t n = t % 5;
        for (int e = 0; e < (t % 4); e++) {
            uint8_t tmp[13] = { 0 };
            (void)tmp;
            size_t trip = (e * 7) % 27;
            size_t bit = 3 * trip + (e % 3);
            size_t pos = (bit / 26) + (4 * (bit % 26));
            size_t abs = (PAYLOAD_OFFSET * 8) + (144 * n) + 40 + pos;
            frame[abs >> 3] ^= (0x80 >> (abs & 7));
        }

        for (int k = 0; k < 5; k++) {
            uint8_t got[9], want[9];
            uint8_t fixed = vd2VoiceToDmr(frame, k, got);
            ref::dmrFrame(a[k], b[k], c[k], ref::prngTable(a[k]), want);
            REQUIRE(memcmp(got, want, 9) == 0);
            if ((size_t)k != n)
                REQUIRE(fixed == 0);
        }
    }
}

TEST_CASE("YSF AMBE field split and rebuild round-trip", "[ysf]")
{
    // DMR AMBE+2 silence frame
    const uint8_t silence[9] = { 0xB9, 0xE8, 0x81, 0x52, 0x61,
                                 0x73, 0x00, 0x2A, 0x6B };
    uint32_t a, b, c;
    REQUIRE(dmrToAmbeFields(silence, a, b, c));
    uint8_t rebuilt[9];
    ambeFieldsToDmr(a, b, c, rebuilt);
    REQUIRE(memcmp(rebuilt, silence, 9) == 0);

    std::mt19937 rng(7);
    for (int t = 0; t < 500; t++) {
        uint32_t a0 = rng() & 0xFFF, b0 = rng() & 0xFFF;
        uint32_t c0 = rng() & 0x1FFFFFF;
        uint8_t f[9];
        ambeFieldsToDmr(a0, b0, c0, f);
        // Up to 2 errors in field A and 2 in field B positions
        f[0] ^= 0x80;     // A bit 0
        if (t & 1)
            f[3] ^= 0x08; // bit 28: A
        f[3] ^= 0x04;     // bit 29: B
        REQUIRE(dmrToAmbeFields(f, a, b, c));
        REQUIRE(a == a0);
        REQUIRE(b == b0);
    }
}

TEST_CASE("YSF V/D mode 1 voice is copied as is", "[ysf]")
{
    frame_t frame;
    for (size_t i = 0; i < frame.size(); i++)
        frame[i] = i;
    uint8_t out[9];
    vd1VoiceToDmr(frame, 2, out);
    for (int i = 0; i < 9; i++)
        REQUIRE(out[i] == PAYLOAD_OFFSET + 36 + 9 + i);
}
