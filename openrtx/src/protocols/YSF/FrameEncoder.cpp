/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <cstring>
#include "protocols/M17/Golay.hpp"
#include "protocols/YSF/Coding.hpp"
#include "protocols/YSF/FrameDecoder.hpp"
#include "protocols/YSF/FrameEncoder.hpp"

using namespace YSF;

Fich YSF::makeFich(const uint8_t fi, const uint8_t fn, const uint8_t ft,
                   const uint8_t dt, const uint8_t dgid)
{
    Fich fich;

    // CS 2, call mode group (CQ), block 0 of 1, normal deviation, direct
    // route, no VoIP: raw bytes start at zero
    fich.setFi(fi);
    fich.setCs(2);
    fich.setCm(CM_GROUP1);
    fich.setFn(fn);
    fich.setFt(ft);
    fich.setDt(dt);
    fich.setDgid(dgid);

    return fich;
}

void YSF::encodeSync(frame_t &frame)
{
    memcpy(frame.data(), SYNC_WORD.data(), SYNC_BYTES);
}

void YSF::encodeFich(const Fich &fich, frame_t &frame)
{
    uint8_t f[6];
    memcpy(f, fich.raw.data(), 4);
    addCrc16(f, 4);

    // Four Golay(24,12) words, then 4 zero tail bits
    uint16_t w[4];
    w[0] = (f[0] << 4) | (f[1] >> 4);
    w[1] = ((f[1] & 0x0F) << 8) | f[2];
    w[2] = (f[3] << 4) | (f[4] >> 4);
    w[3] = ((f[4] & 0x0F) << 8) | f[5];

    uint8_t data[13] = { 0 };
    for (size_t k = 0; k < 4; k++) {
        uint32_t cw = M17::golay24_encode(w[k]);
        data[3 * k] = (cw >> 16) & 0xFF;
        data[3 * k + 1] = (cw >> 8) & 0xFF;
        data[3 * k + 2] = cw & 0xFF;
    }

    uint8_t *chan = frame.data() + SYNC_BYTES;
    memset(chan, 0, FICH_BYTES);
    convEncode(data, 100, chan, 5);
}

/*
 * A data channel: the characters whitened, a CRC, 4 zero tail bits, the
 * convolutional code interleaved over cols columns, then spread over the five
 * 144-bit payload blocks, segBytes bytes starting segOffset bytes in each.
 */
static void encodeDataChannel(const uint8_t *text, const size_t len,
                              const size_t segBytes, const size_t segOffset,
                              frame_t &frame)
{
    uint8_t buf[24] = { 0 }; // up to 20 characters, CRC, tail
    for (size_t i = 0; i < len; i++)
        buf[i] = text[i] ^ WHITENING[i];
    addCrc16(buf, len);

    const size_t nBits = ((len + 2) * 8) + 4;
    uint8_t gathered[45] = { 0 };
    convEncode(buf, nBits, gathered, nBits / 20);

    uint8_t *payload = frame.data() + PAYLOAD_OFFSET;
    for (size_t s = 0; s < 5; s++)
        memcpy(&payload[(s * 18) + segOffset], &gathered[s * segBytes],
               segBytes);
}

void YSF::encodeCsd(const uint8_t csd1[20], const uint8_t csd2[20],
                    frame_t &frame)
{
    encodeDataChannel(csd1, 20, 9, 0, frame);
    encodeDataChannel(csd2, 20, 9, 9, frame);
}

void YSF::encodeVd2Dch(const uint8_t dch[10], frame_t &frame)
{
    encodeDataChannel(dch, 10, 5, 0, frame);
}

void YSF::encodeVd2Voice(const uint8_t ambe[9], const size_t n, frame_t &frame)
{
    uint32_t a = 0, b = 0, c = 0;
    dmrToAmbeFields(ambe, a, b, c);

    // a (12 bits), b (12 bits) and the 3 top bits of c, each sent three
    // times, then the other 22 bits of c and a zero bit
    uint32_t first = (a << 15) | (b << 3) | ((c >> 22) & 0x07);
    uint8_t vch[13] = { 0 };
    for (size_t i = 0; i < 27; i++) {
        bool bit = ((first >> (26 - i)) & 1) != 0;
        writeBit(vch, 3 * i, bit);
        writeBit(vch, 3 * i + 1, bit);
        writeBit(vch, 3 * i + 2, bit);
    }
    for (size_t i = 0; i < 22; i++)
        writeBit(vch, 81 + i, ((c >> (21 - i)) & 1) != 0);

    for (size_t i = 0; i < 13; i++)
        vch[i] ^= WHITENING[i];

    uint8_t *payload = frame.data() + PAYLOAD_OFFSET;
    const size_t offset = (144 * n) + 40; // after the 40-bit DCH segment
    for (size_t i = 0; i < 104; i++)
        writeBit(payload, offset + vchPosition(i), readBit(vch, i));
}
