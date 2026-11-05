/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <cstring>
#include <random>

#include "protocols/NXDN/Coding.hpp"
#include "protocols/NXDN/Frame.hpp"

using namespace NXDN;

/*
 * Reference frames, made with MMDVMHost's NXDN classes (CNXDNLICH,
 * CNXDNSACCH, CNXDNFACCH1, the scrambler of CNXDNControl) as its RF side
 * writes them: a header for a group call from 1234 to 10 with RAN 1, and the
 * third voice frame of a superframe of that call, whose voice frames hold
 * the bytes 0x11 * k + i.
 */
static constexpr frame_t HEADER_RAN1_1234_TG10 = {
    0xCD, 0xF5, 0x9D, 0xD7, 0xD6, 0x3C, 0x2B, 0x42, 0xE2, 0xA8, 0xA2, 0xA1,
    0xC6, 0xB0, 0x65, 0x28, 0x0A, 0x68, 0x8E, 0xA0, 0xAA, 0x09, 0xA2, 0x6E,
    0xB6, 0x0A, 0x22, 0x0E, 0x0B, 0xC9, 0x44, 0xB8, 0x47, 0xA2, 0xA0, 0x60,
    0xA6, 0x28, 0x82, 0x21, 0xA2, 0x64, 0xB4, 0x88, 0x02, 0x26, 0x89, 0xE3
};
static constexpr frame_t VOICE_PART2 = {
    0xCD, 0xF5, 0x9D, 0x5D, 0xF6, 0x2A, 0x9A, 0x58, 0x51, 0x18, 0x63, 0xEB,
    0x82, 0x03, 0x22, 0x0B, 0x8E, 0x25, 0xAC, 0xA5, 0x8A, 0x19, 0x30, 0x99,
    0xBE, 0x1D, 0x3E, 0x9F, 0x30, 0x31, 0x22, 0x29, 0x26, 0xA7, 0x06, 0x0F,
    0xAA, 0x03, 0x80, 0x13, 0x16, 0xB5, 0x9E, 0xBD, 0x30, 0x99, 0x90, 0x39
};

static void flip(frame_t &frame, const size_t bit)
{
    frame[bit / 8] ^= 0x80 >> (bit % 8);
}

TEST_CASE("NXDN frames: header and voice frames as MMDVMHost makes them",
          "[nxdn]")
{
    layer3_t vcall = makeVoiceCall(MSG_VCALL, 1234, 10, true);

    frame_t header;
    buildControlFrame(header, 1, vcall);
    REQUIRE(header == HEADER_RAN1_1234_TG10);

    uint8_t ambe[4][AMBE_BYTES];
    for (size_t k = 0; k < 4; k++)
        for (size_t i = 0; i < AMBE_BYTES; i++)
            ambe[k][i] = (0x11 * k) + i;

    frame_t voice;
    buildVoiceFrame(voice, 1, vcall, 2, ambe);
    REQUIRE(voice == VOICE_PART2);
}

TEST_CASE("NXDN frames: header decoded", "[nxdn]")
{
    frame_t frame = HEADER_RAN1_1234_TG10;
    REQUIRE(syncErrors(frame) == 0);
    scramble(frame);

    uint8_t lich;
    REQUIRE(decodeLich(frame, lich));
    REQUIRE(lichRfct(lich) == RFCT_RDCH);
    REQUIRE(lichFct(lich) == FCT_SACCH_NS);
    REQUIRE(lichOption(lich) == STEAL_FACCH);
    REQUIRE(lichDirection(lich) == DIR_INBOUND);

    sacch_t sacch;
    REQUIRE(decodeSacch(frame, sacch));
    REQUIRE(sacchRan(sacch) == 1);
    REQUIRE(sacchStructure(sacch) == SR_SINGLE);

    for (size_t half = 0; half < 2; half++) {
        layer3_t l3;
        REQUIRE(decodeFacch1(frame, PAYLOAD_OFFSET + (half * FACCH1_BITS), l3));
        REQUIRE(messageType(l3) == MSG_VCALL);
        REQUIRE(sourceId(l3) == 1234);
        REQUIRE(destinationId(l3) == 10);
        REQUIRE(isGroupCall(l3));
    }
}

TEST_CASE("NXDN frames: voice frame decoded", "[nxdn]")
{
    frame_t frame = VOICE_PART2;
    scramble(frame);

    uint8_t lich;
    REQUIRE(decodeLich(frame, lich));
    REQUIRE(lichFct(lich) == FCT_SACCH_SS);
    REQUIRE(lichOption(lich) == STEAL_NONE);

    sacch_t sacch;
    REQUIRE(decodeSacch(frame, sacch));
    REQUIRE(sacchRan(sacch) == 1);
    REQUIRE(sacchStructure(sacch) == partStructure(2));

    for (size_t k = 0; k < 4; k++)
        for (size_t i = 0; i < AMBE_BYTES; i++)
            REQUIRE(frame[VOICE_BYTE_OFFSET + (AMBE_BYTES * k) + i]
                    == ((0x11 * k) + i));
}

TEST_CASE("NXDN frames: the call message rebuilt from a superframe", "[nxdn]")
{
    layer3_t vcall = makeVoiceCall(MSG_VCALL, 65000, 31337, true);
    uint8_t ambe[4][AMBE_BYTES] = {};
    layer3_t rebuilt{};

    for (size_t part = 0; part < 4; part++) {
        frame_t frame;
        buildVoiceFrame(frame, 63, vcall, part, ambe);
        scramble(frame);

        sacch_t sacch;
        REQUIRE(decodeSacch(frame, sacch));
        REQUIRE(sacchRan(sacch) == 63);
        size_t got = SR_1_4 - sacchStructure(sacch);
        REQUIRE(got == part);

        uint8_t data[3];
        sacchData(sacch, data);
        setLayer3Part(rebuilt, got, data);
    }

    // The superframe carries the first 72 bits: type, IDs and options
    REQUIRE(messageType(rebuilt) == MSG_VCALL);
    REQUIRE(sourceId(rebuilt) == 65000);
    REQUIRE(destinationId(rebuilt) == 31337);
    REQUIRE(isGroupCall(rebuilt));
    REQUIRE(memcmp(rebuilt.data(), vcall.data(), 9) == 0);
}

TEST_CASE("NXDN frames: bit errors corrected, or caught by the CRC", "[nxdn]")
{
    std::mt19937 rng(7);
    layer3_t vcall = makeVoiceCall(MSG_TX_REL, 4321, 99, false);

    size_t sacchOk = 0, facchOk = 0, wrong = 0;
    for (int t = 0; t < 500; t++) {
        frame_t frame;
        buildControlFrame(frame, 5, vcall);
        scramble(frame);

        // Two errors in the SACCH, four in each FACCH1
        for (int e = 0; e < 2; e++)
            flip(frame, SACCH_OFFSET + (rng() % 60));
        for (int e = 0; e < 8; e++)
            flip(frame, PAYLOAD_OFFSET + (rng() % (2 * FACCH1_BITS)));

        sacch_t sacch;
        if (decodeSacch(frame, sacch)) {
            sacchOk++;
            if (sacchRan(sacch) != 5)
                wrong++;
        }

        layer3_t l3;
        bool ok = decodeFacch1(frame, PAYLOAD_OFFSET, l3)
               || decodeFacch1(frame, PAYLOAD_OFFSET + FACCH1_BITS, l3);
        if (ok) {
            facchOk++;
            if (l3 != vcall)
                wrong++;
        }
    }

    INFO("SACCH " << sacchOk << ", FACCH1 " << facchOk << " of 500");
    REQUIRE(sacchOk >= 450);
    REQUIRE(facchOk >= 470);
    REQUIRE(wrong == 0);
}

TEST_CASE("NXDN frames: LICH parity and sync word errors", "[nxdn]")
{
    frame_t frame{};
    encodeSync(frame);
    REQUIRE(syncErrors(frame) == 0);
    flip(frame, 3);
    flip(frame, 17);
    REQUIRE(syncErrors(frame) == 2);
    flip(frame, 21); // the LICH: not part of the sync word
    REQUIRE(syncErrors(frame) == 2);

    uint8_t lich = makeLich(RFCT_RDCH, FCT_SACCH_SS, STEAL_NONE, DIR_INBOUND);
    encodeLich(lich, frame);
    uint8_t got;
    REQUIRE(decodeLich(frame, got));
    REQUIRE(got == lich);
    // The "parity" bit is set for the header and terminator LICH (0x8_)
    // and clear for the voice one (0xA_): it tells the two apart
    flip(frame, LICH_OFFSET + 4); // voice LICH, read as the header one
    REQUIRE_FALSE(decodeLich(frame, got));
}
