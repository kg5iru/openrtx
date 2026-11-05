/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <cstring>
#include "protocols/NXDN/Frame.hpp"

using namespace NXDN;

layer3_t NXDN::makeVoiceCall(const uint8_t type, const uint16_t src,
                             const uint16_t dst, const bool group)
{
    layer3_t l3{};
    l3[0] = type & 0x3F;
    // Call type: 001 conference group call, 100 individual call; voice call
    // option 0 (half duplex, 4800 bit/s EHR)
    l3[2] = group ? 0x20 : 0x80;
    l3[3] = src >> 8;
    l3[4] = src & 0xFF;
    l3[5] = dst >> 8;
    l3[6] = dst & 0xFF;
    return l3;
}

void NXDN::layer3Part(const layer3_t &l3, const size_t part, uint8_t data[3])
{
    memset(data, 0, 3);
    for (size_t i = 0; i < 18; i++)
        writeBit(data, i, readBit(l3.data(), (18 * part) + i));
}

void NXDN::setLayer3Part(layer3_t &l3, const size_t part, const uint8_t data[3])
{
    for (size_t i = 0; i < 18; i++)
        writeBit(l3.data(), (18 * part) + i, readBit(data, i));
}

void NXDN::buildControlFrame(frame_t &frame, const uint8_t ran,
                             const layer3_t &l3)
{
    static constexpr uint8_t SACCH_IDLE[3] = { MSG_IDLE, 0x00, 0x00 };

    frame.fill(0);
    encodeSync(frame);
    encodeLich(makeLich(RFCT_RDCH, FCT_SACCH_NS, STEAL_FACCH, DIR_INBOUND),
               frame);
    encodeSacch(makeSacch(ran, SR_SINGLE, SACCH_IDLE), frame);
    encodeFacch1(l3, PAYLOAD_OFFSET, frame);
    encodeFacch1(l3, PAYLOAD_OFFSET + FACCH1_BITS, frame);
    scramble(frame);
}

void NXDN::buildVoiceFrame(frame_t &frame, const uint8_t ran,
                           const layer3_t &l3, const size_t part,
                           const uint8_t ambe[4][AMBE_BYTES])
{
    uint8_t data[3];
    layer3Part(l3, part, data);

    frame.fill(0);
    encodeSync(frame);
    encodeLich(makeLich(RFCT_RDCH, FCT_SACCH_SS, STEAL_NONE, DIR_INBOUND),
               frame);
    encodeSacch(makeSacch(ran, partStructure(part), data), frame);
    for (size_t i = 0; i < 4; i++)
        memcpy(&frame[VOICE_BYTE_OFFSET + (AMBE_BYTES * i)], ambe[i],
               AMBE_BYTES);
    scramble(frame);
}
