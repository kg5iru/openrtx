/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <cstring>
#include "protocols/P25/AMBEFEC.h"
#include "protocols/P25/P25Audio.h"
#include "protocols/YSF/VoiceFR.hpp"

using namespace YSF;

static constexpr size_t BLOCK_BYTES = 18;

size_t YSF::vwVoiceFrames(const Fich &fich)
{
    return ((fich.fn() == 0) && (fich.ft() == 1)) ? 2 : 5;
}

unsigned int YSF::vwVoiceToImbe(const frame_t &frame, const Fich &fich,
                                const size_t k, uint8_t imbe[IMBE_BYTES])
{
    // With two voice frames, they are in the last two of the five blocks
    size_t block = (vwVoiceFrames(fich) == 2) ? (3 + k) : k;

    uint8_t channel[BLOCK_BYTES];
    memcpy(channel, &frame[PAYLOAD_OFFSET + (block * BLOCK_BYTES)],
           BLOCK_BYTES);

    CAMBEFEC fec;
    unsigned int errors = fec.regenerateIMBE(channel);
    CP25Audio::decodeFrame(channel, imbe);
    return errors;
}

void YSF::encodeVwVoice(const uint8_t imbe[IMBE_BYTES], const size_t n,
                        frame_t &frame)
{
    CP25Audio::encodeFrame(imbe, &frame[PAYLOAD_OFFSET + (n * BLOCK_BYTES)]);
}
