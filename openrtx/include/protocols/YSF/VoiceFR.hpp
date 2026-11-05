/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef YSF_VOICE_FR_H
#define YSF_VOICE_FR_H

#ifndef __cplusplus
#error This header is C++ only!
#endif

#include <cstdint>
#include <cstddef>
#include "protocols/YSF/Constants.hpp"
#include "protocols/YSF/Fich.hpp"

namespace YSF
{

/*
 * Voice full rate ("VW", data type 3): IMBE voice, the P25 vocoder, in the
 * 144-bit channel frames P25 uses (CP25Audio::encodeFrame/decodeFrame), one
 * per 18-byte payload block.  The first communications frame after the header
 * (frame number 0, frame total 1) carries a 45-byte data channel and two
 * voice frames, in the last two blocks; the others carry five (MMDVMHost
 * YSFPayload, YSF2P25).
 */

/** IMBE frame length, bytes (88 bits). */
static constexpr size_t IMBE_BYTES = 11;

/**
 * Number of IMBE frames in a VW communications frame.
 *
 * @param fich: the frame's FICH.
 * @return 2 or 5.
 */
size_t vwVoiceFrames(const Fich &fich);

/**
 * IMBE frame k of a VW communications frame, its channel coding corrected.
 *
 * @param frame: received frame.
 * @param fich: the frame's FICH.
 * @param k: 0 .. vwVoiceFrames(fich) - 1.
 * @param imbe: IMBE_BYTES out.
 * @return the number of channel bits corrected.
 */
unsigned int vwVoiceToImbe(const frame_t &frame, const Fich &fich,
                           const size_t k, uint8_t imbe[IMBE_BYTES]);

/**
 * Write IMBE frame n (0..4) of a five-voice VW communications frame.
 *
 * @param imbe: IMBE_BYTES.
 * @param n: 0..4.
 * @param frame: destination frame.
 */
void encodeVwVoice(const uint8_t imbe[IMBE_BYTES], const size_t n,
                   frame_t &frame);

} /* YSF */

#endif /* YSF_VOICE_FR_H */
