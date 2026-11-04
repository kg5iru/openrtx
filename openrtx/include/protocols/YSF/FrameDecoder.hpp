/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef YSF_FRAME_DECODER_H
#define YSF_FRAME_DECODER_H

#ifndef __cplusplus
#error This header is C++ only!
#endif

#include <cstdint>
#include <array>
#include "protocols/M17/Viterbi.hpp"
#include "protocols/YSF/Constants.hpp"
#include "protocols/YSF/Fich.hpp"

namespace YSF
{

/**
 * Decoder for the contents of a received YSF frame (sync word included, as
 * produced by the demodulator).  Hard decision; the Viterbi decoder is kept
 * as a member to stay off the caller's stack.
 */
class FrameDecoder
{
public:
    FrameDecoder()
    {
    }

    ~FrameDecoder()
    {
    }

    /**
     * Decode the FICH.
     *
     * @param frame: received frame.
     * @param fich: decoded FICH, valid only when true is returned.
     * @return true if all four Golay words decoded and the CRC matches.
     */
    bool decodeFich(const frame_t &frame, Fich &fich);

    /**
     * Decode one of the two callsign data channels of a header or terminator
     * frame: CSD1 holds the destination and source callsigns, CSD2 the
     * downlink and uplink ones.
     *
     * @param frame: received frame.
     * @param csd: 1 or 2.
     * @param out: two 10-character callsigns, unwhitened.
     * @return true if the CRC matches.
     */
    bool decodeCsd(const frame_t &frame, const uint8_t csd,
                   std::array<uint8_t, 20> &out);

    /**
     * Decode the data channel of a V/D mode 2 frame: 10 characters, whose
     * meaning depends on the frame number (0 destination, 1 source,
     * 2 downlink, 3 uplink callsign).
     *
     * @param frame: received frame.
     * @param out: 10 characters, unwhitened.
     * @return true if the CRC matches.
     */
    bool decodeVd2Dch(const frame_t &frame, std::array<uint8_t, 10> &out);

    /**
     * Decode the data channel of a V/D mode 1 frame: 20 characters, frame
     * number 0 = destination and source, 1 = downlink and uplink.
     *
     * @param frame: received frame.
     * @param out: 20 characters, unwhitened.
     * @return true if the CRC matches.
     */
    bool decodeVd1Dch(const frame_t &frame, std::array<uint8_t, 20> &out);

private:
    template <size_t CODED_BYTES, size_t OUT_BYTES>
    bool decodeDataChannel(const frame_t &frame, const size_t segBytes,
                           const size_t segOffset, const size_t cols,
                           std::array<uint8_t, OUT_BYTES> &out);

    M17::HardViterbi viterbi;
};

/**
 * Extract voice frame n (0..4) of a V/D mode 2 frame as a 72-bit AMBE+2
 * frame in DMR layout (the format the CS7000 codec and the DMR receive path
 * use).  The voice channel carries the 49 voice bits with the first 27
 * repeated three times; they are majority-voted, then protected again with
 * the DMR Golay codes and scrambling.
 *
 * @param frame: received frame.
 * @param n: voice frame index, 0..4.
 * @param ambe: 9 bytes, DMR layout.
 * @return number of corrected triplets (0..27).
 */
uint8_t vd2VoiceToDmr(const frame_t &frame, const size_t n, uint8_t ambe[9]);

/**
 * Extract voice frame n (0..4) of a V/D mode 1 frame, already in DMR layout.
 *
 * @param frame: received frame.
 * @param n: voice frame index, 0..4.
 * @param ambe: 9 bytes, DMR layout.
 */
void vd1VoiceToDmr(const frame_t &frame, const size_t n, uint8_t ambe[9]);

/**
 * Build a 72-bit DMR-layout AMBE+2 frame from the three voice fields.
 *
 * @param a: 12 bits, the most significant voice bits (Golay(24,12) in DMR).
 * @param b: 12 bits (Golay(23,12), scrambled by a sequence seeded by a).
 * @param c: 25 bits, unprotected.
 * @param ambe: destination, 9 bytes.
 */
void ambeFieldsToDmr(const uint32_t a, const uint32_t b, const uint32_t c,
                     uint8_t ambe[9]);

/**
 * Split a 72-bit DMR-layout AMBE+2 frame into its three voice fields,
 * correcting A and B with their Golay codes.
 *
 * @param ambe: 9 bytes, DMR layout.
 * @param a: 12 bits.
 * @param b: 12 bits.
 * @param c: 25 bits.
 * @return false if A could not be corrected.
 */
bool dmrToAmbeFields(const uint8_t ambe[9], uint32_t &a, uint32_t &b,
                     uint32_t &c);

/**
 * Scrambling sequence applied to field B of a DMR AMBE+2 frame.
 *
 * @param a: the 12 bits of field A.
 * @return 23 bits.
 */
uint32_t ambePrng(const uint32_t a);

/**
 * Trim a received 10-character callsign field: trailing spaces and
 * non-printable characters removed, null terminated.
 *
 * @param in: 10 characters.
 * @param out: 11 bytes.
 */
void callsignToString(const uint8_t *in, char out[11]);

} /* YSF */

#endif /* YSF_FRAME_DECODER_H */
