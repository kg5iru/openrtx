/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef YSF_FRAME_ENCODER_H
#define YSF_FRAME_ENCODER_H

#ifndef __cplusplus
#error This header is C++ only!
#endif

#include <cstdint>
#include <cstddef>
#include "protocols/YSF/Constants.hpp"
#include "protocols/YSF/Fich.hpp"

namespace YSF
{

/**
 * Builder of the FICH information bytes.
 *
 * @param fi: frame information (FI_HEADER, FI_COMMUNICATIONS, ...).
 * @param fn: frame number.
 * @param ft: frame total.
 * @param dt: data type.
 * @param dgid: DG-ID, 0..127.
 * @return a FICH with callsign information 2, call mode group (CQ), block
 * 0 of 1, normal deviation, direct route, no VoIP, as a radio sends it
 * (the values MMDVM's DMR2YSF uses).
 */
Fich makeFich(const uint8_t fi, const uint8_t fn, const uint8_t ft,
              const uint8_t dt, const uint8_t dgid);

/**
 * Write the sync word at the start of a frame.
 */
void encodeSync(frame_t &frame);

/**
 * Encode the FICH (CRC, Golay(24,12), convolutional code, interleaving)
 * into its place after the sync word.
 *
 * @param fich: FICH, information bytes 0..3 (the CRC is computed here).
 * @param frame: destination frame.
 */
void encodeFich(const Fich &fich, frame_t &frame);

/**
 * Encode the two callsign data channels of a header or terminator frame.
 *
 * @param csd1: destination and source callsigns, 20 characters.
 * @param csd2: downlink and uplink callsigns, 20 characters.
 * @param frame: destination frame.
 */
void encodeCsd(const uint8_t csd1[20], const uint8_t csd2[20], frame_t &frame);

/**
 * Encode the data channel of a V/D mode 2 frame.
 *
 * @param dch: 10 characters (meaning given by the frame number).
 * @param frame: destination frame.
 */
void encodeVd2Dch(const uint8_t dch[10], frame_t &frame);

/**
 * Encode voice frame n (0..4) of a V/D mode 2 frame from a 72-bit AMBE+2
 * frame in DMR layout (as the CS7000 codec produces it): the 49 voice bits
 * are taken out of their DMR protection and sent with the first 27
 * repeated three times, whitened and interleaved.
 *
 * @param ambe: 9 bytes, DMR layout.
 * @param n: voice frame index, 0..4.
 * @param frame: destination frame.
 */
void encodeVd2Voice(const uint8_t ambe[9], const size_t n, frame_t &frame);

} /* YSF */

#endif /* YSF_FRAME_ENCODER_H */
