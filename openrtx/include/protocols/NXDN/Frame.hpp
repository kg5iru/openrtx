/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef NXDN_FRAME_H
#define NXDN_FRAME_H

#ifndef __cplusplus
#error This header is C++ only!
#endif

#include <cstdint>
#include <cstddef>
#include "protocols/NXDN/Coding.hpp"

/*
 * Voice call frames, as MMDVMHost sends and accepts them (NXDNControl.cpp,
 * MMDVM_CM DMR2NXDN.cpp):
 *
 *   header      LICH RDCH / SACCH_NS / both halves FACCH1, SACCH idle,
 *               the VCALL message in both FACCH1
 *   voice       LICH RDCH / SACCH_SS / no FACCH1, four AMBE+2 frames; the
 *               SACCH carries the VCALL message over four frames (structure
 *               1/4 .. 4/4, 18 bits each)
 *   terminator  as the header, with the TX_REL message
 *
 * A radio (or a hotspot's RF side) sends inbound: MMDVMHost drops outbound
 * frames received on its RF side.
 */

namespace NXDN
{

/** Layer 3 message, as carried by a FACCH1 (80 bits). */
using layer3_t = facch1_t;

/**
 * Voice call message.
 *
 * @param type: MSG_VCALL or MSG_TX_REL.
 * @param src: source unit ID.
 * @param dst: destination group (or unit, for an individual call) ID.
 * @param group: group call (else individual).
 */
layer3_t makeVoiceCall(const uint8_t type, const uint16_t src,
                       const uint16_t dst, const bool group);

static inline uint8_t messageType(const layer3_t &l3)
{
    return l3[0] & 0x3F;
}

static inline uint16_t sourceId(const layer3_t &l3)
{
    return (static_cast<uint16_t>(l3[3]) << 8) | l3[4];
}

static inline uint16_t destinationId(const layer3_t &l3)
{
    return (static_cast<uint16_t>(l3[5]) << 8) | l3[6];
}

static inline bool isGroupCall(const layer3_t &l3)
{
    return (l3[2] & 0x80) == 0;
}

/**
 * Build a header (VCALL) or terminator (TX_REL) frame, scrambled.
 *
 * @param ran: radio access number, 0..63.
 */
void buildControlFrame(frame_t &frame, const uint8_t ran, const layer3_t &l3);

/**
 * Build a voice frame, scrambled.
 *
 * @param ran: radio access number, 0..63.
 * @param l3: the call's VCALL message, whose part is sent in the SACCH.
 * @param part: 0..3, frame number in the superframe of four.
 * @param ambe: four AMBE+2 frames, 9 bytes each in the DMR layout.
 */
void buildVoiceFrame(frame_t &frame, const uint8_t ran, const layer3_t &l3,
                     const size_t part, const uint8_t ambe[4][AMBE_BYTES]);

/** SACCH structure of a superframe part, 0..3. */
static inline uint8_t partStructure(const size_t part)
{
    return static_cast<uint8_t>(SR_1_4 - part);
}

/** The 18 bits of a layer 3 message a superframe part carries. */
void layer3Part(const layer3_t &l3, const size_t part, uint8_t data[3]);

/** Store the 18 bits of a superframe part into a layer 3 message. */
void setLayer3Part(layer3_t &l3, const size_t part, const uint8_t data[3]);

} /* NXDN */

#endif /* NXDN_FRAME_H */
