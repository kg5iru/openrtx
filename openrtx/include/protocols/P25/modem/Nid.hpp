/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef P25_MODEM_NID_H
#define P25_MODEM_NID_H

#ifndef __cplusplus
#error This header is C++ only!
#endif

#include <cstdint>
#include <cstddef>

namespace P25
{

/**
 * P25 Phase 1 Network Identifier: 12-bit NAC and 4-bit DUID protected by a
 * BCH(63,16) code correcting up to 11 bit errors, plus one even parity bit,
 * 64 bits in all. The NID follows the frame sync in every data unit and its
 * DUID determines the frame length, so the demodulator decodes it to frame
 * the stream; it is decoded independently of the local NAC.
 */
class Nid
{
public:
    static constexpr uint8_t DUID_HDU = 0x0;   ///< Header Data Unit
    static constexpr uint8_t DUID_TDU = 0x3;   ///< Terminator Data Unit
    static constexpr uint8_t DUID_LDU1 = 0x5;  ///< Logical Link Data Unit 1
    static constexpr uint8_t DUID_TSDU = 0x7;  ///< Trunking Signaling DU
    static constexpr uint8_t DUID_LDU2 = 0xA;  ///< Logical Link Data Unit 2
    static constexpr uint8_t DUID_PDU = 0xC;   ///< Packet Data Unit
    static constexpr uint8_t DUID_TDULC = 0xF; ///< Terminator with Link Control

    static constexpr size_t BYTES = 8;         ///< Size of a NID
    static constexpr size_t ERRORS = 11;       ///< Correctable bit errors

    /**
     * Encode a NID.
     *
     * @param nac: network access code, 12 bits.
     * @param duid: data unit ID, 4 bits.
     * @param out: destination, 8 bytes, MSB first.
     */
    static void encode(const uint16_t nac, const uint8_t duid, uint8_t *out);

    /**
     * Decode a NID, correcting up to ERRORS bit errors in the BCH codeword.
     * The parity bit is ignored.
     *
     * @param in: NID, 8 bytes, MSB first.
     * @param nac: decoded network access code.
     * @param duid: decoded data unit ID.
     * @return true if the codeword was valid or correctable.
     */
    static bool decode(const uint8_t *in, uint16_t &nac, uint8_t &duid);

    /**
     * Copy the NID out of a raw frame: bits 48 to 113 of the frame, skipping
     * the status symbol at bits 70 and 71.
     *
     * @param frame: raw frame, at least 15 bytes.
     * @param out: destination, 8 bytes.
     */
    static void fromFrame(const uint8_t *frame, uint8_t *out);

    /**
     * Write a NID into a raw frame, leaving the status symbol untouched.
     *
     * @param in: NID, 8 bytes.
     * @param frame: raw frame, at least 15 bytes.
     */
    static void toFrame(const uint8_t *in, uint8_t *frame);

    /**
     * Raw frame length, in bytes, of the data unit identified by a DUID.
     *
     * @param duid: data unit ID.
     * @return frame length in bytes: HDU 99, TDU 18, LDU1/LDU2 216, TDULC
     * 54; zero for the variable-length TSDU and PDU and for unknown DUIDs.
     */
    static size_t frameBytes(const uint8_t duid);
};

} /* P25 */

#endif /* P25_MODEM_NID_H */
