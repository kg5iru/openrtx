/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef DMR_EMB_HPP
#define DMR_EMB_HPP

#include "QR1676.hpp"

namespace DMR
{

/**
 * \brief The EMB (EMBedded signalling) field, carried in the first 16
 *        of the middle 48 bits of every voice burst B..E (the ones
 *        that are not the burst-A Sync).
 *
 * ETSI TS 102 361-1 clause 9.1.4: 4-bit colour code + 1-bit PI
 * (privacy indicator) + 2-bit LCSS (link-control start/stop, which
 * says where this burst's 32-bit embedded-signalling fragment sits
 * within the superframe's reassembled Embedded LC), QR (16,7,6)
 * protected.
 *
 * Reassembling the four 32-bit fragments (bursts B,C,D,E) into a full
 * 72-bit Embedded LC across a voice superframe is not implemented yet
 * -- see docs/DMR_STATUS.md.
 */
struct Emb
{
    uint8_t colorCode;  // 0-15
    bool    pi;
    uint8_t lcss;       // 0-3: 1=first fragment, 3=last fragment, 0=continuation, 2=single-fragment LC

    static bool decode(const uint8_t bits16[16], Emb &out)
    {
        uint8_t codeword[16];
        for (unsigned int i = 0; i < 16; i++)
            codeword[i] = bits16[i];

        uint8_t info[7];
        if (!QR1676::decode(codeword, info))
            return false;

        out.colorCode = (uint8_t)((info[0] << 3) | (info[1] << 2) | (info[2] << 1) | info[3]);
        out.pi        = info[4] != 0;
        out.lcss      = (uint8_t)((info[5] << 1) | info[6]);
        return true;
    }

    void encode(uint8_t bits16[16]) const
    {
        uint8_t info[7] =
        {
            (uint8_t)((colorCode >> 3) & 1), (uint8_t)((colorCode >> 2) & 1),
            (uint8_t)((colorCode >> 1) & 1), (uint8_t)(colorCode & 1),
            (uint8_t)(pi ? 1 : 0),
            (uint8_t)((lcss >> 1) & 1), (uint8_t)(lcss & 1),
        };
        QR1676::encode(info, bits16);
    }
};

}   // namespace DMR

#endif  // DMR_EMB_HPP
