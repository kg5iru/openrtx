/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef YSF_FICH_H
#define YSF_FICH_H

#ifndef __cplusplus
#error This header is C++ only!
#endif

#include <cstdint>
#include <array>

namespace YSF
{

/**
 * Frame information channel: 32 bits of information plus a CRC-16, each 12
 * bits of the 48 protected by a Golay(24,12) code, then by the rate 1/2
 * convolutional code, interleaved over 200 bits after the sync word.
 */
class Fich
{
public:
    Fich() : raw{}
    {
    }

    /** Raw information bytes 0..3 (bytes 4 and 5 are the CRC). */
    std::array<uint8_t, 6> raw;

    /** Frame information */
    uint8_t fi() const
    {
        return (raw[0] >> 6) & 0x03;
    }

    /** Callsign information */
    uint8_t cs() const
    {
        return (raw[0] >> 4) & 0x03;
    }

    /** Call mode */
    uint8_t cm() const
    {
        return (raw[0] >> 2) & 0x03;
    }

    /** Block number */
    uint8_t bn() const
    {
        return raw[0] & 0x03;
    }

    /** Block total */
    uint8_t bt() const
    {
        return (raw[1] >> 6) & 0x03;
    }

    /** Frame number */
    uint8_t fn() const
    {
        return (raw[1] >> 3) & 0x07;
    }

    /** Frame total */
    uint8_t ft() const
    {
        return raw[1] & 0x07;
    }

    /** Low deviation */
    bool dev() const
    {
        return (raw[2] & 0x40) != 0;
    }

    /** Message route */
    uint8_t mr() const
    {
        return (raw[2] >> 3) & 0x03;
    }

    /** Network path */
    bool voip() const
    {
        return (raw[2] & 0x04) != 0;
    }

    /** Data type */
    uint8_t dt() const
    {
        return raw[2] & 0x03;
    }

    /** Digital group ID */
    uint8_t dgid() const
    {
        return raw[3] & 0x7F;
    }

    void setFi(uint8_t v)
    {
        raw[0] = (raw[0] & 0x3F) | ((v & 0x03) << 6);
    }

    void setCs(uint8_t v)
    {
        raw[0] = (raw[0] & 0xCF) | ((v & 0x03) << 4);
    }

    void setCm(uint8_t v)
    {
        raw[0] = (raw[0] & 0xF3) | ((v & 0x03) << 2);
    }

    void setBn(uint8_t v)
    {
        raw[0] = (raw[0] & 0xFC) | (v & 0x03);
    }

    void setBt(uint8_t v)
    {
        raw[1] = (raw[1] & 0x3F) | ((v & 0x03) << 6);
    }

    void setFn(uint8_t v)
    {
        raw[1] = (raw[1] & 0xC7) | ((v & 0x07) << 3);
    }

    void setFt(uint8_t v)
    {
        raw[1] = (raw[1] & 0xF8) | (v & 0x07);
    }

    void setMr(uint8_t v)
    {
        raw[2] = (raw[2] & 0xE7) | ((v & 0x03) << 3);
    }

    void setDt(uint8_t v)
    {
        raw[2] = (raw[2] & 0xFC) | (v & 0x03);
    }

    void setDgid(uint8_t v)
    {
        raw[3] = (raw[3] & 0x80) | (v & 0x7F);
    }
};

} /* YSF */

#endif /* YSF_FICH_H */
