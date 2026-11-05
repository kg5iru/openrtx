/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef P25_MODEM_VOCODER_H
#define P25_MODEM_VOCODER_H

#ifndef __cplusplus
#error This header is C++ only!
#endif

#include <cstdint>
#include <cstddef>

#include <imbe2.h>

namespace P25
{

/**
 * IMBE 4400 bps vocoder for P25, calling the imbe2 engine API directly
 * (no adapter class in between).
 *
 * Encoder and decoder states are disjoint by construction: encode() and
 * decode() may run concurrently on different threads without locking,
 * matching the radio's concurrent codec threads.
 */
class Vocoder
{
public:
    /**
     * Constructor.
     */
    Vocoder() : initialised(false)
    {
    }

    /**
     * Initialise encoder and decoder states. Must be called once before
     * any encode()/decode() call.
     */
    void init();

    /**
     * Release the vocoder states.
     */
    void terminate();

    /**
     * Encode 160 PCM samples (20 ms at 8 kHz) into one 88-bit IMBE frame.
     *
     * @param speech: 160 signed 16-bit samples, 8 kHz mono.
     * @param bits: output buffer, 11 bytes.
     */
    void encode(const int16_t *speech, uint8_t *bits);

    /**
     * Decode one 88-bit IMBE frame into 160 PCM samples.
     *
     * @param bits: 11 IMBE bytes.
     * @param speech: output buffer, 160 signed 16-bit samples.
     */
    void decode(const uint8_t *bits, int16_t *speech);

private:
    /*
     * Worst-case engine state sizes (imbe_embedded measurements: encoder
     * 8608 B, decoder 1932 B); checked against the runtime-reported sizes
     * at init.
     */
    static constexpr size_t ENC_STATE_SIZE = 8704;
    static constexpr size_t DEC_STATE_SIZE = 2048;

    alignas(8) uint8_t encState[ENC_STATE_SIZE];
    alignas(8) uint8_t decState[DEC_STATE_SIZE];
    bool initialised;
};

} /* P25 */

#endif /* P25_MODEM_VOCODER_H */
