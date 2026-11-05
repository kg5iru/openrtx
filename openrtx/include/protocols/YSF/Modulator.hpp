/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef YSF_MODULATOR_H
#define YSF_MODULATOR_H

#ifndef __cplusplus
#error This header is C++ only!
#endif

#include <cstdint>
#include <memory>
#include <array>
#include "core/audio_path.h"
#include "core/audio_stream.h"
#include "protocols/YSF/Constants.hpp"

namespace YSF
{

/**
 * C4FM baseband generator: 4800 symbols/s, root raised cosine shaped, as
 * the M17 modulator (same filter, 48 kHz output through the RTX audio
 * path), one 100 ms YSF frame at a time.  YSF deviates ±1800 Hz for the
 * outer symbols, three quarters of M17's ±2400 Hz, so the baseband gain is
 * three quarters of M17's.
 */
class Modulator
{
public:
    Modulator();
    ~Modulator();

    /** Allocate the baseband buffers. */
    void init();

    /** Stop any transmission and free the buffers. */
    void terminate();

    /**
     * Open the baseband output path and stream.
     *
     * @return true on success.
     */
    bool start();

    /**
     * Send one frame worth (100 ms) of preamble: alternating outer symbols.
     */
    void sendPreamble();

    /**
     * Modulate and send one frame; returns once the previous one has gone
     * out (about every 100 ms).
     */
    void sendFrame(const frame_t &frame);

    /** Stop the baseband stream and release the output path. */
    void stop();

private:
    void symbolsToBaseband();
    void sendBaseband();

    static constexpr size_t TX_SAMPLE_RATE = 48000;
    static constexpr size_t SAMPLES_PER_SYMBOL = TX_SAMPLE_RATE / SYMBOL_RATE;
    static constexpr size_t FRAME_SAMPLES = FRAME_SYMBOLS * SAMPLES_PER_SYMBOL;
    static constexpr float RRC_GAIN = 23000.0f * 0.75f;

    std::array<int8_t, FRAME_SYMBOLS> symbols;
    std::unique_ptr<int16_t[]> basebandBuffer;
    stream_sample_t *idleBuffer;
    streamId outStream;
    pathId outPath;
    bool txRunning;
};

} /* YSF */

#endif /* YSF_MODULATOR_H */
