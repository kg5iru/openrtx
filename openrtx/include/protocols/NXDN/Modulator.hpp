/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef NXDN_MODULATOR_H
#define NXDN_MODULATOR_H

#ifndef __cplusplus
#error This header is C++ only!
#endif

#include <cstdint>
#include <memory>
#include <array>
#include "core/audio_path.h"
#include "core/audio_stream.h"
#include "protocols/NXDN/Constants.hpp"

namespace NXDN
{

/**
 * NXDN48 baseband generator: 2400 symbols/s, shaped as MMDVM shapes them
 * (see DSP.hpp), 48 kHz output through the RTX audio path, one 80 ms frame
 * at a time, outer symbols at +-1050 Hz.  Same structure as the YSF
 * modulator.
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

    /** Send one frame worth (80 ms) of preamble: alternating outer symbols. */
    void sendPreamble();

    /**
     * Modulate and send one frame; returns once the previous one has gone
     * out (about every 80 ms).
     */
    void sendFrame(const frame_t &frame);

    /**
     * Send one frame worth of unmodulated carrier, that first lets the last
     * symbols out of the pulse shaping filter (6 symbols, 2.5 ms).  To be
     * sent after the last frame of a transmission, before stop().
     */
    void sendFlush();

    /** Stop the baseband stream and release the output path. */
    void stop();

    /**
     * Invert the baseband polarity, for radios whose modulator inverts it.
     *
     * @param status: true to invert.
     */
    void invertPhase(const bool status);

private:
    void symbolsToBaseband();
    void sendBaseband();

    static constexpr size_t TX_SAMPLE_RATE = 48000;
    static constexpr size_t SAMPLES_PER_SYMBOL = TX_SAMPLE_RATE / SYMBOL_RATE;
    static constexpr size_t FRAME_SAMPLES = FRAME_SYMBOLS * SAMPLES_PER_SYMBOL;
    static constexpr size_t FILTER_SYMBOLS = 12;
    /*
     * Baseband units of the +3 symbol: 1050 Hz at the 0.1257 Hz per unit
     * measured off air for the YSF mode (same path and modulation factor).
     * The shaped baseband peaks at 1.58 times this.
     */
    static constexpr float OUTER_LEVEL = 8353.0f;

    std::array<int8_t, FRAME_SYMBOLS> symbols;
    std::array<int8_t, FILTER_SYMBOLS> history; ///< history[0] = newest
    std::unique_ptr<int16_t[]> basebandBuffer;
    stream_sample_t *idleBuffer;
    streamId outStream;
    pathId outPath;
    bool txRunning;
    bool invPhase;
};

} /* NXDN */

#endif /* NXDN_MODULATOR_H */
