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
 * path), one 100 ms YSF frame at a time.  YSF deviates ±2700 Hz for the
 * outer symbols (MMDVM's YSF levels, 1893 against DMR's 1362 for ±1944 Hz).
 * The M17 filter (roll-off 0.5) is kept: into YSF's roll-off 0.2 receive
 * filter it leaves less intersymbol interference than a truncated roll-off
 * 0.2 one, and its lower peaks leave room for the full deviation.
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
    /*
     * Measured off air on a CS7000-PLUS: three quarters of this gave
     * ±2060 Hz for a run of outer symbols, so this is about ±2700 Hz.  The
     * peaks of the filtered baseband reach 1.46 times the outer symbol
     * level, 31300 here: just inside the 16-bit range.
     */
    static constexpr float RRC_GAIN = 23000.0f * 0.98f;

    std::array<int8_t, FRAME_SYMBOLS> symbols;
    std::unique_ptr<int16_t[]> basebandBuffer;
    stream_sample_t *idleBuffer;
    streamId outStream;
    pathId outPath;
    bool txRunning;
    bool invPhase;
};

} /* YSF */

#endif /* YSF_MODULATOR_H */
