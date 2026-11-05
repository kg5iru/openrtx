/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef P25_MODEM_MODULATOR_H
#define P25_MODEM_MODULATOR_H

#ifndef __cplusplus
#error This header is C++ only!
#endif

#include "core/audio_stream.h"
#include "protocols/P25/modem/PwmCompensator.hpp"
#include "protocols/P25/modem/Constants.hpp"
#include "core/audio_path.h"
#include <cstdint>
#include <memory>
#include <array>

namespace P25
{

/**
 * Modulator device for P25 Phase 1 (C4FM), M17-style architecture:
 * 4FSK symbols zero-stuffed at 10 samples/symbol, shaped by the TIA-102
 * C4FM transmit filter at 48 kHz (Nyquist raised cosine, beta = 0.2,
 * times the x/sin(x) shaping filter that pre-compensates the receiver's
 * integrate-and-dump), optionally compensated for the MDx PWM output
 * stage.
 */
class Modulator
{
public:
    /**
     * Constructor.
     */
    Modulator();

    /**
     * Destructor.
     */
    ~Modulator();

    /**
     * Allocate buffers for baseband audio generation and initialise modulator.
     */
    void init();

    /**
     * Forcefully shutdown modulator and deallocate data buffers.
     */
    void terminate();

    /**
     * Start the modulator.
     *
     * @return true if the modulator has been successfully started.
     */
    bool start();

    /**
     * Transmission preamble patterns. All are zero-mean; TIA is the
     * alternating +3/-3 preamble, MMDVM the repeated 0x77 octets (+1/-3),
     * Kenwood the official +3,+3,-3,-3 square wave and the sweep visits
     * all four symbol levels.
     */
    static constexpr int8_t PREAMBLE_TIA[2] = { +3, -3 };
    static constexpr int8_t PREAMBLE_MMDVM[2] = { +1, -3 };
    static constexpr int8_t PREAMBLE_KENWOOD[4] = { +3, +3, -3, -3 };
    static constexpr int8_t PREAMBLE_SWEEP[4] = { +3, +1, -1, -3 };

    /**
     * Send a transmission preamble made of symbolCount symbols from the
     * given pattern.
     */
    void sendPreamble(const int8_t *pattern, size_t patternLen,
                      size_t symbolCount);

    /**
     * Send a 180 ms transmission preamble (alternating +3/-3 symbols).
     */
    void sendPreamble();

    /**
     * Generate and transmit the baseband signal obtained by 4FSK modulation
     * of a data unit. Units of any length are accepted (HDU 99 bytes, LDU
     * 216, TDU 18 ...): symbols are queued and the baseband is emitted in
     * fixed chunks, so the stream across consecutive units is continuous.
     * The last partial chunk is emitted, padded, by stop().
     *
     * @param data: raw frame bytes, frame sync first.
     * @param bytes: frame length.
     */
    void sendFrame(const uint8_t *data, const size_t bytes);

    /**
     * Send one 216-byte frame (an LDU).
     *
     * @param frame: P25 frame to be sent.
     */
    void sendFrame(const frame_t &frame)
    {
        sendFrame(frame.data(), frame.size());
    }

    /**
     * Terminate baseband transmission.
     */
    void stop();

    /**
     * Invert baseband signal phase before output.
     *
     * @param status: if set to true signal phase is inverted.
     */
    void invertPhase(const bool status);

    /** Set an optional sample sink for host-side testing, before start(). */
    using SampleSink = void (*)(const int16_t *, size_t, void *);
    void setSampleSink(SampleSink sink, void *context)
    {
        sampleSink = sink;
        sinkContext = context;
    }

    static constexpr size_t TX_SAMPLE_RATE = 48000;
    static constexpr size_t SAMPLES_PER_SYMBOL = TX_SAMPLE_RATE / SYMBOL_RATE;
    static constexpr size_t FRAME_SAMPLES = FRAME_SYMBOLS * SAMPLES_PER_SYMBOL;
    static constexpr size_t CHUNK_SYMBOLS = FRAME_SYMBOLS / 2;
    static constexpr size_t CHUNK_SAMPLES = FRAME_SAMPLES / 2;

    /*
     * Output scaling. The outer level is the baseband level, in stream
     * counts, of a steady run of +3 symbols: the +1800 Hz outer deviation
     * as seen at a receiver's integrate-and-dump output. Measured on air
     * (CS7000P, HR_C6000 LINE_IN, deviation coefficient 0x25): 17280
     * counts radiate 1948 Hz, i.e. 8.9 counts/Hz, so spec deviation is
     * ~16000 counts on that path.
     *
     * C4FM transitions overshoot the outer level by up to 2.44x, so the
     * peaks saturate above ~13400 counts. Measured and simulated, this is
     * harmless at spec deviation: 0.1% of the samples clip, the symbol
     * clusters through the TIA receive filter are as tight as a factory
     * radio's and the adjacent-channel power is unchanged. setLevel()
     * sets the level at run time from the TX level setting.
     */
    static constexpr float DEFAULT_OUTER_LEVEL = 16000.0f;

    /**
     * Set the outer symbol level of the generated baseband.
     *
     * @param outerLevel: steady +3 level, in stream counts.
     */
    void setLevel(const float outerLevel)
    {
        pulseGain = outerLevel * SAMPLES_PER_SYMBOL / 3.0f;
    }

private:
    /**
     * Queue one symbol; emits a chunk of baseband whenever CHUNK_SYMBOLS
     * symbols are pending.
     */
    void queueSymbol(const int8_t symbol);

    /**
     * Shape the pending chunk of symbols into baseband and hand it to the
     * output stage. Chunks are half an LDU long to bound the double-buffer
     * size (full-LDU double buffering would cost 34 kB on RAM-constrained
     * targets).
     */
    void sendChunk();

    std::array<int8_t, CHUNK_SYMBOLS> symbols; ///< Pending symbols
    size_t pendingSymbols;                     ///< Number of pending symbols
    std::unique_ptr<int16_t[]>
        baseband_buffer;         ///< Buffer for baseband audio handling.
    stream_sample_t *idleBuffer; ///< Half baseband buffer, free for processing.
    streamId outStream;          ///< Baseband output stream ID.
    pathId outPath;              ///< Baseband output path ID.
    bool txRunning;              ///< Transmission running.
    bool invPhase;               ///< Invert signal phase
    float pulseGain = DEFAULT_OUTER_LEVEL * SAMPLES_PER_SYMBOL / 3.0f;
    SampleSink sampleSink = nullptr;
    void *sinkContext = nullptr;

#if defined(PLATFORM_MD3x0) || defined(PLATFORM_MDUV3x0)
    PwmCompensator pwmComp;
#endif
};

} /* P25 */

#endif /* P25_MODEM_MODULATOR_H */
