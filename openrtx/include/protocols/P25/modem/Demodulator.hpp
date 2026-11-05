/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef P25_MODEM_DEMODULATOR_H
#define P25_MODEM_DEMODULATOR_H

#ifndef __cplusplus
#error This header is C++ only!
#endif

#include "core/iir.hpp"
#include <cstdint>
#include <cstddef>
#include <memory>
#include <array>
#include <cmath>
#include "core/audio_path.h"
#include "core/audio_stream.h"
#include "protocols/P25/modem/Constants.hpp"
#include "protocols/P25/modem/Correlator.hpp"
#include "protocols/P25/modem/Synchronizer.hpp"
#include "protocols/P25/modem/DevEstimator.hpp"

namespace P25
{

/**
 * P25 Phase 1 C4FM demodulator, M17-style architecture.
 *
 * The discriminator baseband is sampled at 24 kHz (5 samples/symbol),
 * passed through the TIA-102 receive filter (one-symbol integrate and
 * dump, which pairs with the transmitter's x/sin(x) shaping to a Nyquist
 * raised cosine) and sliced at the symbol centres.
 *
 * Because the integrate-and-dump window has hard edges, a timing error of
 * half a sample (10% of a symbol) already closes the inner eye: symbol
 * timing is therefore estimated with sub-sample resolution from the frame
 * sync correlation (parabolic fit of the peak across phases), at
 * acquisition and again at every validated frame sync, and the filtered
 * baseband is read at the fractional instant with a cubic interpolator.
 */
class Demodulator
{
public:
    /**
     * Constructor.
     */
    Demodulator();

    /**
     * Destructor.
     */
    ~Demodulator();

    /**
     * Allocate buffers for baseband signal sampling and initialise demodulator.
     */
    void init();

    /**
     * Shutdown demodulator and deallocate data buffers.
     */
    void terminate();

    /**
     * Starts the sampling of the baseband signal in a double buffer.
     */
    void startBasebandSampling();

    /**
     * Stops the sampling of the baseband signal in a double buffer.
     */
    void stopBasebandSampling();

    /**
     * Returns the last data unit decoded from the baseband signal. Only the
     * first getFrameLength() bytes are meaningful: units are of different
     * lengths, see Nid::frameBytes().
     *
     * @return reference to the internal data structure containing the last
     * decoded frame.
     */
    const frame_t &getFrame();

    /**
     * @return length in bytes of the last decoded data unit.
     */
    size_t getFrameLength()
    {
        return readyBytes;
    }

    /**
     * @return data unit ID of the last decoded data unit.
     */
    uint8_t getDuid()
    {
        return readyDuid;
    }

    /**
     * @return network access code carried by the last decoded data unit.
     */
    uint16_t getNac()
    {
        return readyNac;
    }

    /**
     * Demodulates data from the ADC and fills the idle frame.
     * Everytime this function is called a whole ADC buffer is consumed.
     *
     * @param invertPhase: invert the phase of the baseband signal before decoding.
     * @return true if a new frame has been fully decoded.
     */
    bool update(const bool invertPhase = false);

    /**
     * Process a single baseband sample through the demodulation chain.
     *
     * @param rawSample: signed 16-bit baseband sample.
     * @param invertPhase: invert the phase of the sample before decoding.
     * @return true if a new frame has been fully decoded and not yet read
     * with getFrame().
     */
    bool sample(int16_t rawSample, bool invertPhase = false);

    /**
     * @return true if a demodulator is locked on a P25 stream.
     */
    bool isLocked();

private:
    /**
     * Quantize a given sample to its corresponding symbol and append it to
     * the ongoing frame.
     *
     * @param sample: baseband sample at the symbol centre.
     */
    void quantize(const float sample);

    /**
     * Reset the demodulator state.
     */
    void reset();

    /**
     * State handler function for DemodState::UNLOCKED: search for the P25
     * frame sync to acquire a lock on.
     */
    void unlockedState();

    /**
     * State handler function for DemodState::LOCKED: read symbols at the
     * estimated centres, re-validate and re-time on every frame sync,
     * decode the NID to learn the unit length, deliver complete units.
     */
    void lockedState();

    /**
     * Drop the lock and go back to hunting for a frame sync.
     */
    void unlock();

    /**
     * Refine the symbol timing from the frame sync correlations gathered
     * around the sampling instant of the last frame sync symbol.
     */
    void refineTiming();

    /**
     * Set the symbol timing from an estimated symbol centre.
     *
     * @param centre: absolute sample count of a symbol centre, as a
     * fractional value.
     */
    void setTiming(const float centre);

    /**
     * Baseband sampled at 24 kHz, half of an LDU per buffer.
     */
    static constexpr size_t RX_SAMPLE_RATE = 24000;
    static constexpr size_t SAMPLES_PER_SYMBOL = RX_SAMPLE_RATE / SYMBOL_RATE;
    static constexpr size_t FRAME_SAMPLES = FRAME_SYMBOLS * SAMPLES_PER_SYMBOL;
    static constexpr size_t SAMPLE_BUF_SIZE = FRAME_SAMPLES / 2;
    static constexpr size_t SYNCWORD_SAMPLES = SAMPLES_PER_SYMBOL
                                             * SYNCWORD_SYMBOLS;

    /*
     * The NID occupies frame bits 48 to 113 (64 bits plus the status symbol
     * at bits 70 and 71): it is complete once 57 symbols are in.
     */
    static constexpr size_t NID_END_SYMBOLS = 57;

    /**
     * Internal state of the demodulator.
     */
    enum class DemodState {
        INIT,     ///< Initializing
        UNLOCKED, ///< Not locked, searching for a frame sync
        LOCKED    ///< Locked
    };

    /**
     * Coefficients of the sample filter
     */
    static constexpr std::array<float, 3> sfNum = { 4.24433681e-05f,
                                                    8.48867363e-05f,
                                                    4.24433681e-05f };
    static constexpr std::array<float, 3> sfDen = { 1.0f, -1.98148851f,
                                                    0.98165828f };

    /*
     * Frame sync detection threshold, in units of the filtered mean
     * |sample|. For equiprobable symbols the mean |sample| is 2/3 of the
     * outer level, so the ideal FS correlation peak (24 outer symbols,
     * weights +-3) is 24 * 3 * 1.5 = 108 times that mean; trigger at
     * half of it. The Hamming-distance check on the next frame sync gates
     * the lock after the trigger.
     */
    static constexpr float SYNC_THRESHOLD = 0.5f * 4.5f * SYNCWORD_SYMBOLS;

    /*
     * Discriminator DC estimate: running mean for the first DC_FAST_SAMPLES
     * after a reset (immediate convergence on the ADC pedestal), then a
     * slow one-pole tracker. The audio DC blocker is unsuitable here: its
     * 8 ms time constant follows symbol runs and closes the inner eye.
     */
    static constexpr uint32_t DC_FAST_SAMPLES = RX_SAMPLE_RATE / 10;
    static constexpr float DC_SLOW_ALPHA = 1.0e-4f;

    DemodState demodState; ///< Demodulator state
    std::unique_ptr<int16_t[]>
        baseband_buffer;   ///< Buffer for baseband audio handling.
    streamId basebandId;   ///< Id of the baseband input stream.
    pathId basebandPath;   ///< Id of the baseband input path.
    std::unique_ptr<frame_t> demodFrame; ///< Frame being demodulated.
    std::unique_ptr<frame_t>
        readyFrame;        ///< Fully demodulated frame to be returned.
    bool newFrame;         ///< A new frame has been fully decoded.
    uint16_t frameIndex;   ///< Index for filling the raw frame.
    uint16_t frameSymbols; ///< Length of the unit being received, symbols
    uint8_t frameDuid;     ///< DUID of the unit being received
    uint16_t frameNac;     ///< NAC of the unit being received
    size_t readyBytes;     ///< Length of the delivered unit, bytes
    uint8_t readyDuid;     ///< DUID of the delivered unit
    uint16_t readyNac;     ///< NAC of the delivered unit
    uint32_t sampleCount;  ///< Free-running sample counter
    uint32_t symbolBase;   ///< Sample count of the next symbol's base instant
    float symbolFrac;      ///< Symbol centre offset from the base instant
    uint8_t missedSyncs;   ///< Missed syncs left before the lock is lost
    bool lastSyncValid;    ///< The last frame sync was valid
    uint32_t initCount;    ///< Downcounter for initialization
    float corrThreshold;   ///< Correlation threshold
    float dcEstimate;      ///< Discriminator DC estimate
    uint32_t dcCount;      ///< Samples seen by the DC estimate since reset
    std::array<int16_t, 4> history; ///< Last filtered samples, newest last
    std::array<int32_t, SAMPLES_PER_SYMBOL> fsCorr; ///< FS correlations

    Correlator<SYNCWORD_SYMBOLS, SAMPLES_PER_SYMBOL> correlator;
    Synchronizer<SYNCWORD_SYMBOLS, SAMPLES_PER_SYMBOL> fsSync{
        std::array<int8_t, SYNCWORD_SYMBOLS>(FS_SYNC_WORD)
    };
    Iir<3> sampleFilter{ sfNum, sfDen };
    DevEstimator devEstimator;
};

} /* P25 */

#endif /* P25_MODEM_DEMODULATOR_H */
