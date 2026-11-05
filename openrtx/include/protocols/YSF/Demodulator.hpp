/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef YSF_DEMODULATOR_H
#define YSF_DEMODULATOR_H

#ifndef __cplusplus
#error This header is C++ only!
#endif

#include <cstdint>
#include <cstddef>
#include <cstdlib>
#include <array>
#include "core/fir.hpp"
#include "core/iir.hpp"
#include "protocols/M17/Correlator.hpp"
#include "protocols/M17/Synchronizer.hpp"
#include "protocols/M17/DevEstimator.hpp"
#include "protocols/YSF/Constants.hpp"
#include "protocols/YSF/DSP.hpp"

namespace YSF
{

/**
 * C4FM demodulator for YSF, on baseband samples at 24 kHz (5 per symbol).
 *
 * Built from the M17 demodulator's parts (same symbol rate): DC block, RRC
 * matched filter (roll-off 0.2 here), sync word correlation and a deviation
 * estimate.  Symbols are interpolated between samples, at an instant tracked
 * on every symbol (see trackTiming).  Either polarity is accepted:
 * the sign of the sync correlation peak sets it for the rest of the lock.
 *
 * Output: whole 120-byte frames, sync word included, aligned on the sync.
 * The lock is held while the sync word recurs every 480 symbols with at most
 * SYNC_ERRORS bit errors, and dropped after MAX_MISSED syncs missed in a row
 * (the first one, for a lock just acquired).  While locked, a sync word
 * found at another position moves the lock there at once.  Checking that the
 * contents make sense (FICH CRC) is up to the caller.
 */
class Demodulator
{
public:
    Demodulator();

    ~Demodulator()
    {
    }

    /** Back to the initial, unlocked state. */
    void reset();

    /**
     * Process one baseband sample.
     *
     * @param sample: raw baseband sample.
     * @return true when a new frame is ready.
     */
    bool sample(const int16_t sample);

    /**
     * Get the last complete frame; it stays available until the next one.
     */
    const frame_t &getFrame();

    /** @return true while locked on a signal. */
    bool isLocked() const;

    /** @return true if the current lock is on an inverted signal. */
    bool isInverted() const
    {
        return inverted;
    }

    /** @return the symbol sampling point, 0 .. SAMPLES_PER_SYMBOL - 1. */
    uint32_t samplingPhase() const
    {
        return samplingPoint;
    }

    /** @return the fractional part of the sampling instant, 0 .. 1. */
    float samplingFraction() const
    {
        return mu;
    }

    /** @return bit errors in the sync word of the last frame. */
    uint8_t lastSyncErrors() const
    {
        return syncErrors;
    }

    /**
     * @return how many times a lock moved to a sync word found at another
     * position (see lockedState), since the last reset().
     */
    uint32_t realignCount() const
    {
        return realigns;
    }

    static constexpr size_t RX_SAMPLE_RATE = 24000;
    static constexpr size_t SAMPLES_PER_SYMBOL = RX_SAMPLE_RATE / SYMBOL_RATE;
    static constexpr uint8_t SYNC_ERRORS_ACQ = 2; ///< To acquire a lock
    static constexpr uint8_t SYNC_ERRORS = 5;     ///< To keep it
    static constexpr uint8_t MAX_MISSED = 4;      ///< Syncs, 400 ms
    static constexpr float DC_GAIN = 1.0f / 2048.0f;
    static constexpr float TIMING_GAIN = 0.01f;

private:
    enum class State { INIT, UNLOCKED, SYNCED, LOCKED, SYNC_UPDATE };

    void unlockedState();
    void syncedState();
    void lockedState(const int16_t sample);
    void syncUpdateState();
    void symbolStep();
    void syncFailed();
    void trackTiming(const int16_t value, const int8_t symbol);
    int16_t interpolate() const;
    int8_t quantize(const int16_t sample);
    uint8_t syncBitErrors(const frame_t &frame) const;

    static constexpr size_t SYNC_SAMPLES = SYNC_SYMBOLS * SAMPLES_PER_SYMBOL;

    // Low pass filter of |sample|, for the correlation threshold (as M17)
    static constexpr std::array<float, 3> sfNum = { 4.24433681e-05f,
                                                    8.48867363e-05f,
                                                    4.24433681e-05f };
    static constexpr std::array<float, 3> sfDen = { 1.0f, -1.98148851f,
                                                    0.98165828f };

    State state;
    frame_t frames[2];
    uint8_t demodIdx; ///< frames[demodIdx] is being filled
    bool newFrame;
    bool inverted;
    uint16_t frameIndex;  ///< Symbols written to the frame being filled
    uint32_t sampleIndex; ///< 0 .. SAMPLES_PER_SYMBOL - 1
    uint32_t samplingPoint;
    uint8_t missedSyncs;
    uint8_t framesInLock; ///< Frames completed since the lock
    uint8_t syncErrors;
    bool realigning;      ///< SYNCED entered from a lock, not from UNLOCKED
    uint32_t realigns;
    /// The lock as it was before a realignment attempt, restored if the
    /// sync word found does not verify
    struct {
        frame_t frame;
        uint16_t frameIndex;
        uint32_t samplingPoint;
        uint8_t missedSyncs;
        float mu;
        bool skipNext;
        float prevValue;
        int8_t prevSymbol;
        DevEstimator devEstimator;
    } held;
    uint32_t initCount;
    float corrThreshold;
    float dcLevel; ///< Slow DC estimate, see sample()

    Fir<std::tuple_size<decltype(rrc_taps_24k)>::value> rrc;
    Correlator<SYNC_SYMBOLS, SAMPLES_PER_SYMBOL> correlator;
    Synchronizer<SYNC_SYMBOLS, SAMPLES_PER_SYMBOL> sync;
    Iir<3> sampleFilter;
    DevEstimator devEstimator;
    float prevValue; ///< Last symbol, in symbol units
    int8_t prevSymbol;
    int16_t hist[4]; ///< Last samples
    float mu;        ///< Timing fraction
    bool skipNext;   ///< See trackTiming
};

} /* YSF */

#endif /* YSF_DEMODULATOR_H */
