/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <cstdlib>
#include <cmath>
#include "protocols/YSF/Demodulator.hpp"

using namespace YSF;

constexpr std::array<float, 3> Demodulator::sfNum;
constexpr std::array<float, 3> Demodulator::sfDen;

static std::array<int8_t, SYNC_SYMBOLS> syncSymbols()
{
    std::array<int8_t, SYNC_SYMBOLS> s;
    for (size_t i = 0; i < SYNC_SYMBOLS; i++)
        s[i] = SYNC_SYMBOLS_VALUES[i];
    return s;
}

static inline void setSymbol(frame_t &frame, const size_t i,
                             const int8_t symbol)
{
    // +3 = 01, +1 = 00, -1 = 10, -3 = 11
    uint8_t dibit;
    switch (symbol) {
        case +3:
            dibit = 0x01;
            break;
        case +1:
            dibit = 0x00;
            break;
        case -1:
            dibit = 0x02;
            break;
        default:
            dibit = 0x03;
            break;
    }

    size_t byte = i / 4;
    uint8_t shift = 6 - (2 * (i % 4));
    frame[byte] = (frame[byte] & ~(0x03 << shift)) | (dibit << shift);
}

Demodulator::Demodulator()
    : rrc(rrc_taps_24k), sync(syncSymbols()), sampleFilter(sfNum, sfDen)
{
    reset();
}

void Demodulator::reset()
{
    state = State::INIT;
    demodIdx = 0;
    newFrame = false;
    inverted = false;
    mu = 0.0f;
    skipNext = false;
    hist[0] = hist[1] = hist[2] = hist[3] = 0;
    frameIndex = 0;
    sampleIndex = 0;
    samplingPoint = 0;
    missedSyncs = 0;
    framesInLock = 0;
    syncErrors = 0;
    initCount = RX_SAMPLE_RATE / 50; // 20 ms for the filters
    corrThreshold = 0.0f;
    frames[0].fill(0);
    frames[1].fill(0);

    dsp_resetState(dcBlock);
    rrc.reset();
    correlator.reset();
    sync.reset();
    sampleFilter.reset();
    devEstimator.init({ 0, 0 });
}

const frame_t &Demodulator::getFrame()
{
    newFrame = false;
    return frames[demodIdx ^ 1];
}

bool Demodulator::isLocked() const
{
    return (state == State::LOCKED) || (state == State::SYNC_UPDATE);
}

bool Demodulator::sample(const int16_t rawSample)
{
    int16_t dc = dsp_dcBlockFilter(&dcBlock, rawSample);
    float in = static_cast<float>(dc);
    if (inverted)
        in = -in;
    int16_t smp = static_cast<int16_t>(rrc(in));

    correlator.sample(smp);
    corrThreshold = sampleFilter(std::abs(smp));

    hist[0] = hist[1];
    hist[1] = hist[2];
    hist[2] = hist[3];
    hist[3] = smp;

    switch (state) {
        case State::INIT:
            if (initCount == 0)
                state = State::UNLOCKED;
            else
                initCount -= 1;
            break;

        case State::UNLOCKED:
            unlockedState();
            break;

        case State::SYNCED:
            syncedState();
            break;

        case State::LOCKED:
            lockedState(smp);
            break;

        case State::SYNC_UPDATE:
            syncUpdateState();
            break;
    }

    sampleIndex = (sampleIndex + 1) % SAMPLES_PER_SYMBOL;

    return newFrame;
}

void Demodulator::unlockedState()
{
    /*
     * A clean sync word correlates to 44 times the outer symbol level (14
     * outer and 6 inner symbols), while the filtered |sample| of random
     * symbols sits at about 2/3 of the outer level: trigger at about 88% of
     * the ideal peak.
     */
    int32_t thresh = static_cast<int32_t>(corrThreshold * 58.0f);
    int8_t sign = sync.update(correlator, thresh, -thresh);
    if (sign != 0) {
        samplingPoint = sync.samplingIndex();

        // A negative peak is the sync word of an inverted signal: flip the
        // signal from now on, and the samples already in the correlator.
        if (sign < 0)
            inverted = !inverted;

        state = State::SYNCED;
    }
}

void Demodulator::syncedState()
{
    // The sync word samples, oldest first, from the correlator memory
    int16_t syms[SYNC_SYMBOLS];
    size_t n = 0;
    const int16_t *data = correlator.data();
    for (size_t i = 0; (i < SYNC_SAMPLES) && (n < SYNC_SYMBOLS); i++) {
        size_t pos = (correlator.index() + i) % SYNC_SAMPLES;
        if ((pos % SAMPLES_PER_SYMBOL) == samplingPoint) {
            // Samples taken before the polarity decision are not flipped
            int16_t v = data[pos];
            syms[n++] = inverted ? -v : v;
        }
    }

    // Outer deviation from the sync word's +-3 symbols only
    int32_t pos = 0, neg = 0, np = 0, nn = 0;
    for (size_t i = 0; i < n; i++) {
        if (SYNC_SYMBOLS_VALUES[i] == +3) {
            pos += syms[i];
            np++;
        }
        if (SYNC_SYMBOLS_VALUES[i] == -3) {
            neg += syms[i];
            nn++;
        }
    }

    if ((n < SYNC_SYMBOLS) || (np == 0) || (nn == 0) || (pos <= 0)
        || (neg >= 0)) {
        state = State::UNLOCKED;
        inverted = false;
        return;
    }

    devEstimator.init({ pos / np, neg / nn });

    frame_t &frame = frames[demodIdx];
    frameIndex = 0;
    for (size_t i = 0; i < SYNC_SYMBOLS; i++)
        quantize(syms[i]);

    uint8_t errs = syncBitErrors(frame);
    if (errs <= SYNC_ERRORS_ACQ) {
        state = State::LOCKED;
        missedSyncs = 0;
        framesInLock = 0;
        syncErrors = errs;
        mu = 0.0f;
        skipNext = false;
        prevValue = 0.0f;
        prevSymbol = 0;
    } else {
        state = State::UNLOCKED;
        inverted = false;
    }

    /*
     * The correlator is never reset outside reset(): its buffer index sets
     * the sampling phase the synchronizer reports, which must stay in step
     * with sampleIndex.  After a polarity change it holds samples of both
     * signs for one sync word length, which only delays a new acquisition.
     */
}

int8_t Demodulator::quantize(const int16_t sample)
{
    auto outer = devEstimator.outerDeviation();
    int8_t symbol;

    if (sample > (2 * outer.first) / 3)
        symbol = +3;
    else if (sample < (2 * outer.second) / 3)
        symbol = -3;
    else if (sample > devEstimator.zeroOffset())
        symbol = +1;
    else
        symbol = -1;

    setSymbol(frames[demodIdx], frameIndex, symbol);
    frameIndex += 1;
    return symbol;
}

void Demodulator::lockedState(const int16_t sample)
{
    (void)sample;

    // Symbols are taken two samples after their base sampling point, from
    // the four samples around it (see trackTiming)
    if (sampleIndex != ((samplingPoint + 2) % SAMPLES_PER_SYMBOL))
        return;

    if (skipNext) {
        skipNext = false;
        return;
    }

    int16_t value = interpolate();
    int8_t symbol = quantize(value);
    devEstimator.sample(value);
    trackTiming(value, symbol);

    if (frameIndex == FRAME_SYMBOLS) {
        demodIdx ^= 1; // the completed frame becomes the ready one
        frameIndex = 0;
        newFrame = true;
        state = State::SYNC_UPDATE;
    }
}

void Demodulator::syncUpdateState()
{
    /*
     * Frames are aligned on their own sync word, so the frame just completed
     * starts with the sync that preceded its 460 other symbols.  The first
     * frame of a lock holds the acquisition sync; from the second one on, a
     * sync word with too many errors is a miss.  A lock just acquired is
     * dropped on its first miss (noise can form one sync word, rarely two),
     * a verified one after MAX_MISSED in a row.
     */
    const frame_t &ready = frames[demodIdx ^ 1];
    uint8_t errs = syncBitErrors(ready);
    bool valid = (errs <= SYNC_ERRORS);
    syncErrors = errs;

    if (framesInLock < 255)
        framesInLock += 1;

    if (valid) {
        devEstimator.update();
        missedSyncs = 0;
        state = State::LOCKED;
        return;
    }

    devEstimator.reinit();
    missedSyncs += 1;

    if ((framesInLock <= 2) || (missedSyncs >= MAX_MISSED)) {
        state = State::UNLOCKED;
        inverted = false;
    } else {
        state = State::LOCKED;
    }
}

/*
 * Symbol timing.  With a roll-off of 0.2 the eye is narrow: sampling a tenth
 * of a symbol off centre already costs bit errors, so whole-sample timing at
 * 5 samples per symbol is not enough (the M17 zero-crossing clock recovery
 * left frames with bit errors at 100 ppm of clock error, and a per-frame
 * estimate from the sync word was too noisy).  The sampling instant is kept as
 * a base sample plus a fraction mu, each symbol is interpolated (cubic, over
 * the samples base-1 .. base+2) two samples after its base, and the instant is
 * tracked on every symbol by a decision-directed Mueller and Mueller detector.
 */
int16_t Demodulator::interpolate() const
{
    // hist[0..3] = samples at base-1, base, base+1, base+2
    float x = mu;
    float p0 = hist[0], p1 = hist[1], p2 = hist[2], p3 = hist[3];
    float a = -p0 / 6.0f + p1 / 2.0f - p2 / 2.0f + p3 / 6.0f;
    float b = p0 / 2.0f - p1 + p2 / 2.0f;
    float c = -p0 / 3.0f - p1 / 2.0f + p2 - p3 / 6.0f;
    float v = ((a * x + b) * x + c) * x + p1;
    return static_cast<int16_t>(v);
}

void Demodulator::trackTiming(const int16_t value, const int8_t symbol)
{
    auto outer = devEstimator.outerDeviation();
    float scale = 0.5f * static_cast<float>(outer.first - outer.second);
    if (scale <= 0.0f)
        return;

    // Value in symbol units (+-1, +-3); the error is about -2 per sample of
    // lateness, averaged over random data.
    float y = 3.0f * static_cast<float>(value) / scale;
    float e = (static_cast<float>(prevSymbol) * y)
            - (static_cast<float>(symbol) * prevValue);
    prevValue = y;
    prevSymbol = symbol;

    if (e > 10.0f)
        e = 10.0f;
    if (e < -10.0f)
        e = -10.0f;
    mu += TIMING_GAIN * e;

    // Move the base when the fraction leaves [0, 1).  Moved later, the new
    // base + 2 comes round again before the next symbol's: skip it once.
    if (mu >= 1.0f) {
        mu -= 1.0f;
        samplingPoint = (samplingPoint + 1) % SAMPLES_PER_SYMBOL;
        skipNext = true;
    } else if (mu < 0.0f) {
        mu += 1.0f;
        samplingPoint = (samplingPoint + SAMPLES_PER_SYMBOL - 1)
                      % SAMPLES_PER_SYMBOL;
    }
}

uint8_t Demodulator::syncBitErrors(const frame_t &frame) const
{
    uint8_t errs = 0;
    for (size_t i = 0; i < SYNC_BYTES; i++)
        errs += __builtin_popcount(frame[i] ^ SYNC_WORD[i]);
    return errs;
}
