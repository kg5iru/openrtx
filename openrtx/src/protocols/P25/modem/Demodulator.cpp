/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "protocols/P25/modem/Demodulator.hpp"
#include "protocols/P25/modem/DSP.hpp"
#include "protocols/P25/modem/Utils.hpp"
#include "protocols/P25/modem/Nid.hpp"
#include "core/audio_stream.h"
#include "hwconfig.h"
#include <math.h>
#include <cstring>
#include <stdio.h>

using namespace P25;

Demodulator::Demodulator() : basebandId(-1), basebandPath(-1)
{
}

Demodulator::~Demodulator()
{
    terminate();
}

void Demodulator::init()
{
    /*
     * Allocate a chunk of memory to contain two complete buffers for baseband
     * audio. Split this chunk in two separate blocks for double buffering using
     * placement new.
     */

    baseband_buffer = std::make_unique<int16_t[]>(2 * SAMPLE_BUF_SIZE);
    demodFrame = std::make_unique<frame_t>();
    readyFrame = std::make_unique<frame_t>();

    reset();
}

void Demodulator::terminate()
{
    // Ensure proper termination of baseband sampling
    stopBasebandSampling();

    // Delete the buffers and deallocate memory.
    baseband_buffer.reset();
    demodFrame.reset();
    readyFrame.reset();
}

void Demodulator::startBasebandSampling()
{
    basebandPath = audioPath_request(SOURCE_RTX, SINK_MCU, PRIO_RX);
    basebandId = audioStream_start(basebandPath, baseband_buffer.get(),
                                   2 * SAMPLE_BUF_SIZE, RX_SAMPLE_RATE,
                                   STREAM_INPUT | BUF_CIRC_DOUBLE);

    reset();
}

void Demodulator::stopBasebandSampling()
{
    audioStream_terminate(basebandId);
    audioPath_release(basebandPath);

    // Stream slots are reused once released: forget both ids
    basebandId = -1;
    basebandPath = -1;
}

const frame_t &Demodulator::getFrame()
{
    // When a frame is read is not new anymore
    newFrame = false;
    return *readyFrame;
}

bool Demodulator::isLocked()
{
    return (demodState == DemodState::LOCKED);
}

bool Demodulator::sample(int16_t rawSample, bool invertPhase)
{
    // Discriminator DC removal, see DC_FAST_SAMPLES.
    if (dcCount < DC_FAST_SAMPLES) {
        dcCount += 1;
        dcEstimate += (rawSample - dcEstimate) / static_cast<float>(dcCount);
    } else {
        dcEstimate += DC_SLOW_ALPHA * (rawSample - dcEstimate);
    }

    float elem = static_cast<float>(rawSample) - dcEstimate;
    if (invertPhase)
        elem = 0.0f - elem;

    // Receive filter: TIA-102 integrate and dump.
    elem = rxf_24k(elem);
    if (elem > 32767.0f)
        elem = 32767.0f;
    if (elem < -32768.0f)
        elem = -32768.0f;
    int16_t sample = static_cast<int16_t>(elem);

    history[0] = history[1];
    history[1] = history[2];
    history[2] = history[3];
    history[3] = sample;

    correlator.sample(sample);
    corrThreshold = sampleFilter(std::abs(sample));

    switch (demodState) {
        case DemodState::INIT: {
            if (initCount == 0)
                demodState = DemodState::UNLOCKED;
            else
                initCount -= 1;
        } break;

        case DemodState::UNLOCKED:
            unlockedState();
            break;

        case DemodState::LOCKED:
            lockedState();
            break;
    }

    sampleCount += 1;

    return newFrame;
}

bool Demodulator::update(const bool invertPhase)
{
    // Audio path closed, nothing to do
    if (audioPath_getStatus(basebandPath) != PATH_OPEN)
        return false;

    // Read samples from the ADC
    dataBlock_t baseband = inputStream_getData(basebandId);
    if (baseband.data == NULL)
        return false;

    // Process samples
    for (size_t i = 0; i < baseband.len; i++)
        sample(baseband.data[i], invertPhase);

    return newFrame;
}

void Demodulator::quantize(const float sample)
{
    auto outerDeviation = devEstimator.outerDeviation();
    int8_t symbol;

    if (sample > (2.0f * outerDeviation.first) / 3.0f) {
        symbol = +3;
    } else if (sample < (2.0f * outerDeviation.second) / 3.0f) {
        symbol = -3;
    } else if (sample > devEstimator.zeroOffset()) {
        symbol = +1;
    } else {
        symbol = -1;
    }

    setSymbol(*demodFrame, frameIndex, symbol);
    frameIndex += 1;
}

void Demodulator::reset()
{
    frameIndex = 0;
    frameSymbols = 0;
    frameDuid = 0;
    frameNac = 0;
    readyBytes = 0;
    readyDuid = 0;
    readyNac = 0;
    sampleCount = 0;
    symbolBase = 0;
    symbolFrac = 0.0f;
    missedSyncs = 0;
    lastSyncValid = false;
    corrThreshold = 0.0f;
    newFrame = false;
    dcEstimate = 0.0f;
    dcCount = 0;
    demodState = DemodState::INIT;
    initCount = RX_SAMPLE_RATE / 50; // 50ms of init time

    history.fill(0);
    fsCorr.fill(0);
    rxf_24k.reset();
    correlator.reset();
    fsSync.reset();
    sampleFilter.reset();
    devEstimator.init({ 0, 0 });
}

void Demodulator::setTiming(const float centre)
{
    float base = floorf(centre);
    symbolBase = static_cast<uint32_t>(base);
    symbolFrac = centre - base;
}

void Demodulator::unlockedState()
{
    int32_t syncThresh = static_cast<int32_t>(corrThreshold * SYNC_THRESHOLD);

    // The P25 frame sync is polarity-rigid: only a positive correlation
    // peak is accepted (hardware polarity inversion is a platform flag).
    // The negative threshold is set to INT32_MIN to disable it.
    if (fsSync.update(correlator, syncThresh, INT32_MIN) == 0)
        return;

    /*
     * The synchronizer reports the phase of the correlation peak, which is
     * the base instant of the last frame sync symbol, and the fractional
     * position of the true peak relative to it. The peak lies within the
     * last few samples: recover its absolute sample count.
     */
    uint32_t phase = fsSync.samplingIndex();
    uint32_t peak =
        sampleCount
        - ((sampleCount + SAMPLES_PER_SYMBOL - phase) % SAMPLES_PER_SYMBOL);

    // Initial symbol levels from the frame sync, all outer symbols.
    devEstimator.init(correlator.maxDeviation(phase));

    // The frame sync is known: store it and start on the first NID symbol.
    for (size_t i = 0; i < FS_SYNC_BYTES.size(); i++)
        (*demodFrame)[i] = FS_SYNC_BYTES[i];
    frameIndex = SYNCWORD_SYMBOLS;

    setTiming(static_cast<float>(peak) + fsSync.fractionalOffset()
              + static_cast<float>(SAMPLES_PER_SYMBOL));

    // The unit length is known once the NID is decoded.
    frameSymbols = 0;
    missedSyncs = 1;
    lastSyncValid = true;
    fsSync.reset();
    demodState = DemodState::LOCKED;
}

void Demodulator::unlock()
{
    frameIndex = 0;
    frameSymbols = 0;
    fsSync.reset();
    demodState = DemodState::UNLOCKED;
}

void Demodulator::lockedState()
{
    /*
     * While the last frame sync symbol is being received, collect the FS
     * correlation at the five instants around its sampling base, for the
     * timing refinement done once the symbol is in.
     */
    if (frameIndex == (SYNCWORD_SYMBOLS - 1)) {
        int32_t pos = static_cast<int32_t>(sampleCount - symbolBase) + 2;
        if ((pos >= 0) && (pos < static_cast<int32_t>(SAMPLES_PER_SYMBOL)))
            fsCorr[pos] = correlator.convolve(FS_SYNC_WORD);
    }

    // Symbols are read two samples after their base instant, when the
    // four samples surrounding the fractional centre are available.
    if (sampleCount != (symbolBase + 2))
        return;

    float value = cubicInterpolate(history.data(), symbolFrac);
    quantize(value);
    devEstimator.sample(static_cast<int16_t>(value));
    symbolBase += SAMPLES_PER_SYMBOL;

    if (frameIndex == SYNCWORD_SYMBOLS) {
        bool valid = checkFrameSync(demodFrame->data(), FS_SYNC_BYTES,
                                    FS_MAX_DISTANCE);

        // Update the levels only between two valid syncs, so dropout noise
        // is left out; update the timing only on a valid sync, so noise
        // cannot drag the sampling lattice.
        if (valid && lastSyncValid)
            devEstimator.update();
        else
            devEstimator.reinit();

        if (valid)
            refineTiming();

        // Noise forms one syncword now and then, rarely two in a row: a new
        // lock is lost on its first missed sync, a verified one on the
        // fifth in a row.
        if (valid)
            missedSyncs = 5;
        else
            missedSyncs -= 1;

        lastSyncValid = valid;

        if (missedSyncs == 0) {
            unlock();
            return;
        }
    }

    /*
     * The NID is in: its DUID gives the length of this unit. A unit whose
     * NID cannot be decoded (or whose type is not framed here) cannot be
     * delimited, so the lock is dropped and the next frame sync is hunted
     * for afresh.
     */
    if (frameIndex == NID_END_SYMBOLS) {
        uint8_t nid[Nid::BYTES];
        Nid::fromFrame(demodFrame->data(), nid);

        uint16_t nac;
        uint8_t duid;
        size_t bytes = 0;
        if (Nid::decode(nid, nac, duid))
            bytes = Nid::frameBytes(duid);

        if (bytes == 0) {
            unlock();
            return;
        }

        frameSymbols = bytes * 4;
        frameDuid = duid;
        frameNac = nac;
    }

    if ((frameSymbols != 0) && (frameIndex == frameSymbols)) {
        std::swap(readyFrame, demodFrame);
        readyBytes = frameSymbols / 4;
        readyDuid = frameDuid;
        readyNac = frameNac;
        frameIndex = 0;
        frameSymbols = 0;
        newFrame = true;
    }
}

void Demodulator::refineTiming()
{
    /*
     * fsCorr[2] is the correlation at the base instant of the last FS
     * symbol, fsCorr[1] and fsCorr[3] one sample either side. A whole
     * sample of drift moves the peak to a neighbour: follow it by at most
     * one sample per frame, then refine to a fraction with a parabola.
     */
    int32_t peakIdx = 2;
    for (int32_t i = 1; i <= 3; i++) {
        if (fsCorr[i] > fsCorr[peakIdx])
            peakIdx = i;
    }

    float offset = static_cast<float>(peakIdx - 2)
                 + parabolicPeakOffset(fsCorr[peakIdx - 1], fsCorr[peakIdx],
                                       fsCorr[peakIdx + 1]);

    // symbolBase already points at the next symbol's integer base: the
    // measured offset is the full fractional position relative to that
    // base, so it replaces the previous estimate rather than adding to it.
    setTiming(static_cast<float>(symbolBase) + offset);
}

constexpr std::array<float, 3> Demodulator::sfNum;
constexpr std::array<float, 3> Demodulator::sfDen;
