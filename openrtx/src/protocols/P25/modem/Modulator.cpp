/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <new>
#include <cstddef>
#include <cstring>
#include <algorithm>
#include "protocols/P25/modem/Modulator.hpp"
#include "protocols/P25/modem/Utils.hpp"
#include "protocols/P25/modem/DSP.hpp"
#include "hwconfig.h"

#if defined(PLATFORM_LINUX)
#include <stdio.h>
#endif

using namespace P25;

Modulator::Modulator()
    : pendingSymbols(0)
    , idleBuffer(nullptr)
    , outStream(-1)
    , outPath(-1)
    , txRunning(false)
    , invPhase(false)
{
}

Modulator::~Modulator()
{
    terminate();
}

void Modulator::init()
{
    /*
     * Allocate a chunk of memory to contain two complete half-frame
     * buffers for baseband audio.
     */

    baseband_buffer = std::make_unique<int16_t[]>(2 * CHUNK_SAMPLES);
    idleBuffer = baseband_buffer.get();
    outStream = -1;
    outPath = -1;
    txRunning = false;
#if defined(PLATFORM_MD3x0) || defined(PLATFORM_MDUV3x0)
    pwmComp.reset();
#endif
}

void Modulator::terminate()
{
    // Terminate an ongoing stream, if present
    if (txRunning) {
        audioStream_terminate(outStream);
        txRunning = false;
    }

    // Always ensure that outgoing audio path is closed
    audioPath_release(outPath);

    // Deallocate memory.
    baseband_buffer.reset();
}

bool Modulator::start()
{
    if (txRunning)
        return true;

#ifndef PLATFORM_LINUX
    outPath = audioPath_request(SOURCE_MCU, SINK_RTX, PRIO_TX);
    if (outPath < 0)
        return false;

    outStream = audioStream_start(outPath, baseband_buffer.get(),
                                  2 * CHUNK_SAMPLES, TX_SAMPLE_RATE,
                                  STREAM_OUTPUT | BUF_CIRC_DOUBLE);

    if (outStream < 0)
        return false;

    idleBuffer = outputStream_getIdleBuffer(outStream);
#endif

    txRunning = true;
    pendingSymbols = 0;
    pulse_48k.reset();

    return true;
}

constexpr int8_t Modulator::PREAMBLE_TIA[2];
constexpr int8_t Modulator::PREAMBLE_MMDVM[2];
constexpr int8_t Modulator::PREAMBLE_KENWOOD[4];
constexpr int8_t Modulator::PREAMBLE_SWEEP[4];

void Modulator::sendPreamble(const int8_t *pattern, const size_t patternLen,
                             const size_t symbolCount)
{
    for (size_t i = 0; i < symbolCount; i++)
        queueSymbol(pattern[i % patternLen]);
}

void Modulator::sendPreamble()
{
    sendPreamble(PREAMBLE_TIA, 2, FRAME_SYMBOLS);
}

void Modulator::sendFrame(const uint8_t *data, const size_t bytes)
{
    for (size_t i = 0; i < bytes; i++) {
        auto sym = byteToSymbols(data[i]);
        for (auto s : sym)
            queueSymbol(s);
    }
}

void Modulator::queueSymbol(const int8_t symbol)
{
    symbols[pendingSymbols] = symbol;
    pendingSymbols += 1;

    if (pendingSymbols == CHUNK_SYMBOLS) {
        sendChunk();
        pendingSymbols = 0;
    }
}

void Modulator::stop()
{
    if (txRunning == false)
        return;

    // Flush the pending symbols, padding the chunk with the preamble
    // pattern; this also plays out the shaping filter's tail.
    if (pendingSymbols > 0) {
        while (pendingSymbols != 0)
            queueSymbol((pendingSymbols & 1) ? -3 : +3);
    }

    audioStream_stop(outStream);
    txRunning = false;
    idleBuffer = baseband_buffer.get();
    audioPath_release(outPath);

#if defined(PLATFORM_MD3x0) || defined(PLATFORM_MDUV3x0)
    pwmComp.reset();
#endif
}

void Modulator::invertPhase(const bool status)
{
    invPhase = status;
}

void Modulator::sendChunk()
{
#ifndef PLATFORM_LINUX
    if (txRunning == false)
        return;
    if (audioPath_getStatus(outPath) != PATH_OPEN)
        return;

    // Transmission is ongoing, syncronise with stream end before proceeding
    outputStream_sync(outStream, true);
    idleBuffer = outputStream_getIdleBuffer(outStream);
#endif

    memset(idleBuffer, 0x00, CHUNK_SAMPLES * sizeof(stream_sample_t));

    for (size_t i = 0; i < CHUNK_SYMBOLS; i++)
        idleBuffer[i * SAMPLES_PER_SYMBOL] = symbols[i];

    for (size_t i = 0; i < CHUNK_SAMPLES; i++) {
        float elem = static_cast<float>(idleBuffer[i]);
        elem = pulse_48k(elem * pulseGain);
#if defined(PLATFORM_MD3x0) || defined(PLATFORM_MDUV3x0)
        elem = pwmComp(elem);
#endif
        if (invPhase)
            elem = 0.0f - elem; // Invert signal phase

        // Saturate, never wrap: C4FM transitions overshoot the outer level
        // and a wrapped peak flips the sign of a whole symbol at the receiver.
        if (elem > 32767.0f)
            elem = 32767.0f;
        if (elem < -32768.0f)
            elem = -32768.0f;
        idleBuffer[i] = static_cast<int16_t>(elem);
    }

    if (sampleSink)
        sampleSink(idleBuffer, CHUNK_SAMPLES, sinkContext);
}
