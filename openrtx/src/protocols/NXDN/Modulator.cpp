/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <cstring>
#include "protocols/M17/Utils.hpp"
#include "protocols/NXDN/DSP.hpp"
#include "protocols/NXDN/Modulator.hpp"

using namespace NXDN;

static_assert(tx_taps_48k.size() == 240, "12 symbols of 20 samples");

Modulator::Modulator()
    : idleBuffer(nullptr)
    , outStream(-1)
    , outPath(-1)
    , txRunning(false)
    , invPhase(false)
{
    history.fill(0);
}

Modulator::~Modulator()
{
    terminate();
}

void Modulator::init()
{
    basebandBuffer = std::make_unique<int16_t[]>(2 * FRAME_SAMPLES);
    idleBuffer = basebandBuffer.get();
    history.fill(0);
    txRunning = false;
}

void Modulator::terminate()
{
    if (txRunning) {
        audioStream_terminate(outStream);
        txRunning = false;
    }

    if (outPath >= 0) {
        audioPath_release(outPath);
        outPath = -1;
    }

    basebandBuffer.reset();
}

bool Modulator::start()
{
    if (txRunning)
        return true;

    outPath = audioPath_request(SOURCE_MCU, SINK_RTX, PRIO_TX);
    if (outPath < 0)
        return false;

    outStream = audioStream_start(outPath, basebandBuffer.get(),
                                  2 * FRAME_SAMPLES, TX_SAMPLE_RATE,
                                  STREAM_OUTPUT | BUF_CIRC_DOUBLE);
    if (outStream < 0) {
        audioPath_release(outPath);
        outPath = -1;
        return false;
    }

    idleBuffer = outputStream_getIdleBuffer(outStream);
    txRunning = true;
    return true;
}

void Modulator::sendPreamble()
{
    for (size_t i = 0; i < symbols.size(); i += 2) {
        symbols[i] = +3;
        symbols[i + 1] = -3;
    }

    symbolsToBaseband();
    sendBaseband();
}

void Modulator::sendFrame(const frame_t &frame)
{
    // Dibits to symbols as M17 maps them: 01 +3, 00 +1, 10 -1, 11 -3
    auto it = symbols.begin();
    for (size_t i = 0; i < frame.size(); i++) {
        auto sym = M17::byteToSymbols(frame[i]);
        it = std::copy(sym.begin(), sym.end(), it);
    }

    symbolsToBaseband();
    sendBaseband();
}

void Modulator::sendFlush()
{
    symbols.fill(0);
    symbolsToBaseband();
    sendBaseband();
}

void Modulator::stop()
{
    if (!txRunning)
        return;

    audioStream_stop(outStream);
    txRunning = false;
    idleBuffer = basebandBuffer.get();
    audioPath_release(outPath);
    outPath = -1;
}

void Modulator::invertPhase(const bool status)
{
    invPhase = status;
}

/*
 * Polyphase pulse shaping: each output sample is the sum of the last 12
 * symbols, each weighed by the filter tap of its age and the sample's phase
 * in the symbol.
 */
void Modulator::symbolsToBaseband()
{
    const float gain = (invPhase ? -OUTER_LEVEL : OUTER_LEVEL) / 3.0f;
    stream_sample_t *out = idleBuffer;

    for (size_t i = 0; i < symbols.size(); i++) {
        for (size_t k = FILTER_SYMBOLS - 1; k > 0; k--)
            history[k] = history[k - 1];
        history[0] = symbols[i];

        for (size_t p = 0; p < SAMPLES_PER_SYMBOL; p++) {
            float acc = 0.0f;
            for (size_t k = 0; k < FILTER_SYMBOLS; k++)
                acc += static_cast<float>(history[k])
                     * tx_taps_48k[(SAMPLES_PER_SYMBOL * k) + p];

            float v = acc * gain;
            if (v > 32767.0f)
                v = 32767.0f;
            else if (v < -32768.0f)
                v = -32768.0f;
            *out++ = static_cast<int16_t>(v);
        }
    }
}

void Modulator::sendBaseband()
{
    if (!txRunning)
        return;
    if (audioPath_getStatus(outPath) != PATH_OPEN)
        return;

    // Wait for the half being played to finish, then fill the other one
    outputStream_sync(outStream, true);
    idleBuffer = outputStream_getIdleBuffer(outStream);
}
