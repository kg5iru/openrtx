/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <cstring>
#include "protocols/M17/DSP.hpp"
#include "protocols/M17/Utils.hpp"
#include "protocols/YSF/Modulator.hpp"

using namespace YSF;

Modulator::Modulator()
    : idleBuffer(nullptr), outStream(-1), outPath(-1), txRunning(false)
{
}

Modulator::~Modulator()
{
    terminate();
}

void Modulator::init()
{
    basebandBuffer = std::make_unique<int16_t[]>(2 * FRAME_SAMPLES);
    idleBuffer = basebandBuffer.get();
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
    // Dibits to symbols as M17 and YSF both map them: 01 +3, 00 +1, 10 -1,
    // 11 -3 (the YSF sync word D4 71 C9 63 4D starts -3 +3 +3 +1)
    auto it = symbols.begin();
    for (size_t i = 0; i < frame.size(); i++) {
        auto sym = M17::byteToSymbols(frame[i]);
        it = std::copy(sym.begin(), sym.end(), it);
    }

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

void Modulator::symbolsToBaseband()
{
    memset(idleBuffer, 0x00, FRAME_SAMPLES * sizeof(stream_sample_t));

    for (size_t i = 0; i < symbols.size(); i++)
        idleBuffer[i * SAMPLES_PER_SYMBOL] = symbols[i];

    for (size_t i = 0; i < FRAME_SAMPLES; i++) {
        float elem = static_cast<float>(idleBuffer[i]);
        elem = M17::rrc_48k(elem * RRC_GAIN);
        idleBuffer[i] = static_cast<int16_t>(elem);
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
