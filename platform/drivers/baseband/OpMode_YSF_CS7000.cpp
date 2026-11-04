/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <cstring>
#include "interfaces/platform.h"
#include "interfaces/radio.h"
#include "interfaces/delays.h"
#include "core/log.h"
#include "drivers/baseband/dmr_audio_CS7000.h"
#include "drivers/baseband/ysf_CS7000.h"
#include "OpMode_YSF_CS7000.hpp"

using namespace YSF;

// Baseband buffer: two halves of 20 ms at 24 kHz, as the M17 mode uses
static constexpr size_t BB_HALF = Demodulator::RX_SAMPLE_RATE / 50;

// Call state, read by the user interface through ysf_getCallInfo()
static ysfCallInfo_t callInfo;
static volatile bool callActive = false;
static uint32_t callFrames = 0;
static uint32_t callDropped = 0;

bool ysf_getCallInfo(ysfCallInfo_t *out)
{
    if (!callActive)
        return false;
    *out = callInfo;
    return true;
}

OpMode_YSF::OpMode_YSF()
    : enterRx(false)
    , rxOn(false)
    , audioOn(false)
    , basebandPath(-1)
    , basebandId(-1)
    , speakerPath(-1)
    , framesSinceGood(0)
{
}

OpMode_YSF::~OpMode_YSF()
{
    disable();
}

void OpMode_YSF::enable()
{
    basebandBuf = std::make_unique<int16_t[]>(2 * BB_HALF);
    demod = std::make_unique<Demodulator>();
    decoder = std::make_unique<FrameDecoder>();
    callActive = false;
    enterRx = true;
    rxOn = false;
    audioOn = false;
}

void OpMode_YSF::disable()
{
    platform_ledOff(GREEN);
    platform_ledOff(RED);

    stopRx();
    if (audioOn) {
        dmrAudio_stop();
        audioPath_release(speakerPath);
        audioOn = false;
    }

    radio_disableRtx();
    callActive = false;
    enterRx = false;

    basebandBuf.reset();
    demod.reset();
    decoder.reset();
}

void OpMode_YSF::startRx()
{
    radio_disableRtx();
    radio_enableRx();

    basebandPath = audioPath_request(SOURCE_RTX, SINK_MCU, PRIO_RX);
    basebandId = audioStream_start(basebandPath, basebandBuf.get(), 2 * BB_HALF,
                                   Demodulator::RX_SAMPLE_RATE,
                                   STREAM_INPUT | BUF_CIRC_DOUBLE);
    demod->reset();
    framesSinceGood = 0;
    rxOn = true;

    // Speaker path for the lifetime of the mode; the audio thread plays
    // silence between calls (as in DMR mode)
    speakerPath = audioPath_request(SOURCE_MCU, SINK_SPK, PRIO_RX);
    if ((speakerPath > 0) && dmrAudio_start(speakerPath))
        audioOn = true;
    else if (speakerPath > 0)
        audioPath_release(speakerPath);

    LOG_I("ysf", "receive started, baseband stream %d, audio %d", basebandId,
          audioOn ? 1 : 0);
}

void OpMode_YSF::stopRx()
{
    if (!rxOn)
        return;

    audioStream_terminate(basebandId);
    audioPath_release(basebandPath);
    basebandId = -1;
    basebandPath = -1;
    rxOn = false;
}

void OpMode_YSF::update(rtxStatus_t *const status, const bool newCfg)
{
    (void)newCfg;

    if ((status->opStatus == OFF) && enterRx) {
        startRx();
        status->opStatus = RX;
        enterRx = false;
    }

    if (rxOn && (audioPath_getStatus(basebandPath) == PATH_OPEN)) {
        // Blocks until the next 20 ms of baseband
        dataBlock_t bb = inputStream_getData(basebandId);
        for (size_t i = 0; (bb.data != NULL) && (i < bb.len); i++) {
            if (demod->sample(bb.data[i]))
                processFrame(demod->getFrame());
        }

        if (callActive && !demod->isLocked())
            endCall("signal lost");
    } else {
        sleepFor(0u, 30u);
    }

    // Transmit is not implemented: PTT is ignored.

    if (rxSquelchOpen())
        platform_ledOn(GREEN);
    else
        platform_ledOff(GREEN);
}

bool OpMode_YSF::rxSquelchOpen()
{
    return callActive;
}

void OpMode_YSF::endCall(const char *why)
{
    if (callActive)
        LOG_I("ysf",
              "call end (%s): %s > %s, %lu frames, %lu voice frames "
              "lost",
              why, callInfo.src, callInfo.dst, (unsigned long)callFrames,
              (unsigned long)callDropped);
    callActive = false;
}

void OpMode_YSF::processFrame(const frame_t &frame)
{
    Fich fich;
    if (decoder->decodeFich(frame, fich) == false) {
        // A frame whose FICH does not decode is skipped; the call ends after
        // 5 of them in a row (500 ms)
        framesSinceGood++;
        if (callActive && (framesSinceGood >= 5))
            endCall("no valid FICH");
        return;
    }

    framesSinceGood = 0;

    switch (fich.fi()) {
        case FI_HEADER: {
            std::array<uint8_t, 20> csd;
            bool ok = decoder->decodeCsd(frame, 1, csd);
            memset(&callInfo, 0, sizeof(callInfo));
            if (ok) {
                callsignToString(&csd[0], callInfo.dst);
                callsignToString(&csd[10], callInfo.src);
            }
            callInfo.dgid = fich.dgid();
            callInfo.dt = fich.dt();
            callActive = true;
            callFrames = 0;
            callDropped = 0;
            LOG_I("ysf", "call start: %s > %s, DG-ID %u, DT %u%s", callInfo.src,
                  callInfo.dst, callInfo.dgid, callInfo.dt,
                  ok ? "" : " (header CSD bad)");
            return;
        }

        case FI_TERMINATOR:
            endCall("terminator");
            return;

        case FI_COMMUNICATIONS:
            break;

        default: // test frames
            return;
    }

    if (!callActive) {
        memset(&callInfo, 0, sizeof(callInfo));
        callActive = true;
        callFrames = 0;
        callDropped = 0;
        LOG_I("ysf", "call start: late entry, DG-ID %u, DT %u", fich.dgid(),
              fich.dt());
    }

    callInfo.dgid = fich.dgid();
    callInfo.dt = fich.dt();
    callFrames++;

    uint8_t ambe[9];
    switch (fich.dt()) {
        case DT_VD_MODE2: {
            // Callsigns arrive in the data channel, one per frame number
            std::array<uint8_t, 10> dch;
            if (decoder->decodeVd2Dch(frame, dch)) {
                if (fich.fn() == 0)
                    callsignToString(dch.data(), callInfo.dst);
                else if (fich.fn() == 1)
                    callsignToString(dch.data(), callInfo.src);
            }

            for (size_t n = 0; n < 5; n++) {
                vd2VoiceToDmr(frame, n, ambe);
                if (!dmrAudio_pushFrame(ambe))
                    callDropped++;
            }
            break;
        }

        case DT_VD_MODE1: {
            std::array<uint8_t, 20> dch;
            if (decoder->decodeVd1Dch(frame, dch) && (fich.fn() == 0)) {
                callsignToString(&dch[0], callInfo.dst);
                callsignToString(&dch[10], callInfo.src);
            }

            for (size_t n = 0; n < 5; n++) {
                vd1VoiceToDmr(frame, n, ambe);
                if (!dmrAudio_pushFrame(ambe))
                    callDropped++;
            }
            break;
        }

        default:
            // Voice full rate (VW, IMBE) and data full rate: no voice here
            break;
    }
}
