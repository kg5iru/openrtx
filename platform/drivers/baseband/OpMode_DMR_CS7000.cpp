/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "interfaces/platform.h"
#include "interfaces/radio.h"
#include "interfaces/delays.h"
#include "core/audio_path.h"
#include "drivers/baseband/dmr_CS7000.h"
#include "drivers/baseband/dmr_audio_CS7000.h"
#include "OpMode_DMR_CS7000.hpp"

OpMode_DMR::OpMode_DMR() : enterRx(true), audioOn(false), rxAudioPath(-1)
{
}

OpMode_DMR::~OpMode_DMR()
{
}

void OpMode_DMR::enable()
{
    enterRx = true;
    audioOn = false;
}

void OpMode_DMR::disable()
{
    platform_ledOff(GREEN);
    platform_ledOff(RED);

    if(audioOn)
    {
        dmrAudio_stop();
        audioPath_release(rxAudioPath);
        audioOn = false;
    }

    dmr_terminate();
    radio_disableRtx();
    enterRx = false;
}

void OpMode_DMR::update(rtxStatus_t *const status, const bool newCfg)
{
    (void) newCfg;

    if((status->opStatus == OFF) && enterRx)
    {
        radio_disableRtx();
        radio_enableRx();
        status->opStatus = RX;
        enterRx = false;

        // Open the speaker path for the lifetime of the mode; the audio
        // thread plays silence between calls.
        rxAudioPath = audioPath_request(SOURCE_MCU, SINK_SPK, PRIO_RX);
        if((rxAudioPath > 0) && dmrAudio_start(rxAudioPath))
            audioOn = true;
        else if(rxAudioPath > 0)
            audioPath_release(rxAudioPath);
    }

    // Transmit is not implemented: ignore PTT.

    if(rxSquelchOpen())
        platform_ledOn(GREEN);
    else
        platform_ledOff(GREEN);

    sleepFor(0u, 30u);
}

bool OpMode_DMR::rxSquelchOpen()
{
    dmrCallInfo_t info;
    return dmr_getCallInfo(&info);
}
