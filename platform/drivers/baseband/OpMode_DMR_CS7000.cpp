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

OpMode_DMR::OpMode_DMR()
    : enterRx(true), audioOn(false), rxAudioPath(-1), idleTicks(0)
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

    if (audioOn) {
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
    (void)newCfg;

    dmr_setRxTalkgroup(status->dmrRxTalkgroup);
    dmr_setRxTimeslot(status->dmrTimeslot);

    if ((status->opStatus == OFF) && enterRx) {
        radio_disableRtx();
        radio_enableRx();
        status->opStatus = RX;
        enterRx = false;
    }

    /*
     * Squelch: the speaker path (and with it the audio amplifier) is open
     * only during a call and while its last frames play out.  Up to v34 it
     * stayed open for the whole mode and the amplifier hissed between calls.
     */
    bool active = rxSquelchOpen() || (dmrAudio_queued() > 0);
    if (active)
        idleTicks = 0;
    else if (idleTicks < 255)
        idleTicks++;

    if (active && !audioOn) {
        rxAudioPath = audioPath_request(SOURCE_MCU, SINK_SPK, PRIO_RX);
        if ((rxAudioPath > 0) && dmrAudio_start(rxAudioPath))
            audioOn = true;
        else if (rxAudioPath > 0)
            audioPath_release(rxAudioPath);
    } else if (!active && audioOn && (idleTicks >= 10)) {
        // 300 ms after the call: the jitter buffer has played out
        dmrAudio_stop();
        audioPath_release(rxAudioPath);
        audioOn = false;
    }

    // Transmit is not implemented: ignore PTT.

    if (rxSquelchOpen())
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
