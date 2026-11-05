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
#include "drivers/baseband/dmr_diag_CS7000.h"
#include "drivers/baseband/nxdn_CS7000.h"
#include "OpMode_NXDN_CS7000.hpp"

using namespace NXDN;

/*
 * Transmit: a group voice call as MMDVMHost's RF side expects one (inbound):
 * one frame of preamble, a header frame (VCALL in both FACCH1), voice frames
 * whose SACCH carries the VCALL over superframes of four, the last
 * superframe completed, then a terminator frame (TX_REL).  The voice comes
 * from the AMBE+2 encoder of the DMR mode, four frames per 80 ms frame.
 * Time-out: 3 minutes (2250 frames).
 */
static constexpr uint32_t TX_TIMEOUT_FRAMES = 2250;

// Baseband buffer: two halves of 20 ms at 24 kHz, as the M17 mode uses
static constexpr size_t BB_HALF = Demodulator::RX_SAMPLE_RATE / 50;

// Call state, read by the user interface through nxdn_getCallInfo()
static nxdnCallInfo_t callInfo;
static volatile bool callActive = false;
static uint32_t callFrames = 0;
static uint32_t callDropped = 0;
static uint32_t callBadLich = 0;   // Frames whose LICH did not decode
static uint32_t callFlywheel = 0;  // Of those, played on the last LICH
static uint32_t callBadSacch = 0;  // Voice frames whose SACCH did not
static uint32_t callLockLost = 0;  // Demodulator lock losses
static uint32_t callRealignAt = 0; // Demodulator realign count at call start

bool nxdn_getCallInfo(nxdnCallInfo_t *out)
{
    if (!callActive)
        return false;
    *out = callInfo;
    return true;
}

OpMode_NXDN::OpMode_NXDN()
    : enterRx(false)
    , rxOn(false)
    , basebandPath(-1)
    , basebandId(-1)
    , ran(0)
    , framesSinceGood(0)
    , unlockedTicks(0)
    , flywheel(0)
    , lastOption(STEAL_NONE)
    , goodVoice(0)
    , partsSeen(0)
    , rxCall{}
    , transmitting(false)
    , pttHeld(false)
    , txFrames(0)
    , txSilence(0)
    , txStart(0)
    , txCall{}
{
}

OpMode_NXDN::~OpMode_NXDN()
{
    disable();
}

void OpMode_NXDN::enable()
{
    // Reports a transmission cut short by a reset, then arms the breadcrumbs
    dmrDiag_init();
    basebandBuf = std::make_unique<int16_t[]>(2 * BB_HALF);
    demod = std::make_unique<Demodulator>();
    callActive = false;
    enterRx = true;
    rxOn = false;
    transmitting = false;
    pttHeld = false;
}

void OpMode_NXDN::disable()
{
    platform_ledOff(GREEN);
    platform_ledOff(RED);

    if (transmitting) {
        LOG_W("nxdn", "tx cut short by a mode or channel change: %lu frames",
              (unsigned long)txFrames);
        dmrAudio_stopTx();
        modulator.reset();
        transmitting = false;
        dmrDiag_txEnd();
    }

    stopRx();
    dmrAudio_speakerOff();

    radio_disableRtx();
    callActive = false;
    enterRx = false;

    basebandBuf.reset();
    demod.reset();
}

void OpMode_NXDN::startRx()
{
    radio_disableRtx();
    radio_enableRx();

    basebandPath = audioPath_request(SOURCE_RTX, SINK_MCU, PRIO_RX);
    basebandId = audioStream_start(basebandPath, basebandBuf.get(), 2 * BB_HALF,
                                   Demodulator::RX_SAMPLE_RATE,
                                   STREAM_INPUT | BUF_CIRC_DOUBLE);
    demod->reset();
    framesSinceGood = 0;
    goodVoice = 0;
    rxOn = true;

    LOG_I("nxdn", "receive started, baseband stream %d", basebandId);
}

void OpMode_NXDN::stopRx()
{
    if (!rxOn)
        return;

    audioStream_terminate(basebandId);
    audioPath_release(basebandPath);
    basebandId = -1;
    basebandPath = -1;
    rxOn = false;
}

void OpMode_NXDN::update(rtxStatus_t *const status, const bool newCfg)
{
    (void)newCfg;

    ran = status->nxdnRan;

    if ((status->opStatus == OFF) && enterRx) {
        startRx();
        status->opStatus = RX;
        enterRx = false;
    }

    /*
     * Transmit: a press of the PTT starts one transmission, which sends a
     * frame (80 ms) per update, paced by the baseband output; releasing the
     * PTT, or the time-out, ends it.
     */
    bool ptt = platform_getPttStatus();
    if (!ptt)
        pttHeld = false;

    if (!transmitting && ptt && !pttHeld && (status->opStatus == RX)) {
        pttHeld = true;
        if (status->txDisable != 0)
            LOG_W("nxdn", "PTT: transmit disabled on this channel");
        else if (status->nxdnId == 0)
            LOG_W("nxdn", "PTT: no NXDN ID set (Settings > NXDN)");
        else
            startTx(status);
    }

    if (transmitting) {
        if (!ptt || (txFrames >= TX_TIMEOUT_FRAMES))
            endTx(status);
        else
            sendVoiceFrame();
        return;
    }

    if (rxOn && (audioPath_getStatus(basebandPath) == PATH_OPEN)) {
        // Blocks until the next 20 ms of baseband
        dataBlock_t bb = inputStream_getData(basebandId);
        for (size_t i = 0; (bb.data != NULL) && (i < bb.len); i++) {
            if (demod->sample(bb.data[i]))
                processFrame(demod->getFrame());
        }

        // A call survives a short loss of lock (half a second)
        if (callActive && !demod->isLocked()) {
            if (unlockedTicks == 0)
                callLockLost++;
            if (++unlockedTicks >= 25)
                endCall("signal lost");
        } else {
            unlockedTicks = 0;
        }
    } else {
        sleepFor(0u, 30u);
    }

    // Speaker open during a call and while its last frames play out
    dmrAudio_squelch(callActive);

    if (rxSquelchOpen())
        platform_ledOn(GREEN);
    else
        platform_ledOff(GREEN);
}

bool OpMode_NXDN::rxSquelchOpen()
{
    return callActive;
}

void OpMode_NXDN::startCall(const char *how)
{
    callActive = true;
    callFrames = 0;
    callDropped = 0;
    callBadLich = 0;
    callFlywheel = 0;
    callBadSacch = 0;
    callLockLost = 0;
    callRealignAt = demod->realignCount();
    unlockedTicks = 0;
    flywheel = 0;
    lastOption = STEAL_NONE;

    if (callInfo.ids)
        LOG_I("nxdn", "call start (%s): %u > %s%u, RAN %u", how, callInfo.src,
              callInfo.group ? "TG " : "", callInfo.dst, callInfo.ran);
    else
        LOG_I("nxdn", "call start (%s), RAN %u", how, callInfo.ran);
}

void OpMode_NXDN::endCall(const char *why)
{
    if (callActive) {
        LOG_I("nxdn",
              "call end (%s): %u > %u, %lu frames, %lu voice frames lost", why,
              callInfo.src, callInfo.dst, (unsigned long)callFrames,
              (unsigned long)callDropped);
        LOG_I("nxdn",
              "  bad LICH %lu (%lu played on the last one), bad SACCH %lu, "
              "lock lost %lu, realigned %lu, level %ld",
              (unsigned long)callBadLich, (unsigned long)callFlywheel,
              (unsigned long)callBadSacch, (unsigned long)callLockLost,
              (unsigned long)(demod->realignCount() - callRealignAt),
              (long)demod->outerLevel());
    }
    callActive = false;
    unlockedTicks = 0;
    goodVoice = 0;
    partsSeen = 0;
}

void OpMode_NXDN::processFrame(const frame_t &rxFrame)
{
    frame_t frame = rxFrame;
    scramble(frame); // descramble

    uint8_t lich;
    if (decodeLich(frame, lich) == false) {
        /*
         * A frame whose LICH does not decode: during a call, with the frame
         * well aligned (its sync word nearly intact), its voice is played as
         * the last voice frame's LICH said, for up to two frames in a row
         * (the AMBE FEC copes with the bit errors that broke the LICH).
         * Otherwise the frame is skipped; the call ends after 5 of them in a
         * row (400 ms).
         */
        framesSinceGood++;
        goodVoice = 0;
        if (!callActive)
            return;
        callBadLich++;
        bool aligned = (demod->lastSyncErrors() <= 1);
        if (aligned && (lastOption == STEAL_NONE) && (flywheel < 2)) {
            flywheel++;
            callFlywheel++;
            callFrames++;
            pushVoice(frame, STEAL_NONE);
        } else if (framesSinceGood >= 5) {
            endCall("no valid LICH");
        }
        return;
    }

    framesSinceGood = 0;
    flywheel = 0;

    // Traffic channels only (direct, repeater or hotspot, either direction)
    if (lichRfct(lich) == RFCT_RCCH)
        return;

    switch (lichFct(lich)) {
        case FCT_SACCH_NS:
            processControl(frame);
            break;

        case FCT_SACCH_SS:
            processVoice(frame, lich);
            break;

        default: // data and idle frames
            break;
    }
}

// Header or terminator: the call message in the FACCH1
void OpMode_NXDN::processControl(const frame_t &frame)
{
    sacch_t sacch;
    if (decodeSacch(frame, sacch)) {
        uint8_t r = sacchRan(sacch);
        if ((ran != 0) && (r != 0) && (r != ran))
            return;
        callInfo.ran = r;
    }

    layer3_t l3;
    if (!decodeFacch1(frame, PAYLOAD_OFFSET, l3)
        && !decodeFacch1(frame, PAYLOAD_OFFSET + FACCH1_BITS, l3))
        return;

    switch (messageType(l3)) {
        case MSG_VCALL:
            callInfo.src = sourceId(l3);
            callInfo.dst = destinationId(l3);
            callInfo.group = isGroupCall(l3);
            callInfo.ids = true;
            if (!callActive)
                startCall("header");
            break;

        case MSG_TX_REL:
            endCall("release");
            break;

        default:
            break;
    }
}

void OpMode_NXDN::processVoice(const frame_t &frame, const uint8_t lich)
{
    sacch_t sacch;
    if (decodeSacch(frame, sacch)) {
        uint8_t r = sacchRan(sacch);
        if ((ran != 0) && (r != 0) && (r != ran)) {
            if (callActive)
                endCall("other RAN");
            goodVoice = 0;
            return;
        }

        // The VCALL message, 18 bits per frame over the superframe
        callInfo.ran = r;
        size_t part = SR_1_4 - sacchStructure(sacch);
        uint8_t data[3];
        sacchData(sacch, data);
        setLayer3Part(rxCall, part, data);
        partsSeen |= 1 << part;
        if ((partsSeen == 0x0F) && (messageType(rxCall) == MSG_VCALL)) {
            callInfo.src = sourceId(rxCall);
            callInfo.dst = destinationId(rxCall);
            callInfo.group = isGroupCall(rxCall);
            callInfo.ids = true;
        }

        if (goodVoice < 255)
            goodVoice++;
    } else {
        goodVoice = 0;
        if (callActive)
            callBadSacch++;
    }

    // Late entry: two voice frames in a row with a valid SACCH (noise can
    // form a frame sync word, and a SACCH CRC-6 passes one time in 64)
    if (!callActive) {
        if (goodVoice < 2)
            return;
        memset(&callInfo, 0, sizeof(callInfo));
        callInfo.ran = sacchRan(sacch);
        if ((partsSeen == 0x0F) && (messageType(rxCall) == MSG_VCALL)) {
            callInfo.src = sourceId(rxCall);
            callInfo.dst = destinationId(rxCall);
            callInfo.group = isGroupCall(rxCall);
            callInfo.ids = true;
        }
        startCall("late entry");
    }

    uint8_t option = lichOption(lich);
    if (option != STEAL_NONE) {
        // A stolen half may carry the end of the call
        layer3_t l3;
        bool ok = false;
        if (option != STEAL_FACCH1_2)
            ok = decodeFacch1(frame, PAYLOAD_OFFSET, l3);
        if (!ok && (option != STEAL_FACCH1_1))
            ok = decodeFacch1(frame, PAYLOAD_OFFSET + FACCH1_BITS, l3);
        if (ok && (messageType(l3) == MSG_TX_REL)) {
            endCall("release");
            return;
        }
    }

    lastOption = option;
    callFrames++;
    pushVoice(frame, option);
}

void OpMode_NXDN::pushVoice(const frame_t &frame, const uint8_t option)
{
    for (size_t k = 0; k < 4; k++) {
        // A FACCH1 in the first half takes voice frames 0 and 1, in the
        // second half 2 and 3
        bool first = (k < 2);
        if ((option == STEAL_FACCH) || (first && (option == STEAL_FACCH1_1))
            || (!first && (option == STEAL_FACCH1_2)))
            continue;

        if (!dmrAudio_pushFrame(&frame[VOICE_BYTE_OFFSET + (AMBE_BYTES * k)]))
            callDropped++;
    }
}

void OpMode_NXDN::startTx(rtxStatus_t *const status)
{
    if (callActive)
        endCall("transmit");
    stopRx();
    dmrAudio_speakerOff();

    // As for DMR: a stall of the RTX thread resets the radio, and a reset
    // during the transmission is reported at the next start
    dmrDiag_startWatchdog();
    dmrDiag_txStart();

    if (!dmrAudio_startTx(status->dmrMicGain)) {
        LOG_W("nxdn", "PTT: no voice encoder (codec missing or damaged)");
        dmrDiag_txEnd();
        startRx();
        return;
    }
    dmrDiag_set(DIAG_STAGE, STAGE_ENCODER);

    txCall = makeVoiceCall(MSG_VCALL, status->nxdnId, status->nxdnTg, true);

    modulator = std::make_unique<Modulator>();
    modulator->init();
    modulator->invertPhase(true); // as the M17 and YSF modes on the CS7000
    radio_disableRtx();
    radio_enableTx();
    if (!modulator->start()) {
        LOG_W("nxdn", "PTT: no baseband output path");
        modulator.reset();
        radio_disableRtx();
        dmrAudio_stopTx();
        dmrDiag_txEnd();
        startRx();
        return;
    }
    dmrDiag_set(DIAG_STAGE, STAGE_KEYED);

    status->opStatus = TX;
    transmitting = true;
    txFrames = 0;
    txSilence = 0;
    txStart = getTick();
    platform_ledOn(RED);

    LOG_I("nxdn", "tx start: %u > TG %u, RAN %u", status->nxdnId,
          status->nxdnTg, ran);

    frame_t frame;
    buildControlFrame(frame, ran, txCall);
    modulator->sendPreamble();
    modulator->sendFrame(frame);
}

void OpMode_NXDN::sendVoiceFrame()
{
    uint8_t ambe[4][AMBE_BYTES];
    for (size_t k = 0; k < 4; k++) {
        if (!dmrAudio_popTxFrame(ambe[k])) // silence if none
            txSilence++;
    }

    frame_t frame;
    buildVoiceFrame(frame, ran, txCall, txFrames % 4, ambe);
    modulator->sendFrame(frame);
    txFrames++;
    dmrDiag_inc(DIAG_BEAT);
}

void OpMode_NXDN::endTx(rtxStatus_t *const status)
{
    // Complete the superframe, so that receivers get the whole SACCH message
    while ((txFrames % 4) != 0)
        sendVoiceFrame();

    dmrDiag_set(DIAG_STAGE, STAGE_TERM);
    layer3_t rel = txCall;
    rel[0] = (rel[0] & 0xC0) | MSG_TX_REL;
    frame_t frame;
    buildControlFrame(frame, ran, rel);
    modulator->sendFrame(frame);
    modulator->sendFlush(); // the end of the terminator, out of the filter
    modulator->stop();      // returns once that has gone out

    dmrAudio_stopTx();
    modulator.reset();

    radio_disableRtx();
    dmrDiag_txEnd();
    LOG_I("nxdn",
          "tx end: %lu frames, %lu voice frames sent as silence, %lu ms",
          (unsigned long)txFrames, (unsigned long)txSilence,
          (unsigned long)(getTick() - txStart));

    transmitting = false;
    status->opStatus = RX;
    platform_ledOff(RED);
    startRx();
}
