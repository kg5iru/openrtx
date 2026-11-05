/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <string.h>
#include "interfaces/platform.h"
#include "interfaces/radio.h"
#include "interfaces/delays.h"
#include "core/audio_path.h"
#include "core/log.h"
#include "drivers/baseband/dmr_CS7000.h"
#include "drivers/baseband/dmr_audio_CS7000.h"
#include "drivers/baseband/dmr_diag_CS7000.h"
#include "OpMode_DMR_CS7000.hpp"

/*
 * Transmit time-out: 3 minutes of 30 ms updates; the terminator is then sent
 * and the PTT has to be released and pressed again.
 */
static constexpr uint16_t TX_TIMEOUT = 6000;

// Talkgroup used when the Talkgroup setting is All (local, as on a hotspot)
static constexpr uint32_t DEFAULT_TX_TG = 9;

/*
 * After a private call to this radio, a PTT press within this time answers
 * the caller with a private call, as DMR radios do (call hang time).
 */
static constexpr long long PRIVATE_REPLY_MS = 5000;

/*
 * Repeater (duplex hotspot) wake-up, with a timeslot set.  v48 on air: the
 * hotspot repeated the owner's Anytone but not the CS7000.  Off air, the
 * Anytone sends a CSBK BS_Dwn_Act on the hotspot input, waits about 0.6 s,
 * then sends its call in step with the hotspot's slots; the CS7000 sent its
 * call at once on its own timing.  A wake-up (two CSBKs, as the Anytone
 * sends), then up to 600 ms for the repeater's signal, 3 tries (OpenGD77).
 * Skipped when the repeater is already on the air.
 */
enum : uint8_t { WAKE_NONE = 0, WAKE_SENDING, WAKE_WAITING };
static constexpr uint8_t WAKE_WAIT_TICKS = 20; // 600 ms
static constexpr uint8_t WAKE_TRIES = 3;

OpMode_DMR::OpMode_DMR()
    : enterRx(true)
    , audioOn(false)
    , rxAudioPath(-1)
    , idleTicks(0)
    , transmitting(false)
    , pttHeld(false)
    , txTicks(0)
    , txStart(0)
    , stopTick(0)
    , micPath(-1)
    , wakePhase(WAKE_NONE)
    , wakeTries(0)
    , wakeTicks(0)
    , wakeStart(0)
    , replyId(0)
    , replyTime(0)
{
}

OpMode_DMR::~OpMode_DMR()
{
}

void OpMode_DMR::enable()
{
    enterRx = true;
    audioOn = false;
    transmitting = false;
    pttHeld = false;
    wakePhase = WAKE_NONE;
}

void OpMode_DMR::disable()
{
    platform_ledOff(GREEN);
    platform_ledOff(RED);

    if (transmitting) {
        dmrAudio_stopTx();
        if (micPath >= 0)
            audioPath_release(micPath);
        micPath = -1;
        dmr_endTx((uint32_t)(getTick() - txStart));
        dmrDiag_txEnd();
        transmitting = false;
    }

    if (wakePhase == WAKE_SENDING)
        dmr_endTx(0);
    wakePhase = WAKE_NONE;

    stopRxAudio();

    dmr_terminate();
    radio_disableRtx();
    enterRx = false;
}

void OpMode_DMR::update(rtxStatus_t *const status, const bool newCfg)
{
    (void)newCfg;

    dmr_setRxTalkgroup(status->dmrRxTalkgroup);
    dmr_setRxTimeslot(status->dmrTimeslot);
    dmr_setOwnId(status->dmrRadioId);
    dmr_setMonitor(status->dmrMonitor);

    // Private call to this radio: remember the caller for a reply
    dmrCallInfo_t call;
    if (dmr_getCallInfo(&call) && (call.flco == 3) && (call.dst != 0)
        && (call.dst == status->dmrRadioId)) {
        replyId = call.src;
        replyTime = getTick();
    }

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
    bool active = !transmitting && (rxSquelchOpen() || (dmrAudio_queued() > 0));
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

    /*
     * Transmit.  A press of the PTT starts one transmission; releasing it, or
     * the time-out, sends the terminator, and once it is out the radio goes
     * back to receive.  With a timeslot set the call goes through a repeater
     * or duplex hotspot, woken up first if its signal is not received.
     */
    bool ptt = platform_getPttStatus();
    if (!ptt)
        pttHeld = false;

    // Checked at every update: part of the check is that slot interrupts
    // keep coming between two updates
    bool bsLocked = !transmitting && dmr_bsLocked();

    if (!transmitting && ptt && !pttHeld && (status->opStatus == RX)
        && (wakePhase == WAKE_NONE)) {
        pttHeld = true;
        if (status->txDisable != 0)
            LOG_W("dmr", "PTT: transmit disabled on this channel");
        else if (status->dmrRadioId == 0)
            LOG_W("dmr", "PTT: no radio ID set (Settings > DMR)");
        else if ((status->dmrTimeslot != 0) && !bsLocked) {
            if (dmr_bsTail())
                LOG_I("dmr", "repeater ending a call: waking it up");
            wakeTries = 0;
            wakeStart = getTick();
            startWakeup(status);
        } else {
            if (status->dmrTimeslot != 0)
                LOG_I("dmr", "repeater on the air: no wake-up");
            startTx(status);
        }
    }

    if (wakePhase != WAKE_NONE)
        updateWakeup(status, ptt, bsLocked);

    if (transmitting) {
        txTicks++;
        dmrDiag_inc(DIAG_BEAT);
        if ((txTicks == 10) && (dmr_txSlotIrqs() == 0))
            dmr_kickTx();
        /*
         * Through a repeater, the first header is armed within two slots of
         * the start.  If it is not after 300 ms the repeater's timing is
         * gone: wake it up again, the PTT still held.
         */
        if ((txTicks == 10) && dmr_txAligning()) {
            LOG_W("dmr", "repeater slot not found in 300 ms: waking it up");
            finishTx(status);
            if (ptt) {
                wakeTries = 0;
                wakeStart = getTick();
                startWakeup(status);
            }
            sleepFor(0u, 30u);
            return;
        }
        if ((!ptt || (txTicks >= TX_TIMEOUT)) && (stopTick == 0)) {
            dmr_stopTx();
            stopTick = txTicks;
        }
        /*
         * Back to receive once the terminator is out.  If the slot interrupt
         * has stopped (v40 on air: the radio sometimes stayed keyed after
         * the PTT was released), give up after 500 ms without it.
         */
        bool stuck = (stopTick != 0) && ((txTicks - stopTick) >= 17);
        if (dmr_txDone() || stuck) {
            if (stuck && !dmr_txDone())
                LOG_W("dmr", "terminator not sent within 500 ms: forced end");
            finishTx(status);
        }
    }

    if (rxSquelchOpen())
        platform_ledOn(GREEN);
    else
        platform_ledOff(GREEN);

    sleepFor(0u, 30u);
}

void OpMode_DMR::stopRxAudio()
{
    if (audioOn) {
        dmrAudio_stop();
        audioPath_release(rxAudioPath);
        audioOn = false;
    }
}

void OpMode_DMR::startTx(rtxStatus_t *const status)
{
    // A lock-up while transmitting resets the radio after 5 s, unkeying it
    dmrDiag_startWatchdog();
    dmrDiag_txStart();

    stopRxAudio();

    micPath = audioPath_request(SOURCE_MIC, SINK_MCU, PRIO_TX);
    if (micPath < 0)
        LOG_W("dmr", "PTT: no microphone path (%d), sending the test vowel",
              micPath);

    if (!dmrAudio_startTx(micPath, status->dmrMicGain)) {
        LOG_W("dmr", "PTT: no voice encoder (codec missing or damaged)");
        if (micPath >= 0)
            audioPath_release(micPath);
        micPath = -1;
        dmrDiag_txEnd();
        return;
    }
    dmrDiag_set(DIAG_STAGE, STAGE_ENCODER);

    /*
     * Destination: the caller of a private call just received; else the
     * Private Call setting; else the talkgroup (TG 9 with All).
     */
    uint32_t dst = status->dmrRxTalkgroup;
    if (dst == 0)
        dst = DEFAULT_TX_TG;
    bool priv = false;
    if ((replyId != 0) && ((getTick() - replyTime) < PRIVATE_REPLY_MS)) {
        dst = replyId;
        priv = true;
        LOG_I("dmr", "reply to the private call from %lu",
              (unsigned long)replyId);
    } else if (status->dmrPrivateId != 0) {
        dst = status->dmrPrivateId;
        priv = true;
    }

    // Talker alias: the callsign, unless unset (default N0CALL) or off
    const char *alias = NULL;
    if (status->dmrTalkerAlias && (status->source_address[0] != '\0')
        && (strncmp(status->source_address, "N0CALL", 10) != 0))
        alias = status->source_address;
    char aliasBuf[11];
    if (alias != NULL) {
        strncpy(aliasBuf, alias, 10);
        aliasBuf[10] = '\0';
        alias = aliasBuf;
    }

    if (status->dmrTimeslot != 0) {
        // Through the repeater: the driver takes the slot timing from the
        // receiver before the RF chain leaves it
        dmr_startTx(status->dmrRadioId, dst, status->dmrTimeslot, priv, alias);
        radio_disableRtx();
        radio_enableTx();
        dmrDiag_set(DIAG_STAGE, STAGE_KEYED);
    } else {
        radio_disableRtx();
        radio_enableTx();
        dmrDiag_set(DIAG_STAGE, STAGE_KEYED);
        dmr_startTx(status->dmrRadioId, dst, 0, priv, alias);
    }

    status->opStatus = TX;
    transmitting = true;
    txTicks = 0;
    stopTick = 0;
    txStart = getTick();
    platform_ledOn(RED);
}

void OpMode_DMR::finishTx(rtxStatus_t *const status)
{
    dmrDiag_set(DIAG_STAGE, STAGE_FINISHING);
    dmrAudio_stopTx();
    if (micPath >= 0)
        audioPath_release(micPath);
    micPath = -1;
    dmr_endTx((uint32_t)(getTick() - txStart));
    radio_disableRtx();
    radio_enableRx();
    dmrDiag_txEnd();

    status->opStatus = RX;
    transmitting = false;
    platform_ledOff(RED);
}

void OpMode_DMR::startWakeup(rtxStatus_t *const status)
{
    dmrDiag_startWatchdog();
    stopRxAudio();
    radio_disableRtx();
    radio_enableTx();
    dmr_startWakeup(status->dmrRadioId);
    wakeTries++;
    wakePhase = WAKE_SENDING;
    wakeTicks = 0;
    platform_ledOn(RED);
}

void OpMode_DMR::updateWakeup(rtxStatus_t *const status, const bool ptt,
                              const bool bsLocked)
{
    wakeTicks++;

    if (wakePhase == WAKE_SENDING) {
        // The two CSBKs go out within four slots; back to receive after them
        if (!dmr_txDone() && (wakeTicks < 8))
            return;
        if (!dmr_txDone())
            LOG_W("dmr", "wake-up not sent within 240 ms");
        dmr_endTx(0);
        radio_disableRtx();
        radio_enableRx();
        platform_ledOff(RED);
        wakePhase = WAKE_WAITING;
        wakeTicks = 0;
        return;
    }

    // Waiting for the repeater
    if (!ptt) {
        LOG_I("dmr", "PTT released while waking the repeater");
        wakePhase = WAKE_NONE;
        return;
    }

    if (bsLocked) {
        LOG_I("dmr", "repeater answered %lu ms after the first wake-up (%u)",
              (unsigned long)(getTick() - wakeStart), wakeTries);
        wakePhase = WAKE_NONE;
        startTx(status);
        return;
    }

    if (wakeTicks >= WAKE_WAIT_TICKS) {
        if (wakeTries < WAKE_TRIES) {
            startWakeup(status);
        } else {
            LOG_W("dmr", "no answer from the repeater after %u wake-ups%s",
                  wakeTries, dmr_bsTail() ? " (terminator received)" : "");
            wakePhase = WAKE_NONE;
        }
    }
}

bool OpMode_DMR::rxSquelchOpen()
{
    dmrCallInfo_t info;
    return dmr_getCallInfo(&info);
}
