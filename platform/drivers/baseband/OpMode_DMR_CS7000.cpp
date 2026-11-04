/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

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
    , lastRxIrqs(0)
    , quietTicks(0)
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
}

void OpMode_DMR::disable()
{
    platform_ledOff(GREEN);
    platform_ledOff(RED);

    if (transmitting) {
        dmrAudio_stopTx();
        dmr_endTx((uint32_t)(getTick() - txStart));
        dmrDiag_txEnd();
        transmitting = false;
    }

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
     * Transmit (bring-up, v40: direct mode, 1 kHz test tone).  A press of the
     * PTT starts one transmission; releasing it, or the time-out, sends the
     * terminator, and once it is out the radio goes back to receive.
     */
    // Is the chip's slot timing running? (see restartReceiver)
    uint32_t irqs = dmr_rxSlotIrqs();
    if (irqs != lastRxIrqs)
        quietTicks = 0;
    else if (quietTicks < 255)
        quietTicks++;
    lastRxIrqs = irqs;

    bool ptt = platform_getPttStatus();
    if (!ptt)
        pttHeld = false;

    if (!transmitting && ptt && !pttHeld && (status->opStatus == RX)) {
        pttHeld = true;
        if (status->txDisable != 0)
            LOG_W("dmr", "PTT: transmit disabled on this channel");
        else if (status->dmrRadioId == 0)
            LOG_W("dmr", "PTT: no radio ID set (Settings > DMR)");
        else {
            /*
             * The transmitter needs the chip's slot interrupt, which only
             * runs while the receiver is searching or locked.  On air
             * (v40..v43), every press made while it had stopped sent
             * nothing.  Restart it first and give it 300 ms.
             */
            if (quietTicks >= 5) {
                restartReceiver();
                uint32_t before = dmr_rxSlotIrqs();
                for (int i = 0; (i < 10) && (dmr_rxSlotIrqs() == before); i++)
                    sleepFor(0u, 30u);
                LOG_I("dmr",
                      "PTT: slot interrupt was stopped, receiver "
                      "restarted: %s",
                      (dmr_rxSlotIrqs() != before) ? "running again" :
                                                     "still stopped");
            }
            startTx(status);
        }
    }

    if (transmitting) {
        txTicks++;
        dmrDiag_inc(DIAG_BEAT);
        if ((txTicks == 10) && (dmr_txSlotIrqs() == 0))
            dmr_kickTx();
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

    if (!dmrAudio_startTx()) {
        LOG_W("dmr", "PTT: no voice encoder (codec missing or damaged)");
        dmrDiag_txEnd();
        return;
    }
    dmrDiag_set(DIAG_STAGE, STAGE_ENCODER);

    uint32_t tg = status->dmrRxTalkgroup;
    if (tg == 0)
        tg = DEFAULT_TX_TG;

    radio_disableRtx();
    radio_enableTx();
    dmrDiag_set(DIAG_STAGE, STAGE_KEYED);
    dmr_startTx(status->dmrRadioId, tg);

    status->opStatus = TX;
    transmitting = true;
    txTicks = 0;
    stopTick = 0;
    txStart = getTick();
    platform_ledOn(RED);
}

/*
 * Full DMR set-up of the chip (reset, dmrMode(), then colour code and
 * timeslot), as at mode entry.  After a transmission the slot interrupt did
 * not come back (v43 log: none until a call was received), and a
 * transmission needs it.
 */
void OpMode_DMR::restartReceiver()
{
    radio_disableRtx();
    radio_setOpmode(OPMODE_DMR);
    radio_updateConfiguration();
    radio_enableRx();
}

void OpMode_DMR::finishTx(rtxStatus_t *const status)
{
    dmrDiag_set(DIAG_STAGE, STAGE_FINISHING);
    dmrAudio_stopTx();
    dmr_endTx((uint32_t)(getTick() - txStart));
    restartReceiver();
    dmrDiag_txEnd();

    status->opStatus = RX;
    transmitting = false;
    platform_ledOff(RED);
}

bool OpMode_DMR::rxSquelchOpen()
{
    dmrCallInfo_t info;
    return dmr_getCallInfo(&info);
}
