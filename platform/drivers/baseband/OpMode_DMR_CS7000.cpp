/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <stdio.h>
#include <string.h>
#include "interfaces/platform.h"
#include "interfaces/radio.h"
#include "interfaces/delays.h"
#include "core/audio_path.h"
#include "core/log.h"
#include "core/state.h"
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
 * GPS position (digital APRS) packet, as the owner's Anytone sends it (v55
 * off-air capture): a compressed UDP/IPv4 header (IPv4 identification, no
 * address or port IDs, then source and destination port 10110), then an
 * NMEA RMC sentence.  BrandMeister forwards it to APRS-IS when the user's
 * profile has APRS enabled.  Returns 0 without a GPS fix.
 */
static size_t buildPosition(uint8_t *out, size_t max, uint16_t ipId)
{
    const gps_t *g = &state.gps_data;
    if ((g->fix_quality == 0) || (max < 96))
        return 0;

    out[0] = ipId >> 8;
    out[1] = ipId & 0xFF;
    out[2] = 0x00;
    out[3] = 0x00;
    out[4] = 0x00;
    out[5] = 0x27; // UDP port 10110
    out[6] = 0x7E;
    out[7] = 0x27;
    out[8] = 0x7E;

    // Coordinates are in millionths of a degree: ddmm.mmmmm, the minutes
    // times 1e5 being the fraction of a degree times 6
    uint32_t lat = (g->latitude < 0) ? -g->latitude : g->latitude;
    uint32_t lon = (g->longitude < 0) ? -g->longitude : g->longitude;
    uint32_t latMin = (lat % 1000000) * 6;
    uint32_t lonMin = (lon % 1000000) * 6;
    uint32_t knots100 = ((uint32_t)g->speed * 100000u) / 1852u;
    int course = g->tmg_true;
    if ((course < 0) || (course > 359))
        course = 0;

    /*
     * The integer-only sniprintf, which the log and the menus already use:
     * snprintf alone pulled in the floating point printf (5 kB).
     */
    char *s = (char *)&out[9];
    size_t room = max - 9 - 6; // room for the checksum and CR LF
    int n = sniprintf(
        s, room,
        "$GPRMC,%02d%02d%02d.000,A,%02lu%02lu.%05lu,%c,"
        "%03lu%02lu.%05lu,%c,%lu.%02lu,%d.00,%02d%02d%02d,,,A",
        g->timestamp.hour, g->timestamp.minute, g->timestamp.second,
        (unsigned long)(lat / 1000000), (unsigned long)(latMin / 100000),
        (unsigned long)(latMin % 100000), (g->latitude < 0) ? 'S' : 'N',
        (unsigned long)(lon / 1000000), (unsigned long)(lonMin / 100000),
        (unsigned long)(lonMin % 100000), (g->longitude < 0) ? 'W' : 'E',
        (unsigned long)(knots100 / 100), (unsigned long)(knots100 % 100),
        course, g->timestamp.date, g->timestamp.month, g->timestamp.year);
    if ((n <= 0) || ((size_t)n >= room))
        return 0;

    uint8_t cs = 0;
    for (int i = 1; i < n; i++)
        cs ^= (uint8_t)s[i];
    n += sniprintf(&s[n], 6, "*%02X\r\n", cs);
    return 9 + (size_t)n;
}

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
    , txIsData(false)
    , wakeForData(false)
    , aprsPending(false)
    , aprsLast(0)
    , aprsIpId(1)
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
    aprsPending = false;
    aprsLast = getTick();
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
            wakeForData = false;
            startWakeup(status);
        } else {
            if (status->dmrTimeslot != 0)
                LOG_I("dmr", "repeater on the air: no wake-up");
            startTx(status);
        }
    }

    /*
     * GPS position (digital APRS): after each transmission and/or every
     * interval, sent once the radio is idle (no transmission, wake-up, PTT
     * or call being received).  Through a repeater, woken up first like a
     * PTT press.
     */
    if ((status->dmrAprsId != 0) && (status->dmrAprsInterval != 0)
        && ((getTick() - aprsLast) >= (status->dmrAprsInterval * 1000LL))) {
        aprsLast = getTick();
        aprsPending = true;
    }
    if (aprsPending && !transmitting && (wakePhase == WAKE_NONE) && !ptt
        && !rxSquelchOpen() && (status->opStatus == RX)) {
        aprsPending = false;
        if ((status->dmrAprsId == 0) || (status->txDisable != 0)
            || (status->dmrRadioId == 0)) {
            // Not configured or not allowed: nothing to send
        } else if (state.gps_data.fix_quality == 0) {
            LOG_W("dmr", "APRS: no GPS fix, position not sent");
        } else if ((status->dmrTimeslot != 0) && !bsLocked) {
            wakeTries = 0;
            wakeStart = getTick();
            wakeForData = true;
            startWakeup(status);
        } else {
            startData(status);
        }
    }

    if (wakePhase != WAKE_NONE)
        updateWakeup(status, ptt, bsLocked);

    if (transmitting && txIsData) {
        // A position: about 15 bursts, out in under a second
        txTicks++;
        dmrDiag_inc(DIAG_BEAT);
        if ((txTicks == 10) && dmr_txAligning()) {
            LOG_W("dmr", "repeater slot not found in 300 ms: waking it up");
            finishTx(status);
            wakeTries = 0;
            wakeStart = getTick();
            wakeForData = true;
            startWakeup(status);
        } else if (dmr_txDone() || (txTicks >= 67)) {
            if (!dmr_txDone())
                LOG_W("dmr", "position not sent within 2 s: forced end");
            finishTx(status);
        }
    } else if (transmitting) {
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
                wakeForData = false;
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
    txIsData = false;
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

    // After a voice transmission, the position (APRS on PTT)
    if (!txIsData && (status->dmrAprsId != 0) && status->dmrAprsPtt)
        aprsPending = true;
    txIsData = false;
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
        // The two CSBKs and two slots of margin (six slots), then receive
        if (!dmr_txDone() && (wakeTicks < 12))
            return;
        if (!dmr_txDone())
            LOG_W("dmr", "wake-up not sent within 360 ms");
        dmr_endTx(0);
        radio_disableRtx();
        radio_enableRx();
        platform_ledOff(RED);
        wakePhase = WAKE_WAITING;
        wakeTicks = 0;
        return;
    }

    // Waiting for the repeater
    if (!ptt && !wakeForData) {
        LOG_I("dmr", "PTT released while waking the repeater");
        wakePhase = WAKE_NONE;
        return;
    }

    if (bsLocked) {
        LOG_I("dmr", "repeater answered %lu ms after the first wake-up (%u)",
              (unsigned long)(getTick() - wakeStart), wakeTries);
        wakePhase = WAKE_NONE;
        if (wakeForData)
            startData(status);
        else
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

bool OpMode_DMR::startData(rtxStatus_t *const status)
{
    uint8_t msg[100];
    size_t len = buildPosition(msg, sizeof(msg), aprsIpId);
    if (len == 0) {
        LOG_W("dmr", "APRS: no GPS fix, position not sent");
        return false;
    }

    dmrDiag_startWatchdog();
    dmrDiag_txStart();
    stopRxAudio();

    bool ok;
    if (status->dmrTimeslot != 0) {
        ok = dmr_startData(status->dmrRadioId, status->dmrAprsId,
                           status->dmrTimeslot, true, msg, len);
        if (ok) {
            radio_disableRtx();
            radio_enableTx();
        }
    } else {
        radio_disableRtx();
        radio_enableTx();
        ok = dmr_startData(status->dmrRadioId, status->dmrAprsId, 0, true, msg,
                           len);
        if (!ok) {
            radio_disableRtx();
            radio_enableRx();
        }
    }
    if (!ok) {
        LOG_W("dmr", "APRS: position too long (%u bytes)", (unsigned)len);
        dmrDiag_txEnd();
        return false;
    }

    LOG_I("dmr", "APRS: position %u to %lu", aprsIpId,
          (unsigned long)status->dmrAprsId);
    aprsIpId++;
    micPath = -1;
    status->opStatus = TX;
    transmitting = true;
    txIsData = true;
    txTicks = 0;
    stopTick = 0;
    txStart = getTick();
    platform_ledOn(RED);
    return true;
}

bool OpMode_DMR::rxSquelchOpen()
{
    dmrCallInfo_t info;
    return dmr_getCallInfo(&info);
}
