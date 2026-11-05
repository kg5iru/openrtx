/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <interfaces/platform.h>
#include <interfaces/delays.h>
#include <interfaces/audio.h>
#include <interfaces/radio.h>
#include <cstring>
#include <cstdint>
#include "protocols/P25/P25Defines.h"
#include "rtx/OpMode_P25.hpp"
#include "rtx/rtx.h"
#include "core/settings.h"
#include "protocols/P25/modem/AudioCodec.hpp"
#include "protocols/P25/modem/Nid.hpp"

#ifdef PLATFORM_MOD17
#include "calibration/calibInfo_Mod17.h"
extern mod17Calib_t mod17CalData;
#endif

using namespace P25;

namespace
{

constexpr size_t IMBE_PER_LDU = 9;

/*
 * Status symbols, one dibit every 70 bits of a unit: a subscriber unit
 * sends them as "unknown, talk-around".
 */
void addStatusSymbols(uint8_t *data, const size_t bits)
{
    for (size_t pos = P25_SS0_START; pos < bits; pos += P25_SS_INCREMENT) {
        data[pos >> 3] |= 0x80 >> (pos & 7);
        data[(pos + 1) >> 3] &= ~(0x80 >> ((pos + 1) & 7));
    }
}

void addSyncAndNid(uint8_t *data, const uint8_t duid, const uint16_t nac)
{
    for (size_t i = 0; i < FS_SYNC_BYTES.size(); i++)
        data[i] = FS_SYNC_BYTES[i];
    uint8_t nid[Nid::BYTES];
    Nid::encode(nac, duid, nid);
    Nid::toFrame(nid, data);
}

} // namespace

OpMode_P25::OpMode_P25()
    : startRx(false)
    , startTx(false)
    , locked(false)
    , dataValid(false)
    , invertTxPhase(false)
    , invertRxPhase(false)
    , ldu2Next(false)
    , callForUs(false)
    , txNac(0)
    , rxAudioPath(-1)
    , txAudioPath(-1)
{
}

OpMode_P25::~OpMode_P25()
{
    disable();
}

void OpMode_P25::enable()
{
    AudioCodec::init();
    modulator.init();
    demodulator.init();

    locked = false;
    dataValid = false;
    startRx = true;
    startTx = false;
}

void OpMode_P25::disable()
{
    startRx = false;
    startTx = false;
    locked = false;
    dataValid = false;

    stopRxAudio();
    AudioCodec::stop(txAudioPath);
    audioPath_release(txAudioPath);
    txAudioPath = -1;

    modulator.terminate();
    demodulator.terminate();
    AudioCodec::terminate();

    platform_ledOff(GREEN);
    platform_ledOff(RED);
    radio_disableRtx();
}

void OpMode_P25::update(rtxStatus_t *const status, const bool newCfg)
{
    (void)newCfg;

#if defined(PLATFORM_MOD17)
    invertTxPhase = (mod17CalData.bb_tx_invert == 1) ? false : true;
    invertRxPhase = (mod17CalData.bb_rx_invert == 1) ? false : true;
#else
    invertTxPhase = status->p25TxInvert;
    invertRxPhase = status->p25RxInvert;
#endif

    // Main FSM logic
    switch (status->opStatus) {
        case OFF:
            offState(status);
            break;

        case RX:
            rxState(status);
            break;

        case TX:
            txState(status);
            break;

        default:
            break;
    }

    // Led control logic
    switch (status->opStatus) {
        case RX:
            if (dataValid)
                platform_ledOn(GREEN);
            else
                platform_ledOff(GREEN);
            break;

        case TX:
            platform_ledOff(GREEN);
            platform_ledOn(RED);
            break;

        default:
            platform_ledOff(GREEN);
            platform_ledOff(RED);
            break;
    }
}

void OpMode_P25::offState(rtxStatus_t *const status)
{
    radio_disableRtx();

    if (startRx) {
        status->opStatus = RX;
        return;
    }

    if (platform_getPttStatus() && (status->txDisable == 0)) {
        startTx = true;
        status->opStatus = TX;
        return;
    }

    // Sleep for 30ms if there is nothing else to do in order to prevent the
    // rtx thread looping endlessly and locking up all the other tasks
    sleepFor(0, 30);
}

void OpMode_P25::rxState(rtxStatus_t *const status)
{
    if (startRx) {
        demodulator.startBasebandSampling();
        radio_enableRx();

        startRx = false;
        locked = false;
        dataValid = false;
        callForUs = false;
        rxData.reset();
    }

    bool newUnit = demodulator.update(invertRxPhase);
    bool lock = demodulator.isLocked();

    // Lock acquired: start from a clean link control.
    if (lock && (locked == false)) {
        rxData.reset();
        callForUs = false;
    }

    locked = lock;

    if (newUnit)
        handleUnit(status);

    // Lock lost, for whatever cause: close the audio and invalidate the call
    // data.
    if (locked == false) {
        stopRxAudio();
        dataValid = false;
        status->lsfOk = false;
    }

    if (platform_getPttStatus()) {
        demodulator.stopBasebandSampling();
        stopRxAudio();
        locked = false;
        dataValid = false;
        status->lsfOk = false;
        status->opStatus = OFF;
    }
}

void OpMode_P25::handleUnit(rtxStatus_t *const status)
{
    const frame_t &unit = demodulator.getFrame();
    uint8_t duid = demodulator.getDuid();
    uint16_t nac = demodulator.getNac();

    // NAC screening
    if ((status->p25Nac != NAC_ANY) && (nac != status->p25Nac))
        return;

    switch (duid) {
        case Nid::DUID_HDU:
            rxData.reset();
            callForUs = false;
            break;

        case Nid::DUID_LDU1:
        case Nid::DUID_LDU2: {
            // Working copy: the FEC repair modifies the unit in place.
            memcpy(txUnit, unit.data(), 216);
            audio.process(txUnit);

            if (duid == Nid::DUID_LDU1) {
                // The link control carries the call's addresses: the
                // decision taken here holds for the LDU2s that follow.
                if (rxData.decodeLDU1(txUnit)) {
                    status->P25_SrcId = rxData.getSrcId();
                    status->P25_DstId = rxData.getDstId();
                    callForUs = callAccepted(status);
                }
            } else {
                rxData.decodeLDU2(txUnit);
            }

            dataValid = true;
            status->lsfOk = true;

            // Audio only for calls addressed to us; a call entered on an
            // LDU2 stays muted until its LDU1 tells who it is for.
            if (callForUs == false) {
                stopRxAudio();
                break;
            }

            uint8_t pthSts = audioPath_getStatus(rxAudioPath);
            if (pthSts == PATH_CLOSED) {
                rxAudioPath = audioPath_request(SOURCE_MCU, SINK_SPK, PRIO_RX);
                pthSts = audioPath_getStatus(rxAudioPath);
            }

            if (pthSts == PATH_OPEN) {
                if (AudioCodec::running() == false)
                    AudioCodec::startDecode(rxAudioPath);

                uint8_t imbe[AudioCodec::FRAME_BYTES];
                for (unsigned n = 0; n < IMBE_PER_LDU; n++) {
                    audio.decode(txUnit, imbe, n);
                    AudioCodec::pushFrame(imbe, false);
                }
            }
        } break;

        case Nid::DUID_TDU:
        case Nid::DUID_TDULC:
            // End of the call
            stopRxAudio();
            dataValid = false;
            status->lsfOk = false;
            rxData.reset();
            callForUs = false;
            break;

        default:
            break;
    }
}

bool OpMode_P25::callAccepted(const rtxStatus_t *const status)
{
    // Destination check disabled: every call is played, like M17 with the
    // CAN check off.
    if (status->p25DstCheck == 0)
        return true;

    uint32_t dst = rxData.getDstId();

    if (rxData.getLCF() == P25_LCF_PRIVATE)
        return (dst == status->p25SrcId);

    return (dst == status->p25DstId) || (dst == TGID_ALL_CALL);
}

void OpMode_P25::stopRxAudio()
{
    if (rxAudioPath < 0)
        return;

    AudioCodec::stop(rxAudioPath);
    audioPath_release(rxAudioPath);
    rxAudioPath = -1;
}

void OpMode_P25::sendVoiceUnit(const bool ldu2)
{
    memset(txUnit, 0x00, sizeof(txUnit));
    addSyncAndNid(txUnit, ldu2 ? Nid::DUID_LDU2 : Nid::DUID_LDU1, txNac);

    if (ldu2)
        txData.encodeLDU2(txUnit);
    else
        txData.encodeLDU1(txUnit);

    // Nine 20 ms IMBE frames; silence if the codec has nothing (path lost)
    static const uint8_t silence[AudioCodec::FRAME_BYTES] = {
        0x04, 0x0C, 0xFD, 0x7B, 0xFB, 0x7D, 0xF2, 0x7B, 0x3D, 0x9E, 0x45
    };
    for (unsigned n = 0; n < IMBE_PER_LDU; n++) {
        uint8_t imbe[AudioCodec::FRAME_BYTES];
        if (AudioCodec::popFrame(imbe, true) != 0)
            memcpy(imbe, silence, sizeof(imbe));
        audio.encode(txUnit, imbe, n);
    }

    lsd.encode(txUnit);
    addStatusSymbols(txUnit, P25_LDU_FRAME_LENGTH_BITS);
    modulator.sendFrame(txUnit, P25_LDU_FRAME_LENGTH_BYTES);
}

void OpMode_P25::txState(rtxStatus_t *const status)
{
    if (startTx) {
        startTx = false;

        txAudioPath = audioPath_request(SOURCE_MIC, SINK_MCU, PRIO_TX);
        AudioCodec::startEncode(txAudioPath);
        radio_enableTx();

        // Group voice call: an individual call to a talkgroup ID is shown
        // as such by factory radios and their screening mutes the audio.
        txNac = status->p25Nac;
        txData.reset();
        txData.setLCF(P25_LCF_GROUP);
        txData.setSrcId(status->p25SrcId);
        txData.setDstId(status->p25DstId);
        lsd.setLSD1(0x00);
        lsd.setLSD2(0x00);

        modulator.invertPhase(invertTxPhase);
        modulator.setLevel(status->p25TxLevel * LEVEL_COUNTS_PER_STEP);
        if (modulator.start() == false) {
            // Baseband output not available: do not key a dead carrier.
            AudioCodec::stop(txAudioPath);
            audioPath_release(txAudioPath);
            txAudioPath = -1;
            startRx = true;
            status->opStatus = OFF;
            return;
        }

        // Preamble pattern, from the settings: a bring-up/interoperability
        // aid. Out-of-range values (stale settings blobs) fall back to the
        // default option.
        uint8_t preamble = status->p25Preamble;
        if (preamble >= P25_PREAMBLE_NUM)
            preamble = P25_PREAMBLE_KENWOOD_SHORT;
        switch (preamble) {
            case P25_PREAMBLE_TIA180:
                modulator.sendPreamble(Modulator::PREAMBLE_TIA, 2,
                                       FRAME_SYMBOLS);
                break;
            case P25_PREAMBLE_MMDVM:
                modulator.sendPreamble(Modulator::PREAMBLE_MMDVM, 2,
                                       FRAME_SYMBOLS);
                break;
            case P25_PREAMBLE_KENWOOD_SHORT:
                modulator.sendPreamble(Modulator::PREAMBLE_KENWOOD, 4, 144 * 4);
                break;
            case P25_PREAMBLE_KENWOOD_LONG:
                modulator.sendPreamble(Modulator::PREAMBLE_KENWOOD, 4, 255 * 4);
                break;
            case P25_PREAMBLE_SWEEP_SHORT:
                modulator.sendPreamble(Modulator::PREAMBLE_SWEEP, 4, 144 * 4);
                break;
            case P25_PREAMBLE_SWEEP_LONG:
                modulator.sendPreamble(Modulator::PREAMBLE_SWEEP, 4, 255 * 4);
                break;
        }

        // Header data unit
        memset(txUnit, 0x00, sizeof(txUnit));
        addSyncAndNid(txUnit, Nid::DUID_HDU, txNac);
        txData.encodeHeader(txUnit);
        addStatusSymbols(txUnit, P25_HDR_FRAME_LENGTH_BITS);
        modulator.sendFrame(txUnit, P25_HDR_FRAME_LENGTH_BYTES);

        ldu2Next = false;
    }

    // One voice unit per call: the codec paces us at 180 ms per unit.
    sendVoiceUnit(ldu2Next);
    ldu2Next = !ldu2Next;

    if (platform_getPttStatus() == false) {
        // Complete the LDU pair, then terminate the call
        if (ldu2Next) {
            sendVoiceUnit(true);
            ldu2Next = false;
        }

        memset(txUnit, 0x00, sizeof(txUnit));
        addSyncAndNid(txUnit, Nid::DUID_TDU, txNac);
        addStatusSymbols(txUnit, P25_TERM_FRAME_LENGTH_BITS);
        modulator.sendFrame(txUnit, P25_TERM_FRAME_LENGTH_BYTES);
        modulator.stop();

        AudioCodec::stop(txAudioPath);
        audioPath_release(txAudioPath);
        txAudioPath = -1;

        startRx = true;
        status->opStatus = OFF;
    }
}
