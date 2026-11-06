/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <cmath>
#include <cstring>
#include "interfaces/platform.h"
#include "interfaces/delays.h"
#include "interfaces/radio.h"
#include "core/log.h"
#include "rtx/rtx.h"
#include "usb_audio.h"
#include "radio_CS7000.h"
#include "OpMode_FM_CS7000.hpp"

static constexpr uint32_t USB_RATE = 48000;
static constexpr size_t BLOCK = USB_RATE / 50; // 20 ms

// RBJ cookbook sections at 48 kHz
static Biquad lowPass(const float f0, const float q)
{
    const float w0 = 2.0f * (float)M_PI * f0 / (float)USB_RATE;
    const float alpha = sinf(w0) / (2.0f * q);
    const float c = cosf(w0);
    const float a0 = 1.0f + alpha;
    Biquad s = {};
    s.b0 = ((1.0f - c) / 2.0f) / a0;
    s.b1 = (1.0f - c) / a0;
    s.b2 = s.b0;
    s.a1 = (-2.0f * c) / a0;
    s.a2 = (1.0f - alpha) / a0;
    return s;
}

static Biquad highPass(const float f0, const float q)
{
    const float w0 = 2.0f * (float)M_PI * f0 / (float)USB_RATE;
    const float alpha = sinf(w0) / (2.0f * q);
    const float c = cosf(w0);
    const float a0 = 1.0f + alpha;
    Biquad s = {};
    s.b0 = ((1.0f + c) / 2.0f) / a0;
    s.b1 = -(1.0f + c) / a0;
    s.b2 = s.b0;
    s.a1 = (-2.0f * c) / a0;
    s.a2 = (1.0f - alpha) / a0;
    return s;
}

// First order low pass (de-emphasis), as a biquad
static Biquad onePole(const float f0)
{
    const float k = tanf((float)M_PI * f0 / (float)USB_RATE);
    Biquad s = {};
    s.b0 = k / (1.0f + k);
    s.b1 = s.b0;
    s.b2 = 0.0f;
    s.a1 = (k - 1.0f) / (k + 1.0f);
    s.a2 = 0.0f;
    return s;
}

/*
 * De-emphasis corner, and the gain that brings 1 kHz back to 0 dB:
 * sqrt(1 + (1000 / 300)^2).
 */
static constexpr float DEEMPH_HZ = 300.0f;
static constexpr float DEEMPH_GAIN = 3.48f;

OpMode_FM_CS7000::OpMode_FM_CS7000()
    : rfSqlOpen(false)
    , sqlOpen(false)
    , enterRx(true)
    , rxAudioPath(-1)
    , txAudioPath(-1)
    , usbRxOn(false)
    , usbRxPath(-1)
    , usbRxStream(-1)
    , usbRxPeak(0)
    , hpf{}
    , deemph{}
    , lpf{}
    , usbTx(false)
    , usbTxStream(-1)
    , usbTxIdle(nullptr)
    , usbTxPeak(0)
{
}

OpMode_FM_CS7000::~OpMode_FM_CS7000()
{
}

void OpMode_FM_CS7000::enable()
{
    rfSqlOpen = false;
    sqlOpen = false;
    enterRx = true;
    usbRxOn = false;
    usbTx = false;
    usbBuf = std::make_unique<int16_t[]>(2 * BLOCK);
    hpf = highPass(300.0f, 0.707f);
    deemph = onePole(DEEMPH_HZ);
    lpf = lowPass(3500.0f, 0.707f);
}

void OpMode_FM_CS7000::disable()
{
    platform_ledOff(GREEN);
    platform_ledOff(RED);
    stopUsbRx();
    stopUsbTx();
    audioPath_release(rxAudioPath);
    audioPath_release(txAudioPath);
    radio_disableRtx();
    radio_setFmTxLineIn(false);
    rfSqlOpen = false;
    sqlOpen = false;
    enterRx = false;
    usbBuf.reset();
}

void OpMode_FM_CS7000::startUsbRx()
{
    if (usbRxOn || !usbBuf)
        return;

    usbRxPath = audioPath_request(SOURCE_RTX, SINK_MCU, PRIO_RX);
    if (usbRxPath < 0)
        return;

    usbRxStream = audioStream_start(usbRxPath, usbBuf.get(), 2 * BLOCK,
                                    USB_RATE, STREAM_INPUT | BUF_CIRC_DOUBLE);
    if (usbRxStream < 0) {
        audioPath_release(usbRxPath);
        usbRxPath = -1;
        return;
    }

    usbRxOn = true;
    usbRxPeak = 0;
    LOG_I("fm", "USB receive audio on");
}

void OpMode_FM_CS7000::stopUsbRx()
{
    if (!usbRxOn)
        return;

    audioStream_terminate(usbRxStream);
    audioPath_release(usbRxPath);
    usbRxStream = -1;
    usbRxPath = -1;
    usbRxOn = false;
    LOG_I("fm", "USB receive audio off");
}

// One 20 ms block of receiver audio to the computer (blocks until it is in)
void OpMode_FM_CS7000::usbRxBlock()
{
    dataBlock_t bb = inputStream_getData(usbRxStream);
    if ((bb.data == NULL) || !sqlOpen)
        return;

    int16_t out[BLOCK];
    size_t n = (bb.len < BLOCK) ? bb.len : BLOCK;
    for (size_t i = 0; i < n; i++) {
        int32_t a = (bb.data[i] < 0) ? -(int32_t)bb.data[i] : bb.data[i];
        if (a > usbRxPeak)
            usbRxPeak = a;

        float v = hpf.step((float)bb.data[i]);
        v = deemph.step(v) * DEEMPH_GAIN;
        v = lpf.step(v);
        if (v > 32767.0f)
            v = 32767.0f;
        if (v < -32768.0f)
            v = -32768.0f;
        out[i] = (int16_t)v;
    }

    usbAudio_write(out, n);
}

bool OpMode_FM_CS7000::startUsbTx()
{
    if (!usbBuf)
        return false;

    txAudioPath = audioPath_request(SOURCE_MCU, SINK_RTX, PRIO_TX);
    if (txAudioPath < 0)
        return false;

    memset(usbBuf.get(), 0, 2 * BLOCK * sizeof(int16_t));
    usbTxStream = audioStream_start(txAudioPath, usbBuf.get(), 2 * BLOCK,
                                    USB_RATE, STREAM_OUTPUT | BUF_CIRC_DOUBLE);
    if (usbTxStream < 0) {
        audioPath_release(txAudioPath);
        txAudioPath = -1;
        return false;
    }

    usbTxIdle = outputStream_getIdleBuffer(usbTxStream);
    usbTxPeak = 0;
    usbAudio_flushTx();
    return true;
}

/*
 * One 20 ms block of computer audio to the transmitter: fill the idle half,
 * then wait for the half being played (radio clock; the resampler follows
 * the computer's).
 */
void OpMode_FM_CS7000::usbTxBlock()
{
    if ((usbTxIdle == nullptr)
        || (audioPath_getStatus(txAudioPath) != PATH_OPEN)) {
        sleepFor(0u, 20u);
        return;
    }

    usbAudio_readResampled(usbTxIdle, BLOCK);
    for (size_t i = 0; i < BLOCK; i++) {
        int32_t a = (usbTxIdle[i] < 0) ? -(int32_t)usbTxIdle[i] : usbTxIdle[i];
        if (a > usbTxPeak)
            usbTxPeak = a;
    }

    outputStream_sync(usbTxStream, true);
    usbTxIdle = outputStream_getIdleBuffer(usbTxStream);
}

void OpMode_FM_CS7000::stopUsbTx()
{
    if (!usbTx)
        return;

    if (usbTxStream >= 0)
        audioStream_stop(usbTxStream); // the half playing goes out
    usbTxStream = -1;
    usbTxIdle = nullptr;
    usbTx = false;
    LOG_I("fm", "USB transmit audio end, peak %ld", (long)usbTxPeak);
}

void OpMode_FM_CS7000::update(rtxStatus_t *const status, const bool newCfg)
{
    (void)newCfg;

    // RX logic, as the generic FM mode
    if (status->opStatus == RX) {
        // This turns squelch (0 to 15) into RSSI (-127.0dbm to -61dbm)
        rssi_t squelch = -127 + (status->sqlLevel * 66) / 15;
        rssi_t rssi = rtx_getRssi();

        // A bit of hysteresis: change state only if the RSSI moved more
        // than 1 dBm on either side of the squelch setting
        if ((rfSqlOpen == false) && (rssi > (squelch + 1)))
            rfSqlOpen = true;
        if ((rfSqlOpen == true) && (rssi < (squelch - 1)))
            rfSqlOpen = false;

        bool rfSql = ((status->rxToneEn == 0) && (rfSqlOpen == true));
        bool toneSql = ((status->rxToneEn == 1)
                        && radio_checkRxDigitalSquelch());

        if ((sqlOpen == false) && (rfSql || toneSql)) {
            rxAudioPath = audioPath_request(SOURCE_RTX, SINK_SPK, PRIO_RX);
            if (rxAudioPath > 0) {
                sqlOpen = true;
                usbRxPeak = 0;
            }
        }

        if ((sqlOpen == true) && (rfSql == false) && (toneSql == false)) {
            audioPath_release(rxAudioPath);
            sqlOpen = false;
            if (usbRxOn)
                LOG_I("fm", "USB receive audio: squelch closed, peak %ld",
                      (long)usbRxPeak);
        }

        // Receiver audio to the computer while it listens
        if (usbAudio_rxStreaming())
            startUsbRx();
        else
            stopUsbRx();
    } else if ((status->opStatus == OFF) && enterRx) {
        radio_disableRtx();

        radio_enableRx();
        status->opStatus = RX;
        enterRx = false;
    }

    // TX logic: the microphone, or the computer when it asked (CAT)
    bool ptt = platform_getPttStatus();
    if (ptt && (status->opStatus != TX) && (status->txDisable == 0)) {
        audioPath_release(rxAudioPath);
        stopUsbRx();
        radio_disableRtx();

        usbTx = usbAudio_ptt();
        if (usbTx && !startUsbTx()) {
            LOG_W("fm", "USB transmit audio: no output path");
            usbTx = false;
        }

        if (usbTx) {
            radio_setFmTxLineIn(true);
            LOG_I("fm", "USB transmit audio start");
        } else {
            radio_setFmTxLineIn(false);
            txAudioPath = audioPath_request(SOURCE_MIC, SINK_RTX, PRIO_TX);
        }

        radio_enableTx();
        status->opStatus = TX;
    }

    if (!ptt && (status->opStatus == TX)) {
        stopUsbTx();
        audioPath_release(txAudioPath);
        radio_disableRtx();
        radio_setFmTxLineIn(false);

        status->opStatus = OFF;
        enterRx = true;
        sqlOpen = false; // Force squelch to be redetected.
    }

    // LED control logic
    switch (status->opStatus) {
        case RX:
            if (radio_checkRxDigitalSquelch()) {
                platform_ledOn(GREEN); // Red + green: tone squelch open
                platform_ledOn(RED);
            } else if (rfSqlOpen) {
                platform_ledOn(GREEN); // Green only: RF squelch open
                platform_ledOff(RED);
            } else {
                platform_ledOff(GREEN);
                platform_ledOff(RED);
            }
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

    // Pace: the USB audio streams (20 ms blocks), or 30 ms as the generic
    // FM mode
    if ((status->opStatus == TX) && usbTx)
        usbTxBlock();
    else if ((status->opStatus == RX) && usbRxOn)
        usbRxBlock();
    else
        sleepFor(0u, 30u);
}

bool OpMode_FM_CS7000::rxSquelchOpen()
{
    return sqlOpen;
}
