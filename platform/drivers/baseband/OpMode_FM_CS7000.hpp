/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef OPMODE_FM_CS7000_H
#define OPMODE_FM_CS7000_H

#include <memory>
#include "core/audio_path.h"
#include "core/audio_stream.h"
#include "rtx/OpMode.hpp"

/**
 * Second order IIR section (direct form I), for the USB receive audio.
 */
struct Biquad {
    float b0, b1, b2, a1, a2;
    float x1, x2, y1, y2;

    float step(const float x)
    {
        float y = (b0 * x) + (b1 * x1) + (b2 * x2) - (a1 * y1) - (a2 * y2);
        x2 = x1;
        x1 = x;
        y2 = y1;
        y1 = y;
        return y;
    }
};

/**
 * FM operating mode for the CS7000-M17 Plus: the generic FM mode (squelch,
 * CTCSS, microphone to transmitter, receiver to speaker through the
 * HR_C6000), plus the USB sound card:
 *
 * - while the computer listens, the receiver's audio is also sampled by the
 *   MCU at 48 kHz, filtered (300 Hz high pass for CTCSS, de-emphasis,
 *   3.5 kHz low pass) and sent to it when the squelch is open;
 * - a transmission the computer asked for (CAT) takes its audio from the
 *   computer: 48 kHz through the DAC to the HR_C6000 line input, which adds
 *   pre-emphasis, limiting and CTCSS as for the microphone.
 */
class OpMode_FM_CS7000 : public OpMode
{
public:
    OpMode_FM_CS7000();
    ~OpMode_FM_CS7000();

    virtual void enable() override;
    virtual void disable() override;
    virtual void update(rtxStatus_t *const status, const bool newCfg) override;

    virtual opmode getID() override
    {
        return OPMODE_FM;
    }

    virtual bool rxSquelchOpen() override;

private:
    void startUsbRx();
    void stopUsbRx();
    void usbRxBlock();
    bool startUsbTx();
    void usbTxBlock();
    void stopUsbTx();

    bool rfSqlOpen;
    bool sqlOpen;
    bool enterRx;
    pathId rxAudioPath;
    pathId txAudioPath;

    // USB receive: receiver audio sampled by the MCU
    bool usbRxOn;
    pathId usbRxPath;
    streamId usbRxStream;
    int32_t usbRxPeak; ///< Peak level sampled while the squelch was open
    Biquad hpf, deemph, lpf;

    // USB transmit: computer audio to the DAC
    bool usbTx;
    streamId usbTxStream;
    stream_sample_t *usbTxIdle;
    int32_t usbTxPeak;

    std::unique_ptr<int16_t[]> usbBuf; ///< Shared: RX and TX never overlap
};

#endif /* OPMODE_FM_CS7000_H */
