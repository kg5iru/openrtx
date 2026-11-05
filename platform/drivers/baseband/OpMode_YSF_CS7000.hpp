/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef OPMODE_YSF_CS7000_H
#define OPMODE_YSF_CS7000_H

#include <memory>
#include "core/audio_path.h"
#include "core/audio_stream.h"
#include "rtx/OpMode.hpp"
#include "protocols/YSF/Demodulator.hpp"
#include "protocols/YSF/FrameDecoder.hpp"
#include "protocols/YSF/Modulator.hpp"

/**
 * Yaesu System Fusion operating mode for the CS7000-M17 Plus.  The C4FM
 * baseband goes through the same paths M17 uses, (de)modulated in software
 * (protocols/YSF).  Voice (V/D modes 1 and 2 received, mode 2 sent; AMBE+2
 * half rate) goes through the codec of the stock firmware, with the queues
 * and threads the DMR mode uses (dmr_audio_CS7000).  Voice full rate (VW,
 * IMBE) is not decoded.
 */
class OpMode_YSF : public OpMode
{
public:
    OpMode_YSF();
    ~OpMode_YSF();

    virtual void enable() override;
    virtual void disable() override;
    virtual void update(rtxStatus_t *const status, const bool newCfg) override;

    virtual opmode getID() override
    {
        return OPMODE_YSF;
    }

    virtual bool rxSquelchOpen() override;

private:
    void startRx();
    void stopRx();
    void processFrame(const YSF::frame_t &frame);
    void startCall();
    void pushVoice(const YSF::frame_t &frame, const uint8_t dt);
    void endCall(const char *why);
    void startTx(rtxStatus_t *const status);
    void sendVoiceFrame();
    void sendControlFrame(const uint8_t fi);
    void endTx(rtxStatus_t *const status);

    bool enterRx;
    bool rxOn;
    pathId basebandPath;
    streamId basebandId;
    uint16_t framesSinceGood; ///< Frames since the last valid FICH
    uint8_t unlockedTicks;    ///< Updates without a lock during a call
    uint8_t lastDt;           ///< Data type of the last valid FICH
    uint8_t flywheel;         ///< Frames in a row decoded on lastDt

    std::unique_ptr<int16_t[]> basebandBuf;
    std::unique_ptr<YSF::Demodulator> demod;
    std::unique_ptr<YSF::FrameDecoder> decoder;

    // Transmit
    bool transmitting;
    bool pttHeld;       ///< PTT pressed and already acted on
    uint8_t txFn;       ///< Frame number of the next communications frame
    uint32_t txFrames;  ///< Communications frames sent
    uint32_t txSilence; ///< Voice frames sent as silence (none encoded)
    long long txStart;  ///< getTick() at the start of the transmission
    uint8_t txSrc[10];  ///< Own callsign, padded with spaces
    std::unique_ptr<YSF::Modulator> modulator;
};

#endif /* OPMODE_YSF_CS7000_H */
