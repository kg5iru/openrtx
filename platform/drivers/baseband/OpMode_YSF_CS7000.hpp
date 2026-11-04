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

/**
 * Receive-only Yaesu System Fusion operating mode for the CS7000-M17 Plus.
 * The C4FM baseband is demodulated in software (protocols/YSF) from the same
 * receive path M17 uses; voice (V/D modes 1 and 2, AMBE+2 half rate) is
 * decoded by the codec of the stock firmware, through the queue and thread
 * the DMR mode uses (dmr_audio_CS7000).  Voice full rate (VW, IMBE) is not
 * decoded.  Transmit is not implemented.
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
    void endCall(const char *why);

    bool enterRx;
    bool rxOn;
    bool audioOn;
    pathId basebandPath;
    streamId basebandId;
    pathId speakerPath;
    uint16_t framesSinceGood; ///< Frames since the last valid FICH

    std::unique_ptr<int16_t[]> basebandBuf;
    std::unique_ptr<YSF::Demodulator> demod;
    std::unique_ptr<YSF::FrameDecoder> decoder;
};

#endif /* OPMODE_YSF_CS7000_H */
