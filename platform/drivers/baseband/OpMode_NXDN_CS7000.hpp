/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef OPMODE_NXDN_CS7000_H
#define OPMODE_NXDN_CS7000_H

#include <memory>
#include "core/audio_path.h"
#include "core/audio_stream.h"
#include "rtx/OpMode.hpp"
#include "protocols/NXDN/Demodulator.hpp"
#include "protocols/NXDN/Frame.hpp"
#include "protocols/NXDN/Modulator.hpp"

/**
 * NXDN48 operating mode for the CS7000-M17 Plus: group voice calls, received
 * and sent, as MMDVMHost handles them.  The 4FSK baseband goes through the
 * same paths M17 and YSF use, (de)modulated in software (protocols/NXDN).
 * Voice is AMBE+2 in the DMR layout, through the codec of the stock
 * firmware, with the queues and threads the DMR mode uses (dmr_audio_CS7000).
 */
class OpMode_NXDN : public OpMode
{
public:
    OpMode_NXDN();
    ~OpMode_NXDN();

    virtual void enable() override;
    virtual void disable() override;
    virtual void update(rtxStatus_t *const status, const bool newCfg) override;

    virtual opmode getID() override
    {
        return OPMODE_NXDN;
    }

    virtual bool rxSquelchOpen() override;

private:
    void startRx();
    void stopRx();
    void processFrame(const NXDN::frame_t &rxFrame);
    void processControl(const NXDN::frame_t &frame);
    void processVoice(const NXDN::frame_t &frame, const uint8_t lich);
    void pushVoice(const NXDN::frame_t &frame, const uint8_t option);
    void startCall(const char *how);
    void endCall(const char *why);
    void startTx(rtxStatus_t *const status);
    void sendVoiceFrame();
    void endTx(rtxStatus_t *const status);

    bool enterRx;
    bool rxOn;
    pathId basebandPath;
    streamId basebandId;
    uint8_t ran;             ///< RAN set: TX, and RX filter unless 0
    uint8_t framesSinceGood; ///< Frames since the last valid LICH
    uint8_t unlockedTicks;   ///< Updates without a lock during a call
    uint8_t flywheel;        ///< Frames in a row played on the last LICH
    uint8_t lastOption;      ///< Steal option of the last voice frame
    uint8_t goodVoice;       ///< Voice frames in a row with a valid SACCH
    uint8_t partsSeen;       ///< Superframe parts of the VCALL received
    NXDN::layer3_t rxCall;   ///< VCALL rebuilt from the SACCH

    std::unique_ptr<int16_t[]> basebandBuf;
    std::unique_ptr<NXDN::Demodulator> demod;

    // Transmit
    bool transmitting;
    bool pttHeld;       ///< PTT pressed and already acted on
    uint32_t txFrames;  ///< Voice frames sent
    uint32_t txSilence; ///< AMBE frames sent as silence (none encoded)
    long long txStart;  ///< getTick() at the start of the transmission
    NXDN::layer3_t txCall;
    std::unique_ptr<NXDN::Modulator> modulator;
};

#endif /* OPMODE_NXDN_CS7000_H */
