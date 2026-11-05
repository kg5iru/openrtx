/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef OPMODE_DMR_CS7000_H
#define OPMODE_DMR_CS7000_H

#include "core/audio_path.h"
#include "rtx/OpMode.hpp"

/**
 * Receive-only DMR operating mode for the CS7000-M17 Plus.  Transmit is not
 * implemented.  Voice is decoded by the AMBE+2 codec extracted from the
 * stock firmware (see platform/drivers/vocoder).
 */
class OpMode_DMR : public OpMode
{
public:
    OpMode_DMR();
    ~OpMode_DMR();

    virtual void enable() override;
    virtual void disable() override;
    virtual void update(rtxStatus_t *const status, const bool newCfg) override;

    virtual opmode getID() override
    {
        return OPMODE_DMR;
    }

    virtual bool rxSquelchOpen() override;

private:
    void stopRxAudio();
    void startTx(rtxStatus_t *const status);
    void finishTx(rtxStatus_t *const status);
    void startWakeup(rtxStatus_t *const status);
    void updateWakeup(rtxStatus_t *const status, const bool ptt);

    bool enterRx;
    bool audioOn;
    pathId rxAudioPath;
    uint8_t idleTicks;   ///< Updates (30 ms) since the call ended
    bool transmitting;
    bool pttHeld;        ///< PTT pressed and already acted on
    uint16_t txTicks;    ///< Updates (30 ms) since the transmission started
    long long txStart;   ///< getTick() at the start of the transmission
    uint16_t stopTick;   ///< txTicks when the stop was requested, 0 = not yet
    pathId micPath;      ///< Microphone path while transmitting
    uint8_t wakePhase;   ///< Repeater wake-up: 0 none, 1 sending, 2 waiting
    uint8_t wakeTries;   ///< Wake-ups sent for this PTT press
    uint8_t wakeTicks;   ///< Updates since the current wake-up phase began
    long long wakeStart; ///< getTick() at the first wake-up
};

#endif /* OPMODE_DMR_CS7000_H */
