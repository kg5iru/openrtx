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
 * DMR operating mode for the CS7000-M17 Plus: voice receive and transmit,
 * direct or through a repeater (woken up when needed), private calls, talker
 * alias and GPS positions (digital APRS).  Voice goes through the AMBE+2
 * codec of the stock firmware (see platform/drivers/vocoder).
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
    void startTx(rtxStatus_t *const status);
    void txStarted(rtxStatus_t *const status, const bool data);
    void finishTx(rtxStatus_t *const status);
    void beginWakeup(rtxStatus_t *const status, const bool forData);
    void startWakeup(rtxStatus_t *const status);
    void updateWakeup(rtxStatus_t *const status, const bool ptt,
                      const bool bsLocked);
    bool startData(rtxStatus_t *const status);

    bool enterRx;
    bool transmitting;
    bool pttHeld;        ///< PTT pressed and already acted on
    uint16_t txTicks;    ///< Updates (30 ms) since the transmission started
    long long txStart;   ///< getTick() at the start of the transmission
    uint16_t stopTick;   ///< txTicks when the stop was requested, 0 = not yet
    uint8_t wakePhase;   ///< Repeater wake-up: 0 none, 1 sending, 2 waiting
    uint8_t wakeTries;   ///< Wake-ups sent for this PTT press
    uint8_t wakeTicks;   ///< Updates since the current wake-up phase began
    long long wakeStart; ///< getTick() at the first wake-up
    uint32_t replyId;    ///< Source of the last private call to this radio
    long long replyTime; ///< getTick() when that call was last received
    bool txIsData;       ///< The transmission is a GPS position
    bool wakeForData;    ///< The wake-up is for a GPS position
    bool aprsPending;    ///< A GPS position is to be sent when idle
    long long aprsLast;  ///< getTick() of the last beacon interval start
    uint16_t aprsIpId;   ///< IPv4 identification of the next position
};

#endif /* OPMODE_DMR_CS7000_H */
