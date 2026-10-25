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
    bool enterRx;
    bool audioOn;
    pathId rxAudioPath;
};

#endif /* OPMODE_DMR_CS7000_H */
