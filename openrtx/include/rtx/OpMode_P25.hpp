/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef OPMODE_P25_H
#define OPMODE_P25_H

#include "core/audio_path.h"
#include "protocols/P25/P25Data.h"
#include "protocols/P25/P25Audio.h"
#include "protocols/P25/P25LowSpeedData.h"
#include "protocols/P25/modem/Modulator.hpp"
#include "protocols/P25/modem/Demodulator.hpp"
#include "OpMode.hpp"

/**
 * Specialisation of the OpMode class for the management of P25 Phase 1
 * (C4FM) conventional voice.
 */
class OpMode_P25 : public OpMode
{
public:
    /**
     * Constructor.
     */
    OpMode_P25();

    /**
     * Destructor.
     */
    ~OpMode_P25();

    /**
     * Enable the operating mode.
     *
     * Application must ensure this function is being called when entering the
     * new operating mode and always before the first call of "update".
     */
    virtual void enable() override;

    /**
     * Disable the operating mode. This function stops the DMA transfers
     * corresponding to the baseband signal and shuts down the IMBE codec.
     *
     * Application must ensure this function is being called when exiting the
     * current operating mode.
     */
    virtual void disable() override;

    /**
     * Update the internal FSM.
     * Application code has to call this function periodically, to ensure proper
     * functionality.
     *
     * @param status: pointer to the rtxStatus_t structure containing the
     * current RTX status. Internal FSM may change the current value of the
     * opStatus flag.
     * @param newCfg: flag used inform the internal FSM that a new RTX
     * configuration has been applied.
     */
    virtual void update(rtxStatus_t *const status, const bool newCfg) override;

    /**
     * Get the mode identifier.
     *
     * @return the constant OPMODE_P25.
     */
    virtual opmode getID() override
    {
        return OPMODE_P25;
    }

    /**
     * Check if RX squelch is open.
     *
     * @return true if the demodulator is locked on a data unit carrying our
     * NAC.
     */
    virtual bool rxSquelchOpen() override
    {
        return dataValid;
    }

private:
    /**
     * Function handling the OFF operating state.
     *
     * @param status: pointer to the rtxStatus_t structure containing the
     * current RTX status.
     */
    void offState(rtxStatus_t *const status);

    /**
     * Function handling the RX operating state.
     *
     * @param status: pointer to the rtxStatus_t structure containing the
     * current RTX status.
     */
    void rxState(rtxStatus_t *const status);

    /**
     * Function handling the TX operating state.
     *
     * @param status: pointer to the rtxStatus_t structure containing the
     * current RTX status.
     */
    void txState(rtxStatus_t *const status);

    /**
     * Handle a received data unit.
     *
     * @param status: pointer to the rtxStatus_t structure containing the
     * current RTX status.
     */
    void handleUnit(rtxStatus_t *const status);

    /**
     * Close the RX audio path and stop the codec, if open.
     */
    void stopRxAudio();

    /**
     * Decide whether the call described by the received link control is
     * for us, according to the destination ID check setting.
     *
     * @param status: pointer to the rtxStatus_t structure containing the
     * current RTX status.
     * @return true if the call's audio is to be played.
     */
    bool callAccepted(const rtxStatus_t *const status);

    /**
     * Build and send one voice unit (LDU1 or LDU2) from nine IMBE frames
     * taken from the codec.
     *
     * @param ldu2: build an LDU2, else an LDU1.
     */
    void sendVoiceUnit(const bool ldu2);

    /*
     * Transmit level: the TX level setting is kept in the same units as the
     * previous P25 implementation, where level 180 produced a 17300 counts
     * outer symbol level (0.93x spec deviation on the CS7000P LINE_IN /
     * HR_C6000 path).
     */
    static constexpr float LEVEL_COUNTS_PER_STEP = 96.0f;

    /// Receive any NAC when the NAC setting is this value.
    static constexpr uint16_t NAC_ANY = 0xF7E;

    /// Talkgroup addressed to every unit.
    static constexpr uint32_t TGID_ALL_CALL = 0xFFFF;

    bool startRx;             ///< Flag for RX management.
    bool startTx;             ///< Flag for TX management.
    bool locked;              ///< Flag for modem lock.
    bool dataValid;           ///< A voice unit with our NAC is being received.
    bool invertTxPhase;       ///< TX signal phase inversion setting.
    bool invertRxPhase;       ///< RX signal phase inversion setting.
    bool ldu2Next;            ///< Next voice unit to send is an LDU2.
    bool callForUs;           ///< The call being received passed the ID check.
    uint16_t txNac;           ///< NAC of the transmission in progress.
    pathId rxAudioPath;       ///< Audio path ID for RX.
    pathId txAudioPath;       ///< Audio path ID for TX.

    P25::Modulator modulator; ///< C4FM modulator.
    P25::Demodulator demodulator; ///< C4FM demodulator.
    CP25Data rxData;              ///< Link control of the received call.
    CP25Data txData;              ///< Link control of our call.
    CP25Audio audio;              ///< IMBE frame placement and FEC.
    CP25LowSpeedData lsd;         ///< Low speed data of our call.
    uint8_t txUnit[216];          ///< Unit under construction.
};

#endif /* OPMODE_P25_H */
