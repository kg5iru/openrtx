/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "core/cat.h"
#include "core/event.h"
#include "core/state.h"
#include "interfaces/platform.h"
#include "rtx/rtx.h"
#include "core/ui.h"

/*
 * The radio's side of CAT.  Frequencies are changed in the radio state under
 * its mutex, then the user interface is told (EVENT_CAT), which applies them
 * to the RTX and leaves a memory channel for the VFO, as a frequency typed on
 * the keypad would.
 */

static bool inBand(freq_t freq)
{
    const hwInfo_t *hw = platform_getHwInfo();

    // hwInfo_t frequencies are in MHz
    if (hw->vhf_band && (freq >= (hw->vhf_minFreq * 1000000u))
        && (freq <= (hw->vhf_maxFreq * 1000000u)))
        return true;

    if (hw->uhf_band && (freq >= (hw->uhf_minFreq * 1000000u))
        && (freq <= (hw->uhf_maxFreq * 1000000u)))
        return true;

    return false;
}

static freq_t rxFrequency(void)
{
    pthread_mutex_lock(&state_mutex);
    freq_t f = state.channel.rx_frequency;
    pthread_mutex_unlock(&state_mutex);
    return f;
}

static freq_t txFrequency(void)
{
    pthread_mutex_lock(&state_mutex);
    freq_t f = state.channel.tx_frequency;
    pthread_mutex_unlock(&state_mutex);
    return f;
}

static bool setRxFrequency(freq_t freq)
{
    if (!inBand(freq))
        return false;

    pthread_mutex_lock(&state_mutex);
    // Keep the repeater shift, if the transmit frequency stays in band
    int64_t shift = (int64_t)state.channel.tx_frequency
                  - (int64_t)state.channel.rx_frequency;
    int64_t tx = (int64_t)freq + shift;
    state.channel.rx_frequency = freq;
    state.channel.tx_frequency = ((tx > 0) && inBand((freq_t)tx)) ? (freq_t)tx :
                                                                    freq;
    ui_pushEvent(EVENT_CAT, 0);
    pthread_mutex_unlock(&state_mutex);
    return true;
}

static bool setTxFrequency(freq_t freq)
{
    if (!inBand(freq))
        return false;

    pthread_mutex_lock(&state_mutex);
    state.channel.tx_frequency = freq;
    ui_pushEvent(EVENT_CAT, 0);
    pthread_mutex_unlock(&state_mutex);
    return true;
}

static bool transmitting(void)
{
    return rtx_getCurrentStatus().opStatus == TX;
}

static rssi_t rssi(void)
{
    return rtx_getRssi();
}

static bool busy(void)
{
    return rtx_rxSquelchOpen();
}

const struct catRadio cat_radio = { rxFrequency,    txFrequency,
                                    setRxFrequency, setTxFrequency,
                                    transmitting,   cat_portSetPtt,
                                    rssi,           busy };
