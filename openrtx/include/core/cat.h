/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef CAT_H
#define CAT_H

#include "interfaces/chardev.h"
#include "core/datatypes.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * CAT (computer aided transceiver) control over a serial port, answering as
 * a Kenwood TS-2000 does, the most widely supported CAT dialect: Hamlib
 * (rigctl model 2014, rigctld), flrig, WSJT-X, fldigi, Direwolf through
 * rigctld, VARA through its own Kenwood option.
 *
 * Commands are ASCII, ended by ';'.  Supported:
 *
 *   ID       identification: ID019 (TS-2000)
 *   FA, FB   VFO A (receive) and B (transmit) frequency, 11 digits in Hz;
 *            setting FA keeps the repeater shift, FB sets the transmit
 *            frequency alone
 *   FR, FT   receive and transmit VFO: always A (0)
 *   IF       status: frequency, transmitting, mode, split
 *   MD       mode: 4 (FM) for every mode of the radio; setting it is
 *            accepted and ignored (the mode is chosen on the radio)
 *   TX, RX   transmit, receive
 *   SM       S-meter, 0000..0030 (S9 = 0015, S9+60 dB = 0030)
 *   BY       busy: squelch open
 *   AI, PS   auto information (off), power (on): set commands accepted
 *   FV       firmware version
 *
 * Anything else is answered "?;", as the radio does for a command it does
 * not know.  A transmit request lasts at most CAT_TX_TIMEOUT_S seconds.
 */

#define CAT_TX_TIMEOUT_S 300

/** What the CAT interpreter needs from the radio. */
struct catRadio {
    freq_t (*rxFrequency)(void);
    freq_t (*txFrequency)(void);
    bool (*setRxFrequency)(freq_t freq); ///< false if out of the radio's band
    bool (*setTxFrequency)(freq_t freq);
    bool (*transmitting)(void);
    void (*setPtt)(bool on); ///< transmit request from the computer
    rssi_t (*rssi)(void);
    bool (*busy)(void);      ///< squelch open
};

/**
 * Execute one command.
 *
 * @param radio: the radio.
 * @param cmd: the command, without its ';'.
 * @param len: its length.
 * @param reply: where to write the answer, with its ';' (nothing for a set
 * command), not 0-terminated.
 * @param max: space in reply (40 bytes are always enough).
 * @return the length of the answer.
 */
size_t cat_execute(const struct catRadio *radio, const char *cmd, size_t len,
                   char *reply, size_t max);

/** Kenwood S-meter reading, 0..30, of a signal level. */
unsigned cat_smeter(rssi_t rssi);

/**
 * Start CAT control on a serial port.
 *
 * @return the result of the port's init().
 */
int cat_init(const struct chardev *dev, const struct catRadio *radio);

/**
 * Read and execute the commands received, send the answers, and end a
 * transmit request that ran too long.  To be called every few milliseconds.
 *
 * @param nowMs: current time in milliseconds.
 */
void cat_task(long long nowMs);

/** The radio's side of CAT: struct catRadio on the radio state (core). */
extern const struct catRadio cat_radio;

/**
 * Transmit request from the computer, implemented by the platform; it ends
 * up in platform_getPttStatus().
 */
void cat_portSetPtt(bool on);

#ifdef __cplusplus
}
#endif

#endif /* CAT_H */
