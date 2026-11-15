/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef BT_PROTO_CS7000_H
#define BT_PROTO_CS7000_H

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Line protocol of the CS7000-M17 Plus Bluetooth module, as the stock
 * firmware (HOST V9.00.96) speaks it.  No hardware access: the driver feeds
 * the lines and sends the commands.
 *
 * Commands (stock fn 0x0807C184): 30 NUL bytes, the command, its argument,
 * CR.  Known: "AT*I" (status / paired device query), "ATO" (sent once at
 * power-up), "AT*C<addr>" (connect), "AT*A" (open the audio link), "AT*D",
 * "AT*P", "AT*U", "AT*LM", "AT*LS", "AT*V", "AT*W", "AT*K...".
 *
 * Status lines (stock fn 0x0807B846 and the USART1 receive callback):
 * - "I*IF"            -> answer with AT*I
 * - "I*PD<addr>..."   -> a paired device: connect to it (AT*C<addr>)
 * - "I*CHC0<addr>"    -> headset connected: open the audio (AT*A)
 * - "I*CHD0"          -> headset disconnected
 * - "I*CHD1"          -> link attempt failed: AT*I again after 500 ms
 * - "I*AHC0"/"I*AHD0" -> audio link open / closed
 * - push-to-talk from accessories, press / release:
 *   "I*BH01" / "I*BH0x", "C:BEGIN*" / "C:END*", "AT+CTKST=1" / "=0",
 *   "+PTT=P" / "+PTT=R", "#p" / "#r"; "I*E0" toggles.
 */

#define BT_ADDR_LEN 12 // characters, normally hex digits

enum btAction {
    BT_ACT_NONE = 0,
    BT_ACT_INFO,       // send AT*I now
    BT_ACT_INFO_LATER, // send AT*I in 500 ms
    BT_ACT_CONNECT,    // send AT*C<addr>, addr in the state
    BT_ACT_AUDIO       // send AT*A
};

struct btLink {
    bool connected;
    bool audio;
    bool ptt;
    char addr[BT_ADDR_LEN + 1]; // device to connect to, or connected
};

/** Reset the link state (module powered up or down). */
void btProto_reset(struct btLink *link);

/**
 * Handle one status line from the module, without its CR.
 *
 * @return the action the stock firmware takes in answer.
 */
enum btAction btProto_line(struct btLink *link, const char *line, size_t len);

/**
 * Build a command as the stock firmware sends it: 30 NUL bytes, command,
 * argument (may be NULL), CR.
 *
 * @return the number of bytes written, 0 if it does not fit.
 */
size_t btProto_command(char *out, size_t max, const char *cmd, const char *arg);

#ifdef __cplusplus
}
#endif

#endif /* BT_PROTO_CS7000_H */
