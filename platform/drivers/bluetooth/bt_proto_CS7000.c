/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "drivers/bluetooth/bt_proto_CS7000.h"
#include <string.h>

#define WAKE_NULS 30

static bool startsWith(const char *line, size_t len, const char *prefix)
{
    size_t n = strlen(prefix);
    return (len >= n) && (memcmp(line, prefix, n) == 0);
}

static bool isPrintable(char c)
{
    return (c > ' ') && (c < 0x7F);
}

/*
 * Copy the BT_ADDR_LEN characters from line[from], if there.  The stock
 * firmware copies them as they are (normally hex digits) and passes them back
 * in AT*C.
 */
static bool takeAddr(struct btLink *link, const char *line, size_t len,
                     size_t from)
{
    if (len < (from + BT_ADDR_LEN))
        return false;

    for (size_t i = 0; i < BT_ADDR_LEN; i++) {
        if (!isPrintable(line[from + i]))
            return false;
    }

    memcpy(link->addr, &line[from], BT_ADDR_LEN);
    link->addr[BT_ADDR_LEN] = '\0';
    return true;
}

void btProto_reset(struct btLink *link)
{
    memset(link, 0, sizeof(*link));
}

enum btAction btProto_line(struct btLink *link, const char *line, size_t len)
{
    // Push-to-talk, in every form the stock firmware accepts
    if (startsWith(line, len, "I*BH0") && (len >= 6)) {
        link->ptt = (line[5] == '1');
        return BT_ACT_NONE;
    }

    if (startsWith(line, len, "I*E0")) {
        link->ptt = !link->ptt;
        return BT_ACT_NONE;
    }

    if (startsWith(line, len, "C:BEGIN*") || startsWith(line, len, "+PTT=P")
        || startsWith(line, len, "AT+CTKST=1") || startsWith(line, len, "#p")) {
        link->ptt = true;
        return BT_ACT_NONE;
    }

    if (startsWith(line, len, "C:END*") || startsWith(line, len, "+PTT=R")
        || startsWith(line, len, "AT+CTKST=0") || startsWith(line, len, "#r")) {
        link->ptt = false;
        return BT_ACT_NONE;
    }

    if (startsWith(line, len, "I*IF"))
        return BT_ACT_INFO;

    // A paired device: connect to it unless already connected
    if (startsWith(line, len, "I*PD")) {
        if (link->connected)
            return BT_ACT_NONE;
        if (!takeAddr(link, line, len, 4))
            return BT_ACT_NONE;
        return BT_ACT_CONNECT;
    }

    if (startsWith(line, len, "I*CHC0")) {
        link->connected = true;
        takeAddr(link, line, len, 6);
        return BT_ACT_AUDIO;
    }

    if (startsWith(line, len, "I*CHD0")) {
        link->connected = false;
        link->audio = false;
        link->ptt = false;
        return BT_ACT_NONE;
    }

    if (startsWith(line, len, "I*CHD1"))
        return BT_ACT_INFO_LATER;

    if (startsWith(line, len, "I*AHC0")) {
        link->audio = true;
        return BT_ACT_NONE;
    }

    if (startsWith(line, len, "I*AHD0")) {
        link->audio = false;
        return BT_ACT_NONE;
    }

    return BT_ACT_NONE;
}

size_t btProto_command(char *out, size_t max, const char *cmd, const char *arg)
{
    size_t cmdLen = strlen(cmd);
    size_t argLen = (arg != NULL) ? strlen(arg) : 0;
    size_t total = WAKE_NULS + cmdLen + argLen + 1;

    if (total > max)
        return 0;

    memset(out, 0, WAKE_NULS);
    memcpy(&out[WAKE_NULS], cmd, cmdLen);
    if (argLen > 0)
        memcpy(&out[WAKE_NULS + cmdLen], arg, argLen);
    out[total - 1] = '\r';

    return total;
}
