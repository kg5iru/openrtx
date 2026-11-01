/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/*
 * Deferred system log. Producers (any context) store raw records in a ring
 * with interrupts disabled; log_task() formats them and sends one rtxlink
 * LOG frame per record.
 *
 * LOG frame payloads, radio to host:
 *   'R' level ts[8] tag NUL text      a log record; ts is microseconds since
 *                                     boot, little-endian; text has no NUL
 *   'L' level                         reply to a level request
 * Host to radio:
 *   'L' level                         set the run-time level, replies 'L'
 *   '?'                               get the run-time level, replies 'L'
 */

#include "core/log.h"
#include "core/rtxlink.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

// newlib's integer-only snprintf is already linked by the UI; the full one
// would drag in about 5 kB of floating point formatting. Floats are not
// supported by the log anyway (see log.h).
#ifdef __NEWLIB__
#define log_snprintf sniprintf
#else
#define log_snprintf snprintf
#endif

#define LOG_RECORDS 128 // must be a power of two
#define TAG_MAX 15

enum { KIND_FMT = 0, KIND_HEX = 1, KIND_TEXT = 2 };

struct logRecord {
    uint64_t time;
    const char *tag;
    const char *fmt; // format string, or label for a hex dump
    uint8_t level;
    uint8_t kind;
    uint8_t len; // number of arguments or of bytes
    union {
        uintptr_t args[LOG_MAX_ARGS];
        uint8_t bytes[LOG_MAX_BYTES];
    } u;
};

static struct logRecord ring[LOG_RECORDS];
static volatile uint32_t head; // next slot to write, changed under lock
static volatile uint32_t tail; // next slot to read, changed under lock
static volatile uint32_t lost; // records dropped since the last notice
static volatile uint8_t runLevel = LOG_DEFAULT_LEVEL;

static uint8_t pending[RTXLINK_MAX_PAYLOAD];
static size_t pendingLen;

/**
 * Claim the next free slot and stamp it. Must be called with the lock held.
 * Returns NULL, and counts the record as lost, when the ring is full.
 */
static struct logRecord *claim(uint8_t level, const char *tag, const char *fmt,
                               uint8_t kind)
{
    if ((head - tail) >= LOG_RECORDS) {
        lost = lost + 1;
        return NULL;
    }

    struct logRecord *r = &ring[head & (LOG_RECORDS - 1)];
    r->time = log_portTime();
    r->tag = tag;
    r->fmt = fmt;
    r->level = level;
    r->kind = kind;
    r->len = 0;

    return r;
}

void log_write(uint8_t level, const char *tag, const char *fmt, unsigned nargs,
               ...)
{
    if (level > runLevel)
        return;

    if (nargs > LOG_MAX_ARGS)
        nargs = LOG_MAX_ARGS;

    va_list ap;
    va_start(ap, nargs);

    uint32_t key = log_portLock();
    struct logRecord *r = claim(level, tag, fmt, KIND_FMT);
    if (r != NULL) {
        for (unsigned i = 0; i < nargs; i++)
            r->u.args[i] = va_arg(ap, uintptr_t);

        r->len = nargs;
        head = head + 1;
    }
    log_portUnlock(key);

    va_end(ap);
}

static void writeBytes(uint8_t level, const char *tag, const char *label,
                       uint8_t kind, const void *data, size_t len)
{
    if (level > runLevel)
        return;

    if (len > LOG_MAX_BYTES)
        len = LOG_MAX_BYTES;

    uint32_t key = log_portLock();
    struct logRecord *r = claim(level, tag, label, kind);
    if (r != NULL) {
        memcpy(r->u.bytes, data, len);
        r->len = len;
        head = head + 1;
    }
    log_portUnlock(key);
}

void log_hexdump(uint8_t level, const char *tag, const char *label,
                 const void *data, size_t len)
{
    writeBytes(level, tag, label, KIND_HEX, data, len);
}

void log_text(uint8_t level, const char *tag, const char *text)
{
    writeBytes(level, tag, NULL, KIND_TEXT, text, strlen(text));
}

void log_setLevel(uint8_t level)
{
    if (level > LOG_LVL_DEBUG)
        level = LOG_LVL_DEBUG;

    runLevel = level;
}

uint8_t log_getLevel()
{
    return runLevel;
}

/**
 * Format a record into a LOG frame payload, returns the payload length.
 */
static size_t format(const struct logRecord *r, uint8_t *out, size_t max)
{
    static const char hex[] = "0123456789abcdef";
    uint64_t us = log_portTimeToUs(r->time);
    size_t pos = 0;

    out[pos++] = 'R';
    out[pos++] = r->level;
    for (int i = 0; i < 8; i++)
        out[pos++] = (uint8_t)(us >> (8 * i));

    const char *tag = (r->tag != NULL) ? r->tag : "";
    size_t tagLen = strlen(tag);
    if (tagLen > TAG_MAX)
        tagLen = TAG_MAX;

    memcpy(&out[pos], tag, tagLen);
    pos += tagLen;
    out[pos++] = '\0';

    char *text = (char *)&out[pos];
    size_t room = max - pos; // includes space for snprintf's NUL
    size_t n = 0;

    switch (r->kind) {
        case KIND_FMT: {
            // Unused trailing arguments are ignored by snprintf.
            const uintptr_t *a = r->u.args;
            int ret = log_snprintf(text, room, r->fmt, a[0], a[1], a[2], a[3],
                                   a[4], a[5], a[6], a[7]);
            if (ret > 0)
                n = (size_t)ret;
        } break;

        case KIND_HEX: {
            const char *label = (r->fmt != NULL) ? r->fmt : "";
            while ((*label != '\0') && (n + 1 < room))
                text[n++] = *label++;

            if (n + 1 < room)
                text[n++] = ':';

            for (uint8_t i = 0; (i < r->len) && (n + 3 < room); i++) {
                text[n++] = ' ';
                text[n++] = hex[r->u.bytes[i] >> 4];
                text[n++] = hex[r->u.bytes[i] & 0x0F];
            }
        } break;

        case KIND_TEXT:
            n = r->len;
            if (n >= room)
                n = room - 1;
            memcpy(text, r->u.bytes, n);
            break;
    }

    if (n >= room)
        n = room - 1;

    return pos + n;
}

/**
 * Take the oldest record, or a dropped-records notice when the ring has
 * drained after an overflow. Returns false when there is nothing to send.
 */
static bool nextRecord(struct logRecord *out)
{
    bool found = false;

    uint32_t key = log_portLock();
    if (head != tail) {
        *out = ring[tail & (LOG_RECORDS - 1)];
        tail = tail + 1;
        found = true;
    } else if (lost > 0) {
        out->time = log_portTime();
        out->tag = "log";
        out->fmt = "%u records dropped, buffer full";
        out->level = LOG_LVL_WARN;
        out->kind = KIND_FMT;
        out->len = 1;
        out->u.args[0] = lost;
        lost = 0;
        found = true;
    }
    log_portUnlock(key);

    return found;
}

static size_t logHandler(const uint8_t *data, size_t len, uint8_t *reply,
                         size_t maxReply)
{
    if ((len < 1) || (maxReply < 2))
        return 0;

    if ((data[0] == 'L') && (len >= 2))
        log_setLevel(data[1]);
    else if (data[0] != '?')
        return 0;

    reply[0] = 'L';
    reply[1] = runLevel;
    return 2;
}

int log_init(const struct chardev *dev)
{
    log_portInit();
    rtxlink_setProtocolHandler(RTXLINK_FRAME_LOG, logHandler);
    return rtxlink_init(dev);
}

void log_task()
{
    rtxlink_task();

    // Read the clock at least once per call, so that a free-running counter
    // that wraps is tracked even when nothing is logged for a long time.
    uint32_t key = log_portLock();
    (void)log_portTime();
    log_portUnlock(key);

    while (true) {
        if (pendingLen == 0) {
            struct logRecord r;
            if (nextRecord(&r) == false)
                break;

            pendingLen = format(&r, pending, sizeof(pending));
        }

        int ret = rtxlink_send(RTXLINK_FRAME_LOG, pending, pendingLen);
        if ((ret == -EAGAIN) || (ret == -ENOTCONN))
            break;      // keep it for the next call

        pendingLen = 0; // sent, or impossible to send
    }
}
