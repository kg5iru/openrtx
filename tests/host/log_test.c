/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/*
 * Host test of the log core and rtxlink framing. Uses a fake character
 * device that records what the radio would send, with switches for "no host"
 * (-ENOTCONN) and "buffer full" (-EAGAIN). Writes the captured byte stream
 * to the file given as argv[1], for scripts/rtxlink_log.py --input.
 *
 * Build and run: tests/host/run.sh
 */

#include "core/log.h"
#include "core/rtxlink.h"
#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>

// Fake platform hooks ------------------------------------------------------

static uint64_t fakeTime;

void log_portInit()
{
}
uint32_t log_portLock()
{
    return 0;
}
void log_portUnlock(uint32_t key)
{
    (void)key;
}
uint64_t log_portTime()
{
    return fakeTime;
}
uint64_t log_portTimeToUs(uint64_t time)
{
    return time;
}

// Fake character device ----------------------------------------------------

static uint8_t sent[1 << 20];
static size_t sentLen;
static bool connected = false;
static int busyWrites; // next N writes fail with -EAGAIN
static uint8_t inbox[256];
static size_t inboxLen;

static int fakeInit(const struct chardev *d)
{
    (void)d;
    return 0;
}
static int fakeTerm(const struct chardev *d)
{
    (void)d;
    return 0;
}

static ssize_t fakeRead(const struct chardev *d, void *data, const size_t len)
{
    (void)d;
    size_t n = (len < inboxLen) ? len : inboxLen;
    memcpy(data, inbox, n);
    memmove(inbox, inbox + n, inboxLen - n);
    inboxLen -= n;
    return n;
}

static ssize_t fakeWrite(const struct chardev *d, const void *data,
                         const size_t len)
{
    (void)d;
    if (!connected)
        return -ENOTCONN;
    if (busyWrites > 0) {
        busyWrites--;
        return -EAGAIN;
    }
    memcpy(&sent[sentLen], data, len);
    sentLen += len;
    return len;
}

static int fakeIoctl(const struct chardev *d, const int c, const void *a)
{
    (void)d;
    (void)c;
    (void)a;
    return -ENOTSUP;
}

static const struct chardev_api fakeApi = { fakeInit, fakeTerm, fakeRead,
                                            fakeWrite, fakeIoctl };
static const struct chardev fakeDev = { NULL, NULL, &fakeApi };

// Helpers ------------------------------------------------------------------

/** Count complete LOG 'R' frames in the captured stream (by END pairs). */
static unsigned countFrames()
{
    unsigned ends = 0;
    for (size_t i = 0; i < sentLen; i++)
        if (sent[i] == 0xC0)
            ends++;
    return ends / 2;
}

int main(int argc, char **argv)
{
    assert(argc == 2);
    assert(log_init(&fakeDev) == 0);

    // 1. Records logged while no host is connected are kept.
    fakeTime = 1000000; // 1 s
    LOG_I("boot", "OpenRTX %s on %s", "test", "host");
    fakeTime = 1000123;
    LOG_I("boot", "reset flags 0x%08lx:%s%s", 0x0C000000ul, " pin", "");
    log_task();
    assert(sentLen == 0);

    connected = true;
    log_task();
    assert(countFrames() == 2);

    // 2. Formats, negative numbers, widths, eight arguments, chars.
    fakeTime = 2000000;
    LOG_D("fmt", "%d %u %x %X %c %s %5d|%-4d|", -42, 42u, 0xbeef, 0xBEEF, 'z',
          "str", 7, 3);
    LOG_W("fmt", "%d %d %d %d %d %d %d %d", 1, 2, 3, 4, 5, 6, 7, 8);
    LOG_E("fmt", "no arguments");

    // 3. Hex dump with bytes that need SLIP escaping, and copied text.
    static const uint8_t burst[27] = { 0xC0, 0xDB, 0xDC, 0xDD, 0x00, 0xFF, 1,
                                       2,    3,    4,    5,    6,    7,    8,
                                       9,    10,   11,   12,   13,   14,   15,
                                       16,   17,   18,   19,   20,   21 };
    LOG_HEX(LOG_LVL_DEBUG, "dmr", "voice", burst, sizeof(burst));
    char volatileText[48];
    strcpy(volatileText, "temporary buffer, overwritten after the call");
    LOG_TEXT(LOG_LVL_INFO, "txt", volatileText);
    memset(volatileText, 'X', sizeof(volatileText) - 1);

    // 4. A timestamp containing 0xC0 bytes must be escaped too.
    fakeTime = 0xC0C0C0ull;
    LOG_I("time", "timestamp with END bytes");
    fakeTime = 0xC0C0C1ull;

    // 5. Device busy: nothing is lost, it goes out on the next call.
    busyWrites = 1;
    log_task();
    unsigned before = countFrames();
    assert(before == 2); // first write failed, loop stopped
    log_task();
    assert(countFrames() == 8);

    // 6. Level filter.
    log_setLevel(LOG_LVL_WARN);
    LOG_I("lvl", "hidden");
    LOG_D("lvl", "hidden");
    LOG_W("lvl", "shown at warn");
    log_setLevel(LOG_LVL_DEBUG);
    log_task();
    assert(countFrames() == 9);

    // 7. Overflow: 128 slots, the rest dropped and reported once.
    connected = false;
    for (int i = 0; i < 200; i++) {
        fakeTime = 13000000 + i;
        LOG_D("flood", "record %d", i);
    }
    connected = true;
    log_task();
    assert(countFrames() == 9 + 128 + 1);

    // 8. Host request: set level to info. Frame split across two reads to
    //    check the decoder keeps state, with the CRC little-endian as the
    //    upstream host code sends it.
    {
        // proto 0x04, 'L', 3, CRC computed by hand below
        uint8_t body[] = { 0x04, 'L', 3 };
        uint16_t crc = 0x1D0F;
        for (size_t i = 0; i < sizeof(body); i++) {
            crc ^= body[i] << 8;
            for (int b = 0; b < 8; b++)
                crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021) : (crc << 1);
        }
        uint8_t frame[] = { 0xC0, 0x04, 'L', 3, crc & 0xFF, crc >> 8, 0xC0 };
        memcpy(inbox, frame, 3);
        inboxLen = 3;
        log_task();
        assert(log_getLevel() == LOG_LVL_DEBUG);
        memcpy(inbox, frame + 3, sizeof(frame) - 3);
        inboxLen = sizeof(frame) - 3;
        log_task();
        assert(log_getLevel() == LOG_LVL_INFO);
        log_setLevel(LOG_LVL_DEBUG);
    }

    // 9. A corrupted frame is ignored.
    {
        uint8_t bad[] = { 0xC0, 0x04, 'L', 1, 0x12, 0x34, 0xC0 };
        memcpy(inbox, bad, sizeof(bad));
        inboxLen = sizeof(bad);
        log_task();
        assert(log_getLevel() == LOG_LVL_DEBUG);
    }

    // 10. Over-long text is truncated, not overflowed.
    fakeTime = 14000000;
    LOG_I("long", "%s",
          "0123456789012345678901234567890123456789"
          "0123456789012345678901234567890123456789"
          "0123456789012345678901234567890123456789"
          "0123456789012345678901234567890123456789"
          "0123456789012345678901234567890123456789");
    log_task();

    // 11. Payload limit enforced by rtxlink.
    static uint8_t big[RTXLINK_MAX_PAYLOAD + 1];
    assert(rtxlink_send(RTXLINK_FRAME_STDIO, big, sizeof(big)) == -E2BIG);

    FILE *f = fopen(argv[1], "wb");
    assert(f != NULL);
    fwrite(sent, 1, sentLen, f);
    fclose(f);

    printf("log_test: %u frames, %zu bytes, all checks passed\n", countFrames(),
           sentLen);
    return 0;
}
