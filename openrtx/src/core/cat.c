/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "core/cat.h"
#include <string.h>

#define CMD_MAX 32

static const struct chardev *catDev;
static const struct catRadio *catRadio;
static char line[CMD_MAX];
static size_t lineLen;
static bool overflow;
static bool pttRequested;
static long long pttSince;

unsigned cat_smeter(rssi_t rssi)
{
    /*
     * V/UHF S-meter: S9 = -93 dBm, 6 dB per S unit, S0 at -147 dBm.  The
     * TS-2000 scale gives 15 steps to S0..S9 and 15 to S9..S9+60 dB.
     */
    if (rssi <= -147)
        return 0;

    if (rssi <= -93)
        return (unsigned)(((rssi + 147) * 15) / 54);

    unsigned v = 15 + (unsigned)((rssi + 93) / 4);
    return (v > 30) ? 30 : v;
}

// Up to 11 digits; false if none or anything else
static bool parseFreq(const char *s, size_t len, freq_t *out)
{
    if ((len == 0) || (len > 11))
        return false;

    uint64_t v = 0;
    for (size_t i = 0; i < len; i++) {
        if ((s[i] < '0') || (s[i] > '9'))
            return false;
        v = (v * 10) + (uint64_t)(s[i] - '0');
    }

    if (v > 0xFFFFFFFFu)
        return false;

    *out = (freq_t)v;
    return true;
}

// Decimal digits, zero padded to width, at p; returns the end
static char *putDigits(char *p, uint32_t value, int width)
{
    for (int i = width - 1; i >= 0; i--) {
        p[i] = (char)('0' + (value % 10));
        value /= 10;
    }
    return p + width;
}

static size_t answer(char *reply, size_t max, const char *text)
{
    size_t n = strlen(text);
    if (n > max)
        return 0;
    memcpy(reply, text, n);
    return n;
}

size_t cat_execute(const struct catRadio *radio, const char *cmd, size_t len,
                   char *reply, size_t max)
{
    char buf[48];

    if (len < 2)
        return answer(reply, max, "?;");

    char c0 = cmd[0], c1 = cmd[1];
    if ((c0 >= 'a') && (c0 <= 'z'))
        c0 -= 'a' - 'A';
    if ((c1 >= 'a') && (c1 <= 'z'))
        c1 -= 'a' - 'A';

    const char *arg = &cmd[2];
    size_t argLen = len - 2;
    uint16_t op = ((uint16_t)c0 << 8) | (uint8_t)c1;

    switch (op) {
        case ('I' << 8) | 'D':
            return answer(reply, max, "ID019;");

        case ('F' << 8) | 'V':
            return answer(reply, max, "FV1.00;");

        case ('A' << 8) | 'I':
            if (argLen == 0)
                return answer(reply, max, "AI0;");
            return 0; // auto information stays off

        case ('P' << 8) | 'S':
            if (argLen == 0)
                return answer(reply, max, "PS1;");
            return 0;

        case ('F' << 8) | 'A':
        case ('F' << 8) | 'B': {
            bool rx = (c1 == 'A');
            if (argLen == 0) {
                freq_t f = rx ? radio->rxFrequency() : radio->txFrequency();
                char *p = buf;
                *p++ = 'F';
                *p++ = c1;
                p = putDigits(p, f, 11);
                *p++ = ';';
                *p = '\0';
                return answer(reply, max, buf);
            }

            freq_t f;
            if (!parseFreq(arg, argLen, &f))
                return answer(reply, max, "?;");
            bool ok = rx ? radio->setRxFrequency(f) : radio->setTxFrequency(f);
            return ok ? 0 : answer(reply, max, "?;");
        }

        case ('F' << 8) | 'R':
        case ('F' << 8) | 'T':
            if (argLen == 0) {
                const char fr[] = { 'F', c1, '0', ';', '\0' };
                return answer(reply, max, fr);
            }
            return 0; // VFO A only

        case ('M' << 8) | 'D':
            if (argLen == 0)
                return answer(reply, max, "MD4;");
            return 0; // the mode is set on the radio

        case ('I' << 8) | 'F': {
            freq_t rxf = radio->rxFrequency();
            freq_t txf = radio->txFrequency();
            char shift = '0';
            if (txf > rxf)
                shift = '1';
            else if (txf < rxf)
                shift = '2';
            // IF, frequency, step (5 spaces), RIT offset, RIT, XIT, memory
            // bank and channel, TX, mode (FM), VFO, scan, split, tone,
            // tone number, shift
            char *p = buf;
            *p++ = 'I';
            *p++ = 'F';
            p = putDigits(p, rxf, 11);
            memcpy(p, "     +000000000", 15);
            p += 15;
            *p++ = radio->transmitting() ? '1' : '0';
            memcpy(p, "4000000", 7);
            p += 7;
            *p++ = shift;
            *p++ = ';';
            *p = '\0';
            return answer(reply, max, buf);
        }

        case ('T' << 8) | 'X':
            radio->setPtt(true);
            return 0;

        case ('R' << 8) | 'X':
            radio->setPtt(false);
            return 0;

        case ('S' << 8) | 'M': {
            char *p = buf;
            *p++ = 'S';
            *p++ = 'M';
            *p++ = (argLen > 0) ? arg[0] : '0';
            p = putDigits(p, cat_smeter(radio->rssi()), 4);
            *p++ = ';';
            *p = '\0';
            return answer(reply, max, buf);
        }

        case ('B' << 8) | 'Y':
            return answer(reply, max, radio->busy() ? "BY10;" : "BY00;");

        default:
            break;
    }

    return answer(reply, max, "?;");
}

int cat_init(const struct chardev *dev, const struct catRadio *radio)
{
    catDev = dev;
    catRadio = radio;
    lineLen = 0;
    overflow = false;
    pttRequested = false;
    return chardev_init(dev);
}

// Wrapper that tracks the transmit request, for the time-out
static const struct catRadio *realRadio;
static long long taskNow;

static void trackedSetPtt(bool on)
{
    if (on && !pttRequested)
        pttSince = taskNow;
    pttRequested = on;
    realRadio->setPtt(on);
}

static void rxChar(const struct catRadio *radio, char c)
{
    if (c == ';') {
        if (!overflow && (lineLen > 0)) {
            char reply[48];
            size_t r = cat_execute(radio, line, lineLen, reply, sizeof(reply));
            if (r > 0)
                catDev->api->write(catDev, reply, r);
        }
        lineLen = 0;
        overflow = false;
        return;
    }

    // Line ends and spaces some programs add between commands
    if ((c == '\r') || (c == '\n') || ((c == ' ') && (lineLen == 0)))
        return;

    if (lineLen < sizeof(line))
        line[lineLen++] = c;
    else
        overflow = true;
}

void cat_task(long long nowMs)
{
    if (catDev == NULL)
        return;

    taskNow = nowMs;
    realRadio = catRadio;
    struct catRadio radio = *catRadio;
    radio.setPtt = trackedSetPtt;

    // Everything waiting, in chunks (bounded, to keep the call short)
    uint8_t in[64];
    for (int chunk = 0; chunk < 8; chunk++) {
        ssize_t n = catDev->api->read(catDev, in, sizeof(in));
        for (ssize_t i = 0; i < n; i++)
            rxChar(&radio, (char)in[i]);
        if (n < (ssize_t)sizeof(in))
            break;
    }

    if (pttRequested && ((nowMs - pttSince) > (CAT_TX_TIMEOUT_S * 1000LL))) {
        pttRequested = false;
        catRadio->setPtt(false);
    }
}
