/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/*
 * Small rtxlink core: SLIP framing, CRC check and protocol dispatch. Wire
 * compatible with the upstream rtxlink branch (see rtxlink.h), written anew
 * to keep the escape state across reads and to never write past the frame
 * buffers.
 */

#include "core/rtxlink.h"
#include <string.h>
#include <errno.h>

#define SLIP_END 0xC0
#define SLIP_ESC 0xDB
#define SLIP_ESC_END 0xDC
#define SLIP_ESC_ESC 0xDD

// Protocol id + payload + CRC
#define FRAME_MAX (1 + RTXLINK_MAX_PAYLOAD + 2)

static const struct chardev *cDev = NULL;
static protocolHandler handlers[RTXLINK_NUM_PROTOCOLS];

static uint8_t rxFrame[FRAME_MAX];
static size_t rxLen;
static bool rxEsc;
static bool rxOverflow;

// Worst case: every byte escaped, plus the two END markers
static uint8_t txBuf[2 * FRAME_MAX + 2];

#define CRC_INIT 0x1D0F

static uint16_t crc16(uint16_t crc, const uint8_t *data, size_t len)
{
    while (len--) {
        crc ^= ((uint16_t)*data++) << 8;
        for (int i = 0; i < 8; i++)
            crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021) : (crc << 1);
    }

    return crc;
}

static size_t slipPut(size_t pos, const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        uint8_t b = data[i];
        if (b == SLIP_END) {
            txBuf[pos++] = SLIP_ESC;
            txBuf[pos++] = SLIP_ESC_END;
        } else if (b == SLIP_ESC) {
            txBuf[pos++] = SLIP_ESC;
            txBuf[pos++] = SLIP_ESC_ESC;
        } else {
            txBuf[pos++] = b;
        }
    }

    return pos;
}

static void dispatch()
{
    if (rxLen < 3)
        return;

    size_t len = rxLen - 2;
    uint16_t crc = crc16(CRC_INIT, rxFrame, len);
    uint8_t hi = rxFrame[len];
    uint8_t lo = rxFrame[len + 1];
    bool isBE = (hi == (crc >> 8)) && (lo == (crc & 0xFF));
    bool isLE = (lo == (crc >> 8)) && (hi == (crc & 0xFF));
    if (!isBE && !isLE)
        return;

    uint8_t proto = rxFrame[0];
    if ((proto >= RTXLINK_NUM_PROTOCOLS) || (handlers[proto] == NULL))
        return;

    // A reply that does not fit in the device buffer right now is dropped:
    // the host is expected to retry its request.
    static uint8_t reply[RTXLINK_MAX_PAYLOAD];
    size_t rLen = handlers[proto](&rxFrame[1], len - 1, reply, sizeof(reply));
    if (rLen > 0)
        rtxlink_send((enum ProtocolID)proto, reply, rLen);
}

static void rxByte(uint8_t b)
{
    if (b == SLIP_END) {
        if ((rxLen > 0) && (rxOverflow == false))
            dispatch();

        rxLen = 0;
        rxEsc = false;
        rxOverflow = false;
        return;
    }

    if (b == SLIP_ESC) {
        rxEsc = true;
        return;
    }

    if (rxEsc) {
        rxEsc = false;
        if (b == SLIP_ESC_END)
            b = SLIP_END;
        else if (b == SLIP_ESC_ESC)
            b = SLIP_ESC;
    }

    if (rxLen < sizeof(rxFrame))
        rxFrame[rxLen++] = b;
    else
        rxOverflow = true;
}

int rtxlink_init(const struct chardev *dev)
{
    if (dev == NULL)
        return -EINVAL;

    cDev = dev;
    rxLen = 0;
    rxEsc = false;
    rxOverflow = false;
    return chardev_init(cDev);
}

void rtxlink_task()
{
    if (cDev == NULL)
        return;

    uint8_t buf[64];
    ssize_t n;

    while ((n = chardev_read(cDev, buf, sizeof(buf))) > 0) {
        for (ssize_t i = 0; i < n; i++)
            rxByte(buf[i]);
    }
}

void rtxlink_terminate()
{
    if (cDev == NULL)
        return;

    chardev_terminate(cDev);
    cDev = NULL;
}

int rtxlink_send(const enum ProtocolID proto, const void *data,
                 const size_t len)
{
    if (cDev == NULL)
        return -ENODEV;

    if (len > RTXLINK_MAX_PAYLOAD)
        return -E2BIG;

    if ((data == NULL) && (len > 0))
        return -EINVAL;

    // CRC over protocol id and payload, computed without copying
    const uint8_t *p = (const uint8_t *)data;
    uint8_t id = (uint8_t)proto;
    uint16_t crc = crc16(CRC_INIT, &id, 1);
    crc = crc16(crc, p, len);

    uint8_t tail[2] = { (uint8_t)(crc >> 8), (uint8_t)(crc & 0xFF) };

    size_t pos = 0;
    txBuf[pos++] = SLIP_END;
    pos = slipPut(pos, &id, 1);
    pos = slipPut(pos, p, len);
    pos = slipPut(pos, tail, 2);
    txBuf[pos++] = SLIP_END;

    ssize_t ret = chardev_write(cDev, txBuf, pos);
    if (ret < 0)
        return (int)ret;

    if ((size_t)ret != pos)
        return -EAGAIN;

    return 0;
}

bool rtxlink_setProtocolHandler(const enum ProtocolID proto,
                                protocolHandler handler)
{
    if (proto >= RTXLINK_NUM_PROTOCOLS)
        return false;

    if (handlers[proto] != NULL)
        return false;

    handlers[proto] = handler;
    return true;
}

void rtxlink_removeProtocolHandler(const enum ProtocolID proto)
{
    if (proto >= RTXLINK_NUM_PROTOCOLS)
        return;

    handlers[proto] = NULL;
}
