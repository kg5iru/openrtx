/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef RTXLINK_H
#define RTXLINK_H

#include "interfaces/chardev.h"
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * rtxlink: framed, multiplexed link between the radio and a host.
 *
 * Wire format, compatible with the upstream OpenRTX rtxlink branch:
 *
 *     END | SLIP( protocol id | payload | CRC16 hi | CRC16 lo ) | END
 *
 * - SLIP as in RFC 1055: END = 0xC0, ESC = 0xDB, ESC_END = 0xDC,
 *   ESC_ESC = 0xDD. Every frame starts and ends with END.
 * - CRC16: CCITT polynomial 0x1021, initial value 0x1D0F, no reflection, no
 *   final XOR, over the protocol id and the payload. Sent big-endian. On
 *   receive both byte orders are accepted (upstream decodes little-endian).
 *
 * Only one thread may call rtxlink_task() and rtxlink_send(): on the radio
 * that is the log thread.
 */

/**
 * Identifiers for each sub-protocol of rtxlink. 0x00..0x03 are the upstream
 * ones, LOG is new.
 */
enum ProtocolID {
    RTXLINK_FRAME_STDIO = 0x00,
    RTXLINK_FRAME_CAT = 0x01,
    RTXLINK_FRAME_FMP = 0x02,
    RTXLINK_FRAME_DAT = 0x03,
    RTXLINK_FRAME_LOG = 0x04,

    RTXLINK_NUM_PROTOCOLS
};

/** Largest payload carried by one frame, excluding protocol id and CRC. */
#define RTXLINK_MAX_PAYLOAD 192

/**
 * Protocol handler. Gets the payload of a received frame (protocol id and
 * CRC removed) and may write a reply payload of up to maxReply bytes.
 * Returns the reply length, 0 for no reply.
 */
typedef size_t (*protocolHandler)(const uint8_t *data, size_t len,
                                  uint8_t *reply, size_t maxReply);

/**
 * Initialize rtxlink on a character device. Also initializes the device.
 *
 * @return the result of the device initialization, 0 on success. Unlike the
 * upstream branch, which returns nothing.
 */
int rtxlink_init(const struct chardev *dev);

/**
 * Read and dispatch incoming frames. Non-blocking, call periodically.
 */
void rtxlink_task();

/**
 * Shut down rtxlink and the underlying device.
 */
void rtxlink_terminate();

/**
 * Send one frame. Either the whole frame is queued or nothing is.
 *
 * @return 0 on success, -EAGAIN if the device has no room for the frame now,
 * -ENOTCONN if no host is listening, -E2BIG if len exceeds
 * RTXLINK_MAX_PAYLOAD, -ENODEV if rtxlink is not initialized.
 */
int rtxlink_send(const enum ProtocolID proto, const void *data,
                 const size_t len);

/**
 * Register a protocol handler. Returns false if one is already registered.
 */
bool rtxlink_setProtocolHandler(const enum ProtocolID proto,
                                protocolHandler handler);

/**
 * Remove a protocol handler.
 */
void rtxlink_removeProtocolHandler(const enum ProtocolID proto);

#ifdef __cplusplus
}
#endif

#endif /* RTXLINK_H */
