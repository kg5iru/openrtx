/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef USB_CDC_H
#define USB_CDC_H

#include "interfaces/chardev.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * USB CDC-ACM virtual serial port on the STM32H743 USB2 OTG full-speed core
 * (PA11 = D-, PA12 = D+), as a character device.
 *
 * - VID:PID 0483:5740, serial number from the MCU unique ID. On macOS the
 *   port shows up as /dev/cu.usbmodem<serial>, no driver needed.
 * - 48 MHz USB clock from PLL3 (HSE 25 MHz / 5 * 96 / 10).
 * - VBUS is not sensed: the device always presents itself as attached.
 *
 * Character device semantics:
 * - write() is all-or-nothing and non-blocking: it returns len when the data
 *   is queued, -EAGAIN when the 2 KiB transmit buffer cannot take all of it,
 *   and -ENOTCONN while no host has the port open (DTR clear), the bus is
 *   suspended or the device is not configured.
 * - read() is non-blocking and returns what has arrived, possibly 0.
 * - init() returns -ETIMEDOUT if the USB supply or the core never becomes
 *   ready; the radio keeps running without the port in that case.
 */
extern const struct chardev usb_cdc_chardev;

/** True while a host has the port open (configured, DTR set, not suspended). */
bool usbcdc_connected();

#ifdef __cplusplus
}
#endif

#endif /* USB_CDC_H */
