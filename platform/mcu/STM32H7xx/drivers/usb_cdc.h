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
 * USB serial ports of the composite USB device on the STM32H743 USB2 OTG
 * full-speed core (PA11 = D-, PA12 = D+), as character devices.  The same
 * device is also a USB sound card (usb_audio.h).
 *
 * - VID:PID 0483:5740, serial number from the MCU unique ID.  Two CDC-ACM
 *   ports, no driver needed: the system log (interface 0, "OpenRTX log")
 *   and CAT control (interface 2, "OpenRTX CAT").  On macOS they show up as
 *   /dev/cu.usbmodem<serial>1 and ...3.
 * - 48 MHz USB clock from PLL3 (HSE 25 MHz / 5 * 96 / 10).
 * - VBUS is not sensed: the device always presents itself as attached.
 * - The first init() of either port starts the whole device.
 *
 * Character device semantics:
 * - write() is all-or-nothing and non-blocking: it returns len when the data
 *   is queued, -EAGAIN when the transmit buffer (log 2 KiB, CAT 512 bytes)
 *   cannot take all of it, and -ENOTCONN while the port is not usable: the
 *   device is not configured or the bus is suspended, and for the log port
 *   also while no host has it open (DTR clear).  The CAT port answers
 *   whatever DTR says, as programs do not all set it.
 * - read() is non-blocking and returns what has arrived, possibly 0.
 * - init() returns -ETIMEDOUT if the USB supply or the core never becomes
 *   ready; the radio keeps running without the device in that case.
 */
extern const struct chardev usb_cdc_chardev;
extern const struct chardev usb_cat_chardev;

/** True while a host has the log port open (configured, DTR set, awake). */
bool usbcdc_connected();

/** True while the CAT port is usable (configured, not suspended). */
bool usbcat_connected();

#ifdef __cplusplus
}
#endif

#endif /* USB_CDC_H */
