/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef BT_CS7000_H
#define BT_CS7000_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Bluetooth module of the CS7000-M17 Plus (IC804, "BTM869"), driven the way
 * the stock firmware drives it (HOST V9.00.96, see the project notes):
 *
 * - USART1 at 38400 8N1: PA9 to the module's UART_RX, PA10 from its UART_TX,
 *   no flow control.  Commands are text lines ("AT*I", "ATO", "AT*C<addr>",
 *   ...) preceded by 30 NUL bytes and ended by CR; the module answers with
 *   CR-terminated status lines ("I*CHC0<addr>", "I*AHC0", "C:OK", ...).
 * - Power: VREG_ENABLE is pulsed from the shift register line 4 like a power
 *   button (low 200 ms, high 500 ms, low).
 * - Presence: PA14 reads high when the module is fitted.
 * - Audio is analog both ways and needs no driver: the module's MIC input
 *   takes VOL_OUT (receive audio ahead of the volume control), its speaker
 *   output feeds the microphone network (BLUE-MIC).
 *
 * The driver runs a thread that powers the module up, keeps the headset link
 * as the stock firmware does (connect to the paired device it reports, then
 * open the audio link), logs every line from the module, and bridges the
 * module's UART to the "OpenRTX Bluetooth" USB serial port.  Push-to-talk
 * from Bluetooth accessories is decoded and logged; it does not key the
 * radio yet.
 */

/** Start the driver.  Detection and power-up happen in its thread. */
void btCs7000_init();

/** True when the module is fitted. */
bool btCs7000_present();

/** True while a headset is connected (I*CHC0 until I*CHD0). */
bool btCs7000_connected();

/** True while the audio link to the headset is open (I*AHC0 / I*AHD0). */
bool btCs7000_audio();

/** True while push-to-talk is held on a Bluetooth accessory. */
bool btCs7000_ptt();

#ifdef __cplusplus
}
#endif

#endif /* BT_CS7000_H */
