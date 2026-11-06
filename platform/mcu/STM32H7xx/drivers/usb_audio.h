/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef USB_AUDIO_H
#define USB_AUDIO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * USB sound card of the composite USB device (usb_cdc.cpp): USB Audio Class
 * 1.0, 48 kHz, mono, signed 16 bit, both directions.
 *
 * - Receive ("microphone" for the computer): the radio writes received audio
 *   with usbAudio_write(), at its own pace; the device sends 47, 48 or 49
 *   samples per millisecond frame (asynchronous endpoint) to follow it.
 *   Silence is sent while nothing is written.
 * - Transmit ("speaker" for the computer): the computer sends 48 samples per
 *   frame, at its own clock (adaptive endpoint).  The radio takes them either
 *   at the computer's pace (usbAudio_read(), for the vocoders, which queue
 *   frames anyway) or at its own pace through a resampler that keeps the
 *   buffer half full (usbAudio_readResampled(), for a DAC stream).
 *
 * Each direction has a volume and mute control (feature unit) the computer
 * shows as a slider, applied here.  The buffers hold about 85 ms each; the
 * latency is about 40 ms each way.
 */

/** True while the computer is capturing (receive stream selected). */
bool usbAudio_rxStreaming();

/**
 * Queue received audio for the computer.  Samples that do not fit are
 * dropped.  Single producer.
 *
 * @param samples: 48 kHz samples.
 * @param n: number of samples.
 */
void usbAudio_write(const int16_t *samples, size_t n);

/** True while the computer is playing into the radio (transmit stream). */
bool usbAudio_txStreaming();

/** Number of transmit samples waiting. */
size_t usbAudio_available();

/**
 * Take transmit samples at the computer's pace.
 *
 * @return the number of samples copied, up to n.
 */
size_t usbAudio_read(int16_t *samples, size_t n);

/**
 * Take exactly n transmit samples at the radio's pace (48 kHz by the radio's
 * clock), resampled to keep the buffer level steady; silence while the
 * computer sends nothing.
 */
void usbAudio_readResampled(int16_t *samples, size_t n);

/** Discard the transmit samples waiting (start of a transmission). */
void usbAudio_flushTx();

/**
 * Transmit request from the computer (CAT), see platform_getPttStatus().
 * It is dropped when the USB link goes down (unplug, suspend, reset).
 */
void usbAudio_setPtt(bool on);

/** True while the computer asks to transmit. */
bool usbAudio_ptt();

#ifdef __cplusplus
}
#endif

#endif /* USB_AUDIO_H */
