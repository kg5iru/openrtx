/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef USB_VOICE_CS7000_H
#define USB_VOICE_CS7000_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Vocoder audio (8 kHz) to and from the USB sound card (48 kHz), for the
 * digital voice modes: received voice goes to the computer as well as to the
 * speaker, and a transmission the computer asked for (CAT) takes its voice
 * from the computer instead of the microphone.  Low pass filtered at 3.5 kHz
 * both ways (61-tap windowed sinc).
 */

/**
 * Send decoded voice to the computer, if it is listening.
 *
 * @param pcm: 8 kHz samples.
 * @param n: number of samples.
 */
void usbVoice_rx(const int16_t *pcm, size_t n);

/**
 * Take voice from the computer, at its pace: waits until 6 * n samples have
 * come in at 48 kHz, then decimates them.
 *
 * @param pcm: n samples at 8 kHz.
 * @param timeoutMs: longest wait.
 * @return false (pcm untouched) if they did not come in time.
 */
bool usbVoice_tx(int16_t *pcm, size_t n, unsigned timeoutMs);

/** Start of a transmission: discard the computer's old samples. */
void usbVoice_txStart();

#ifdef __cplusplus
}
#endif

#endif /* USB_VOICE_CS7000_H */
