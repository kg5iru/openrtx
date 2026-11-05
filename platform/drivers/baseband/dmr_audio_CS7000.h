/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef DMR_AUDIO_CS7000_H
#define DMR_AUDIO_CS7000_H

#include <stdbool.h>
#include <stdint.h>
#include "core/audio_path.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * AMBE+2 voice for the DMR and YSF modes of the CS7000.  Requires CS7000_AMBE
 * and the codec of the stock firmware (see platform/drivers/vocoder).
 */

/**
 * Receive audio squelch, to be called at every update of the mode while
 * receiving.  While a call is on, or received frames are queued, the speaker
 * path is open and a thread runs the frames queued with dmrAudio_pushFrame()
 * through the AMBE+2 decoder to the speaker; 300 ms after the last of them
 * both are stopped, so that the audio amplifier does not hiss between calls.
 *
 * @param call: a call is being received.
 */
void dmrAudio_squelch(const bool call);

/** Close the speaker path at once (to transmit, or on leaving the mode). */
void dmrAudio_speakerOff();

/**
 * Queue one received AMBE+2 frame (9 bytes, 72 bits in DMR layout) for
 * playback.  Single producer: callable from an interrupt.
 *
 * @return false if the queue was full and the frame was dropped.
 */
bool dmrAudio_pushFrame(const uint8_t frame[9]);

/** Number of frames waiting to be played. */
unsigned dmrAudio_queued();

/**
 * Start the transmit encoder: the microphone path is opened and AMBE+2 frames
 * are produced into a queue that the mode empties (DMR: 3 frames per voice
 * burst, YSF: 5 per frame).  Without a microphone path a synthetic test vowel
 * is encoded instead (bring-up aid).
 *
 * @param gainStep: microphone gain, 0..8 for -12 .. +12 dB in 3 dB steps
 * around CONFIG_MIC_GAIN (4 = 0 dB).
 * @return false when the codec is not available.
 */
bool dmrAudio_startTx(const uint8_t gainStep);

/** Stop the transmit encoder and release the microphone path. */
void dmrAudio_stopTx();

/**
 * Take the next encoded frame (9 bytes, DMR layout) for transmission.  Called
 * from the DMR slot interrupt, or by the YSF mode.
 *
 * @return false, with an AMBE+2 silence frame in frame, when none is ready.
 */
bool dmrAudio_popTxFrame(uint8_t frame[9]);

#define DMR_CODEC_OK 0      ///< codec region present and intact
#define DMR_CODEC_MISSING 1 ///< built without the codec, or region erased
#define DMR_CODEC_DAMAGED 2 ///< region present but its checksums differ

/** State of the codec region in flash (checked once, at the first call). */
uint8_t dmrAudio_codecStatus();

#ifdef __cplusplus
}
#endif

#endif /* DMR_AUDIO_CS7000_H */
