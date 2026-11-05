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

/**
 * Start the AMBE+2 receive audio thread (DMR and YSF).  It takes the voice
 * frames queued with dmrAudio_pushFrame(), runs them through the AMBE+2
 * decoder and plays the result on the given (already requested) MCU ->
 * speaker audio path.
 * Requires CS7000_AMBE and the codec blob (see platform/drivers/vocoder).
 *
 * @return true if the thread was started.
 */
bool dmrAudio_start(const pathId path);

/** Stop the thread and wait for it to terminate. */
void dmrAudio_stop();

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
 * Start the transmit encoder: AMBE+2 frames are produced into a queue that
 * the DMR driver empties, 3 frames per voice burst.  Returns false when the
 * codec is not available.
 *
 * @param micPath: open microphone path (SOURCE_MIC to SINK_MCU), or -1 for
 * the synthetic test vowel.
 */
bool dmrAudio_startTx(const pathId micPath);

/** Stop the transmit encoder. */
void dmrAudio_stopTx();

/**
 * Take the next encoded frame (9 bytes, DMR layout) for transmission; false
 * when none is ready.  Called from the DMR slot interrupt.
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
