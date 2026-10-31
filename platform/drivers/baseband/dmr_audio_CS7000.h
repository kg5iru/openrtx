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
 * Start the DMR receive audio thread.  It takes the voice bursts collected by
 * dmr_CS7000.cpp, runs them through the extracted AMBE+2 decoder and plays the
 * result on the given (already requested) MCU -> speaker audio path.
 * Requires CS7000_AMBE and the codec blob (see platform/drivers/vocoder).
 *
 * @return true if the thread was started.
 */
bool dmrAudio_start(const pathId path);

/** Stop the thread and wait for it to terminate. */
void dmrAudio_stop();

/** Diagnostics: decoder thread state (0 off, 1 running, 2 codec initialised) and voice frames decoded. */
uint8_t dmrAudio_state();
unsigned dmrAudio_frames();

/** Diagnostics: FEC errors (total, last frame) and silent buffers played during a call. */
unsigned dmrAudio_errTotal();
unsigned dmrAudio_errLast();
unsigned dmrAudio_gaps();

/** Diagnostics: FEC errors summed per frame position in the burst (0..2), frames with 4+ errors. */
unsigned dmrAudio_errPos(int i);
unsigned dmrAudio_badFrames();

/** Diagnostics: frames and frames with 4+ errors under register set v (0..3). */
unsigned dmrAudio_variantFrames(int v);
unsigned dmrAudio_variantBad(int v);

/** Diagnostics: integrity check of the codec region (NULL without codec support). */
const void *dmrAudio_blobCheck();

#ifdef __cplusplus
}
#endif

#endif /* DMR_AUDIO_CS7000_H */
