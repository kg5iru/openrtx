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

#ifdef __cplusplus
}
#endif

#endif /* DMR_AUDIO_CS7000_H */
