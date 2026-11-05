/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef P25_MODEM_AUDIO_CODEC_H
#define P25_MODEM_AUDIO_CODEC_H

#ifndef __cplusplus
#error This header is C++ only!
#endif

#include <cstdint>
#include <cstddef>
#include "core/audio_path.h"

namespace P25
{

/**
 * IMBE audio codec glue, the P25 counterpart of core/audio_codec: a thread
 * encodes microphone audio into 20 ms IMBE frames (11 bytes) or decodes
 * IMBE frames into speaker audio, exchanging frames with the operating mode
 * through a small queue. One codec runs at a time, on one audio path.
 */
class AudioCodec
{
public:
    static constexpr size_t FRAME_BYTES = 11; ///< One 88-bit IMBE frame

    /**
     * Initialise the codec, to be called before any other function.
     */
    static void init();

    /**
     * Shut down the codec, stopping the running thread if any.
     */
    static void terminate();

    /**
     * Start encoding audio from a microphone path into IMBE frames.
     *
     * @param path: audio path, must be open.
     * @return true on success.
     */
    static bool startEncode(const pathId path);

    /**
     * Start decoding IMBE frames to a speaker path.
     *
     * @param path: audio path, must be open.
     * @return true on success.
     */
    static bool startDecode(const pathId path);

    /**
     * Stop the codec, if running on the given path.
     *
     * @param path: audio path the codec was started on.
     */
    static void stop(const pathId path);

    /**
     * @return true if the codec thread is running.
     */
    static bool running();

    /**
     * Get an encoded IMBE frame.
     *
     * @param frame: destination, FRAME_BYTES.
     * @param blocking: wait for a frame when the queue is empty.
     * @return 0 on success, -EAGAIN if no frame is available and not
     * blocking, -EPERM if the codec is not running.
     */
    static int popFrame(uint8_t *frame, const bool blocking);

    /**
     * Queue an IMBE frame for decoding.
     *
     * @param frame: FRAME_BYTES.
     * @param blocking: wait for space when the queue is full.
     * @return 0 on success, -EAGAIN if the queue is full and not blocking,
     * -EPERM if the codec is not running.
     */
    static int pushFrame(const uint8_t *frame, const bool blocking);
};

} /* P25 */

#endif /* P25_MODEM_AUDIO_CODEC_H */
