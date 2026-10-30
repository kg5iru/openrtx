/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/*
 * DMR receive audio for the CS7000: voice bursts from the HR_C6000 are
 * decoded by the AMBE+2 half-rate codec extracted from the stock firmware
 * and streamed to the speaker through the standard OpenRTX audio stream API,
 * the same way the M17 Codec2 decoder does it (8 kHz, signed 16 bit, 160
 * samples = 20 ms per frame, double buffered).
 *
 * One burst = 27 bytes = 3 voice frames of 9 bytes.  The stock firmware
 * feeds each 9 byte slice of the burst straight into the codec (it unpacks 72
 * bits MSB first at offset 0, 9 or 18), so no de-interleaving is done here.
 *
 * STATUS: not compiled or tested on hardware.
 */

#include <pthread.h>
#include <string.h>
#include "core/audio_stream.h"
#include "core/threads.h"
#include "drivers/baseband/dmr_CS7000.h"
#include "drivers/baseband/dmr_audio_CS7000.h"

#ifdef CS7000_AMBE

#include "ambe_cs7000.h"

#define FRAME_SAMPLES AMBE_CS7000_FRAME_SAMPLES
#define DMA_BUF_SAMPLES (FRAME_SAMPLES * 2)
#define DEC_STACK_SIZE 16384  /* codec needs 4.2 kB measured; same as Codec2 */

static ambe_cs7000_t decoder; /* ~10 kB, too big for the stack */
static pthread_t thread;
static volatile bool running = false;
static volatile bool reqStop = false;
static pathId outPath;
static volatile unsigned framesDecoded = 0;
static volatile uint8_t decState = 0;

static void *decodeThread(void *arg)
{
    (void)arg;

    static int16_t audioBuf[DMA_BUF_SAMPLES];
    memset(audioBuf, 0x00, sizeof(audioBuf));

    streamId oStream = audioStream_start(outPath, audioBuf, DMA_BUF_SAMPLES,
                                         8000, STREAM_OUTPUT | BUF_CIRC_DOUBLE);
    if (oStream < 0) {
        running = false;
        return NULL;
    }

    // The codec is initialised on the first voice burst, not here, so that a
    // fault in the codec and a fault in the audio stream can be told apart
    // on the radio (a crash with no DMR traffic is not the codec).
    bool codecReady = false;

    uint8_t frames[27];
    uint8_t next = 3; // index of the next frame to decode, 3 = none left

    // Avoid writing to the buffer while the stream is reading it.
    outputStream_sync(oStream, false);
    decState = 1;

    while (!reqStop) {
        if (audioPath_getStatus(outPath) != PATH_OPEN)
            break;

        if (next >= 3 && dmr_popVoiceBurst(frames))
            next = 0;

        stream_sample_t *buf = outputStream_getIdleBuffer(oStream);
        if (buf == NULL)
            break;

        if (next < 3 && !codecReady) {
            ambe_cs7000_init(&decoder);
            codecReady = true;
            decState = 2;
        }

        if (next < 3) {
            ambe_cs7000_decode(&decoder,
                               &frames[next * AMBE_CS7000_FRAME_BYTES], buf);
            framesDecoded++;
            next += 1;
        } else {
            memset(buf, 0x00, FRAME_SAMPLES * sizeof(stream_sample_t));
        }

        outputStream_sync(oStream, true);
    }

    audioStream_stop(oStream);
    decState = 0;
    running = false;
    return NULL;
}

bool dmrAudio_start(const pathId path)
{
    if (running || !ambe_cs7000_available())
        return false;

    outPath = path;
    reqStop = false;
    running = true;

    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setstacksize(&attr, DEC_STACK_SIZE);

    struct sched_param param;
    param.sched_priority = THREAD_PRIO_HIGH; // as the Codec2 thread
    pthread_attr_setschedparam(&attr, &param);

    if (pthread_create(&thread, &attr, decodeThread, NULL) != 0) {
        running = false;
        return false;
    }

    return true;
}

void dmrAudio_stop()
{
    if (!running)
        return;

    reqStop = true;
    pthread_join(thread, NULL);
}

uint8_t dmrAudio_state()
{
    return decState;
}

unsigned dmrAudio_frames()
{
    return framesDecoded;
}

#else /* !CS7000_AMBE */

uint8_t dmrAudio_state()
{
    return 0;
}

unsigned dmrAudio_frames()
{
    return 0;
}

bool dmrAudio_start(const pathId path)
{
    (void)path;
    return false;
}

void dmrAudio_stop()
{
}

#endif
