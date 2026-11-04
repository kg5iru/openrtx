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
 */

#include <pthread.h>
#include <string.h>
#include "core/audio_stream.h"
#include "core/threads.h"
#include "drivers/baseband/dmr_CS7000.h"
#include "drivers/baseband/dmr_audio_CS7000.h"
#include "core/log.h"
#include "interfaces/delays.h"

/*
 * Received AMBE+2 frames waiting to be played, 9 bytes each in DMR layout.
 * Single producer (the DMR interrupt, or the YSF mode's thread; only one mode
 * runs at a time), single consumer (the decode thread).
 */
static constexpr unsigned QUEUE_FRAMES = 32;
static uint8_t frameQueue[QUEUE_FRAMES][9];
static volatile unsigned qHead = 0; // written by the producer
static volatile unsigned qTail = 0; // written by the decode thread

bool dmrAudio_pushFrame(const uint8_t frame[9])
{
    unsigned next = (qHead + 1) % QUEUE_FRAMES;
    if (next == qTail)
        return false;
    for (int i = 0; i < 9; i++)
        frameQueue[qHead][i] = frame[i];
    qHead = next;
    return true;
}

unsigned dmrAudio_queued()
{
    return (qHead + QUEUE_FRAMES - qTail) % QUEUE_FRAMES;
}

static bool popFrame(uint8_t frame[9])
{
    if (qTail == qHead)
        return false;
    for (int i = 0; i < 9; i++)
        frame[i] = frameQueue[qTail][i];
    qTail = (qTail + 1) % QUEUE_FRAMES;
    return true;
}

/*
 * Encoded AMBE+2 frames waiting to be transmitted.  Single producer (the
 * encode thread), single consumer (the DMR slot interrupt, 3 frames per
 * voice burst).
 */
static constexpr unsigned TX_QUEUE_FRAMES = 16;
static uint8_t txQueue[TX_QUEUE_FRAMES][9];
static volatile unsigned txHead = 0;
static volatile unsigned txTail = 0;

static unsigned txQueued()
{
    return (txHead + TX_QUEUE_FRAMES - txTail) % TX_QUEUE_FRAMES;
}

bool dmrAudio_popTxFrame(uint8_t frame[9])
{
    if (txTail == txHead)
        return false;
    for (int i = 0; i < 9; i++)
        frame[i] = txQueue[txTail][i];
    txTail = (txTail + 1) % TX_QUEUE_FRAMES;
    return true;
}

static bool pushTxFrame(const uint8_t frame[9])
{
    unsigned next = (txHead + 1) % TX_QUEUE_FRAMES;
    if (next == txTail)
        return false;
    for (int i = 0; i < 9; i++)
        txQueue[txHead][i] = frame[i];
    txHead = next;
    return true;
}

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
static unsigned framesDecoded = 0;
static unsigned errTotal = 0;
static unsigned badFrames = 0;
static unsigned gaps = 0;

static void *decodeThread(void *arg)
{
    (void)arg;

    static int16_t audioBuf[DMA_BUF_SAMPLES];
    memset(audioBuf, 0x00, sizeof(audioBuf));

    streamId oStream = audioStream_start(outPath, audioBuf, DMA_BUF_SAMPLES,
                                         8000, STREAM_OUTPUT | BUF_CIRC_DOUBLE);
    if (oStream < 0) {
        LOG_E("ambe", "output stream start failed (%d)", oStream);
        running = false;
        return NULL;
    }

    LOG_I("ambe", "decode thread started");

    // The codec is initialised on the first voice burst, not here, so that a
    // fault in the codec and a fault in the audio stream can be told apart
    // on the radio (a crash with no DMR traffic is not the codec).
    bool codecReady = false;
    bool playing = false;
    uint8_t sinceFrame = 255; // 20 ms buffers since the last frame played

    uint8_t frame[9];

    // Avoid writing to the buffer while the stream is reading it.
    outputStream_sync(oStream, false);

    while (!reqStop) {
        if (audioPath_getStatus(outPath) != PATH_OPEN)
            break;

        // Jitter buffer: start (or restart after running dry) only once six
        // frames (120 ms: two DMR bursts, more than one YSF frame) are
        // queued, so that the bursty arrival does not insert silence.
        if (!playing && dmrAudio_queued() >= 6)
            playing = true;
        bool have = playing && popFrame(frame);
        if (playing && !have)
            playing = false;

        stream_sample_t *buf = outputStream_getIdleBuffer(oStream);
        if (buf == NULL)
            break;

        if (have && !codecReady) {
            ambe_cs7000_init(&decoder);
            codecReady = true;
        }

        if (have) {
            int e = ambe_cs7000_decode(&decoder, frame, buf);
            errTotal += (unsigned)e;
            if (e >= 4)
                badFrames++;
            framesDecoded++;
            /*
             * Frames resumed after running dry for less than 300 ms: those
             * 20 ms buffers were a gap in the audio.  (Up to v39 the dry
             * buffers were counted as they happened, so the end of every
             * call added about 14 to the count.)
             */
            if ((sinceFrame > 0) && (sinceFrame < 15))
                gaps += sinceFrame;
            sinceFrame = 0;
        } else {
            if (sinceFrame < 255)
                sinceFrame++;
            memset(buf, 0x00, FRAME_SAMPLES * sizeof(stream_sample_t));
        }

        outputStream_sync(oStream, true);
    }

    audioStream_stop(oStream);
    LOG_I("ambe",
          "decode thread stopped: %u frames, %u errors, %u bad, "
          "%u gaps",
          framesDecoded, errTotal, badFrames, gaps);
    running = false;
    return NULL;
}

/*
 * Transmit: an encode thread keeps the TX queue a few frames ahead of the
 * slot interrupt, which takes 3 frames per voice burst (every 60 ms).
 * Bring-up stage (v40): the audio is a 1 kHz test tone, not the microphone.
 */
static ambe_cs7000_t encoder; /* ~10 kB, one context per direction */
static pthread_t txThread;
static volatile bool txRunning = false;
static volatile bool txReqStop = false;
static unsigned txFramesEncoded = 0;

static void *encodeThread(void *arg)
{
    (void)arg;

    static int16_t pcm[FRAME_SAMPLES];
    uint8_t frame[9];
    uint32_t phase = 0; // 1 kHz at 8 kHz: 8 samples per period

    // One period of a 1 kHz sine at 8 kHz, peak 8000 (about -12 dBFS)
    static const int16_t sine[8] = {
        0, 5657, 8000, 5657, 0, -5657, -8000, -5657
    };

    ambe_cs7000_init(&encoder);
    txFramesEncoded = 0;
    LOG_I("ambe", "encode thread started (1 kHz test tone)");

    while (!txReqStop) {
        // Four frames ahead covers the next burst whatever the slot phase
        if (txQueued() >= 4) {
            sleepFor(0u, 5u);
            continue;
        }

        for (size_t i = 0; i < FRAME_SAMPLES; i++)
            pcm[i] = sine[(phase + i) % 8];
        phase = (phase + FRAME_SAMPLES) % 8;

        ambe_cs7000_encode(&encoder, pcm, frame);
        pushTxFrame(frame);
        txFramesEncoded++;
    }

    LOG_I("ambe", "encode thread stopped: %u frames", txFramesEncoded);
    txRunning = false;
    return NULL;
}

bool dmrAudio_startTx()
{
    if (txRunning || (dmrAudio_codecStatus() != DMR_CODEC_OK))
        return false;

    txHead = 0;
    txTail = 0;
    txReqStop = false;
    txRunning = true;

    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setstacksize(&attr, DEC_STACK_SIZE);

    struct sched_param param;
    param.sched_priority = THREAD_PRIO_HIGH;
    pthread_attr_setschedparam(&attr, &param);

    if (pthread_create(&txThread, &attr, encodeThread, NULL) != 0) {
        txRunning = false;
        return false;
    }

    return true;
}

void dmrAudio_stopTx()
{
    if (!txRunning)
        return;

    txReqStop = true;
    pthread_join(txThread, NULL);
}

bool dmrAudio_start(const pathId path)
{
    // A damaged codec region would crash on the first voice frame.
    if (running || (dmrAudio_codecStatus() != DMR_CODEC_OK))
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

uint8_t dmrAudio_codecStatus()
{
    static bool logged = false;

    if (!ambe_cs7000_available())
        return DMR_CODEC_MISSING;

    const ambe_cs7000_check_t *ck = ambe_cs7000_check();
    if (!logged) {
        logged = true;
#ifdef CS7000_AMBE_STOCK
        const char *where = "in the stock image";
#else
        const char *where = "copied into the OpenRTX slot";
#endif
        LOG_I("ambe", "codec %s: %lu bytes, sum %04x, %u bad kB, first %lx",
              where, (unsigned long)ck->end, ck->sum, ck->badKb,
              (unsigned long)ck->firstBad);
    }
    if (ck->badKb != 0)
        return DMR_CODEC_DAMAGED;
    return DMR_CODEC_OK;
}

#else /* !CS7000_AMBE */

uint8_t dmrAudio_codecStatus()
{
    return DMR_CODEC_MISSING;
}

bool dmrAudio_start(const pathId path)
{
    (void)path;
    return false;
}

void dmrAudio_stop()
{
}

bool dmrAudio_startTx()
{
    return false;
}

void dmrAudio_stopTx()
{
}

#endif
