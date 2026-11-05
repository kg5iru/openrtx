/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "protocols/P25/modem/AudioCodec.hpp"
#include "protocols/P25/modem/Vocoder.hpp"
#include "core/audio_stream.h"
#include "core/threads.h"
#include "core/dsp.h"
#include "core/utils.h"
#include "hwconfig.h"
#include <pthread.h>
#include <cstdlib>
#include <cstring>
#include <errno.h>

#ifdef CONFIG_P25

#ifndef CONFIG_MIC_GAIN
#define CONFIG_MIC_GAIN 1
#endif

#ifndef CONFIG_MIC_OVERSAMPLE
#define CONFIG_MIC_OVERSAMPLE 1
#endif

using namespace P25;

namespace
{

constexpr size_t FRAME_SAMPLES = 160;                 // 20 ms at 8 kHz
constexpr size_t DMA_BUF_SAMPLES = FRAME_SAMPLES * 2; // Double-buffered DMA

/*
 * Queue depth: an LDU carries nine frames (180 ms), the operating mode
 * drains or fills it one LDU at a time.
 */
constexpr size_t QUEUE_SIZE = 24;

struct ImbeFrame {
    uint8_t bytes[AudioCodec::FRAME_BYTES];
};

pathId audioPath;
uint8_t initCnt = 0;
bool isRunning = false;
bool reqStop = false;
pthread_t codecThread;
pthread_attr_t codecAttr;
pthread_mutex_t data_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t init_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t wakeup_cond = PTHREAD_COND_INITIALIZER;

size_t readPos;
size_t writePos;
size_t numElements;
ImbeFrame queue[QUEUE_SIZE];

Vocoder vocoder;

void *encodeFunc(void *arg)
{
    pathId iPath = *((pathId *)arg);
    struct dcBlock dcBlock;
    struct decimatorState decimator;

    // Allocate on the heap, as the stack isn't big enough for oversampling
    // above 2 or so.
    stream_sample_t *adcBuf = static_cast<stream_sample_t *>(malloc(
        DMA_BUF_SAMPLES * CONFIG_MIC_OVERSAMPLE * sizeof(stream_sample_t)));
    if (adcBuf == NULL) {
        if (reqStop == false)
            pthread_detach(pthread_self());
        isRunning = false;
        return NULL;
    }

    streamId iStream = audioStream_start(
        iPath, adcBuf, DMA_BUF_SAMPLES * CONFIG_MIC_OVERSAMPLE,
        8000 * CONFIG_MIC_OVERSAMPLE, STREAM_INPUT | BUF_CIRC_DOUBLE);
    if (iStream < 0) {
        free(adcBuf);
        if (reqStop == false)
            pthread_detach(pthread_self());
        isRunning = false;
        return NULL;
    }

    dsp_resetState(dcBlock);
    dsp_resetState(decimator);

    while (reqStop == false) {
        // Invalid audio path, quit
        if (audioPath_getStatus(iPath) != PATH_OPEN)
            break;

        dataBlock_t audio = inputStream_getData(iStream);
        if (audio.data == NULL || audio.len == 0)
            break;

        // Each inputStream_getData call returns one half of the double
        // buffer: FRAME_SAMPLES * CONFIG_MIC_OVERSAMPLE raw samples.
        // Decimate by averaging each group of CONFIG_MIC_OVERSAMPLE
        // samples.
        size_t outIdx = 0;
        for (size_t i = 0; i < audio.len; i++) {
            int16_t sample = audio.data[i];

#if CONFIG_MIC_OVERSAMPLE > 1
            if (!dsp_decimator(&decimator, &sample, CONFIG_MIC_OVERSAMPLE))
                continue;
#endif

            sample = dsp_dcBlockFilter(&dcBlock, sample);
            audio.data[outIdx++] = s16_mulSat(sample, CONFIG_MIC_GAIN);
        }

        ImbeFrame frame;
        vocoder.encode(audio.data, frame.bytes);

        pthread_mutex_lock(&data_mutex);

        // If the queue is full discard the oldest frame
        if (numElements >= QUEUE_SIZE)
            readPos = (readPos + 1) % QUEUE_SIZE;

        queue[writePos] = frame;
        writePos = (writePos + 1) % QUEUE_SIZE;

        if (numElements == 0)
            pthread_cond_signal(&wakeup_cond);

        if (numElements < QUEUE_SIZE)
            numElements += 1;

        pthread_mutex_unlock(&data_mutex);
    }

    audioStream_terminate(iStream);
    free(adcBuf);

    // In case thread terminates due to invalid path or stream error, detach
    // it to ensure that its memory gets freed by the OS.
    if (reqStop == false)
        pthread_detach(pthread_self());

    isRunning = false;
    return NULL;
}

void *decodeFunc(void *arg)
{
    pathId oPath = *((pathId *)arg);
    int16_t audioBuf[DMA_BUF_SAMPLES];

    // Open output stream
    memset(audioBuf, 0x00, DMA_BUF_SAMPLES * sizeof(stream_sample_t));
    streamId oStream = audioStream_start(oPath, audioBuf, DMA_BUF_SAMPLES, 8000,
                                         STREAM_OUTPUT | BUF_CIRC_DOUBLE);
    if (oStream < 0) {
        pthread_detach(pthread_self());
        isRunning = false;
        return NULL;
    }

    // Synchronise with the output stream before writing the first buffer,
    // to avoid writing the half being played (cracking noises otherwise).
    outputStream_sync(oStream, false);

    while (reqStop == false) {
        // Invalid path, quit
        if (audioPath_getStatus(oPath) != PATH_OPEN)
            break;

        ImbeFrame frame;
        bool newData = false;

        pthread_mutex_lock(&data_mutex);

        if (numElements != 0) {
            frame = queue[readPos];
            readPos = (readPos + 1) % QUEUE_SIZE;
            if (numElements >= QUEUE_SIZE)
                pthread_cond_signal(&wakeup_cond);

            numElements -= 1;
            newData = true;
        }

        pthread_mutex_unlock(&data_mutex);

        stream_sample_t *outBuf = outputStream_getIdleBuffer(oStream);
        if (outBuf == NULL)
            break;

        if (newData)
            vocoder.decode(frame.bytes, outBuf);
        else
            memset(outBuf, 0x00, FRAME_SAMPLES * sizeof(int16_t));

        outputStream_sync(oStream, true);
    }

    // Stop stream and wait until its effective termination
    audioStream_stop(oStream);

    if (reqStop == false)
        pthread_detach(pthread_self());

    isRunning = false;
    return NULL;
}

void stopThread()
{
    reqStop = true;
    pthread_join(codecThread, NULL);
    isRunning = false;

#ifdef __ZEPHYR__
    void *addr;
    size_t size;

    pthread_attr_getstack(&codecAttr, &addr, &size);
    free(addr);
#endif
}

bool startThread(const pathId path, void *(*func)(void *))
{
    // Bad incoming path
    if (audioPath_getStatus(path) != PATH_OPEN)
        return false;

    // Only one call at a time can start the thread.
    pthread_mutex_lock(&init_mutex);
    if (isRunning) {
        // Same path as before, path open, codec already running: all good.
        if (path == audioPath) {
            pthread_mutex_unlock(&init_mutex);
            return true;
        }

        // New path takes over the current one only if it has an higher
        // priority or the current one is closed/suspended.
        pathInfo_t newPath = audioPath_getInfo(path);
        pathInfo_t curPath = audioPath_getInfo(audioPath);
        if ((curPath.status == PATH_OPEN) && (curPath.prio >= newPath.prio)) {
            pthread_mutex_unlock(&init_mutex);
            return false;
        } else {
            stopThread();
        }
    }

    isRunning = true;
    audioPath = path;
    pthread_mutex_unlock(&init_mutex);

    readPos = 0;
    writePos = 0;
    numElements = 0;
    reqStop = false;

    pthread_attr_init(&codecAttr);

#if defined(_MIOSIX)
    pthread_attr_setstacksize(&codecAttr, IMBE_THREAD_STKSIZE);

    // Same priority as the RTX thread.
    struct sched_param param;
    param.sched_priority = THREAD_PRIO_HIGH;
    pthread_attr_setschedparam(&codecAttr, &param);
#elif defined(__ZEPHYR__)
    void *codec_thread_stack = malloc(IMBE_THREAD_STKSIZE * sizeof(uint8_t));
    pthread_attr_setstack(&codecAttr, codec_thread_stack, IMBE_THREAD_STKSIZE);
#endif

    int ret = pthread_create(&codecThread, &codecAttr, func, &audioPath);
    if (ret < 0)
        isRunning = false;

    return isRunning;
}

} // namespace

void AudioCodec::init()
{
    pthread_mutex_lock(&init_mutex);
    initCnt += 1;
    bool first = (initCnt == 1);
    pthread_mutex_unlock(&init_mutex);

    if (first == false)
        return;

    isRunning = false;
    readPos = 0;
    writePos = 0;
    numElements = 0;
    vocoder.init();
}

void AudioCodec::terminate()
{
    pthread_mutex_lock(&init_mutex);
    if (initCnt > 0)
        initCnt -= 1;
    bool last = (initCnt == 0);
    pthread_mutex_unlock(&init_mutex);

    if (last == false)
        return;

    if (isRunning)
        stopThread();

    vocoder.terminate();
}

bool AudioCodec::startEncode(const pathId path)
{
    return startThread(path, encodeFunc);
}

bool AudioCodec::startDecode(const pathId path)
{
    return startThread(path, decodeFunc);
}

void AudioCodec::stop(const pathId path)
{
    if (isRunning == false)
        return;

    if (audioPath != path)
        return;

    stopThread();
}

bool AudioCodec::running()
{
    return isRunning;
}

int AudioCodec::popFrame(uint8_t *frame, const bool blocking)
{
    if (isRunning == false)
        return -EPERM;

    // No data available and non-blocking call: just return.
    if ((numElements == 0) && (blocking == false))
        return -EAGAIN;

    // Blocking call: wait until some data is pushed
    pthread_mutex_lock(&data_mutex);
    while ((numElements == 0) && isRunning)
        pthread_cond_wait(&wakeup_cond, &data_mutex);

    if (numElements == 0) {
        pthread_mutex_unlock(&data_mutex);
        return -EPERM;
    }

    ImbeFrame element = queue[readPos];
    readPos = (readPos + 1) % QUEUE_SIZE;
    numElements -= 1;
    pthread_mutex_unlock(&data_mutex);

    memcpy(frame, element.bytes, FRAME_BYTES);
    return 0;
}

int AudioCodec::pushFrame(const uint8_t *frame, const bool blocking)
{
    if (isRunning == false)
        return -EPERM;

    ImbeFrame element;
    memcpy(element.bytes, frame, FRAME_BYTES);

    // No space available and non-blocking call: return
    if ((numElements >= QUEUE_SIZE) && (blocking == false))
        return -EAGAIN;

    // Blocking call: wait until there is some free space
    pthread_mutex_lock(&data_mutex);
    while (numElements >= QUEUE_SIZE)
        pthread_cond_wait(&wakeup_cond, &data_mutex);

    queue[writePos] = element;
    writePos = (writePos + 1) % QUEUE_SIZE;
    numElements += 1;

    pthread_mutex_unlock(&data_mutex);
    return 0;
}

#endif /* CONFIG_P25 */
