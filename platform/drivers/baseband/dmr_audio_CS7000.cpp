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

#ifdef CONFIG_USB_AUDIO_CS7000
#include "usb_audio.h"
#include "usb_voice_CS7000.h"
#endif
#include <pthread.h>
#include <string.h>
#include "core/audio_stream.h"
#include "core/threads.h"
#include "drivers/baseband/dmr_CS7000.h"
#include "drivers/baseband/dmr_audio_CS7000.h"
#include "core/log.h"
#include "interfaces/delays.h"
#include "drivers/baseband/dmr_diag_CS7000.h"
#include <math.h>
#include <stdlib.h>
#include "core/dsp.h"
#include "hwconfig.h"

/*
 * Voice frame lengths, which also tell the vocoders apart: AMBE+2 (DMR, YSF
 * DN, 9 bytes in DMR layout) or IMBE (YSF VW, 11 bytes).
 */
static constexpr uint8_t AMBE_BYTES = 9;
static constexpr uint8_t IMBE_BYTES = 11;

/*
 * Queue of voice frames for a single producer and a single consumer (an
 * interrupt may be either).
 */
template <unsigned N>
class FrameQueue
{
public:
    bool push(const uint8_t *frame, const uint8_t len)
    {
        unsigned next = (head + 1) % N;
        if (next == tail)
            return false;
        memcpy(frames[head], frame, len);
        lens[head] = len;
        head = next;
        return true;
    }

    // Returns the frame length, 0 when the queue is empty
    uint8_t pop(uint8_t frame[IMBE_BYTES])
    {
        if (tail == head)
            return 0;
        uint8_t len = lens[tail];
        memcpy(frame, frames[tail], len);
        tail = (tail + 1) % N;
        return len;
    }

    unsigned count() const
    {
        return (head + N - tail) % N;
    }

    void clear()
    {
        head = 0;
        tail = 0;
    }

private:
    // Every member initialised: a static queue needs no startup constructor
    uint8_t frames[N][IMBE_BYTES] = {};
    uint8_t lens[N] = {};
    volatile unsigned head = 0; // written by the producer
    volatile unsigned tail = 0; // written by the consumer
};

/*
 * Received frames waiting to be played: the DMR interrupt or the YSF mode's
 * thread (only one mode runs at a time) to the decode thread.
 */
static FrameQueue<32> rxQueue;

/*
 * Encoded frames waiting to be transmitted: the encode thread to the DMR
 * slot interrupt (3 frames per voice burst) or the YSF mode (5 per frame).
 */
static FrameQueue<16> txQueue;

// Silence frames, sent when the encoder has no frame ready
static const uint8_t ambeSilence[AMBE_BYTES] = { 0xB9, 0xE8, 0x81, 0x52, 0x61,
                                                 0x73, 0x00, 0x2A, 0x6B };
static const uint8_t imbeSilence[IMBE_BYTES] = { 0x04, 0x0C, 0xFD, 0x7B,
                                                 0xFB, 0x7D, 0xF2, 0x7B,
                                                 0x3D, 0x9E, 0x44 };

bool dmrAudio_pushFrame(const uint8_t frame[9])
{
    return rxQueue.push(frame, AMBE_BYTES);
}

unsigned dmrAudio_queued()
{
    return rxQueue.count();
}

// Next encoded frame, or silence of the given length if none is ready
static bool popTx(uint8_t *frame, const uint8_t len, const uint8_t *silence)
{
    uint8_t buf[IMBE_BYTES];
    if (txQueue.pop(buf) == len) {
        memcpy(frame, buf, len);
        return true;
    }
    memcpy(frame, silence, len);
    return false;
}

bool dmrAudio_popTxFrame(uint8_t frame[9])
{
    return popTx(frame, AMBE_BYTES, ambeSilence);
}

#ifdef CONFIG_P25
bool dmrAudio_pushImbe(const uint8_t frame[11])
{
    return rxQueue.push(frame, IMBE_BYTES);
}

bool dmrAudio_popTxImbe(uint8_t frame[11])
{
    return popTx(frame, IMBE_BYTES, imbeSilence);
}
#endif

static bool audioStart(const pathId path);
static void audioStop();

/*
 * Speaker squelch, shared by the DMR and YSF modes: the speaker path (and the
 * audio amplifier) is open only during a call and while its last frames play
 * out, so that the amplifier does not hiss between calls (up to v34 it stayed
 * open for the whole mode).
 */
static constexpr long long SPEAKER_HANG_MS = 300;
static pathId speakerPath = -1;
static bool speakerOn = false;
static long long lastActive = 0;

void dmrAudio_squelch(const bool call)
{
    bool active = call || (rxQueue.count() > 0);
    if (active)
        lastActive = getTick();

    if (active && !speakerOn) {
        speakerPath = audioPath_request(SOURCE_MCU, SINK_SPK, PRIO_RX);
        if ((speakerPath > 0) && audioStart(speakerPath))
            speakerOn = true;
        else if (speakerPath > 0)
            audioPath_release(speakerPath);
    } else if (!active && speakerOn
               && ((getTick() - lastActive) >= SPEAKER_HANG_MS)) {
        dmrAudio_speakerOff();
    }
}

void dmrAudio_speakerOff()
{
    if (!speakerOn)
        return;
    audioStop();
    audioPath_release(speakerPath);
    speakerOn = false;
}

#ifdef CS7000_AMBE

#include "ambe_cs7000.h"

#define FRAME_SAMPLES AMBE_CS7000_FRAME_SAMPLES
#define DMA_BUF_SAMPLES (FRAME_SAMPLES * 2)
#define DEC_STACK_SIZE 16384 /* codec needs 4.2 kB measured; same as Codec2 */

// Codec threads (decode, encode): high priority, as the Codec2 thread
static bool startThread(pthread_t *t, void *(*fn)(void *))
{
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setstacksize(&attr, DEC_STACK_SIZE);

    struct sched_param param;
    param.sched_priority = THREAD_PRIO_HIGH;
    pthread_attr_setschedparam(&attr, &param);

    return pthread_create(t, &attr, fn, NULL) == 0;
}

static ambe_cs7000_t decoder; /* ~10 kB, too big for the stack */

#ifdef CONFIG_P25
/*
 * IMBE (YSF VW) through the P25 vocoder engine: one decoder and one encoder
 * state, apart from the P25 mode's own, as both directions run in the threads
 * here.  Sizes from imbe_embedded (decoder 1932 B, encoder 8608 B), checked
 * at initialisation.
 */
#include <imbe2.h>
alignas(8) static uint8_t imbeDec[2048];
alignas(8) static uint8_t imbeEnc[8704];
#endif
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
#ifdef CONFIG_P25
    bool imbeReady = false;
#endif
    bool playing = false;
    uint8_t sinceFrame = 255; // 20 ms buffers since the last frame played

    uint8_t frame[IMBE_BYTES];

    // Avoid writing to the buffer while the stream is reading it.
    outputStream_sync(oStream, false);

    while (!reqStop) {
        if (audioPath_getStatus(outPath) != PATH_OPEN)
            break;

        // Jitter buffer: start (or restart after running dry) only once six
        // frames (120 ms: two DMR bursts, more than one YSF frame) are
        // queued, so that the bursty arrival does not insert silence.
        if (!playing && rxQueue.count() >= 6)
            playing = true;
        uint8_t len = playing ? rxQueue.pop(frame) : 0;
        bool have = (len != 0);
        if (playing && !have)
            playing = false;

        stream_sample_t *buf = outputStream_getIdleBuffer(oStream);
        if (buf == NULL)
            break;

#ifdef CONFIG_P25
        if (len == IMBE_BYTES) {
            if (!imbeReady && (imbe2_decoder_state_size() <= sizeof(imbeDec))) {
                imbe2_decoder_init(imbeDec);
                imbeReady = true;
            }
            if (imbeReady)
                imbe2_decode(imbeDec, buf, frame);
            else
                memset(buf, 0x00, FRAME_SAMPLES * sizeof(stream_sample_t));
        }
#endif

        if ((len == AMBE_BYTES) && !codecReady) {
            ambe_cs7000_init(&decoder);
            codecReady = true;
        }

        if (len == AMBE_BYTES) {
            int e = ambe_cs7000_decode(&decoder, frame, buf);
            errTotal += (unsigned)e;
            if (e >= 4)
                badFrames++;
        }

        if (have) {
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

#ifdef CONFIG_USB_AUDIO_CS7000
        // The computer hears what the speaker plays
        if (have)
            usbVoice_rx(buf, FRAME_SAMPLES);
#endif

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
 *
 * Bring-up test signal (v41): a synthetic vowel, a pulse-like harmonic
 * series whose pitch glides between 90 and 170 Hz, shaped by two formants
 * (700 and 1200 Hz) and gated on and off like syllables.  A steady 1 kHz
 * tone (v40) encodes, after 0.2 s, to the same frame over and over (checked
 * in emulation), which receivers treat as repeats and mute.
 */
static ambe_cs7000_t encoder; /* ~10 kB, one context per direction */
static pthread_t txThread;
static volatile bool txRunning = false;
static volatile bool txReqStop = false;
static unsigned txFramesEncoded = 0;

static void testVowel(int16_t *pcm, uint32_t frame)
{
    static float phase = 0.0f;
    const float twoPi = 6.2831853f;

    for (size_t i = 0; i < FRAME_SAMPLES; i++) {
        float t = (float)((frame * FRAME_SAMPLES) + i) / 8000.0f;
        float f0 = 130.0f + (40.0f * sinf(twoPi * 0.7f * t));
        phase += twoPi * f0 / 8000.0f;
        if (phase > twoPi)
            phase -= twoPi;

        // Syllables: 400 ms of sound, 200 ms of near silence
        float syl = fmodf(t, 0.6f);
        float env = (syl < 0.4f) ? sinf(3.1415927f * syl / 0.4f) : 0.02f;

        float v = 0.0f;
        for (int k = 1; k <= 24; k++) {
            float fk = f0 * (float)k;
            if (fk > 3600.0f)
                break;
            float d1 = (fk - 700.0f) / 150.0f;
            float d2 = (fk - 1200.0f) / 200.0f;
            float a = (1.0f / (1.0f + (d1 * d1))) + (0.6f / (1.0f + (d2 * d2)))
                    + 0.05f;
            v += a * sinf(phase * (float)k);
        }

        pcm[i] = (int16_t)(v * env * 2500.0f);
    }
}

static pathId txMicPath = -1;
static bool txImbe = false; // IMBE (YSF VW) rather than AMBE+2

// One 20 ms frame through the vocoder chosen, into the TX queue
static bool encodeFrame(const int16_t *pcm)
{
    uint8_t frame[IMBE_BYTES];
    uint8_t len = AMBE_BYTES;

    dmrDiag_set(DIAG_ENC, txFramesEncoded | 0x80000000u);
#ifdef CONFIG_P25
    if (txImbe) {
        imbe2_encode(imbeEnc, frame, pcm);
        len = IMBE_BYTES;
    } else
#endif
        ambe_cs7000_encode(&encoder, pcm, frame);
    txFramesEncoded++;
    dmrDiag_set(DIAG_ENC, txFramesEncoded);

    return txQueue.push(frame, len);
}

// Bring-up test signal: frames paced by the TX queue (4 frames ahead)
static void encodeVowel()
{
    static int16_t pcm[FRAME_SAMPLES];

    while (!txReqStop) {
        if (txQueue.count() >= 4) {
            sleepFor(0u, 5u);
            continue;
        }

        testVowel(pcm, txFramesEncoded);
        encodeFrame(pcm);
    }
}

/*
 * Microphone: the same capture as the Codec2 encoder (core/audio_codec.c),
 * 8 kHz oversampled CONFIG_MIC_OVERSAMPLE times, decimated, DC blocked and
 * amplified by CONFIG_MIC_GAIN (saturated here), 160 samples per frame.
 * Frames are paced by the microphone clock; the slot interrupt takes three
 * every 60 ms, so the queue settles at about the 180 ms of the headers.
 */
static unsigned txDropped = 0;
static int16_t txPeak = 0;
static int32_t txGain = CONFIG_MIC_GAIN; // x CONFIG_MIC_GAIN, 3 dB steps

// CONFIG_MIC_GAIN scaled by -12 .. +12 dB, in 3 dB steps (x 1/16)
static const uint8_t gainSteps[9] = { 4, 6, 8, 11, 16, 23, 32, 45, 64 };

static void encodeMic()
{
    const size_t bufLen = FRAME_SAMPLES * 2 * CONFIG_MIC_OVERSAMPLE;
    stream_sample_t *adcBuf =
        (stream_sample_t *)malloc(bufLen * sizeof(stream_sample_t));
    if (adcBuf == NULL) {
        LOG_E("ambe", "no memory for the microphone buffer");
        return;
    }

    streamId iStream = audioStream_start(txMicPath, adcBuf, bufLen,
                                         8000 * CONFIG_MIC_OVERSAMPLE,
                                         STREAM_INPUT | BUF_CIRC_DOUBLE);
    if (iStream < 0) {
        LOG_E("ambe", "microphone stream start failed (%d)", iStream);
        free(adcBuf);
        return;
    }

    struct dcBlock dcBlock;
    struct decimatorState decimator;
    dsp_resetState(dcBlock);
    dsp_resetState(decimator);

    static int16_t pcm[FRAME_SAMPLES];

    while (!txReqStop) {
        if (audioPath_getStatus(txMicPath) != PATH_OPEN)
            break;

        dataBlock_t audio = inputStream_getData(iStream);
        if ((audio.data == NULL) || (audio.len == 0))
            break;

        size_t n = 0;
        for (size_t i = 0; (i < audio.len) && (n < FRAME_SAMPLES); i++) {
            int16_t sample = audio.data[i];
#if CONFIG_MIC_OVERSAMPLE > 1
            if (!dsp_decimator(&decimator, &sample, CONFIG_MIC_OVERSAMPLE))
                continue;
#endif
            sample = dsp_dcBlockFilter(&dcBlock, sample);
            int32_t v = ((int32_t)sample * txGain) / 16;
            // Soft limit above 3/4 of full scale, then saturate
            const int32_t knee = 24576;
            if (v > knee)
                v = knee + ((v - knee) / 4);
            if (v < -knee)
                v = -knee + ((v + knee) / 4);
            if (v > 32767)
                v = 32767;
            if (v < -32768)
                v = -32768;
            pcm[n++] = (int16_t)v;
            int32_t a = (v < 0) ? -v : v;
            if (a > txPeak)
                txPeak = (int16_t)((a > 32767) ? 32767 : a);
        }
        while (n < FRAME_SAMPLES)
            pcm[n++] = 0;

        if (!encodeFrame(pcm))
            txDropped++;
    }

    audioStream_terminate(iStream);
    free(adcBuf);
}

#ifdef CONFIG_USB_AUDIO_CS7000
/*
 * Voice from the computer (a transmission it asked for over CAT): 48 kHz
 * from the USB sound card, decimated to 8 kHz, paced by the computer.  The
 * USB volume control sets the level; the microphone gain does not apply.
 */
static bool txFromUsb = false;

static void encodeUsb()
{
    static int16_t pcm[FRAME_SAMPLES];

    while (!txReqStop) {
        // Nothing for 100 ms: the slot interrupt sends silence meanwhile
        if (!usbVoice_tx(pcm, FRAME_SAMPLES, 100))
            continue;

        for (size_t i = 0; i < FRAME_SAMPLES; i++) {
            int32_t a = (pcm[i] < 0) ? -(int32_t)pcm[i] : pcm[i];
            if (a > txPeak)
                txPeak = (int16_t)((a > 32767) ? 32767 : a);
        }

        if (!encodeFrame(pcm))
            txDropped++;
    }
}
#endif

static void *encodeThread(void *arg)
{
    (void)arg;

#ifdef CONFIG_P25
    if (txImbe) {
        if (imbe2_encoder_state_size() > sizeof(imbeEnc)) {
            LOG_E("ambe", "IMBE encoder state too large");
            txRunning = false;
            return NULL;
        }
        imbe2_encoder_init(imbeEnc);
    } else
#endif
        ambe_cs7000_init(&encoder);
    txFramesEncoded = 0;
    txDropped = 0;
    txPeak = 0;

    const char *vocoder = txImbe ? "IMBE" : "AMBE+2";
    bool usbSource = false;
#ifdef CONFIG_USB_AUDIO_CS7000
    usbSource = txFromUsb;
#endif
    if (usbSource) {
#ifdef CONFIG_USB_AUDIO_CS7000
        LOG_I("ambe", "encode thread started (USB audio, %s)", vocoder);
        encodeUsb();
#endif
    } else if (txMicPath >= 0) {
        LOG_I("ambe", "encode thread started (microphone, %s)", vocoder);
        encodeMic();
    } else {
        LOG_I("ambe", "encode thread started (test vowel, %s)", vocoder);
        encodeVowel();
    }

    LOG_I("ambe",
          "encode thread stopped: %u frames, %u dropped, mic peak %d, "
          "gain %ld/16 x %d",
          txFramesEncoded, txDropped, txPeak, (long)(txGain / CONFIG_MIC_GAIN),
          CONFIG_MIC_GAIN);
    txRunning = false;
    return NULL;
}

static bool startTx(const uint8_t gainStep, const bool imbe)
{
    if (txRunning || (dmrAudio_codecStatus() != DMR_CODEC_OK))
        return false;

    txImbe = imbe;

#ifdef CONFIG_USB_AUDIO_CS7000
    // A transmission the computer asked for takes its voice from the computer
    txFromUsb = usbAudio_ptt();
    if (txFromUsb) {
        usbVoice_txStart();
        txMicPath = -1;
    } else
#endif
    {
        txMicPath = audioPath_request(SOURCE_MIC, SINK_MCU, PRIO_TX);
        if (txMicPath < 0)
            LOG_W("ambe", "no microphone path (%d), sending the test vowel",
                  txMicPath);
    }

    txGain = (int32_t)CONFIG_MIC_GAIN
           * gainSteps[(gainStep < 9) ? gainStep : 4];

    txQueue.clear();
    txReqStop = false;
    txRunning = true;

    if (!startThread(&txThread, encodeThread)) {
        txRunning = false;
        if (txMicPath >= 0)
            audioPath_release(txMicPath);
        txMicPath = -1;
        return false;
    }

    return true;
}

bool dmrAudio_startTx(const uint8_t gainStep)
{
    return startTx(gainStep, false);
}

#ifdef CONFIG_P25
bool dmrAudio_startTxImbe(const uint8_t gainStep)
{
    return startTx(gainStep, true);
}
#endif

void dmrAudio_stopTx()
{
    if (txRunning) {
        txReqStop = true;
        pthread_join(txThread, NULL);
    }
    if (txMicPath >= 0)
        audioPath_release(txMicPath);
    txMicPath = -1;
}

static bool audioStart(const pathId path)
{
    // A damaged codec region would crash on the first voice frame.
    if (running || (dmrAudio_codecStatus() != DMR_CODEC_OK))
        return false;

    outPath = path;
    reqStop = false;
    running = true;

    if (!startThread(&thread, decodeThread)) {
        running = false;
        return false;
    }

    return true;
}

static void audioStop()
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

static bool audioStart(const pathId path)
{
    (void)path;
    return false;
}

static void audioStop()
{
}

bool dmrAudio_startTx(const uint8_t gainStep)
{
    (void)gainStep;
    return false;
}

#ifdef CONFIG_P25
bool dmrAudio_startTxImbe(const uint8_t gainStep)
{
    (void)gainStep;
    return false;
}
#endif

void dmrAudio_stopTx()
{
}

#endif
