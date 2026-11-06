/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <math.h>
#include <string.h>
#include "interfaces/delays.h"
#include "usb_audio.h"
#include "usb_voice_CS7000.h"

static constexpr size_t RATIO = 6;                         // 48 kHz / 8 kHz
static constexpr size_t TAPS = 61;
static constexpr size_t HIST = (TAPS + RATIO - 1) / RATIO; // 11

/*
 * Low pass, 3.5 kHz at 48 kHz: windowed sinc (Hamming), unity DC gain.
 * Computed once.
 */
static float taps[TAPS];
static bool tapsReady = false;

static void makeTaps()
{
    if (tapsReady)
        return;

    const float fc = 3500.0f / 48000.0f;
    const float pi = 3.14159265f;
    float sum = 0.0f;
    for (size_t i = 0; i < TAPS; i++) {
        float m = (float)i - (float)(TAPS - 1) / 2.0f;
        float h = (m == 0.0f) ? (2.0f * fc) :
                                (sinf(2.0f * pi * fc * m) / (pi * m));
        h *= 0.54f - 0.46f * cosf(2.0f * pi * (float)i / (float)(TAPS - 1));
        taps[i] = h;
        sum += h;
    }
    for (size_t i = 0; i < TAPS; i++)
        taps[i] /= sum;

    tapsReady = true;
}

static inline int16_t saturate(float v)
{
    if (v > 32767.0f)
        return 32767;
    if (v < -32768.0f)
        return -32768;
    return (int16_t)v;
}

// Receive: interpolation by 6, polyphase (8 kHz history, newest first)
static float rxHist[HIST];

void usbVoice_rx(const int16_t *pcm, size_t n)
{
    if (!usbAudio_rxStreaming())
        return;

    makeTaps();

    int16_t out[RATIO * 40];
    size_t o = 0;
    for (size_t i = 0; i < n; i++) {
        memmove(&rxHist[1], &rxHist[0], (HIST - 1) * sizeof(float));
        rxHist[0] = pcm[i];

        // Output p of this input sample: taps p, p + 6, ... on the history
        for (size_t p = 0; p < RATIO; p++) {
            float acc = 0.0f;
            for (size_t k = 0; (p + (RATIO * k)) < TAPS; k++)
                acc += rxHist[k] * taps[p + (RATIO * k)];
            out[o++] = saturate(acc * (float)RATIO);
        }

        if (o == (sizeof(out) / sizeof(out[0]))) {
            usbAudio_write(out, o);
            o = 0;
        }
    }

    if (o > 0)
        usbAudio_write(out, o);
}

// Transmit: decimation by 6 (48 kHz history, newest last)
static float txHist[TAPS];

void usbVoice_txStart()
{
    usbAudio_flushTx();
    memset(txHist, 0, sizeof(txHist));
}

bool usbVoice_tx(int16_t *pcm, size_t n, unsigned timeoutMs)
{
    makeTaps();

    size_t need = RATIO * n;
    unsigned waited = 0;
    while (usbAudio_available() < need) {
        if (waited >= timeoutMs)
            return false;
        sleepFor(0u, 2u);
        waited += 2;
    }

    int16_t in[RATIO];
    for (size_t i = 0; i < n; i++) {
        usbAudio_read(in, RATIO);
        memmove(&txHist[0], &txHist[RATIO], (TAPS - RATIO) * sizeof(float));
        for (size_t k = 0; k < RATIO; k++)
            txHist[TAPS - RATIO + k] = in[k];

        float acc = 0.0f;
        for (size_t k = 0; k < TAPS; k++)
            acc += txHist[k] * taps[k];
        pcm[i] = saturate(acc);
    }

    return true;
}
