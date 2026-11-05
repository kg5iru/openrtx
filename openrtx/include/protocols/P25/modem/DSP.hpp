/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef P25_MODEM_DSP_H
#define P25_MODEM_DSP_H

#ifndef __cplusplus
#error This header is C++ only!
#endif

#include <array>
#include <tuple>
#include <cstdint>
#include "core/fir.hpp"

namespace P25
{

/*
 * Coefficients for the P25 modem filters, TIA-102 C4FM model as implemented
 * by OP25 (gr-op25_repeater/apps/tx/op25_c4fm_mod.py, transfer_function_tx
 * and transfer_function_rx; tables regenerated and checked by
 * scripts/o25_filters.py).
 *
 * Unlike M17, which splits the Nyquist shaping as RRC 0.5 at both ends,
 * C4FM puts a Nyquist raised cosine (beta = 0.2: flat to 1920 Hz, zero at
 * 2880 Hz) times a shaping filter P(f) = (pi f/4800)/sin(pi f/4800) in the
 * transmitter, and an integrate-and-dump D(f) = sin(pi f/4800)/(pi f/4800)
 * in the receiver. D cancels P, so the end-to-end response is the raised
 * cosine; neither half is Nyquist on its own. Both tables sum to one.
 */
// TX filter: raised cosine (beta 0.2) x P(f), 10 sps @ 48 kHz, span 13
// symbols, 1 Hz frequency table up to 2880 Hz inverse-transformed at 48k.
static constexpr std::array<float, 131> tx_pulse_48k = {
    7.0110872784e-04f,  8.8008811483e-04f,  9.7093040636e-04f,
    9.4087342622e-04f,  7.6897244092e-04f,  4.5087281640e-04f,
    1.9446445974e-06f,  -5.4184050926e-04f, -1.1256812833e-03f,
    -1.6804539189e-03f, -2.1296974068e-03f, -2.3984711082e-03f,
    -2.4231740164e-03f, -2.1611933058e-03f, -1.5991478655e-03f,
    -7.5853388690e-04f, 3.0222754609e-04f,  1.4900062402e-03f,
    2.6836587108e-03f,  3.7449018789e-03f,  4.5325296142e-03f,
    4.9186525616e-03f,  4.8052673120e-03f,  4.1392506133e-03f,
    2.9238669111e-03f,  1.2250914212e-03f,  -8.2851970562e-04f,
    -3.0530669725e-03f, -5.2235646767e-03f, -7.0943306002e-03f,
    -8.4238523098e-03f, -9.0018236121e-03f, -8.6755943844e-03f,
    -7.3730921953e-03f, -5.1193953533e-03f, -2.0445738864e-03f,
    1.6188568247e-03f,  5.5495631047e-03f,  9.3616850876e-03f,
    1.2638230181e-02f,  1.4970935984e-02f,  1.6003099899e-02f,
    1.5471302767e-02f,  1.3241762733e-02f,  9.3372856265e-03f,
    3.9514304325e-03f,  -2.5524599300e-03f, -9.6583852569e-03f,
    -1.6730490734e-02f, -2.3059524889e-02f, -2.7919509221e-02f,
    -3.0630015934e-02f, -3.0618678967e-02f, -2.7478272824e-02f,
    -2.1012930661e-02f, -1.1268834302e-02f, 1.4540613708e-03f,
    1.6611112007e-02f,  3.3442899240e-02f,  5.1024127609e-02f,
    6.8327899357e-02f,  8.4300300319e-02f,  9.7939200901e-02f,
    1.0837068546e-01f,  1.1491662541e-01f,  1.1714760405e-01f,
    1.1491662541e-01f,  1.0837068546e-01f,  9.7939200901e-02f,
    8.4300300319e-02f,  6.8327899357e-02f,  5.1024127609e-02f,
    3.3442899240e-02f,  1.6611112007e-02f,  1.4540613708e-03f,
    -1.1268834302e-02f, -2.1012930661e-02f, -2.7478272824e-02f,
    -3.0618678967e-02f, -3.0630015934e-02f, -2.7919509221e-02f,
    -2.3059524889e-02f, -1.6730490734e-02f, -9.6583852569e-03f,
    -2.5524599300e-03f, 3.9514304325e-03f,  9.3372856265e-03f,
    1.3241762733e-02f,  1.5471302767e-02f,  1.6003099899e-02f,
    1.4970935984e-02f,  1.2638230181e-02f,  9.3616850876e-03f,
    5.5495631047e-03f,  1.6188568247e-03f,  -2.0445738864e-03f,
    -5.1193953533e-03f, -7.3730921953e-03f, -8.6755943844e-03f,
    -9.0018236121e-03f, -8.4238523098e-03f, -7.0943306002e-03f,
    -5.2235646767e-03f, -3.0530669725e-03f, -8.2851970562e-04f,
    1.2250914212e-03f,  2.9238669111e-03f,  4.1392506133e-03f,
    4.8052673120e-03f,  4.9186525616e-03f,  4.5325296142e-03f,
    3.7449018789e-03f,  2.6836587108e-03f,  1.4900062402e-03f,
    3.0222754609e-04f,  -7.5853388690e-04f, -1.5991478655e-03f,
    -2.1611933058e-03f, -2.4231740164e-03f, -2.3984711082e-03f,
    -2.1296974068e-03f, -1.6804539189e-03f, -1.1256812833e-03f,
    -5.4184050926e-04f, 1.9446445974e-06f,  4.5087281640e-04f,
    7.6897244092e-04f,  9.4087342622e-04f,  9.7093040636e-04f,
    8.8008811483e-04f,  7.0110872784e-04f,
};

// RX filter: integrate-and-dump D(f) tabulated to 4800 Hz, 5 sps @ 24 kHz,
// span 9 symbols. TX*RX pair ISI at the symbol centres: worst 0.20%, rms
// 0.05% (bilateral, normalised to the centre tap).
static constexpr std::array<float, 45> rx_filter_24k = {
    4.0312432198e-04f,  -2.2037895551e-04f, -6.3724152930e-04f,
    -1.6166325335e-04f, 6.7865385656e-04f,  6.6618120420e-04f,
    -4.0420558050e-04f, -1.1383620250e-03f, -2.6115800979e-04f,
    1.3417428514e-03f,  1.3060668391e-03f,  -9.6070100493e-04f,
    -2.5975062956e-03f, -4.3705610688e-04f, 3.8328930923e-03f,
    3.6392152810e-03f,  -4.3930370271e-03f, -1.1280588508e-02f,
    2.8130577702e-03f,  5.2971011626e-02f,  1.3150365306e-01f,
    2.0542515300e-01f,  2.3582229080e-01f,  2.0542515300e-01f,
    1.3150365306e-01f,  5.2971011626e-02f,  2.8130577702e-03f,
    -1.1280588508e-02f, -4.3930370271e-03f, 3.6392152810e-03f,
    3.8328930923e-03f,  -4.3705610688e-04f, -2.5975062956e-03f,
    -9.6070100493e-04f, 1.3060668391e-03f,  1.3417428514e-03f,
    -2.6115800979e-04f, -1.1383620250e-03f, -4.0420558050e-04f,
    6.6618120420e-04f,  6.7865385656e-04f,  -1.6166325335e-04f,
    -6.3724152930e-04f, -2.2037895551e-04f, 4.0312432198e-04f,
};
/*
 * FIR implementations of the P25 modem filters.
 */
extern Fir<std::tuple_size<decltype(P25::tx_pulse_48k)>::value> pulse_48k;
extern Fir<std::tuple_size<decltype(P25::rx_filter_24k)>::value> rxf_24k;

} /* P25 */

#endif /* P25_MODEM_DSP_H */
