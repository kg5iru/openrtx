#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
# SPDX-License-Identifier: GPL-3.0-or-later
"""Generate/check o25 FIR tables against the OP25 C4FM reference model.

Reference: OP25 gr-op25_repeater/apps/tx/op25_c4fm_mod.py,
transfer_function_tx/rx and c4fm_taps.generate. TX = RC(beta=.2)/sinc(f/Rs),
RX = sinc(f/Rs); their ideal product is RC. This is discriminator-domain
validation, not a claim about RF emission-mask compliance.
Requires NumPy. --check checks committed tables; --write regenerates them.
"""
import argparse
from pathlib import Path
import re
import numpy as np

HEADER = Path(__file__).resolve().parents[1] / 'openrtx/include/protocols/P25/modem/DSP.hpp'


def taps(rate, tx):
    f = np.arange(2881 if tx else 4800, dtype=float)
    d = np.sinc(f / 4800)
    if tx:
        h = np.where(f < 1920, 1., .5 + .5 * np.cos(2 * np.pi * f / 1920))
        response = h / d
    else:
        response = d
    impulse = np.fft.fftshift(np.fft.irfft(response, rate))
    n = (rate // 4800 * (13 if tx else 9)) | 1
    start = int(np.argmax(impulse)) - (n - 1) // 2
    result = impulse[start:start + n].copy()
    return result / result.sum()


def metrics(tx, rx):
    # Lift the RX impulse to the 48 kHz grid, preserving the exact centre.
    up = np.zeros(2 * len(rx) - 1)
    up[::2] = rx
    pair = np.convolve(tx, up)
    centre = (len(pair) - 1) // 2
    indices = np.arange(centre % 10, len(pair), 10)
    residual = pair[indices[indices != centre]] / pair[centre]
    worst = np.abs(residual).sum()
    rms = np.sqrt(np.sum(residual ** 2) * 5 / 9)
    peak = 3 * max(np.abs(tx[p::10]).sum() for p in range(10))
    return worst, rms, peak


def array(name, values):
    rows = [f'static constexpr std::array<float, {len(values)}> {name} = {{']
    for i in range(0, len(values), 4):
        rows.append('    ' + ', '.join(f'{x:.10e}f' for x in values[i:i+4]) + ',')
    return '\n'.join(rows) + '\n};'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--write', action='store_true')
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    tx, rx = taps(48000, True), taps(24000, False)
    if args.write:
        text = HEADER.read_text()
        for name, values in [('tx_pulse_48k', tx), ('rx_filter_24k', rx)]:
            text, n = re.subn(r'static constexpr std::array<float, \d+> ' + name + r' = \{.*?\n\};', array(name, values), text, flags=re.S)
            assert n == 1
        HEADER.write_text(text)
    text = HEADER.read_text()
    for name, expected in [('tx_pulse_48k', tx), ('rx_filter_24k', rx)]:
        match = re.search(name + r' = \{(.*?)\};', text, re.S)
        assert match is not None, name
        body = match.group(1)
        actual = np.array([float(s.strip().rstrip('f')) for s in body.split(',') if s.strip()])
        assert actual.shape == expected.shape, (name, actual.shape, expected.shape)
        np.testing.assert_allclose(actual, expected, rtol=1e-7, atol=1e-10)
    worst, rms, peak = metrics(tx, rx)
    assert worst < .003 and rms < .001
    print(f'OP25-model tables verified: TX {len(tx)} / RX {len(rx)} taps')
    print(f'Bilateral normalized ISI: worst {100*worst:.5f}% RMS {100*rms:.5f}%')
    # Modulator scaling: a steady +3 run gives 3 * gain * sum(taps) / 10 = outer
    # level; `peak` is 3 * max polyphase |tap| sum, i.e. the worst-case sample
    # for unit gain, so peak * gain is the worst-case sample in counts.
    outer = 16000.0  # spec deviation on the CS7000P path, measured on air
    gain = outer * 10 / 3
    print(f'At outer level {outer:.0f}: worst-case symbol-sequence peak '
          f'{peak*gain:.0f} counts ({peak*gain/outer:.2f}x outer, before PWM compensation)')


if __name__ == '__main__':
    main()
