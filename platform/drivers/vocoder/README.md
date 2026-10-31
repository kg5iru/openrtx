<!--
SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors

SPDX-License-Identifier: GPL-3.0-or-later
-->

# AMBE+2 vocoder for the CS7000-PLUS (software, from your own firmware)

The CS7000-PLUS has no vocoder chip.  The HR_C6000 hands 72-bit half-rate
frames to the MCU over V_SPI and the MCU is expected to turn them into PCM
(and back).  Connect Systems' own firmware does that with an AMBE+2 library
running on the STM32H743.  OpenRTX cannot ship that code.  Instead:

* `scripts/cs7000_codec_extract.py` copies the codec out of **your own**
  `HOST_V9.00.96` firmware file, repoints its 80 absolute addresses and writes a
  relocated blob;
* `ambe_cs7000.c` is a small GPL wrapper (init / encode / decode) that calls
  into the blob at a fixed flash address (`AMBE_BLOB_BASE`, 0x08138000, right after the OpenRTX image inside the 512 kB slot the vendor updater writes).

The blob and any image containing it are Connect Systems' code.  Keep them for
your own radio.  Do not publish them, attach them to issues, or commit them.

## Build and flash

    meson setup build_cs7000p --cross-file cross_cm7.txt -Dcs7000_ambe=true
    meson compile -C build_cs7000p openrtx_cs7000p_bin     # flat image
    python3 scripts/cs7000_codec_extract.py HOST_V9.00.96.bin --out /tmp/ambe \
        --merge build_cs7000p/openrtx_cs7000p_bin --merged-out /tmp/openrtx_ambe.bin
    python3 scripts/cs7000_wrap.py /tmp/openrtx_ambe.bin openrtx_cs7000p_ambe.bin

Flash the wrapped file as usual.  `ambe_cs7000_available()` returns false when
the blob is missing, so builds without it still run.

## API

    ambe_cs7000_t enc, dec;              /* ~10.5 kB each; one per direction */
    ambe_cs7000_init(&enc);  ambe_cs7000_init(&dec);
    ambe_cs7000_encode(&enc, pcm160, bytes9);        /* 20 ms -> 72 bits */
    int err = ambe_cs7000_decode(&dec, bytes9, pcm160);

Peak stack use measured in emulation: 4.2 kB (give the calling thread 5 kB or more).  Not reentrant per context.

## What has and has not been checked

Checked (in a Cortex-M7 emulator, with the stock image absent and the rest of
flash filled with 0x00 or 0xFF): the relocated blob plus this wrapper reproduce
the stock firmware's codec bit-for-bit on 286 operations (24-frame encode
sessions of tones, noise, clipping, silence, impulses; 180 decode frames of
crafted, random and bit-flipped frames), at different load addresses and
RAM locations, and for -O0..-O3/-Oz, hard and soft float.

NOT checked on hardware.  Open items:

* Only the half-rate (72-bit) mode used by DMR was exercised.  Tables used only
  by other modes are copied but were never read in the test corpus.
* The licence chip (ALPU-MP, I2C1 0x7A) is polled by the stock boot code.  The
  codec functions themselves never touch it in any test, but whether the real
  firmware depends on that handshake having happened is unknown.
* The decoder's interrupt-driven calling pattern (10 ms I2S ping-pong buffers,
  EXTI line mapping) is not implemented yet; see dmr_CS7000.cpp.
* Re-running `ambe_cs7000_init()` zeroes the whole state.  The stock firmware
  does not, but the outputs were identical in the corpus.
