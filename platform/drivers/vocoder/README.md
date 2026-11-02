<!--
SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors

SPDX-License-Identifier: GPL-3.0-or-later
-->

# AMBE+2 vocoder for the CS7000-PLUS (software, from your own firmware)

The CS7000-PLUS has no vocoder chip.  The HR_C6000 hands 72-bit half-rate
frames to the MCU over V_SPI and the MCU is expected to turn them into PCM
(and back).  Connect Systems' own firmware does that with an AMBE+2 library
running on the STM32H743.  OpenRTX cannot ship that code.  `ambe_cs7000.c` is
a small GPL wrapper (init / encode / decode) that calls into it, in one of two
ways.

## Recommended: call the codec where it is, in the stock image

The radio is dual-boot: the stock firmware stays flashed at 0x08020000 next
to OpenRTX.  With `-Dcs7000_ambe_stock=true` the wrapper calls the codec
inside it, at its stock addresses (`ambe_cs7000_stock.h`, generated with
`scripts/cs7000_codec_extract.py HOST_V9.00.96.bin --stock-header FILE`:
addresses and per-kilobyte sums only).  The OpenRTX image then holds no
Connect Systems code at all, has the whole 512 kB slot, and is flashed as is:

    meson setup build_cs7000p --cross-file cross_cm7.txt -Dcs7000_ambe=true \
        -Dcs7000_ambe_stock=true
    meson compile -C build_cs7000p openrtx_cs7000p_bin
    python3 scripts/cs7000_wrap.py build_cs7000p/openrtx_cs7000p_bin \
        openrtx_cs7000p_dmr.bin

The stock image on the radio must be **HOST V9.00.96**.  At the first DMR use
the firmware sums the codec's two ranges in the stock image (code
0x080A4F24..0x080BABC0, data 0x080C4600..0x080F5A00) against the header; on a
mismatch the DMR screen shows "need stock V9.00.96" and no audio is played
(nothing is called).  Checked in the Cortex-M7 emulator with the stock image
loaded at 0x08020000: decoding 600 captured on-air frames gives the same PCM,
bit for bit, as the relocated copy below, and the range check passes (and
catches a single changed byte).

## Alternative: a relocated copy merged into the OpenRTX image

* `scripts/cs7000_codec_extract.py` copies the codec out of **your own**
  `HOST_V9.00.96` firmware file, repoints its 80 absolute addresses and writes a
  relocated blob;
* the wrapper (built without `cs7000_ambe_stock`) calls into the blob at a fixed
  flash address (`AMBE_BLOB_BASE`, 0x08138800, 2 kB after the OpenRTX image
  inside the 512 kB slot the vendor updater writes).

The blob and any image containing it are Connect Systems' code.  Keep them for
your own radio.  Do not publish them, attach them to issues, or commit them.

    meson setup build_cs7000p --cross-file cross_cm7.txt -Dcs7000_ambe=true \
        -Dvoice_prompts=false
    meson compile -C build_cs7000p openrtx_cs7000p_bin     # flat image
    python3 scripts/cs7000_codec_extract.py HOST_V9.00.96.bin --out /tmp/ambe \
        --merge build_cs7000p/openrtx_cs7000p_bin --merged-out /tmp/openrtx_ambe.bin
    python3 scripts/cs7000_wrap.py /tmp/openrtx_ambe.bin openrtx_cs7000p_ambe.bin

`-Dvoice_prompts=false` leaves out the 68 kB of voice prompt audio: OpenRTX
must end where the blob starts (224 kB), and the image with the USB log does
not fit otherwise (the extractor refuses to merge an image that runs into the
blob).

Flash the wrapped file as usual.  `ambe_cs7000_available()` returns false when
the codec is missing, so builds without it still run.

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
