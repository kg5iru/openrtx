# DMR support status

This document tracks what the `DMR` protocol module (`openrtx/include/protocols/DMR`,
`openrtx/src/protocols/DMR`) actually implements, what it deliberately
does not, and why -- so nobody mistakes "there is a DMR directory now"
for "this radio talks DMR."

## Scope decision

DMR voice uses the patented AMBE+2 vocoder. This module contains **no
software AMBE implementation** and never will as part of this effort.
The CS7000 M17 Plus's baseband chip (HR_C6000, shared with the
CS7000, GD-77 and MD-UV3x0 targets) has its own onboard/bridged
AMBE3000/AMBE1000 vocoder, selected via dedicated `AMBE3K`/`AMBE1K` SPI
operating modes (`platform/drivers/baseband/HR_C6000.h`). So "DMR
encoder/decoder" here means the open, ETSI-standard air-interface
protocol/framing layer (TDMA burst structure, FEC, CRC, Slot Type,
embedded signalling), plus driving that existing hardware vocoder --
not reimplementing voice coding.

## What is implemented (and cross-validated)

All of the following were cross-checked bit-for-bit against an
independent reference implementation (`okdmr/dmrlib`, AGPL-3.0, used
only as a correctness oracle -- never copied; see "Licensing" below)
over randomised and in several cases exhaustive test vectors, and are
covered by Catch2 unit tests in `tests/unit/DMR_*.cpp`:

- **Golay (20,8,7)** -- `Golay2087.hpp`. Exhaustive: all 256 possible
  8-bit inputs encode correctly; every single-bit error in all 20
  codeword positions is corrected.
- **Quadratic Residue (16,7,6)** -- `QR1676.hpp`. Exhaustive over all
  128 inputs and all 16 single-bit error positions.
- **Hamming (15,11,3)** and **Hamming (13,9,3)** -- `Hamming.hpp`,
  the row/column codes of BPTC(196,96). Exhaustive over all inputs and
  all single-bit error positions.
- **BPTC(196,96)** -- `Bptc19696.hpp`/`.cpp`. The full 196-bit
  interleave table from ETSI TS 102 361-1 Annex B.1.1, plus
  encode/decode-with-correction. Verified against 100 random 96-bit
  payloads (exact match) and, for a subset, against all 196
  single-bit-error positions (1960/1960 corrected).
- **CRC7/8/9/16/32** -- `Crc.hpp`. Generic bit-serial CRC parameterised
  by width/polynomial; CRC16 on-air value (the complement+mask form
  actually used by CSBK/Data Header/MBC Header/USBD) checked against
  200 random vectors.
- **Reed-Solomon (12,9,4)** over GF(256) -- `ReedSolomon1294.hpp`/`.cpp`,
  used by Full LC / Terminator-with-LC (this is *not* a CRC, see
  ETSI Annex B.3.6). Checked against 50 random vectors.
- **Slot Type** (`SlotType.hpp`), **EMB** (`Emb.hpp`): the
  Golay/QR-protected fields, with round-trip tests over every valid
  field combination.
- **Full Link Control** envelope (`FullLinkControl.hpp`): protect
  flag / FLCO / Feature Set ID / 7-byte payload, RS(12,9,4) validated,
  mask-checked (wrong mask is correctly rejected).
- **Generic single-block PDU envelope** (`SingleBlockPdu.hpp`): the
  80-bit-content + masked-CRC16 shape shared by CSBK, Data Header, MBC
  Header and Unified Single Block Data.
- **Burst framing helper** (`Burst.hpp`): splitting a 264-bit on-air
  burst into Info1(98)/middle(48)/Info2(98), and classifying the
  middle 48 bits against the four ETSI sync patterns
  (`Constants.hpp`, cross-checked against MMDVM's `DMRDefines.h`).

Also: the CRC/RS XOR masks used throughout (`CrcMask::*` in
`Crc.hpp`) independently match the literal byte values CS7000's own
firmware programs into the HR_C6000 chip's hardware FEC engine during
init (`HR_C6000_CS7000.cpp`, `initSeq4`: `69 69 96 96 96 99 99 99 A5
A5 AA AA CC CC`) -- a nice confirmation from the radio's own firmware
that these constants, and the fact that "DMR mode" on this chip really
is the standard ETSI air interface, are right.

## What is explicitly NOT implemented

- **CSBK opcode semantics.** `SingleBlockPdu` validates/builds the
  envelope but does not interpret the 80 bits of content (CSBK has
  dozens of opcodes: polling, lateral/outbound channel grants, each
  with its own payload layout). Needed for anything beyond
  simplex/basic conventional operation.
- **Data Header fields**, similarly left as opaque content.
- **Embedded LC reassembly.** A voice superframe's Full LC is split
  into four 32-bit fragments carried in bursts B-E's embedded
  signalling, reassembled using the EMB field's LCSS bits and a
  Hamming(16,11,4) per fragment (not yet transcribed -- only
  (15,11,3) and (13,9,3), used by BPTC, are in `Hamming.hpp`).
  `Emb.hpp` decodes the EMB field itself but there is no superframe
  state machine yet.
- **CACH**, trunking signalling, and any call-setup/state machine.
- **CRC32** (Rate 1/2/3/4 data continuation blocks) and **CRC9** (PI
  Header) are implemented as raw primitives in `Crc.hpp` but have no
  on-air wrapper or PDU wired up yet -- nothing in scope currently
  needs them.
- **`HR_Cx000::dmrMode()` for CS7000** (`HR_C6000_CS7000.cpp`) has
  been ported from the MD-UV3x0 target's implementation (the only
  other HR_C6000 target with a non-stub version -- itself never
  called anywhere in the tree) and is annotated as unverified. It is
  **not called from anywhere**, deliberately: there was no physical
  CS7000 M17 Plus available to validate register-level behaviour
  against, and blindly wiring an unverified init sequence into the
  live radio/PTT path risks bricking behaviour on real hardware.
- **No wiring into `radio_CS7000.cpp`** (burst TX/RX over the SPI
  DATA/DATA_R channel) or into the `rtx`/`OpMode_*` operational layer.
  The M17 equivalent of that wiring (`OpMode_M17.cpp`) is a
  non-trivial real-time state machine; doing it for DMR without
  hardware to test against would mean shipping an untested TX/RX path
  on a live radio, which is out of scope here.

## Build / test notes

The four Catch2 suites (`tests/unit/DMR_fec.cpp`, `DMR_bptc.cpp`,
`DMR_crc.cpp`, `DMR_frames.cpp`) are wired into `meson.build`
(`dmr_fec_test`, `dmr_bptc_test`, `dmr_crc_test`, `dmr_frames_test`,
registered with `test()`) the same way the existing `M17_*` suites
are. In this environment, `meson compile` successfully compiles every
new DMR source and test file against the real build (object files
link-ready); full executable linking for the whole test suite is
blocked by a **pre-existing, unrelated** issue in this shallow clone's
vendored `codec2` subproject (missing generated codebook sources,
e.g. `lsp_cbjvm`/`ge_cb`/`mbest_*`), not by anything in this change --
`openrtx_linux` and the existing `m17_*_test` targets hit the same
link failure. The logic itself was additionally verified standalone
(plain `g++ -std=c++14`, no meson) against the `okdmr/dmrlib`
reference implementation with the exhaustive/randomised vectors
described above, all passing.

## Licensing notes

- All FEC/CRC generator matrices, interleave tables, GF(256) tables
  and sync-pattern constants are the ETSI standard's own mandated
  data (ETSI TS 102 361-1), reproduced as plain numeric data -- not
  creative expression unique to any implementation.
- `okdmr/dmrlib` (AGPL-3.0) was used only as a correctness oracle to
  generate cross-check test vectors (via its Python CLI/library, kept
  entirely in `/home/claude/ref`, outside this tree) -- no code from
  it was copied into this module.
- `g4klx/MMDVM`'s `DMRDefines.h` (GPLv2+) and `f4exb/dsdcc`'s
  `fec.h`/`fec.cpp` (GPLv3) were likewise used only to cross-reference
  the standard's own constants (sync patterns, FEC matrices); the
  encode/decode algorithms here (`LinearBlockCode.hpp`'s generic
  syndrome decoder, the BPTC traversal, the RS systematic encoder) are
  original, not ports of either project's code.
- Everything new is licensed `GPL-3.0-or-later`, matching the rest of
  OpenRTX.
