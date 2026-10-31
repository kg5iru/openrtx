#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
#
# SPDX-License-Identifier: GPL-3.0-or-later
"""
cs7000_codec_extract.py - relocate the AMBE+2 half-rate codec out of YOUR OWN
Connect Systems CS7000 firmware image so OpenRTX can call it.

The proprietary codec is never shipped with OpenRTX.  This tool runs on the
user's machine, reads the user's firmware file (the wrapped .bin from Connect
Systems, or an already-unwrapped image), checks that it is the exact release
the offsets were derived from (HOST V9.00.96), copies two address ranges out
of it, repoints the 80 absolute addresses the codec code contains, and writes:

    ambe_cs7000.bin          relocated blob, load it at --base in flash
    ambe_cs7000_layout.h     entry-point addresses for the C wrapper
    ambe_cs7000_manifest.json  what moved where (for audit / the test)

usage: cs7000_codec_extract.py HOST_V9.00.96.bin --out DIR
       cs7000_codec_extract.py HOST_V9.00.96.bin --out DIR \\
               --merge build/openrtx_cs7000p.bin --merged-out openrtx_ambe.bin

--merge appends the blob to an OpenRTX image (padding with 0xFF up to --base),
giving one flat image you can then wrap with scripts/cs7000_wrap.py.

THE BLOB AND ANY IMAGE CONTAINING IT ARE CONNECT SYSTEMS' CODE.  Keep them
for your own radio; do not publish or attach them to issues/PRs.

Why a plain slice copy: the codec code is position dependent only through
literal-pool words and movw/movt pairs.  Branches between functions are
PC-relative, so moving whole ranges by one constant keeps them valid.
"""
import argparse, hashlib, json, os, struct, sys

TOOL_VERSION = 1
STOCK_BASE   = 0x08020000                 # link address of the stock host image
KNOWN_SHA256 = {                          # sha256 of the UNWRAPPED image
    '9bcd6e63d85cea5c7dda9f41466902bd54d097e5b3ecdbeacd8f49dbf91573d1': 'HOST V9.00.96',
}
# (name, stock start, stock end).  'D' contains the codec's lookup tables and
# the two function-pointer helpers at 0x080F1768..0x080F1906.
PIECES = [
    ('code', 0x080A4F24, 0x080BABC0),
    ('data', 0x080C4600, 0x080F5A00),
]
# Absolute addresses embedded in the codec code, found by decoding every
# function in the dependency closure (ldr rX,[pc,#n] literals, movw/movt pairs).
WORD_FIXUPS = [
    (0x080A506C,0x080ED098), (0x080A54E4,0x080F5994), (0x080A54E8,0x080F4672),
    (0x080A596C,0x080F148C), (0x080A59FC,0x080F4680), (0x080A5A00,0x080F468C),
    (0x080A5BC0,0x080F4672), (0x080A5F18,0x080F148C), (0x080A5F1C,0x080F5994),
    (0x080AC6C8,0x080F21DC), (0x080ACB0C,0x080ED160), (0x080AD0A4,0x080F148C),
    (0x080AD36C,0x080DF228), (0x080ADBFC,0x080D9B09), (0x080ADF20,0x080F5994),
    (0x080AE1EC,0x080F5994), (0x080AE300,0x080F196C), (0x080AE3CC,0x080F3638),
    (0x080AE5E0,0x080F19B4), (0x080AE5E4,0x080F1924), (0x080AE7D4,0x080F4B5C),
    (0x080AE7D8,0x080F148C), (0x080AF53C,0x080D1D50), (0x080AF604,0x080D1D44),
    (0x080AF6F4,0x080D1D3C), (0x080B1A10,0x080F0974), (0x080B1A14,0x080F0A58),
    (0x080B1C24,0x080D0D28), (0x080B1CE4,0x080F59AC), (0x080B1E58,0x080D0D28),
    (0x080B4FEC,0x080DF278), (0x080B57EC,0x080F2834), (0x080B57F0,0x080F2844),
    (0x080B57F4,0x080F2838), (0x080B57F8,0x080F281C), (0x080B57FC,0x080F282C),
    (0x080B5800,0x080F2820), (0x080B5804,0x080F2814), (0x080B5808,0x080F2840),
    (0x080B580C,0x080F2828), (0x080B5BE4,0x080F2814), (0x080B6464,0x080E9ECC),
    (0x080B6C48,0x080CFC20), (0x080B6C4C,0x080CF020), (0x080B6C50,0x080CEC20),
    (0x080B6C54,0x080CEAA0), (0x080B6C58,0x080CEA20), (0x080B6C5C,0x080CEB20),
    (0x080B6C60,0x080CE9E0), (0x080B7218,0x080E9ECC), (0x080B7804,0x080F148C),
    (0x080B79A0,0x080F148C), (0x080B7C9C,0x080F1E84), (0x080B810C,0x080D1928),
    (0x080B8298,0x080D1928), (0x080B8E10,0x080F4E04), (0x080B9270,0x080D1928),
    (0x080B93DC,0x080E0770), (0x080B94B4,0x080F2218), (0x080B9570,0x080D1D3C),
    (0x080B98FC,0x080D1D50), (0x080B99A4,0x080D1928), (0x080B9AD4,0x080D1D2C),
    (0x080BA400,0x080F599C), (0x080BA644,0x080C469C), (0x080BA648,0x080D1928),
    (0x080BAA38,0x080C469C), (0x080BAA3C,0x080D1928), (0x080BAB3C,0x080F2B64),
    (0x080F17DC,0x080F148C),
]
MOVWT_FIXUPS = [   # (addr of movw, addr of movt, combined value)
    (0x080A5C48,0x080A5C4C,0x080AF849), (0x080A5C52,0x080A5C5A,0x080F17E1),
    (0x080A5CF4,0x080A5CF8,0x080AF849), (0x080A5CFE,0x080A5D02,0x080F17E1),
    (0x080B2606,0x080B2612,0x080F2224), (0x080B5554,0x080B555C,0x080F284C),
    (0x080B55FA,0x080B5602,0x080F284C), (0x080B9EF8,0x080B9EFC,0x080D1928),
    (0x080B9F12,0x080B9F18,0x080D1928), (0x080B9F46,0x080B9F4E,0x080D1928),
]
ENTRY_POINTS = {   # name: stock address (thumb function)
    'enc_half':   0x080A5070,   # one 10 ms analysis/quantise pass (call twice per frame)
    'enc_fec':    0x080A5438,   # 49 -> 72 bit FEC expansion
    'dec_fec':    0x080A5A04,   # FEC decode / unpack
    'dec_params': 0x080A5F20,
    'dec_prep':   0x080A54EC,
    'synth':      0x080A5528,   # 80-sample synthesis (also used by init)
    'init_a':     0x080A51B8,
    'init_b':     0x080A5420,
    'init_c':     0x080A5614,
    'init_d':     0x080A5778,
    'init_e':     0x080A5104,
    'init_f':     0x080A55D0,
}

def unwrap(d):
    """Undo the Connect Systems wrapper (see cs7000_unwrap.py); returns the image."""
    KEYLEN, HDR = 256, 0x80
    pay = d[HDR:]
    N = len(pay)
    n = (N - 10) // 2 - 4
    P = 4 + n
    if N != 2 * P + 10:
        sys.exit('not a wrapped CS7000 image (payload size does not fit)')
    lw = struct.pack('<I', n)
    key = [None] * KEYLEN
    for j in range(4):
        key[j] = pay[j] ^ lw[j]
        key[(P + j) % KEYLEN] = pay[P + j] ^ lw[j]
    shift = P % KEYLEN
    rel = {}
    for j in range(KEYLEN):
        vals = {pay[i] ^ pay[i + P] for i in range(j, P, KEYLEN)}
        if len(vals) != 1:
            sys.exit('wrapper structure check failed')
        rel[j] = vals.pop()
    for _ in range(KEYLEN):
        for j in range(KEYLEN):
            t = (j + shift) % KEYLEN
            if key[j] is not None and key[t] is None: key[t] = key[j] ^ rel[j]
            if key[t] is not None and key[j] is None: key[j] = key[t] ^ rel[j]
    if None in key:
        sys.exit('could not recover the wrapper key')
    plain = bytes(b ^ key[i % KEYLEN] for i, b in enumerate(pay))
    return plain[4:4 + n]

def layout(base):
    """Assign a new address to each piece, keeping every piece's address mod 8."""
    out, cur = [], base
    for name, s, e in PIECES:
        cur += (s - cur) % 8
        out.append((name, s, e, cur))
        cur += e - s
    return out, cur - base

def remap(v, lay):
    """Map a stock address (thumb bit preserved) into the blob, or None."""
    a = v & ~1
    for _, s, e, n in lay:
        if s <= a < e:
            return v - s + n
    for _, s, e, n in lay:            # one-past-the-end pointers
        if a == e:
            return v - s + n
    return None

def set_imm16(buf, off, imm, is_movt):
    h1, h2 = struct.unpack_from('<HH', buf, off)
    want = 0xF2C0 if is_movt else 0xF240
    if (h1 & 0xFBF0) != want or (h2 & 0x8000):
        sys.exit('expected %s at blob offset 0x%x' % ('movt' if is_movt else 'movw', off))
    imm4, i, imm3, imm8 = (imm >> 12) & 0xF, (imm >> 11) & 1, (imm >> 8) & 7, imm & 0xFF
    h1 = (h1 & 0xFBF0) | (i << 10) | imm4
    h2 = (h2 & 0x8F00) | (imm3 << 12) | imm8
    struct.pack_into('<HH', buf, off, h1, h2)

def get_imm16(buf, off):
    h1, h2 = struct.unpack_from('<HH', buf, off)
    return (((h1 & 0xF) << 12) | (((h1 >> 10) & 1) << 11) | (((h2 >> 12) & 7) << 8) | (h2 & 0xFF))

def build(img, base):
    lay, size = layout(base)
    blob = bytearray(b'\xff' * size)
    def off(addr):                       # stock address -> blob offset
        for _, s, e, n in lay:
            if s <= addr < e:
                return addr - s + n - base
        sys.exit('address 0x%08x is outside the copied ranges' % addr)
    for name, s, e, n in lay:
        blob[n - base:n - base + (e - s)] = img[s - STOCK_BASE:e - STOCK_BASE]
    nfix = 0
    for a, v in WORD_FIXUPS:
        o = off(a)
        got = struct.unpack_from('<I', blob, o)[0]
        if got != v:
            sys.exit('fixup check failed at 0x%08x: image holds 0x%08x, expected 0x%08x' % (a, got, v))
        nv = remap(v, lay)
        if nv is None:
            sys.exit('pointer 0x%08x at 0x%08x is not inside a copied range' % (v, a))
        struct.pack_into('<I', blob, o, nv); nfix += 1
    for lo, hi, v in MOVWT_FIXUPS:
        o1, o2 = off(lo), off(hi)
        got = (get_imm16(blob, o2) << 16) | get_imm16(blob, o1)
        if got != v:
            sys.exit('movw/movt check failed at 0x%08x' % lo)
        nv = remap(v, lay)
        if nv is None:
            sys.exit('movw/movt pointer 0x%08x at 0x%08x is not inside a copied range' % (v, lo))
        set_imm16(blob, o1, nv & 0xFFFF, False)
        set_imm16(blob, o2, nv >> 16, True)
        nfix += 1
    entries = {k: remap(v, lay) for k, v in ENTRY_POINTS.items()}
    return bytes(blob), lay, entries, nfix

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('firmware')
    ap.add_argument('--base', default='0x08138800', help='flash address of the blob (multiple of 8); must match ambe_cs7000_layout.h')
    ap.add_argument('--merge', help='OpenRTX flat image (.bin) to append the blob to')
    ap.add_argument('--flash-origin', default='0x08100000', help='flash address the --merge image starts at')
    ap.add_argument('--merged-out', help='output file for --merge')
    ap.add_argument('--out', default='.')
    a = ap.parse_args()
    base = int(a.base, 0)
    if base % 8: sys.exit('--base must be a multiple of 8')
    raw = open(a.firmware, 'rb').read()
    img = raw
    sha = hashlib.sha256(img).hexdigest()
    if sha not in KNOWN_SHA256:
        img = unwrap(raw)
        sha = hashlib.sha256(img).hexdigest()
    if sha not in KNOWN_SHA256:
        sys.exit('unrecognised firmware (sha256 of image %s).\nThis tool only knows: %s' % (sha, ', '.join(KNOWN_SHA256.values())))
    print('firmware    : %s' % KNOWN_SHA256[sha])
    blob, lay, entries, nfix = build(img, base)
    os.makedirs(a.out, exist_ok=True)
    open(os.path.join(a.out, 'ambe_cs7000.bin'), 'wb').write(blob)
    with open(os.path.join(a.out, 'ambe_cs7000_layout.h'), 'w') as f:
        f.write('/* generated by cs7000_codec_extract.py - do not edit */\n#pragma once\n')
        f.write('#define AMBE_BLOB_BASE 0x%08XU\n#define AMBE_BLOB_SIZE 0x%XU\n' % (base, len(blob)))
        for k, v in sorted(entries.items()):
            f.write('#define AMBE_ADDR_%s 0x%08XU\n' % (k.upper(), v | 1))
    # Per-kilobyte byte sums of the blob, so the firmware can show on screen
    # which parts of the codec region do not match after flashing.  Sums only,
    # no Connect Systems code.
    with open(os.path.join(a.out, 'ambe_cs7000_ksums.h'), 'w') as f:
        ks = [sum(blob[i:i + 1024]) & 0xFFFF for i in range(0, len(blob), 1024)]
        f.write('/* generated by cs7000_codec_extract.py - do not edit */\n#pragma once\n')
        f.write('#define AMBE_BLOB_KSUMS_N %d\n' % len(ks))
        f.write('static const unsigned short ambe_blob_ksums[AMBE_BLOB_KSUMS_N] = {\n')
        for i in range(0, len(ks), 12):
            f.write('    ' + ', '.join('0x%04X' % v for v in ks[i:i + 12]) + ',\n')
        f.write('};\n')
    man = {'tool': TOOL_VERSION, 'source': KNOWN_SHA256[sha], 'source_sha256': sha,
           'base': base, 'size': len(blob), 'blob_sha256': hashlib.sha256(blob).hexdigest(),
           'pieces': [{'name': n, 'stock': [s, e], 'new': nn} for n, s, e, nn in lay],
           'entries': {k: v | 1 for k, v in entries.items()}, 'fixups': nfix}
    json.dump(man, open(os.path.join(a.out, 'ambe_cs7000_manifest.json'), 'w'), indent=1)
    print('blob        : %d bytes at 0x%08x, %d pointers repointed' % (len(blob), base, nfix))
    print('blob sha256 : %s' % man['blob_sha256'])
    if a.merge:
        if not a.merged_out: sys.exit('--merge needs --merged-out')
        org = int(a.flash_origin, 0)
        rt = open(a.merge, 'rb').read()
        sp, rv = struct.unpack_from('<II', rt, 0)
        if not (0x20000000 <= sp < 0x30000000 and org <= rv < org + len(rt)):
            sys.exit('--merge wants the flat (unwrapped) OpenRTX .bin: its vector table looks wrong')
        if org + len(rt) > base:
            sys.exit('OpenRTX image (0x%x bytes) runs into the codec blob at 0x%08x' % (len(rt), base))
        # The gap between OpenRTX and the blob is filled with zeros, not 0xFF:
        # on the radio the first 32-byte flash word at 0x08138000 read back
        # as zeros after flashing with the vendor updater, right after an
        # all-0xFF gap, so the blob also starts 2 kB later now (0x08138800).
        open(a.merged_out, 'wb').write(rt + b'\x00' * (base - org - len(rt)) + blob)
        print('merged image: %s (%d bytes)' % (a.merged_out, base - org + len(blob)))

if __name__ == '__main__':
    main()
