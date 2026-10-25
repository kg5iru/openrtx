/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "ambe_cs7000.h"
#include "ambe_cs7000_layout.h"
#include <string.h>

/* Field offsets inside the codec state block, from the stock init routine
 * at 0x08085E7E (see README.md next to this file).  R4 = parameter block,
 * X = decoder control block.                                              */
#define R4_OFF 0x172c
#define X_OFF 0x23b8
#define OUT_OFF 0x189c  /* 72 one-bit halfwords: encoder output     */
#define BITS_OFF 0x25bc /* 72 one-bit halfwords: decoder input      */
#define PAR_OFF 0x2280  /* 3 x 0x34-byte decoded-parameter slots    */
#define MASK_OFF 0x26dc /* 0x62-byte FEC mask (zero in half-rate)   */

typedef int16_t
    __attribute__((may_alias)) s16a; /* state block is a byte array */

typedef int32_t (*fn_enc_half)(s16a *out, int32_t a1, const int16_t *pcm,
                               int32_t n, int32_t s0, int32_t half, int32_t s2,
                               void *state);
typedef int32_t (*fn_enc_fec)(s16a *out, s16a *in, int32_t a2, void *blk);
typedef int32_t (*fn_dec_fec)(s16a *bits, int32_t a1, s16a *bits2, int32_t one,
                              int32_t s0, void *blk);
typedef void (*fn_dec_par)(void *par, void *blk);
typedef void (*fn_dec_prep)(void *blk, void *par);
typedef void (*fn_synth)(int16_t *out, int32_t n, s16a *bits, int32_t a3,
                         int32_t s0, int32_t flag, void *blk);
typedef void (*fn_init_a)(void *st, int32_t a, int32_t b);
typedef void (*fn_init_b)(void *blk, int32_t a);
typedef void (*fn_init_d)(void *st, int32_t a, int32_t b, int32_t c, int32_t s0,
                          int32_t s1, int32_t s2, int32_t s3, int32_t s4,
                          int32_t s5, int32_t s6);
typedef void (*fn_init_e)(void *st, int32_t a, int32_t b);

#define FN(T, name) ((T)(uintptr_t)AMBE_ADDR_##name)

static inline int16_t rs(const uint8_t *p, unsigned o)
{
    int16_t v;
    memcpy(&v, p + o, 2);
    return v;
}
static inline uint16_t ru(const uint8_t *p, unsigned o)
{
    uint16_t v;
    memcpy(&v, p + o, 2);
    return v;
}
static inline void ws(uint8_t *p, unsigned o, int v)
{
    int16_t x = (int16_t)v;
    memcpy(p + o, &x, 2);
}
static inline void wz16(uint8_t *p, unsigned o)
{
    memset(p + o, 0, 2);
}
static inline void wz32(uint8_t *p, unsigned o)
{
    memset(p + o, 0, 4);
}

bool ambe_cs7000_available(void)
{
    /* the first function in the blob starts with a push; an erased slot reads 0xFFFF */
    return *(const volatile uint16_t *)(AMBE_ADDR_ENC_HALF & ~1U) != 0xFFFF;
}

void ambe_cs7000_init(ambe_cs7000_t *c)
{
    uint8_t *st = c->state;
    uint8_t *R4 = st + R4_OFF, *X = st + X_OFF;
    memset(c, 0, sizeof *c);

    ws(R4, 0x04, 0x1840);
    ws(R4, 0x12, 0x2000);
    ws(R4, 0x18, -0x16);
    ws(R4, 0x1a, 0x14);
    ws(R4, 0x1c, -10);
    ws(R4, 0x1e, -0x28);
    ws(R4, 0x20, 0x4000);
    ws(R4, 0x22, 0xccc);
    ws(R4, 0x24, 0xccc);
    ws(R4, 0x26, 0x51e);
    ws(R4, 0x28, 10);
    ws(R4, 0x2a, 0x14);
    ws(R4, 0x14, 0x80);
    ws(R4, 0x16, -0x14);
    ws(X, 0x02, 0);
    ws(X, 0x1a, 0x80);
    ws(X, 0x1c, -0x14);

    FN(fn_init_a, INIT_A)(st, 1, 1);
    FN(fn_init_b, INIT_B)(st + 0x16e4, 1);
    FN(fn_init_b, INIT_C)(st + 0x1a40, 1); /* same shape: (block, 1) */
    FN(fn_init_b, INIT_B)(st + 0x2238, 1);
    FN(fn_init_d, INIT_D)
    (st, rs(R4, 0x18), rs(R4, 0x1a), rs(R4, 0x1c), rs(R4, 0x1e), rs(R4, 0x20),
     rs(R4, 0x22), rs(R4, 0x24), rs(R4, 0x26), rs(R4, 0x28), rs(R4, 0x2a));
    FN(fn_init_e, INIT_E)(st, rs(R4, 0x14), rs(R4, 0x16));
    FN(fn_init_e, INIT_F)(st + 0x1a40, rs(X, 0x1a), rs(X, 0x1c));
    FN(fn_synth, SYNTH)
    ((int16_t *)(st + 0x2516), 0x50, (s16a *)(st + BITS_OFF), 0, 8, 0,
     st + 0x1a40);

    /* the stock init clears these after priming the synthesiser */
    R4[2] = 0;
    wz16(R4, 0xa);
    wz32(R4, 0xc);
    wz16(R4, 0x10);
    R4[1] = 0;
    R4[0] = 0;
    wz32(st, 0x1898);
    wz16(st, 0x1a3a);
    wz16(st, 0x1a3c);
    wz16(X, 0x12);
    wz32(X, 0x14);
    wz16(X, 0x18);
    X[1] = 0;
    X[0] = 0;
    memset(st + 0x25b6, 0, 6);
    memset(st + 0x275e, 0, 6);
    memset(st + 0x19bc, 0, 0x62);
    memset(st + MASK_OFF, 0, 0x62);
    X[0] = 1; /* flag byte the verified reference leaves set */
}

void ambe_cs7000_encode(ambe_cs7000_t *c, const int16_t pcm[160],
                        uint8_t out[9])
{
    uint8_t *st = c->state, *R4 = st + R4_OFF;
    s16a *bits = (s16a *)(st + OUT_OFF);
    for (int half = 0; half < 2; half++) {
        memcpy(c->pcm_in, pcm + 80 * half, 160);
        FN(fn_enc_half, ENC_HALF)
        (bits, rs(R4, 0xe), c->pcm_in, 80, 0x800, half, rs(R4, 0x12), st);
    }
    FN(fn_enc_fec, ENC_FEC)(bits, bits, rs(R4, 0x10), st + 0x16e4);
    memset(out, 0, 9); /* 72 bit halfwords -> 9 bytes, MSB first */
    for (int i = 0; i < 72; i++)
        if (bits[i] & 1)
            out[i >> 3] |= (uint8_t)(0x80 >> (i & 7));
}

int ambe_cs7000_decode(ambe_cs7000_t *c, const uint8_t in[9], int16_t pcm[160])
{
    uint8_t *st = c->state, *X = st + X_OFF;
    s16a *bits = (s16a *)(st + BITS_OFF);
    for (int i = 0; i < 72; i++)
        bits[i] = (in[i >> 3] >> (7 - (i & 7))) & 1;
    memcpy(X + 4, X + 2, 2);
    int32_t err = FN(fn_dec_fec, DEC_FEC)(bits, rs(X, 0x18), bits, 1, rs(X, 4),
                                          st + 0x2238)
                & 0xffff;
    uint8_t *par = st + PAR_OFF + 0x34 * c->rx_slot;
    FN(fn_dec_par, DEC_PARAMS)(par, st + 0x2238);
    FN(fn_dec_prep, DEC_PREP)(st + 0x1a40, par);
    for (int i = 0; i < 0x31; i++) { /* FEC mask (all zero in this mode) */
        uint16_t v = ru((uint8_t *)bits, 2 * i) ^ ru(st, MASK_OFF + 2 * i);
        memcpy((uint8_t *)bits + 2 * i, &v, 2);
    }
    int32_t stk0 = (int16_t)(ru(X, 4) | err);
    for (int flag = 0; flag < 2; flag++) {
        FN(fn_synth, SYNTH)
        (c->pcm_out, 0x50, bits, rs(X, 0x16), stk0, flag, st + 0x1a40);
        memcpy(pcm + 80 * flag, c->pcm_out, 160);
    }
    c->rx_slot = (uint8_t)((c->rx_slot + 1) % 3);
    return err;
}
