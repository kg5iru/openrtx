/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "protocols/P25/modem/Nid.hpp"
#include "hwconfig.h"

#ifdef CONFIG_P25

#include <cstring>

using namespace P25;

namespace
{

/*
 * BCH(63,16): GF(2^6) built on the primitive polynomial x^6 + x^5 + 1, the
 * generator polynomial has roots alpha^1 .. alpha^22 (t = 11). Codeword
 * polynomial c(x) = x^47 i(x) + b(x), with the 16 information bits as the
 * coefficients of degrees 47 .. 62 and the parity bits as degrees 0 .. 46.
 * In the transmitted 64-bit NID, bit n (MSB first) is: n < 16 the
 * coefficient of x^(47 + n); 16 <= n < 63 the coefficient of x^(n - 16);
 * n = 63 the parity bit.
 */
constexpr unsigned N = 63;
constexpr unsigned K = 16;
constexpr unsigned T = 11;
constexpr unsigned FIELD_POLY = 0x61; // x^6 + x^5 + 1

// g(x) coefficients, bit i = coefficient of x^i (degree 47), the LCM of the
// minimal polynomials of alpha^1 .. alpha^22 over the field above.
constexpr uint64_t GENERATOR = 0xD4DCBBD0C9B3ULL;

struct Field {
    uint8_t exp[126];
    uint8_t log[64];

    Field()
    {
        unsigned x = 1;
        for (unsigned i = 0; i < 63; i++) {
            exp[i] = exp[i + 63] = static_cast<uint8_t>(x);
            log[x] = static_cast<uint8_t>(i);
            x <<= 1;
            if (x & 0x40)
                x ^= FIELD_POLY;
        }
        log[0] = 0;
    }

    uint8_t mul(const uint8_t a, const uint8_t b) const
    {
        if ((a == 0) || (b == 0))
            return 0;
        return exp[log[a] + log[b]];
    }

    uint8_t inv(const uint8_t a) const
    {
        return exp[63 - log[a]];
    }
};

const Field &field()
{
    static const Field f;
    return f;
}

inline bool getBit(const uint8_t *data, const unsigned pos)
{
    return (data[pos >> 3] >> (7 - (pos & 7))) & 0x01;
}

inline void setBit(uint8_t *data, const unsigned pos, const bool value)
{
    uint8_t mask = 0x80 >> (pos & 7);
    if (value)
        data[pos >> 3] |= mask;
    else
        data[pos >> 3] &= ~mask;
}

// Position in the 64-bit NID of the codeword coefficient of x^degree.
inline unsigned bitOfDegree(const unsigned degree)
{
    return (degree >= 47) ? (degree - 47) : (degree + 16);
}

} // namespace

void Nid::encode(const uint16_t nac, const uint8_t duid, uint8_t *out)
{
    uint64_t info = (static_cast<uint64_t>(nac & 0x0FFF) << 4) | (duid & 0x0F);

    // Information word: bit n of the 16-bit word (MSB first) is NID bit n,
    // the coefficient of x^(47 + n).
    uint64_t poly = 0;
    for (unsigned n = 0; n < K; n++) {
        if ((info >> (15 - n)) & 1)
            poly |= 1ULL << (47 + n);
    }

    // Remainder of x^47 i(x) modulo g(x)
    uint64_t rem = poly;
    for (int degree = 62; degree >= 47; degree--) {
        if ((rem >> degree) & 1)
            rem ^= GENERATOR << (degree - 47);
    }

    uint64_t code = poly | rem;

    memset(out, 0x00, BYTES);
    unsigned ones = 0;
    for (unsigned degree = 0; degree < N; degree++) {
        bool bit = (code >> degree) & 1;
        setBit(out, bitOfDegree(degree), bit);
        ones += bit ? 1 : 0;
    }

    // Even parity over the 64 bits
    setBit(out, 63, (ones & 1) != 0);
}

bool Nid::decode(const uint8_t *in, uint16_t &nac, uint8_t &duid)
{
    const Field &gf = field();

    // Received polynomial coefficients
    uint8_t recd[N];
    for (unsigned degree = 0; degree < N; degree++)
        recd[degree] = getBit(in, bitOfDegree(degree)) ? 1 : 0;

    // Syndromes S_1 .. S_2t
    uint8_t synd[2 * T + 1];
    bool allZero = true;
    for (unsigned i = 1; i <= 2 * T; i++) {
        uint8_t s = 0;
        for (unsigned j = 0; j < N; j++) {
            if (recd[j])
                s ^= gf.exp[(i * j) % 63];
        }
        synd[i] = s;
        if (s != 0)
            allZero = false;
    }

    if (allZero == false) {
        // Berlekamp-Massey: error locator sigma(x), degree <= t
        uint8_t sigma[2 * T + 2] = { 1 };
        uint8_t prev[2 * T + 2] = { 1 };
        uint8_t temp[2 * T + 2];
        unsigned L = 0;
        unsigned m = 1;
        uint8_t b = 1;

        for (unsigned n = 0; n < 2 * T; n++) {
            uint8_t d = synd[n + 1];
            for (unsigned i = 1; i <= L; i++)
                d ^= gf.mul(sigma[i], synd[n + 1 - i]);

            if (d == 0) {
                m += 1;
                continue;
            }

            uint8_t coef = gf.mul(d, gf.inv(b));
            for (unsigned i = 0; i < 2 * T + 2; i++)
                temp[i] = sigma[i];
            for (unsigned i = 0; i + m < 2 * T + 2; i++)
                sigma[i + m] ^= gf.mul(coef, prev[i]);

            if (2 * L <= n) {
                L = n + 1 - L;
                for (unsigned i = 0; i < 2 * T + 2; i++)
                    prev[i] = temp[i];
                b = d;
                m = 1;
            } else {
                m += 1;
            }
        }

        if (L > T)
            return false;

        // Chien search: sigma(alpha^-j) == 0 marks an error at position j
        unsigned found = 0;
        for (unsigned j = 0; j < N; j++) {
            uint8_t value = 0;
            for (unsigned i = 0; i <= L; i++) {
                if (sigma[i] != 0)
                    value ^= gf.exp[(gf.log[sigma[i]] + i * (63 - j)) % 63];
            }

            if (value == 0) {
                recd[j] ^= 1;
                found += 1;
            }
        }

        if (found != L)
            return false;
    }

    // Information bit n is the coefficient of x^(47 + n)
    uint16_t info = 0;
    for (unsigned n = 0; n < K; n++)
        info = (info << 1) | recd[47 + n];

    nac = info >> 4;
    duid = info & 0x0F;

    return true;
}

void Nid::fromFrame(const uint8_t *frame, uint8_t *out)
{
    unsigned n = 0;
    for (unsigned i = 48; i < 114; i++) {
        if ((i == 70) || (i == 71))
            continue;
        setBit(out, n, getBit(frame, i));
        n += 1;
    }
}

void Nid::toFrame(const uint8_t *in, uint8_t *frame)
{
    unsigned n = 0;
    for (unsigned i = 48; i < 114; i++) {
        if ((i == 70) || (i == 71))
            continue;
        setBit(frame, i, getBit(in, n));
        n += 1;
    }
}

size_t Nid::frameBytes(const uint8_t duid)
{
    switch (duid) {
        case DUID_HDU:
            return 99;
        case DUID_TDU:
            return 18;
        case DUID_LDU1:
        case DUID_LDU2:
            return 216;
        case DUID_TDULC:
            return 54;
        default: // TSDU and PDU: variable length, not framed here
            return 0;
    }
}

#endif /* CONFIG_P25 */
