/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "protocols/DMR/Bptc19696.hpp"
#include "protocols/DMR/Hamming.hpp"

namespace DMR
{

// ETSI TS 102 361-1 Annex B.1.1: for each of the 196 cells of the
// 13x15 BPTC matrix (numbered in encoding order, matrix cell order
// row-major starting at row 1), the position that cell occupies in
// the 196-bit on-air stream, plus whether it is reserved padding or a
// row/column Hamming parity bit rather than information.
const Bptc19696::MapEntry Bptc19696::map[196] =
{
    {0,    0,  0,  true,  false},

    {181,  1,  0,  true,  false},
    {166,  1,  1,  true,  false},
    {151,  1,  2,  true,  false},
    {136,  1,  3,  false, false},
    {121,  1,  4,  false, false},
    {106,  1,  5,  false, false},
    {91,   1,  6,  false, false},
    {76,   1,  7,  false, false},
    {61,   1,  8,  false, false},
    {46,   1,  9,  false, false},
    {31,   1,  10, false, false},
    {16,   1,  11, false, true},
    {1,    1,  12, false, true},
    {182,  1,  13, false, true},
    {167,  1,  14, false, true},

    {152,  2,  0,  false, false},
    {137,  2,  1,  false, false},
    {122,  2,  2,  false, false},
    {107,  2,  3,  false, false},
    {92,   2,  4,  false, false},
    {77,   2,  5,  false, false},
    {62,   2,  6,  false, false},
    {47,   2,  7,  false, false},
    {32,   2,  8,  false, false},
    {17,   2,  9,  false, false},
    {2,    2,  10, false, false},
    {183,  2,  11, false, true},
    {168,  2,  12, false, true},
    {153,  2,  13, false, true},
    {138,  2,  14, false, true},

    {123,  3,  0,  false, false},
    {108,  3,  1,  false, false},
    {93,   3,  2,  false, false},
    {78,   3,  3,  false, false},
    {63,   3,  4,  false, false},
    {48,   3,  5,  false, false},
    {33,   3,  6,  false, false},
    {18,   3,  7,  false, false},
    {3,    3,  8,  false, false},
    {184,  3,  9,  false, false},
    {169,  3,  10, false, false},
    {154,  3,  11, false, true},
    {139,  3,  12, false, true},
    {124,  3,  13, false, true},
    {109,  3,  14, false, true},

    {94,   4,  0,  false, false},
    {79,   4,  1,  false, false},
    {64,   4,  2,  false, false},
    {49,   4,  3,  false, false},
    {34,   4,  4,  false, false},
    {19,   4,  5,  false, false},
    {4,    4,  6,  false, false},
    {185,  4,  7,  false, false},
    {170,  4,  8,  false, false},
    {155,  4,  9,  false, false},
    {140,  4,  10, false, false},
    {125,  4,  11, false, true},
    {110,  4,  12, false, true},
    {95,   4,  13, false, true},
    {80,   4,  14, false, true},

    {65,   5,  0,  false, false},
    {50,   5,  1,  false, false},
    {35,   5,  2,  false, false},
    {20,   5,  3,  false, false},
    {5,    5,  4,  false, false},
    {186,  5,  5,  false, false},
    {171,  5,  6,  false, false},
    {156,  5,  7,  false, false},
    {141,  5,  8,  false, false},
    {126,  5,  9,  false, false},
    {111,  5,  10, false, false},
    {96,   5,  11, false, true},
    {81,   5,  12, false, true},
    {66,   5,  13, false, true},
    {51,   5,  14, false, true},

    {36,   6,  0,  false, false},
    {21,   6,  1,  false, false},
    {6,    6,  2,  false, false},
    {187,  6,  3,  false, false},
    {172,  6,  4,  false, false},
    {157,  6,  5,  false, false},
    {142,  6,  6,  false, false},
    {127,  6,  7,  false, false},
    {112,  6,  8,  false, false},
    {97,   6,  9,  false, false},
    {82,   6,  10, false, false},
    {67,   6,  11, false, true},
    {52,   6,  12, false, true},
    {37,   6,  13, false, true},
    {22,   6,  14, false, true},

    {7,    7,  0,  false, false},
    {188,  7,  1,  false, false},
    {173,  7,  2,  false, false},
    {158,  7,  3,  false, false},
    {143,  7,  4,  false, false},
    {128,  7,  5,  false, false},
    {113,  7,  6,  false, false},
    {98,   7,  7,  false, false},
    {83,   7,  8,  false, false},
    {68,   7,  9,  false, false},
    {53,   7,  10, false, false},
    {38,   7,  11, false, true},
    {23,   7,  12, false, true},
    {8,    7,  13, false, true},
    {189,  7,  14, false, true},

    {174,  8,  0,  false, false},
    {159,  8,  1,  false, false},
    {144,  8,  2,  false, false},
    {129,  8,  3,  false, false},
    {114,  8,  4,  false, false},
    {99,   8,  5,  false, false},
    {84,   8,  6,  false, false},
    {69,   8,  7,  false, false},
    {54,   8,  8,  false, false},
    {39,   8,  9,  false, false},
    {24,   8,  10, false, false},
    {9,    8,  11, false, true},
    {190,  8,  12, false, true},
    {175,  8,  13, false, true},
    {160,  8,  14, false, true},

    {145,  9,  0,  false, false},
    {130,  9,  1,  false, false},
    {115,  9,  2,  false, false},
    {100,  9,  3,  false, false},
    {85,   9,  4,  false, false},
    {70,   9,  5,  false, false},
    {55,   9,  6,  false, false},
    {40,   9,  7,  false, false},
    {25,   9,  8,  false, false},
    {10,   9,  9,  false, false},
    {191,  9,  10, false, false},
    {176,  9,  11, false, true},
    {161,  9,  12, false, true},
    {146,  9,  13, false, true},
    {131,  9,  14, false, true},

    {116,  10, 0,  false, true},
    {101,  10, 1,  false, true},
    {86,   10, 2,  false, true},
    {71,   10, 3,  false, true},
    {56,   10, 4,  false, true},
    {41,   10, 5,  false, true},
    {26,   10, 6,  false, true},
    {11,   10, 7,  false, true},
    {192,  10, 8,  false, true},
    {177,  10, 9,  false, true},
    {162,  10, 10, false, true},
    {147,  10, 11, false, true},
    {132,  10, 12, false, true},
    {117,  10, 13, false, true},
    {102,  10, 14, false, true},

    {87,   11, 0,  false, true},
    {72,   11, 1,  false, true},
    {57,   11, 2,  false, true},
    {42,   11, 3,  false, true},
    {27,   11, 4,  false, true},
    {12,   11, 5,  false, true},
    {193,  11, 6,  false, true},
    {178,  11, 7,  false, true},
    {163,  11, 8,  false, true},
    {148,  11, 9,  false, true},
    {133,  11, 10, false, true},
    {118,  11, 11, false, true},
    {103,  11, 12, false, true},
    {88,   11, 13, false, true},
    {73,   11, 14, false, true},

    {58,   12, 0,  false, true},
    {43,   12, 1,  false, true},
    {28,   12, 2,  false, true},
    {13,   12, 3,  false, true},
    {194,  12, 4,  false, true},
    {179,  12, 5,  false, true},
    {164,  12, 6,  false, true},
    {149,  12, 7,  false, true},
    {134,  12, 8,  false, true},
    {119,  12, 9,  false, true},
    {104,  12, 10, false, true},
    {89,   12, 11, false, true},
    {74,   12, 12, false, true},
    {59,   12, 13, false, true},
    {44,   12, 14, false, true},

    {29,   13, 0,  false, true},
    {14,   13, 1,  false, true},
    {195,  13, 2,  false, true},
    {180,  13, 3,  false, true},
    {165,  13, 4,  false, true},
    {150,  13, 5,  false, true},
    {135,  13, 6,  false, true},
    {120,  13, 7,  false, true},
    {105,  13, 8,  false, true},
    {90,   13, 9,  false, true},
    {75,   13, 10, false, true},
    {60,   13, 11, false, true},
    {45,   13, 12, false, true},
    {30,   13, 13, false, true},
    {15,   13, 14, false, true},
};

void Bptc19696::deinterleave(const uint8_t onAir[196], uint8_t matrix[13][15])
{
    for (unsigned int i = 0; i < 196; i++)
    {
        const MapEntry &e = map[i];
        matrix[e.row - 1][e.col] = onAir[e.interleaveIndex];
    }
}

void Bptc19696::interleave(const uint8_t matrix[13][15], uint8_t onAir[196])
{
    for (unsigned int i = 0; i < 196; i++)
    {
        const MapEntry &e = map[i];
        if (e.reserved)
            continue;
        onAir[e.interleaveIndex] = matrix[e.row - 1][e.col];
    }
}

bool Bptc19696::decode(const uint8_t onAir[196], uint8_t info[96])
{
    uint8_t matrix[13][15];
    deinterleave(onAir, matrix);

    bool ok = true;

    for (unsigned int row = 0; row < 13; row++)
    {
        uint8_t codeword[15];
        uint8_t data[11];
        for (unsigned int c = 0; c < 15; c++)
            codeword[c] = matrix[row][c];

        if (!Hamming15113::decode(codeword, data))
            ok = false;

        for (unsigned int c = 0; c < 15; c++)
            matrix[row][c] = codeword[c];
    }

    for (unsigned int col = 0; col < 15; col++)
    {
        uint8_t codeword[13];
        uint8_t data[9];
        for (unsigned int r = 0; r < 13; r++)
            codeword[r] = matrix[r][col];

        if (!Hamming1393::decode(codeword, data))
            ok = false;

        for (unsigned int r = 0; r < 13; r++)
            matrix[r][col] = codeword[r];
    }

    unsigned int infoIdx = 0;
    for (unsigned int i = 0; i < 196; i++)
    {
        const MapEntry &e = map[i];
        if (e.reserved || e.hamming)
            continue;
        info[infoIdx++] = matrix[e.row - 1][e.col];
    }

    return ok;
}

void Bptc19696::encode(const uint8_t info[96], uint8_t onAir[196])
{
    uint8_t matrix[13][15] = { };

    unsigned int infoIdx = 0;
    for (unsigned int i = 0; i < 196; i++)
    {
        const MapEntry &e = map[i];
        if (e.reserved || e.hamming)
            matrix[e.row - 1][e.col] = 0;
        else
            matrix[e.row - 1][e.col] = info[infoIdx++];
    }

    for (unsigned int row = 0; row < 13; row++)
    {
        uint8_t data[11];
        uint8_t codeword[15];
        for (unsigned int c = 0; c < 11; c++)
            data[c] = matrix[row][c];

        Hamming15113::encode(data, codeword);

        for (unsigned int c = 0; c < 15; c++)
            matrix[row][c] = codeword[c];
    }

    for (unsigned int col = 0; col < 15; col++)
    {
        uint8_t data[9];
        uint8_t codeword[13];
        for (unsigned int r = 0; r < 9; r++)
            data[r] = matrix[r][col];

        Hamming1393::encode(data, codeword);

        for (unsigned int r = 0; r < 13; r++)
            matrix[r][col] = codeword[r];
    }

    interleave(matrix, onAir);
}

}   // namespace DMR
