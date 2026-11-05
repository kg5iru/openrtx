/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "protocols/P25/modem/Vocoder.hpp"
#include "hwconfig.h"

#ifdef CONFIG_P25

#include <assert.h>

namespace P25
{

void Vocoder::init()
{
    assert(imbe2_encoder_state_size() <= ENC_STATE_SIZE);
    assert(imbe2_decoder_state_size() <= DEC_STATE_SIZE);

    imbe2_encoder_init(encState);
    imbe2_decoder_init(decState);

    initialised = true;
}

void Vocoder::terminate()
{
    initialised = false;
}

void Vocoder::encode(const int16_t *speech, uint8_t *bits)
{
    assert(initialised);
    imbe2_encode(encState, bits, speech);
}

void Vocoder::decode(const uint8_t *bits, int16_t *speech)
{
    assert(initialised);
    imbe2_decode(decState, speech, bits);
}

} /* P25 */

#endif /* CONFIG_P25 */
