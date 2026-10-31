/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 * 
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "peripherals/gpio.h"
#include "interfaces/delays.h"
#include "hwconfig.h"
#include "drivers/baseband/HR_C6000.h"

static const uint8_t initSeq1[] =
{
    0x04, 0x00, 0xFF, 0xB0, 0x00, 0x00, 0x00, 0x00, 0x21, 0x0B, 0xB8, 0x00
};

static const uint8_t initSeq2[] =
{
    0x04, 0x0D, 0x10, 0x8C, 0xBA, 0x6C, 0x80, 0x98, 0x00, 0x00, 0x00, 0x00,
    0x33, 0xEF, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x10, 0x00, 0x03, 0x06,
    0x3B, 0xF8, 0x0E, 0x7D, 0x40, 0x7D, 0x40, 0x0B, 0x0B, 0x17, 0x05, 0x04,
    0x0B, 0x00, 0x17, 0x02, 0xFF, 0xF0, 0x28, 0x12, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x06, 0x10
};

static const uint8_t initSeq3[] =
{
    0x04, 0xE0, 0x8B, 0x0F, 0x06, 0x52, 0x27, 0x1A
};

static const uint8_t initSeq4[] =
{
    0x01, 0x10, 0x69, 0x69, 0x96, 0x96, 0x96, 0x99, 0x99, 0x99, 0xA5, 0xA5,
    0xAA, 0xAA, 0xCC, 0xCC, 0x00, 0xF0, 0x01, 0xFF, 0x01, 0x0F, 0x00, 0x00,
    0x00, 0x00, 0x0D, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static const uint8_t initSeq5[] =
{
    0x01, 0x30, 0x00, 0x00, 0x20, 0x3C, 0xFF, 0xFF, 0x3F, 0x50, 0x07, 0x60,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static const uint8_t initSeq6[] =
{
    0x01, 0x40, 0x00, 0x01, 0x01, 0x02, 0x01, 0x1E, 0xF0
};

static const uint8_t initSeq7[] =
{
    0x01, 0x51, 0x00, 0x08, 0xEB, 0x78, 0x67
};

static const uint8_t initSeq8[] =
{
    0x01, 0x60, 0x10, 0xEF, 0x00, 0x11, 0xEF, 0x00, 0x12, 0xEF, 0x00, 0x13,
    0xEF, 0x00, 0x14, 0xEF, 0x00, 0x15, 0xEF, 0x00, 0x16, 0xEF, 0x00, 0x17,
    0xEF, 0x00, 0x18, 0xEF, 0x00, 0x19, 0xEF, 0x00, 0x1A, 0xEF, 0x00, 0x1B,
    0xEF, 0x00, 0x1C, 0xEF, 0x00, 0x1D, 0xEF, 0x00, 0x1E, 0xEF, 0x00, 0x1F,
    0xEF, 0x00, 0x20, 0xEF, 0x00, 0x21, 0xEF, 0x00, 0x22, 0xEF, 0x00, 0x23,
    0xEF, 0x00, 0x24, 0xEF, 0x00, 0x25, 0xEF, 0x00, 0x26, 0xEF, 0x00, 0x27,
    0xEF, 0x00, 0x28, 0xEF, 0x00, 0x29, 0xEF, 0x00, 0x2A, 0xEF, 0x00, 0x2B,
    0xEF, 0x00, 0x2C, 0xEF, 0x00, 0x2D, 0xEF, 0x00, 0x2E, 0xEF, 0x00, 0x2F,
    0xEF, 0x00
};


template class HR_Cx000 < C6000_SpiOpModes >;

template< class M >
void HR_Cx000< M >::init()
{
    writeReg(M::CONFIG, 0x0b, 0x28);    // Set PLLM
    writeReg(M::CONFIG, 0x0c, 0x33);    // Set PLLDO, PLLN, use PLL

    delayMs(2);

    writeReg(M::CONFIG, 0x0a, 0x01);    // Enable CLKOUT
    writeReg(M::CONFIG, 0x0b, 0x28);    // Set PLLM
    writeReg(M::CONFIG, 0x0c, 0x33);    // Set PLLDO, PLLN, use PLL
    writeReg(M::CONFIG, 0xb9, 0x05);    // Sysclock frequency
    writeReg(M::CONFIG, 0xba, 0x04);    // Codec clock frequency
    writeReg(M::CONFIG, 0xbb, 0x02);    // Clkout frequency

    delayMs(50);
    writeReg(M::CONFIG, 0x0a, 0x01);    // Enable CLKOUT
    delayMs(20);


    writeReg(M::CONFIG, 0x00, 0x00);    // Reset all
    delayMs(2);

    sendSequence(initSeq1, sizeof(initSeq1));
    sendSequence(initSeq2, sizeof(initSeq2));
    sendSequence(initSeq3, sizeof(initSeq3));
    sendSequence(initSeq4, sizeof(initSeq4));
    sendSequence(initSeq5, sizeof(initSeq5));
    sendSequence(initSeq6, sizeof(initSeq6));
    sendSequence(initSeq7, sizeof(initSeq7));
    sendSequence(initSeq8, sizeof(initSeq8));

    writeReg(M::CONFIG, 0x64, 0x10);    // Undocumented register
    writeReg(M::CONFIG, 0x65, 0x0a);    // Undocumented register
    writeReg(M::CONFIG, 0xa1, 0x00);    // FM mod
    writeReg(M::CONFIG, 0x12, 0x00);    // RF interrupt advance
    writeReg(M::CONFIG, 0x13, 0x00);    // TX delay compensation
    writeReg(M::CONFIG, 0x40, 0x03);    // Non-test mde
    writeReg(M::CONFIG, 0x41, 0x04);    // No CC match for RX
    writeReg(M::CONFIG, 0x22, 0x26);    // LRCK_CNT_H
    delayMs(10);

    writeReg(M::CONFIG, 0x22, 0x86);    // LRCK_CNT_H
    writeReg(M::CONFIG, 0x06, 0x21);    // Use V_SPI for vocoder, MCU controls the vocoder
    delayMs(10);

    writeReg(M::CONFIG, 0x22, 0x26);    // LRCK_CNT_H
}

template< class M >
void HR_Cx000< M >::terminate()
{

}

template< class M >
void HR_Cx000< M >::setModOffset(const uint16_t offset)
{
    /*
     * Same as Connect Systems's firmware.
     * Reference: function at address 0x08056746
     */

    uint16_t bias = (0xFF - static_cast< uint8_t>(offset)) & 0xFF;
    if(bias < 0x80)
        bias = 0x7F - bias;
    else
        bias = 0x47F - bias;

    uint8_t offUpper  = (bias >> 8) & 0x03;
    uint8_t offLower  = bias & 0xFF;
    uint8_t sigCenter = 0x80 + (static_cast< uint8_t>(offset) & 0xFF);

    writeReg(M::CONFIG, 0x47, offLower);    // Two-point bias, lower value
    writeReg(M::CONFIG, 0x48, offUpper);    // Two-point bias, upper value
    writeReg(M::CONFIG, 0x04, sigCenter);   // Bias value for TX, Q-channel
}

/*
 * DMR sync pattern loaded into AUX page 1 @0x04 by the stock firmware when it
 * enters DMR mode (fn 0x0802C4EC; two other 6-byte patterns at 0x080F54C4 and
 * 0x080F54CC are selected by other configuration bits and are not used here).
 */
static const uint8_t dmrSyncSeq[] =
{
    0x01, 0x04, 0xD5, 0xD7, 0xF7, 0x7F, 0xD7, 0x57
};

template< class M >
void HR_Cx000< M >::dmrMode()
{
    /*
     * Follows Connect Systems' V9.00.96 "enter DMR mode" routine (fn
     * 0x0802AD44), the branch that loads sync pattern 0x080F54BC, sets
     * reg 0x10 = 0x6A and reg 0x5F = 0x80.  The stock code picks other
     * register 0x10 values (0x6B, 0x6C, 0x6D) and reg 0x5F values (0x30, 0xC0)
     * depending on two configuration bits whose meaning is not yet known
     * (presumably relay/repeater/time-slot options); those are NOT handled.
     *
     * NOTE: there is no dedicated vocoder chip on this board.  The HR_C6000
     * exchanges 72-bit AMBE+2 frames with the MCU over V_SPI (reg 0x06 = 0x21,
     * set in init()), and the MCU runs the codec: see drivers/vocoder.
     *
     * Not yet verified on hardware.
     */

    writeReg(M::CONFIG, 0x00, 0x3F);    // Reset DMR and physical layer
    sendSequence(dmrSyncSeq, sizeof(dmrSyncSeq));
    writeReg(M::CONFIG, 0x10, 0x6A);    // Layer 2 DMR mode
    // Sync pattern detection.  Stock uses 0x80 for this branch, but the MD-UV3x0
    // DMR work (dmr-poc, same chip) uses 0xF0 = mobile, base station, TDMA1 and
    // TDMA2 sequences, which a hotspot or repeater needs.
    writeReg(M::CONFIG, 0x5F, 0xF0);
    writeReg(M::CONFIG, 0x81, 0xFF);    // InterClass1Mask, all interrupts enabled
    writeReg(M::CONFIG, 0x11, 0x80);    // LocalChanMode = 1

    /*
     * Receive front end for a digital channel, as the stock channel load does
     * it (HOST V9.00.96, fn 0x0804B390, pure-DMR branch at 0x0804BE90):
     * tones off, then 0x35/0x3E/0x3F as in FM but reg 0x34 = 0xA0 instead of
     * FM's 0xBC.  Without this, DMR inherited fmMode()'s 0xBC, i.e. the FM
     * voice band-pass and de-emphasis in front of the 4FSK demodulator.
     */
    writeReg(M::CONFIG, 0xA1, 0x00);    // Tones off
    writeReg(M::CONFIG, 0x35, 0x20);
    writeReg(M::CONFIG, 0x3E, 0x06);
    writeReg(M::CONFIG, 0x34, 0xA0);    // Stock value for digital channels
    writeReg(M::CONFIG, 0x3F, 0x04);

    /*
     * Data link layer and demodulator registers as the stock firmware leaves
     * them on a DMR channel.  Up to v22 this block used the GD-77 values from
     * the MD-UV3x0 DMR work (0x0E = 0x8E, 0x0F = 0xB8, 0x21 = 0xA0, ...), and
     * DMR also inherited fmMode()'s reg 0x26 = 0xFE.  The stock firmware
     * never rewrites 0x0D..0x0F, 0x23..0x2D after its boot presets (page 4
     * @0x0D: 10 8C BA ... 0x26 = 0x7D ...), and its DMR channel load does
     * 0x20 = 0, 0x21 = 1, 0x21 = 2, 0x22 = 0x16, 0x22 = 0x46 (0x0804B4F2).
     * On air with the GD-77 values about 45% of the AMBE frames had 4 or more
     * FEC errors.
     */
    writeReg(M::CONFIG, 0x0D, 0x10);    // Stock boot preset values
    writeReg(M::CONFIG, 0x0E, 0x8C);
    writeReg(M::CONFIG, 0x0F, 0xBA);
    writeReg(M::CONFIG, 0x13, 0x00);
    writeReg(M::CONFIG, 0x26, 0x7D);    // Stock preset; fmMode() leaves 0xFE
    writeReg(M::CONFIG, 0x2E, 0x04);
    writeReg(M::CONFIG, 0x20, 0x00);    // Stock DMR channel load sequence
    writeReg(M::CONFIG, 0x21, 0x01);
    writeReg(M::CONFIG, 0x21, 0x02);
    writeReg(M::CONFIG, 0x22, 0x16);
    writeReg(M::CONFIG, 0x22, 0x46);
    writeReg(M::CONFIG, 0x00, 0x3F);    // Restart the DMR modules with this configuration

    // Start passive reception (same steps as the stock helper at 0x0802CF54)
    writeReg(M::CONFIG, 0x40, 0x03);
    writeReg(M::CONFIG, 0x41, 0x20);    // SyncFail = 1: search again
    writeReg(M::CONFIG, 0x41, 0x00);
    writeReg(M::CONFIG, 0x41, 0x40);    // RxNxtSlotEn
    writeReg(M::CONFIG, 0x40, 0x43);    // RxEn, receive synchronisation active

    writeReg(M::AUX, 0x24, 0x00);       // Same four AUX registers fmMode() clears
    writeReg(M::AUX, 0x25, 0x00);
    writeReg(M::AUX, 0x26, 0x00);
    writeReg(M::AUX, 0x27, 0x00);
    writeReg(M::CONFIG, 0x64, 0x10);    // Undocumented register

    // Colour code in the upper nibble. TODO: take it from rtxStatus_t / codeplug.
    writeReg(M::CONFIG, 0x1F, 0x10);    // Colour code 1
}

template< class M >
void HR_Cx000< M >::fmMode()
{
    writeReg(M::CONFIG, 0x36, 0x10);  // Vocoder codec packet interface enabled
    writeReg(M::CONFIG, 0x36, 0x12);  // Receiving and opening the voice channel in FM mode Codec switch, 1 means on, 0 means off.
    writeReg(M::CONFIG, 0x20, 0x00);  // Local access policy, important
    writeReg(M::CONFIG, 0x21, 0x01);  // Control enable to clear the data in the vocoder decoding cache buffer
    writeReg(M::CONFIG, 0x21, 0x02);  // Control enable for clearing the data in the vocoder encoding buffer
    writeReg(M::CONFIG, 0x22, 0x16);  // Polite to all, polite, reserved = 1
    writeReg(M::CONFIG, 0x22, 0x46);  // Polite to all, polite, reserved = 1
    writeReg(M::CONFIG, 0x40, 0x03);  // Decode mode = non test mode
    writeReg(M::CONFIG, 0x41, 0x20);  // SyncFail = 1, no synchronization information exists, requiring the physical layer to search again.
    writeReg(M::CONFIG, 0x41, 0x00);  // SyncFail = 0
    writeReg(M::CONFIG, 0x41, 0x40);  // RxNxtSlotEn = 1, Start receiving the interrupt for the upcoming time slot. receive
    writeReg(M::CONFIG, 0x40, 0x43);  // RxEn = 1, receive synchronization active
    writeReg(M::CONFIG, 0xE0, 0x8B);  // CPU controls the codec, LineOut2 enabled, Mic_p enabled, HR_C6000 is I2S slave
    writeReg(M::CONFIG, 0x11, 0x80);  // LocalChanMode = 1

    writeReg(M::CONFIG, 0x00, 0x3F);  // Reset DMR and physical layer
    writeReg(M::CONFIG, 0x10, 0x80);  // Modulator mode FM
    writeReg(M::CONFIG, 0x35, 0x20);  // FM deviation coefficient
    writeReg(M::CONFIG, 0x3E, 0x06);  // FM receiving end modulation frequency offset coefficient
    writeReg(M::CONFIG, 0x81, 0x00);  // InterClass1Mask, all interrupts masked
    writeReg(M::CONFIG, 0x60, 0x00);  // TransControl, all off
    writeReg(M::CONFIG, 0x34, 0xBC);  // Band pass filter on, pre-emphasis on, wide bandwidth, 25kHz bandiwdth, RX bandwidth 25kHz
    writeReg(M::CONFIG, 0x3F, 0x04);  // FM limiting modulation coefficient
    writeReg(M::CONFIG, 0xE4, 0x25);  // Undocumented register
    writeReg(M::CONFIG, 0x37, 0xC1);  // DAC gain changed, +1.5dB
    writeReg(M::AUX,    0x24, 0x00);  // Undocumented register
    writeReg(M::AUX,    0x25, 0x00);  // Undocumented register
    writeReg(M::AUX,    0x26, 0x00);  // Undocumented register
    writeReg(M::AUX,    0x27, 0x00);  // Undocumented register
    writeReg(M::CONFIG, 0x64, 0x10);  // Undocumented register

    writeReg(M::CONFIG, 0x81, 0x00); // InterClass1Mask, all interrupts masked
    writeReg(M::CONFIG, 0xE0, 0x83); // CPU controls the codec, Mic_p enabled, HR_C6000 is I2S slave
    writeReg(M::CONFIG, 0x36, 0x10); // Vocoder codec packet interface enabled
    writeReg(M::CONFIG, 0x36, 0x12); // Receiving and opening the voice channel in FM mode Codec switch, 1 means on, 0 means off.
    writeReg(M::CONFIG, 0xE0, 0xC9); // Codec enabled, LineIn1, LineOut2, I2S slave mode
    writeReg(M::CONFIG, 0x26, 0xFE); // Undocumented register, disable FM audio output
}

template< class M >
void HR_Cx000< M >::startAnalogTx(const TxAudioSource source, const FmConfig cfg)
{
    uint8_t audioCfg = 0x81;
    if(source == TxAudioSource::MIC)     audioCfg |= 0x02;
    if(source == TxAudioSource::LINE_IN) audioCfg |= 0x40;

    writeReg(M::CONFIG, 0xE0, audioCfg);
    writeReg(M::CONFIG, 0x34, static_cast< uint8_t >(cfg));
    writeReg(M::CONFIG, 0x60, 0x80);    // Start analog transmission
}

template< class M >
void HR_Cx000< M >::stopAnalogTx()
{
    writeReg(M::CONFIG, 0x60, 0x00);    // Stop analog transmission
    writeReg(M::CONFIG, 0xE0, 0xC9);    // Codec enabled, LineIn1, LineOut2, I2S slave mode
    writeReg(M::CONFIG, 0x34, 0x98);    // FM bpf enabled, 25kHz bandwidth
}
