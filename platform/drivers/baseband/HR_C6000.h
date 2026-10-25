/*
 * SPDX-FileCopyrightText: Copyright 2020-2026 OpenRTX Contributors
 * 
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef HRC6000_H
#define HRC6000_H

#include "core/datatypes.h"
#include "drivers/baseband/HR_Cx000.h"

enum class C6000_SpiOpModes : uint8_t
{
    AUX    = 1,     ///< Auxiliary configuration registers.
    DATA   = 2,     ///< Write TX data register and read RX data register.
    SOUND  = 3,     ///< Voice prompt sample register.
    CONFIG = 4,     ///< Main configuration registers.
    AMBE3K = 5,     ///< AMBE3000 configuration register.
    DATA_R = 6,     ///< Write RX data register and read TX data register.
    AMBE1K = 7      ///< AMBE1000 configuration register.
};

class HR_C6000 : public HR_Cx000 < C6000_SpiOpModes >
{
public:

    /**
     * Constructor.
     *
     * @param uSpi: pointer to SPI device for "user" SPI interface.
     * @param uCs: gpioPin object for "user" SPI chip select.
     */
    HR_C6000(const struct spiDevice *uSpi, const struct gpioPin uCs) :
        HR_Cx000< C6000_SpiOpModes >(uSpi, uCs) { }

    /**
     * Configure CTCSS tone transmission.
     *
     * @param tone: CTCSS tone frequency.
     * @param deviation: CTCSS tone deviation.
     */
    void setTxCtcss(const tone_t tone, const uint8_t deviation);

    /**
     * Configure CTCSS tone detection.
     *
     * @param tone: CTCSS tone frequency.
     */
    void setRxCtcss(const tone_t tone);

    /**
     * Test if RX CTCSS tone has been detected.
     *
     * @return true if RX CTCSS tone has been detected.
     */
    inline bool ctcssDetected()
    {
        uint8_t reg = readCfgRegister(0x93);
        return ((reg & 0x01) != 0) ? true : false;
    }

    /**
     * Disable all tone encode and decode.
     */
    inline void disableTones()
    {
        writeCfgRegister(0xA1, 0x00);     // Disable all tones
    }

    /**
     * Transmit a tone of a given frequency.
     *
     * @param freq: tone frequency in Hz.
     * @param deviation: tone deviation.
     */
    void sendTone(const uint32_t freq, const uint8_t deviation);

    /**
     * Write a block of bytes into the chip's TX-side 1.2K RAM (DMR Layer 2
     * frame buffer). Address map and per-frame-type sizes are documented in
     * the HR_C6000 manual, section 5.3.
     *
     * NOTE: unverified against real hardware -- addressing scheme mirrors
     * writeReg16()/readReg16() below, which *are* exercised elsewhere in
     * this driver (sendTone()), but the extended addressing byte order in
     * the manual's translation is garbled, so double-check with a logic
     * analyzer before trusting this for anything beyond bring-up.
     *
     * @param addr: start address within TX RAM (see manual table 5.2).
     * @param data: pointer to the bytes to write.
     * @param len: number of bytes to write.
     */
    inline void writeTxRam(const uint16_t addr, const uint8_t *data, const size_t len)
    {
        uint8_t cmd[3];
        cmd[0] = static_cast< uint8_t >(C6000_SpiOpModes::DATA) | SPI_FLAGS_EXTD;
        cmd[1] = addr & 0xFF;
        cmd[2] = (addr >> 8) & 0x07;

        ScopedChipSelect cs(uSpi, uCs);
        spi_send(uSpi, cmd, 3);
        spi_send(uSpi, data, len);
    }

    /**
     * Read a block of bytes from the chip's RX-side 1.2K RAM (DMR Layer 2
     * frame buffer). See writeTxRam() for caveats.
     *
     * @param addr: start address within RX RAM (see manual table 5.3).
     * @param data: pointer to the buffer that will receive the read bytes.
     * @param len: number of bytes to read.
     */
    inline void readRxRam(const uint16_t addr, uint8_t *data, const size_t len)
    {
        uint8_t cmd[3];
        cmd[0] = static_cast< uint8_t >(C6000_SpiOpModes::DATA)
                | SPI_FLAGS_READ | SPI_FLAGS_EXTD;
        cmd[1] = addr & 0xFF;
        cmd[2] = (addr >> 8) & 0x07;

        ScopedChipSelect cs(uSpi, uCs);
        spi_send(uSpi, cmd, 3);
        spi_receive(uSpi, data, len);
    }

    /**
     * Read a received burst the way the stock CS7000 firmware does: a two
     * byte command (read | DATA mode, 8-bit address) followed by the data.
     * The stock firmware reads 36 bytes from address 0x30 or 0x60:
     * 3 header bytes, 6 bytes sync/EMB, 27 bytes (3 x 72 bit) of voice.
     */
    inline void readRxBurst(const uint8_t addr, uint8_t *data, const size_t len)
    {
        uint8_t cmd[2];
        cmd[0] = SPI_FLAGS_READ | static_cast< uint8_t >(C6000_SpiOpModes::DATA);
        cmd[1] = addr;

        ScopedChipSelect cs(uSpi, uCs);
        spi_send(uSpi, cmd, 2);
        spi_receive(uSpi, data, len);
    }

private:

    /**
     * Write a register with 16-bit address.
     *
     * @param opMode: "operating mode" specifier, see datasheet for details.
     * @param addr: register address.
     * @param value: value to be written.
     */
    void writeReg16(const C6000_SpiOpModes opMode, const uint16_t addr, const uint8_t value)
    {
        uint8_t data[4];

        data[0] = SPI_FLAGS_EXTD | static_cast< uint8_t >(opMode);
        data[2] = (addr >> 8) & 0x07;
        data[1] = addr & 0xFF;
        data[3] = value;

        ScopedChipSelect cs(uSpi, uCs);
        spi_send(uSpi, data, 4);
    }

    /**
     * Read a register with 16-bit address.
     *
     * @param opMode: "operating mode" specifier, see datasheet for details.
     * @param addr: register address.
     * @return: value read from the register.
     */
    uint8_t readReg16(const C6000_SpiOpModes opMode, const uint16_t addr)
    {
        uint8_t data[3];
        uint8_t value[1];

        data[0] = SPI_FLAGS_READ | SPI_FLAGS_EXTD
                | static_cast< uint8_t >(opMode);
        data[2] = (addr >> 8) & 0x07;
        data[1] = addr & 0xFF;

        ScopedChipSelect cs(uSpi, uCs);
        spi_send(uSpi, data, sizeof(data));
        spi_receive(uSpi, value, 1);

        return value[0];
    }
};

#endif /* HRC6000_H */
