/**
 * @file ADS79xx.h
 * @author Ivory Electronics Ltd
 * @brief Library for interfacing with the ADS7952 and ADS7953 Analog-to-Digital Converters (ADCs).
 * @version 0.2
 * @date 2025-03-20
 *
 * @copyright Copyright (c) 2025
 *
 * This library provides an interface to control and read data from the Texas Instruments ADS7952 and ADS7953
 * 12-bit, multi-channel ADCs. It supports both manual and auto-scan modes, allowing users to read data from
 * individual channels or cycle through multiple channels automatically. The library also includes utility
 * functions for converting raw ADC values to actual voltages and vice versa.
 *
 * Key Features:
 * - Supports up to 16 channels (ADS7953) or 12 channels (ADS7952).
 * - Configurable SPI communication speed.
 * - Auto and manual channel selection modes.
 * - Voltage conversion functions for raw ADC data.
 * - Cross-platform compatibility with ESP32, STM32, and other Arduino-compatible boards.
 *
 * Note:
 * This library is loosely based on the ADC library from UTAT Space Systems: https://github.com/utat-ss
 * located at: https://github.com/utat-ss/HERON-lib-common
 *
 * For more information, refer to the Texas Instruments datasheet:
 * http://www.ti.com/lit/ds/slas605c/slas605c.pdf
 */

#ifndef ADC_H
#define ADC_H

#include <Arduino.h>
#include <stdint.h>
#include <SPI.h>

// Number of channels in the ADC
#define ADC_CHANNELS 16 ///< Supports 16 channels for ADS7953, 12 for ADS7952

// Reference voltage (with doubler enabled)
#define ADC_EXT_VREF 2.5000f ///< External reference voltage (2.5V)
#define ADC_VREF 5.0000f     ///< Internal reference voltage (5V)

// Debugging macro
// Uncomment the following line to enable debugging
// #define DEBUG
#define DBGLN(x) (Serial.println(x)) ///< Macro to print debug messages

// SPI Speed
#define ADS79xx_SPI_MAX_SPEED 20000000    ///< Max SPI speed for 1 Msps throughput
#define ADS79xx_SPI_NORMAL_SPEED 10000000 ///< Normal SPI speed

// Programming the program register
#define PGM_AUTO1_REG 0x8000 ///< Auto-1 program register; used to tell the ADC which channels to cycle through in Auto-1 mode

// DI15-12 - mode of operation
#define AUTO1_MODE 0x2000    ///< Auto-1 mode
#define AUTO2_MODE 0x3000    ///< Auto-2 mode
#define MANUAL_MODE 0x1000   ///< Manual mode
#define CONTINUE_MODE 0x0000 ///< Continue mode

// DI11 - programming or retaining bits DI06-00
#define EN_PGM 0x0800 ///< Enable programming

// DI10 - channel reset
#define CHAN_CTR_RST 0x0400 ///< Channel counter reset

// DI06 - voltage reference range
#define RANGE2 0x0040 ///< 5V range
#define RANGE1 0x0000 ///< 2.5V range

// DI05 - power down (p.32)
#define POWER_DN 0x0020 ///< Power down
#define POWER_UP 0x0000 ///< Power up

// Mode and configuration commands
#define REQUEST_AUTO1_MODE (AUTO1_MODE | EN_PGM | RANGE2 | CHAN_CTR_RST) ///< Request Auto-1 mode with reset
#define ENTER_AUTO1_MODE (AUTO1_MODE | EN_PGM | RANGE2)                  ///< Enter Auto-1 mode
#define CONTINUE_AUTO1_MODE (AUTO1_MODE | EN_PGM | RANGE2)               ///< Continue Auto-1 mode
#define START_RESET (MANUAL_MODE | EN_PGM | RANGE2 | POWER_DN)           ///< Start reset sequence
#define STOP_RESET (MANUAL_MODE | EN_PGM | RANGE2 | POWER_UP)            ///< Stop reset sequence

/**
 * @class ADS79xx
 * @brief Class for interfacing with the ADS7952 and ADS7953 ADCs.
 *
 * This class provides methods for configuring and reading data from the ADCs. It supports
 * auto-scan and manual modes, SPI communication, and utility functions for voltage conversion.
 */
class ADS79xx
{
public:
    /**
     * @brief Enumeration for ADC operational modes.
     */
    typedef enum
    {
        MANUAL, ///< Manual mode for individual channel reading
        AUTO1   ///< Auto-1 mode for cycling through channels
    } adc_mode;

    // Auto channels
    uint16_t auto_channels; ///< Bitmask for auto-scan channels

    /**
     * @brief Default Constructor.
     * Initializes the ADC instance with default values for later initialization.
     */
    ADS79xx();

    /**
     * @brief Parameterized Constructor.
     * Initializes the ADC instance with the SPI interface and pin configuration.
     *
     * @param spi Pointer to the SPI instance.
     * @param miso SPI MISO pin.
     * @param mosi SPI MOSI pin.
     * @param sclk SPI SCLK pin.
     * @param cs SPI Chip-Select pin.
     */
    ADS79xx(SPIClass *spi, uint8_t miso, uint8_t mosi, uint8_t sclk, uint8_t cs);

    /**
     * @brief Initialize the ADC hardware.
     * Used only with the default constructor for later initialization.
     *  @param spi SPI bus object pointer
     * @param miso SPI MISO pin.
     * @param mosi SPI MOSI pin.
     * @param sclk SPI SCLK pin.
     * @param cs SPI Chip-Select pin.
     */
    void init(SPIClass *spi, uint8_t miso, uint8_t mosi, uint8_t sclk, uint8_t cs);

    /**
     * @brief Check if the ADC instance is initialized.
     * @return True if initialized, false otherwise.
     */
    bool isInitialized() const;

    /**
     * @brief Reset the ADC.
     */
    void reset();

    /**
     * @brief Fetch data from all active channels in AUTO1 mode.
     */
    void fetch_all_channels();

    /**
     * @brief Fetch data from all active channels in AUTO1 mode using external CS pin.
     * This function is used when the ADC is connected to an external chip-select pin.
     */
    void fetch_all_channels_externalCS();

    /**
     * @brief Fetch data from a specific ADC channel in MANUAL mode.
     * @param channel Channel number to fetch data from.
     */
    void fetch_channel(uint8_t channel);

    /**
     * @brief Read previously fetched data for a specific ADC channel.
     * @param channel Channel number to read data from.
     * @return The raw ADC value for the specified channel.
     */
    uint16_t read_channel(uint8_t channel);

    /**
     * @brief Fetch and read data from a specific ADC channel.
     * Combines the functionality of fetch_channel and read_channel.
     *
     * @param channel Channel number to fetch and read data from.
     * @return The raw ADC value for the specified channel.
     */
    uint16_t fetch_and_read_channel(uint8_t channel);

    /**
     * @brief Configure the ADC operational mode.
     * @param newMode Desired operational mode (MANUAL or AUTO1).
     */
    void configureMode(adc_mode newMode);

    /**
     * @brief Convert raw ADC data to voltage.
     * @param raw Raw ADC value.
     * @return The calculated voltage.
     */
    double raw_to_ch_vol(uint16_t raw);

    /**
     * @brief Convert voltage to raw ADC data.
     * @param ch_vol Channel voltage.
     * @return The corresponding raw ADC value.
     */
    uint16_t ch_vol_to_raw(double ch_vol);

    /**
     * @brief Convert channel voltage using a resistor voltage divider.
     * @param ch_vol Channel voltage.
     * @param low_res Lower resistor value in the divider.
     * @param high_res Upper resistor value in the divider.
     * @return The calculated output voltage.
     */
    double ch_vol_to_vol_div(double ch_vol, double low_res, double high_res);

    /**
     * @brief Set the SPI communication speed.
     * @param speed Desired SPI speed.
     */
    void setSPIspeed(uint32_t speed);

protected:
    uint8_t _misoPin; ///< SPI MISO pin
    uint8_t _mosiPin; ///< SPI MOSI pin
    uint8_t _sclkPin; ///< SPI SCLK pin
    uint8_t _csPin;   ///< SPI Chip-Select pin

    SPIClass *_mySPI;                     ///< SPI instance
    SPISettings _spi_settings;            ///< SPI settings
    uint16_t _channel_data[ADC_CHANNELS]; ///< Channel data array
    adc_mode _mode;                       ///< Current operational mode
    bool _initialized;                    ///< Indicates if the instance is initialized
    uint32_t _SPIspeed;                   ///< SPI speed

    /**
     * @brief Send a frame over SPI.
     * @param frame Frame to send.
     * @return Response received from the ADC.
     */
    uint16_t _send_frame(uint16_t frame);

    //! ESP32-specific function to set the CS pin
    bool _fastGpioRead(uint8_t gpio);
    void _fastGpioClear(uint8_t gpio);
    void _fastGpioSet(uint8_t gpio);
    void _fastGpioSetOrClear(uint8_t gpio, bool value);
};

#endif // ADC_H