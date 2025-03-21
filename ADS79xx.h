/**
 * @file ADS79xx.h
 * @author Ivory Electronics Ltd
 * @brief Library for interfacing with the ADS7952 and ADS7953 Analog-to-Digital Converters (ADCs).
 * @version 0.1
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
 * 
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
#include <HardwareSerial.h>

// Number of channels in the ADC
// 12 for ADS7952, 16 for ADS7953
// Use 16 to always have array space for channel data
#define ADC_CHANNELS 16

// Reference voltage (with doubler enabled)
#define ADC_EXT_VREF 2.5000f // External reference voltage (2.5V)
#define ADC_VREF 5.0000f     // Internal reference voltage (5V)

// Debugging macro
// Uncomment the following line to enable debugging
// #define DEBUG
#define DBGLN(x) (Serial.println(x)); // Macro to print debug messages

// SPI Speed
#define ADS79xx_SPI_MAX_SPEED 20000000   // Max SPI speed for 1 Msps throughput
#define ADS79xx_SPI_NORMAL_SPEED 10000000 // Normal SPI speed

/*
NOTE: The standing assumption is that the power down bit
is never set, nor are the GPIO pins used.
*/

// Programming the program register
#define PGM_AUTO1_REG 0x8000
// Auto-1 program register; used to tell the ADC which channels
// to cycle through in Auto-1 mode

// DI15-12 - mode of operation
#define AUTO1_MODE 0x2000  // Auto-1 mode
#define AUTO2_MODE 0x3000  // Auto-2 mode
#define MANUAL_MODE 0x1000 // Manual mode
#define CONTINUE_MODE 0x0000 // Continue mode

// DI11 - programming or retaining bits DI06-00
#define EN_PGM 0x0800 // Enable programming

// DI10 - channel reset
#define CHAN_CTR_RST 0x0400 // Channel counter reset

// DI06 - voltage reference range
#define RANGE2 0x0040 // 5V range
#define RANGE1 0x0000 // 2.5V range

// DI05 - power down (p.32)
#define POWER_DN 0x0020 // Power down
#define POWER_UP 0x0000 // Power up

// Mode and configuration commands
#define REQUEST_AUTO1_MODE (AUTO1_MODE | EN_PGM | RANGE2 | CHAN_CTR_RST) // Request Auto-1 mode with reset
#define ENTER_AUTO1_MODE (AUTO1_MODE | EN_PGM | RANGE2) // Enter Auto-1 mode
#define CONTINUE_AUTO1_MODE (AUTO1_MODE | EN_PGM | RANGE2) // Continue Auto-1 mode
#define START_RESET (MANUAL_MODE | EN_PGM | RANGE2 | POWER_DN) // Start reset sequence
#define STOP_RESET (MANUAL_MODE | EN_PGM | RANGE2 | POWER_UP) // Stop reset sequence

// ADC type
class ADS79xx
{
public:
    // ADC operational modes
    typedef enum
    {
        MANUAL, // Manual mode
        AUTO1   // Auto-1 mode
    } adc_mode;

    // Constructor
    ADS79xx(SPIClass *spi = &SPI, uint8_t miso, uint8_t mosi, uint8_t sclk, uint8_t cs);

    // Auto channels
    uint16_t auto_channels;

    // Private members
    adc_mode mode; // Current mode (MANUAL or AUTO1)
    uint16_t channel_data[ADC_CHANNELS]; // Stores the high/low reading after being fetched

    // Public methods
    void init(); // Initialize the ADC
    void reset(); // Reset the ADC
    void fetch_all_channels(); // Fetch data from all channels
    void fetch_channel(uint8_t channel); // Fetch data from a specific channel
    uint16_t read_channel(uint8_t channel); // Read data from a specific channel
    uint16_t fetch_and_read_channel(uint8_t channel); // Fetch and read data from a specific channel
    double raw_to_ch_vol(uint16_t raw); // Convert raw ADC value to channel voltage
    uint16_t ch_vol_to_raw(double ch_vol); // Convert channel voltage to raw ADC value
    double ch_vol_to_vol_div(double ch_vol, double low_res, double high_res); // Convert channel voltage to voltage divider output
    static void setSPIspeed(ADS79xx *adc, uint32_t speed); // Set SPI speed
    inline uint32_t getSPIspeed() { return _SPIspeed; }; // Get current SPI speed

protected:
    // SPI pins
    uint8_t _misoPin; // MISO pin
    uint8_t _mosiPin; // MOSI pin
    uint8_t _sclkPin; // SCLK pin
    uint8_t _csPin;   // CS pin

    HardwareSerial *_dbgPrt = nullptr; // Debug print (for devices with native Serial)
    SPIClass *_mySPI = nullptr; // SPI class instance
    SPISettings _spi_settings; // SPI settings

    uint32_t _SPIspeed = ADS79xx_SPI_MAX_SPEED; // Current SPI speed

    // Private method to send a frame over SPI
    uint16_t _send_frame(uint16_t frame);
};

#endif // ADC_H