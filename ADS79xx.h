

// https://github.com/utat-ss/HERON-lib-common

#ifndef ADC_H
#define ADC_H

#include <Arduino.h>
#include <stdint.h>
#include <SPI.h>
#include <HardwareSerial.h>

// Number channels in the ADC
// 12 for ADS7952, 16 for ADS7953
// Use 16 to always have array space for channel data
#define ADC_CHANNELS 16

// Reference voltage (with doubler enabled)
#define ADC_EXT_VREF 2.5000f
#define ADC_VREF 5.0000f

// #define DEBUG
#define DBGLN(x) (Serial.println(x));

// SPI Speed

#define ADS79xx_SPI_MAX_SPEED 20000000 // To achieve max throughput of 1 Msps
#define ADS79xx_SPI_NORMAL_SPEED 10000000
/*
NOTE: The standing assumption is that the power down bit
is never set, nor are the GPIO pins used.
*/

// Programming the program register
#define PGM_AUTO1_REG 0x8000
// Auto-1 program register; used to tell the ADC which channels
// to cycle through in Auto-1 mode

// DI15-12 - mode of operation
#define AUTO1_MODE 0x2000  // Correct
#define AUTO2_MODE 0x3000  // Correct
#define MANUAL_MODE 0x1000 // Correct
#define CONTINUE_MODE 0x0000

// DI11 - programming or retaining bits DI06-00
#define EN_PGM 0x0800

// DI10 - channel reset
#define CHAN_CTR_RST 0x0400

// DI06 - voltage reference range
#define RANGE2 0x0040 // 5V
#define RANGE1 0x0000 // 2.5V

// DI05 - power down (p.32)
#define POWER_DN 0x0020
#define POWER_UP 0x0000

#define REQUEST_AUTO1_MODE (AUTO1_MODE | EN_PGM | RANGE2 | CHAN_CTR_RST)
#define ENTER_AUTO1_MODE (AUTO1_MODE | EN_PGM | RANGE2)
#define CONTINUE_AUTO1_MODE (AUTO1_MODE | EN_PGM | RANGE2)
#define START_RESET (MANUAL_MODE | EN_PGM | RANGE2 | POWER_DN)
#define STOP_RESET (MANUAL_MODE | EN_PGM | RANGE2 | POWER_UP)

// ADC type
class ADS79xx
{

public:
    // ADC operational modes
    typedef enum
    {
        MANUAL,
        AUTO1
    } adc_mode;

    ADS79xx(SPIClass *spi = &SPI, uint8_t miso = 9, uint8_t mosi = 11, uint8_t sclk = 7, uint8_t cs = 5);

    // auto channels
    uint16_t auto_channels;
    // private
    // MANUAL or AUTO1 mode
    adc_mode mode;
    // channel data: stores the high/low reading after being fetched.
    uint16_t channel_data[ADC_CHANNELS];

    void init();
    void reset();
    void fetch_all_channels();
    void fetch_channel(uint8_t channel);
    uint16_t read_channel(uint8_t channel);
    uint16_t fetch_and_read_channel(uint8_t channel);
    double raw_to_ch_vol(uint16_t raw);
    uint16_t ch_vol_to_raw(double ch_vol);
    double ch_vol_to_vol_div(double ch_vol, double low_res, double high_res);
    static void setSPIspeed(ADS79xx *adc, uint32_t speed);
    inline uint32_t getSPIspeed() { return _SPIspeed; };
    // double adc_raw_to_ch_vol(uint16_t raw);

protected:
    // Define some SPI pins
    uint8_t _misoPin;
    uint8_t _mosiPin;
    uint8_t _sclkPin;
    uint8_t _csPin; // previously: pin_info_t* cs;

    HardwareSerial *_dbgPrt = nullptr; // For devices with native Serial (CDC) this changes
    SPIClass *_mySPI = nullptr;
    SPISettings _spi_settings;

    uint32_t _SPIspeed = ADS79xx_SPI_MAX_SPEED;

    uint16_t _send_frame(uint16_t frame);
};

#endif // ADC_H