#include "ADS79xx.h"

// Constructor for ADS79xx class
ADS79xx::ADS79xx(SPIClass *spi, uint8_t miso, uint8_t mosi, uint8_t sclk, uint8_t cs)
{
    _mySPI = spi;
    _misoPin = miso;
    _mosiPin = mosi;
    _sclkPin = sclk;
    _csPin = cs;
}

// Initialize the ADC
void ADS79xx::init()
{
#if defined(DEBUG)
    DBGLN("Initialising ADC...");
#endif

    // Set GPIO port for ADC CS
    pinMode(_csPin, OUTPUT);
    digitalWrite(_csPin, HIGH);

    // Initialize SPI
    _mySPI->begin(_sclkPin, _misoPin, _mosiPin, _csPin);

    _spi_settings = SPISettings(_SPIspeed, MSBFIRST, SPI_MODE0);

#if defined(DEBUG)
    DBGLN("SPI Configured...");
#endif

    // Initialize channel data array
    for (uint8_t i = 0; i < ADC_CHANNELS; i++)
    {
        channel_data[i] = 0;
    }

    this->mode = adc_mode::MANUAL;
    reset();
}

// Reset the ADC
void ADS79xx::reset()
{
#if defined(DEBUG)
    DBGLN("Resetting ADC...");
#endif

    uint16_t frame = START_RESET;
    _send_frame(frame);
    delayMicroseconds(10);
    frame = STOP_RESET;
    _send_frame(frame);
    delayMicroseconds(10);
}

// Fetch data from all channels
void ADS79xx::fetch_all_channels()
{
    if (this->mode == adc_mode::MANUAL)
    {
        _send_frame(REQUEST_AUTO1_MODE);
        _send_frame(ENTER_AUTO1_MODE);
    }

    for (uint8_t i = 0; i < ADC_CHANNELS; i++)
    {
        if (auto_channels & _BV(i))
        {
            uint16_t raw_data = _send_frame(CONTINUE_AUTO1_MODE);
            uint8_t channel = raw_data >> 12;
            channel_data[channel] = raw_data & 0x0FFF;
        }
    }

    this->mode = adc_mode::AUTO1;
}

// Fetch data from a specific channel
void ADS79xx::fetch_channel(uint8_t channel)
{
    uint16_t channel_addr = ((uint16_t)channel) << 7;
    uint16_t frame = MANUAL_MODE | EN_PGM | channel_addr | RANGE2;

    _send_frame(frame);
    _send_frame(frame);

    channel_data[channel] = _send_frame(frame) & 0x0FFF;

    this->mode = adc_mode::MANUAL;
}

// Read data from a specific channel
uint16_t ADS79xx::read_channel(uint8_t channel)
{
    return channel_data[channel];
}

// Convert raw ADC data to voltage
double ADS79xx::raw_to_ch_vol(uint16_t raw)
{
    double ratio = (double)raw / (double)0x0FFF;
    return ratio * ADC_VREF;
}

// Convert voltage to raw ADC data
uint16_t ADS79xx::ch_vol_to_raw(double ch_vol)
{
    return (uint16_t)((ch_vol / ADC_VREF) * 0x0FFF);
}

// Convert channel voltage to voltage divider output
double ADS79xx::ch_vol_to_vol_div(double ch_vol, double low_res, double high_res)
{
    return ch_vol / low_res * (low_res + high_res);
}

// Set SPI speed
void ADS79xx::setSPIspeed(ADS79xx *adc, uint32_t speed)
{
    adc->_SPIspeed = speed;
}

// Fetch and read data from a specific channel
uint16_t ADS79xx::fetch_and_read_channel(uint8_t channel)
{
    fetch_channel(channel);
    return read_channel(channel);
}

// Send a 16-bit frame over SPI
uint16_t ADS79xx::_send_frame(uint16_t frame)
{
    uint16_t received = 0;

    _mySPI->beginTransaction(_spi_settings);
    digitalWrite(_csPin, LOW);
    received = _mySPI->transfer16(frame);
    digitalWrite(_csPin, HIGH);
    _mySPI->endTransaction();

#if defined(DEBUG)
    DBGLN("Sent: 0x" + String(frame, HEX) + ", Received: 0x" + String(received, HEX));
#endif

    return received;
}