#include "ADS79xx.h"

/**
 * @brief Default Constructor.
 * Initializes the ADC instance with default values for later initialization.
 */
ADS79xx::ADS79xx()
    : _mySPI(nullptr), _misoPin(255), _mosiPin(255), _sclkPin(255), _csPin(255), _initialized(false), _mode(MANUAL), _SPIspeed(ADS79xx_SPI_MAX_SPEED)
{
    for (uint8_t i = 0; i < ADC_CHANNELS; i++)
    {
        _channel_data[i] = 0;
    }
}

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
ADS79xx::ADS79xx(SPIClass *spi, uint8_t miso, uint8_t mosi, uint8_t sclk, uint8_t cs)
    : _mySPI(spi),
      _misoPin(miso),
      _mosiPin(mosi),
      _sclkPin(sclk),
      _csPin(cs),
      _initialized(false),
      _mode(MANUAL),
      _SPIspeed(ADS79xx_SPI_MAX_SPEED)
{

    _mySPI = spi;

    for (uint8_t i = 0; i < ADC_CHANNELS; i++)
    {
        _channel_data[i] = 0;
    }

    //! Redundant initialization --> we should orce the user to call init() after this constructor
    // init(spi, miso, mosi, sclk, cs); // Initialize the hardware
}

/**
 * @brief Initialize the ADC hardware.
 * Used only with the default constructor for later initialization.
 */
void ADS79xx::init(SPIClass *spi, uint8_t miso, uint8_t mosi, uint8_t sclk, uint8_t cs)
{
    _mySPI = spi;
    _misoPin = miso;
    _mosiPin = mosi;
    _sclkPin = sclk;
    _csPin = cs;

    if (_misoPin == 255 || _mosiPin == 255 || _sclkPin == 255 || _csPin == 255 || _mySPI == nullptr)
    {
        DBGLN("Error: SPI pins or SPI instance not set!");
        return;
    }

    auto_channels = 0xFFFF; // <-- Enable all channels by default

    // Initialize SPI

    pinMode(_csPin, OUTPUT);

    // Set initial state for CS pin
    digitalWrite(_csPin, HIGH);

    // Initialize SPI
    //_mySPI->begin(_sclkPin, _misoPin, _mosiPin, _csPin);

    // Initialize SPI settings
    _spi_settings = SPISettings(_SPIspeed, MSBFIRST, SPI_MODE0);

    // Initialize channel data array
    for (uint8_t i = 0; i < ADC_CHANNELS; i++)
    {
        _channel_data[i] = 0;
    }

    _mode = MANUAL;

    // Mark as initialized
    _initialized = true;

    // Reset the ADC
    reset();
}

/**
 * @brief Check if the ADC instance is initialized.
 * @return True if initialized, false otherwise.
 */
bool ADS79xx::isInitialized() const
{
    return _initialized;
}

/**
 * @brief Reset the ADC.
 */
void ADS79xx::reset()
{
    _send_frame(START_RESET);
    delayMicroseconds(10);
    _send_frame(STOP_RESET);
    delayMicroseconds(10);
}

/**
 * @brief Fetch data from all active channels in AUTO1 _mode.
 */
void ADS79xx::fetch_all_channels()
{
    if (!_initialized)
    {
        DBGLN("ADC not initialized!");
        return;
    }

    if (_mode == MANUAL)
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
            _channel_data[channel] = raw_data & 0x0FFF;
        }
    }

    _mode = AUTO1;
}

/**
 * @brief Fetch data from a specific ADC channel in MANUAL _mode.
 * @param channel Channel number to fetch data from.
 */
void ADS79xx::fetch_channel(uint8_t channel)
{
    if (!_initialized)
    {
        DBGLN("ADC not initialized!");
        return;
    }

    if (channel >= ADC_CHANNELS)
    {
        DBGLN("Invalid channel number!");
        return;
    }

    uint16_t channel_addr = ((uint16_t)channel) << 7;
    uint16_t frame = MANUAL_MODE | EN_PGM | channel_addr | RANGE2;

    _send_frame(frame);
    _channel_data[channel] = _send_frame(frame) & 0x0FFF;

    _mode = MANUAL;
}

/**
 * @brief Read previously fetched data for a specific ADC channel.
 * @param channel Channel number to read data from.
 * @return The raw ADC value for the specified channel.
 */
uint16_t ADS79xx::read_channel(uint8_t channel)
{
    uint16_t dataPoint;

    if (!_initialized)
    {
        DBGLN("ADC not initialized!");
        return 0;
    }

    if (channel >= ADC_CHANNELS)
    {
        DBGLN("Invalid channel number!");
        return 0;
    }

    return _channel_data[channel];
}

/**
 * @brief Fetch and read data from a specific ADC channel.
 * Combines the functionality of fetch_channel and read_channel.
 *
 * @param channel Channel number to fetch and read data from.
 * @return The raw ADC value for the specified channel.
 */
uint16_t ADS79xx::fetch_and_read_channel(uint8_t channel)
{
    fetch_channel(channel);
    return read_channel(channel);
}

/**
 * @brief Configure the ADC operational _mode.
 * @param newMode Desired operational _mode (MANUAL or AUTO1).
 */
void ADS79xx::configureMode(adc_mode newMode)
{
    if (!_initialized)
    {
        DBGLN("ADC not initialized!");
        return;
    }

    if (newMode == _mode)
    {
        return;
    }

    if (newMode == AUTO1)
    {
        _send_frame(REQUEST_AUTO1_MODE);
        _send_frame(ENTER_AUTO1_MODE);
    }
    else if (newMode == MANUAL)
    {
        _send_frame(MANUAL_MODE | EN_PGM | RANGE2);
    }

    _mode = newMode;
}

/**
 * @brief Convert raw ADC data to voltage.
 * @param raw Raw ADC value.
 * @return The calculated voltage.
 */
double ADS79xx::raw_to_ch_vol(uint16_t raw)
{
    return ((double)raw / 0x0FFF) * ADC_VREF;
}

/**
 * @brief Convert voltage to raw ADC data.
 * @param ch_vol Channel voltage.
 * @return The corresponding raw ADC value.
 */
uint16_t ADS79xx::ch_vol_to_raw(double ch_vol)
{
    return (uint16_t)((ch_vol / ADC_VREF) * 0x0FFF);
}

/**
 * @brief Convert channel voltage using a resistor voltage divider.
 * @param ch_vol Channel voltage.
 * @param low_res Lower resistor value in the divider.
 * @param high_res Upper resistor value in the divider.
 * @return The calculated output voltage.
 */
double ADS79xx::ch_vol_to_vol_div(double ch_vol, double low_res, double high_res)
{
    if (low_res == 0)
    {
        DBGLN("Error: low_res cannot be zero!");
        return 0;
    }
    return ch_vol * (low_res + high_res) / low_res;
}

/**
 * @brief Set the SPI communication speed.
 * @param speed Desired SPI speed.
 */
void ADS79xx::setSPIspeed(uint32_t speed)
{
    if (!_initialized)
    {
        DBGLN("ADC not initialized!");
        return;
    }

    _SPIspeed = speed;
    _spi_settings = SPISettings(_SPIspeed, MSBFIRST, SPI_MODE0);
}

/**
 * @brief Send a frame over SPI.
 * @param frame Frame to send.
 * @return Response received from the ADC.
 */
uint16_t ADS79xx::_send_frame(uint16_t frame)
{
    if (!_initialized)
    {
        DBGLN("ADC not initialized!");
        return 0;
    }

    uint16_t received = 0;

    _mySPI->beginTransaction(_spi_settings);
    digitalWrite(_csPin, LOW);
    received = _mySPI->transfer16(frame);
    digitalWrite(_csPin, HIGH);
    _mySPI->endTransaction();

    return received;
}

//! TODO: Implement ESP32 fast_GPIO_write functions for SPI communication