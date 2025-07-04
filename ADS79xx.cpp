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
#ifdef DEBUG
        DBGLN("Error: SPI pins or SPI instance not set!");
#endif
        return;
    }

    auto_channels = 0xFFFF; // <-- Enable all channels by default

    // Initialize CS pin
    pinMode(_csPin, OUTPUT);

// Set initial state for CS pin
//! ESP32-specific function to set the CS pin
#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
    _fastGpioSet(_csPin);
#else
    digitalWrite(_csPin, HIGH);
#endif

    // Initialize SPI settings
    _spi_settings = SPISettings(_SPIspeed, MSBFIRST, SPI_MODE0);

    // Initialize channel data array
    for (uint8_t i = 0; i < ADC_CHANNELS; i++)
    {
        _channel_data[i] = 0;
    }

    // Set MANUAL mode
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
    delayMicroseconds(1);
    _send_frame(STOP_RESET);
    delayMicroseconds(1);
}

/**
 * @brief Fetch data from all active channels in AUTO1 _mode.
 */
void ADS79xx::fetch_all_channels()
{
    if (!_initialized)
    {

#ifdef DEBUG
        DBGLN("ADC not initialized!");
#endif

        return;
    }
    /*
        if (_mode == MANUAL)
        {
            _send_frame(REQUEST_AUTO1_MODE);
            _send_frame(ENTER_AUTO1_MODE);
        }
            */

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

void ADS79xx::fetch_all_channels_externalCS()
{
    if (!_initialized)
        return;

    // Do NOT toggle CS or begin/end transaction here.
    // Just issue SPI transfers and decode responses.

    for (uint8_t i = 0; i < ADC_CHANNELS; ++i)
    {
        if (auto_channels & _BV(i))
        {
            uint16_t raw_data = _mySPI->transfer16(CONTINUE_AUTO1_MODE);

            delayMicroseconds(3); // Ensure the transfer is complete

            uint8_t channel = raw_data >> 12;
            _channel_data[channel] = raw_data & 0x0FFF;
        }
    }
}

/**
 * @brief Fetch data from a specific ADC channel in MANUAL _mode.
 * @param channel Channel number to fetch data from.
 */
void ADS79xx::fetch_channel(uint8_t channel)
{
    if (!_initialized)
    {
#ifdef DEBUG
        DBGLN("ADC not initialized!");
#endif
        return;
    }

    if (channel >= ADC_CHANNELS)
    {
#ifdef DEBUG
        DBGLN("Invalid channel number!");
#endif
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
#ifdef DEBUG
        DBGLN("ADC not initialized!");
#endif
        return 0;
    }

    if (channel >= ADC_CHANNELS)
    {
#ifdef DEBUG
        DBGLN("Invalid channel number!");
#endif
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
#ifdef DEBUG
        DBGLN("ADC not initialized!");
#endif
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
#ifdef DEBUG
        DBGLN("Error: low_res cannot be zero!");
#endif
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
#ifdef DEBUG
        DBGLN("ADC not initialized!");
#endif
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
#ifdef DEBUG
        DBGLN("ADC not initialized!");
#endif
        return 0;
    }

    uint16_t received = 0;

    _mySPI->beginTransaction(_spi_settings);

#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
    _fastGpioClear(_csPin);
#else
    digitalWrite(_csPin, LOW);
#endif

    received = _mySPI->transfer16(frame);

#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
    _fastGpioSet(_csPin);
#else
    digitalWrite(_csPin, HIGH);
#endif

    _mySPI->endTransaction();

    return received;
}

// --- Fast GPIO for ESP32 ---

void ADS79xx::_fastGpioSet(uint8_t gpio)
{
    if (gpio < 32)
        GPIO.out_w1ts = (1 << gpio);
    else
        GPIO.out1_w1ts.val = (1 << (gpio - 32));
}

void ADS79xx::_fastGpioClear(uint8_t gpio)
{
    if (gpio < 32)
        GPIO.out_w1tc = (1 << gpio);
    else
        GPIO.out1_w1tc.val = (1 << (gpio - 32));
}

bool ADS79xx::_fastGpioRead(uint8_t gpio)
{
    if (gpio < 32)
        return (GPIO.in >> gpio) & 0x1;
    else
        return (GPIO.in1.data >> (gpio - 32)) & 0x1;
}

// Utility for fast set/clear
void ADS79xx::_fastGpioSetOrClear(uint8_t gpio, bool value)
{
    if (value)
        _fastGpioSet(gpio);
    else
        _fastGpioClear(gpio);
}

////TODO: Implement ESP32 fast_GPIO_write functions for SPI communication