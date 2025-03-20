/*
ADS7952/ADS7953 Analog to Digital Converter

Link to Datasheet:
http://www.ti.com/lit/ds/slas605c/slas605c.pdf

The ADS7952 is the 12-channel variant, while the ADS7953 is the 16-channel
variant (both have 12-bit precision).

Brief Overview:
An analog to digital converter is a component that takes an analog signal input
(which can take on any value within some range) and converts it to a
digital representation (either a "high" or "low" value, indicated by a "0" or "1"
bit), which in turn can be used by the 32M1 (ie. the microcontrollers recognize
only digital data, hence the need to convert analog to digital data).

Terms:
- A "frame" is defined as every sending/recieving of two bytes.
- A "channel" is simply an input. The ADC has 12 channels, thus four bits are
required to describe them. you can select one of these input channels to read from.
   - Channel 0 is the default.
   - For channel selection, there is manual mode (default on reset) or
     auto 1 mode.


How it Works:
This device operates in 16 bit (2 byte) "frames". In every frame, we (that is,
the microcontroller) can specify whether to continue the current operational
settings, or change the operation (e.g. change selected channel number).
Simultaneously in every frame, the ADC sends the microcontroller 16 bits- the
first four of which are the channel number, while the next 12 are the actual
converted data corresponding to that channel.
The input data is 12 bits describing some voltage, and the ADC takes this in and
converts it to some digital 12 bit value. If the output is to be "high", this will
look like 12 1's, while an output of "low" would look like 12 0's.


Usages:
1. Initialize ADC -> init_adc()
2. Call a fetch on a particular channel or all the channels to update the values
on the channel array -> fetch_all_adc_channels() or fetch_adc_channel(c)
where c is the channel number
3. Call read to return the value on a particular channel -> read_adc_channel(c)

*/
#include "ADS79xx.h"

/**
 * @brief Construct a new ADS79xx::ADS79xx object
 *
 * @param spi
 * @param miso
 * @param mosi
 * @param sclk
 * @param cs
 */
ADS79xx::ADS79xx(SPIClass *spi, uint8_t miso, uint8_t mosi, uint8_t sclk, uint8_t cs)
{
    _mySPI = spi;
    _misoPin = miso;
    _mosiPin = mosi;
    _sclkPin = sclk;
    _csPin = cs;
}

/*
Initializing ADC:
Channel 0 is the default.
Manual mode is the default on reset for channel selection.

*/

/**
 * @brief
 *
 */
void ADS79xx::init()
{

#if defined(DEBUG)
    DBGLN("Initialising ADC...");
#endif

    // Set GPIO port for ADC CS
    pinMode(_csPin, OUTPUT);    // init_cs(adc->cs->pin, adc->cs->ddr);
    digitalWrite(_csPin, HIGH); // set_cs_high(adc->cs->pin, adc->cs->port);

    // Intialize SPI

#if defined(ARDUINO_ARCH_ESP32)

    DBGLN("Configuring SPI for ESP32...");

    _mySPI->begin(_sclkPin, _misoPin, _mosiPin, _csPin); // init_spi();

#elif defined(ARDUINO_ARCH_STM32)

    DBGLN("Configuring SPI Pins for STM32...");

    _mySPI->setMISO(_misoPin);
    _mySPI->setMOSI(_mosiPin);
    _mySPI->setSCLK(_sclkPin);

    //_mySPI->setSSEL(_csPin); // Aha! Do not pass the SSEL pin

    DBGLN("Beggining SPI for STM32...");

    _mySPI->begin();

#endif

    DBGLN("Setting up SPI for STM32...");

    _spi_settings = SPISettings(_SPIspeed, MSBFIRST, SPI_MODE0);

#if defined(DEBUG)
    DBGLN("SPI Configured...");
#endif

    for (uint8_t i = 0; i < ADC_CHANNELS; i++)
    {
        channel_data[i] = 0;
    }

    this->mode = adc_mode::MANUAL;

    reset();

    // Program auto-1 register --> Why do they do this?
    // uint16_t f1 = PGM_AUTO1_REG; // cmd of changing mode to auto-mode- where it fetches all channels
    // uint16_t f2 = auto_channels;
    //_send_frame(f1);
    //_send_frame(f2);
}
/*
Resets the ADC: powers it off, then powers it on.
*/

/**
 * @brief
 *
 */
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
    // based on table p.33 of datasheet
}

/*
Gets the digital data from all channels in the ADC and updates
the channel array at all indicies.
@param adc_t* adc - ADC
*/

/**
 * @brief
 *
 */
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
        { // if the bits representing the channel match the auto_channels
            uint16_t raw_data = _send_frame(CONTINUE_AUTO1_MODE);
            uint8_t channel = raw_data >> 12;
            channel_data[channel] = raw_data & 0x0fff;
            // adc->channel_data[i] = send_frame(CONTINUE_AUTO1_MODE) & 0x0fff;
        }
    }

    this->mode = adc_mode::AUTO1;
}

/*
Gets the digital data from one channel in the ADC and updates
channel array at index c.
@param adc_t* adc - ADC
@param uint8_t c - the specified channel
*/

/**
 * @brief
 *
 * @param channel
 */
void ADS79xx::fetch_channel(uint8_t channel)
{
    uint16_t channel_addr = ((uint16_t)channel) << 7;
    uint16_t frame = MANUAL_MODE | EN_PGM | channel_addr | RANGE2;

    _send_frame(frame);
    _send_frame(frame);

    channel_data[channel] = _send_frame(frame) & 0x0fff;

    this->mode = adc_mode::MANUAL;
}

/*
Reads the information currently stored in the ADC.
Returns a 16-bit unsigned integer that is stored in the channel array at index c.
@param adc_t* adc - the ADC
@param uint8_t c - the specified channel
*/

/**
 * @brief
 *
 * @param channel
 * @return uint16_t
 */
uint16_t ADS79xx::read_channel(uint8_t channel)
{
    return channel_data[channel];
}

/*
Converts raw data from an ADC channel to the voltage on that ADC channel input pin.
raw_data - 12 bit ADC data
returns - voltage on ADC input channel pin (in V)
*/

/**
 * @brief
 *
 * @param raw
 * @return double
 */
double ADS79xx::raw_to_ch_vol(uint16_t raw)
{
    double ratio = (double)raw / (double)0x0FFF;
    double voltage = ratio * ADC_VREF;
    return voltage;
}

/*
Converts a voltage on an ADC channel pin to a voltage in the circuit using
    a voltage divider ratio.
raw_voltage - voltage on an ADC channel input pin (in V)
low_res - low-side resistance (in ohms)
high_res - high-side resistance (in ohms)
returns - voltage in the circuit (in V)
*/

/**
 * @brief
 *
 * @param ch_vol
 * @param low_res
 * @param high_res
 * @return double
 */
double ADS79xx::ch_vol_to_vol_div(double ch_vol, double low_res, double high_res)
{
    // Use voltage divider circuit ratio to recover original voltage before division
    return ch_vol / low_res * (low_res + high_res);
}

/*
Converts the voltage on an ADC channel input pin to raw data from an ADC channel.
raw_data - voltage on ADC input channel pin (in V)
returns - 12 bit ADC data
*/

/**
 * @brief
 *
 * @param ch_vol
 * @return uint16_t
 */
uint16_t ADS79xx::ch_vol_to_raw(double ch_vol)
{
    return (uint16_t)((ch_vol / (double)ADC_VREF) * 0x0FFF);
}

/**
 * @brief Configure SPI Bus speed
 *
 * @param speed
 */
void ADS79xx::setSPIspeed(ADS79xx *adc, uint32_t speed)
{
    adc->_SPIspeed = speed;
}

/*
Fetches and reads 16-bit raw data for the specified channel in manual mode.
*/

/**
 * @brief
 *
 * @param channel
 * @return uint16_t
 */
uint16_t ADS79xx::fetch_and_read_channel(uint8_t channel)
{
    fetch_channel(channel);
    return read_channel(channel);
}

/*
Sending 16 bits to and from the ADC as per SPI protocol.
'Send SPI frame' called twice in order to get the 16 bits through.
This function gets each device to send one bit of information at a time
simultaneously, until the full 16 bits are sent and recieved.

@param adc_t* adc - the ADC
@param uint16_t frame - the 16 bits of the frame
*/

/**
 * @brief
 *
 * @param frame
 * @return uint16_t
 */
uint16_t ADS79xx::_send_frame(uint16_t frame)
{
    uint16_t received = 0;

    _mySPI->beginTransaction(_spi_settings);

    digitalWrite(_csPin, LOW);
    // delayMicroseconds(1);

    received = _mySPI->transfer16(frame); // Single 16-bit transfer

    digitalWrite(_csPin, HIGH);
    // delayMicroseconds(1);

    _mySPI->endTransaction();

#ifdef DEBUG
    DBGLN("Sent: 0x" + String(frame, HEX) + ", Received: 0x" + String(received, HEX));
#endif

    return received;
}
