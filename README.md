# ADS79xx ADC Library

This library provides an interface to control and read data from the **ADS79xx** [family](https://www.ti.com/product/ADS7953?utm_source=google&utm_medium=cpc&utm_campaign=asc-null-null-GPN_EN-cpc-pf-google-soas&utm_content=ADS7953&ds_k=ADS7953&DCM=yes&gad_source=1&gclid=Cj0KCQjw1um-BhDtARIsABjU5x61C_Hn-_1tx492KCy7FjdV7nNZp1u41BZIAsCvnCUxpxlU0XBnP1AaAqA-EALw_wcB&gclsrc=aw.ds) of SPI Analog-to-Digital Converters (ADCs). These ADCs are 12-bit, multi-channel devices that support up to 16 channels (ADS7953) or 12 channels (ADS7952). The library is designed to work with **Arduino-compatible boards** such as AVR and ESP32  (future tests will be carried out for STM32)

---

## Features

- **Multi-Channel Support**: Read data from up to 16 channels per ADC.
- **SPI Interface**: Utilizes the SPI protocol for communication.
- **Auto and Manual Modes**: Supports both auto-scan and manual channel selection modes. (_under test_)
- **Voltage Conversion**: Converts raw ADC values to actual voltage readings.
- **Multi-ADC Support**: Easily manage multiple ADCs with a single library instance.
- **Cross-Platform Compatibility**: Works with ESP32* and other Arduino-compatible boards. (_STM32 to be tested_)

(*)_please enote that, so far, it has been tested on ESP32 only (ESP32-S2 Lolin Mini S2)_

## Hardware Setup

### Components

- **ADS79xx** ADC
- Microcontroller (ESP32, AVR, etc.)
- SPI connections (MISO, MOSI, SCLK, CS)
- Voltage reference (2.5V or 5V)

### Pin correspondence

| ADC Pin | Microcontroller Pin |
|---------|---------------------|
| VDD     | 3.3V or 5V          |
| GND     | GND                 |
| MISO    | MISO (e.g., GPIO11)  |
| MOSI    | MOSI (e.g., GPIO9)   |
| SCLK    | SCLK (e.g., GPIO7)   |
| CS      | GPIO (e.g., GPIO5)   |

---

## Installation

1. Download the library as a `.zip` file or clone the repository:

   ```bash
   git clone https://github.com/Ivory-Electronics-Ltd/ADS79xx.git
   ```

2. Add the library to your Arduino IDE:
   - Go to **Sketch > Include Library > Add .ZIP Library**.
   - Select the downloaded `.zip` file.

---

## Usage

### Examples

(*) Read_two_ADCs

This example demonstrates how to read data from two ADS7953 ADCs and print the results to the serial monitor.

```cpp
#include "ADS79xx.h"

// SPI Pins
const int MISO_PIN = 11;
const int MOSI_PIN = 9;
const int SCLK_PIN = 7;
const int CS_H_PIN = 5; // High ADC (Channels 16-31)
const int CS_L_PIN = 3; // Low ADC (Channels 0-15)

// Create ADC instances
ADS79xx adcH(&SPI, MISO_PIN, MOSI_PIN, SCLK_PIN, CS_H_PIN);
ADS79xx adcL(&SPI, MISO_PIN, MOSI_PIN, SCLK_PIN, CS_L_PIN);

void setup() {
  Serial.begin(2000000);
  delay(3000);

  // Initialize ADCs
  adcH.init();
  adcL.init();

  Serial.println("ADCs Initialized!");
}

void loop() {
  // Read and print data from all channels
  for (uint8_t ch = 0; ch < 16; ch++) {
    uint16_t raw = adcL.fetch_and_read_channel(ch);
    double voltage = adcL.raw_to_ch_vol(raw);
    Serial.print("Channel ");
    Serial.print(ch);
    Serial.print(": ");
    Serial.print(voltage);
    Serial.println(" V");
  }
  delay(1000);
}
```

---

## API Reference

### Class: `ADS79xx`

#### Constructor

```cpp
ADS79xx(SPIClass *spi, uint8_t miso, uint8_t mosi, uint8_t sclk, uint8_t cs);
```

Initializes the ADC with the specified SPI pins.

#### Methods

```cpp
void init();

```

Initializes the ADC and sets it to manual mode.

```cpp
void reset();
```

Resets the ADC by powering it off and on.

```cpp
uint16_t fetch_and_read_channel(uint8_t channel);
```

Fetches and reads raw data from the specified channel.

```cpp
double raw_to_ch_vol(uint16_t raw);
```

Converts raw ADC data to voltage.

```cpp
uint16_t ch_vol_to_raw(double ch_vol);
```

Converts voltage to raw ADC data.

```cpp
void setSPIspeed(uint32_t speed);
```

Sets the SPI communication speed.

```cpp
uint32_t getSPIspeed();
```

Returns the current SPI speed.

---

## Examples

- **Example 1: Single ADC**  
  Read data from a single ADC and print the results.

- **Example 2: Dual ADCs**  
  Read data from two ADCs and map channels globally (0-31).

- **Example 3: Voltage Divider**  
  Convert ADC readings to actual circuit voltages using a voltage divider.

---

## Troubleshooting

### No Data Received:

- Check SPI connections (MISO, MOSI, SCLK, CS).
- Ensure the ADC is powered correctly.

### Incorrect Voltage Readings:

- Verify the reference voltage (`ADC_VREF`).
- Check for noise or unstable power supply.

### SPI Communication Errors:

- Ensure the SPI pins are correctly configured for your board.
- Use a logic analyzer to debug SPI signals.

---

## License

This library is licensed under the **MIT License**. See the `LICENSE` file for details.

---

## Contributing

Contributions are welcome! Please follow these steps:

1. Fork the repository.
2. Create a new branch for your feature or bugfix.
3. Submit a pull request.

---

## Acknowledgments

- I would like to thank [UTAT Space Systems](https://github.com/utat-ss) for the [reference code](https://github.com/utat-ss/HERON-lib-common) this library is based on.
- **Texas Instruments** for the ADS79xx datasheet.
- **Arduino community** for SPI library support.

---

## Contact

For questions or feedback, please open an issue on GitHub or contact **contact-AT-ivory-electronics.com**.

## #TODOs

- [ ] Add more example codes
- [ ] Buy beer :beers:
