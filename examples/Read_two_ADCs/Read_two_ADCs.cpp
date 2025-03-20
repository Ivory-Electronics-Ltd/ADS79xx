/**
 * @file Read_two_ADCs.cpp
 * @author Ivory Electronics Ltd
 * @brief Example code for reading two ADS7953 ADCs and printing their channel data.
 * @version 0.1
 * @date 2024-12-09
 * @copyright Copyright (c) 2024
 */

#include "ADS79xx.h" // Include necessary libraries

// Define constants for LED and board configuration
#define OFF false
#define ON true

const int LED = 3; // GPIO3 for ESP32

unsigned long prevTime; // Track the last time the LED was toggled
bool ledState = OFF;    // Current state of the LED

// Buffer for printing data
char printBuf[250];

// ADC and SPI configuration
const uint8_t NUM_CHANNELS = 16;   // Number of channels per ADC
const uint8_t TOTAL_CHANNELS = 32; // Total number of channels (two ADCs)

// SPI Pins

const int MISO_PIN = 11;
const int MOSI_PIN = 9;
const int SCLK_PIN = 7;

// Chip Select Pins for two ADCs
const int CS_H_PIN = 5; // High ADC (Channels 16-31)
const int CS_L_PIN = 3; // Low ADC (Channels 0-15)

const char *labels[2] = {"LOW", "HIGH"}; // Labels for ADC instances

// Create two instances of the ADS79xx ADCs
ADS79xx adcH(&SPI, MISO_PIN, MOSI_PIN, SCLK_PIN, CS_H_PIN); // High ADC
ADS79xx adcL(&SPI, MISO_PIN, MOSI_PIN, SCLK_PIN, CS_L_PIN); // Low ADC

// Array to hold the ADC instances, reordered for correct mapping
ADS79xx ADCs[2] = {adcL, adcH}; // adcL handles CH0-15, adcH handles CH16-31

// Function Prototypes
void initialize_adcs();                                           // Initialize both ADCs
void plot_all_channels();                                         // Plot all channel voltages
void raw_and_volts_printer();                                     // Print raw and voltage data
void print_voltage(uint8_t channel);                              // Print voltage for a specific channel
void adc_idx_and_ch_extractor(uint8_t channel, uint8_t *indexes); // Map global channel to ADC and local channel

void setup()
{
    // Start serial communication for debugging
    Serial.begin(2000000);
    delay(3000); // Allow time for initialization

    pinMode(LED, OUTPUT);
    digitalWrite(LED, ledState);

    // Debug startup message
    Serial.println(__FILE__);
    Serial.println(F("*** ADS7953 16-CH 12-bit ADC Test ***"));
    Serial.println(F("Initialising ADC group..."));

    // Initialize ADCs
    initialize_adcs();
    Serial.println(F("...done!"));

    delay(500);
    prevTime = millis();
}

void loop()
{
    unsigned long now = millis();

    // Toggle LED every second
    if (now - prevTime >= 1000)
    {
        prevTime = now;
        ledState = !ledState;
        digitalWrite(LED, ledState);
    }

    // Plot raw and converted channel values
    raw_and_volts_printer();

    delay(1000); // Optional delay to manage serial output rate
}

// Initialize both ADCs in the array
void initialize_adcs()
{
    for (uint8_t idx = 0; idx < 2; idx++)
    {
        ADCs[idx].init(); // Initialize each ADC
    }
    delay(1000); // Ensure ADCs are fully initialized
}

// Print raw and voltage data for all channels
void raw_and_volts_printer()
{
    for (uint8_t ch = 0; ch < TOTAL_CHANNELS; ch++)
    {
        print_voltage(ch); // Print voltage for each channel
        delay(5);          // Small delay to manage serial output
    }
    Serial.print("\r\n"); // Newline after printing all channels
}

// Plot all channels' voltages to the serial monitor
void plot_all_channels()
{
    uint8_t chIdx = 0; // Channel index tracker
    char voltsBuf[20]; // Buffer to hold voltage string

    for (uint8_t adcIdx = 0; adcIdx < 2; adcIdx++)
    {
        for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++)
        {
            // Fetch raw data and convert to voltage
            uint16_t raw = ADCs[adcIdx].fetch_and_read_channel(ch);
            double ch_volts = ADCs[adcIdx].raw_to_ch_vol(raw);

            dtostrf(ch_volts, 6, 5, voltsBuf); // Convert voltage to string

            // Format and print voltage
            sprintf(printBuf, "%s", voltsBuf);
            Serial.print(printBuf);

            // Add a comma or newline based on channel index
            chIdx++;
            if (chIdx >= TOTAL_CHANNELS)
            {
                Serial.println(); // End of line for the last channel
            }
            else
            {
                Serial.print(","); // Comma-separated values
            }
        }
    }
}

// Print detailed voltage information for a specific channel
void print_voltage(uint8_t channel)
{
    char voltsBuf[10];  // Buffer to hold voltage string
    uint8_t indexes[2]; // Array to store ADC index and local channel index

    // Get ADC index and local channel index
    adc_idx_and_ch_extractor(channel, indexes);

    // Fetch raw data and convert to voltage
    uint16_t raw = ADCs[indexes[0]].fetch_and_read_channel(indexes[1]);
    double ch_volts = ADCs[indexes[0]].raw_to_ch_vol(raw);

    dtostrf(ch_volts, 6, 5, voltsBuf); // Convert voltage to string

    // Print channel details
    sprintf(printBuf, "ADC %s > Glob.Ch: %u [Int. Ch:%u], Raw: 0x%.3x, Voltage: %s V\n",
            labels[indexes[0]], channel, indexes[1], raw, voltsBuf);
    Serial.print(printBuf);
}

// Map a global channel index to the corresponding ADC and its local channel
void adc_idx_and_ch_extractor(uint8_t channel, uint8_t *indexes)
{
    // Ensure the channel is within valid bounds
    if (channel >= TOTAL_CHANNELS)
    {
        Serial.println(F("Error: Channel out of bounds!"));
        indexes[0] = indexes[1] = 0; // Default values to avoid undefined behavior
        return;
    }

    // Determine ADC index (0 or 1) and the local channel (0-15)
    indexes[0] = (channel < NUM_CHANNELS) ? 0 : 1; // ADC index
    indexes[1] = channel % NUM_CHANNELS;           // Local channel index
}