#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h> // DAC
#include <Adafruit_ADS1X15.h> // ADC
#include <FastLED.h> // Addressable LEDs
#include "TCA9548.h" // I2C multiplexer

#define ledPin 6

TCA9548 i2c_mux;

// Cell functions:
// - Set Cell Voltage
// - Get Cell Voltage: Measure voltage with ADC
// - Get Cell Current: Measure voltage drop across shunt resistor
// - Calibrate Cell Voltage: Sweep DAC output, measure ADC output, store in EEPROM
// - Connect/Disconnect DMM: First check all cells are disconnected, then connect DMM to cell

void setupLEDs() {
    FastLED.addLeds<NEOPIXEL, ledPin>(leds, 6);
}

class Cell {
public:
    Cell(uint8_t mux_channel)
        : mux_channel(mux_channel) {
        i2c_mux.selectChannel(mux_channel);
        buck_dac.begin(buck_dac_address);
        ldo_dac.begin(ldo_dac_address);
        adc.begin(adc_address);
    }

    void setVoltage(float voltage) {
        uint16_t dac_value = voltageToDac(voltage);
        i2c_mux.selectChannel(mux_channel);
        buck_dac.setVoltage(dac_value * 1.1, false);
        ldo_dac.setVoltage(dac_value, false);
    }

    float getVoltage() {
        i2c_mux.selectChannel(mux_channel);
        int16_t adc_value = adc.readADC_SingleEnded(2);
        return adcToVoltage(adc_value);
    }

    float getCurrent() {
        i2c_mux.selectChannel(mux_channel);
        int16_t adc_value = adc.readADC_SingleEnded(3);
        return adcToCurrent(adc_value);
    }

    void measure() {
        i2c_mux.selectChannel(mux_channel);
        int16_t adc_value = adc.readADC_SingleEnded(0);
        buck_voltage = adcToVoltage(adc_value);

        adc_value = adc.readADC_SingleEnded(1);
        ldo_voltage = adcToVoltage(adc_value);

        adc_value = adc.readADC_SingleEnded(2);
        output_voltage = adcToVoltage(adc_value);

        adc_value = adc.readADC_SingleEnded(3);
        current = adcToCurrent(adc_value);
    }

private:
    uint8_t mux_channel;
    Adafruit_MCP4725 buck_dac;
    Adafruit_MCP4725 ldo_dac;
    Adafruit_ADS1115 adc;

    static const uint8_t buck_dac_address = 0x61;
    static const uint8_t ldo_dac_address = 0x60;
    static const uint8_t adc_address = 0x48;

    float buck_voltage = 0;
    float ldo_voltage = 0;
    float output_voltage = 0;
    float current = 0;

    uint16_t voltageToDac(float voltage) {
        return (uint16_t)((5.0 - voltage) * (4096 / 3.3)); // Inverted output for 0-5V with 3.3V reference
    }

    float adcToVoltage(int16_t adc_value) {
        return (float)adc_value * (3.3 / 32768.0); // Assuming 3.3V reference and 16-bit ADC
    }

    float adcToCurrent(int16_t adc_value) {
        float voltage = adcToVoltage(adc_value);
        float gain = 50;
        float shunt_resistance = 0.11;
        return (voltage / shunt_resistance) / gain;
    }
};

const int numberOfCells = 3;
Cell cells[numberOfCells] = {
    Cell(0),
    Cell(1),
    Cell(2)
};

CRGB leds[6]; // Assuming each cell will have 2 corresponding LEDs

void updateLEDColors() {
    for (int i = 0; i < numberOfCells; i++) { // Loop through cells
        float cellVoltage = cells[i].getVoltage();
        float cellCurrent = cells[i].getCurrent();

        // Map voltage and current to color brightness
        uint8_t brightnessVoltage = map(cellVoltage, 0, 5, 0, 255);
        uint8_t brightnessCurrent = map(cellCurrent, 0, 0.5, 0, 255);

        // Set LED colors: one LED for voltage, one for current per cell
        leds[2 * i] = CHSV(160, 255, brightnessVoltage); // Blueish color for voltage
        leds[2 * i + 1] = CHSV(0, 255, brightnessCurrent); // Redish color for current
    }
    FastLED.show();
}

void updateCells() {
    // Update cells
    // Get cell voltage and current
    for (int i = 0; i < numberOfCells; i++) {
        cells[i].measure();
    }
}

void setup() {
    Serial.begin(115200);
    Wire.begin();
    i2c_mux.begin(); // Initialize the I2C multiplexer
    setupLEDs();
}

static unsigned long lastRun = 0;
void loop() {
    unsigned long currentMillis = millis();
    if (currentMillis - lastRun >= 10) {
        lastRun = currentMillis;
        // Task to be performed every 10ms
        updateCells();
        updateLEDColors();
    }
}
