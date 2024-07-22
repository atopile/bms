#ifndef CELL_H
#define CELL_H

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_MCP4725.h> // DAC
#include <Adafruit_ADS1X15.h> // ADC

class Cell
{
public:
    // Constructor
    Cell(uint8_t mux_channel);

    // Public methods
    void init();
    void enable();
    void disable();
    float getVoltage();
    float getCurrent();
    void setVoltage(float voltage);
    void turnOnDMMRelay();
    void turnOffDMMRelay();
    void turnOnOutputRelay();
    void turnOffOutputRelay();
    uint8_t GPIO_STATE = 0b00000000;

private:
    // Hardcoded I2C addresses
    const uint8_t LDO_ADDRESS = 0x60;
    const uint8_t BUCK_ADDRESS = 0x61;
    const uint8_t ADC_ADDRESS = 0x48;
    const uint8_t GPIO_ADDRESS = 0x20;
    const uint8_t MUX_ADDRESS = 0x70;
    const uint8_t TCA6408_ADDR = 0x20;

    // Mux channel
    const uint8_t mux_channel;

    // GPIO state

    // Device instances
    Adafruit_MCP4725 ldo_dac;
    Adafruit_MCP4725 buck_dac;
    Adafruit_ADS1115 adc;

    // Private methods
    float getLDOVoltage();
    void setLDOVoltage(float voltage);
    float getBuckVoltage();
    void setBuckVoltage(float voltage);
    uint16_t calculateSetpoint(float voltage);

    // Helper methods for I2C communication
    void setMuxChannel();
    void setGPIOState();
    float readShuntCurrent();
};

#endif // CELL_H
