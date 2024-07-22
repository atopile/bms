#include "Cell.h"

// Constructor
Cell::Cell(uint8_t mux_channel) : mux_channel(mux_channel) {}

// Implementation of public methods
void Cell::init()
{
    // Set the mux channel
    setMuxChannel();
    // Initialize DACs
    ldo_dac.begin(LDO_ADDRESS);
    buck_dac.begin(BUCK_ADDRESS);

    // Initialize ADC
    adc.begin(ADC_ADDRESS);

    // Configure GPIO expander
    Wire.beginTransmission(0x20);
    Wire.write(0x03); // Configuration register
    Wire.write(0x00); // Set all pins as output
    Wire.endTransmission();

    // Set the outputs to 0
    Wire.beginTransmission(0x20);
    Wire.write(0x01); // Configuration register
    Wire.write(0x00); // Set all pins as output
    Wire.endTransmission();
}

void Cell::disable()
{
    GPIO_STATE &= ~(1 << 2);
    GPIO_STATE &= ~(1 << 3);
    setGPIOState();
}

void Cell::enable()
{
    GPIO_STATE |= (1 << 2);
    GPIO_STATE |= (1 << 3);
    setGPIOState();
}

float Cell::getVoltage()
{
    int16_t adc_value = adc.readADC_SingleEnded(2);
    float volts = adc.computeVolts(adc_value);
    return volts;
}

void Cell::setVoltage(float voltage)
{
    // Set the mux channel
    setMuxChannel();

    // calculate LDO setpoint
    uint16_t ldo_setpoint = calculateSetpoint(voltage);

    // Buck voltage should not be less than 1.5V
    if (voltage < 1.5)
    {
        voltage = 1.5;
    }

    // Set the buck voltage
    uint16_t buck_setpoint = calculateSetpoint(voltage);

    // Set LDO to desired voltage
    buck_dac.setVoltage(buck_setpoint, false);
    ldo_dac.setVoltage(ldo_setpoint, false);
}

void Cell::turnOnDMMRelay()
{
    GPIO_STATE |= (1 << 6);
    setGPIOState();
}

void Cell::turnOffDMMRelay()
{
    GPIO_STATE &= ~(1 << 6);
    setGPIOState();
}

void Cell::turnOnOutputRelay()
{
    GPIO_STATE |= (1 << 7);
    setGPIOState();
}

void Cell::turnOffOutputRelay()
{
    GPIO_STATE &= ~(1 << 7);
    setGPIOState();
}

float Cell::getCurrent()
{
    return readShuntCurrent();
}

// Implementation of private methods

float Cell::getLDOVoltage()
{
    setMuxChannel();
    int16_t adc_value = adc.readADC_SingleEnded(1);
    float volts = adc.computeVolts(adc_value);
    return volts;
}

uint16_t Cell::calculateSetpoint(float desiredOutput)
{
    // Known setpoints and corresponding output voltages
    float setpoint1 = 3566;
    float output1 = 0.5537;
    float setpoint2 = 1038;
    float output2 = 3.3971;

    // Calculate slope (m)
    float m = (setpoint2 - setpoint1) / (output2 - output1);

    // Calculate intercept (b)
    float b = setpoint1 - m * output1;

    // Calculate and return the setpoint for the desired output
    return static_cast<uint16_t>(m * desiredOutput + b);
}

float Cell::getBuckVoltage()
{
    setMuxChannel();
    int16_t adc_value = adc.readADC_SingleEnded(0);
    float volts = adc.computeVolts(adc_value);
    return volts;
}

void Cell::setBuckVoltage(float voltage)
{
    setMuxChannel();
    uint16_t dac_value = (uint16_t)(voltage / 3.3 * 4095);
    buck_dac.setVoltage(dac_value, false);
}

// Implementation of helper methods

void Cell::setGPIOState()
{
    setMuxChannel();
    // Set output relay on P7
    Wire.beginTransmission(TCA6408_ADDR);
    Wire.write(0x01); // Output register
    Wire.write(GPIO_STATE);
    Wire.endTransmission();
}

float Cell::readShuntCurrent()
{
    setMuxChannel();
    int16_t adc_value = adc.readADC_SingleEnded(3);
    float volts = adc.computeVolts(adc_value);
    return volts / 0.1; // Assuming a 0.1 ohm shunt resistor
}

void Cell::setMuxChannel()
{
    Wire.beginTransmission(MUX_ADDRESS);
    Wire.write(1 << mux_channel);
    Wire.endTransmission();
}
