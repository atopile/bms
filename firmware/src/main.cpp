#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Cell.h>
// #include <Adafruit_SSD1306.h> // OLED
#include <Adafruit_MCP4725.h> // DAC
#include <Adafruit_ADS1X15.h> // ADC
// #include <FastLED.h>          // Addressable LEDs


// Micro pinout
#define ledPin 0
#define sdaPin 7
#define sclPin 6

// // Addressable LEDs
// #define NUM_LEDS 6
// CRGB leds[NUM_LEDS];

// Create cells
Cell cell1(1);
Cell cell2(2);
Cell cell3(3);

// void setupLEDs()
// {
//     FastLED.addLeds<NEOPIXEL, ledPin>(leds, NUM_LEDS);
// }

void setup()
{
    USBSerial.begin(115200);
    Wire.setPins(sdaPin, sclPin);
    Wire.begin();

    cell1.init();
    cell2.init();
    cell3.init();

    cell1.turnOnOutputRelay();
    cell2.turnOnOutputRelay();
    cell3.turnOnOutputRelay();

    cell1.turnOffDMMRelay();
    cell2.turnOffDMMRelay();
    cell3.turnOffDMMRelay();

    cell1.enable();
    cell2.enable();
    cell3.enable();
}

float voltage = 0.02;
float sum = 0.01;
void loop()
{
    if (voltage > 4)
    {
        sum = -0.01;
    }
    if (voltage < 0.02)
    {
        sum = 0.01;
    }
    USBSerial.print(">Setpoint: ");
    USBSerial.println(voltage);


    cell1.setVoltage(voltage);
    cell2.setVoltage(voltage);
    cell3.setVoltage(voltage);

    float cell1_voltage = cell1.getVoltage();
    float cell2_voltage = cell2.getVoltage();
    float cell3_voltage = cell3.getVoltage();

    // USBSerial.println(cell1.GPIO_STATE, BIN);

    // print the voltage of each cell
    USBSerial.print(">Cell 1 voltage: ");
    USBSerial.println(cell1_voltage,5);
    USBSerial.print(">Cell 2 voltage: ");
    USBSerial.println(cell2_voltage,5);
    USBSerial.print(">Cell 3 voltage: ");
    USBSerial.println(cell3_voltage,5);
    voltage += sum;

    delay(10);
}




// // GPIOd
// #define TCA6408_ADDR 0b00100000

// // // ADC
// Adafruit_ADS1115 adc;

// // DAC
// Adafruit_MCP4725 ldo_dac;
// Adafruit_MCP4725 buck_dac;

// #define LDO_DAC_ADDR 0x60
// #define BUCK_DAC_ADDR 0x61

// // // OLED
// // #define SCREEN_WIDTH 128 // OLED display width, in pixels
// // #define SCREEN_HEIGHT 32 // OLED display height, in pixels

// // #define OLED_RESET     -1
// // #define SCREEN_ADDRESS 0x3C
// // Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


// void setup()
// {
//     USBSerial.begin(115200);
//     Wire.setPins(sdaPin, sclPin);
//     Wire.begin();

//     delay(100);

//     // set the port to 1
//     Wire.beginTransmission(0x70);
//     Wire.write(0b000000100);
//     Wire.endTransmission();

//     adc.begin();

//     ldo_dac.begin(LDO_DAC_ADDR);
//     buck_dac.begin(BUCK_DAC_ADDR);

//     setupLEDs();

//     // make leds purple and 10% brightness
//     for (int i = 0; i < NUM_LEDS; i++)
//     {
//         leds[i] = CRGB(255, 0, 255);
//         leds[i].fadeLightBy(230);
//     }

//     FastLED.show();

// }
// int counter = 0;

// void loop()
// {
//     // Configure GPIO expander
//     Wire.beginTransmission(0x20);
//     Wire.write(0x03); // Configuration register
//     Wire.write(0x00); // Set P7 as output
//     Wire.endTransmission();

//     // Set output relay on P7
//     Wire.beginTransmission(TCA6408_ADDR);
//     Wire.write(0x01); // Output register
//     Wire.write(0b10001100); // Set P7
//     Wire.endTransmission();

//     // delay(100);
//     // // Read back GPIO expander
//     // Wire.beginTransmission(TCA6408_ADDR);
//     // Wire.write(0x00); // Output register
//     // Wire.endTransmission();
//     // Wire.requestFrom(TCA6408_ADDR, 1);
//     // uint8_t data = Wire.read();
//     // USBSerial.println(data, BIN);

//     if (counter > 4095)
//     {
//         counter = 0;
//     }
//     counter += 100;
//     buck_dac.setVoltage(0, false);
//     ldo_dac.setVoltage(0, false);
//     delay(10);

//     int16_t adc0, adc1, adc2, adc3;
//     float volts0, volts1, volts2, volts3;

//     adc0 = adc.readADC_SingleEnded(0);
//     volts0 = adc.computeVolts(adc0);
//     adc1 = adc.readADC_SingleEnded(1);
//     volts1 = adc.computeVolts(adc1);
//     adc2 = adc.readADC_SingleEnded(2);
//     volts2 = adc.computeVolts(adc2);
//     adc3 = adc.readADC_SingleEnded(3);
//     volts3 = adc.computeVolts(adc3);
//     USBSerial.print("buck: ");
//     USBSerial.println(volts0);
//     USBSerial.print("ldo: ");
//     USBSerial.println(volts1);
//     USBSerial.print("Output: ");
//     USBSerial.println(volts2);
//     USBSerial.print("Shunt: ");
//     USBSerial.print((volts3/50)/0.13);
//     USBSerial.println();

// }