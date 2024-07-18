#include <Arduino.h>
#include <SPI.h>

// Define SPI pins
#define SPI1_MOSI 11
#define SPI1_MISO 8
#define SPI1_SCK 10
#define SPI1_CS 9

// Initialize SPI with specified settings
arduino::MbedSPI SPI1(SPI1_MISO, SPI1_MOSI, SPI1_SCK);
SPISettings settings(1000000, MSBFIRST, SPI_MODE3); // 1 MHz clock, MSB first, SPI mode 3

// Commands and constants
#define ADBMS6948_CMD_ADCV ((uint16_t)0x0260u)
#define ADBMS6948_CMD_RDACALL ((uint16_t)0x004Cu)
#define ADBMS6948_SHIFT_BY_8 ((uint8_t)8u)
#define ADBMS6948_RDALL_CELLVOLTAGES_BYTES ((uint8_t)34u)
#define ADBMS6948_CMD_DATA_LEN ((uint8_t)0x04u)
#define ADBMS6948_REG_DATA_LEN_WITH_PEC ((uint8_t)0x08u)
#define ADBMS6948_MAX_NO_OF_DEVICES_IN_DAISY_CHAIN (1U)
#define ADBMS6948_REG_GRP_LEN ((uint8_t)0x06u)
#define ADBMS6948_CMD_WRCFGA ((uint16_t)0x0001u)
#define ADBMS6948_CMD_WRCFGB ((uint16_t)0x0024u)
#define ADBMS6948_CMD_WRCFGC ((uint16_t)0x0081u)
#define ADBMS6948_CMD_WRCFGD ((uint16_t)0x00A4u)
#define ADBMS6948_CMD_WRCFGE ((uint16_t)0x0073u)
#define ADBMS6948_CMD_WRCFGF ((uint16_t)0x0075u)
#define ADBMS6948_CMD_WRCFGG ((uint16_t)0x0077u)
#define ADBMS6948_CMD_WRCFGH ((uint16_t)0x0079u)
#define ADBMS6948_CMD_WRCFGI ((uint16_t)0x007Bu)
#define ADBMS6948_CMD_RDCFGA ((uint16_t)0x0002u)
#define ADBMS6948_CMD_RDCFGB ((uint16_t)0x0026u)
#define ADBMS6948_CMD_RDCFGC ((uint16_t)0x0082u)
#define ADBMS6948_CMD_RDCFGD ((uint16_t)0x00A6u)
#define ADBMS6948_CMD_RDCFGE ((uint16_t)0x0074u)
#define ADBMS6948_CMD_RDCFGF ((uint16_t)0x0076u)
#define ADBMS6948_CMD_RDCFGG ((uint16_t)0x0078u)
#define ADBMS6948_CMD_RDCFGH ((uint16_t)0x007Au)
#define ADBMS6948_CMD_RDCFGI ((uint16_t)0x007Cu)
#define RDCVA ((uint16_t)0x0004u)
#define RDCVB ((uint16_t)0x0006u)
#define RDCVC ((uint16_t)0x0008u)
#define RDCVD ((uint16_t)0x000Au)
#define RDCVE ((uint16_t)0x0009u)
#define RDCVF ((uint16_t)0x000Bu)
#define RDCVALL ((uint16_t)0x000Cu)

/* Pre-computed CRC15 Table */
static const uint16_t Adbms6948_Crc15Table[256] = {0x0u, 0xc599u, 0xceabu, 0xb32u, 0xd8cfu, 0x1d56u, 0x1664u, 0xd3fdu, 0xf407u, 0x319eu, 0x3aacu,
                                                   0xff35u, 0x2cc8u, 0xe951u, 0xe263u, 0x27fau, 0xad97u, 0x680eu, 0x633cu, 0xa6a5u, 0x7558u, 0xb0c1u,
                                                   0xbbf3u, 0x7e6au, 0x5990u, 0x9c09u, 0x973bu, 0x52a2u, 0x815fu, 0x44c6u, 0x4ff4u, 0x8a6du, 0x5b2eu,
                                                   0x9eb7u, 0x9585u, 0x501cu, 0x83e1u, 0x4678u, 0x4d4au, 0x88d3u, 0xaf29u, 0x6ab0u, 0x6182u, 0xa41bu,
                                                   0x77e6u, 0xb27fu, 0xb94du, 0x7cd4u, 0xf6b9u, 0x3320u, 0x3812u, 0xfd8bu, 0x2e76u, 0xebefu, 0xe0ddu,
                                                   0x2544u, 0x02beu, 0xc727u, 0xcc15u, 0x098cu, 0xda71u, 0x1fe8u, 0x14dau, 0xd143u, 0xf3c5u, 0x365cu,
                                                   0x3d6eu, 0xf8f7u, 0x2b0au, 0xee93u, 0xe5a1u, 0x2038u, 0x07c2u, 0xc25bu, 0xc969u, 0x0cf0u, 0xdf0du,
                                                   0x1a94u, 0x11a6u, 0xd43fu, 0x5e52u, 0x9bcbu, 0x90f9u, 0x5560u, 0x869du, 0x4304u, 0x4836u, 0x8dafu,
                                                   0xaa55u, 0x6fccu, 0x64feu, 0xa167u, 0x729au, 0xb703u, 0xbc31u, 0x79a8u, 0xa8ebu, 0x6d72u, 0x6640u,
                                                   0xa3d9u, 0x7024u, 0xb5bdu, 0xbe8fu, 0x7b16u, 0x5cecu, 0x9975u, 0x9247u, 0x57deu, 0x8423u, 0x41bau,
                                                   0x4a88u, 0x8f11u, 0x057cu, 0xc0e5u, 0xcbd7u, 0x0e4eu, 0xddb3u, 0x182au, 0x1318u, 0xd681u, 0xf17bu,
                                                   0x34e2u, 0x3fd0u, 0xfa49u, 0x29b4u, 0xec2du, 0xe71fu, 0x2286u, 0xa213u, 0x678au, 0x6cb8u, 0xa921u,
                                                   0x7adcu, 0xbf45u, 0xb477u, 0x71eeu, 0x5614u, 0x938du, 0x98bfu, 0x5d26u, 0x8edbu, 0x4b42u, 0x4070u,
                                                   0x85e9u, 0x0f84u, 0xca1du, 0xc12fu, 0x04b6u, 0xd74bu, 0x12d2u, 0x19e0u, 0xdc79u, 0xfb83u, 0x3e1au, 0x3528u,
                                                   0xf0b1u, 0x234cu, 0xe6d5u, 0xede7u, 0x287eu, 0xf93du, 0x3ca4u, 0x3796u, 0xf20fu, 0x21f2u, 0xe46bu, 0xef59u,
                                                   0x2ac0u, 0x0d3au, 0xc8a3u, 0xc391u, 0x0608u, 0xd5f5u, 0x106cu, 0x1b5eu, 0xdec7u, 0x54aau, 0x9133u, 0x9a01u,
                                                   0x5f98u, 0x8c65u, 0x49fcu, 0x42ceu, 0x8757u, 0xa0adu, 0x6534u, 0x6e06u, 0xab9fu, 0x7862u, 0xbdfbu, 0xb6c9u,
                                                   0x7350u, 0x51d6u, 0x944fu, 0x9f7du, 0x5ae4u, 0x8919u, 0x4c80u, 0x47b2u, 0x822bu, 0xa5d1u, 0x6048u, 0x6b7au,
                                                   0xaee3u, 0x7d1eu, 0xb887u, 0xb3b5u, 0x762cu, 0xfc41u, 0x39d8u, 0x32eau, 0xf773u, 0x248eu, 0xe117u, 0xea25u,
                                                   0x2fbcu, 0x0846u, 0xcddfu, 0xc6edu, 0x0374u, 0xd089u, 0x1510u, 0x1e22u, 0xdbbbu, 0x0af8u, 0xcf61u, 0xc453u,
                                                   0x01cau, 0xd237u, 0x17aeu, 0x1c9cu, 0xd905u, 0xfeffu, 0x3b66u, 0x3054u, 0xf5cdu, 0x2630u, 0xe3a9u, 0xe89bu,
                                                   0x2d02u, 0xa76fu, 0x62f6u, 0x69c4u, 0xac5du, 0x7fa0u, 0xba39u, 0xb10bu, 0x7492u, 0x5368u, 0x96f1u, 0x9dc3u,
                                                   0x585au, 0x8ba7u, 0x4e3eu, 0x450cu, 0x8095u};

/* Pre-computed CRC10 Table */
static const uint16_t Adbms6948_Crc10Table[256] =
    {
        0x000, 0x08f, 0x11e, 0x191, 0x23c, 0x2b3, 0x322, 0x3ad, 0x0f7, 0x078, 0x1e9, 0x166, 0x2cb, 0x244, 0x3d5, 0x35a,
        0x1ee, 0x161, 0x0f0, 0x07f, 0x3d2, 0x35d, 0x2cc, 0x243, 0x119, 0x196, 0x007, 0x088, 0x325, 0x3aa, 0x23b, 0x2b4,
        0x3dc, 0x353, 0x2c2, 0x24d, 0x1e0, 0x16f, 0x0fe, 0x071, 0x32b, 0x3a4, 0x235, 0x2ba, 0x117, 0x198, 0x009, 0x086,
        0x232, 0x2bd, 0x32c, 0x3a3, 0x00e, 0x081, 0x110, 0x19f, 0x2c5, 0x24a, 0x3db, 0x354, 0x0f9, 0x076, 0x1e7, 0x168,
        0x337, 0x3b8, 0x229, 0x2a6, 0x10b, 0x184, 0x015, 0x09a, 0x3c0, 0x34f, 0x2de, 0x251, 0x1fc, 0x173, 0x0e2, 0x06d,
        0x2d9, 0x256, 0x3c7, 0x348, 0x0e5, 0x06a, 0x1fb, 0x174, 0x22e, 0x2a1, 0x330, 0x3bf, 0x012, 0x09d, 0x10c, 0x183,
        0x0eb, 0x064, 0x1f5, 0x17a, 0x2d7, 0x258, 0x3c9, 0x346, 0x01c, 0x093, 0x102, 0x18d, 0x220, 0x2af, 0x33e, 0x3b1,
        0x105, 0x18a, 0x01b, 0x094, 0x339, 0x3b6, 0x227, 0x2a8, 0x1f2, 0x17d, 0x0ec, 0x063, 0x3ce, 0x341, 0x2d0, 0x25f,
        0x2e1, 0x26e, 0x3ff, 0x370, 0x0dd, 0x052, 0x1c3, 0x14c, 0x216, 0x299, 0x308, 0x387, 0x02a, 0x0a5, 0x134, 0x1bb,
        0x30f, 0x380, 0x211, 0x29e, 0x133, 0x1bc, 0x02d, 0x0a2, 0x3f8, 0x377, 0x2e6, 0x269, 0x1c4, 0x14b, 0x0da, 0x055,
        0x13d, 0x1b2, 0x023, 0x0ac, 0x301, 0x38e, 0x21f, 0x290, 0x1ca, 0x145, 0x0d4, 0x05b, 0x3f6, 0x379, 0x2e8, 0x267,
        0x0d3, 0x05c, 0x1cd, 0x142, 0x2ef, 0x260, 0x3f1, 0x37e, 0x024, 0x0ab, 0x13a, 0x1b5, 0x218, 0x297, 0x306, 0x389,
        0x1d6, 0x159, 0x0c8, 0x047, 0x3ea, 0x365, 0x2f4, 0x27b, 0x121, 0x1ae, 0x03f, 0x0b0, 0x31d, 0x392, 0x203, 0x28c,
        0x038, 0x0b7, 0x126, 0x1a9, 0x204, 0x28b, 0x31a, 0x395, 0x0cf, 0x040, 0x1d1, 0x15e, 0x2f3, 0x27c, 0x3ed, 0x362,
        0x20a, 0x285, 0x314, 0x39b, 0x036, 0x0b9, 0x128, 0x1a7, 0x2fd, 0x272, 0x3e3, 0x36c, 0x0c1, 0x04e, 0x1df, 0x150,
        0x3e4, 0x36b, 0x2fa, 0x275, 0x1d8, 0x157, 0x0c6, 0x049, 0x313, 0x39c, 0x20d, 0x282, 0x12f, 0x1a0, 0x031, 0x0be};

uint16_t calculatePEC15(uint8_t *pDataBuf, uint8_t nLength)
{
    uint16_t nRemainder, nTableAddr;
    uint8_t nByteIndex;

    nRemainder = 16u; /* initialize the PEC */

    /* loops for each byte in data array */
    for (nByteIndex = 0u; nByteIndex < nLength; nByteIndex++)
    {
        /* calculate PEC table address */
        nTableAddr = (uint16_t)(((uint16_t)(nRemainder >> 7) ^ (uint8_t)pDataBuf[nByteIndex]) &
                                (uint8_t)0xff);
        nRemainder = (uint16_t)((nRemainder << 8) ^ Adbms6948_Crc15Table[nTableAddr]);
    }
    /* The CRC15 has a 0 in the LSB so the remainder must be multiplied by 2 */
    return (nRemainder * 2u);
}

uint16_t calculatePEC10(uint8_t *pDataBuf, uint8_t nLength, boolean bIsRxCmd)
{
    uint16_t nRemainder = 16u; /* PEC_SEED */
    /* x10 + x7 + x3 + x2 + x + 1 <- the CRC10 polynomial 100 1000 1111 */
    uint16_t nPolynomial = 0x8Fu;
    uint8_t nByteIndex, nBitIndex;
    uint16_t nTableAddr;

    for (nByteIndex = 0u; nByteIndex < nLength; ++nByteIndex)
    {
        /* calculate PEC table address */
        nTableAddr = (uint16_t)(((uint16_t)(nRemainder >> 2) ^ (uint8_t)pDataBuf[nByteIndex]) &
                                (uint8_t)0xff);
        nRemainder = (uint16_t)(((uint16_t)(nRemainder << 8)) ^ Adbms6948_Crc10Table[nTableAddr]);
    }
    /* If array is from received buffer add command counter to crc calculation */
    if (bIsRxCmd == true)
    {
        nRemainder ^= (uint16_t)(((uint16_t)pDataBuf[nLength] & (uint8_t)0xFC) << 2u);
    }
    /* Perform modulo-2 division, a bit at a time */
    for (nBitIndex = 6u; nBitIndex > 0u; --nBitIndex)
    {
        /* Try to divide the current data bit */
        if ((nRemainder & 0x200u) > 0u)
        {
            nRemainder = (uint16_t)((nRemainder << 1u));
            nRemainder = (uint16_t)(nRemainder ^ nPolynomial);
        }
        else
        {
            nRemainder = (uint16_t)((nRemainder << 1u));
        }
    }
    return ((uint16_t)(nRemainder & 0x3FFu));
}

void sendCommand(uint16_t nCommand)
{
    uint16_t nCmdPec;
    uint8_t aCmd[0x04u];

    aCmd[0] = (uint8_t)((nCommand & 0xFF00u) >> ADBMS6948_SHIFT_BY_8);
    aCmd[1] = (uint8_t)((nCommand & 0x00FFu));
    /* Calculate the 15-bit PEC for the command bytes */
    nCmdPec = (uint16_t)calculatePEC15(&aCmd[0], 2u);
    /* Append the PEC to the command buffer */
    aCmd[2] = (uint8_t)(nCmdPec >> ADBMS6948_SHIFT_BY_8);
    aCmd[3] = (uint8_t)(nCmdPec);

    // Start SPI transaction
    SPI1.beginTransaction(settings);
    digitalWrite(SPI1_CS, LOW);

    // SPI1.transfer(&nCommand, sizeof(nCommand));
    SPI1.transfer(&aCmd, sizeof(aCmd));

    // End SPI transaction
    SPI1.endTransaction();
    digitalWrite(SPI1_CS, HIGH);
}

// CMD0 CMD1 PEC0 PEC1 Data0 .. DataN DataPEC0 DataPEC1

// CMD0 = 00000 CMD10 CMD9 CMD8
// CMD1 = CMD7 CMD6 CMD5 CMD4 CMD3 CMD2 CMD1 CMD0
void writeData(uint16_t nCommand, uint8_t *pTxBuf)
{
    uint8_t aTxBuf[ADBMS6948_CMD_DATA_LEN +
                   (ADBMS6948_REG_DATA_LEN_WITH_PEC * ADBMS6948_MAX_NO_OF_DEVICES_IN_DAISY_CHAIN)];
    uint8_t nDevIndex, nByteIndex, nNoOfDevices;
    uint8_t *pDevCfgArray;
    uint16_t nCfgPec, nCmdPec;
    uint8_t nLen = 0u;

    nNoOfDevices = 1; // Assuming a single device for now

    // Populate the command part of the buffer
    aTxBuf[nLen++] = (uint8_t)((nCommand & 0xFF00U) >> 8U);
    aTxBuf[nLen++] = (uint8_t)(nCommand & 0x00FFU);

    // Calculate the 15-bit PEC for the command bytes
    nCmdPec = calculatePEC15(&aTxBuf[0], 2u);

    // Append the command PEC to the transmit buffer
    aTxBuf[nLen++] = (uint8_t)(nCmdPec >> 8U);
    aTxBuf[nLen++] = (uint8_t)(nCmdPec);

    // Fill the data part of the buffer for the single device
    pDevCfgArray = pTxBuf; // Point to the start of pTxBuf
    for (nByteIndex = 0u; nByteIndex < ADBMS6948_REG_GRP_LEN; nByteIndex++)
    {
        aTxBuf[nLen++] = *(pDevCfgArray + nByteIndex);
    }

    // Calculate the 10-bit PEC for the transmit data bytes for the single device
    nCfgPec = calculatePEC10(pDevCfgArray, false, ADBMS6948_REG_GRP_LEN);

    // Append the data PEC to the transmit buffer
    aTxBuf[nLen++] = (uint8_t)((nCfgPec & 0x0300u) >> 8u);
    aTxBuf[nLen++] = (uint8_t)(nCfgPec & 0x00FFu);

    // Debug: Print the aTxBuf contents
    Serial.print("aTxBuf: ");
    for (int i = 0; i < nLen; i++)
    {
        Serial.print(aTxBuf[i], HEX);
        Serial.print(" ");
    }
    Serial.println();

    // Start SPI transaction
    SPI1.beginTransaction(settings);
    digitalWrite(SPI1_CS, LOW);

    // Send the data
    SPI1.transfer(aTxBuf, nLen);

    // End SPI transaction
    SPI1.endTransaction();
    digitalWrite(SPI1_CS, HIGH);

    return;
}

void readData(uint16_t nCommand, uint8_t *pRxBuf, uint8_t numBytes)
{
    uint16_t nCmdPec;
    // 4 bytes for command, return bytes, 2 bytes for PEC
    uint8_t bytesToRead = numBytes + 2;
    uint8_t bufferSize = 4 + bytesToRead;
    uint8_t aCmd[bufferSize];

    // Populate the command part of the buffer
    aCmd[0] = (uint8_t)((nCommand & 0xFF00u) >> ADBMS6948_SHIFT_BY_8);
    aCmd[1] = (uint8_t)((nCommand & 0x00FFu));

    // Calculate the 15-bit PEC for the command bytes
    nCmdPec = (uint16_t)calculatePEC15(&aCmd[0], 2u);

    // Append the PEC to the command buffer
    aCmd[2] = (uint8_t)(nCmdPec >> ADBMS6948_SHIFT_BY_8);
    aCmd[3] = (uint8_t)(nCmdPec);

    // Fill the rest of the buffer with 0x00
    for (int i = 0; i < bytesToRead; i++)
    {
        aCmd[4 + i] = 0x00; // Uncomment if necessary
    }

    // Start SPI transaction
    SPI1.beginTransaction(settings);
    digitalWrite(SPI1_CS, LOW);

    // Send the data
    SPI1.transfer(aCmd, 12); // Correct buffer size

    // End SPI transaction
    digitalWrite(SPI1_CS, HIGH);
    SPI1.endTransaction();

    // Copy the received data to the pRxBuf
    for (int i = 0; i < numBytes; i++)
    {
        pRxBuf[i] = aCmd[4 + i];
    }
}

float convertToVoltage(uint8_t highByte, uint8_t lowByte)
{
    // Combine highByte and lowByte into a uint16_t
    uint16_t twoByteValue = (static_cast<uint16_t>(highByte) << 8) | lowByte;

    // Convert the 2-byte value to voltage
    float voltage = 150e-6 * twoByteValue + 1.5;
    return voltage;
}

void measureVoltages(uint8_t cellVoltageRegisters[])
{
    // Send ADCV command
    sendCommand(ADBMS6948_CMD_ADCV);

    // Wait for the ADC to finish
    delay(10);

    // Read voltages
    float cellVoltages[18];
    uint8_t n = 0;
    // Read cell voltages
    for (int i = 0; i < 6; i++)
    {
        uint8_t data[6];
        readData(cellVoltageRegisters[i], data, 6);
        cellVoltages[n] = convertToVoltage(data[1], data[0]);
        cellVoltages[n + 1] = convertToVoltage(data[3], data[2]);
        cellVoltages[n + 2] = convertToVoltage(data[5], data[4]);
        n += 3;
    }

    // Print cell voltages
    for (int i = 0; i < 16; i++)
    {
        Serial.print(cellVoltages[i], 4);
        Serial.print(" ");
    }
    Serial.println();
}

void setup()
{
    Serial.begin(115200);
    SPI1.begin();
    pinMode(SPI1_CS, OUTPUT);
    digitalWrite(SPI1_CS, LOW);
}

void loop()
{
    // // Define the register addresses
    // uint8_t cellVoltageRegisters[] = {RDCVA, RDCVB, RDCVC, RDCVD, RDCVE, RDCVF};

    // // Measure voltages
    // measureVoltages(cellVoltageRegisters);

    // // Wait before next measurement
    // delay(100);

    uint16_t nCommand = ADBMS6948_CMD_WRCFGA;
    uint8_t data[1] = {0x80};
    uint16_t nCmdPec;
    uint8_t bytesToWrite = 1;
    uint8_t bufferSize = 4 + bytesToWrite + 2;
    uint8_t aCmd[bufferSize];

    // Populate the command part of the buffer
    aCmd[0] = (uint8_t)((nCommand & 0xFF00u) >> ADBMS6948_SHIFT_BY_8);
    aCmd[1] = (uint8_t)((nCommand & 0x00FFu));

    // Calculate the 15-bit PEC for the command bytes
    nCmdPec = (uint16_t)calculatePEC15(&aCmd[0], 2u);

    // Append the PEC to the command buffer
    aCmd[2] = (uint8_t)(nCmdPec >> ADBMS6948_SHIFT_BY_8);
    aCmd[3] = (uint8_t)(nCmdPec);

    // Append the data to the command buffer
    aCmd[4] = data[0];

    // Calculate the PEC for the data
    uint16_t dataPec = calculatePEC10(data, bytesToWrite, false);

    // Append the data PEC to the command buffer
    aCmd[5] = (uint8_t)(dataPec >> ADBMS6948_SHIFT_BY_8);
    aCmd[6] = (uint8_t)(dataPec);

    // Start SPI transaction
    digitalWrite(SPI1_CS, LOW);
    SPI1.beginTransaction(settings);
    SPI1.transfer(aCmd, bufferSize);
    SPI1.endTransaction();
    digitalWrite(SPI1_CS, HIGH);

    delay(1000);

    // // read back the register
    // uint8_t rxData[1];
    // readData(nCommand, rxData, 1);

    // Serial.print("Data: ");
    // Serial.println(data[0], HEX);
    // delay(1000);

    // void readData(uint16_t nCommand, uint8_t *pRxBuf, uint8_t numBytes)
}
