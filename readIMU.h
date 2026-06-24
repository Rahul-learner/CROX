#ifndef BMI160_H
#define BMI160_H

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include <stdio.h>

class BMI160 {
private:
    spi_inst_t *spi;
    uint CS_PIN, SCK_PIN, MOSI_PIN, MISO_PIN;

    // --- BMI160 Registers ---
    const uint8_t REG_CHIP_ID = 0x00;
    const uint8_t REG_ERR_REG = 0x02;
    const uint8_t REG_PMU_STATUS = 0x03;
    const uint8_t REG_DATA_START = 0x0C; // Gyro X (Low Byte) starts here

    const uint8_t REG_ACCEL_CONF = 0x40;
    const uint8_t REG_ACCEL_RANGE = 0x41;
    const uint8_t REG_GYRO_CONF = 0x42;
    const uint8_t REG_GYRO_RANGE = 0x43;

    // Interrupt Registers
    const uint8_t REG_INT_EN_1 = 0x51;
    const uint8_t REG_INT_OUT_CTRL = 0x53;
    const uint8_t REG_INT_MAP_1 = 0x56;

    const uint8_t REG_CMD = 0x7E;

    // Expected Chip ID for BMI160
    const uint8_t EXPECTED_CHIP_ID = 0xD1;

    // --- Scaling Factors ---
    float accel_scale;
    float gyro_scale;

    inline void cs_select() {
        asm volatile("nop \n nop \n nop");
        gpio_put(CS_PIN, 0);
        asm volatile("nop \n nop \n nop");
    }

    inline void cs_deselect() {
        asm volatile("nop \n nop \n nop");
        gpio_put(CS_PIN, 1);
        asm volatile("nop \n nop \n nop");
    }

    void writeRegister(uint8_t reg, uint8_t data) {
        uint8_t buf[2];
        buf[0] = reg & 0x7F; // BMI160 Write: MSB = 0
        buf[1] = data;
        cs_select();
        spi_write_blocking(spi, buf, 2);
        cs_deselect();
    }

    uint8_t readRegister(uint8_t reg) {
        uint8_t reg_addr = reg | 0x80; // BMI160 Read: MSB = 1
        uint8_t data = 0;
        cs_select();
        spi_write_blocking(spi, &reg_addr, 1);
        spi_read_blocking(spi, 0x00, &data, 1);
        cs_deselect();
        return data;
    }

    void readRegisters(uint8_t reg, uint8_t* buffer, uint16_t length) {
        uint8_t reg_addr = reg | 0x80;
        cs_select();
        spi_write_blocking(spi, &reg_addr, 1);
        spi_read_blocking(spi, 0x00, buffer, length);
        cs_deselect();
    }

public:
    BMI160(spi_inst_t *spi_port, uint cs, uint sck, uint mosi, uint miso)
    : spi(spi_port), CS_PIN(cs), SCK_PIN(sck), MOSI_PIN(mosi), MISO_PIN(miso) {}

    bool checkConnection() {
        spi_init(spi, 1000 * 1000);
        spi_set_format(spi, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST); // BMI160 uses Mode 0!

        gpio_set_function(SCK_PIN, GPIO_FUNC_SPI);
        gpio_set_function(MOSI_PIN, GPIO_FUNC_SPI);
        gpio_set_function(MISO_PIN, GPIO_FUNC_SPI);

        gpio_init(CS_PIN);
        gpio_set_dir(CS_PIN, GPIO_OUT);
        gpio_put(CS_PIN, 1);

        sleep_ms(10);

        // CRITICAL BOSCH QUIRK: Must do a dummy read to force chip from I2C mode into SPI mode
        readRegister(0x7F);
        sleep_ms(1);

        uint8_t id = readRegister(REG_CHIP_ID);

        if (id == EXPECTED_CHIP_ID) {
            printf("BMI160 verified successfully (ID: 0x%02X).\n", id);
            return true;
        } else {
            printf("CRITICAL ERROR: BMI160 not found! WHO_AM_I returned: 0x%02X\n", id);
            return false;
        }
    }

    void init() {
        // 1. Soft Reset
        writeRegister(REG_CMD, 0xB6);
        sleep_ms(100); // Increased to ensure complete reboot

        // Dummy read to ensure SPI mode after reset
        readRegister(0x7F);
        sleep_ms(5);

        // 2. Clear any lingering PMU crashes in the Error Register
        readRegister(REG_ERR_REG);

        // --- 3. BOSCH BULLET-PROOF POWER SEQUENCE ---
        // MUST power up BEFORE configuring to avoid triggering a PMU Error

        writeRegister(REG_CMD, 0x11); // Power Up Accelerometer
        sleep_ms(50);

        writeRegister(REG_CMD, 0x15); // Power Up Gyroscope
        sleep_ms(100);

        // Verify Power State
        uint8_t pmu = readRegister(REG_PMU_STATUS);

        // 0x14 means BOTH sensors are awake. If not, aggressively loop until they are.
        while (pmu != 0x14) {
            printf("PMU stubborn (0x%02X). Forcing Wakeup...\n", pmu);
            writeRegister(REG_CMD, 0x11);
            sleep_ms(50);
            writeRegister(REG_CMD, 0x15);
            sleep_ms(100);
            pmu = readRegister(REG_PMU_STATUS);
        }
        printf("Sensors Power Online (0x14).\n");

        // 4. NOW it is safe to configure the ranges and speeds
        writeRegister(REG_ACCEL_RANGE, 0x08); // +/- 8G
        writeRegister(REG_ACCEL_CONF, 0x2C);  // ODR = 1600Hz, Normal filter
        accel_scale = 4096.0f; // 16-bit range over 8G

        writeRegister(REG_GYRO_RANGE, 0x00); // +/- 2000 dps
        writeRegister(REG_GYRO_CONF, 0x2D);  // ODR = 3200Hz (Max Speed), Normal filter
        gyro_scale = 16.4f; // 16-bit range over 2000dps

        // 5. Configure Interrupt Pin (INT1)
        writeRegister(REG_INT_OUT_CTRL, 0x0A); // INT1 push-pull, active HIGH
        writeRegister(REG_INT_MAP_1, 0x80);    // Map Data Ready to INT1
        writeRegister(REG_INT_EN_1, 0x10);     // Enable Data Ready Interrupt

        // 6. Max out SPI Speed (BMI160 handles up to 10MHz for data read)
        spi_set_baudrate(spi, 10 * 1000 * 1000);
    }

    // --- Fast Burst Read (12 bytes) ---
    void readData(float* ax, float* ay, float* az, float* gx, float* gy, float* gz) {
        uint8_t buffer[12];

        // Read 12 continuous bytes starting at Gyro X
        readRegisters(REG_DATA_START, buffer, 12);

        // CRITICAL BOSCH QUIRK: Data is LITTLE ENDIAN (Low Byte First)
        // Cast to uint16_t before ORing to prevent C++ from misinterpreting negative bits
        int16_t raw_gx = (int16_t)(((uint16_t)buffer[1] << 8)  | buffer[0]);
        int16_t raw_gy = (int16_t)(((uint16_t)buffer[3] << 8)  | buffer[2]);
        int16_t raw_gz = (int16_t)(((uint16_t)buffer[5] << 8)  | buffer[4]);

        int16_t raw_ax = (int16_t)(((uint16_t)buffer[7] << 8)  | buffer[6]);
        int16_t raw_ay = (int16_t)(((uint16_t)buffer[9] << 8)  | buffer[8]);
        int16_t raw_az = (int16_t)(((uint16_t)buffer[11] << 8) | buffer[10]);

        // Scale to G's
        *ax = (float)raw_ax / accel_scale;
        *ay = (float)raw_ay / accel_scale;
        *az = (float)raw_az / accel_scale;

        // Scale to Radians/sec for the EKF
        *gx = ((float)raw_gx / gyro_scale) * (3.14159265f / 180.0f);
        *gy = ((float)raw_gy / gyro_scale) * (3.14159265f / 180.0f);
        *gz = ((float)raw_gz / gyro_scale) * (3.14159265f / 180.0f);

        // Scale to Radians/sec for the EKF
        // *gx = (((float)raw_gx - (-0.2059)) / gyro_scale) * (3.14159265f / 180.0f);
        // *gy = (((float)raw_gy - (4.4470)) / gyro_scale) * (3.14159265f / 180.0f);
        // *gz = (((float)raw_gz  - (-6.3151)) / gyro_scale) * (3.14159265f / 180.0f);
    }
};

#endif // BMI160_H
