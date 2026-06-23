#pragma once
#include "pico/stdlib.h"
#include "hardware/spi.h"

class MPU6500 {
private:
    // --- Hardware Definitions ---
    spi_inst_t *spi;
    uint CS_PIN;
    uint SCK_PIN;
    uint MOSI_PIN;
    uint MISO_PIN;

    // --- MPU6000 Registers ---
    const uint8_t REG_USER_CTRL = 0x6A;
    const uint8_t REG_PWR_MGMT_1 = 0x6B;
    const uint8_t REG_CONFIG = 0x1A;
    const uint8_t REG_GYRO_CONFIG = 0x1B;
    const uint8_t REG_ACCEL_CONFIG = 0x1C;
    const uint8_t REG_ACCEL_XOUT_H = 0x3B;
    const uint8_t REG_INT_PIN_CFG = 0x37;
    const uint8_t REG_INT_ENABLE = 0x38;
    const uint8_t REG_WHO_AM_I = 0x75;
    const uint8_t EXPECTED_WHO_AM_I = 0x70;

    // --- SPI Read/Write Flags ---
    const uint8_t SPI_READ_BIT = 0x80;

    // --- Scaling Factors ---
    float accel_scale;
    float gyro_scale;

    // Fast inline functions for Chip Select management
    inline void cs_select() {
        asm volatile("nop \n nop \n nop"); // Tiny delay for electrical stability
        gpio_put(CS_PIN, 0);
        asm volatile("nop \n nop \n nop");
    }

    inline void cs_deselect() {
        asm volatile("nop \n nop \n nop");
        gpio_put(CS_PIN, 1);
        asm volatile("nop \n nop \n nop");
    }

    // --- Low-Level SPI Functions ---
    void writeRegister(uint8_t reg, uint8_t data) {
        uint8_t buf[2];
        buf[0] = reg & ~SPI_READ_BIT; // Ensure MSB is 0 for WRITE
        buf[1] = data;
        
        cs_select();
        spi_write_blocking(spi, buf, 2);
        cs_deselect();
    }

    uint8_t readRegister(uint8_t reg) {
        uint8_t reg_addr = reg | SPI_READ_BIT; // Set MSB to 1 for READ
        uint8_t data = 0;
        
        cs_select();
        spi_write_blocking(spi, &reg_addr, 1);
        spi_read_blocking(spi, 0x00, &data, 1);
        cs_deselect();
        return data;
    }

    void readRegisters(uint8_t reg, uint8_t* buffer, uint16_t length) {
        uint8_t reg_addr = reg | SPI_READ_BIT;
        
        cs_select();
        spi_write_blocking(spi, &reg_addr, 1);
        spi_read_blocking(spi, 0x00, buffer, length);
        cs_deselect();
    }

public:
    MPU6500(spi_inst_t *spi_port, uint cs, uint sck, uint mosi, uint miso)
        : spi(spi_port), CS_PIN(cs), SCK_PIN(sck), MOSI_PIN(mosi), MISO_PIN(miso) {}

    bool checkConnection() {
        // We must initialize the SPI pins briefly just to run this check,
        // in case checkConnection() is called before init()
        spi_init(spi, 1000 * 1000);
        spi_set_format(spi, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);

        gpio_set_function(SCK_PIN, GPIO_FUNC_SPI);
        gpio_set_function(MOSI_PIN, GPIO_FUNC_SPI);
        gpio_set_function(MISO_PIN, GPIO_FUNC_SPI);

        gpio_init(CS_PIN);
        gpio_set_dir(CS_PIN, GPIO_OUT);
        gpio_put(CS_PIN, 1);

        sleep_ms(10); // Brief power stabilization

        // Ask the chip for its hardcoded ID
        uint8_t id = readRegister(REG_WHO_AM_I);

        if (id == EXPECTED_WHO_AM_I) {
            printf("MPU-6500 verified successfully (ID: 0x%02X).\n", id);
            return true;
        } else {
            // Print the exact hex code it returned to help with debugging
            printf("CRITICAL ERROR: MPU-6500 not found! WHO_AM_I returned: 0x%02X\n", id);
            return false;
        }
    }

    void init() {
        // 1. Initialize Pico SPI Hardware at 1MHz (Safe speed for configuration)
        spi_init(spi, 1000 * 1000);
        
        // Format: 8 data bits, CPOL=1, CPHA=1 (SPI Mode 3 is standard for MPU6500)
        spi_set_format(spi, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);

        gpio_set_function(SCK_PIN, GPIO_FUNC_SPI);
        gpio_set_function(MOSI_PIN, GPIO_FUNC_SPI);
        gpio_set_function(MISO_PIN, GPIO_FUNC_SPI);

        // Initialize Chip Select as standard GPIO output
        gpio_init(CS_PIN);
        gpio_set_dir(CS_PIN, GPIO_OUT);
        gpio_put(CS_PIN, 1); // Deselect to start

        sleep_ms(100); // Give sensor time to power up

        // 2. MPU-6000 Configuration Sequence
        writeRegister(REG_PWR_MGMT_1, 0x80); // Reset the device
        sleep_ms(100);
        
        writeRegister(REG_PWR_MGMT_1, 0x01); // Wake up, use X-Axis Gyro Clock (Most stable)
        sleep_ms(10);

        writeRegister(REG_USER_CTRL, 0x10);  // CRITICAL: Disable I2C interface, force SPI only
        sleep_ms(1);

        writeRegister(REG_CONFIG, 0x00);     // Set DLPF (Digital Low Pass Filter) to 42Hz
        sleep_ms(1);

        writeRegister(REG_GYRO_CONFIG, 0x18);  // Gyro scale: +/- 2000 deg/sec
        gyro_scale = 16.4f;                    // LSB per deg/sec

        writeRegister(REG_ACCEL_CONFIG, 0x10); // Accel scale: +/- 8g
        accel_scale = 4096.0f;                 // LSB per g

        // 1. Configure the Interrupt Pin Logic
        // 0x10 means: Active HIGH, Push-Pull, and clear the pin the moment ANY read occurs
        writeRegister(REG_INT_PIN_CFG, 0x10); 
        sleep_ms(1);

        // 2. Enable the "Data Ready" Interrupt
        // 0x01 means: Fire the INT pin every time a new sensor reading is generated
        writeRegister(REG_INT_ENABLE, 0x01);
        sleep_ms(1);

        // 3. Shift SPI bus into high gear! 
        // Sensor allows up to 20MHz for register reading. We will set 10MHz for rock-solid stability.
        spi_set_baudrate(spi, 10 * 1000 * 1000);
    }

    // --- Fast Burst Read (Reads all 14 bytes of Accel, Temp, and Gyro in one massive shot) ---
    void readData(float* ax, float* ay, float* az, float* gx, float* gy, float* gz) {
        uint8_t buffer[14];
        readRegisters(REG_ACCEL_XOUT_H, buffer, 14);

        // Reconstruct 16-bit integers from the High and Low bytes
        int16_t raw_ax = (buffer[0] << 8) | buffer[1];
        int16_t raw_ay = (buffer[2] << 8) | buffer[3];
        int16_t raw_az = (buffer[4] << 8) | buffer[5];
        
        // bytes [6] and [7] are the Temperature sensor. We can skip them.
        
        int16_t raw_gx = (buffer[8] << 8) | buffer[9];
        int16_t raw_gy = (buffer[10] << 8) | buffer[11];
        int16_t raw_gz = (buffer[12] << 8) | buffer[13];

        // Scale to standard physics units (G's and Radians/sec)
        // Note: EKF requires Gyro in radians!
        *ax = (float)raw_ax / accel_scale;
        *ay = (float)raw_ay / accel_scale;
        *az = (float)raw_az / accel_scale;

        *gx = ((float)raw_gx / gyro_scale) * (3.14159265f / 180.0f);
        *gy = ((float)raw_gy / gyro_scale) * (3.14159265f / 180.0f);
        *gz = ((float)raw_gz / gyro_scale) * (3.14159265f / 180.0f);
    }
};

class BMI160 {
private:
    spi_inst_t *spi;
    uint CS_PIN, SCK_PIN, MOSI_PIN, MISO_PIN;

    // --- BMI160 Registers ---
    const uint8_t REG_CHIP_ID = 0x00;
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
        sleep_ms(50); // Give it plenty of time to reboot

        // Dummy read to ensure SPI mode after reset
        readRegister(0x7F);

        // 2. Power Up Accelerometer (Normal Mode)
        writeRegister(REG_CMD, 0x11);
        sleep_ms(10);

        // 3. Power Up Gyroscope (Normal Mode)
        writeRegister(REG_CMD, 0x15);
        sleep_ms(100); // Gyro takes 80ms+ to spin up physically

        // 4. Configure Accel (+/- 8G, 1600Hz ODR)
        writeRegister(REG_ACCEL_RANGE, 0x08); // +/- 8G
        writeRegister(REG_ACCEL_CONF, 0x2C);  // ODR = 1600Hz, Normal filter
        accel_scale = 4096.0f; // 16-bit range over 8G

        // 5. Configure Gyro (+/- 2000 dps, 3200Hz ODR)
        writeRegister(REG_GYRO_RANGE, 0x00); // +/- 2000 dps
        writeRegister(REG_GYRO_CONF, 0x2D);  // ODR = 3200Hz (Max Speed), Normal filter
        gyro_scale = 16.4f; // 16-bit range over 2000dps

        // 6. Configure Interrupt Pin (INT1)
        writeRegister(REG_INT_OUT_CTRL, 0x0A); // INT1 push-pull, active HIGH
        writeRegister(REG_INT_MAP_1, 0x80);    // Map Data Ready to INT1
        writeRegister(REG_INT_EN_1, 0x10);     // Enable Data Ready Interrupt

        // 7. Max out SPI Speed (BMI160 handles up to 10MHz for data read)
        spi_set_baudrate(spi, 10 * 1000 * 1000);
    }

    // --- Fast Burst Read (12 bytes) ---
    void readData(float* ax, float* ay, float* az, float* gx, float* gy, float* gz) {
        uint8_t buffer[12];

        // Read 12 continuous bytes starting at Gyro X
        readRegisters(REG_DATA_START, buffer, 12);

        // CRITICAL BOSCH QUIRK: Data is LITTLE ENDIAN (Low Byte First)
        // MPU6000 was (buffer[0] << 8 | buffer[1]). BMI160 is reversed!
        int16_t raw_gx = (buffer[1] << 8)  | buffer[0];
        int16_t raw_gy = (buffer[3] << 8)  | buffer[2];
        int16_t raw_gz = (buffer[5] << 8)  | buffer[4];

        int16_t raw_ax = (buffer[7] << 8)  | buffer[6];
        int16_t raw_ay = (buffer[9] << 8)  | buffer[8];
        int16_t raw_az = (buffer[11] << 8) | buffer[10];

        // Scale to G's
        *ax = (float)raw_ax / accel_scale;
        *ay = (float)raw_ay / accel_scale;
        *az = (float)raw_az / accel_scale;

        // Scale to Radians/sec for the EKF
        *gx = ((float)raw_gx / gyro_scale) * (3.14159265f / 180.0f);
        *gy = ((float)raw_gy / gyro_scale) * (3.14159265f / 180.0f);
        *gz = ((float)raw_gz / gyro_scale) * (3.14159265f / 180.0f);
    }
};
