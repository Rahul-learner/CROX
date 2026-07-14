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

    void writeRegister(uint8_t reg, uint8_t data);
    uint8_t readRegister(uint8_t reg);
    void readRegisters(uint8_t reg, uint8_t* buffer, uint16_t length);

public:
    BMI160(spi_inst_t *spi_port, uint cs, uint sck, uint mosi, uint miso)
    : spi(spi_port), CS_PIN(cs), SCK_PIN(sck), MOSI_PIN(mosi), MISO_PIN(miso) {}

    bool checkConnection();
    void init();
    void readData(float* ax, float* ay, float* az, float* gx, float* gy, float* gz);
};

#endif // BMI160_H
