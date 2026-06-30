#ifndef NRF24_TELEMETRY_H
#define NRF24_TELEMETRY_H

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include <cstdint>
#include <stdio.h>

// --- PACKET HEADERS ---
#define TELEMETRY_HEADER_1 0xAA
#define TELEMETRY_HEADER_2 0x55
#define PID_HEADER_1       0xBB
#define PID_HEADER_2       0x66

#pragma pack(push, 1) // Prevent compiler from adding padding bytes

// Sent from Drone -> Ground Station (23 bytes)
struct TelemetryPacket {
    uint8_t header1;
    uint8_t header2;
    int16_t roll;      // degrees * 100
    int16_t pitch;     // degrees * 100
    int16_t yaw;       // degrees * 100
    int16_t rc_roll;   // raw stick values
    int16_t rc_pitch;
    int16_t rc_yaw;
    int16_t pid_roll;  // raw motor correction output
    int16_t pid_pitch;
    int16_t pid_yaw;
    uint16_t dt_us;    // delta time in microseconds
    uint8_t checksum;
};

// Received from Ground Station -> Drone (21 bytes)
struct PIDTuningPacket {
    uint8_t header1;
    uint8_t header2;
    int16_t kp_roll;   // gains * 1000
    int16_t ki_roll;
    int16_t kd_roll;
    int16_t kp_pitch;
    int16_t ki_pitch;
    int16_t kd_pitch;
    int16_t kp_yaw;
    int16_t ki_yaw;
    int16_t kd_yaw;
    uint8_t checksum;
};

#pragma pack(pop)

class NRF24 {
private:
    spi_inst_t *spi;
    uint ce_pin, csn_pin;

    // NRF24 Commands
    const uint8_t R_REG = 0x00;
    const uint8_t W_REG = 0x20;
    const uint8_t RX_PAYLOAD = 0x61;
    const uint8_t TX_PAYLOAD = 0xA0;
    const uint8_t FLUSH_TX = 0xE1;
    const uint8_t FLUSH_RX = 0xE2;

    // NRF24 Registers
    const uint8_t CONFIG = 0x00;
    const uint8_t EN_AA = 0x01;
    const uint8_t SETUP_RETR = 0x04;
    const uint8_t RF_CH = 0x05;
    const uint8_t RF_SETUP = 0x06;
    const uint8_t STATUS = 0x07;
    const uint8_t RX_ADDR_P0 = 0x0A;
    const uint8_t TX_ADDR = 0x10;
    const uint8_t RX_PW_P0 = 0x11;

    inline void csn_low() { gpio_put(csn_pin, 0); asm volatile("nop \n nop \n nop"); }
    inline void csn_high() { gpio_put(csn_pin, 1); asm volatile("nop \n nop \n nop"); }
    inline void ce_low() { gpio_put(ce_pin, 0); }
    inline void ce_high() { gpio_put(ce_pin, 1); }

    void writeReg(uint8_t reg, uint8_t value) {
        uint8_t buf[2] = {(uint8_t)(W_REG | reg), value};
        csn_low();
        spi_write_blocking(spi, buf, 2);
        csn_high();
    }

    uint8_t readReg(uint8_t reg) {
        uint8_t buf = (R_REG | reg);
        uint8_t response;
        csn_low();
        spi_write_blocking(spi, &buf, 1);
        spi_read_blocking(spi, 0x00, &response, 1);
        csn_high();
        return response;
    }

    void writeCmd(uint8_t cmd) {
        csn_low();
        spi_write_blocking(spi, &cmd, 1);
        csn_high();
    }

public:
    NRF24(spi_inst_t* spi_port, uint ce, uint csn, uint sck, uint mosi, uint miso)
    : spi(spi_port), ce_pin(ce), csn_pin(csn) {

        gpio_set_function(sck, GPIO_FUNC_SPI);
        gpio_set_function(mosi, GPIO_FUNC_SPI);
        gpio_set_function(miso, GPIO_FUNC_SPI);

        gpio_init(ce_pin);
        gpio_set_dir(ce_pin, GPIO_OUT);
        ce_low();

        gpio_init(csn_pin);
        gpio_set_dir(csn_pin, GPIO_OUT);
        csn_high();
    }

    bool checkConnection() {
        spi_init(spi, 2000 * 1000);
        spi_set_format(spi, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
        sleep_ms(10);

        uint8_t test_channel = 88;
        writeReg(RF_CH, test_channel);
        sleep_ms(2);
        uint8_t response = readReg(RF_CH);

        if (response == test_channel) {
            return true;
        } else {
            printf("[NRF24 ERROR] SPI Verification Failed! Expected: %d, Got: %d\n", test_channel, response);
            return false;
        }
    }

    void init() {
        spi_init(spi, 2000 * 1000);
        spi_set_format(spi, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
        sleep_ms(100);

        writeReg(CONFIG, 0x0A);
        sleep_ms(5);

        writeReg(EN_AA, 0x03);       // Enable Auto-ACK on Pipe 0 AND Pipe 1
        writeReg(0x02, 0x03);        // EN_RXADDR: Enable receiving on Pipe 0 AND Pipe 1
        writeReg(SETUP_RETR, 0x2F);
        writeReg(RF_CH, 76);
        writeReg(RF_SETUP, 0x27);    // 250 kbps, Max Power (0dBm)

        writeReg(RX_PW_P0, 32);
        writeReg(0x12, 32);          // RX_PW_P1 Register
        // FEATURE register removed entirely to prevent clone chip lockups
    }

    void setAddresses(const uint8_t* tx_addr, const uint8_t* rx_addr) {
        csn_low();
        uint8_t cmd_tx = W_REG | TX_ADDR;
        spi_write_blocking(spi, &cmd_tx, 1);
        spi_write_blocking(spi, tx_addr, 5);
        csn_high();

        csn_low();
        uint8_t cmd_rx0 = W_REG | RX_ADDR_P0;
        spi_write_blocking(spi, &cmd_rx0, 1);
        spi_write_blocking(spi, tx_addr, 5);
        csn_high();

        csn_low();
        uint8_t cmd_rx1 = W_REG | 0x0B; // RX_ADDR_P1 Register
        spi_write_blocking(spi, &cmd_rx1, 1);
        spi_write_blocking(spi, rx_addr, 5);
        csn_high();
    }

    void startListening() {
        writeReg(CONFIG, readReg(CONFIG) | 0x01);
        writeCmd(FLUSH_RX);
        writeReg(STATUS, 0x70);
        ce_high();
        sleep_us(130);
    }

    void stopListening() {
        ce_low();
        writeReg(CONFIG, readReg(CONFIG) & ~0x01);
        writeCmd(FLUSH_TX);
    }

    bool dataAvailable() {
        uint8_t status = readReg(STATUS);
        bool has_data = ((status & 0x0E) != 0x0E);
        if (status & 0x40) {
            writeReg(STATUS, 0x40);
        }
        return has_data;
    }

    // ==========================================================
    // DRONE FUNCTIONS
    // ==========================================================

    // (Drone -> Ground)
    bool sendTelemetry(TelemetryPacket* data) {
        data->header1 = TELEMETRY_HEADER_1;
        data->header2 = TELEMETRY_HEADER_2;

        uint8_t calc_cs = 0;
        uint8_t* ptr = (uint8_t*)data;
        for (size_t i = 0; i < sizeof(TelemetryPacket) - 1; i++) calc_cs ^= ptr[i];
        data->checksum = calc_cs;

        uint8_t buffer[32] = {0};
        memcpy(buffer, data, sizeof(TelemetryPacket));

        stopListening();

        // --- CLONE BUG FIX: Disable Auto-ACK dynamically via Register ---
        uint8_t en_aa_bak = readReg(EN_AA);
        writeReg(EN_AA, en_aa_bak & ~0x01); // Temporarily turn off ACK for Pipe 0

        csn_low();
        uint8_t cmd = TX_PAYLOAD; // Reverted back to standard Command
        spi_write_blocking(spi, &cmd, 1);
        spi_write_blocking(spi, buffer, 32);
        csn_high();

        ce_high();
        sleep_us(15);
        ce_low();

        uint32_t start_time = time_us_32();
        uint8_t status;
        while (true) {
            status = readReg(STATUS);
            if (status & 0x20 || status & 0x10) break;
            if (time_us_32() - start_time > 10000) {
                printf("[NRF24 ERROR] Telemetry TX Hardware Timeout! Status: 0x%02X\n", status);
                break;
            }
        }

        writeReg(STATUS, 0x30);

        if (status & 0x10) {
            printf("[NRF24 ERROR] Telemetry TX MAX_RT hit! Buffer flushed.\n");
            writeCmd(FLUSH_TX);
        }

        writeReg(EN_AA, en_aa_bak); // Restore Auto-ACK state before listening
        startListening();

        return (status & 0x20);
    }

    // (Drone <- Ground)
    bool readPID(PIDTuningPacket* data) {
        uint8_t buffer[32] = {0};

        csn_low();
        uint8_t cmd = RX_PAYLOAD;
        spi_write_blocking(spi, &cmd, 1);
        spi_read_blocking(spi, 0x00, buffer, 32);
        csn_high();

        memcpy(data, buffer, sizeof(PIDTuningPacket));

        if (data->header1 != PID_HEADER_1 || data->header2 != PID_HEADER_2) {
            printf("[NRF24 ERROR] Invalid PID Header! Got: 0x%02X 0x%02X\n", data->header1, data->header2);
            return false;
        }

        uint8_t calc_cs = 0;
        uint8_t* ptr = (uint8_t*)data;
        for (size_t i = 0; i < sizeof(PIDTuningPacket) - 1; i++) calc_cs ^= ptr[i];

        if (calc_cs != data->checksum) {
            printf("[NRF24 ERROR] PID Checksum Mismatch! Calc: 0x%02X, Got: 0x%02X\n", calc_cs, data->checksum);
            return false;
        }

        return true;
    }

    // ==========================================================
    // GROUND STATION FUNCTIONS
    // ==========================================================

    // (Ground -> Drone)
    bool sendPID(PIDTuningPacket* data) {
        data->header1 = PID_HEADER_1;
        data->header2 = PID_HEADER_2;

        uint8_t calc_cs = 0;
        uint8_t* ptr = (uint8_t*)data;
        for (size_t i = 0; i < sizeof(PIDTuningPacket) - 1; i++) calc_cs ^= ptr[i];
        data->checksum = calc_cs;

        uint8_t buffer[32] = {0};
        memcpy(buffer, data, sizeof(PIDTuningPacket));

        stopListening();

        csn_low();
        uint8_t cmd = TX_PAYLOAD;
        spi_write_blocking(spi, &cmd, 1);
        spi_write_blocking(spi, buffer, 32);
        csn_high();

        ce_high();
        sleep_us(15);
        ce_low();

        uint32_t start_time = time_us_32();
        uint8_t status;
        while (true) {
            status = readReg(STATUS);
            if (status & 0x20 || status & 0x10) break;
            if (time_us_32() - start_time > 10000) {
                printf("[NRF24 ERROR] PID TX Hardware Timeout! Status: 0x%02X\n", status);
                break;
            }
        }

        writeReg(STATUS, 0x30);

        if (status & 0x10) {
            printf("[NRF24 ERROR] PID TX MAX_RT (Max Retries) hit! Drone offline or out of range.\n");
            writeCmd(FLUSH_TX);
        }

        startListening();

        return (status & 0x20);
    }

    // (Ground <- Drone)
    bool readTelemetry(TelemetryPacket* data) {
        uint8_t buffer[32] = {0};

        csn_low();
        uint8_t cmd = RX_PAYLOAD;
        spi_write_blocking(spi, &cmd, 1);
        spi_read_blocking(spi, 0x00, buffer, 32);
        csn_high();

        memcpy(data, buffer, sizeof(TelemetryPacket));

        if (data->header1 != TELEMETRY_HEADER_1 || data->header2 != TELEMETRY_HEADER_2) {
            printf("[NRF24 ERROR] Invalid Telemetry Header! Got: 0x%02X 0x%02X\n", data->header1, data->header2);
            return false;
        }

        uint8_t calc_cs = 0;
        uint8_t* ptr = (uint8_t*)data;
        for (size_t i = 0; i < sizeof(TelemetryPacket) - 1; i++) calc_cs ^= ptr[i];

        if (calc_cs != data->checksum) {
            printf("[NRF24 ERROR] Telemetry Checksum Mismatch! Calc: 0x%02X, Got: 0x%02X\n", calc_cs, data->checksum);
            return false;
        }

        return true;
    }
};

#endif // NRF24_TELEMETRY_H
