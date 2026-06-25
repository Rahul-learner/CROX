#pragma once
#include "pico/stdlib.h"
#include "hardware/spi.h"

// --- THE DATA PACKETS (Max 32 Bytes) ---

// Sent from Drone -> Ground Station (16 bytes)
struct TelemetryPacket {
    float roll;
    float pitch;
    float yaw;
    float battery_v;
};

// Received from Ground Station -> Drone (24 bytes)
struct PIDTuningPacket {
    float kp_roll;
    float ki_roll;
    float kd_roll;
    float kp_pitch;
    float ki_pitch;
    float kd_pitch;
};

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

    void init() {
        // NRF24 max SPI speed is 10MHz. 2MHz is very stable.
        spi_init(spi, 2000 * 1000);
        spi_set_format(spi, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
        sleep_ms(100);

        // Power on, enable CRC
        writeReg(CONFIG, 0x0A);
        sleep_ms(5);

        writeReg(EN_AA, 0x01);       // Enable Auto-ACK on pipe 0
        writeReg(SETUP_RETR, 0x2F);  // 750us delay, 15 retries
        writeReg(RF_CH, 76);         // Channel 76 (Out of WiFi range)
        writeReg(RF_SETUP, 0x07);    // 1Mbps, Max Power (0dBm)

        // Setup payload sizes (32 bytes max)
        writeReg(RX_PW_P0, sizeof(PIDTuningPacket));
    }

    // Set communication addresses (Match these exactly on your Ground Station!)
    void setAddresses(const uint8_t* tx_addr, const uint8_t* rx_addr) {
        csn_low();
        uint8_t cmd_tx = W_REG | TX_ADDR;
        spi_write_blocking(spi, &cmd_tx, 1);
        spi_write_blocking(spi, tx_addr, 5);
        csn_high();

        csn_low();
        uint8_t cmd_rx = W_REG | RX_ADDR_P0;
        spi_write_blocking(spi, &cmd_rx, 1);
        spi_write_blocking(spi, rx_addr, 5);
        csn_high();
    }

    void startListening() {
        writeReg(CONFIG, readReg(CONFIG) | 0x01); // Set PRX bit
        writeCmd(FLUSH_RX);
        writeReg(STATUS, 0x70); // Clear interrupts
        ce_high(); // Activate antenna to listen
        sleep_us(130);
    }

    void stopListening() {
        ce_low();
        writeReg(CONFIG, readReg(CONFIG) & ~0x01); // Clear PRX bit (Sets PTX)
        writeCmd(FLUSH_TX);
    }

    // Returns true if new PID data is waiting for us
    bool dataAvailable() {
        uint8_t status = readReg(STATUS);
        if (status & 0x40) { // RX_DR bit set
            writeReg(STATUS, 0x40); // Clear bit
            return true;
        }
        return false;
    }

    void readPID(PIDTuningPacket* data) {
        csn_low();
        uint8_t cmd = RX_PAYLOAD;
        spi_write_blocking(spi, &cmd, 1);
        spi_read_blocking(spi, 0x00, (uint8_t*)data, sizeof(PIDTuningPacket));
        csn_high();
    }

    // Send telemetry to the ground
    bool sendTelemetry(TelemetryPacket* data) {
        stopListening();

        csn_low();
        uint8_t cmd = TX_PAYLOAD;
        spi_write_blocking(spi, &cmd, 1);
        spi_write_blocking(spi, (uint8_t*)data, sizeof(TelemetryPacket));
        csn_high();

        ce_high(); // Pulse CE to transmit
        sleep_us(15);
        ce_low();

        // Non-blocking timeout wait for Auto-Ack
        uint32_t start_time = time_us_32();
        uint8_t status;
        while (true) {
            status = readReg(STATUS);
            if (status & 0x20) break; // TX_DS (Success)
            if (status & 0x10) break; // MAX_RT (Failed/No Ack)
            if (time_us_32() - start_time > 10000) break; // 10ms timeout
        }

        writeReg(STATUS, 0x30); // Clear TX interrupts
        startListening(); // Immediately go back to listening for PID tuning

        return (status & 0x20); // Return true if ground station acknowledged
    }
};
