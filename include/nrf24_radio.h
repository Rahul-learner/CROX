#ifndef NRF24_TELEMETRY_H
#define NRF24_TELEMETRY_H

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include <cstdint>
#include <stdio.h>
#include <string.h>

// --- PACKET HEADERS ---
#define TELEMETRY_HEADER_1 0xAA
#define TELEMETRY_HEADER_2 0x55
#define PID_HEADER_1       0xBB
#define PID_HEADER_2       0x66

#pragma pack(push, 1)

struct TelemetryPacket {
    uint8_t header1;
    uint8_t header2;
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
    int16_t rc_roll;
    int16_t rc_pitch;
    int16_t rc_yaw;
    int16_t pid_roll;
    int16_t pid_pitch;
    int16_t pid_yaw;
    uint16_t dt_s;
    uint8_t checksum;
};

struct PIDTuningPacket {
    uint8_t header1;
    uint8_t header2;
    int16_t kp_roll_pitch;
    int16_t ki_roll_pitch;
    int16_t kd_roll_pitch;
    int16_t kp_yaw;
    int16_t ki_yaw;
    int16_t kd_yaw;
    int16_t bias_roll;
    int16_t bias_pitch;
    int16_t bias_yaw;
    uint8_t checksum;
};

#pragma pack(pop)

class NRF24 {
private:
    spi_inst_t *spi;
    uint ce_pin, csn_pin;
    bool _is_sending = false;

    // NRF24 Commands
    const uint8_t R_REG = 0x00;
    const uint8_t W_REG = 0x20;
    const uint8_t RX_PAYLOAD = 0x61;
    const uint8_t TX_PAYLOAD = 0xA0; // We will strictly use standard payloads now
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
    const uint8_t FEATURE = 0x1D;

    inline void csn_low() { gpio_put(csn_pin, 0); asm volatile("nop \n nop \n nop"); }
    inline void csn_high() { gpio_put(csn_pin, 1); asm volatile("nop \n nop \n nop"); }
    inline void ce_low() { gpio_put(ce_pin, 0); }
    inline void ce_high() { gpio_put(ce_pin, 1); }

    void writeReg(uint8_t reg, uint8_t value);
    uint8_t readReg(uint8_t reg);
    void writeCmd(uint8_t cmd);
    bool isSending();
    bool isReady();

public:
    NRF24(spi_inst_t* spi_port, uint ce, uint csn, uint sck, uint mosi, uint miso);
    bool checkConnection();
    void init();
    void setAddresses(const uint8_t* tx_addr, const uint8_t* rx_addr);
    void startListening();
    void stopListening();

    // --- NEW: Hardware Soft Reset ---
    void restart();
    bool dataAvailable();

    // ==========================================================
    // DRONE FUNCTIONS (Drone -> Ground)
    // ==========================================================

    bool sendTelemetry(TelemetryPacket* data);
    bool readPID(PIDTuningPacket* data);

    // ==========================================================
    // GROUND STATION FUNCTIONS (Ground -> Drone)
    // ==========================================================

    bool sendPID(PIDTuningPacket* data);
    bool readTelemetry(TelemetryPacket* data);
};

#endif // NRF24_TELEMETRY_H
