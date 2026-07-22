#ifndef NRF24_TELEMETRY_H
#define NRF24_TELEMETRY_H

#include "config.h"
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include <cstdint>
#include <stdio.h>
#include <string.h>


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
    uint8_t reserved[9]; // Pad to 32 bytes (23 bytes used + 9 padding)
    uint8_t checksum;
};

struct RadioCommandPacket {
    uint8_t header1;     // 0xCC
    uint8_t header2;     // 0xDD
    uint8_t cmd_id;
    uint8_t payload[28];
    uint8_t checksum;
};

struct RadioResponsePacket {
    uint8_t header1;     // 0xDD
    uint8_t header2;     // 0xCC
    uint8_t cmd_id;
    uint8_t payload[28];
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
    const uint8_t EN_RXADDR = 0x02; // Enable RX addresses for pipes 0-5
    const uint8_t SETUP_RETR = 0x04;
    const uint8_t RF_CH = 0x05;
    const uint8_t RF_SETUP = 0x06;
    const uint8_t STATUS = 0x07;
    const uint8_t RX_ADDR_P0 = 0x0A;
    const uint8_t TX_ADDR = 0x10;
    const uint8_t RX_PW_P0 = 0x11;
    const uint8_t RX_PW_P1 = 0x12; // Payload size for pipe 1 (future use)
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
    bool readCommand(RadioCommandPacket* data);
    bool sendResponse(RadioResponsePacket* data);

    // ==========================================================
    // GROUND STATION FUNCTIONS (Ground -> Drone)
    // ==========================================================

    bool sendCommand(RadioCommandPacket* data);
    bool readResponse(RadioResponsePacket* data);
    bool readTelemetry(TelemetryPacket* data);
};

#endif // NRF24_TELEMETRY_H
