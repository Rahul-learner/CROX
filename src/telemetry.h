#pragma once
#include "config.h"
#include <stdint.h>
#include "hardware/uart.h"
#include "nrf24_telemetry.h" // Provides TelemetryPacket definition

// __attribute__((packed)) is critical here to prevent memory padding issues
struct __attribute__((packed)) ConfigPacket {
    uint8_t header1;       // 0xCC
    uint8_t header2;       // 0xDD
    int16_t roll_p, roll_i, roll_d;
    int16_t pitch_p, pitch_i, pitch_d;
    int16_t yaw_p, yaw_i, yaw_d;
    int16_t roll_bias;
    int16_t pitch_bias;
    uint8_t checksum;
};

// Instantiate the packet globally to avoid reallocation
extern TelemetryPacket telemetry_data;
extern ConfigPacket incomingConfig;
extern uint8_t* packet_ptr;
extern int packet_index;

class PIDController;

class Telemetry {
private:
    void setup_telemetry_uart();

public:
    Telemetry();

    // 2. The Transmission Function
    void send_telemetry(float roll, float pitch, float yaw, float dt, float desired_roll, float desired_pitch, float desired_yaw, float roll_pid, float pitch_pid, float yaw_pid);

    // --- Non-Blocking Read Function ---
    void process_incoming_config(PIDController& roll_pid, PIDController& pitch_pid, PIDController& yaw_pid, float& roll_bias, float& pitch_bias);
};

