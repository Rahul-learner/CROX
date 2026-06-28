#pragma once
#include <stdint.h>
#include "hardware/uart.h"

#define UART_ID uart0 // For UART connection
#define UART_TX_PIN 16
#define UART_RX_PIN 17
#define BAUD_RATE 38400

// 1. Define the Binary Packet Structure
#pragma pack(push, 1) // Force 1-byte alignment
struct TelemetryPacket {
    uint8_t header1 = 0xAA;  // Sync byte 1
    uint8_t header2 = 0xBB;  // Sync byte 2
    int16_t roll;            // Roll * 100
    int16_t pitch;           // Pitch * 100
    int16_t yaw;             // Yaw * 100
    int16_t dt;
    int16_t desired_roll;
    int16_t desired_pitch;
    int16_t desired_yaw;
    int16_t roll_pid;
    int16_t pitch_pid;
    int16_t yaw_pid;
    uint8_t checksum;        // XOR of the payload
};

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
#pragma pack(pop) // Restore default alignment

// Instantiate the packet globally to avoid reallocation
TelemetryPacket telemetry_data;
ConfigPacket incomingConfig;
uint8_t* packet_ptr = (uint8_t*)&incomingConfig;
int packet_index = 0;

class Telemetry {
private:
    void setup_telemetry_uart() {
        // 1. Initialize the UART hardware block at the target baud rate
        uart_init(UART_ID, BAUD_RATE);

        // 2. Tell the GPIO pins to switch from standard I/O mode into UART mode
        gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
        gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

        // 3. Configure the UART format (8 data bits, 1 stop bit, no parity)
        uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);

        // 4. Disable hardware flow control (we only have TX and RX wires, no RTS/CTS)
        uart_set_hw_flow(UART_ID, false, false);

        // 5. Enable the internal FIFO buffer
        // This allows you to dump your 9-byte struct into the hardware instantly
        // without blocking your flight controller's math loop.
        uart_set_fifo_enabled(UART_ID, true);
    }

public:
    Telemetry() {
        setup_telemetry_uart();
    }

    // 2. The Transmission Function
    void send_telemetry(float roll, float pitch, float yaw, float dt, float desired_roll, float desired_pitch, float desired_yaw, float roll_pid, float pitch_pid, float yaw_pid) {
        // Quantize floats to integers (preserves 2 decimal places)
        telemetry_data.roll = (int16_t)(roll * 100.0f);
        telemetry_data.pitch = (int16_t)(pitch * 100.0f);
        telemetry_data.yaw = (int16_t)(yaw * 100.0f);
        telemetry_data.dt = (int16_t)(dt * 1000000.0f);
        telemetry_data.desired_roll = (int16_t)(desired_roll * 100.0f);
        telemetry_data.desired_pitch = (int16_t)(desired_pitch * 100.0f);
        telemetry_data.desired_yaw = (int16_t)(desired_yaw * 100.0f);
        telemetry_data.roll_pid = (int16_t)(roll_pid * 100.0f);
        telemetry_data.pitch_pid = (int16_t)(pitch_pid * 100.0f);
        telemetry_data.yaw_pid = (int16_t)(yaw_pid * 100.0f);

        // Calculate Checksum (XOR all payload bytes together)
        // We cast the payload to an array of bytes to iterate over them
        uint8_t* payload_bytes = (uint8_t*)&telemetry_data.roll;
        uint8_t calc_checksum = 0;

        // 6 bytes total for three 16-bit integers
        for (int i = 0; i < 20; i++) {
            calc_checksum ^= payload_bytes[i];
        }
        telemetry_data.checksum = calc_checksum;

        // 3. Blast it over UART
        // Send the memory block directly to the hardware
        uart_write_blocking(UART_ID, (uint8_t*)&telemetry_data, sizeof(TelemetryPacket));
    }

    // --- Non-Blocking Read Function ---
    void process_incoming_config(PIDController& roll_pid, PIDController& pitch_pid, PIDController& yaw_pid, float& roll_bias, float& pitch_bias) {
        // uart_is_readable() instantly returns true or false.
        // It will NEVER pause your flight controller loop waiting for data.
        while (uart_is_readable(UART_ID)) {
            uint8_t c = uart_getc(UART_ID);

            // 1. Wait for Header 1 (0xCC)
            if (packet_index == 0 && c != 0xCC) continue;

            // 2. Wait for Header 2 (0xDD)
            if (packet_index == 1 && c != 0xDD) {
                packet_index = 0;
                continue;
            }

            // 3. Store the byte directly into the struct memory
            packet_ptr[packet_index++] = c;

            // 4. Packet is complete
            if (packet_index == sizeof(ConfigPacket)) {
                packet_index = 0; // Reset for the next incoming packet

                // 5. Verify the Checksum
                uint8_t calc_checksum = 0;
                // Start reading from roll_p, read for 22 bytes (the payload)
                uint8_t* payload = (uint8_t*)&incomingConfig.roll_p;
                for (int i = 0; i < 22; i++) {
                    calc_checksum ^= payload[i];
                }

                // 6. Apply settings if data is uncorrupted
                if (calc_checksum == incomingConfig.checksum) {
                    // Convert back to floating point math
                    float rp_p = incomingConfig.roll_p / 100.0f;
                    float rp_i = incomingConfig.roll_i / 100.0f;
                    float rp_d = incomingConfig.roll_d / 100.0f;

                    float y_p = incomingConfig.yaw_p / 100.0f;
                    float y_i = incomingConfig.yaw_i / 100.0f;
                    float y_d = incomingConfig.yaw_d / 100.0f;

                    roll_bias = incomingConfig.roll_bias / 100.0f;
                    pitch_bias = incomingConfig.pitch_bias / 100.0f;

                    printf("SUCCESS! New Roll Bias updated to: %.2f\n", roll_bias);
                    printf("SUCCESS! New Pitch Bias updated to: %.2f\n", pitch_bias);

                    // Updating the PID
                    roll_pid.set_pid(rp_p, rp_i, rp_d);
                    pitch_pid.set_pid(rp_p, rp_i, rp_d);
                    yaw_pid.set_pid(y_p, y_i, y_d);


                } else {
                    printf("Config Checksum failed! Electrical noise detected.\n");
                }
            }
        }
    }
};
