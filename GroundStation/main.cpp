#include <cstdint>
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "nrf24_telemetry.h"

// --- NRF24 Hardware Pin Definitions ---
#define NRF_SPI_PORT spi1
#define NRF_MISO_PIN 12
#define NRF_CSN_PIN  13
#define NRF_SCK_PIN  10
#define NRF_MOSI_PIN 11
#define NRF_CE_PIN   14

NRF24 radio(NRF_SPI_PORT, NRF_CE_PIN, NRF_CSN_PIN, NRF_SCK_PIN, NRF_MOSI_PIN, NRF_MISO_PIN);

// --- Tuning Variables & Flags ---
// Defaults match those in DroneFC/RP2350/config/config.h
float bias_roll = -3.95f, bias_pitch = 1.87f, bias_yaw = 0.0f;
float q_gyro = 0.001f, q_bias = 0.00001f, r_accel = 20.0f;
float pid_p_roll_pitch = 1.95f, pid_i_roll_pitch = 0.15f, pid_d_roll_pitch = 0.47f;
float pid_p_yaw = 2.1f, pid_i_yaw = 0.1f, pid_d_yaw = 0.0f;
float angle_strength = 4.5f, angle_max_deg = 45.0f, angle_max_rate = 360.0f;
bool send_tuning_flag = false;
uint8_t send_tuning_mask = 0;
bool send_angle_flag = false;
bool radio_restarted = false;

// ============================================================================
// USB SERIAL COMMAND PARSING
// ============================================================================

// Parse the received usb string
void process_command(char* buffer) {
    if (strncmp(buffer, "EKF,", 4) == 0) {
        sscanf(buffer, "EKF,%f,%f,%f", &q_gyro, &q_bias, &r_accel);
        printf("ACK EKF: %f, %f, %f\n", q_gyro, q_bias, r_accel);
        send_tuning_mask |= 0x08;
        send_tuning_flag = true;
    }
    // NEW: Roll & Pitch Acro PID
    else if (strncmp(buffer, "PID_ACRO,", 9) == 0) {
        sscanf(buffer, "PID_ACRO,%f,%f,%f", &pid_p_roll_pitch, &pid_i_roll_pitch, &pid_d_roll_pitch);
        printf("ACK PID_ACRO: %f, %f, %f\n", pid_p_roll_pitch, pid_i_roll_pitch, pid_d_roll_pitch);
        send_tuning_mask |= 0x01;
        send_tuning_flag = true;
    }
    // NEW: Angle Tuning
    else if (strncmp(buffer, "ANGLE_TUNE,", 11) == 0) {
        sscanf(buffer, "ANGLE_TUNE,%f,%f,%f", &angle_strength, &angle_max_deg, &angle_max_rate);
        printf("ACK ANGLE_TUNE: %f, %f, %f\n", angle_strength, angle_max_deg, angle_max_rate);
        send_angle_flag = true;
    }
    // NEW: Yaw PID
    else if (strncmp(buffer, "PID_YAW,", 8) == 0) {
        sscanf(buffer, "PID_YAW,%f,%f,%f", &pid_p_yaw, &pid_i_yaw, &pid_d_yaw);
        printf("ACK PID_YAW: %f, %f, %f\n", pid_p_yaw, pid_i_yaw, pid_d_yaw);
        send_tuning_mask |= 0x02;
        send_tuning_flag = true;
    }
    else if (strncmp(buffer, "BIAS,", 5) == 0) {
        sscanf(buffer, "BIAS,%f,%f,%f", &bias_roll, &bias_pitch, &bias_yaw);
        printf("ACK BIAS: R:%f, P:%f, Y:%f\n", bias_roll, bias_pitch, bias_yaw);
        send_tuning_mask |= 0x04;
        send_tuning_flag = true;
    }
}

// Non-blocking serial listener
void check_serial_commands() {
    static char rx_buffer[128];
    static int rx_index = 0;
    int c;

    // Read characters until the buffer is empty (timeout of 0 microseconds)
    while ((c = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
        if (c == '\n' || c == '\r') {
            if (rx_index > 0) {
                rx_buffer[rx_index] = '\0'; // Terminate string
                process_command(rx_buffer); // Parse it
                rx_index = 0;               // Reset for next command
            }
        } else if (rx_index < sizeof(rx_buffer) - 1) {
            rx_buffer[rx_index++] = (char)c;
        }
    }
}

// ============================================================================
// MAIN GROUND STATION LOOP
// ============================================================================

int main() {
    stdio_init_all();

    // Give your PC extra time to recognize the USB COM port before printing
    sleep_ms(3000);

    printf("\n======================================\n");
    printf("   GROUND STATION BOOTING (RX)        \n");
    printf("======================================\n");

    // 1. Verify SPI Hardware Connection
    while (!radio.checkConnection()) {
        printf("ERROR: NRF24 not responding! Check SPI wiring.\n");
        sleep_ms(2000);
    }
    printf("NRF24 Hardware Verified Successfully!\n");

    radio.init();

    // 2. Setup Addresses (Mirrored from Drone)
    const uint8_t drone_addr[5]  = {'D', 'R', 'O', 'N', '1'};
    const uint8_t ground_addr[5] = {'G', 'R', 'N', 'D', '1'};

    // Ground Station Transmits TO Drone, Receives AS Ground
    radio.setAddresses(drone_addr, ground_addr);

    radio.startListening();

    printf("Ground Station Online. Listening for live Telemetry...\n");
    printf("Type 'PID,1.5,0.05,0.2' and press Enter to send tuning data.\n\n");

    TelemetryPacket incoming_telemetry = {};
    PIDTuningPacket outgoing_pids = {};
    AngleTuningPacket outgoing_angle = {};
    uint32_t last_heartbeat = time_us_32();
    float last_dt_s = 0.0f;


    while (true) {
        // 1. Check for user typing in the Serial Console
        check_serial_commands();

        // 2. If user updated the PIDs, send them to the drone!
        if (send_tuning_flag) {
            radio_restarted = false;
            // Pack the floats into 16-bit integers (* 1000)
            int16_t p_roll_pitch = (int16_t)(pid_p_roll_pitch * 1000.0f);
            int16_t i_roll_pitch = (int16_t)(pid_i_roll_pitch * 1000.0f);
            int16_t d_roll_pitch = (int16_t)(pid_d_roll_pitch * 1000.0f);
            int16_t p_yaw = (int16_t)(pid_p_yaw * 1000.0f);
            int16_t i_yaw = (int16_t)(pid_i_yaw * 1000.0f);
            int16_t d_yaw = (int16_t)(pid_d_yaw * 1000.0f);
            int16_t bias_roll_rec = (int16_t)(bias_roll * 1000.0f);
            int16_t bias_pitch_rec = (int16_t)(bias_pitch * 1000.0f);
            int16_t bias_yaw_rec = (int16_t)(bias_yaw * 1000.0f);
            int16_t ekf_q_gyro = (int16_t)(q_gyro * 100000.0f);
            int16_t ekf_q_bias = (int16_t)(q_bias * 1000000.0f);
            int16_t ekf_r_accel = (int16_t)(r_accel * 100.0f);

            // Apply to all axes (or you can separate them in your command string later)
            outgoing_pids.update_mask = send_tuning_mask;
            outgoing_pids.kp_roll_pitch = p_roll_pitch; outgoing_pids.ki_roll_pitch = i_roll_pitch; outgoing_pids.kd_roll_pitch = d_roll_pitch;
            outgoing_pids.kp_yaw = p_yaw; outgoing_pids.ki_yaw = i_yaw; outgoing_pids.kd_yaw = d_yaw;
            outgoing_pids.bias_roll = bias_roll_rec ; outgoing_pids.bias_pitch = bias_pitch_rec; outgoing_pids.bias_yaw = bias_yaw_rec;
            outgoing_pids.q_gyro = ekf_q_gyro; outgoing_pids.q_bias = ekf_q_bias; outgoing_pids.r_accel = ekf_r_accel;

            printf("\nBroadcasting new PIDs to Drone...\n");

            if (radio.sendPID(&outgoing_pids)) {
                printf("--> SUCCESS: Drone received and Acknowledged PIDs!\n\n");
            } else {
                printf("--> FAILED: Drone offline or out of range. PIDs dropped.\n\n");
            }

            send_tuning_flag = false;
            send_tuning_mask = 0;
        }

        if (send_angle_flag) {
            radio_restarted = false;
            outgoing_angle.angle_strength = (int16_t)(angle_strength * 1000.0f);
            outgoing_angle.angle_max_deg = (int16_t)(angle_max_deg * 10.0f);
            outgoing_angle.angle_max_rate = (int16_t)(angle_max_rate * 10.0f);

            printf("\nBroadcasting new Angle Tuning to Drone...\n");

            if (radio.sendAngleTune(&outgoing_angle)) {
                printf("--> SUCCESS: Drone received and Acknowledged Angle Tuning!\n\n");
            } else {
                printf("--> FAILED: Drone offline or out of range. Angle Tuning dropped.\n\n");
            }

            send_angle_flag = false;
        }
        // 3. Check for incoming Telemetry from the drone
        while (radio.dataAvailable()) {
            radio_restarted = false;


            if (radio.readTelemetry(&incoming_telemetry)) {
                float current_dt_s = incoming_telemetry.dt_s / 100.0f;
                float dt = (current_dt_s - last_dt_s);

                // Print the live data to the serial monitor
                printf("Roll: %.2f, Pitch: %.2f, Yaw: %.2f, RC_Roll: %.2f, RC_Pitch: %.2f, RC_Yaw: %.2f, PID_Roll: %.2f, PID_Pitch: %.2f, PID_Yaw: %.2f, current_dt_s: %.2f, dt: %f\n",
                       incoming_telemetry.roll / 100.0f,
                       incoming_telemetry.pitch / 100.0f,
                       incoming_telemetry.yaw / 100.0f,
                       incoming_telemetry.rc_roll / 100.0f,
                       incoming_telemetry.rc_pitch / 100.0f,
                       incoming_telemetry.rc_yaw / 100.0f,
                       incoming_telemetry.pid_roll / 100.0f,
                       incoming_telemetry.pid_pitch / 100.0f,
                       incoming_telemetry.pid_yaw / 100.0f,
                       current_dt_s,
                       dt);
                last_dt_s = current_dt_s;
                last_heartbeat = time_us_32();
            } else {
                // We received a packet, but it failed the checksum/headers
                printf("[GROUND] WARNING: Corrupted telemetry packet dropped.\n");
            }
        }

        // 4. Heartbeat (Lets you know the Ground Station hasn't frozen)
        if (time_us_32() - last_heartbeat > 2000000) {
            printf("[GROUND] Waiting for Drone telemetry...\n");
            if (!radio_restarted) {
                printf("[NRF24] Restarting...\n");
                radio.restart();
                printf("[NRF24] Restarted...\n");
                radio_restarted = true;
            }
            last_heartbeat = time_us_32();
        }
    }

    return 0;
}
