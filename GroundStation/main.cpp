#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "config.h"
#include "nrf24_radio.h"

// Store current tuning values (initialized to defaults from config.h)
float gs_kp_roll_pitch = DEFAULT_PID_P_ROLL_PITCH;
float gs_ki_roll_pitch = DEFAULT_PID_I_ROLL_PITCH;
float gs_kd_roll_pitch = DEFAULT_PID_D_ROLL_PITCH;
float gs_kp_yaw = DEFAULT_PID_P_YAW;
float gs_ki_yaw = DEFAULT_PID_I_YAW;
float gs_kd_yaw = DEFAULT_PID_D_YAW;
float gs_bias_roll = DEFAULT_BIAS_ROLL;
float gs_bias_pitch = DEFAULT_BIAS_PITCH;
float gs_bias_yaw = DEFAULT_BIAS_YAW;

bool send_pid_update = false;

// Parse the received usb string from the ground station PC
void process_command(char* buffer) {
    if (strncmp(buffer, "PID_RP,", 7) == 0) {
        sscanf(buffer, "PID_RP,%f,%f,%f", &gs_kp_roll_pitch, &gs_ki_roll_pitch, &gs_kd_roll_pitch);
        printf("GS ACK PID_RP: %f, %f, %f\n", gs_kp_roll_pitch, gs_ki_roll_pitch, gs_kd_roll_pitch);
        send_pid_update = true;
    }
    else if (strncmp(buffer, "PID_YAW,", 8) == 0) {
        sscanf(buffer, "PID_YAW,%f,%f,%f", &gs_kp_yaw, &gs_ki_yaw, &gs_kd_yaw);
        printf("GS ACK PID_YAW: %f, %f, %f\n", gs_kp_yaw, gs_ki_yaw, gs_kd_yaw);
        send_pid_update = true;
    }
    else if (strncmp(buffer, "BIAS,", 5) == 0) {
        sscanf(buffer, "BIAS,%f,%f,%f", &gs_bias_roll, &gs_bias_pitch, &gs_bias_yaw);
        printf("GS ACK BIAS: R:%f, P:%f, Y:%f\n", gs_bias_roll, gs_bias_pitch, gs_bias_yaw);
        send_pid_update = true;
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

int main() {
    stdio_init_all();
    sleep_ms(2000); // Wait for USB serial connection
    
    printf("Ground Station Starting...\n");

    // Initialize SPI for NRF24 using the same pins as the drone
    spi_init(RADIO_SPI_PORT, 2000 * 1000);
    gpio_set_function(RADIO_SCK, GPIO_FUNC_SPI);
    gpio_set_function(RADIO_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(RADIO_MISO, GPIO_FUNC_SPI);

    NRF24 radio(RADIO_SPI_PORT, RADIO_CE, RADIO_CSN, RADIO_SCK, RADIO_MOSI, RADIO_MISO);

    if (radio.checkConnection()) {
        printf("NRF24 Connected Successfully!\n");
    } else {
        printf("NRF24 Connection Failed!\n");
    }

    radio.init();

    // Ground station addresses (Flipped compared to drone)
    uint8_t tx_addr[5] = DRONE_RX_ADDR;
    uint8_t rx_addr[5] = DRONE_TX_ADDR;
    radio.setAddresses(tx_addr, rx_addr);

    radio.startListening();

    while (true) {
        // 1. Process incoming serial commands from PC
        check_serial_commands();

        // 2. If a PID update command was received via Serial, send it to the drone
        if (send_pid_update) {
            PIDTuningPacket pid_packet;
            pid_packet.kp_roll_pitch = (int16_t)(gs_kp_roll_pitch * 1000.0f);
            pid_packet.ki_roll_pitch = (int16_t)(gs_ki_roll_pitch * 1000.0f);
            pid_packet.kd_roll_pitch = (int16_t)(gs_kd_roll_pitch * 1000.0f);
            pid_packet.kp_yaw = (int16_t)(gs_kp_yaw * 1000.0f);
            pid_packet.ki_yaw = (int16_t)(gs_ki_yaw * 1000.0f);
            pid_packet.kd_yaw = (int16_t)(gs_kd_yaw * 1000.0f);
            pid_packet.bias_roll = (int16_t)(gs_bias_roll * 1000.0f);
            pid_packet.bias_pitch = (int16_t)(gs_bias_pitch * 1000.0f);
            pid_packet.bias_yaw = (int16_t)(gs_bias_yaw * 1000.0f);

            if (radio.sendPID(&pid_packet)) {
                printf("PID Tuning Packet Sent Successfully!\n");
            } else {
                printf("Failed to send PID Tuning Packet.\n");
            }
            send_pid_update = false;
        }

        // 3. Receive telemetry from the drone and print to serial
        if (radio.dataAvailable()) {
            TelemetryPacket telemetry_data;
            if (radio.readTelemetry(&telemetry_data)) {
                // Decode the data (drone multiplied by 100.0f)
                float roll = telemetry_data.roll / 100.0f;
                float pitch = telemetry_data.pitch / 100.0f;
                float yaw = telemetry_data.yaw / 100.0f;
                float rc_roll = telemetry_data.rc_roll / 100.0f;
                float rc_pitch = telemetry_data.rc_pitch / 100.0f;
                float rc_yaw = telemetry_data.rc_yaw / 100.0f;
                float pid_roll = telemetry_data.pid_roll / 100.0f;
                float pid_pitch = telemetry_data.pid_pitch / 100.0f;
                float pid_yaw = telemetry_data.pid_yaw / 100.0f;
                float dt = telemetry_data.dt_s / 100.0f; 

                // Print in the same format as debug.py expects
                printf("Roll: %.2f, Pitch: %.2f, Yaw: %.2f, RC_R: %.2f, RC_P: %.2f, RC_Y: %.2f, PID_R: %.2f, PID_P: %.2f, PID_Y: %.2f, Time: %.2fs\n",
                       roll, pitch, yaw, rc_roll, rc_pitch, rc_yaw, pid_roll, pid_pitch, pid_yaw, dt);
            }
        }
        sleep_ms(1);
    }

    return 0;
}
