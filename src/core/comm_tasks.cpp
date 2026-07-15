#include "core/comm_tasks.h"
#include "config.h"
#include "core/globals.h"
#include "pico/time.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include <stdio.h>
#include <string.h>

#ifndef DEBUG_PRINT
#ifdef DEBUG_MODE
#define DEBUG_PRINT(...) printf(__VA_ARGS__)
#else
#define DEBUG_PRINT(...)
#endif
#endif

// Parse the received usb string
void process_command(char* buffer) {
    if (strncmp(buffer, "EKF,", 4) == 0) {
        sscanf(buffer, "EKF,%f,%f,%f", &q_gyro, &q_bias, &r_accel);
        DEBUG_PRINT("ACK EKF: %f, %f, %f\n", q_gyro, q_bias, r_accel);
    }
    // NEW: Roll & Pitch PID
    else if (strncmp(buffer, "PID_RP,", 7) == 0) {
        sscanf(buffer, "PID_RP,%f,%f,%f", &pid_p_roll_pitch, &pid_i_roll_pitch, &pid_d_roll_pitch); // Replace with your variables
        DEBUG_PRINT("ACK PID_RP: %f, %f, %f\n", pid_p_roll_pitch, pid_i_roll_pitch, pid_d_roll_pitch);
        pitch_pid.set_pid(pid_p_roll_pitch, pid_i_roll_pitch, pid_d_roll_pitch);
    }
    // NEW: Yaw PID
    else if (strncmp(buffer, "PID_YAW,", 8) == 0) {
        sscanf(buffer, "PID_YAW,%f,%f,%f", &pid_p_yaw, &pid_i_yaw, &pid_d_yaw); // Replace with your yaw variables
        DEBUG_PRINT("ACK PID_YAW: %f, %f, %f\n", pid_p_yaw, pid_i_yaw, pid_d_yaw);
        yaw_pid.set_pid(pid_p_yaw, pid_i_yaw, pid_d_yaw);
    }
    else if (strncmp(buffer, "BIAS,", 5) == 0) {
        sscanf(buffer, "BIAS,%f,%f,%f", &bias_roll, &bias_pitch, &bias_yaw);
        DEBUG_PRINT("ACK BIAS: R:%f, P:%f, Y:%f\n", bias_roll, bias_pitch, bias_yaw);
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

void core1_entry() {
    multicore_lockout_victim_init();
    uint32_t last_telemetry_time = time_us_32();
    TelemetryPacket my_telemetry;
    PIDTuningPacket new_pids;
    bool radio_restarted = false;

    while (true) {

        if (!send_telemetry && !radio_restarted) {
            radio.restart();
            radio_restarted = true;
        }

        while (radio.dataAvailable()) {
            radio_restarted = false;
            DEBUG_PRINT("New Data Available!..");
            // Try to read it as a PID update (your existing code)
            if (radio.readPID(&new_pids)) {
                DEBUG_PRINT("NEW PID GAINS and BIAS ANGLES RECEIVED!\n");
                float kp_roll_pitch = new_pids.kp_roll_pitch / 1000.0f;
                float ki_roll_pitch = new_pids.ki_roll_pitch / 1000.0f;
                float kd_roll_pitch = new_pids.kd_roll_pitch / 1000.0f;
                float kp_yaw = new_pids.kp_yaw / 1000.0f;
                float ki_yaw = new_pids.ki_yaw / 1000.0f;
                float kd_yaw = new_pids.kd_yaw / 1000.0f;
                bias_roll = new_pids.bias_roll / 1000.0f;
                bias_pitch = new_pids.bias_pitch / 1000.0f;
                bias_yaw = new_pids.bias_yaw / 1000.0f;
                DEBUG_PRINT("Bias Updated: roll: %f, pitch: %f, yaw: %f\n",bias_roll,bias_pitch,bias_yaw);
                roll_pid.set_pid(kp_roll_pitch, ki_roll_pitch, kd_roll_pitch);
                pitch_pid.set_pid(kp_roll_pitch, ki_roll_pitch, kd_roll_pitch);
                yaw_pid.set_pid(kp_yaw, ki_yaw, kd_yaw);
                fc_buzzer.play_tone(1);
                sleep_ms(200);
                fc_buzzer.stop();
            }
        }

        // Send Telemetry at 10Hz
        uint32_t current_time = time_us_32();
        if (send_telemetry && (current_time - last_telemetry_time > 100000)) {
            radio_restarted = false;
            last_telemetry_time = current_time;

            // 1. PACKING TELEMETRY (Sending to Ground)
            my_telemetry.roll      = (int16_t)(shared_roll * 100.0f);
            my_telemetry.pitch     = (int16_t)(shared_pitch * 100.0f);
            my_telemetry.yaw       = (int16_t)(shared_yaw * 100.0f);
            my_telemetry.rc_roll   = (int16_t)(shared_rc_roll * 100.0f);
            my_telemetry.rc_pitch  = (int16_t)(shared_rc_pitch * 100.0f);
            my_telemetry.rc_yaw    = (int16_t)(shared_rc_yaw * 100.0f);
            my_telemetry.pid_roll  = (int16_t)(shared_pid_roll * 100.0f);
            my_telemetry.pid_pitch = (int16_t)(shared_pid_pitch * 100.0f);
            my_telemetry.pid_yaw   = (int16_t)(shared_pid_yaw * 100.0f);
            my_telemetry.dt_s     = (uint16_t)((current_time / 1000000.0f) * 100.0f);

            radio.sendTelemetry(&my_telemetry);
        }

        // Tiny sleep to prevent Core 1 from aggressively locking the system bus
        sleep_ms(1);
    }
}
