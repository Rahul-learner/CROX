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
        tuning_updates[3] = true;
    }
    // NEW: Roll & Pitch PID
    else if (strncmp(buffer, "PID_RP,", 7) == 0) {
        sscanf(buffer, "PID_RP,%f,%f,%f", &pid_p_roll_pitch, &pid_i_roll_pitch, &pid_d_roll_pitch); // Replace with your variables
        DEBUG_PRINT("ACK PID_RP: %f, %f, %f\n", pid_p_roll_pitch, pid_i_roll_pitch, pid_d_roll_pitch);
        pitch_pid.set_pid(pid_p_roll_pitch, pid_i_roll_pitch, pid_d_roll_pitch);
        roll_pid.set_pid(pid_p_roll_pitch, pid_i_roll_pitch, pid_d_roll_pitch);
        tuning_updates[0] = true;
    }
    // NEW: Yaw PID
    else if (strncmp(buffer, "PID_YAW,", 8) == 0) {
        sscanf(buffer, "PID_YAW,%f,%f,%f", &pid_p_yaw, &pid_i_yaw, &pid_d_yaw); // Replace with your yaw variables
        DEBUG_PRINT("ACK PID_YAW: %f, %f, %f\n", pid_p_yaw, pid_i_yaw, pid_d_yaw);
        yaw_pid.set_pid(pid_p_yaw, pid_i_yaw, pid_d_yaw);
        tuning_updates[1] = true;
    }
    else if (strncmp(buffer, "BIAS,", 5) == 0) {
        sscanf(buffer, "BIAS,%f,%f,%f", &bias_roll, &bias_pitch, &bias_yaw);
        DEBUG_PRINT("ACK BIAS: R:%f, P:%f, Y:%f\n", bias_roll, bias_pitch, bias_yaw);
        tuning_updates[2] = true;
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
    TelemetryPacket my_telemetry = {};
    PIDTuningPacket new_pids = {};
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
                DEBUG_PRINT("NEW TUNING PACKET RECEIVED! Mask: %d\n", new_pids.update_mask);
                bool rp_changed = false, yaw_changed = false, bias_changed = false;

                if (new_pids.update_mask & 0x01) {
                    float new_kp_rp = new_pids.kp_roll_pitch / 1000.0f;
                    float new_ki_rp = new_pids.ki_roll_pitch / 1000.0f;
                    float new_kd_rp = new_pids.kd_roll_pitch / 1000.0f;
                    if (pid_p_roll_pitch != new_kp_rp || pid_i_roll_pitch != new_ki_rp || pid_d_roll_pitch != new_kd_rp) {
                        pid_p_roll_pitch = new_kp_rp;
                        pid_i_roll_pitch = new_ki_rp;
                        pid_d_roll_pitch = new_kd_rp;
                        rp_changed = true;
                    }
                }

                if (new_pids.update_mask & 0x02) {
                    float new_kp_y = new_pids.kp_yaw / 1000.0f;
                    float new_ki_y = new_pids.ki_yaw / 1000.0f;
                    float new_kd_y = new_pids.kd_yaw / 1000.0f;
                    if (pid_p_yaw != new_kp_y || pid_i_yaw != new_ki_y || pid_d_yaw != new_kd_y) {
                        pid_p_yaw = new_kp_y;
                        pid_i_yaw = new_ki_y;
                        pid_d_yaw = new_kd_y;
                        yaw_changed = true;
                    }
                }

                if (new_pids.update_mask & 0x04) {
                    float new_b_r = new_pids.bias_roll / 1000.0f;
                    float new_b_p = new_pids.bias_pitch / 1000.0f;
                    float new_b_y = new_pids.bias_yaw / 1000.0f;
                    if (bias_roll != new_b_r || bias_pitch != new_b_p || bias_yaw != new_b_y) {
                        bias_roll = new_b_r;
                        bias_pitch = new_b_p;
                        bias_yaw = new_b_y;
                        bias_changed = true;
                    }
                }

                if (new_pids.update_mask & 0x08) {
                    float new_q_g = new_pids.q_gyro / 100000.0f;
                    float new_q_b = new_pids.q_bias / 1000000.0f;
                    float new_r_a = new_pids.r_accel / 100.0f;
                    if (q_gyro != new_q_g || q_bias != new_q_b || r_accel != new_r_a) {
                        q_gyro = new_q_g;
                        q_bias = new_q_b;
                        r_accel = new_r_a;
                        tuning_updates[3] = true;
                        DEBUG_PRINT("EKF Updated: %f, %f, %f\n", q_gyro, q_bias, r_accel);
                        fc_buzzer.play_tone(1);
                        sleep_ms(200);
                        fc_buzzer.stop();
                    }
                }

                if (rp_changed) {
                    tuning_updates[0] = true;
                    roll_pid.set_pid(pid_p_roll_pitch, pid_i_roll_pitch, pid_d_roll_pitch);
                    pitch_pid.set_pid(pid_p_roll_pitch, pid_i_roll_pitch, pid_d_roll_pitch);
                }
                if (yaw_changed) {
                    tuning_updates[1] = true;
                    yaw_pid.set_pid(pid_p_yaw, pid_i_yaw, pid_d_yaw);
                }
                if (bias_changed) {
                    tuning_updates[2] = true;
                    DEBUG_PRINT("Bias Updated: roll: %f, pitch: %f, yaw: %f\n",bias_roll,bias_pitch,bias_yaw);
                }

                if (rp_changed || yaw_changed || bias_changed) {
                    fc_buzzer.play_tone(1);
                    sleep_ms(200);
                    fc_buzzer.stop();
                }
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
