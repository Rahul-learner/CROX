#include "core/comm_tasks.h"
#include "config.h"
#include "core/globals.h"
#include "pico/time.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "blackbox.h"
#include "hardware/watchdog.h"
#include "writePWM.h"
#include <stdio.h>
#include <string.h>

#ifndef DEBUG_PRINT
#ifdef DEBUG_MODE
#define DEBUG_PRINT(...) printf(__VA_ARGS__)
#else
#define DEBUG_PRINT(...)
#endif
#endif

static uint32_t buzzer_off_time = 0;


void process_command(char* buffer) {
    DEBUG_PRINT("Processing Command: %s\n", buffer);
    
    bool is_set = (strncmp(buffer, "SET_", 4) == 0) ||
                  (strncmp(buffer, "EKF,", 4) == 0) ||
                  (strncmp(buffer, "PID_", 4) == 0 && strncmp(buffer, "PID_YAW", 7) != 0 && strncmp(buffer, "PID_ACRO_RP", 11) != 0) || // Exclude GET versions
                  (strncmp(buffer, "PID_ACRO_RP,", 12) == 0) ||
                  (strncmp(buffer, "PID_YAW,", 8) == 0) ||
                  (strncmp(buffer, "ANGLE_TUNE,", 11) == 0) ||
                  (strncmp(buffer, "BIAS,", 5) == 0) ||
                  (strncmp(buffer, "CALIBRATE_", 10) == 0) ||
                  (strncmp(buffer, "REBOOT", 6) == 0);

    if (is_set) {
        fc_buzzer.play_tone(2); // 2kHz tone for tuning changes
        buzzer_off_time = time_us_32() + 150000; // 150ms beep
    }

    if (strncmp(buffer, "GET_EKF", 7) == 0) {
        printf("EKF,%f,%f,%f\n", q_gyro, q_bias, r_accel);
    }
    else if (strncmp(buffer, "GET_PID_ACRO_RP", 15) == 0) {
        printf("PID_ACRO_RP,%f,%f,%f\n", pid_p_roll_pitch_acro, pid_i_roll_pitch_acro, pid_d_roll_pitch_acro);
    }
    else if (strncmp(buffer, "GET_ANGLE_TUNE", 14) == 0) {
        printf("ANGLE_TUNE,%f,%f,%f\n", angle_strength, angle_max_deg, angle_max_rate);
    }
    else if (strncmp(buffer, "GET_PID_YAW", 11) == 0) {
        printf("PID_YAW,%f,%f,%f\n", pid_p_yaw, pid_i_yaw, pid_d_yaw);
    }
    else if (strncmp(buffer, "GET_BIAS", 8) == 0) {
        printf("BIAS,%f,%f,%f\n", bias_roll, bias_pitch, bias_yaw);
    }
    else if (strncmp(buffer, "GET_BLACKBOX_INFO", 17) == 0) {
        extern Blackbox blackbox;
        blackbox.send_binary_info();
    }
    else if (strncmp(buffer, "GET_BLACKBOX", 12) == 0) {
        extern Blackbox blackbox;
        blackbox.send_to_usb();
    }
    else if (strncmp(buffer, "CLEAR_BLACKBOX", 14) == 0) {
        extern Blackbox blackbox;
        // The clear operation would involve erasing the flash, but for now we just reset the pointers in RAM.
        // A true flash erase will happen when the buffer wraps, or we can add a flash erase here if needed.
        blackbox.clear_blackbox_data();
        printf("BLACKBOX_CLEARED\n");
    }
    else if (strncmp(buffer, "GET_ATTITUDE", 12) == 0) {
        printf("ATTITUDE,%f,%f,%f\n", roll, pitch, shared_yaw);
    }
    else if (strncmp(buffer, "EKF,", 4) == 0) {
        sscanf(buffer, "EKF,%f,%f,%f", &q_gyro, &q_bias, &r_accel);
        DEBUG_PRINT("ACK EKF: %f, %f, %f\n", q_gyro, q_bias, r_accel);
    }
    else if (strncmp(buffer, "PID_ACRO_RP,", 12) == 0) {
        sscanf(buffer, "PID_ACRO_RP,%f,%f,%f", &pid_p_roll_pitch_acro, &pid_i_roll_pitch_acro, &pid_d_roll_pitch_acro);
        DEBUG_PRINT("ACK PID_ACRO_RP: %f, %f, %f\n", pid_p_roll_pitch_acro, pid_i_roll_pitch_acro, pid_d_roll_pitch_acro);
        pids_updated_via_msp = true;
    }
    else if (strncmp(buffer, "ANGLE_TUNE,", 11) == 0) {
        sscanf(buffer, "ANGLE_TUNE,%f,%f,%f", &angle_strength, &angle_max_deg, &angle_max_rate);
        DEBUG_PRINT("ACK ANGLE_TUNE: %f, %f, %f\n", angle_strength, angle_max_deg, angle_max_rate);
    }
    else if (strncmp(buffer, "PID_YAW,", 8) == 0) {
        sscanf(buffer, "PID_YAW,%f,%f,%f", &pid_p_yaw, &pid_i_yaw, &pid_d_yaw);
        DEBUG_PRINT("ACK PID_YAW: %f, %f, %f\n", pid_p_yaw, pid_i_yaw, pid_d_yaw);
    }
    else if (strncmp(buffer, "BIAS,", 5) == 0) {
        sscanf(buffer, "BIAS,%f,%f,%f", &bias_roll, &bias_pitch, &bias_yaw);
        DEBUG_PRINT("ACK BIAS: R:%f, P:%f, Y:%f\n", bias_roll, bias_pitch, bias_yaw);
    }
    // --- NEW TUNING COMMANDS ---
    else if (strncmp(buffer, "GET_FF", 6) == 0) {
        printf("FF,%f,%f,%f\n", ff_roll, ff_pitch, ff_yaw);
    }
    else if (strncmp(buffer, "SET_FF,", 7) == 0) {
        sscanf(buffer, "SET_FF,%f,%f,%f", &ff_roll, &ff_pitch, &ff_yaw);
        DEBUG_PRINT("ACK FF: %f, %f, %f\n", ff_roll, ff_pitch, ff_yaw);
    }
    else if (strncmp(buffer, "GET_TPA", 7) == 0) {
        printf("TPA,%f,%f\n", tpa_breakpoint, tpa_factor);
    }
    else if (strncmp(buffer, "SET_TPA,", 8) == 0) {
        sscanf(buffer, "SET_TPA,%f,%f", &tpa_breakpoint, &tpa_factor);
        DEBUG_PRINT("ACK TPA: %f, %f\n", tpa_breakpoint, tpa_factor);
    }
    else if (strncmp(buffer, "GET_I_LIMIT", 11) == 0) {
        printf("I_LIMIT,%f\n", pid_integral_limit);
    }
    else if (strncmp(buffer, "SET_I_LIMIT,", 12) == 0) {
        sscanf(buffer, "SET_I_LIMIT,%f", &pid_integral_limit);
        DEBUG_PRINT("ACK I_LIMIT: %f\n", pid_integral_limit);
    }
    else if (strncmp(buffer, "GET_D_CUTOFF", 12) == 0) {
        printf("D_CUTOFF,%f\n", pid_d_cutoff_hz);
    }
    else if (strncmp(buffer, "SET_D_CUTOFF,", 13) == 0) {
        sscanf(buffer, "SET_D_CUTOFF,%f", &pid_d_cutoff_hz);
        DEBUG_PRINT("ACK D_CUTOFF: %f\n", pid_d_cutoff_hz);
    }
    // --- NEW CONFIGURATOR COMMANDS ---
    else if (strncmp(buffer, "GET_RC_TUNE", 11) == 0) {
        printf("RC_TUNE,%f,%f,%f,%f,%f,%f,%d,%d,%d\n",
               rc_expo, rc_deadband, rc_yaw_deadband,
               rc_roll_center, rc_pitch_center, rc_yaw_center,
               rc_roll_reverse ? 1 : 0, rc_pitch_reverse ? 1 : 0, rc_yaw_reverse ? 1 : 0);
    }
    else if (strncmp(buffer, "SET_RC_TUNE,", 12) == 0) {
        int rr, pr, yr;
        sscanf(buffer, "SET_RC_TUNE,%f,%f,%f,%f,%f,%f,%d,%d,%d",
               &rc_expo, &rc_deadband, &rc_yaw_deadband,
               &rc_roll_center, &rc_pitch_center, &rc_yaw_center,
               &rr, &pr, &yr);
        rc_roll_reverse = rr > 0;
        rc_pitch_reverse = pr > 0;
        rc_yaw_reverse = yr > 0;
        DEBUG_PRINT("ACK RC_TUNE\n");
    }
    else if (strncmp(buffer, "GET_RC", 6) == 0) {
        printf("RC,%.0f,%.0f,%.0f,%.0f,%.1f,%.1f,%.0f,%.1f\n",
               raw_receiver_pwm[0], raw_receiver_pwm[1],
               raw_receiver_pwm[2], raw_receiver_pwm[3],
               receiver_pwm[0], receiver_pwm[1],
               receiver_pwm[2], receiver_pwm[3]);
    }
    else if (strncmp(buffer, "GET_MOTORS", 10) == 0) {
        if (global_motor_ptr) {
            printf("MOTORS,%u,%u,%u,%u\n",
                   global_motor_ptr->motor1_speed, global_motor_ptr->motor2_speed,
                   global_motor_ptr->motor3_speed, global_motor_ptr->motor4_speed);
        } else {
            printf("MOTORS,1000,1000,1000,1000\n");
        }
    }
    else if (strncmp(buffer, "GET_STATUS", 10) == 0) {
        printf("STATUS,%d,%d,%lu,%d\n",
               is_armed ? 1 : 0,
               (int)current_flight_mode,
               (unsigned long)loop_time_us,
               CPU_FREQ_KHZ);
    }
    else if (strncmp(buffer, "GET_IMU", 7) == 0) {
        printf("IMU,%f,%f,%f,%f,%f,%f\n",
               (float)shared_ax, (float)shared_ay, (float)shared_az,
               (float)shared_gx, (float)shared_gy, (float)shared_gz);
    }
    else if (strncmp(buffer, "CALIBRATE_ACCEL", 15) == 0) {
        request_accel_calibration = true;
        printf("ACK CALIBRATE_ACCEL\n");
    }
    else if (strncmp(buffer, "GET_ACCEL_BIAS", 14) == 0) {
        printf("ACCEL_BIAS,%f,%f\n", bias_roll, bias_pitch);
    }
    else if (strncmp(buffer, "CALIBRATE_NOISE", 15) == 0) {
        request_noise_calibration = true;
        printf("ACK CALIBRATE_NOISE\n");
    }
    else if (strncmp(buffer, "ESC_CAL_START", 13) == 0) {
        esc_calibrating = true;
        printf("ACK ESC_CAL_START\n");
    }
    else if (strncmp(buffer, "ESC_CAL_END", 11) == 0) {
        esc_calibrating = false;
        printf("ACK ESC_CAL_END\n");
    }
    else if (strncmp(buffer, "GET_CONFIG", 10) == 0) {
        printf("CONFIG,%d,%d,BMI160,%d,%d\n",
               CPU_FREQ_KHZ, PWM_FREQUENCY_HZ,
               USE_SD_CARD_LOGGING ? 1 : 0,
               USE_NRF24_RADIO ? 1 : 0);
    }
    else if (strncmp(buffer, "SET_MOTOR_TEST,", 15) == 0) {
        if (!is_armed) {
            uint32_t m1, m2, m3, m4;
            sscanf(buffer, "SET_MOTOR_TEST,%u,%u,%u,%u", &m1, &m2, &m3, &m4);
            // For safety, clamp to 1000-1300 (low speed only)
            auto clamp = [](uint32_t v) -> uint32_t {
                if (v < 1000) return 1000;
                if (v > 1300) return 1300;
                return v;
            };
            m1 = clamp(m1); m2 = clamp(m2); m3 = clamp(m3); m4 = clamp(m4);
            if (global_motor_ptr) {
                global_motor_ptr->motor1_speed = m1;
                global_motor_ptr->motor2_speed = m2;
                global_motor_ptr->motor3_speed = m3;
                global_motor_ptr->motor4_speed = m4;
                global_motor_ptr->update_motors_pwm(m1, (float)((int32_t)m2-(int32_t)m1), (float)((int32_t)m3-(int32_t)m1), 0);
            }
            printf("ACK MOTOR_TEST,%u,%u,%u,%u\n", m1, m2, m3, m4);
        } else {
            printf("ERR: Cannot test motors while armed\n");
        }
    }
    else if (strncmp(buffer, "SET_MODE,", 9) == 0) {
        if (strncmp(buffer + 9, "ACRO", 4) == 0) {
            current_flight_mode = MODE_ACRO;
            printf("ACK MODE,ACRO\n");
        } else if (strncmp(buffer + 9, "ANGLE", 5) == 0) {
            current_flight_mode = MODE_ANGLE;
            printf("ACK MODE,ANGLE\n");
        }
    }
    else if (strncmp(buffer, "REBOOT", 6) == 0) {
        printf("ACK REBOOT\n");
        sleep_ms(100);
        watchdog_reboot(0, 0, 0);
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
#if USE_NRF24_RADIO
    uint32_t last_radio_activity = time_us_32();
    TelemetryPacket outgoing_telemetry = {};
    PIDTuningPacket incoming_pids = {};
    AngleTuningPacket incoming_angle = {};
    uint32_t last_telemetry_send = time_us_32();
#endif

    while (true) {

        // Auto-stop buzzer after its duration expires (works for both radio and serial)
        if (buzzer_off_time > 0 && time_us_32() >= buzzer_off_time) {
            fc_buzzer.stop();
            buzzer_off_time = 0;
        }

#if USE_NRF24_RADIO
        // Only restart radio after 5 seconds of inactivity (idle recovery)
        if (time_us_32() - last_radio_activity > 5000000) {
            DEBUG_PRINT("[DRONE] Radio idle for 5s, restarting NRF24...\n");
            radio.restart();
            last_radio_activity = time_us_32();
        }

        while (radio.dataAvailable()) {
            last_radio_activity = time_us_32();

            int pkt_type = radio.readCommand(&incoming_pids, &incoming_angle);

            if (pkt_type == 1) { // PID Packet
                if (incoming_pids.update_mask & 0x01) {
                    pid_p_roll_pitch_acro = incoming_pids.kp_roll_pitch / 1000.0f;
                    pid_i_roll_pitch_acro = incoming_pids.ki_roll_pitch / 1000.0f;
                    pid_d_roll_pitch_acro = incoming_pids.kd_roll_pitch / 1000.0f;
                    pids_updated_via_msp = true;
                }
                if (incoming_pids.update_mask & 0x02) {
                    pid_p_yaw = incoming_pids.kp_yaw / 1000.0f;
                    pid_i_yaw = incoming_pids.ki_yaw / 1000.0f;
                    pid_d_yaw = incoming_pids.kd_yaw / 1000.0f;
                }
                if (incoming_pids.update_mask & 0x04) {
                    bias_roll = incoming_pids.bias_roll / 1000.0f;
                    bias_pitch = incoming_pids.bias_pitch / 1000.0f;
                    bias_yaw = incoming_pids.bias_yaw / 1000.0f;
                }
                if (incoming_pids.update_mask & 0x08) {
                    q_gyro = incoming_pids.q_gyro / 100000.0f;
                    q_bias = incoming_pids.q_bias / 1000000.0f;
                    r_accel = incoming_pids.r_accel / 100.0f;
                }

                fc_buzzer.play_tone(2); // 2kHz tone for tuning changes
                buzzer_off_time = time_us_32() + 150000; // 150ms beep
            } else if (pkt_type == 2) { // Angle Packet
                angle_strength = incoming_angle.angle_strength / 1000.0f;
                angle_max_deg = incoming_angle.angle_max_deg / 10.0f;
                angle_max_rate = incoming_angle.angle_max_rate / 10.0f;
                pids_updated_via_msp = true; // Signal outer loop that config changed

                fc_buzzer.play_tone(3); // 3kHz tone for angle tuning changes
                buzzer_off_time = time_us_32() + 150000; // 150ms beep
            }
        }

        if (send_telemetry && (time_us_32() - last_telemetry_send > 20000)) { // 50Hz telemetry
            outgoing_telemetry.roll = (int16_t)(shared_roll * 100);
            outgoing_telemetry.pitch = (int16_t)(shared_pitch * 100);
            outgoing_telemetry.yaw = (int16_t)(shared_yaw * 100);
            
            outgoing_telemetry.rc_roll = (int16_t)(shared_rc_roll * 100);
            outgoing_telemetry.rc_pitch = (int16_t)(shared_rc_pitch * 100);
            outgoing_telemetry.rc_yaw = (int16_t)(shared_rc_yaw * 100);
            
            outgoing_telemetry.pid_roll = (int16_t)(shared_pid_roll * 100);
            outgoing_telemetry.pid_pitch = (int16_t)(shared_pid_pitch * 100);
            outgoing_telemetry.pid_yaw = (int16_t)(shared_pid_yaw * 100);
            
            outgoing_telemetry.dt_s = (uint16_t)(shared_dt_us / 10); // scaled by 100 
            
            radio.sendTelemetry(&outgoing_telemetry);
            last_telemetry_send = time_us_32();
            send_telemetry = false;
        }
#endif
        check_serial_commands();

        // Process SD Card Logging Queue
        extern Blackbox blackbox;
        blackbox.sd_logging_task();

        // Tiny sleep to prevent Core 1 from aggressively locking the system bus
        sleep_us(100);
    }
}
