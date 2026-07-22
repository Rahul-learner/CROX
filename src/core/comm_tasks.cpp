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

void process_command(char* buffer) {
    DEBUG_PRINT("Processing Command: %s\n", buffer);
    
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

void process_radio_command(RadioCommandPacket* cmd, RadioResponsePacket* resp) {
    resp->cmd_id = cmd->cmd_id;
    memset(resp->payload, 0, sizeof(resp->payload));
    
    int16_t* i16_p = (int16_t*)resp->payload;
    int32_t* i32_p = (int32_t*)resp->payload;
    uint16_t* u16_p = (uint16_t*)resp->payload;
    
    switch (cmd->cmd_id) {
        case 0x01: // GET_ATTITUDE
            i16_p[0] = (int16_t)(roll * 100.0f);
            i16_p[1] = (int16_t)(pitch * 100.0f);
            i16_p[2] = (int16_t)(shared_yaw * 100.0f);
            break;
        case 0x02: // GET_PID_ACRO_RP
            i16_p[0] = (int16_t)(pid_p_roll_pitch_acro * 1000.0f);
            i16_p[1] = (int16_t)(pid_i_roll_pitch_acro * 1000.0f);
            i16_p[2] = (int16_t)(pid_d_roll_pitch_acro * 1000.0f);
            break;
        case 0x03: // SET_PID_ACRO_RP
            pid_p_roll_pitch_acro = ((int16_t*)cmd->payload)[0] / 1000.0f;
            pid_i_roll_pitch_acro = ((int16_t*)cmd->payload)[1] / 1000.0f;
            pid_d_roll_pitch_acro = ((int16_t*)cmd->payload)[2] / 1000.0f;
            resp->payload[0] = 0x01; // ACK
            break;
        case 0x04: // GET_ANGLE_TUNE
            i16_p[0] = (int16_t)(angle_strength * 1000.0f);
            i16_p[1] = (int16_t)(angle_max_deg * 10.0f);
            i16_p[2] = (int16_t)(angle_max_rate * 10.0f);
            break;
        case 0x05: // SET_ANGLE_TUNE
            angle_strength = ((int16_t*)cmd->payload)[0] / 1000.0f;
            angle_max_deg = ((int16_t*)cmd->payload)[1] / 10.0f;
            angle_max_rate = ((int16_t*)cmd->payload)[2] / 10.0f;
            resp->payload[0] = 0x01;
            break;
        case 0x06: // GET_PID_YAW
            i16_p[0] = (int16_t)(pid_p_yaw * 1000.0f);
            i16_p[1] = (int16_t)(pid_i_yaw * 1000.0f);
            i16_p[2] = (int16_t)(pid_d_yaw * 1000.0f);
            break;
        case 0x07: // SET_PID_YAW
            pid_p_yaw = ((int16_t*)cmd->payload)[0] / 1000.0f;
            pid_i_yaw = ((int16_t*)cmd->payload)[1] / 1000.0f;
            pid_d_yaw = ((int16_t*)cmd->payload)[2] / 1000.0f;
            resp->payload[0] = 0x01;
            break;
        case 0x08: // GET_EKF
            i32_p[0] = (int32_t)(q_gyro * 1e6f);
            i32_p[1] = (int32_t)(q_bias * 1e6f);
            i32_p[2] = (int32_t)(r_accel * 1e6f);
            break;
        case 0x09: // SET_EKF
            q_gyro = ((int32_t*)cmd->payload)[0] / 1e6f;
            q_bias = ((int32_t*)cmd->payload)[1] / 1e6f;
            r_accel = ((int32_t*)cmd->payload)[2] / 1e6f;
            resp->payload[0] = 0x01;
            break;
        case 0x0A: // GET_BIAS
            i16_p[0] = (int16_t)(bias_roll * 1000.0f);
            i16_p[1] = (int16_t)(bias_pitch * 1000.0f);
            i16_p[2] = (int16_t)(bias_yaw * 1000.0f);
            break;
        case 0x0B: // SET_BIAS
            bias_roll = ((int16_t*)cmd->payload)[0] / 1000.0f;
            bias_pitch = ((int16_t*)cmd->payload)[1] / 1000.0f;
            bias_yaw = ((int16_t*)cmd->payload)[2] / 1000.0f;
            resp->payload[0] = 0x01;
            break;
        case 0x0C: // GET_RC_TUNE
            memcpy(resp->payload, &rc_expo, 4);
            memcpy(resp->payload + 4, &rc_deadband, 4);
            memcpy(resp->payload + 8, &rc_yaw_deadband, 4);
            memcpy(resp->payload + 12, &rc_roll_center, 4);
            memcpy(resp->payload + 16, &rc_pitch_center, 4);
            memcpy(resp->payload + 20, &rc_yaw_center, 4);
            resp->payload[24] = rc_roll_reverse ? 1 : 0;
            resp->payload[25] = rc_pitch_reverse ? 1 : 0;
            resp->payload[26] = rc_yaw_reverse ? 1 : 0;
            break;
        case 0x0D: // SET_RC_TUNE
            memcpy(&rc_expo, cmd->payload, 4);
            memcpy(&rc_deadband, cmd->payload + 4, 4);
            memcpy(&rc_yaw_deadband, cmd->payload + 8, 4);
            memcpy(&rc_roll_center, cmd->payload + 12, 4);
            memcpy(&rc_pitch_center, cmd->payload + 16, 4);
            memcpy(&rc_yaw_center, cmd->payload + 20, 4);
            rc_roll_reverse = cmd->payload[24] > 0;
            rc_pitch_reverse = cmd->payload[25] > 0;
            rc_yaw_reverse = cmd->payload[26] > 0;
            resp->payload[0] = 0x01;
            break;
        case 0x0E: // GET_RC
            i16_p[0] = (int16_t)raw_receiver_pwm[0];
            i16_p[1] = (int16_t)raw_receiver_pwm[1];
            i16_p[2] = (int16_t)raw_receiver_pwm[2];
            i16_p[3] = (int16_t)raw_receiver_pwm[3];
            i16_p[4] = (int16_t)receiver_pwm[0];
            i16_p[5] = (int16_t)receiver_pwm[1];
            i16_p[6] = (int16_t)receiver_pwm[2];
            i16_p[7] = (int16_t)receiver_pwm[3];
            break;
        case 0x0F: // GET_STATUS
            resp->payload[0] = is_armed ? 1 : 0;
            resp->payload[1] = (uint8_t)current_flight_mode;
            u16_p[1] = (uint16_t)loop_time_us;
            break;
        case 0x10: // GET_IMU
            i16_p[0] = (int16_t)(shared_ax * 100.0f);
            i16_p[1] = (int16_t)(shared_ay * 100.0f);
            i16_p[2] = (int16_t)(shared_az * 100.0f);
            i16_p[3] = (int16_t)(shared_gx * 100.0f);
            i16_p[4] = (int16_t)(shared_gy * 100.0f);
            i16_p[5] = (int16_t)(shared_gz * 100.0f);
            break;
        case 0x11: // SET_MODE
            current_flight_mode = (FlightMode)cmd->payload[0];
            resp->payload[0] = 0x01;
            break;
        case 0x12: // GET_MOTORS
            if (global_motor_ptr) {
                u16_p[0] = (uint16_t)global_motor_ptr->motor1_speed;
                u16_p[1] = (uint16_t)global_motor_ptr->motor2_speed;
                u16_p[2] = (uint16_t)global_motor_ptr->motor3_speed;
                u16_p[3] = (uint16_t)global_motor_ptr->motor4_speed;
            } else {
                u16_p[0] = u16_p[1] = u16_p[2] = u16_p[3] = 1000;
            }
            break;
        // 0x13 SET_MOTOR_TEST omitted for safety
        case 0x14: // GET_FF
            i16_p[0] = (int16_t)(ff_roll * 1000.0f);
            i16_p[1] = (int16_t)(ff_pitch * 1000.0f);
            i16_p[2] = (int16_t)(ff_yaw * 1000.0f);
            break;
        case 0x15: // SET_FF
            ff_roll = ((int16_t*)cmd->payload)[0] / 1000.0f;
            ff_pitch = ((int16_t*)cmd->payload)[1] / 1000.0f;
            ff_yaw = ((int16_t*)cmd->payload)[2] / 1000.0f;
            resp->payload[0] = 0x01;
            break;
        case 0x16: // GET_TPA
            i16_p[0] = (int16_t)(tpa_breakpoint * 1000.0f);
            i16_p[1] = (int16_t)(tpa_factor * 1000.0f);
            break;
        case 0x17: // SET_TPA
            tpa_breakpoint = ((int16_t*)cmd->payload)[0] / 1000.0f;
            tpa_factor = ((int16_t*)cmd->payload)[1] / 1000.0f;
            resp->payload[0] = 0x01;
            break;
        case 0x18: // GET_I_LIMIT
            i16_p[0] = (int16_t)(pid_integral_limit * 100.0f);
            break;
        case 0x19: // SET_I_LIMIT
            pid_integral_limit = ((int16_t*)cmd->payload)[0] / 100.0f;
            resp->payload[0] = 0x01;
            break;
        case 0x1A: // GET_D_CUTOFF
            i16_p[0] = (int16_t)(pid_d_cutoff_hz * 100.0f);
            break;
        case 0x1B: // SET_D_CUTOFF
            pid_d_cutoff_hz = ((int16_t*)cmd->payload)[0] / 100.0f;
            resp->payload[0] = 0x01;
            break;
        case 0x1C: // CALIBRATE_ACCEL
            request_accel_calibration = true;
            resp->payload[0] = 0x01;
            break;
        case 0x1D: // CALIBRATE_NOISE
            request_noise_calibration = true;
            resp->payload[0] = 0x01;
            break;
        case 0x1E: // REBOOT
            resp->payload[0] = 0x01;
            break;
        case 0x1F: // GET_CONFIG
            u16_p[0] = CPU_FREQ_KHZ;
            u16_p[1] = PWM_FREQUENCY_HZ;
            resp->payload[4] = USE_SD_CARD_LOGGING ? 1 : 0;
            resp->payload[5] = USE_NRF24_RADIO ? 1 : 0;
            break;
        default:
            break;
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
    RadioCommandPacket incoming_cmd = {};
    RadioResponsePacket outgoing_resp = {};
#endif

    while (true) {

#if USE_NRF24_RADIO
        // Only restart radio after 5 seconds of inactivity (idle recovery)
        if (time_us_32() - last_radio_activity > 5000000) {
            radio.restart();
            last_radio_activity = time_us_32();
        }

        while (radio.dataAvailable()) {
            last_radio_activity = time_us_32();

            if (radio.readCommand(&incoming_cmd)) {
                DEBUG_PRINT("RADIO CMD: %d\n", incoming_cmd.cmd_id);
                process_radio_command(&incoming_cmd, &outgoing_resp);
                radio.sendResponse(&outgoing_resp);

                if (incoming_cmd.cmd_id == 0x1E) { // REBOOT
                    sleep_ms(100);
                    watchdog_reboot(0, 0, 0);
                }
            }
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
