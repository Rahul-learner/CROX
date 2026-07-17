#include <hardware/timer.h>
#include <stdio.h>        // For standard input/output functions like printf
#include "pico/stdio_usb.h"
#include <string.h>       // For memset
#include "pico/stdlib.h"  // For general Pico standard library functions (gpio_init, sleep_ms, etc.)
#include "hardware/gpio.h" // For GPIO (General Purpose Input/Output) functions and interrupts
#include "pico/time.h"    // For time-related functions like sleep_us, make_timeout_time_ms
#include "hardware/clocks.h" // For setting CPU frequency (overclocking)
#include "stdint.h"
#include "pico/multicore.h"
#include "calibration.h"
#include "EKF.h"
#include "config.h"
#include "readIMU.h"
#include "readPWM.h"
#include "writePWM.h"
#include "PID.h"
#include "buzzer.h"
#include "nrf24_radio.h"
#include "blackbox.h"

// Core logic includes
#include "core/hardware_setup.h"
#include "core/flight_tasks.h"
#include "core/comm_tasks.h"
#include "core/sitl.h"

// Create a custom print macro
#ifdef DEBUG_MODE
#define DEBUG_PRINT(...) printf(__VA_ARGS__)
#else
#define DEBUG_PRINT(...) // Does absolutely nothing when flying
#endif

// --- Define the GPIO pin for the onboard LED ---
#ifndef PICO_DEFAULT_LED_PIN
#define PICO_DEFAULT_LED_PIN 25
#endif

float bias_roll = DEFAULT_BIAS_ROLL;
float bias_pitch = DEFAULT_BIAS_PITCH;
float bias_yaw = DEFAULT_BIAS_YAW;

float q_gyro = DEFAULT_Q_GYRO;
float q_bias = DEFAULT_Q_BIAS;
float r_accel = DEFAULT_R_ACCEL;

// PID values (initialised from config defaults, tuneable at runtime)
float pid_p_roll_pitch = DEFAULT_PID_P_ROLL_PITCH;
float pid_i_roll_pitch = DEFAULT_PID_I_ROLL_PITCH;
float pid_d_roll_pitch = DEFAULT_PID_D_ROLL_PITCH;
float pid_p_yaw = DEFAULT_PID_P_YAW;
float pid_i_yaw = DEFAULT_PID_I_YAW;
float pid_d_yaw = DEFAULT_PID_D_YAW;

BlackboxPacket bb_packet;

// --- Shared Variables for Dual-Core Communication ---
volatile bool tuning_updates[4] = {false, false, false, false};
volatile bool send_telemetry = false;
volatile float shared_roll = 0.0f;
volatile float shared_pitch = 0.0f;
volatile float shared_yaw = 0.0f;
volatile float shared_rc_roll = 0.0f;
volatile float shared_rc_pitch = 0.0f;
volatile float shared_rc_yaw = 0.0f;
volatile float shared_pid_roll = 0.0f;
volatile float shared_pid_pitch = 0.0f;
volatile float shared_pid_yaw = 0.0f;
volatile float shared_dt_us = 0.0f;

// receiver data
float receiver_pwm[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float rc_roll_bias = 0.0f;
float rc_pitch_bias = 0.0f;
float rc_yaw_bias = 0.0f;

// Global variables for filtered data
float roll = 0.0f, pitch = 0.0f, yaw = 0.0f; // Added yaw
volatile bool imu_data_ready = false; // Flag set by interrupt handler
bool is_armed = false;

// Global variable for tracking time
uint64_t last_update_ekf_us = 0;
uint64_t last_update_pid_us = 0;
uint32_t last_update_telemetry_us = 0;

// Radio addresses (from config)
extern const uint8_t drone_tx_addr[5] = DRONE_TX_ADDR;
extern const uint8_t drone_rx_addr[5] = DRONE_RX_ADDR;

// --- Initialize Class ---
Blackbox blackbox;
BMI160 imu(IMU_SPI_PORT, IMU_PIN_CS, IMU_PIN_SCK, IMU_PIN_MOSI, IMU_PIN_MISO);
NRF24 radio(RADIO_SPI_PORT, RADIO_CE, RADIO_CSN, RADIO_SCK, RADIO_MOSI, RADIO_MISO);
Buzzer fc_buzzer(BUZZER_PIN);

// --- Initializing PID ---
PIDController roll_pid(pid_p_roll_pitch, pid_i_roll_pitch, pid_d_roll_pitch);
PIDController pitch_pid(pid_p_roll_pitch, pid_i_roll_pitch, pid_d_roll_pitch);
PIDController yaw_pid(pid_p_yaw, pid_i_yaw, pid_d_yaw);

float roll_control_output = 0.0f, pitch_control_output = 0.0f, yaw_control_output = 0.0f;

int main() {
    init_hardware();

    ReadPWM receiver;
    WritePWM motor;

    bool calibrate_esc = false; // Set to true to run ESC calibration once
    motor.calibrate_escs(calibrate_esc);

    // ====================================================================
    // GYROSCOPE CALIBRATION
    // ====================================================================
    DEBUG_PRINT("Calibrating Gyroscope... DO NOT MOVE THE DRONE!\n");
    fc_buzzer.play_tone(NOTE_C5);
    sleep_ms(100);
    fc_buzzer.stop();

    float gyro_bias_x = 0.0f;
    float gyro_bias_y = 0.0f;
    float gyro_bias_z = 0.0f;

    calibrate_gyro(imu, gyro_bias_x, gyro_bias_y, gyro_bias_z);

    DEBUG_PRINT("Calibration Complete! Biases: X:%.4f Y:%.4f Z:%.4f\n", gyro_bias_x, gyro_bias_y, gyro_bias_z);

    // Play the success startup tune!
    fc_buzzer.play_melody(Tunes::startup, Tunes::startup_len);
    // ====================================================================

    // Initialize the Filter
    QuaternionEKF filter;

    bool tune_EKF = false;
    if (tune_EKF) {
        while (!stdio_usb_connected()) {
            sleep_ms(100);
        }
        stdio_set_translate_crlf(&stdio_usb, false);
        sleep_ms(1000);
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        record_readings_for_SITL(imu);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        stdio_set_translate_crlf(&stdio_usb, true);
    }

    // Re-initialize last_update_us after settling
    last_update_ekf_us = time_us_64();
    last_update_pid_us = time_us_64();
    uint64_t last_update_rc_us = time_us_64();
    
    uint64_t arming_start_us = 0;
    uint64_t disarming_start_us = 0;
    float raw_receiver_pwm[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    bool run_accel_update = true;
    bool blackbox_updated = false;
    bool blackbox_dumped = false;

    float dt_ekf, dt_pid;
    uint32_t loop_counter = 0;

    // --- Main Loop ---
    while (true) {

        uint64_t current_pwm_update = time_us_64();
        if ((current_pwm_update - last_update_rc_us) > 1000000/50) {
            receiver.read_pwm(receiver_pwm, raw_receiver_pwm);

            // Apply calibration biases
            receiver_pwm[0] -= rc_roll_bias;
            receiver_pwm[1] -= rc_pitch_bias;
            receiver_pwm[3] -= rc_yaw_bias;

            // reverse the roll
            receiver_pwm[0] *= -1;

            last_update_rc_us = current_pwm_update;
        }
        
        // Always process IMU data if ready, regardless of armed state
        if (imu_data_ready) {
            imu_data_ready = false;
            loop_counter++;

            uint64_t start_ekf = time_us_64();
            float gz_rate = 0.0f;
            update_sensors_and_ekf(filter, gyro_bias_x, gyro_bias_y, gyro_bias_z, dt_ekf, run_accel_update, gz_rate);
            uint64_t end_ekf = time_us_64();
            uint64_t ekf_calc_time = end_ekf - start_ekf;

            bool throttle_active = receiver_pwm[2] > 1005.0f;

            if (throttle_active) {
                if (!is_armed) {
                    bool is_level = (roll < 25.0f && roll > -25.0f) && (pitch < 25.0f && pitch > -25.0f);
                    bool is_throttle_safe = receiver_pwm[2] < 1050.0f && receiver_pwm[2] > 1000.0f;
                    
                    if (is_level && is_throttle_safe) {
                        // Calibrate receiver PWM offsets before arming
                        fc_buzzer.play_tone(1); // Warning tone during calibration
                        calibrate_receiver(receiver, rc_roll_bias, rc_pitch_bias, rc_yaw_bias);
                        fc_buzzer.stop();

                        is_armed = true;
                        fc_buzzer.stop();
                        fc_buzzer.play_melody(Tunes::armed, Tunes::armed_len);
                        
                        BlackboxPacket sep;
                        memset(&sep, 0, sizeof(BlackboxPacket));
                        sep.dt_us = 0xFFFE;
                        blackbox.write_packet(sep);
                        
                        last_update_ekf_us = time_us_64();
                    } else {
                        // Play error beep if not level
                        static uint64_t last_error_beep = 0;
                        if (time_us_64() - last_error_beep > 1000000) {
                            fc_buzzer.play_melody(Tunes::error_level, Tunes::error_level_len);
                            last_error_beep = time_us_64();
                        }
                    }
                }
            } else {
                if (is_armed) {
                    is_armed = false;
                    motor.reset();
                    fc_buzzer.stop();
                    fc_buzzer.play_melody(Tunes::disarmed, Tunes::disarmed_len);
                }
            }

            if (!is_armed) {
                handle_disarmed_state(motor, filter, blackbox_updated, blackbox_dumped);
            } else {
                gpio_put(PICO_DEFAULT_LED_PIN, 1);

                uint64_t current_time_pid_us = time_us_64();
                uint64_t pid_calc_time = 0;
                if ((current_time_pid_us - last_update_pid_us) >= 2040) {
                    dt_pid = (current_time_pid_us - last_update_pid_us) / 1000000.0f;
                    last_update_pid_us = current_time_pid_us;
                    
                    uint64_t start_pid = time_us_64();
                    update_pid_and_motors(motor, gz_rate, dt_pid);
                    uint64_t end_pid = time_us_64();
                    pid_calc_time = end_pid - start_pid;
                }
                
                uint64_t current_time_telemetry_us = time_us_64();
                if ((current_time_telemetry_us - last_update_telemetry_us) > 1000000 / 60) {
                    update_telemetry_and_blackbox(motor, gz_rate, dt_ekf, dt_pid, current_time_telemetry_us, blackbox_updated);
                    last_update_telemetry_us = current_time_telemetry_us;
                    
                    DEBUG_PRINT("Roll: %.2f, Pitch: %.2f, Yaw: %.2f, RC_Roll: %.2f, RC_Pitch: %.2f, "
                                "EKF_calc_us: %llu, PID_calc_us: %llu, "
                                "RC_Yaw: %.2f, RC_Throttle: %.2f, PID_Roll: %.2f, PID_Pitch: %.2f, PID_Yaw: %.2f, "
                                "dt_pid: %f, dt_ekf: %f\n",
                                roll, pitch, gz_rate, receiver_pwm[0], receiver_pwm[1],
                                ekf_calc_time, pid_calc_time,
                                receiver_pwm[3], receiver_pwm[2], roll_control_output, pitch_control_output,
                                yaw_control_output, dt_pid, dt_ekf);
                }
            }
        }
    }

    return 0;
}
