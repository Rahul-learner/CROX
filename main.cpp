#include <hardware/timer.h>
#include <stdio.h>        // For standard input/output functions like printf
#include "pico/stdio_usb.h"
#include <math.h>         // For mathematical functions like atan2, asin, sqrt
#include <string.h>       // For memset
#include "pico/stdlib.h"  // For general Pico standard library functions (gpio_init, sleep_ms, etc.)
#include "hardware/gpio.h" // For GPIO (General Purpose Input/Output) functions and interrupts
#include "pico/time.h"    // For time-related functions like sleep_us, make_timeout_time_ms
#include "hardware/clocks.h" // For setting CPU frequency (overclocking)
#include "stdint.h"
#include "pico/multicore.h"
#include "EKF.h"
#include "readIMU.h"
#include "readPWM.h"
#include "writePWM.h"
#include "PID.h"
#include "buzzer.h"
#include "nrf24_radio.h"
// #include "calibration.h"

// #define DEBUG_MODE

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

float bias_roll = 0.0f;
float bias_pitch = 0.0f;
float bias_yaw = 0.0f;

float q_gyro = 0.001f;
float q_bias = 0.00001f;
float r_accel = 1.5f;

// pid values
// p = 1.75
// i = 0.04
// d = 0.47
float pid_p_roll_pitch = 1.75f;
float pid_i_roll_pitch = 0.04f;
float pid_d_roll_pitch = 0.47f;
// yaw pid values
float pid_p_yaw = 1.0f;
float pid_i_yaw = 0.0f;
float pid_d_yaw = 0.05f;

// --- Shared Variables for Dual-Core Communication ---
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

#define SPI_PORT spi0 
#define PIN_MISO 16
#define PIN_CS 17
#define PIN_SCK 18
#define PIN_MOSI 19
#define MPU_INT_PIN 20

// receiver data
float receiver_pwm[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float rc_roll_bias = -0.357097f, rc_pitch_bias = 0.553079f, rc_yaw_bias = -0.632232f;

// Global variables for filtered data
float roll = 0.0f, pitch = 0.0f, yaw = 0.0f; // Added yaw
volatile bool imu_data_ready = false; // Flag set by interrupt handler

// Global variable for tracking time
static uint64_t last_update_ekf_us = 0;
static uint64_t last_update_pid_us = 0;
static uint32_t last_update_telemetry_us = 0;


// --- BMI160 Interrupt handler ---
void mpu_isr(uint gpio, uint32_t events) {
    if (gpio == MPU_INT_PIN && (events & GPIO_IRQ_EDGE_FALL)) {
        imu_data_ready = true; // Set flag to process data in main loop
    }
}

// SPI communication for nRF24 Transceiver
#define RADIO_SPI_PORT spi1
#define RADIO_CE 14
#define RADIO_CSN 13
#define RADIO_SCK 10
#define RADIO_MOSI 11
#define RADIO_MISO 12

// 5 byte address (Drone and Ground Station Must be Flipped)
const uint8_t drone_tx_addr[5] = {'G', 'R', 'N', 'D', '1'}; // Sending to Ground
const uint8_t drone_rx_addr[5] = {'D', 'R', 'O', 'N', '1'}; // Receiving from Ground

// --- Initialize Class ---
BMI160 imu(SPI_PORT, PIN_CS, PIN_SCK, PIN_MOSI, PIN_MISO);
NRF24 radio(RADIO_SPI_PORT, RADIO_CE, RADIO_CSN, RADIO_SCK, RADIO_MOSI, RADIO_MISO);
Buzzer fc_buzzer(15);

// --- Initializing PID ---
PIDController roll_pid(pid_p_roll_pitch, pid_i_roll_pitch, pid_d_roll_pitch);
PIDController pitch_pid(pid_p_roll_pitch, pid_i_roll_pitch, pid_d_roll_pitch);
PIDController yaw_pid(pid_p_yaw, pid_i_yaw, pid_d_yaw);

void record_readings_for_SITL(BMI160& imu) {
  #pragma pack(push, 1)
  struct Readings {
    uint8_t header1;
    uint8_t header2;
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
    uint64_t timestamp;
    uint8_t checksum;
  };
  #pragma pack(pop)
  Readings readings;
  readings.header1 = 0xAA;
  readings.header2 = 0xBB;
  size_t data_size = sizeof(readings);
  uint64_t sec_of_reading = 1000000ULL * 50ULL;
  
  uint64_t start = time_us_64();
  uint64_t end = time_us_64();
  while ((end - start) < sec_of_reading) {
    if (imu_data_ready) {
      imu_data_ready = false;
      imu.readData(&readings.ax, &readings.ay, &readings.az, &readings.gx, &readings.gy, &readings.gz);
      readings.timestamp = time_us_64();
      // Compute simple XOR Checksum across the entire data struct (excluding headers and checksum byte itself)
      uint8_t* byte_ptr = (uint8_t*)&readings;
      uint8_t calc_checksum = 0;
      // Offset by 2 bytes to skip 0xAA and 0xBB. Stop before the final checksum byte.
      for (size_t i = 2; i < data_size - 1; i++) {
          calc_checksum ^= byte_ptr[i];
      }
      readings.checksum = calc_checksum;
      fwrite(&readings, data_size, 1, stdout);
      fflush(stdout);
    }
    end = time_us_64();
  }
  Readings stop_packet;
  stop_packet.header1 = 0xAA;
  stop_packet.header2 = 0xBB;
  
  // Zero out sensor values
  stop_packet.ax = 0.0f; stop_packet.ay = 0.0f; stop_packet.az = 0.0f;
  stop_packet.gx = 0.0f; stop_packet.gy = 0.0f; stop_packet.gz = 0.0f;
  
  // Set timestamp to MAX value as a stop indicator flag
  stop_packet.timestamp = 0xFFFFFFFFFFFFFFFFULL; 

  // Compute checksum for the stop packet so it passes validation on the PC
  uint8_t* byte_ptr = (uint8_t*)&stop_packet;
  uint8_t calc_checksum = 0;
  for (size_t i = 2; i < data_size - 1; i++) {
      calc_checksum ^= byte_ptr[i];
  }
  stop_packet.checksum = calc_checksum;

  // Send the stop signal multiple times (e.g., 5 times) to guarantee delivery 
  // in case of transmission noise
  for (int i = 0; i < 5; i++) {
      fwrite(&stop_packet, data_size, 1, stdout);
      fflush(stdout);
      sleep_ms(2);
  }
}

// Parse the received usb string
void process_command(char* buffer) {
    if (strncmp(buffer, "EKF,", 4) == 0) {
        sscanf(buffer, "EKF,%f,%f,%f", &q_gyro, &q_bias, &r_accel);
        printf("ACK EKF: %f, %f, %f\n", q_gyro, q_bias, r_accel);
    }
    // NEW: Roll & Pitch PID
    else if (strncmp(buffer, "PID_RP,", 7) == 0) {
        sscanf(buffer, "PID_RP,%f,%f,%f", &pid_p_roll_pitch, &pid_i_roll_pitch, &pid_d_roll_pitch); // Replace with your variables
        printf("ACK PID_RP: %f, %f, %f\n", pid_p_roll_pitch, pid_i_roll_pitch, pid_d_roll_pitch);
        pitch_pid.set_pid(pid_p_roll_pitch, pid_i_roll_pitch, pid_d_roll_pitch);
        // roll_pid.set_pid(pid_p_roll_pitch, pid_i_roll_pitch, pid_d_roll_pitch);
    }
    // NEW: Yaw PID
    else if (strncmp(buffer, "PID_YAW,", 8) == 0) {
        sscanf(buffer, "PID_YAW,%f,%f,%f", &pid_p_yaw, &pid_i_yaw, &pid_d_yaw); // Replace with your yaw variables
        printf("ACK PID_YAW: %f, %f, %f\n", pid_p_yaw, pid_i_yaw, pid_d_yaw);
        yaw_pid.set_pid(pid_p_yaw, pid_i_yaw, pid_d_yaw);
    }
    else if (strncmp(buffer, "BIAS,", 5) == 0) {
        sscanf(buffer, "BIAS,%f,%f,%f", &bias_roll, &bias_pitch, &bias_yaw);
        printf("ACK BIAS: R:%f, P:%f, Y:%f\n", bias_roll, bias_pitch, bias_yaw);
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
            printf("New Data Available!..");
            // Try to read it as a PID update (your existing code)
            if (radio.readPID(&new_pids)) {
                printf("NEW PID GAINS and BIAS ANGLES RECEIVED!\n");
                float kp_roll_pitch = new_pids.kp_roll_pitch / 1000.0f;
                float ki_roll_pitch = new_pids.ki_roll_pitch / 1000.0f;
                float kd_roll_pitch = new_pids.kd_roll_pitch / 1000.0f;
                float kp_yaw = new_pids.kp_yaw / 1000.0f;
                float ki_yaw = new_pids.ki_yaw / 1000.0f;
                float kd_yaw = new_pids.kd_yaw / 1000.0f;
                bias_roll = new_pids.bias_roll / 1000.0f;
                bias_pitch = new_pids.bias_pitch / 1000.0f;
                bias_yaw = new_pids.bias_yaw / 1000.0f;
                printf("Bias Updated: roll: %f, pitch: %f, yaw: %f\n",bias_roll,bias_pitch,bias_yaw);
                roll_pid.set_pid(kp_roll_pitch, ki_roll_pitch, kd_roll_pitch);
                pitch_pid.set_pid(kp_roll_pitch, ki_roll_pitch, kd_roll_pitch);
                yaw_pid.set_pid(kp_yaw, ki_yaw, kd_yaw);
                fc_buzzer.play_melody(Tunes::startup, Tunes::startup_len);
            }
        }

        // Send Telemetry at 10Hz
        uint32_t current_time = time_us_32();
        if (send_telemetry && (current_time - last_telemetry_time > 20000)) {
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
            // printf("Roll: %.2f, Pitch: %.2f, Yaw: %.2f, RC_Roll: %f, RC_Pitch: %f, RC_Yaw: %f, PID_Roll: %f, PID_Pitch: %f, PID_Yaw: %f, dt: %f\n",
            //        my_telemetry.roll / 100.0f,
            //        my_telemetry.pitch / 100.0f,
            //        my_telemetry.yaw / 100.0f,
            //        my_telemetry.rc_roll / 100.0f,
            //        my_telemetry.rc_pitch / 100.0f,
            //        my_telemetry.rc_yaw / 100.0f,
            //        my_telemetry.pid_roll / 100.0f,
            //        my_telemetry.pid_pitch / 100.0f,
            //        my_telemetry.pid_yaw / 100.0f,
            //        my_telemetry.dt_us / 100.0f);


        }

        // Tiny sleep to prevent Core 1 from aggressively locking the system bus
        sleep_ms(1);
    }
}

float roll_control_output = 0.0f, pitch_control_output = 0.0f, yaw_control_output = 0.0f;

int main() {
    sleep_ms(100);
    // --- Overclock the CPU to 250 MHz ---
    set_sys_clock_khz(250000, true); // Set system clock to 250 MHz, and wait for it to stabilize

    // Initialize standard I/O for USB serial
    stdio_init_all();
    sleep_ms(3000);
    printf("Started...\n");
    printf("Pico started!\n");
    printf("CPU Frequency: %i kHz\n", clock_get_hz(clk_sys) / 1000);

    // --- Initialize onboard LED ---
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    ReadPWM receiver;
    WritePWM motor;

    fc_buzzer.init();
    fc_buzzer.play_melody(Tunes::startup, Tunes::startup_len);

    // 1. Check Hardware Connections
    bool hardware_ok = true;

    if (!imu.checkConnection()) {
        printf("CRITICAL ERROR: BMI160 not found!\n");
        hardware_ok = false;
    }

    if (!radio.checkConnection()) {
        printf("CRITICAL ERROR: NRF24 not found!\n");
        hardware_ok = false;
    }

    // Halt if ANY critical sensor is missing
    if (!hardware_ok) {
        while (true) {
            fc_buzzer.play_melody(Tunes::error_sensor, Tunes::error_sensor_len);
            sleep_ms(1000);
        }
    }

    printf("All hardware detected! Initializing...\n");
    imu.init();
    radio.init();
    radio.setAddresses(drone_tx_addr, drone_rx_addr);
    radio.startListening();

    printf("Launching Core 1 for Telemetry...\n");
    multicore_launch_core1(core1_entry);

    // --- Configure MPU6000 Interrupt Pin ---
    gpio_init(MPU_INT_PIN);
    gpio_set_dir(MPU_INT_PIN, GPIO_IN);
    gpio_pull_up(MPU_INT_PIN); // BMI160 INT pin is active low, so pull-up is needed
    gpio_set_irq_enabled_with_callback(MPU_INT_PIN, GPIO_IRQ_EDGE_FALL, true, &mpu_isr);
    printf("BMI160 Interrupt configured on GP%d.\n", MPU_INT_PIN);

    // ====================================================================
    // GYROSCOPE CALIBRATION
    // ====================================================================
    printf("Calibrating Gyroscope... DO NOT MOVE THE DRONE!\n");
    fc_buzzer.play_tone(NOTE_C5);
    sleep_ms(100);
    fc_buzzer.stop();

    float gyro_bias_x = 0.0f;
    float gyro_bias_y = 0.0f;
    float gyro_bias_z = 0.0f;
    const int CALIBRATION_SAMPLES = 1000;

    // Discard the first few samples as the sensor settles
    for(int i=0; i<50; i++) {
        while (!imu_data_ready) { tight_loop_contents(); }
        imu_data_ready = false;
    }

    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        while (!imu_data_ready) { tight_loop_contents(); } // Wait for interrupt
        imu_data_ready = false;

        float ax, ay, az, gx, gy, gz;
        imu.readData(&ax, &ay, &az, &gx, &gy, &gz);

        gyro_bias_x += gx;
        gyro_bias_y += gy;
        gyro_bias_z += gz;
    }

    // Average out the errors
    gyro_bias_x /= CALIBRATION_SAMPLES;
    gyro_bias_y /= CALIBRATION_SAMPLES;
    gyro_bias_z /= CALIBRATION_SAMPLES;

    printf("Calibration Complete! Biases: X:%.4f Y:%.4f Z:%.4f\n", gyro_bias_x, gyro_bias_y, gyro_bias_z);

    // Play the success startup tune!
    fc_buzzer.play_melody(Tunes::startup, Tunes::startup_len);
    // ====================================================================

    // Initialize the Filter
    QuaternionEKF filter;

    // Calibration
    //calibrate_receiver(receiver, rc_roll_bias, rc_pitch_bias, rc_yaw_bias);
    printf("Receiver Calibrated: roll_bias: %f, pitch_bias: %f, yaw_bias: %f\n", rc_roll_bias, rc_pitch_bias, rc_yaw_bias);
    bool esc_calibration = false;

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
    last_update_telemetry_us = time_us_32();
    uint64_t last_update_rc_us = time_us_64();
    uint64_t last_update_print_us = time_us_64();
    bool first_throttle_on = true;
    bool run_accel_update = true;

    float dt_ekf, dt_pid;
    uint32_t loop_counter = 0;


    // --- Main Loop ---
    while (true) {

        uint64_t current_pwm_update = time_us_64();
        if ((current_pwm_update - last_update_rc_us) > 1000000/50) {
            receiver.read_pwm(receiver_pwm);
            receiver_pwm[0] -= rc_roll_bias;
            // reverse the roll
            receiver_pwm[0] *= -1;
            receiver_pwm[1] -= rc_pitch_bias;
            receiver_pwm[3] -= rc_yaw_bias;
            last_update_rc_us = current_pwm_update;
        }
        // receiver_pwm[2] = 1006.0f;
        if (receiver_pwm[2] > 1005.0f) {
            if (first_throttle_on) {
                fc_buzzer.play_melody(Tunes::armed, Tunes::armed_len);

                while (receiver_pwm[2] > 1051.0f) {
                    motor.reset();
                    fc_buzzer.play_tone(1);
                    DEBUG_PRINT("Throttle is high! Throttle: %f\n", receiver_pwm[2]);
                    gpio_put(PICO_DEFAULT_LED_PIN, 1);
                    sleep_ms(200);
                    receiver.read_throttle(receiver_pwm[2]);
                    gpio_put(PICO_DEFAULT_LED_PIN, 0);
                    fc_buzzer.stop();
                    sleep_ms(200);
                }
                first_throttle_on = false;
                last_update_ekf_us = time_us_64();
                continue;
            } else {
                gpio_put(PICO_DEFAULT_LED_PIN, 1);
            }
            // Only process data if the IMU interrupt indicates new data is ready
            if (imu_data_ready) {
                // Reset the flag immediately to avoid missing subsequent interrupts
                imu_data_ready = false;
                loop_counter++;

                uint64_t current_time_ekf_us = time_us_64();
                dt_ekf = (current_time_ekf_us - last_update_ekf_us) / 1000000.0f;
                last_update_ekf_us = current_time_ekf_us;
                uint64_t start = time_us_64();

                //Read the IMU data
                float ax, ay, az, gx, gy, gz;
                imu.readData(&ax, &ay, &az, &gx, &gy, &gz);
                // subtract the Biases
                gx -= gyro_bias_x;
                gy -= gyro_bias_y;
                gz -= gyro_bias_z;

                // Execute the Filter
                filter.predict(gx, gy, gz, dt_ekf, q_gyro, q_bias);
                if (run_accel_update) {
                    filter.update(ax, ay, az, r_accel);
                }
                run_accel_update = !run_accel_update;


                uint64_t current_time_pid_us = time_us_64();
                if ((current_time_pid_us - last_update_pid_us) >= 2040) {
                    dt_pid = (current_time_pid_us - last_update_pid_us) / 1000000.0f;
                    last_update_pid_us = current_time_pid_us;
                    filter.getEulerAngles(roll, pitch, yaw);
                    // apply bias
                    roll -= bias_roll;
                    pitch -= bias_pitch;
                    yaw -= bias_yaw;
                    gz = gz * 180.0f / 3.14159265358979323846f;

                    // get the setpoint
                    roll_control_output = roll_pid.compute(receiver_pwm[0], roll, dt_pid);
                    pitch_control_output = pitch_pid.compute(receiver_pwm[1], pitch, dt_pid);
                    yaw_control_output = yaw_pid.compute(receiver_pwm[3], gz, dt_pid);

                    // Motor PWM output
                    motor.update_motors_pwm(receiver_pwm[2], roll_control_output, pitch_control_output, yaw_control_output);

                }

                uint64_t end = time_us_64();
                float dt_us = (end - start) / 1000000.0f;

                if ((end - last_update_print_us) > 1000000/60) {
                    shared_roll = roll;
                    shared_pitch = pitch;
                    shared_yaw = gz;
                    shared_rc_roll = receiver_pwm[0];
                    shared_rc_pitch = receiver_pwm[1];
                    shared_rc_yaw = receiver_pwm[3];
                    shared_pid_roll = roll_control_output;
                    shared_pid_pitch = pitch_control_output;
                    shared_pid_yaw = yaw_control_output;
                    shared_dt_us = dt_ekf;
                    send_telemetry = true;
                    DEBUG_PRINT("Roll: %.2f, Pitch: %.2f, Yaw: %.2f, RC_Roll: %f, RC_Pitch: %f, RC_Yaw: %f, PID_Roll: %f, PID_Pitch: %f, PID_Yaw: %f, dt_pid: %f, dt_ekf: %f\n",
                            roll, pitch, gz, receiver_pwm[0], receiver_pwm[1], receiver_pwm[3], roll_control_output, pitch_control_output, yaw_control_output,
                            dt_pid, dt_ekf);
                    last_update_print_us = end;
                }

            }

        } else {
            motor.reset();
            check_serial_commands();
            send_telemetry = false;
            filter.reset();
            roll_pid.reset();
            pitch_pid.reset();
            yaw_pid.reset();
            first_throttle_on = true;
            DEBUG_PRINT("Not Armed. Throttle: %f\n", receiver_pwm[2]);
            gpio_put(PICO_DEFAULT_LED_PIN, 0);
            sleep_ms(200);
        }
    }

    return 0;
}


