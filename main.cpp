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
#include "EKF.h"
#include "readIMU.h"
#include "readPWM.h"
#include "writePWM.h"
#include "PID.h"
#include "buzzer.h"
#include "nrf24_radio.h"
// #include "calibration.h"


// --- Define the GPIO pin for the onboard LED ---
#ifndef PICO_DEFAULT_LED_PIN
#define PICO_DEFAULT_LED_PIN 25
#endif

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
volatile bool new_data_ready = false; // Flag set by interrupt handler

// Global variable for tracking time for filter
static uint64_t last_update_us = 0;

// --- BMI160 Interrupt handler ---
void mpu_isr(uint gpio, uint32_t events) {
    if (gpio == MPU_INT_PIN && (events & GPIO_IRQ_EDGE_FALL)) {
        new_data_ready = true; // Set flag to process data in main loop
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
const uint8_t drone_tx_addr[5] = "GRND1"; // Sending to Ground
const uint8_t drone_rx_addr[5] = "DRON1"; // Receiving from Ground

// Global structs for telemetry
TelemetryPacket telemetry;
PIDTuningPacket new_pids;

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
    if (new_data_ready) {
      new_data_ready = false;
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


    // --- Launch Core1 Telemetry ---
    //Telemetry telemetry;

    // --- Initialize onboard LED ---
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    // --- Initializing PID ---
    PIDController roll_pid(0.0f, 0.0f, 0.0f);
    PIDController pitch_pid(0.0f, 0.0f, 0.0f);
    PIDController yaw_pid(0.0f, 0.0f, 0.0f);

    // --- Initialize Class ---
    BMI160 imu(SPI_PORT, PIN_CS, PIN_SCK, PIN_MOSI, PIN_MISO);
    NRF24 radio(RADIO_SPI_PORT, RADIO_CE, RADIO_CSN, RADIO_SCK, RADIO_MOSI, RADIO_MISO);
    //HMC5883L hmc5883l;
    ReadPWM receiver;
    WritePWM motor;
    Buzzer fc_buzzer(15);

    fc_buzzer.init();
    fc_buzzer.play_melody(Tunes::startup, Tunes::startup_len);

    radio.init();
    radio.setAddresses(drone_tx_addr, drone_rx_addr);
    radio.startListening();
    uint32_t last_telemetry_time = time_us_32();

    // --- Configure MPU6000 Interrupt Pin ---
    gpio_init(MPU_INT_PIN);
    gpio_set_dir(MPU_INT_PIN, GPIO_IN);
    gpio_pull_up(MPU_INT_PIN); // BMI160 INT pin is active low, so pull-up is needed
    gpio_set_irq_enabled_with_callback(MPU_INT_PIN, GPIO_IRQ_EDGE_FALL, true, &mpu_isr);
    printf("BMI160 Interrupt configured on GP%d.\n", MPU_INT_PIN);

    // --- Configure ---
    if (imu.checkConnection()) {
        printf("BMI160 detected! Initializing sensors...\n");
        imu.init();
        printf("BMI160 configured for 3200Hz.\n");
    } else {
        printf("ERROR: BMI160 not found. Check wiring!\n");
        while (true) {
            fc_buzzer.play_melody(Tunes::error_sensor, Tunes::error_sensor_len);
            sleep_ms(1000); } // Halt execution
    }

    // Initialize the Filter
    QuaternionEKF filter;
    //MahonyFilter filter;

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
    last_update_us = time_us_64();
    last_telemetry_time = time_us_32();
    uint64_t last_pwm_update = time_us_64();
    uint64_t last_print_update = time_us_64();
    bool first_throttle_on = true;
    bool run_accel_update = true;

    float yaw_rate, dt;
    float roll_bias = 0.0f, pitch_bias = 0.0f;
    uint32_t loop_counter = 0;


    // --- Main Loop ---
    while (true) {

        uint64_t current_pwm_update = time_us_64();
        float elasped = current_pwm_update - last_pwm_update;
        if (elasped > 1000000/50) {
            receiver.read_pwm(receiver_pwm);
            receiver_pwm[0] -= rc_roll_bias;
            // reverse the roll
            receiver_pwm[0] *= -1;
            receiver_pwm[1] -= rc_pitch_bias;
            receiver_pwm[3] -= rc_yaw_bias;
            // Send Telemetry
            //telemetry.send_telemetry(roll, pitch, yaw_rate, dt, receiver_pwm[0], receiver_pwm[1], receiver_pwm[3], 0.0f, 0.0f, 0.0f);
            last_pwm_update = current_pwm_update;
        }
        // receiver_pwm[2] = 1006.0f;
        if (receiver_pwm[2] > 1005.0f) {
            if (first_throttle_on) {
                fc_buzzer.play_melody(Tunes::armed, Tunes::armed_len);

                while (receiver_pwm[2] > 1051.0f) {
                    motor.reset();
                    fc_buzzer.play_tone(1);
                    printf("Throttle is high! Throttle: %f\n", receiver_pwm[2]);
                    gpio_put(PICO_DEFAULT_LED_PIN, 1);
                    sleep_ms(200);
                    receiver.read_throttle(receiver_pwm[2]);
                    gpio_put(PICO_DEFAULT_LED_PIN, 0);
                    fc_buzzer.stop();
                    sleep_ms(200);
                }
                first_throttle_on = false;
                last_update_us = time_us_64();
                continue;
            } else {
                gpio_put(PICO_DEFAULT_LED_PIN, 1);
            }
            // ESC Calibration
            if (esc_calibration) {
                motor.update_motors_pwm(2000.0f, 0.0f, 0.0f, 0.0f);
                printf("ESC Calibration. THROTTLE IS MAX!\n");
                sleep_ms(200);
                continue;
            }
            // Only process data if the MPU6050 interrupt indicates new data is ready
            if (new_data_ready) {
                // Reset the flag immediately to avoid missing subsequent interrupts
                new_data_ready = false;
                loop_counter++;

                uint64_t current_time_us = time_us_64();
                dt = (current_time_us - last_update_us) / 1000000.0f;
                last_update_us = current_time_us;
                uint64_t start = time_us_64();

                //Read the IMU data
                float ax, ay, az, gx, gy, gz;
                imu.readData(&ax, &ay, &az, &gx, &gy, &gz);

                // Execute the Filter
                filter.predict(gx, gy, gz, dt, 0.001f, 0.00001f);
                if (run_accel_update) {
                    filter.update(ax, ay, az, 1.5f);
                }
                run_accel_update = !run_accel_update;


                if (loop_counter % 64 == 0) {
                    filter.getEulerAngles(roll, pitch, yaw);
                    filter.get_clean_rates(gz, yaw_rate);
                    // get the setpoint
                    roll_control_output = roll_pid.compute(receiver_pwm[0], roll, dt);
                    pitch_control_output = pitch_pid.compute(receiver_pwm[1], pitch, dt);
                    yaw_control_output = yaw_pid.compute(receiver_pwm[3], yaw_rate, dt);

                    // Motor PWM output
                    motor.update_motors_pwm(receiver_pwm[2], roll_control_output, pitch_control_output, yaw_control_output);

                }

                uint64_t end = time_us_64();
                float dt_s = (end - start) / 1000000.0f;

                if ((end - last_telemetry_time) > 50000) {
                    last_telemetry_time = end;
                    telemetry.roll = roll;
                    telemetry.pitch = pitch;

                    radio.sendTelemetry(&telemetry);
                }

                if ((end - last_print_update) > 1000000/100) {
                    gz = gz * 180.0f / 3.14159265358979323846f; 
                    printf("Roll: %f, Pitch: %f, Yaw_Rate: %f, raw_yaw_rate: %f, dt: %f, dt_s: %f\n", roll, pitch, yaw_rate, gz, dt, dt_s);
                    // printf("AX: %f, AY: %f, AZ: %f, GX: %f, GY: %f, GZ: %f, dt: %f\n", ax, ay, az, gx, gy, gz, dt);
                    // printf("D-Roll: %f, D-Pitch: %f, Y-Pitch: %f\n", receiver_pwm[0], receiver_pwm[1], receiver_pwm[3]);
                    last_print_update = end;
                }

            }

        } else {
            motor.reset();
            if (radio.dataAvailable()) {
                radio.readPID(&new_pids);

                printf("New Pid Gains Received!\n");
            }
            roll_pid.reset();
            pitch_pid.reset();
            yaw_pid.reset();
            first_throttle_on = true;
            printf("Not Armed. Throttle: %f\n", receiver_pwm[2]);
            gpio_put(PICO_DEFAULT_LED_PIN, 0);
            sleep_ms(200);
        }
    }

    return 0;
}


