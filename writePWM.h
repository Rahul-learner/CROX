#pragma once
#include "pico/stdlib.h"
#include "hardware/pwm.h"

class WritePWM {

private:
    // --- MOTOR CONFIG ---
    //gpio_motor1 = 2 front left, clockwise
    //gpio_motor2 = 3 front right, counter clockwise
    //gpio_motor3 = 5 rear left, counter clockwise
    //gpio_motor4 = 4 rear right, clockwise
    #define MOTOR1_PWM_PIN 2 // Example: GP2 (PWM Slice 0, Channel A)
    #define MOTOR2_PWM_PIN 3 // Example: GP3 (PWM Slice 0, Channel B)
    #define MOTOR3_PWM_PIN 5 // Example: GP4 (PWM Slice 1, Channel A)
    #define MOTOR4_PWM_PIN 4 // Example: GP5 (PWM Slice 1, Channel B)
    #define PWM_FREQUENCY_HZ 490
    #define PWM_WRAP_VALUE 2040 // For 490Hz, Period = 1,000,000 / 490 = 2040
    #define ESC_MIN_PULSE_US 1000
    #define ESC_MAX_PULSE_US 2000
    const uint64_t MOTOR_PWM_INTERVAL_US = 1000000 / PWM_FREQUENCY_HZ;

    void set_esc_pulse_us(uint gpio, uint32_t pulse_us) {
        // Ensure pulse_us is within the valid ESC range
        if (pulse_us < ESC_MIN_PULSE_US) {
            pulse_us = ESC_MIN_PULSE_US;
        }
        if (pulse_us > ESC_MAX_PULSE_US) {
            pulse_us = ESC_MAX_PULSE_US;
        }

        pwm_set_chan_level(pwm_gpio_to_slice_num(gpio), pwm_gpio_to_channel(gpio), pulse_us);
    }

    void init_esc_pwm_channel(uint gpio_pin) {
        // Get the slice and channel for the chosen GPIO pin
        uint slice_num = pwm_gpio_to_slice_num(gpio_pin);
        uint channel_num = pwm_gpio_to_channel(gpio_pin);

        // Tell GPIO pin to connect to PWM
        gpio_set_function(gpio_pin, GPIO_FUNC_PWM);

        // Configure PWM for this slice (only needs to be done once peer slice)
        pwm_config config = pwm_get_default_config();

        // Calculate the clock divider
        // clk_div = System_Clock / (PWM_Frequency * (Wrap_Value + 1))
        float clock_divider = (float)clock_get_hz(clk_sys) / 1000000.0f; // Each tick = 1us
        pwm_config_set_clkdiv(&config, clock_divider);

        // Set the wrap value (TOP)
        pwm_config_set_wrap(&config, PWM_WRAP_VALUE);
        pwm_init(slice_num, &config, true);

        printf("PWM configured on GP%d (Slice %d, Channel %d) at %d Hz.\n", gpio_pin, slice_num, channel_num, PWM_FREQUENCY_HZ);
    }

public:
    WritePWM() {
        init_esc_pwm_channel(MOTOR1_PWM_PIN);
        init_esc_pwm_channel(MOTOR2_PWM_PIN);
        init_esc_pwm_channel(MOTOR3_PWM_PIN);
        init_esc_pwm_channel(MOTOR4_PWM_PIN);
        printf("Sending MIN throttle to arm ESCs...\n");
        set_esc_pulse_us(MOTOR1_PWM_PIN, ESC_MIN_PULSE_US);
        set_esc_pulse_us(MOTOR2_PWM_PIN, ESC_MIN_PULSE_US);
        set_esc_pulse_us(MOTOR3_PWM_PIN, ESC_MIN_PULSE_US);
        set_esc_pulse_us(MOTOR4_PWM_PIN, ESC_MIN_PULSE_US);
    }
    void update_motors_pwm(uint throttle, float roll_control_output, float pitch_control_output, float yaw_control_output) {
        uint32_t motor1_speed = throttle + roll_control_output - pitch_control_output + yaw_control_output;
        uint32_t motor2_speed = throttle - roll_control_output - pitch_control_output - yaw_control_output;
        uint32_t motor3_speed = throttle + roll_control_output + pitch_control_output - yaw_control_output;
        uint32_t motor4_speed = throttle - roll_control_output + pitch_control_output + yaw_control_output;

        set_esc_pulse_us(MOTOR1_PWM_PIN, motor1_speed);
        set_esc_pulse_us(MOTOR2_PWM_PIN, motor2_speed);
        set_esc_pulse_us(MOTOR3_PWM_PIN, motor3_speed);
        set_esc_pulse_us(MOTOR4_PWM_PIN, motor4_speed);
    }

    void reset() {
        set_esc_pulse_us(MOTOR1_PWM_PIN, ESC_MIN_PULSE_US);
        set_esc_pulse_us(MOTOR2_PWM_PIN, ESC_MIN_PULSE_US);
        set_esc_pulse_us(MOTOR3_PWM_PIN, ESC_MIN_PULSE_US);
        set_esc_pulse_us(MOTOR4_PWM_PIN, ESC_MIN_PULSE_US);
    }
};
