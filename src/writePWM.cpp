#include "writePWM.h"
#include "config.h"
#include <stdio.h>
#include "hardware/clocks.h"

void WritePWM::set_esc_pulse_us(uint gpio, uint32_t pulse_us) {
    // Ensure pulse_us is within the valid ESC range
    if (pulse_us < ESC_MIN_PULSE_US) {
        pulse_us = ESC_MIN_PULSE_US;
    }
    if (pulse_us > ESC_MAX_PULSE_US) {
        pulse_us = ESC_MAX_PULSE_US;
    }

    pwm_set_chan_level(pwm_gpio_to_slice_num(gpio), pwm_gpio_to_channel(gpio), pulse_us);
}

void WritePWM::init_esc_pwm_channel(uint gpio_pin) {
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

    DEBUG_PRINT("PWM configured on GP%d (Slice %d, Channel %d) at %d Hz.\\n", gpio_pin, slice_num, channel_num, PWM_FREQUENCY_HZ);
}

WritePWM::WritePWM() {
    init_esc_pwm_channel(MOTOR1_PWM_PIN);
    init_esc_pwm_channel(MOTOR2_PWM_PIN);
    init_esc_pwm_channel(MOTOR3_PWM_PIN);
    init_esc_pwm_channel(MOTOR4_PWM_PIN);
    DEBUG_PRINT("Sending MIN throttle to arm ESCs...\\n");
    set_esc_pulse_us(MOTOR1_PWM_PIN, ESC_MIN_PULSE_US);
    set_esc_pulse_us(MOTOR2_PWM_PIN, ESC_MIN_PULSE_US);
    set_esc_pulse_us(MOTOR3_PWM_PIN, ESC_MIN_PULSE_US);
    set_esc_pulse_us(MOTOR4_PWM_PIN, ESC_MIN_PULSE_US);

    // set all motor_speed to 1000
    motor1_speed = ESC_MIN_PULSE_US;
    motor2_speed = ESC_MIN_PULSE_US;
    motor3_speed = ESC_MIN_PULSE_US;
    motor4_speed = ESC_MIN_PULSE_US;
}

void WritePWM::update_motors_pwm(uint throttle, float roll_control_output, float pitch_control_output, float yaw_control_output) {
    int32_t roll = (int32_t)roll_control_output;
    int32_t pitch = (int32_t)pitch_control_output;
    int32_t yaw = (int32_t)yaw_control_output;
    // FIX 1: Flipped Pitch Signs (Front +, Rear -) to fix the tilting issue
    // FIX 2: Flipped Yaw Signs for your specific CW/CCW configuration
    // We use int32_t temporarily to safely handle numbers that dip below 0
    int32_t m1 = throttle + roll - pitch + yaw; // M1: Front Left (CW)
    int32_t m2 = throttle - roll - pitch - yaw; // M2: Front Right (CCW)
    int32_t m3 = throttle + roll + pitch - yaw; // M3: Rear Left (CCW)
    int32_t m4 = throttle - roll + pitch + yaw; // M4: Rear Right (CW)

    // FIX 3: Add constraints to prevent ESC desync if PID spikes outside 1000us-2000us
    motor1_speed = m1 > ESC_MAX_PULSE_US ? ESC_MAX_PULSE_US : (m1 < ESC_MIN_PULSE_US ? ESC_MIN_PULSE_US : m1);
    motor2_speed = m2 > ESC_MAX_PULSE_US ? ESC_MAX_PULSE_US : (m2 < ESC_MIN_PULSE_US ? ESC_MIN_PULSE_US : m2);
    motor3_speed = m3 > ESC_MAX_PULSE_US ? ESC_MAX_PULSE_US : (m3 < ESC_MIN_PULSE_US ? ESC_MIN_PULSE_US : m3);
    motor4_speed = m4 > ESC_MAX_PULSE_US ? ESC_MAX_PULSE_US : (m4 < ESC_MIN_PULSE_US ? ESC_MIN_PULSE_US : m4);

    set_esc_pulse_us(MOTOR1_PWM_PIN, motor1_speed);
    set_esc_pulse_us(MOTOR2_PWM_PIN, motor2_speed);
    set_esc_pulse_us(MOTOR3_PWM_PIN, motor3_speed);
    set_esc_pulse_us(MOTOR4_PWM_PIN, motor4_speed);
}

void WritePWM::reset() {
    set_esc_pulse_us(MOTOR1_PWM_PIN, ESC_MIN_PULSE_US);
    set_esc_pulse_us(MOTOR2_PWM_PIN, ESC_MIN_PULSE_US);
    set_esc_pulse_us(MOTOR3_PWM_PIN, ESC_MIN_PULSE_US);
    set_esc_pulse_us(MOTOR4_PWM_PIN, ESC_MIN_PULSE_US);

    // set all motor_speed to 1000
    motor1_speed = ESC_MIN_PULSE_US;
    motor2_speed = ESC_MIN_PULSE_US;
    motor3_speed = ESC_MIN_PULSE_US;
    motor4_speed = ESC_MIN_PULSE_US;
}

void WritePWM::calibrate_escs(bool calibrate) {
    if (!calibrate) return;

    DEBUG_PRINT("Starting ESC Calibration...\n");
    DEBUG_PRINT("Setting MAX throttle (2000us) to all motors...\n");
    set_esc_pulse_us(MOTOR1_PWM_PIN, ESC_MAX_PULSE_US);
    set_esc_pulse_us(MOTOR2_PWM_PIN, ESC_MAX_PULSE_US);
    set_esc_pulse_us(MOTOR3_PWM_PIN, ESC_MAX_PULSE_US);
    set_esc_pulse_us(MOTOR4_PWM_PIN, ESC_MAX_PULSE_US);
    
    DEBUG_PRINT("Wait for ESCs to register MAX throttle. Plug in battery now if unplugged!\n");
    sleep_ms(5000);
    
    DEBUG_PRINT("Setting MIN throttle (1000us) to all motors...\n");
    set_esc_pulse_us(MOTOR1_PWM_PIN, ESC_MIN_PULSE_US);
    set_esc_pulse_us(MOTOR2_PWM_PIN, ESC_MIN_PULSE_US);
    set_esc_pulse_us(MOTOR3_PWM_PIN, ESC_MIN_PULSE_US);
    set_esc_pulse_us(MOTOR4_PWM_PIN, ESC_MIN_PULSE_US);
    
    DEBUG_PRINT("Wait for ESC confirmation beeps...\n");
    sleep_ms(3000);
    DEBUG_PRINT("ESC Calibration Complete!\n");
}
