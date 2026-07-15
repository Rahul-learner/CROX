#pragma once
#include "config.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <cstdint>

class WritePWM {

private:
    // --- MOTOR CONFIG ---
    //gpio_motor1 = 2 front left, clockwise
    //gpio_motor2 = 3 front right, counter clockwise
    //gpio_motor3 = 5 rear left, counter clockwise
    //gpio_motor4 = 4 rear right, clockwise
    const uint64_t MOTOR_PWM_INTERVAL_US = 1000000 / PWM_FREQUENCY_HZ;


    void set_esc_pulse_us(uint gpio, uint32_t pulse_us);

    void init_esc_pwm_channel(uint gpio_pin);

public:
    uint32_t motor1_speed = 1000;
    uint32_t motor2_speed = 1000;
    uint32_t motor3_speed = 1000;
    uint32_t motor4_speed = 1000;

    WritePWM();
    void update_motors_pwm(uint throttle, float roll_control_output, float pitch_control_output, float yaw_control_output);
    void calibrate_escs(bool calibrate);

    void reset();
};
