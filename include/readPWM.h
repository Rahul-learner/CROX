#pragma once
#include "hardware/pwm.h"
#include "PwmIn.pio.h"
#include "pico/stdlib.h"

class ReadPWM {

private:
    #define NUM_CHANNELS 4
    const uint16_t channel_gpio_pins[NUM_CHANNELS] = {9, 8, 7, 6};
    //volatile uint32_t channel[4] = {1500, 1500, 1000, 1500};
    uint32_t initial_channel[4] = {1500, 1500, 1002, 1500};

    const float CONTROL_OUT_MIN = -30.0f;
    const float CONTROL_OUT_MAX = 30.0f;
    const float DEADBAND_US = 8.0f;   // +/- 8us from center
    const float RC_CENTER_US = 1500.0f;

    float map_value(float x, float in_min, float in_max, float out_min, float out_max);

    // --- NEW: Map with Deadband ---
    float map_with_deadband(float pulse_us);

    float ticks_to_us(uint32_t delta_ticks);

    void pwm_in_program_init(PIO pio, uint sm, uint offset, uint pin);

public:
    PIO pio;
    uint offset;

    ReadPWM();

    // Read the PWM receiver signal from FIFO
    void read_pwm(float (&channel)[4]);

    // Read the PWM throttle
    void read_throttle(float& throttle);
};
