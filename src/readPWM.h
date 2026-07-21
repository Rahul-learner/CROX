#pragma once
#include "config.h"
#include "hardware/pwm.h"
#include "PwmIn.pio.h"
#include "pico/stdlib.h"

class ReadPWM {

private:
    const uint16_t channel_gpio_pins[NUM_CHANNELS] = {RC_CH1_PIN, RC_CH2_PIN, RC_CH3_PIN, RC_CH4_PIN};
    //volatile uint32_t channel[4] = {1500, 1500, 1000, 1500};
    uint32_t initial_channel[4] = {1500, 1500, 1002, 1500};


    float map_value(float x, float in_min, float in_max, float out_min, float out_max);

    float ticks_to_us(uint32_t delta_ticks);

    void pwm_in_program_init(PIO pio, uint sm, uint offset, uint pin);

public:
    PIO pio;
    uint offset;

    ReadPWM();

    // Read the PWM receiver signal from FIFO (both mapped and raw microseconds)
    void read_pwm(float (&channel)[4], float (&raw_channel)[4]);

    // Read the PWM throttle
    void read_throttle(float& throttle);
};
