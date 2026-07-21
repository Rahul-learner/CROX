#include "readPWM.h"

float ReadPWM::map_value(float x, float in_min, float in_max, float out_min, float out_max) {
    return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}

// --- NEW: Map with Deadband ---
// Moved to main.cpp

float ReadPWM::ticks_to_us(uint32_t delta_ticks) {
    // Each tick is 1 cycle at 125MHz = 8ns
    return (int)(0xFFFFFFFF - delta_ticks) * (2.0f / 125.0f); // convert to microseconds
}

void ReadPWM::pwm_in_program_init(PIO pio, uint sm, uint offset, uint pin) {
    pio_sm_config c = PwmIn_program_get_default_config(offset);
    sm_config_set_in_pins(&c, pin);
    sm_config_set_jmp_pin(&c, pin);
    sm_config_set_clkdiv(&c, 2.0f); // 250MHz system clock / 1.0 = full speed

    pio_gpio_init(pio, pin);
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, false); //input

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}

ReadPWM::ReadPWM() {
    pio = pio0;
    offset = pio_add_program(pio, &PwmIn_program);
    for (int i = 0; i < NUM_CHANNELS; ++i) {
        pwm_in_program_init(pio, i, offset, channel_gpio_pins[i]);
    }
}

// Read the PWM receiver signal from FIFO
void ReadPWM::read_pwm(float (&channel)[4], float (&raw_channel)[4]) {
    for (int i = 0; i < NUM_CHANNELS; ++i) {
        PIO p = pio;
        uint sm = i;
        if (!pio_sm_is_rx_fifo_empty(p, sm)) {
            uint32_t period_ticks = pio_sm_get(p, sm);
            float period_us = ticks_to_us(period_ticks);
            
            raw_channel[i] = period_us;
            channel[i] = period_us; // Raw values pass through, main.cpp handles mapping
        } else {
            raw_channel[i] = (float)initial_channel[i];
            channel[i] = (float)initial_channel[i];
        }
    }
}

// Read the PWM throttle
void ReadPWM::read_throttle(float& throttle) {
    PIO p = pio;
    uint sm = 2;
    if (!pio_sm_is_rx_fifo_empty(p, sm)) {
        uint32_t period_ticks = pio_sm_get(p, sm);
        float period_us = ticks_to_us(period_ticks);
        throttle = (float)period_us;
    } else {
        throttle = 1000.0f;
    }
}
