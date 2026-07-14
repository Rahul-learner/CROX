#include "buzzer.h"

Buzzer::Buzzer(uint gpio_pin) {
    pin = gpio_pin;
}

void Buzzer::init() {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    slice_num = pwm_gpio_to_slice_num(pin);
    channel = pwm_gpio_to_channel(pin);

    // 1MHz PWM clock (Assuming standard 125MHz Pico clock)
    pwm_set_clkdiv(slice_num, 125.0f);

    pwm_set_chan_level(slice_num, channel, 0); // Start silent
    pwm_set_enabled(slice_num, true);
}

// Play a single continuous frequency
void Buzzer::play_tone(uint16_t frequency_hz) {
    if (frequency_hz == 0) {
        stop();
        return;
    }
    uint32_t wrap = (1000000 / frequency_hz) - 1;
    pwm_set_wrap(slice_num, wrap);
    // 50% duty cycle creates a perfect square wave for max volume
    pwm_set_chan_level(slice_num, channel, wrap / 2);
}

void Buzzer::stop() {
    pwm_set_chan_level(slice_num, channel, 0);
}

// --- PLAY A SEQUENCE OF NOTES (BLOCKING!) ---
// Safely use this during setup() or fatal error loops.
// WARNING: Do not call this inside your main 3.2kHz flight loop!
void Buzzer::play_melody(const MelodyNote* notes, int num_notes) {
    for (int i = 0; i < num_notes; i++) {
        if (notes[i].frequency == REST) {
            stop();
        } else {
            play_tone(notes[i].frequency);
        }

        // Hold the note
        sleep_ms(notes[i].duration_ms);

        // Add a tiny 20ms silent gap between notes so identical notes don't blur together
        stop();
        sleep_ms(20);
    }
}
