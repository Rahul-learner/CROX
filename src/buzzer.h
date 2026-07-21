#pragma once
#include "pico/stdlib.h"
#include "hardware/pwm.h"

// --- Standard Musical Frequencies (Hz) ---
#define REST      0
#define NOTE_C4  262
#define NOTE_D4  294
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_G4  392
#define NOTE_A4  440
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_D5  587
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_G5  784
#define NOTE_A5  880
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_E6  1319
#define NOTE_G6  1568

// Structure to hold a musical note and its duration
struct MelodyNote {
    uint16_t frequency;
    uint16_t duration_ms;
};

// --- Flight Controller Status Tunes ---
namespace Tunes {
    // 1. Successful Startup (Ascending triumphant chord)
    static const MelodyNote startup[] = {
        {NOTE_C5, 100}, {NOTE_E5, 100}, {NOTE_G5, 100}, {NOTE_C6, 300}
    };
    static const int startup_len = sizeof(startup) / sizeof(startup[0]);

    // 2. Sensor Error / MPU Not Found (Sad descending tones)
    static const MelodyNote error_sensor[] = {
        {NOTE_G4, 250}, {REST, 50}, {NOTE_C4, 500}
    };
    static const int error_sensor_len = sizeof(error_sensor) / sizeof(error_sensor[0]);

    // 3. System Armed / Ready to fly (Two quick high beeps)
    static const MelodyNote armed[] = {
        {NOTE_G6, 100}, {REST, 50}, {NOTE_G6, 100}
    };
    static const int armed_len = sizeof(armed) / sizeof(armed[0]);

    // 4. System Disarmed (Two descending tones)
    static const MelodyNote disarmed[] = {
        {NOTE_G5, 100}, {REST, 50}, {NOTE_C5, 150}
    };
    static const int disarmed_len = sizeof(disarmed) / sizeof(disarmed[0]);

    // 5. Error: Not Level (Harsh buzzer warning)
    static const MelodyNote error_level[] = {
        {NOTE_C4, 150}, {REST, 50}, {NOTE_C4, 150}, {REST, 50}, {NOTE_C4, 150}
    };
    static const int error_level_len = sizeof(error_level) / sizeof(error_level[0]);

    // 4. Low Battery or RC Signal Loss Alarm (Loud, annoying repeating pattern)
    static const MelodyNote alarm_warning[] = {
        {NOTE_C6, 150}, {REST, 150}, {NOTE_C6, 150}, {REST, 500}
    };
    static const int alarm_warning_len = sizeof(alarm_warning) / sizeof(alarm_warning[0]);
}

class Buzzer {
private:
    uint pin;
    uint slice_num;
    uint channel;

public:
    Buzzer(uint gpio_pin);

    void init();

    // Play a single continuous frequency
    void play_tone(uint16_t frequency_hz);

    void stop();

    // --- PLAY A SEQUENCE OF NOTES (BLOCKING!) ---
    // Safely use this during setup() or fatal error loops.
    // WARNING: Do not call this inside your main 3.2kHz flight loop!
    void play_melody(const MelodyNote* notes, int num_notes);
};
