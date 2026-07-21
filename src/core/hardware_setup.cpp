#include "core/hardware_setup.h"
#include "core/globals.h"
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "pico/multicore.h"
#include "config.h"
#include "core/comm_tasks.h"
#include <stdio.h>

#ifndef PICO_DEFAULT_LED_PIN
#define PICO_DEFAULT_LED_PIN 25
#endif

#ifndef DEBUG_PRINT
#ifdef DEBUG_MODE
#define DEBUG_PRINT(...) printf(__VA_ARGS__)
#else
#define DEBUG_PRINT(...)
#endif
#endif

void mpu_isr(uint gpio, uint32_t events) {
    if (gpio == IMU_INT_PIN && (events & GPIO_IRQ_EDGE_FALL)) {
        imu_data_ready = true;
    }
}

bool init_hardware() {
    sleep_ms(100);
    set_sys_clock_khz(CPU_FREQ_KHZ, true);

    stdio_init_all();
    sleep_ms(3000);
    DEBUG_PRINT("Started...\n");
    DEBUG_PRINT("Pico started!\n");
    DEBUG_PRINT("CPU Frequency: %i kHz\n", clock_get_hz(clk_sys) / 1000);

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    fc_buzzer.init();
    fc_buzzer.play_melody(Tunes::startup, Tunes::startup_len);

    bool hardware_ok = true;

    if (!imu.checkConnection()) {
        DEBUG_PRINT("CRITICAL ERROR: BMI160 not found!\n");
        hardware_ok = false;
    }

#if USE_NRF24_RADIO
    if (!radio.checkConnection()) {
        DEBUG_PRINT("CRITICAL ERROR: NRF24 not found!\n");
        hardware_ok = false;
    }
#endif

    if (!hardware_ok) {
        while (true) {
            fc_buzzer.play_melody(Tunes::error_sensor, Tunes::error_sensor_len);
            sleep_ms(1000);
        }
    }

    DEBUG_PRINT("All hardware detected! Initializing...\n");
    imu.init();
#if USE_NRF24_RADIO
    radio.init();
    radio.setAddresses(drone_tx_addr, drone_rx_addr);
    radio.startListening();
#endif

    DEBUG_PRINT("Launching Core 1 for Telemetry...\n");
    multicore_launch_core1(core1_entry);

    gpio_init(IMU_INT_PIN);
    gpio_set_dir(IMU_INT_PIN, GPIO_IN);
    gpio_pull_up(IMU_INT_PIN);
    gpio_set_irq_enabled_with_callback(IMU_INT_PIN, GPIO_IRQ_EDGE_FALL, true, &mpu_isr);
    DEBUG_PRINT("BMI160 Interrupt configured on GP%d.\n", IMU_INT_PIN);

    return true;
}
