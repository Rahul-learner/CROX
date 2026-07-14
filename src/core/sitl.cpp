#include "core/sitl.h"
#include "core/globals.h"
#include "pico/time.h"
#include "pico/stdlib.h"
#include <stdio.h>

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
        if (imu_data_ready) {
            imu_data_ready = false;
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
