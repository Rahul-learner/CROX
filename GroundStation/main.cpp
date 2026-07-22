#include <cstdint>
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "nrf24_radio.h"

// --- NRF24 Hardware Pin Definitions ---
#define NRF_SPI_PORT spi1
#define NRF_MISO_PIN 12
#define NRF_CSN_PIN  13
#define NRF_SCK_PIN  10
#define NRF_MOSI_PIN 11
#define NRF_CE_PIN   14

NRF24 radio(NRF_SPI_PORT, NRF_CE_PIN, NRF_CSN_PIN, NRF_SCK_PIN, NRF_MOSI_PIN, NRF_MISO_PIN);

uint32_t last_radio_activity = 0;

// Helper to convert float to/from bytes
void pack_float(uint8_t* dest, float val) {
    memcpy(dest, &val, 4);
}
float unpack_float(uint8_t* src) {
    float val;
    memcpy(&val, src, 4);
    return val;
}

// Translates a USB string command into a RadioCommandPacket, sends it, and prints the response.
void process_command_and_bridge(char* buffer) {
    RadioCommandPacket cmd = {0};
    cmd.header1 = CMD_HEADER_1;
    cmd.header2 = CMD_HEADER_2;

    // Parse the command string to cmd_id and payload
if (strncmp(buffer, "CHECK_RADIO", 11) == 0) {
    if (radio.checkConnection()) { printf("RADIO_OK\n"); } else { printf("RADIO_FAIL\n"); }
    return;
} else if (strncmp(buffer, "GET_PID_ACRO_RP", 15) == 0) {
        cmd.cmd_id = 0x02;
    } else if (strncmp(buffer, "SET_PID_ACRO_RP,", 16) == 0) {
        cmd.cmd_id = 0x03;
        float p, i, d;
        sscanf(buffer, "SET_PID_ACRO_RP,%f,%f,%f", &p, &i, &d);
        ((int16_t*)cmd.payload)[0] = (int16_t)(p * 1000.0f);
        ((int16_t*)cmd.payload)[1] = (int16_t)(i * 1000.0f);
        ((int16_t*)cmd.payload)[2] = (int16_t)(d * 1000.0f);
    } else if (strncmp(buffer, "GET_PID_ANGLE_RP", 16) == 0) {
        cmd.cmd_id = 0x04;
    } else if (strncmp(buffer, "SET_PID_ANGLE_RP,", 17) == 0) {
        cmd.cmd_id = 0x05;
        float p, i, d;
        sscanf(buffer, "SET_PID_ANGLE_RP,%f,%f,%f", &p, &i, &d);
        ((int16_t*)cmd.payload)[0] = (int16_t)(p * 1000.0f);
        ((int16_t*)cmd.payload)[1] = (int16_t)(i * 1000.0f);
        ((int16_t*)cmd.payload)[2] = (int16_t)(d * 1000.0f);
    } else if (strncmp(buffer, "GET_PID_YAW", 11) == 0) {
        cmd.cmd_id = 0x06;
    } else if (strncmp(buffer, "SET_PID_YAW,", 12) == 0) {
        cmd.cmd_id = 0x07;
        float p, i, d;
        sscanf(buffer, "SET_PID_YAW,%f,%f,%f", &p, &i, &d);
        ((int16_t*)cmd.payload)[0] = (int16_t)(p * 1000.0f);
        ((int16_t*)cmd.payload)[1] = (int16_t)(i * 1000.0f);
        ((int16_t*)cmd.payload)[2] = (int16_t)(d * 1000.0f);
    } else if (strncmp(buffer, "GET_EKF", 7) == 0) {
        cmd.cmd_id = 0x08;
    } else if (strncmp(buffer, "SET_EKF,", 8) == 0) {
        cmd.cmd_id = 0x09;
        float qg, qb, ra;
        sscanf(buffer, "SET_EKF,%f,%f,%f", &qg, &qb, &ra);
        ((int32_t*)cmd.payload)[0] = (int32_t)(qg * 1e6f);
        ((int32_t*)cmd.payload)[1] = (int32_t)(qb * 1e6f);
        ((int32_t*)cmd.payload)[2] = (int32_t)(ra * 1e6f);
    } else if (strncmp(buffer, "GET_BIAS", 8) == 0) {
        cmd.cmd_id = 0x0A;
    } else if (strncmp(buffer, "SET_BIAS,", 9) == 0) {
        cmd.cmd_id = 0x0B;
        float r, p, y;
        sscanf(buffer, "SET_BIAS,%f,%f,%f", &r, &p, &y);
        ((int16_t*)cmd.payload)[0] = (int16_t)(r * 1000.0f);
        ((int16_t*)cmd.payload)[1] = (int16_t)(p * 1000.0f);
        ((int16_t*)cmd.payload)[2] = (int16_t)(y * 1000.0f);
    } else if (strncmp(buffer, "GET_RC_TUNE", 11) == 0) {
        cmd.cmd_id = 0x0C;
    } else if (strncmp(buffer, "SET_RC_TUNE,", 12) == 0) {
        cmd.cmd_id = 0x0D;
        float expo, db, ydb, rc, pc, yc;
        int rr, pr, yr;
        sscanf(buffer, "SET_RC_TUNE,%f,%f,%f,%f,%f,%f,%d,%d,%d", &expo, &db, &ydb, &rc, &pc, &yc, &rr, &pr, &yr);
        pack_float(cmd.payload, expo);
        pack_float(cmd.payload + 4, db);
        pack_float(cmd.payload + 8, ydb);
        pack_float(cmd.payload + 12, rc);
        pack_float(cmd.payload + 16, pc);
        pack_float(cmd.payload + 20, yc);
        cmd.payload[24] = rr > 0 ? 1 : 0;
        cmd.payload[25] = pr > 0 ? 1 : 0;
        cmd.payload[26] = yr > 0 ? 1 : 0;
    } else if (strncmp(buffer, "GET_RC", 6) == 0) {
        cmd.cmd_id = 0x0E;
    } else if (strncmp(buffer, "GET_STATUS", 10) == 0) {
        cmd.cmd_id = 0x0F;
    } else if (strncmp(buffer, "GET_IMU", 7) == 0) {
        cmd.cmd_id = 0x10;
    } else if (strncmp(buffer, "SET_MODE,", 9) == 0) {
        cmd.cmd_id = 0x11;
        if (strncmp(buffer + 9, "ACRO", 4) == 0) cmd.payload[0] = 1;
        else if (strncmp(buffer + 9, "ANGLE", 5) == 0) cmd.payload[0] = 0;
    } else if (strncmp(buffer, "GET_MOTORS", 10) == 0) {
        cmd.cmd_id = 0x12;
    } else if (strncmp(buffer, "GET_FF", 6) == 0) {
        cmd.cmd_id = 0x14;
    } else if (strncmp(buffer, "SET_FF,", 7) == 0) {
        cmd.cmd_id = 0x15;
        float r, p, y;
        sscanf(buffer, "SET_FF,%f,%f,%f", &r, &p, &y);
        ((int16_t*)cmd.payload)[0] = (int16_t)(r * 1000.0f);
        ((int16_t*)cmd.payload)[1] = (int16_t)(p * 1000.0f);
        ((int16_t*)cmd.payload)[2] = (int16_t)(y * 1000.0f);
    } else if (strncmp(buffer, "GET_TPA", 7) == 0) {
        cmd.cmd_id = 0x16;
    } else if (strncmp(buffer, "SET_TPA,", 8) == 0) {
        cmd.cmd_id = 0x17;
        float bp, f;
        sscanf(buffer, "SET_TPA,%f,%f", &bp, &f);
        ((int16_t*)cmd.payload)[0] = (int16_t)(bp * 1000.0f);
        ((int16_t*)cmd.payload)[1] = (int16_t)(f * 1000.0f);
    } else if (strncmp(buffer, "GET_I_LIMIT", 11) == 0) {
        cmd.cmd_id = 0x18;
    } else if (strncmp(buffer, "SET_I_LIMIT,", 12) == 0) {
        cmd.cmd_id = 0x19;
        float l;
        sscanf(buffer, "SET_I_LIMIT,%f", &l);
        ((int16_t*)cmd.payload)[0] = (int16_t)(l * 100.0f);
    } else if (strncmp(buffer, "GET_D_CUTOFF", 12) == 0) {
        cmd.cmd_id = 0x1A;
    } else if (strncmp(buffer, "SET_D_CUTOFF,", 13) == 0) {
        cmd.cmd_id = 0x1B;
        float f;
        sscanf(buffer, "SET_D_CUTOFF,%f", &f);
        ((int16_t*)cmd.payload)[0] = (int16_t)(f * 100.0f);
    } else if (strncmp(buffer, "CALIBRATE_ACCEL", 15) == 0) {
        cmd.cmd_id = 0x1C;
    } else if (strncmp(buffer, "CALIBRATE_NOISE", 15) == 0) {
        cmd.cmd_id = 0x1D;
    } else if (strncmp(buffer, "REBOOT", 6) == 0) {
        cmd.cmd_id = 0x1E;
    } else if (strncmp(buffer, "GET_CONFIG", 10) == 0) {
        // Special case: configurator uses this for auto-detection
        printf("DEVICE_TYPE,GROUNDSTATION\n");
        return;
    } else {
        // Unknown or unsupported command (e.g. BLACKBOX, MOTOR_TEST)
        return;
    }

    // Transmit over radio
    if (!radio.sendCommand(&cmd)) {
        printf("GS_NACK %s\n", buffer);
        return;
    }
    printf("GS_ACK %s\n", buffer);
    last_radio_activity = time_us_32();
    // Small delay to give radio time to settle
    sleep_ms(5);
    // Wait for response up to 200ms
    uint32_t start_time = time_us_32();
    RadioResponsePacket resp = {};
    bool got_resp = false;

    while (time_us_32() - start_time < 200000) {
        if (radio.dataAvailable()) {
            if (radio.readResponse(&resp)) {
                got_resp = true;
                last_radio_activity = time_us_32();
                break;
            }
        }
    }

    if (!got_resp) {
        printf("GS_TIMEOUT %s\n", buffer);
        return;
    }

    // Translate response back to USB text
    int16_t* i16_p = (int16_t*)resp.payload;
    int32_t* i32_p = (int32_t*)resp.payload;
    uint16_t* u16_p = (uint16_t*)resp.payload;

    switch (resp.cmd_id) {
        case 0x02: // GET_PID_ACRO_RP
            printf("PID_ACRO_RP,%f,%f,%f\n", i16_p[0]/1000.0f, i16_p[1]/1000.0f, i16_p[2]/1000.0f);
            break;
        case 0x03: // SET_PID_ACRO_RP
            printf("ACK PID_ACRO_RP\n");
            break;
        case 0x04: // GET_PID_ANGLE_RP
            printf("PID_ANGLE_RP,%f,%f,%f\n", i16_p[0]/1000.0f, i16_p[1]/1000.0f, i16_p[2]/1000.0f);
            break;
        case 0x05: // SET_PID_ANGLE_RP
            printf("ACK PID_ANGLE_RP\n");
            break;
        case 0x06: // GET_PID_YAW
            printf("PID_YAW,%f,%f,%f\n", i16_p[0]/1000.0f, i16_p[1]/1000.0f, i16_p[2]/1000.0f);
            break;
        case 0x07: // SET_PID_YAW
            printf("ACK PID_YAW\n");
            break;
        case 0x08: // GET_EKF
            printf("EKF,%f,%f,%f\n", i32_p[0]/1e6f, i32_p[1]/1e6f, i32_p[2]/1e6f);
            break;
        case 0x09: // SET_EKF
            printf("ACK EKF\n");
            break;
        case 0x0A: // GET_BIAS
            printf("BIAS,%f,%f,%f\n", i16_p[0]/1000.0f, i16_p[1]/1000.0f, i16_p[2]/1000.0f);
            break;
        case 0x0B: // SET_BIAS
            printf("ACK BIAS\n");
            break;
        case 0x0C: // GET_RC_TUNE
            printf("RC_TUNE,%f,%f,%f,%f,%f,%f,%d,%d,%d\n",
                   unpack_float(resp.payload), unpack_float(resp.payload+4), unpack_float(resp.payload+8),
                   unpack_float(resp.payload+12), unpack_float(resp.payload+16), unpack_float(resp.payload+20),
                   resp.payload[24], resp.payload[25], resp.payload[26]);
            break;
        case 0x0D: // SET_RC_TUNE
            printf("ACK RC_TUNE\n");
            break;
        case 0x0E: // GET_RC
            printf("RC,%d,%d,%d,%d,%d,%d,%d,%d\n",
                   i16_p[0], i16_p[1], i16_p[2], i16_p[3], i16_p[4], i16_p[5], i16_p[6], i16_p[7]);
            break;
        case 0x0F: // GET_STATUS
            printf("STATUS,%d,%d,%u,0\n", resp.payload[0], resp.payload[1], u16_p[1]);
            break;
        case 0x10: // GET_IMU
            printf("IMU,%f,%f,%f,%f,%f,%f\n",
                   i16_p[0]/100.0f, i16_p[1]/100.0f, i16_p[2]/100.0f,
                   i16_p[3]/100.0f, i16_p[4]/100.0f, i16_p[5]/100.0f);
            break;
        case 0x11: // SET_MODE
            printf("ACK MODE\n");
            break;
        case 0x12: // GET_MOTORS
            printf("MOTORS,%u,%u,%u,%u\n", u16_p[0], u16_p[1], u16_p[2], u16_p[3]);
            break;
        case 0x14: // GET_FF
            printf("FF,%f,%f,%f\n", i16_p[0]/1000.0f, i16_p[1]/1000.0f, i16_p[2]/1000.0f);
            break;
        case 0x15: // SET_FF
            printf("ACK FF\n");
            break;
        case 0x16: // GET_TPA
            printf("TPA,%f,%f\n", i16_p[0]/1000.0f, i16_p[1]/1000.0f);
            break;
        case 0x17: // SET_TPA
            printf("ACK TPA\n");
            break;
        case 0x18: // GET_I_LIMIT
            printf("I_LIMIT,%f\n", i16_p[0]/100.0f);
            break;
        case 0x19: // SET_I_LIMIT
            printf("ACK I_LIMIT\n");
            break;
        case 0x1A: // GET_D_CUTOFF
            printf("D_CUTOFF,%f\n", i16_p[0]/100.0f);
            break;
        case 0x1B: // SET_D_CUTOFF
            printf("ACK D_CUTOFF\n");
            break;
        case 0x1C: // CALIBRATE_ACCEL
            printf("ACK CALIBRATE_ACCEL\n");
            break;
        case 0x1D: // CALIBRATE_NOISE
            printf("ACK CALIBRATE_NOISE\n");
            break;
        case 0x1E: // REBOOT
            printf("ACK REBOOT\n");
            break;
    }
}

// Non-blocking serial listener
void check_serial_commands() {
    static char rx_buffer[128];
    static int rx_index = 0;
    int c;

    while ((c = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
        if (c == '\n' || c == '\r') {
            if (rx_index > 0) {
                rx_buffer[rx_index] = '\0';
                process_command_and_bridge(rx_buffer);
                rx_index = 0;
            }
        } else if (rx_index < (int)sizeof(rx_buffer) - 1) {
            rx_buffer[rx_index++] = (char)c;
        }
    }
}

// ============================================================================
// MAIN GROUND STATION LOOP
// ============================================================================

int main() {
    stdio_init_all();

    // Give the PC time to recognize the USB COM port
    sleep_ms(3000);

    // 1. Verify SPI Hardware Connection
    while (!radio.checkConnection()) {
        printf("ERROR: NRF24 not responding! Check SPI wiring.\n");
        sleep_ms(2000);
    }

    radio.init();

    // 2. Setup Addresses (mirrored from Drone)
    const uint8_t drone_addr[5]  = {'D', 'R', 'O', 'N', '1'};
    const uint8_t ground_addr[5] = {'G', 'R', 'N', 'D', '1'};

    // GS transmits TO Drone, receives AS Ground
    radio.setAddresses(drone_addr, ground_addr);
    radio.startListening();

    last_radio_activity = time_us_32();

    while (true) {
        // Process any incoming serial commands and bridge to radio
        check_serial_commands();

        // Idle radio recovery: restart after 5 seconds with no activity
        if (time_us_32() - last_radio_activity > 5000000) {
            radio.restart();
            last_radio_activity = time_us_32();
        }
    }

    return 0;
}
