#include "msp_protocol.h"
#include "pico/stdlib.h"
#include <stdio.h>

extern float pid_p_roll_pitch;
extern float pid_i_roll_pitch;
extern float pid_d_roll_pitch;
extern float pid_p_yaw;
extern float pid_i_yaw;
extern float pid_d_yaw;
extern bool pids_updated_via_msp;

void MSP_Protocol::serialize8(uint8_t a) {
    putchar(a);
    checksum ^= a;
}

void MSP_Protocol::serialize16(int16_t a) {
    serialize8(a & 0xFF);
    serialize8((a >> 8) & 0xFF);
}

void MSP_Protocol::serialize32(uint32_t a) {
    serialize8(a & 0xFF);
    serialize8((a >> 8) & 0xFF);
    serialize8((a >> 16) & 0xFF);
    serialize8((a >> 24) & 0xFF);
}

uint8_t MSP_Protocol::read8(uint8_t* payload, uint8_t& idx) {
    return payload[idx++];
}

uint16_t MSP_Protocol::read16(uint8_t* payload, uint8_t& idx) {
    uint16_t res = payload[idx++];
    res |= (payload[idx++] << 8);
    return res;
}

void MSP_Protocol::headSerialResponse(uint8_t err, uint8_t s, uint8_t cmd) {
    putchar('$');
    putchar('M');
    putchar(err ? '!' : '>');
    checksum = 0;
    serialize8(s);
    serialize8(cmd);
}

void MSP_Protocol::tailSerialReply() {
    putchar(checksum);
}

void MSP_Protocol::evaluateCommand(float roll_deg, float pitch_deg, float yaw_deg, float ax, float ay, float az, float gx, float gy, float gz) {
    switch (cmd_id) {
        case MSP_API_VERSION:
            headSerialResponse(0, 3, MSP_API_VERSION);
            serialize8(2);  // MSP Protocol Version
            serialize8(1);  // API Major
            serialize8(40); // API Minor (4.1 compatible)
            tailSerialReply();
            break;

        case MSP_FC_VARIANT:
            headSerialResponse(0, 4, MSP_FC_VARIANT);
            // "BF  " (Betaflight) tells Configurator to unlock PID tuning
            putchar('B'); putchar('F'); putchar(' '); putchar(' ');
            checksum ^= 'B'; checksum ^= 'F'; checksum ^= ' '; checksum ^= ' ';
            tailSerialReply();
            break;

        case MSP_FC_VERSION:
            headSerialResponse(0, 3, MSP_FC_VERSION);
            serialize8(4); serialize8(2); serialize8(0);
            tailSerialReply();
            break;

        case MSP_BOARD_INFO:
            headSerialResponse(0, 9, MSP_BOARD_INFO);
            for(int i=0; i<4; i++) serialize8('P'); // Board ID "PICO"
            serialize8(0); serialize8(0); serialize8(0); serialize8(0); serialize8(0);
        tailSerialReply();
        break;

        case MSP_ATTITUDE:
            // Betaflight expects angles in decidegrees (e.g. 450 = 45.0 degrees)
            // Pitch and Roll are often inverted depending on sensor orientation in BF,
            // you may need to flip the signs here if the 3D model moves backward.
            headSerialResponse(0, 6, MSP_ATTITUDE);
            serialize16((int16_t)(roll_deg * 10.0f));
            serialize16((int16_t)(pitch_deg * 10.0f));
            serialize16((int16_t)(yaw_deg * 10.0f));
            tailSerialReply();
            break;

        case MSP_RAW_IMU:
            // Betaflight expects raw 16-bit integer IMU values (e.g. standard MPU6050 output)
            // We will convert our floating point G's and Rad/s back into rough 16-bit integers
            // so the Betaflight sensor graphs work!
            headSerialResponse(0, 18, MSP_RAW_IMU);
            serialize16((int16_t)(ax * 512.0f)); // Accel X
            serialize16((int16_t)(ay * 512.0f)); // Accel Y
            serialize16((int16_t)(az * 512.0f)); // Accel Z

            // Gyro is roughly scaled so the graphs look reasonable
            serialize16((int16_t)(gx * 14.3f)); // Gyro X
            serialize16((int16_t)(gy * 14.3f)); // Gyro Y
            serialize16((int16_t)(gz * 14.3f)); // Gyro Z

            serialize16(0); // Mag X (Not used yet)
            serialize16(0); // Mag Y
            serialize16(0); // Mag Z
            tailSerialReply();
            break;

        case MSP_PID:
            // Send current PIDs to the Betaflight Configurator
            // Betaflight sends PIDs in an array.
            // Index 0 = Roll, 1 = Pitch, 2 = Yaw
            // Values are usually sent as integers (e.g., a gain of 1.75 might be sent as 175)
            headSerialResponse(0, 3 * 3, MSP_PID); // 3 axes * 3 values (P, I, D) = 9 bytes

            // Roll (P, I, D)
            serialize8((uint8_t)(pid_p_roll_pitch * 100.0f));
            serialize8((uint8_t)(pid_i_roll_pitch * 100.0f));
            serialize8((uint8_t)(pid_d_roll_pitch * 100.0f));

            // Pitch (P, I, D) - We use the same variables for Roll and Pitch currently
            serialize8((uint8_t)(pid_p_roll_pitch * 100.0f));
            serialize8((uint8_t)(pid_i_roll_pitch * 100.0f));
            serialize8((uint8_t)(pid_d_roll_pitch * 100.0f));

            // Yaw (P, I, D)
            serialize8((uint8_t)(pid_p_yaw * 100.0f));
            serialize8((uint8_t)(pid_i_yaw * 100.0f));
            serialize8((uint8_t)(pid_d_yaw * 100.0f));

            tailSerialReply();
            break;

        case MSP_SET_PID:
            // Betaflight is sending us new PIDs!
            {
                uint8_t idx = 0;

                // Read Roll
                float new_r_p = (float)read8(payload_buffer, idx) / 100.0f;
                float new_r_i = (float)read8(payload_buffer, idx) / 100.0f;
                float new_r_d = (float)read8(payload_buffer, idx) / 100.0f;

                // Read Pitch
                float new_p_p = (float)read8(payload_buffer, idx) / 100.0f;
                float new_p_i = (float)read8(payload_buffer, idx) / 100.0f;
                float new_p_d = (float)read8(payload_buffer, idx) / 100.0f;

                // Read Yaw
                float new_y_p = (float)read8(payload_buffer, idx) / 100.0f;
                float new_y_i = (float)read8(payload_buffer, idx) / 100.0f;
                float new_y_d = (float)read8(payload_buffer, idx) / 100.0f;

                // Update our global variables.
                // Since you currently link Roll and Pitch together, we'll just use the Roll values.
                pid_p_roll_pitch = new_r_p;
                pid_i_roll_pitch = new_r_i;
                pid_d_roll_pitch = new_r_d;

                pid_p_yaw = new_y_p;
                pid_i_yaw = new_y_i;
                pid_d_yaw = new_y_d;

                // Flag main loop to apply these to the PID objects
                pids_updated_via_msp = true;

                // Acknowledge the command
                headSerialResponse(0, 0, MSP_SET_PID);
                tailSerialReply();
            }
            break;

        default:
            // Send an empty error response for unsupported commands
            // This keeps Betaflight happy so it doesn't hang waiting for data.
            headSerialResponse(1, 0, cmd_id);
            tailSerialReply();
            break;
    }
}

void MSP_Protocol::update(float current_roll, float current_pitch, float current_yaw, float ax, float ay, float az, float gx, float gy, float gz) {
    int c;
    // Non-blocking read from USB
    while ((c = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
        uint8_t byte = (uint8_t)c;

        switch (state) {
            case IDLE:
                if (byte == '$') state = HEADER_START;
                break;
            case HEADER_START:
                state = (byte == 'M') ? HEADER_M : IDLE;
                break;
            case HEADER_M:
                if (byte == '<') {
                    state = HEADER_ARROW;
                    is_error = false;
                } else if (byte == '!') {
                    state = HEADER_ARROW;
                    is_error = true;
                } else {
                    state = IDLE;
                }
                break;
            case HEADER_ARROW:
                if (byte > sizeof(payload_buffer)) {
                    state = IDLE; // Payload too large, abort
                } else {
                    data_size = byte;
                    checksum = byte;
                    state = HEADER_SIZE;
                }
                break;
            case HEADER_SIZE:
                cmd_id = byte;
                checksum ^= byte;
                payload_index = 0;
                state = (data_size > 0) ? PAYLOAD : HEADER_CMD; // Skip payload if size is 0
                break;
            case PAYLOAD:
                payload_buffer[payload_index++] = byte;
                checksum ^= byte;
                if (payload_index >= data_size) {
                    state = HEADER_CMD;
                }
                break;
            case HEADER_CMD:
                if (checksum == byte) {
                    // Checksum passed! Execute the command.
                    evaluateCommand(current_roll, current_pitch, current_yaw, ax, ay, az, gx, gy, gz);
                }
                state = IDLE;
                break;
        }
    }
}
