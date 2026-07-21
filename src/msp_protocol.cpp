#include "msp_protocol.h"
#include "pico/stdlib.h"
#include <stdio.h>



uint8_t MSP_Protocol::crc8_dvb_s2(uint8_t crc, const uint8_t a) {
    crc ^= a;
    for (size_t i = 0; i < 8; ++i) {
        if (crc & 0x80) crc = (crc << 1) ^ 0xD5;
        else crc = crc << 1;
    }
    return crc;
}

void MSP_Protocol::serialize8(uint8_t a) {
    putchar(a);
    if (is_mspv2) checksum2 = crc8_dvb_s2(checksum2, a);
    else checksum ^= a;
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

int16_t MSP_Protocol::read16(uint8_t* payload, uint8_t& idx) {
    int16_t res = payload[idx++];
    res |= (payload[idx++] << 8);
    return res;
}

void MSP_Protocol::headSerialResponse(uint8_t err, uint16_t s, uint16_t cmd) {
    putchar('$');
    if (is_mspv2) {
        putchar('X');
        putchar(err ? '!' : '>');
        checksum2 = 0;
        serialize8(0); // flag
        serialize8(cmd & 0xFF);
        serialize8((cmd >> 8) & 0xFF);
        serialize8(s & 0xFF);
        serialize8((s >> 8) & 0xFF);
    } else {
        putchar('M');
        putchar(err ? '!' : '>');
        checksum = 0;
        serialize8(s);
        serialize8(cmd);
    }
}

void MSP_Protocol::tailSerialReply() {
    if (is_mspv2) putchar(checksum2);
    else putchar(checksum);
    fflush(stdout);
}

void MSP_Protocol::evaluateCommand(float roll_deg, float pitch_deg, float yaw_deg, float ax, float ay, float az, float gx, float gy, float gz) {
    switch (cmd_id) {
        case MSP_API_VERSION:
            headSerialResponse(0, 3, MSP_API_VERSION);
            serialize8(0);  // MSP Protocol Version
            serialize8(1);  // API Major
            serialize8(45); // API Minor (4.4 compatible)
            tailSerialReply();
            break;

        case MSP_FC_VARIANT:
            headSerialResponse(0, 4, MSP_FC_VARIANT);
            // "BTFL" (Betaflight) tells Configurator to unlock PID tuning
            putchar('B'); putchar('T'); putchar('F'); putchar('L');
            checksum ^= 'B'; checksum ^= 'T'; checksum ^= 'F'; checksum ^= 'L';
            tailSerialReply();
            break;

        case MSP_FC_VERSION:
            headSerialResponse(0, 3, MSP_FC_VERSION);
            serialize8(4); serialize8(4); serialize8(0);
            tailSerialReply();
            break;

        case MSP_BOARD_INFO:
            headSerialResponse(0, 57, MSP_BOARD_INFO);
            // boardIdentifier (4)
            serialize8('C'); serialize8('R'); serialize8('O'); serialize8('X');
            // hardwareRevision (2)
            serialize16(0);
            // fcType (1)
            serialize8(0);
            // capabilities (1)
            serialize8(0);
            // targetNameLength (1)
            serialize8(4);
            // targetName (4)
            serialize8('C'); serialize8('R'); serialize8('O'); serialize8('X');
            // boardNameLength (1)
            serialize8(0);
            // manufacturerNameLength (1)
            serialize8(0);
            // signature (32)
            for(int i=0; i<32; i++) serialize8(0);
            // mcuTypeId (1)
            serialize8(255);
            // configurationState (1)
            serialize8(2); // configured
            // sampleRateHz (2)
            serialize16(1000);
            // configurationProblems (4)
            serialize32(0);
            // spiDeviceCount (1)
            serialize8(0);
            // i2cDeviceCount (1)
            serialize8(0);
            tailSerialReply();
            break;

        case MSP_BUILD_INFO:
            headSerialResponse(0, 30, MSP_BUILD_INFO);
            for(int i=0; i<11; i++) serialize8('0'); // date
            for(int i=0; i<8; i++) serialize8('0');  // time
            for(int i=0; i<11; i++) serialize8('0'); // git revision
            tailSerialReply();
            break;

        case MSP_UID:
            headSerialResponse(0, 12, MSP_UID);
            serialize32(0x12345678);
            serialize32(0x87654321);
            serialize32(0xABCDEF01);
            tailSerialReply();
            break;

        case MSP_STATUS_EX:
        case MSP_STATUS:
            headSerialResponse(0, 22, cmd_id);
            serialize16(1000); // cycle time
            serialize16(0);    // i2c errors
            serialize16(0);    // sensors
            serialize32(0);    // mode
            serialize8(0);     // pid profile
            serialize16(5);    // cpu load
            if (cmd_id == MSP_STATUS_EX) {
                serialize8(1); // max profile count
                serialize8(0); // current rate profile index
            } else {
                serialize16(0); // gyro cycle time
            }
            serialize8(0);     // flight mode flags count
            serialize8(1);     // arming disabled count
            serialize32(0);    // arming disabled flags
            serialize8(0);     // reboot required
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

        case MSP_RC:
            headSerialResponse(0, 32, MSP_RC);
            for(int i = 0; i < 16; i++) {
                if (i < 4) serialize16(1500); // AETR center
                else serialize16(1000); // Aux low
            }
            tailSerialReply();
            break;

        case MSP_ANALOG:
            headSerialResponse(0, 9, MSP_ANALOG);
            serialize8(0);  // vbat legacy (0.1V)
            serialize16(0); // mah drawn
            serialize16(0); // rssi
            serialize16(0); // amperage (0.01A)
            serialize16(0); // vbat (0.01V)
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

        case MSP_NAME:
            headSerialResponse(0, 4, MSP_NAME);
            serialize8('C'); serialize8('R'); serialize8('O'); serialize8('X');
            tailSerialReply();
            break;

        case MSP_BATTERY_CONFIG:
            headSerialResponse(0, 14, MSP_BATTERY_CONFIG);
            serialize8(34);  // vbatmincellvoltage
            serialize8(42);  // vbatmaxcellvoltage
            serialize8(35);  // vbatwarningcellvoltage
            serialize16(0);  // batteryCapacity
            serialize8(0);   // voltageMeterSource
            serialize8(0);   // currentMeterSource
            serialize16(340); // vbatmincellvoltage
            serialize16(420); // vbatmaxcellvoltage
            serialize16(350); // vbatwarningcellvoltage
            tailSerialReply();
            break;

        case MSP_FEATURE_CONFIG:
            headSerialResponse(0, 4, MSP_FEATURE_CONFIG);
            serialize32(0);
            tailSerialReply();
            break;

        case MSP_MIXER_CONFIG:
            headSerialResponse(0, 2, MSP_MIXER_CONFIG);
            serialize8(3); // QUADX
            serialize8(0); // yaw_motor_direction
            tailSerialReply();
            break;

        case MSP_CF_SERIAL_CONFIG:
            headSerialResponse(0, 7, MSP_CF_SERIAL_CONFIG);
            serialize8(0);  // id (USB VCP)
            serialize16(32); // functionMask (MSP)
            serialize8(0);  // msp_baudrateIndex
            serialize8(0);  // gps_baudrateIndex
            serialize8(0);  // telemetry_baudrateIndex
            serialize8(0);  // blackbox_baudrateIndex
            tailSerialReply();
            break;

        case MSP_SENSOR_ALIGNMENT:
            headSerialResponse(0, 5, MSP_SENSOR_ALIGNMENT);
            serialize8(1); serialize8(1); serialize8(1); serialize8(1); serialize8(0);
            tailSerialReply();
            break;

        case MSP_BOXNAMES: {
            const char* boxnames = "ARM;ANGLE;";
            headSerialResponse(0, strlen(boxnames), MSP_BOXNAMES);
            for(size_t i=0; i<strlen(boxnames); i++) serialize8(boxnames[i]);
            tailSerialReply();
            break;
        }

        case MSP_BOXIDS:
            headSerialResponse(0, 2, MSP_BOXIDS);
            serialize8(0); serialize8(1);
            tailSerialReply();
            break;

        case MSP_PIDNAMES: {
            const char* pidnames = "ROLL;PITCH;YAW;ALT;POS;POSR;NAVR;LEVEL;MAG;VEL;";
            headSerialResponse(0, strlen(pidnames), MSP_PIDNAMES);
            for(size_t i=0; i<strlen(pidnames); i++) serialize8(pidnames[i]);
            tailSerialReply();
            break;
        }

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
                if (byte == 'M') {
                    state = HEADER_M;
                    is_mspv2 = false;
                } else if (byte == 'X') {
                    state = HEADER_X;
                    is_mspv2 = true;
                } else {
                    state = IDLE;
                }
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
            case HEADER_X:
                if (byte == '<') {
                    state = HEADER_FLAG;
                    is_error = false;
                    checksum2 = 0;
                } else if (byte == '!') {
                    state = HEADER_FLAG;
                    is_error = true;
                    checksum2 = 0;
                } else {
                    state = IDLE;
                }
                break;
            // -- MSPv1 --
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
                state = (data_size > 0) ? PAYLOAD : HEADER_CMD;
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
                    evaluateCommand(current_roll, current_pitch, current_yaw, ax, ay, az, gx, gy, gz);
                }
                state = IDLE;
                break;
            // -- MSPv2 --
            case HEADER_FLAG:
                checksum2 = crc8_dvb_s2(checksum2, byte); // flag
                state = HEADER_CMD_V2;
                payload_index = 0; // use payload_index temporarily for reading 16-bit fields
                break;
            case HEADER_CMD_V2:
                if (payload_index == 0) {
                    cmd_id = byte;
                    checksum2 = crc8_dvb_s2(checksum2, byte);
                    payload_index++;
                } else {
                    cmd_id |= (byte << 8);
                    checksum2 = crc8_dvb_s2(checksum2, byte);
                    payload_index = 0;
                    state = HEADER_SIZE_V2;
                }
                break;
            case HEADER_SIZE_V2:
                if (payload_index == 0) {
                    data_size = byte;
                    checksum2 = crc8_dvb_s2(checksum2, byte);
                    payload_index++;
                } else {
                    data_size |= (byte << 8);
                    checksum2 = crc8_dvb_s2(checksum2, byte);
                    payload_index = 0;
                    if (data_size > sizeof(payload_buffer)) state = IDLE;
                    else state = (data_size > 0) ? PAYLOAD_V2 : CHECKSUM_V2;
                }
                break;
            case PAYLOAD_V2:
                payload_buffer[payload_index++] = byte;
                checksum2 = crc8_dvb_s2(checksum2, byte);
                if (payload_index >= data_size) {
                    state = CHECKSUM_V2;
                }
                break;
            case CHECKSUM_V2:
                if (checksum2 == byte) {
                    evaluateCommand(current_roll, current_pitch, current_yaw, ax, ay, az, gx, gy, gz);
                }
                state = IDLE;
                break;
        }
    }
}
