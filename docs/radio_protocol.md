# CROX NRF24 Radio Protocol

This document describes the generic command/response protocol used over NRF24 for communicating between the Drone and the Ground Station. 

## Packet Formats

All packets are padded to exactly 32 bytes to match the NRF24 payload size.

### RadioCommandPacket (Ground → Drone)

```c
struct RadioCommandPacket {
    uint8_t header1;     // 0xCC
    uint8_t header2;     // 0xDD
    uint8_t cmd_id;
    uint8_t payload[28];
    uint8_t checksum;
};
```

### RadioResponsePacket (Drone → Ground)

```c
struct RadioResponsePacket {
    uint8_t header1;     // 0xDD
    uint8_t header2;     // 0xCC
    uint8_t cmd_id;
    uint8_t payload[28];
    uint8_t checksum;
};
```

## Command Table

| ID   | Command             | Payload (Cmd)                                  | Payload (Response)                             |
|------|---------------------|-------------------------------------------------|------------------------------------------------|
| 0x01 | GET_ATTITUDE        | (empty)                                         | roll(i16), pitch(i16), yaw(i16)                |
| 0x02 | GET_PID_ACRO_RP     | (empty)                                         | P(i16), I(i16), D(i16) (×1000)                |
| 0x03 | SET_PID_ACRO_RP     | P(i16), I(i16), D(i16) (×1000)                 | ACK (0x01)                                     |
| 0x04 | GET_PID_ANGLE_RP    | (empty)                                         | P(i16), I(i16), D(i16) (×1000)                |
| 0x05 | SET_PID_ANGLE_RP    | P(i16), I(i16), D(i16) (×1000)                 | ACK (0x01)                                     |
| 0x06 | GET_PID_YAW         | (empty)                                         | P(i16), I(i16), D(i16) (×1000)                |
| 0x07 | SET_PID_YAW         | P(i16), I(i16), D(i16) (×1000)                 | ACK (0x01)                                     |
| 0x08 | GET_EKF             | (empty)                                         | q_gyro(i32), q_bias(i32), r_accel(i32) (×1e6) |
| 0x09 | SET_EKF             | q_gyro(i32), q_bias(i32), r_accel(i32) (×1e6)  | ACK (0x01)                                     |
| 0x0A | GET_BIAS            | (empty)                                         | roll(i16), pitch(i16), yaw(i16) (×1000)       |
| 0x0B | SET_BIAS            | roll(i16), pitch(i16), yaw(i16) (×1000)         | ACK (0x01)                                     |
| 0x0C | GET_RC_TUNE         | (empty)                                         | expo, deadband, yaw_db, centers, reverse flags |
| 0x0D | SET_RC_TUNE         | expo, deadband, yaw_db, centers, reverse flags   | ACK (0x01)                                     |
| 0x0E | GET_RC              | (empty)                                         | 4× raw(i16) + 4× cal(i16)                     |
| 0x0F | GET_STATUS          | (empty)                                         | armed(u8), mode(u8), loop_us(u16)              |
| 0x10 | GET_IMU             | (empty)                                         | ax,ay,az,gx,gy,gz (i16 ×100)                  |
| 0x11 | SET_MODE            | mode(u8)                                        | ACK (0x01)                                     |
| 0x12 | GET_MOTORS          | (empty)                                         | m1,m2,m3,m4 (u16)                              |
| 0x14 | GET_FF              | (empty)                                         | roll(i16), pitch(i16), yaw(i16) (×1000)        |
| 0x15 | SET_FF              | roll(i16), pitch(i16), yaw(i16) (×1000)         | ACK (0x01)                                     |
| 0x16 | GET_TPA             | (empty)                                         | breakpoint(i16), factor(i16) (×1000)           |
| 0x17 | SET_TPA             | breakpoint(i16), factor(i16) (×1000)            | ACK (0x01)                                     |
| 0x18 | GET_I_LIMIT         | (empty)                                         | limit(i16 ×100)                                |
| 0x19 | SET_I_LIMIT         | limit(i16 ×100)                                 | ACK (0x01)                                     |
| 0x1A | GET_D_CUTOFF        | (empty)                                         | hz(i16 ×100)                                   |
| 0x1B | SET_D_CUTOFF        | hz(i16 ×100)                                    | ACK (0x01)                                     |
| 0x1C | CALIBRATE_ACCEL     | (empty)                                         | ACK (0x01)                                     |
| 0x1D | CALIBRATE_NOISE     | (empty)                                         | ACK (0x01)                                     |
| 0x1E | REBOOT              | (empty)                                         | ACK (0x01)                                     |
| 0x1F | GET_CONFIG          | (empty)                                         | cpu_khz(u16), pwm_hz(u16), flags(u8)           |
| 0xF0 | START_TELEMETRY     | (empty)                                         | (starts continuous stream)                     |
| 0xF1 | STOP_TELEMETRY      | (empty)                                         | ACK (0x01)                                     |

Note: SET_MOTOR_TEST is not supported via Ground Station for safety.
