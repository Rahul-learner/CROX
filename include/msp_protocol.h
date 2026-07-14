#ifndef MSP_PROTOCOL_H
#define MSP_PROTOCOL_H

#include "pico/stdlib.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>

// --- ESP-FC / BETAFLIGHT MSP COMMAND IDS ---
#define MSP_API_VERSION 1
#define MSP_FC_VARIANT  2
#define MSP_FC_VERSION  3
#define MSP_BOARD_INFO  4
#define MSP_RAW_IMU     102
#define MSP_ATTITUDE    108
#define MSP_PID         112
#define MSP_SET_PID     202

// External references to your PID variables so we can update them!
extern float pid_p_roll_pitch;
extern float pid_i_roll_pitch;
extern float pid_d_roll_pitch;
extern float pid_p_yaw;
extern float pid_i_yaw;
extern float pid_d_yaw;

// Flag to tell main loop to re-apply PIDs
extern bool pids_updated_via_msp;

class MSP_Protocol {
private:
    enum MspState {
        IDLE,
        HEADER_START,
        HEADER_M,
        HEADER_ARROW,
        HEADER_SIZE,
        HEADER_CMD,
        HEADER_ERR,
        PAYLOAD
    };

    MspState state = IDLE;
    uint8_t data_size = 0;
    uint8_t cmd_id = 0;
    uint8_t checksum = 0;
    uint8_t payload_buffer[128];
    uint8_t payload_index = 0;
    bool is_error = false;

    // Helper to send data back to Betaflight
    void serialize8(uint8_t a);

    void serialize16(int16_t a);

    void serialize32(uint32_t a);

    // Helper to read data incoming from Betaflight
    uint8_t read8(uint8_t* payload, uint8_t& idx);

    uint16_t read16(uint8_t* payload, uint8_t& idx);

    void headSerialResponse(uint8_t err, uint8_t s, uint8_t cmd);

    void tailSerialReply();

    void evaluateCommand(float roll_deg, float pitch_deg, float yaw_deg, float ax, float ay, float az, float gx, float gy, float gz);

public:
    // Call this inside your main `while(true)` loop!
    void update(float current_roll, float current_pitch, float current_yaw, float ax, float ay, float az, float gx, float gy, float gz);
};

#endif // MSP_PROTOCOL_H
