#pragma once
#include <stdint.h>
#include "EKF.h"
#include "readIMU.h"
#include "nrf24_radio.h"
#include "buzzer.h"
#include "PID.h"
#include "blackbox.h"

extern float bias_roll;
extern float bias_pitch;
extern float bias_yaw;

extern float q_gyro;
extern float q_bias;
extern float r_accel;

extern float pid_p_roll_pitch;
extern float pid_i_roll_pitch;
extern float pid_d_roll_pitch;
extern float pid_p_yaw;
extern float pid_i_yaw;
extern float pid_d_yaw;

extern BlackboxPacket bb_packet;

extern volatile bool tuning_updates[4]; // 0: PID_RP, 1: PID_YAW, 2: BIAS, 3: EKF

extern volatile bool send_telemetry;
extern volatile float shared_roll;
extern volatile float shared_pitch;
extern volatile float shared_yaw;
extern volatile float shared_rc_roll;
extern volatile float shared_rc_pitch;
extern volatile float shared_rc_yaw;
extern volatile float shared_pid_roll;
extern volatile float shared_pid_pitch;
extern volatile float shared_pid_yaw;
extern volatile float shared_dt_us;

extern float receiver_pwm[4];

extern float roll;
extern float pitch;
extern float yaw;
extern volatile bool imu_data_ready;
extern bool was_armed;

extern uint64_t last_update_ekf_us;
extern uint64_t last_update_pid_us;
extern uint32_t last_update_telemetry_us;

extern const uint8_t drone_tx_addr[5];
extern const uint8_t drone_rx_addr[5];

extern Blackbox blackbox;
extern BMI160 imu;
extern NRF24 radio;
extern Buzzer fc_buzzer;

extern PIDController roll_pid;
extern PIDController pitch_pid;
extern PIDController yaw_pid;

extern float roll_control_output;
extern float pitch_control_output;
extern float yaw_control_output;
