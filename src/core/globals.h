#pragma once
#include "config.h"
#include <stdint.h>
#include "EKF.h"
#include "readIMU.h"
#if USE_NRF24_RADIO
#include "nrf24_telemetry.h"
#endif
#include "buzzer.h"
#include "PID.h"
#include "blackbox.h"
#include "writePWM.h"

extern float bias_roll;
extern float bias_pitch;
extern float bias_yaw;

extern float q_gyro;
extern float q_bias;
extern float r_accel;

extern FlightMode current_flight_mode;

extern float angle_strength;
extern float angle_max_deg;
extern float angle_max_rate;

extern float pid_p_roll_pitch_acro;
extern float pid_i_roll_pitch_acro;
extern float pid_d_roll_pitch_acro;
extern float pid_p_yaw;
extern float pid_i_yaw;
extern float pid_d_yaw;

extern float ff_roll;
extern float ff_pitch;
extern float ff_yaw;

extern float tpa_breakpoint;
extern float tpa_factor;

extern float pid_integral_limit;
extern float pid_d_cutoff_hz;

extern BlackboxPacket bb_packet;

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

extern float setpoint_roll;
extern float setpoint_pitch;
extern float setpoint_yaw;

extern volatile float shared_ax;
extern volatile float shared_ay;
extern volatile float shared_az;
extern volatile float shared_gx;
extern volatile float shared_gy;
extern volatile float shared_gz;

extern volatile bool pids_updated_via_msp;

extern volatile bool request_accel_calibration;
extern volatile bool request_noise_calibration;

extern float receiver_pwm[4];
extern float raw_receiver_pwm[4];

extern float rc_expo;
extern float rc_deadband;
extern float rc_yaw_deadband;
extern float rc_roll_center;
extern float rc_pitch_center;
extern float rc_yaw_center;
extern bool rc_roll_reverse;
extern bool rc_pitch_reverse;
extern bool rc_yaw_reverse;

extern float roll;
extern float pitch;
extern float yaw;
extern volatile bool imu_data_ready;
extern bool is_armed;
extern volatile bool esc_calibrating;

extern uint64_t last_update_ekf_us;
extern uint64_t last_update_pid_us;
extern uint32_t last_update_telemetry_us;
extern volatile uint32_t loop_time_us;

extern const uint8_t drone_tx_addr[5];
extern const uint8_t drone_rx_addr[5];

extern Blackbox blackbox;
extern BMI160 imu;
#if USE_NRF24_RADIO
extern NRF24 radio;
#endif
extern Buzzer fc_buzzer;

extern PIDController roll_pid;
extern PIDController pitch_pid;
extern PIDController yaw_pid;

extern WritePWM* global_motor_ptr;

extern float roll_control_output;
extern float pitch_control_output;
extern float yaw_control_output;
