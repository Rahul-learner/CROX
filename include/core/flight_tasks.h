#pragma once
#include <stdint.h>
#include "EKF.h"
#include "writePWM.h"

void update_sensors_and_ekf(QuaternionEKF& filter, float gyro_bias_x, float gyro_bias_y, float gyro_bias_z, float& dt_ekf, bool& run_accel_update, float& gz_out);
void update_pid_and_motors(WritePWM& motor, float gz_rate, float dt_pid);
void update_telemetry_and_blackbox(WritePWM& motor, float gz_rate, float dt_ekf, float dt_pid, uint64_t end_time_us, bool& blackbox_updated);
void handle_disarmed_state(WritePWM& motor, QuaternionEKF& filter, bool& blackbox_updated, bool& blackbox_dumped);
