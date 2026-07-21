#pragma once

class ReadPWM;
class WritePWM;
class BMI160;

// Calibrate Receiver (Only Once)
void calibrate_receiver(ReadPWM& receiver, float& rc_roll_bias, float& rc_pitch_bias, float& rc_yaw_bias);

// --- WELFORD'S ALGORITHM CALIBRATION ---
void run_noise_calibration(BMI160& imu, WritePWM& motor);

// Calibrate Accelerometer (Only once)
void calibrate_accel(BMI160& imu, float& accel_bias_roll, float& accel_bias_pitch);
// Calibrate Gyrometer (Before every flight)
void calibrate_gyro(BMI160& imu, float& gyro_bias_x, float& gyro_bias_y, float& gyro_bias_z);

// Calibrate Magnetometer (Only Once)
