#pragma once

class ReadPWM;
class WritePWM;
class IMU;
class HMC5883L;
class BMI160;

// Calibrate Receiver (Only Once)
void calibrate_receiver(ReadPWM& receiver, float& rc_roll_bias, float& rc_pitch_bias, float& rc_yaw_bias);

// --- WELFORD'S ALGORITHM CALIBRATION ---
void run_noise_calibration(IMU& mpu6050,HMC5883L& hmc5883l, WritePWM& motor);

// Calibrate Accelerometer (Only once)

// Calibrate Gyrometer (Before every flight)
void calibrate_gyro(BMI160& imu, float& gyro_bias_x, float& gyro_bias_y, float& gyro_bias_z);

// Calibrate Magnetometer (Only Once)
