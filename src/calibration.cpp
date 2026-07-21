#include "calibration.h"
#include "readPWM.h"
#include "writePWM.h"
#include "pico/stdlib.h"
#include <stdio.h>
#include "readIMU.h"
#include "config.h"
#include "math.h"

extern volatile bool imu_data_ready;

void calibrate_receiver(ReadPWM& receiver, float& rc_roll_bias, float& rc_pitch_bias, float& rc_yaw_bias) {
    float rc_pwm[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float raw_rc_pwm[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float pwm_sum[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float reading_count = 10;
    for (int i = 0; i < reading_count; i++) {
        receiver.read_pwm(rc_pwm, raw_rc_pwm);


        pwm_sum[0] += rc_pwm[0];
        pwm_sum[1] += rc_pwm[1];
        // Index 2 (throttle)
        pwm_sum[3] += rc_pwm[3];

        sleep_ms(51);
    }

    rc_roll_bias = pwm_sum[0] / reading_count;
    rc_pitch_bias = pwm_sum[1] / reading_count;
    rc_yaw_bias = pwm_sum[3] / reading_count;

    DEBUG_PRINT("\n--- RC CALIBRATION COMPLETE! ---\n");
    DEBUG_PRINT("Roll Bias: %f\n", rc_roll_bias);
    DEBUG_PRINT("Pitch Bias: %f\n", rc_pitch_bias);
    DEBUG_PRINT("Yaw Bias: %f\n", rc_yaw_bias);
    DEBUG_PRINT("\n--- CALIBRATED RC PWM ---\n");
    DEBUG_PRINT("Roll: %f\n", rc_pwm[0] - rc_roll_bias);
    DEBUG_PRINT("Pitch: %f\n", rc_pwm[1] - rc_pitch_bias);
    DEBUG_PRINT("Yaw: %f\n", rc_pwm[3] - rc_yaw_bias);
    DEBUG_PRINT("--------------------------------\n\n");
}

void run_noise_calibration(BMI160& imu, WritePWM& motor) { // <-- Added motor back
    DEBUG_PRINT("\n--- STARTING EKF NOISE CALIBRATION ---\n");
    DEBUG_PRINT("WARNING: REMOVE PROPELLERS! Spinning motors to measure vibration...\n");
    
    // Spin the motors at a hover-like throttle to simulate flight vibrations
    motor.update_motors_pwm(1500.0f, 0.0f, 0.0f, 0.0f);
    sleep_ms(2000); // Give the motors 2 seconds to spool up and stabilize

    const int SAMPLES = 10000;

    // Arrays to hold Mean and M2 (Squared distance from mean)
    float a_mean[3] = {0}, a_M2[3] = {0};

    // Discard the first few samples as the sensor settles
    for(int i=0; i<50; i++) {
        while (!imu_data_ready) { tight_loop_contents(); }
        imu_data_ready = false;
    }

    for (int count = 1; count <= SAMPLES; count++) {
        while (!imu_data_ready) { tight_loop_contents(); } // Wait for interrupt
        imu_data_ready = false;

        // 1. READ SENSORS 
        float ax, ay, az, gx, gy, gz;
        imu.readData(&ax, &ay, &az, &gx, &gy, &gz);

        // 2. WELFORD'S MATH FOR ACCELEROMETER
        float a_curr[3] = {ax, ay, az};
        for (int i = 0; i < 3; i++) {
            float delta = a_curr[i] - a_mean[i];
            a_mean[i] += delta / count;
            float delta2 = a_curr[i] - a_mean[i];
            a_M2[i] += delta * delta2;
        }
    }

    // Turn off the motors immediately after collecting samples!
    motor.reset();

    // 4. CALCULATE VARIANCE (This goes straight into your EKF 'R' Matrix)
    // Variance = M2 / (Samples - 1)
    float a_var[3];
    for (int i = 0; i < 3; i++) {
        a_var[i] = a_M2[i] / (SAMPLES - 1);
    }

    // 5. PRINT RESULTS TO TERMINAL
    DEBUG_PRINT("\nCALIBRATION COMPLETE!\n");
    DEBUG_PRINT("Copy these values into your EKF R Matrices:\n");
    DEBUG_PRINT("Accel Variance (R): X: %f, Y: %f, Z: %f\n", a_var[0], a_var[1], a_var[2]);
    DEBUG_PRINT("--------------------------------------\n\n");
}


void calibrate_accel(BMI160& imu, float& accel_bias_roll, float& accel_bias_pitch) {
    DEBUG_PRINT("\n--- STARTING ACCELEROMETER CALIBRATION ---\n");
    DEBUG_PRINT("Keep drone still on a flat level surface...\n");

    const int SAMPLES = 1000;
    float sum_roll = 0.0f;
    float sum_pitch = 0.0f;

    // Discard the first few samples as the sensor settles
    for(int i=0; i<50; i++) {
        while (!imu_data_ready) { tight_loop_contents(); }
        imu_data_ready = false;
    }

    for (int count = 0; count < SAMPLES; count++) {
        while (!imu_data_ready) { tight_loop_contents(); } // Wait for interrupt
        imu_data_ready = false;

        float ax, ay, az, gx, gy, gz;
        imu.readData(&ax, &ay, &az, &gx, &gy, &gz);
        
        // Calculate raw pitch and roll from accelerometer (in degrees)
        float accel_pitch = atan2(ay, sqrt(ax * ax + az * az)) * 180.0f / 3.14159265f;
        float accel_roll = atan2(-ax, az) * 180.0f / 3.14159265f;
        
        sum_roll += accel_roll;
        sum_pitch += accel_pitch;
    }

    accel_bias_roll = sum_roll / SAMPLES;
    accel_bias_pitch = sum_pitch / SAMPLES;

    DEBUG_PRINT("\nACCEL CALIBRATION COMPLETE!\n");
    DEBUG_PRINT("Calculated Bias -> Roll: %f, Pitch: %f\n", accel_bias_roll, accel_bias_pitch);
    DEBUG_PRINT("--------------------------------------\n\n");
}


void calibrate_gyro(BMI160& imu, float& gyro_bias_x, float& gyro_bias_y, float& gyro_bias_z) {
    gyro_bias_x = 0.0f;
    gyro_bias_y = 0.0f;
    gyro_bias_z = 0.0f;

    // Discard the first few samples as the sensor settles
    for(int i=0; i<50; i++) {
        while (!imu_data_ready) { tight_loop_contents(); }
        imu_data_ready = false;
    }

    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        while (!imu_data_ready) { tight_loop_contents(); } // Wait for interrupt
        imu_data_ready = false;

        float ax, ay, az, gx, gy, gz;
        imu.readData(&ax, &ay, &az, &gx, &gy, &gz);

        gyro_bias_x += gx;
        gyro_bias_y += gy;
        gyro_bias_z += gz;
    }

    // Average out the errors
    gyro_bias_x /= CALIBRATION_SAMPLES;
    gyro_bias_y /= CALIBRATION_SAMPLES;
    gyro_bias_z /= CALIBRATION_SAMPLES;
}
