#include "calibration.h"
#include "readPWM.h"
#include "writePWM.h"
#include "pico/stdlib.h"
#include <stdio.h>
#include "readIMU.h"
#include "config.h"

extern volatile bool imu_data_ready;

class IMU {
public:
    void mpu6050_read(float& ax, float& ay, float& az, float& gx, float& gy, float& gz) {}
};

class HMC5883L {
public:
    void hmc5883l_read(float& mx, float& my, float& mz) {}
};

void calibrate_receiver(ReadPWM& receiver, float& rc_roll_bias, float& rc_pitch_bias, float& rc_yaw_bias) {
    float rc_pwm[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float pwm_sum[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    for (int i = 0; i < 50; i++) {
        receiver.read_pwm(rc_pwm);


        pwm_sum[0] += rc_pwm[0];
        pwm_sum[1] += rc_pwm[1];
        // Index 2 (throttle)
        pwm_sum[3] += rc_pwm[3];

        sleep_ms(51);
    }

    rc_roll_bias = pwm_sum[0] / 50.0f;
    rc_pitch_bias = pwm_sum[1] / 50.0f;
    rc_yaw_bias = pwm_sum[3] / 50.0f;
}

void run_noise_calibration(IMU& mpu6050,HMC5883L& hmc5883l, WritePWM& motor) {
    printf("\n--- STARTING EKF NOISE CALIBRATION ---\n");
    printf("Keep the drone perfectly still with motors running (NO PROPS!)...\n");
    motor.update_motors_pwm(1800.0f, 0.0f, 0.0f, 0.0f);

    const int SAMPLES = 1000;

    // Arrays to hold Mean and M2 (Squared distance from mean)
    float a_mean[3] = {0}, a_M2[3] = {0};
    float m_mean[3] = {0}, m_M2[3] = {0};

    for (int count = 1; count <= SAMPLES; count++) {
        // 1. READ SENSORS (Replace with your actual read functions)
        float ax, ay, az, gx, gy, gz, mx, my, mz;
        mpu6050.mpu6050_read(ax, ay, az, gx, gy, gz);
        hmc5883l.hmc5883l_read(mx, my, mz);

        // 2. WELFORD'S MATH FOR ACCELEROMETER
        float a_curr[3] = {ax, ay, az};
        for (int i = 0; i < 3; i++) {
            float delta = a_curr[i] - a_mean[i];
            a_mean[i] += delta / count;
            float delta2 = a_curr[i] - a_mean[i];
            a_M2[i] += delta * delta2;
        }

        // 3. WELFORD'S MATH FOR MAGNETOMETER
        float m_curr[3] = {mx, my, mz};
        for (int i = 0; i < 3; i++) {
            float delta = m_curr[i] - m_mean[i];
            m_mean[i] += delta / count;
            float delta2 = m_curr[i] - m_mean[i];
            m_M2[i] += delta * delta2;
        }

        // Wait 10ms to match your 100Hz flight loop
        sleep_ms(10);
    }

    // Turn off motor
    motor.reset();

    // 4. CALCULATE VARIANCE (This goes straight into your EKF 'R' Matrix)
    // Variance = M2 / (Samples - 1)
    float a_var[3], m_var[3];
    for (int i = 0; i < 3; i++) {
        a_var[i] = a_M2[i] / (SAMPLES - 1);
        m_var[i] = m_M2[i] / (SAMPLES - 1);
    }

    // 5. PRINT RESULTS TO TERMINAL
    printf("\nCALIBRATION COMPLETE!\n");
    printf("Copy these values into your EKF R Matrices:\n");
    printf("Accel Variance (R): X: %f, Y: %f, Z: %f\n", a_var[0], a_var[1], a_var[2]);
    printf("Mag Variance (R_mag): X: %f, Y: %f, Z: %f\n", m_var[0], m_var[1], m_var[2]);
    printf("--------------------------------------\n\n");

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
