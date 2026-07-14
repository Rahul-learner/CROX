#pragma once
#include <math.h>


class QuaternionEKF {
public:
    const float DEG_TO_RAD = 0.01745329251f;
    // --- STATE VECTOR (7 Elements) ---
    // x[0] to x[3] = Quaternions (q0, q1, q2, q3)
    // x[4] to x[6] = Gyro Biases (bx, by, bz)
    float x[7];

    // --- COVARIANCE MATRIX (7x7) ---
    float P[7][7];

    // Fast inverse square-root
    // See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
    inline float invSqrt(float x)
    {
        // return 1.f / sqrt(x);
        #pragma GCC diagnostic push
        #pragma GCC diagnostic ignored "-Wstrict-aliasing"
        #pragma GCC diagnostic ignored "-Wuninitialized"
        float halfx = 0.5f * x;
        float y = x;
        long i = *(long*)&y;
        i = 0x5f3759df - (i >> 1);
        y = *(float*)&i;
        y = y * (1.5f - (halfx * y * y));
        y = y * (1.5f - (halfx * y * y));
        #pragma GCC diagnostic pop
        return y;
    }

    QuaternionEKF();

    // --- PREDICT STEP (Gyro Integration) ---
    void predict(float gx, float gy, float gz, float dt, float q_quat, float q_bias);

    // --- UPDATE STEP (Accelerometer Correction) ---
    void update(float ax, float ay, float az, float r_accel);

    // Reset QuaternionEKF
    void reset();

    // --- GET EULER ANGLES FOR PID CONTROLLER (In Degrees) ---
    // Converts the current Quaternion state back into Roll, Pitch, Yaw
    void getEulerAngles(float &roll_deg, float &pitch_deg, float &yaw_deg);
};

//=============================================================================================================
class MahonyFilter {
private:
    /* Quaternion representing 3D orientation */
    float q0, q1, q2, q3;

    /* Integral error terms scaled by Ki */
    float eInt[3];

    /* Tuning parameters */
    float Kp; // Proportional gain: determines how quickly it trusts the accelerometer
    float Ki; // Integral gain: determines how quickly it corrects gyroscope bias

public:
    MahonyFilter(float kp = 10.0f, float ki = 0.0f);

    /**
     * @brief Fuses accel and gyro data into a quaternion
     * @param ax, ay, az Accelerometer data (must be normalized to a vector of length 1.0)
     * @param gx, gy, gz Gyroscope data (must be in RADIANS/second)
     * @param dt Delta time in seconds since the last loop
     */
    void update(float ax, float ay, float az, float gx, float gy, float gz, float dt);

    /* Extraction functions for Euler Angles */

    // Returns Roll in degrees
    float getRoll();

    // Returns Pitch in degrees
    float getPitch();
};
