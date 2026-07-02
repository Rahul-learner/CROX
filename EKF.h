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

    QuaternionEKF() {
        // 1. Initialize State
        for (int i = 0; i < 7; i++) x[i] = 0.0f;
        x[0] = 1.0f; // CRITICAL: Quaternions must initialize to 1,0,0,0 (perfectly level)

        // 2. Initialize Covariance
        for (int i = 0; i < 7; i++) {
            for (int j = 0; j < 7; j++) {
                P[i][j] = (i == j) ? 0.1f : 0.0f;
            }
        }
    }

    // --- PREDICT STEP (Gyro Integration) ---
    void predict(float gx, float gy, float gz, float dt, float q_quat, float q_bias) {
        // Remove tracked bias from raw gyro (must be in Rad/s)
        float wx = gx - x[4];
        float wy = gy - x[5];
        // float wz = gz - x[6]; // Z-axis is unobservable without a magnetometer
        float wz = gz;

        float q0 = x[0], q1 = x[1], q2 = x[2], q3 = x[3];

        // 1. Integrate Quaternions (Quaternion Derivative)
        x[0] += 0.5f * (-q1 * wx - q2 * wy - q3 * wz) * dt;
        x[1] += 0.5f * ( q0 * wx - q3 * wy + q2 * wz) * dt;
        x[2] += 0.5f * ( q3 * wx + q0 * wy - q1 * wz) * dt;
        x[3] += 0.5f * (-q2 * wx + q1 * wy + q0 * wz) * dt;

        // Normalize Quaternion to prevent math drift
        float invNorm = invSqrt(x[0]*x[0] + x[1]*x[1] + x[2]*x[2] + x[3]*x[3]);
        x[0] *= invNorm; x[1] *= invNorm; x[2] *= invNorm; x[3] *= invNorm;

        // 2. State Transition Jacobian (F Matrix)
        float F[7][7] = {0.0f};
        for (int i = 0; i < 7; i++) F[i][i] = 1.0f;

        F[0][1] = -0.5f*wx*dt; F[0][2] = -0.5f*wy*dt; F[0][3] = -0.5f*wz*dt;
        F[1][0] =  0.5f*wx*dt; F[1][2] =  0.5f*wz*dt; F[1][3] = -0.5f*wy*dt;
        F[2][0] =  0.5f*wy*dt; F[2][1] = -0.5f*wz*dt; F[2][3] =  0.5f*wx*dt;
        F[3][0] =  0.5f*wz*dt; F[3][1] =  0.5f*wy*dt; F[3][2] = -0.5f*wx*dt;

        F[0][4] =  0.5f*q1*dt; F[0][5] =  0.5f*q2*dt; F[0][6] =  0.5f*q3*dt;
        F[1][4] = -0.5f*q0*dt; F[1][5] =  0.5f*q3*dt; F[1][6] = -0.5f*q2*dt;
        F[2][4] = -0.5f*q3*dt; F[2][5] = -0.5f*q0*dt; F[2][6] =  0.5f*q1*dt;
        F[3][4] =  0.5f*q2*dt; F[3][5] = -0.5f*q1*dt; F[3][6] = -0.5f*q0*dt;

        // 3. Update Covariance Matrix: P = F*P*F^T + Q
        float FP[7][7] = {0.0f};
        for (int i = 0; i < 7; i++) {
            for (int j = 0; j < 7; j++) {
                for (int k = 0; k < 7; k++) FP[i][j] += F[i][k] * P[k][j];
            }
        }

        // Q Matrix: Set z-bias noise to 0.0 so the EKF stops trying to track it
        // float Q[7] = {q_quat, q_quat, q_quat, q_quat, q_bias, q_bias, q_bias};
        float Q[7] = {q_quat, q_quat, q_quat, q_quat, q_bias, q_bias, 0.0f};

        for (int i = 0; i < 7; i++) {
            for (int j = 0; j < 7; j++) {
                float sum = 0.0f;
                for (int k = 0; k < 7; k++) sum += FP[i][k] * F[j][k]; // F^T handled via index swap
                P[i][j] = sum;
            }
            P[i][i] += Q[i] * dt;
        }
    }

    // --- UPDATE STEP (Accelerometer Correction) ---
    void update(float ax, float ay, float az, float r_accel) {
        float sum_sq = ax*ax + ay*ay + az*az;
        if(sum_sq == 0.0f) return;

        // Normalize Accelerometer
        float invNorm = invSqrt(sum_sq);
        ax *= invNorm; ay *= invNorm; az *= invNorm;

        float q0 = x[0], q1 = x[1], q2 = x[2], q3 = x[3];

        // 1. Predicted Gravity Vector from Quaternions
        float pred_ax = 2.0f * (q1*q3 - q0*q2);
        float pred_ay = 2.0f * (q0*q1 + q2*q3);
        float pred_az = q0*q0 - q1*q1 - q2*q2 + q3*q3;

        // 2. Innovation (Error between prediction and measurement)
        float y[3] = {ax - pred_ax, ay - pred_ay, az - pred_az};

        // 3. Observation Jacobian (H Matrix)
        float H[3][7] = {0.0f};
        H[0][0] = -2.0f*q2; H[0][1] =  2.0f*q3; H[0][2] = -2.0f*q0; H[0][3] =  2.0f*q1;
        H[1][0] =  2.0f*q1; H[1][1] =  2.0f*q0; H[1][2] =  2.0f*q3; H[1][3] =  2.0f*q2;
        H[2][0] =  2.0f*q0; H[2][1] = -2.0f*q1; H[2][2] = -2.0f*q2; H[2][3] =  2.0f*q3;

        // 4. Innovation Covariance (S = H*P*H^T + R)
        float HP[3][7] = {0.0f};
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 7; j++) {
                for (int k = 0; k < 7; k++) HP[i][j] += H[i][k] * P[k][j];
            }
        }

        float S[3][3] = {0.0f};
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                for (int k = 0; k < 7; k++) S[i][j] += HP[i][k] * H[j][k];
            }
            S[i][i] += r_accel;
        }

        // 5. Invert S Matrix (Cramer's Rule for 3x3)
        float det = S[0][0] * (S[1][1] * S[2][2] - S[2][1] * S[1][2]) -
        S[0][1] * (S[1][0] * S[2][2] - S[1][2] * S[2][0]) +
        S[0][2] * (S[1][0] * S[2][1] - S[1][1] * S[2][0]);

        if (std::abs(det) < 1e-6f) return;
        float invdet = 1.0f / det;

        float S_inv[3][3];
        S_inv[0][0] = (S[1][1] * S[2][2] - S[2][1] * S[1][2]) * invdet;
        S_inv[0][1] = (S[0][2] * S[2][1] - S[0][1] * S[2][2]) * invdet;
        S_inv[0][2] = (S[0][1] * S[1][2] - S[0][2] * S[1][1]) * invdet;
        S_inv[1][0] = (S[1][2] * S[2][0] - S[1][0] * S[2][2]) * invdet;
        S_inv[1][1] = (S[0][0] * S[2][2] - S[0][2] * S[2][0]) * invdet;
        S_inv[1][2] = (S[1][0] * S[0][2] - S[0][0] * S[1][2]) * invdet;
        S_inv[2][0] = (S[1][0] * S[2][1] - S[2][0] * S[1][1]) * invdet;
        S_inv[2][1] = (S[2][0] * S[0][1] - S[0][0] * S[2][1]) * invdet;
        S_inv[2][2] = (S[0][0] * S[1][1] - S[1][0] * S[0][1]) * invdet;

        // 6. Kalman Gain (K = P*H^T*S_inv)
        float PHT[7][3] = {0.0f};
        for (int i = 0; i < 7; i++) {
            for (int j = 0; j < 3; j++) {
                for (int k = 0; k < 7; k++) PHT[i][j] += P[i][k] * H[j][k];
            }
        }

        float K[7][3] = {0.0f};
        for (int i = 0; i < 7; i++) {
            for (int j = 0; j < 3; j++) {
                for (int k = 0; k < 3; k++) K[i][j] += PHT[i][k] * S_inv[k][j];
            }
        }

        // 7. Update State (x = x + K*y)
        for (int i = 0; i < 7; i++) {
            for (int j = 0; j < 3; j++) {
                x[i] += K[i][j] * y[j];
            }
        }

        // Normalize Quaternion again after update
        invNorm = invSqrt(x[0]*x[0] + x[1]*x[1] + x[2]*x[2] + x[3]*x[3]);
        x[0] *= invNorm; x[1] *= invNorm; x[2] *= invNorm; x[3] *= invNorm;

        // 8. Update Covariance (P = (I - K*H)*P)
        for (int i = 0; i < 7; i++) {
            for (int j = 0; j < 7; j++) {
                float K_HP = 0.0f;
                for (int k = 0; k < 3; k++) K_HP += K[i][k] * HP[k][j];
                P[i][j] -= K_HP;
            }
        }
    }

    // Reset QuaternionEKF
    void reset() {
        for (int i=0; i<7; i++) {
            x[i] = 0.0f;
        }
        x[0] = 1.0f;
    }

    // --- GET EULER ANGLES FOR PID CONTROLLER (In Degrees) ---
    // Converts the current Quaternion state back into Roll, Pitch, Yaw
    void getEulerAngles(float &roll_deg, float &pitch_deg, float &yaw_deg) {
        float q0 = x[0], q1 = x[1], q2 = x[2], q3 = x[3];

        roll_deg  = std::atan2(2.0f * (q0*q1 + q2*q3), 1.0f - 2.0f * (q1*q1 + q2*q2)) / DEG_TO_RAD;
        float sinp = 2.0f * (q0 * q2 - q3 * q1);
        if (fabs(sinp) >= 1)
            pitch_deg = copysignf(M_PI/2.0f, sinp)*(180.0f/M_PI); // Use 90 degrees if out of range
        else
            pitch_deg = asinf(sinp) * (180.0f / M_PI);
        yaw_deg   = std::atan2(2.0f * (q0*q3 + q1*q2), 1.0f - 2.0f * (q2*q2 + q3*q3)) / DEG_TO_RAD;
    }
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
    MahonyFilter(float kp = 10.0f, float ki = 0.0f) {
        q0 = 1.0f;
        q1 = 0.0f;
        q2 = 0.0f;
        q3 = 0.0f;

        eInt[0] = 0.0f;
        eInt[1] = 0.0f;
        eInt[2] = 0.0f;

        Kp = kp;
        Ki = ki;
    }

    /**
     * @brief Fuses accel and gyro data into a quaternion
     * @param ax, ay, az Accelerometer data (must be normalized to a vector of length 1.0)
     * @param gx, gy, gz Gyroscope data (must be in RADIANS/second)
     * @param dt Delta time in seconds since the last loop
     */
    void update(float ax, float ay, float az, float gx, float gy, float gz, float dt) {
        // Normalise accelerometer measurement
        float recipNorm = 1.0f / sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Estimated direction of gravity based on the current quaternion
        float vx = 2.0f * (q1 * q3 - q0 * q2);
        float vy = 2.0f * (q0 * q1 + q2 * q3);
        float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

        // Error is cross product between estimated direction and measured direction of gravity
        float ex = (ay * vz - az * vy);
        float ey = (az * vx - ax * vz);
        float ez = (ax * vy - ay * vx);

        // Compute and apply integral feedback if enabled
        if (Ki > 0.0f) {
            eInt[0] += ex * dt;
            eInt[1] += ey * dt;
            eInt[2] += ez * dt;

            // Apply integral feedback
            gx += Ki * eInt[0];
            gy += Ki * eInt[1];
            gz += Ki * eInt[2];
        }

        // Apply proportional feedback
        gx += Kp * ex;
        gy += Kp * ey;
        gz += Kp * ez;

        // Integrate rate of change of quaternion
        // Rate of change of quaternion from gyroscope
        float pa = q1;
        float pb = q2;
        float pc = q3;

        q0 += (-q1 * gx - q2 * gy - q3 * gz) * (0.5f * dt);
        q1 += (q0 * gx + pb * gz - pc * gy) * (0.5f * dt);
        q2 += (q0 * gy - pa * gz + pc * gx) * (0.5f * dt);
        q3 += (q0 * gz + pa * gy - pb * gx) * (0.5f * dt);

        // Normalise quaternion
        recipNorm = 1.0f / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= recipNorm;
        q1 *= recipNorm;
        q2 *= recipNorm;
        q3 *= recipNorm;
    }

    /* Extraction functions for Euler Angles */

    // Returns Roll in degrees
    float getRoll() {
        return atan2(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 180.0f / M_PI;
    }

    // Returns Pitch in degrees
    float getPitch() {
        return asin(2.0f * (q0 * q2 - q3 * q1)) * 180.0f / M_PI;
    }
};
