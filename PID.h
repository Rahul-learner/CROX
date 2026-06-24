#pragma once
#include "pico/stdlib.h"

class PIDController {
private:
    float kp, ki, kd;
    float integral_error = 0.0f;
    float previous_error = 0.0f;
    float integral_limit = 400.0f; // Anti-windup limit

public:
    PIDController(float p, float i, float d) : kp(p), ki(i), kd(d) {}

    float compute(float setpoint, float measured_value, float dt) {
        // 1. Calculate Error
        float error = setpoint - measured_value;

        // 2. Proportional Term
        float P = kp * error;

        // 3. Integral Term (with Anti-Windup)
        integral_error += error * dt;

        // Clamp the integral to prevent windup
        if (integral_error > integral_limit) integral_error = integral_limit;
        if (integral_error < -integral_limit) integral_error = -integral_limit;

        float I = ki * integral_error;

        // 4. Derivative Term
        // Note: In advanced controllers, the derivative is calculated
        // directly from the gyro rate to avoid "derivative kick" when the setpoint changes.
        float derivative = (error - previous_error) / dt;
        float D = kd * derivative;

        previous_error = error;

        // 5. Total Output
        return P + I + D;
    }

    void set_pid(float p, float i, float d) {
        kp = p; ki = i; kd = d;
        printf("Pid changed to P: %f, I: %f, D: %f\n", p, i, d);
    }

    void reset() {
        integral_error = 0.0f;
        previous_error = 0.0f;
    }
};
