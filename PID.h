#pragma once
#include "pico/stdlib.h"
#include <math.h>
#include <stdio.h>

class PIDController {
private:
    float kp, ki, kd;

    // State variables
    float integral_error = 0.0f;
    float previous_measurement = 0.0f; // Track measurement instead of error for D-term
    float previous_derivative = 0.0f;  // Track previous D-term for the PT1 filter

    // Constraints & Filtering
    float integral_limit = 400.0f;     // Anti-windup limit
    float d_cutoff_hz;                 // PT1 Filter cutoff frequency

public:
    PIDController(float p, float i, float d, float cutoff_hz = 25.0f)
    : kp(p), ki(i), kd(d), d_cutoff_hz(cutoff_hz) {}

    float compute(float setpoint, float measured_value, float dt) {
        // Safety check to prevent divide-by-zero crashes
        if (dt <= 0.0f) return 0.0f;

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

        // 4. Derivative Term (Avoiding "Derivative Kick")
        // We negate it so that it acts against the direction of movement
        float raw_derivative = -(measured_value - previous_measurement) / dt;

        // --- PT1 LOW PASS FILTER MATH ---
        // Calculate the RC time constant: RC = 1 / (2 * PI * cutoff_frequency)
        float rc = 1.0f / (2.0f * 3.14159265f * d_cutoff_hz);

        // Calculate the dynamic alpha based on the exact time delta of this loop
        float alpha = dt / (rc + dt);

        // Apply the exponential moving average (PT1)
        float filtered_derivative = previous_derivative + alpha * (raw_derivative - previous_derivative);

        float D = kd * filtered_derivative;

        // 5. Update states for the next loop
        previous_measurement = measured_value;
        previous_derivative = filtered_derivative;

        // 6. Total Output
        return P + I + D;
    }

    void set_pid(float p, float i, float d) {
        kp = p;
        ki = i;
        kd = d;
        printf("PID updated: P=%.3f, I=%.3f, D=%.3f\n", kp, ki, kd);
    }

    // Call this immediately upon arming the motors
    void reset() {
        integral_error = 0.0f;
        previous_measurement = 0.0f;
        previous_derivative = 0.0f;
    }
};
