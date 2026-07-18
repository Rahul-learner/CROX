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
    PIDController(float p, float i, float d, float cutoff_hz = 25.0f);

    float compute(float setpoint, float measured_value, float dt);

    void set_pid(float p, float i, float d);

    // Call this immediately upon arming the motors
    void reset();
    
    // Clear the integral error (used to prevent windup on the ground)
    void reset_integral();
};
