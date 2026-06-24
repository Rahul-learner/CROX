#pragma once

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
