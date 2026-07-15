#include "core/flight_tasks.h"
#include "config.h"
#include "core/globals.h"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include <stdio.h>

#ifndef PICO_DEFAULT_LED_PIN
#define PICO_DEFAULT_LED_PIN 25
#endif

#ifndef DEBUG_PRINT
#ifdef DEBUG_MODE
#define DEBUG_PRINT(...) printf(__VA_ARGS__)
#else
#define DEBUG_PRINT(...)
#endif
#endif

void update_sensors_and_ekf(QuaternionEKF &filter, float gyro_bias_x,
                            float gyro_bias_y, float gyro_bias_z, float &dt_ekf,
                            bool &run_accel_update, float &gz_out) {
  uint64_t current_time_ekf_us = time_us_64();
  dt_ekf = (current_time_ekf_us - last_update_ekf_us) / 1000000.0f;
  last_update_ekf_us = current_time_ekf_us;

  // Read the IMU data
  float ax, ay, az, gx, gy, gz;
  imu.readData(&ax, &ay, &az, &gx, &gy, &gz);
  // subtract the Biases
  gx -= gyro_bias_x;
  gy -= gyro_bias_y;
  gz -= gyro_bias_z;

  // Execute the Filter
  filter.predict(gx, gy, gz, dt_ekf, q_gyro, q_bias);
  if (run_accel_update) {
    filter.update(ax, ay, az, r_accel);
  }
  run_accel_update = !run_accel_update;
  filter.getEulerAngles(roll, pitch, yaw);
  // apply bias
  roll -= bias_roll;
  pitch -= bias_pitch;
  yaw -= bias_yaw;
  gz_out = gz * 180.0f / 3.14159265358979323846f;
}

void update_pid_and_motors(WritePWM &motor, float gz_rate, float &dt_pid) {
  uint64_t current_time_pid_us = time_us_64();
  if (((current_time_pid_us - last_update_pid_us) >= 2040) &&
      receiver_pwm[2] > 1050.0f) {
    dt_pid = (current_time_pid_us - last_update_pid_us) / 1000000.0f;
    last_update_pid_us = current_time_pid_us;

    // get the setpoint
    roll_control_output = roll_pid.compute(receiver_pwm[0], roll, dt_pid);
    pitch_control_output = pitch_pid.compute(receiver_pwm[1], pitch, dt_pid);
    yaw_control_output = yaw_pid.compute(receiver_pwm[3], gz_rate, dt_pid);

    // Motor PWM output
    motor.update_motors_pwm(receiver_pwm[2], roll_control_output,
                            pitch_control_output, yaw_control_output);
  }
}

void update_telemetry_and_blackbox(WritePWM &motor, float gz_rate, float dt_ekf,
                                   float dt_pid, uint64_t end_time_us,
                                   bool &blackbox_updated) {
  static uint64_t last_update_print_us = 0;
  if ((end_time_us - last_update_print_us) > 1000000 / 200) {
    // set pid outputs to 0.0 if not armed
    if (!was_armed){
      roll_control_output = 0.0f;
      pitch_control_output = 0.0f;
      yaw_control_output = 0.0f;
    }
    shared_roll = roll;
    shared_pitch = pitch;
    shared_yaw = gz_rate;
    shared_rc_roll = receiver_pwm[0];
    shared_rc_pitch = receiver_pwm[1];
    shared_rc_yaw = receiver_pwm[3];
    shared_pid_roll = roll_control_output;
    shared_pid_pitch = pitch_control_output;
    shared_pid_yaw = yaw_control_output;
    shared_dt_us = dt_ekf;
    send_telemetry = true;

    // update the blackbox packet
    bb_packet.roll = (int16_t)(roll * 100.0f);
    bb_packet.pitch = (int16_t)(pitch * 100.0f);
    bb_packet.yaw_rate = (int16_t)(gz_rate * 100.0f);
    bb_packet.pid_roll = (int16_t)(roll_control_output * 100.0f);
    bb_packet.pid_pitch = (int16_t)(pitch_control_output * 100.0f);
    bb_packet.pid_yaw = (int16_t)(yaw_control_output * 100.0f);
    bb_packet.rc_roll = (int16_t)(receiver_pwm[0] * 100.0f);
    bb_packet.rc_pitch = (int16_t)(receiver_pwm[1] * 100.0f);
    bb_packet.rc_yaw = (int16_t)(receiver_pwm[3] * 100.0f);
    bb_packet.rc_throttle = (uint16_t)(receiver_pwm[2]);
    bb_packet.motor1 = (uint16_t)(motor.motor1_speed);
    bb_packet.motor2 = (uint16_t)(motor.motor2_speed);
    bb_packet.motor3 = (uint16_t)(motor.motor3_speed);
    bb_packet.motor4 = (uint16_t)(motor.motor4_speed);
    bb_packet.dt_us = (uint16_t)((end_time_us / 1000000.0f) * 100.0f);
    blackbox.write_packet(bb_packet);
    blackbox_updated = true;

    DEBUG_PRINT("Roll: %.2f, Pitch: %.2f, Yaw: %.2f, RC_Roll: %f, RC_Pitch: "
                "%f, RC_Yaw: %f, PID_Roll: %f, PID_Pitch: %f, PID_Yaw: %f, "
                "dt_pid: %f, dt_ekf: %f\n",
                roll, pitch, gz_rate, receiver_pwm[0], receiver_pwm[1],
                receiver_pwm[3], roll_control_output, pitch_control_output,
                yaw_control_output, dt_pid, dt_ekf);
    last_update_print_us = end_time_us;
  }
}

void handle_disarmed_state(WritePWM &motor, QuaternionEKF &filter,
                           bool &blackbox_updated, bool &blackbox_dumped,
                           bool &first_throttle_on) {
  motor.reset();
  // write the blackbox to the flash
  if (blackbox_updated) {
    blackbox.write_blackbox_to_flash();
    blackbox_updated = false;
  }
  if (!blackbox_updated && !blackbox_dumped && stdio_usb_connected()) {
    blackbox.dump_flash_to_usb();
    blackbox_dumped = true;
  }
  if (!stdio_usb_connected()) {
    blackbox_dumped = false;
  }
  send_telemetry = false;
  filter.reset();
  roll_pid.reset();
  pitch_pid.reset();
  yaw_pid.reset();
  first_throttle_on = true;
  DEBUG_PRINT("Not Armed. Throttle: %f\n", receiver_pwm[2]);
  gpio_put(PICO_DEFAULT_LED_PIN, 0);
  sleep_ms(200);
}
