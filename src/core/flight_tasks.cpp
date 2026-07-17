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

void update_pid_and_motors(WritePWM &motor, float gz_rate, float dt_pid) {
  // get the setpoint
  roll_control_output = roll_pid.compute(receiver_pwm[0], roll, dt_pid);
  pitch_control_output = pitch_pid.compute(receiver_pwm[1], pitch, dt_pid);
  yaw_control_output = yaw_pid.compute(receiver_pwm[3], gz_rate, dt_pid);

  // Motor PWM output
  motor.update_motors_pwm(receiver_pwm[2], roll_control_output,
                          pitch_control_output, yaw_control_output);
}

void update_telemetry_and_blackbox(WritePWM &motor, float gz_rate, float dt_ekf,
                                   float dt_pid, uint64_t end_time_us,
                                   bool &blackbox_updated) {
  if (tuning_updates[0]) {
      BlackboxPacket p;
      memset(&p, 0, sizeof(p));
      p.dt_us = 0xFFFD;
      p.pid_roll = (int16_t)(pid_p_roll_pitch * 1000.0f);
      p.pid_pitch = (int16_t)(pid_i_roll_pitch * 1000.0f);
      p.pid_yaw = (int16_t)(pid_d_roll_pitch * 1000.0f);
      blackbox.write_packet(p);
      tuning_updates[0] = false;
  }
  if (tuning_updates[1]) {
      BlackboxPacket p;
      memset(&p, 0, sizeof(p));
      p.dt_us = 0xFFFC;
      p.pid_roll = (int16_t)(pid_p_yaw * 1000.0f);
      p.pid_pitch = (int16_t)(pid_i_yaw * 1000.0f);
      p.pid_yaw = (int16_t)(pid_d_yaw * 1000.0f);
      blackbox.write_packet(p);
      tuning_updates[1] = false;
  }
  if (tuning_updates[2]) {
      BlackboxPacket p;
      memset(&p, 0, sizeof(p));
      p.dt_us = 0xFFFB;
      p.pid_roll = (int16_t)(bias_roll * 1000.0f);
      p.pid_pitch = (int16_t)(bias_pitch * 1000.0f);
      p.pid_yaw = (int16_t)(bias_yaw * 1000.0f);
      blackbox.write_packet(p);
      tuning_updates[2] = false;
  }
  if (tuning_updates[3]) {
      BlackboxPacket p;
      memset(&p, 0, sizeof(p));
      p.dt_us = 0xFFFA;
      p.pid_roll = (int16_t)(q_gyro * 100000.0f); // storing in unused fields, higher precision for EKF if needed. q_gyro might be small? 
      p.pid_pitch = (int16_t)(q_bias * 100000.0f);
      p.pid_yaw = (int16_t)(r_accel * 1000.0f);
      blackbox.write_packet(p);
      tuning_updates[3] = false;
  }

  // set pid outputs to 0.0 if not armed
  if (!is_armed){
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
}

void handle_disarmed_state(WritePWM &motor, QuaternionEKF &filter,
                           bool &blackbox_updated, bool &blackbox_dumped) {
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
  roll_pid.reset();
  pitch_pid.reset();
  yaw_pid.reset();
  
  static uint64_t last_print_time = 0;
  if (time_us_64() - last_print_time > 200000) {
      DEBUG_PRINT("Not Armed. Throttle: %f\n", receiver_pwm[2]);
      last_print_time = time_us_64();
  }
  
  gpio_put(PICO_DEFAULT_LED_PIN, 0);
}
