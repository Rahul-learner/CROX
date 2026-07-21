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

static float raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz;

void update_sensors_and_ekf(QuaternionEKF &filter, float gyro_bias_x,
                            float gyro_bias_y, float gyro_bias_z, float &dt_ekf,
                            bool &run_accel_update, float &gx_out, float &gy_out, float &gz_out) {
  uint64_t current_time_ekf_us = time_us_64();
  dt_ekf = (current_time_ekf_us - last_update_ekf_us) / 1000000.0f;
  last_update_ekf_us = current_time_ekf_us;

  // Read the IMU data
  float ax, ay, az, gx, gy, gz;
  imu.readData(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Store raw values for Blackbox
  raw_ax = ax; raw_ay = ay; raw_az = az;
  raw_gx = gx; raw_gy = gy; raw_gz = gz;

  // subtract the Biases
  gx -= gyro_bias_x;
  gy -= gyro_bias_y;
  gz -= gyro_bias_z;

  // Execute the Filter
  filter.predict(gx, gy, gz, dt_ekf, q_gyro, q_bias);
  if (run_accel_update) {
    float r_accel_dynamic = r_accel;
    if (receiver_pwm[2] > 1050.0f) { // Only scale if actually flying
        float throttle_pct = (receiver_pwm[2] - 1050.0f) / (2000.0f - 1050.0f);
        if (throttle_pct > 1.0f) throttle_pct = 1.0f;
        if (throttle_pct < 0.0f) throttle_pct = 0.0f;
        r_accel_dynamic = r_accel + throttle_pct * (DYNAMIC_R_MAX - r_accel);
    }
    filter.update(ax, ay, az, r_accel_dynamic);
  }
  run_accel_update = !run_accel_update;
  filter.getEulerAngles(roll, pitch, yaw);
  // apply bias
  roll -= bias_roll;
  pitch -= bias_pitch;
  yaw -= bias_yaw;
  gx_out = gx * 180.0f / 3.14159265358979323846f;
  gy_out = gy * 180.0f / 3.14159265358979323846f;
  gz_out = gz * 180.0f / 3.14159265358979323846f;
}

void update_pid_and_motors(WritePWM &motor, float gx_rate, float gy_rate, float gz_rate, float dt_pid) {
  // Prevent Integral Windup while sitting on the ground
  if (receiver_pwm[2] < 1050.0f) {
      roll_pid.reset_integral();
      pitch_pid.reset_integral();
      yaw_pid.reset_integral();
  }

  // Set limits and cutoffs dynamically
  roll_pid.set_integral_limit(pid_integral_limit);
  pitch_pid.set_integral_limit(pid_integral_limit);
  yaw_pid.set_integral_limit(pid_integral_limit);
  
  roll_pid.set_d_cutoff_hz(pid_d_cutoff_hz);
  pitch_pid.set_d_cutoff_hz(pid_d_cutoff_hz);
  yaw_pid.set_d_cutoff_hz(pid_d_cutoff_hz);

  // Calculate TPA Factor (1.0 = no attenuation, 0.7 = 30% attenuation)
  float tpa_factor_dynamic = 1.0f;
  if (receiver_pwm[2] > tpa_breakpoint) {
      float throttle_pct = (receiver_pwm[2] - tpa_breakpoint) / (2000.0f - tpa_breakpoint);
      if (throttle_pct > 1.0f) throttle_pct = 1.0f;
      tpa_factor_dynamic = 1.0f - (throttle_pct * tpa_factor);
  }

  static float prev_acro_roll_sp = 0.0f;
  static float prev_acro_pitch_sp = 0.0f;
  static float prev_acro_yaw_sp = 0.0f;

  float acro_roll_sp = receiver_pwm[0] * ACRO_RATE_SCALING;
  float acro_pitch_sp = receiver_pwm[1] * ACRO_RATE_SCALING;
  float acro_yaw_sp = receiver_pwm[3] * ACRO_RATE_SCALING;
  
  // Calculate FeedForward (derivative of setpoint)
  float ff_roll_out = 0.0f, ff_pitch_out = 0.0f, ff_yaw_out = 0.0f;
  if (dt_pid > 0.0f) {
      ff_roll_out = ((acro_roll_sp - prev_acro_roll_sp) / dt_pid) * ff_roll * 0.01f;
      ff_pitch_out = ((acro_pitch_sp - prev_acro_pitch_sp) / dt_pid) * ff_pitch * 0.01f;
      ff_yaw_out = ((acro_yaw_sp - prev_acro_yaw_sp) / dt_pid) * ff_yaw * 0.01f;
  }
  prev_acro_roll_sp = acro_roll_sp;
  prev_acro_pitch_sp = acro_pitch_sp;
  prev_acro_yaw_sp = acro_yaw_sp;

  // Dynamically set PIDs, scaling P and D by the TPA factor
  if (current_flight_mode == MODE_ACRO) {
      setpoint_roll = acro_roll_sp;
      setpoint_pitch = acro_pitch_sp;
      setpoint_yaw = acro_yaw_sp;
      
      roll_pid.set_pid(pid_p_roll_pitch_acro * tpa_factor_dynamic, pid_i_roll_pitch_acro, pid_d_roll_pitch_acro * tpa_factor_dynamic);
      pitch_pid.set_pid(pid_p_roll_pitch_acro * tpa_factor_dynamic, pid_i_roll_pitch_acro, pid_d_roll_pitch_acro * tpa_factor_dynamic);
      yaw_pid.set_pid(pid_p_yaw * tpa_factor_dynamic, pid_i_yaw, pid_d_yaw * tpa_factor_dynamic);

      roll_control_output = roll_pid.compute(acro_roll_sp, gx_rate, dt_pid) + ff_roll_out;
      pitch_control_output = pitch_pid.compute(acro_pitch_sp, gy_rate, dt_pid) + ff_pitch_out;
      yaw_control_output = yaw_pid.compute(acro_yaw_sp, gz_rate, dt_pid) + ff_yaw_out;
  } else {
      setpoint_roll = receiver_pwm[0];
      setpoint_pitch = receiver_pwm[1];
      setpoint_yaw = acro_yaw_sp;
      
      roll_pid.set_pid(pid_p_roll_pitch_angle * tpa_factor_dynamic, pid_i_roll_pitch_angle, pid_d_roll_pitch_angle * tpa_factor_dynamic);
      pitch_pid.set_pid(pid_p_roll_pitch_angle * tpa_factor_dynamic, pid_i_roll_pitch_angle, pid_d_roll_pitch_angle * tpa_factor_dynamic);
      yaw_pid.set_pid(pid_p_yaw * tpa_factor_dynamic, pid_i_yaw, pid_d_yaw * tpa_factor_dynamic);

      roll_control_output = roll_pid.compute(receiver_pwm[0], roll, dt_pid) + ff_roll_out;
      pitch_control_output = pitch_pid.compute(receiver_pwm[1], pitch, dt_pid) + ff_pitch_out;
      yaw_control_output = yaw_pid.compute(acro_yaw_sp, gz_rate, dt_pid) + ff_yaw_out;
  }

  // Motor PWM output
  motor.update_motors_pwm(receiver_pwm[2], roll_control_output,
                          pitch_control_output, yaw_control_output);
}

void update_telemetry_and_blackbox(WritePWM &motor, float gz_rate, float dt_ekf,
                                   float dt_pid, uint64_t end_time_us,
                                   bool &blackbox_updated) {
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
  
  // Populate IMU for Configurator GET_IMU
  shared_ax = raw_ax;
  shared_ay = raw_ay;
  shared_az = raw_az;
  shared_gx = raw_gx;
  shared_gy = raw_gy;
  shared_gz = raw_gz;
  
  send_telemetry = true;

  // update the blackbox packet
  static uint32_t loop_iter = 0;
  static uint64_t last_bb_time_us = 0;
  
  if (last_bb_time_us == 0 || !is_armed) {
      last_bb_time_us = end_time_us;
  }
  
  bb_packet.loop_iteration = loop_iter++;
  bb_packet.dt_us = (uint32_t)(end_time_us - last_bb_time_us);
  last_bb_time_us = end_time_us;
  
  bb_packet.setpoint_roll = (int32_t)(setpoint_roll * 100.0f);
  bb_packet.setpoint_pitch = (int32_t)(setpoint_pitch * 100.0f);
  bb_packet.setpoint_yaw = (int32_t)(setpoint_yaw * 100.0f);
  
  bb_packet.roll = (int32_t)(roll * 100.0f);
  bb_packet.pitch = (int32_t)(pitch * 100.0f);
  bb_packet.yaw_rate = (int32_t)(gz_rate * 100.0f);
  
  bb_packet.gyro_x = (int32_t)(raw_gx * 1000.0f);
  bb_packet.gyro_y = (int32_t)(raw_gy * 1000.0f);
  bb_packet.gyro_z = (int32_t)(raw_gz * 1000.0f);
  bb_packet.accel_x = (int32_t)(raw_ax * 1000.0f);
  bb_packet.accel_y = (int32_t)(raw_ay * 1000.0f);
  bb_packet.accel_z = (int32_t)(raw_az * 1000.0f);
  
  bb_packet.pid_roll = (int32_t)(roll_control_output * 100.0f);
  bb_packet.pid_pitch = (int32_t)(pitch_control_output * 100.0f);
  bb_packet.pid_yaw = (int32_t)(yaw_control_output * 100.0f);
  bb_packet.rc_roll = (int32_t)(receiver_pwm[0] * 100.0f);
  bb_packet.rc_pitch = (int32_t)(receiver_pwm[1] * 100.0f);
  bb_packet.rc_yaw = (int32_t)(receiver_pwm[3] * 100.0f);
  bb_packet.rc_throttle = (uint16_t)(receiver_pwm[2]);
  bb_packet.motor1 = (uint16_t)(motor.motor1_speed);
  bb_packet.motor2 = (uint16_t)(motor.motor2_speed);
  bb_packet.motor3 = (uint16_t)(motor.motor3_speed);
  bb_packet.motor4 = (uint16_t)(motor.motor4_speed);
  blackbox.write_packet(bb_packet);
  blackbox_updated = true;
}

void handle_disarmed_state(WritePWM &motor, QuaternionEKF &filter,
                           bool &blackbox_updated, bool &blackbox_dumped) {
  if (esc_calibrating) {
      motor.update_motors_pwm(2000.0f, 2000.0f, 2000.0f, 2000.0f);
  } else {
      motor.reset();
  }
  // write the blackbox to the flash
  if (blackbox_updated) {
    blackbox.write_blackbox_to_flash();
    blackbox_updated = false;
  }
  // Automatic USB dumping removed; Configurator will explicitly request it.
  
  send_telemetry = false;
  roll_pid.reset();
  pitch_pid.reset();
  yaw_pid.reset();
  
  static uint64_t mode_switch_timer = 0;
  bool is_yaw_stick_left = raw_receiver_pwm[3] < 1150.0f;
  bool is_yaw_stick_right = raw_receiver_pwm[3] > 1850.0f;
  bool is_throttle_low = raw_receiver_pwm[2] < 1050.0f;

  if (is_throttle_low && is_yaw_stick_left) {
      if (mode_switch_timer == 0) {
          mode_switch_timer = time_us_64();
      } else if (time_us_64() - mode_switch_timer > 1000000) {
          if (current_flight_mode != MODE_ACRO) {
              current_flight_mode = MODE_ACRO;
              fc_buzzer.play_tone(1500);
              sleep_ms(200);
              fc_buzzer.stop();
          }
      }
  } else if (is_throttle_low && is_yaw_stick_right) {
      if (mode_switch_timer == 0) {
          mode_switch_timer = time_us_64();
      } else if (time_us_64() - mode_switch_timer > 1000000) {
          if (current_flight_mode != MODE_ANGLE) {
              current_flight_mode = MODE_ANGLE;
              fc_buzzer.play_tone(1000);
              sleep_ms(200);
              fc_buzzer.stop();
          }
      }
  } else {
      mode_switch_timer = 0;
  }
  
  static uint64_t last_print_time = 0;
  if (time_us_64() - last_print_time > 200000) {
      DEBUG_PRINT("Not Armed. Throttle: %f\n", receiver_pwm[2]);
      last_print_time = time_us_64();
  }
  
  gpio_put(PICO_DEFAULT_LED_PIN, 0);
}
