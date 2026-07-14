# CROX Flight Controller Documentation

Welcome to the documentation for the CROX Flight Controller (RP2350). These guides cover everything you need to know from initial configuration to taking off for the first time.

### ⚙️ [Configuration Reference](config_reference.md)
Start here. This document covers every macro and setting in `config/config.h`, including pin assignments for the IMU, NRF24 radio, motors, RC receiver, and default values for PIDs and EKF. 

### 📐 [EKF Calibration Guide](calibrating_ekf.md)
The Extended Kalman Filter (EKF) is the brain of the drone, figuring out exactly what angle it is currently pitched or rolled. This guide explains how the startup gyro calibration works, how to tune the covariance ($Q$ and $R$) matrices, and how to test the EKF visually over a UART telemetry connection.

### 🛠️ [Flight Readiness & Pre-Flight Checks](flight_readiness.md)
Before you attach the propellers and fly, go through this checklist. It covers critical safety and mechanical setups like ESC calibration, motor direction verification, propeller balancing, Center of Gravity (CG), and failsafe testing.

### 🕹️ [PID Calibration Guide](pid_calibration.md)
Once the EKF is tuned and the drone is flight-ready, you can dial in the flight characteristics. This guide explains how to use the NRF24 radio link and a ground station to live-tune the Proportional, Integral, and Derivative (PID) gains, as well as the angle biases, for perfectly locked-in flight.
