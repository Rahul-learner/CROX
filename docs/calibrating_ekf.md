# EKF Calibration Guide

This document explains how to calibrate and tune the **Extended Kalman Filter (EKF)** on the CROX flight controller. The EKF fuses the BMI160 gyroscope and accelerometer data into a stable, drift‑free 3D orientation (roll, pitch, yaw).

> **Note:** EKF tuning is distinct from PID tuning. The EKF determines *where the drone is pointing*. The PID determines *how to move the motors* to achieve the desired orientation. Always ensure the EKF is perfectly tuned and outputs clean angles before attempting to tune the PIDs.

---

## 1. Hardware Calibration (Gyro Bias)

The gyroscope is highly susceptible to temperature changes and minor physical offsets, causing it to "drift" over time.

### Startup Calibration
To fix this, the flight controller automatically calibrates the gyro on **every boot**.

1. Place the drone on a perfectly still, level surface.
2. Plug in the battery / USB power.
3. The firmware will sample the gyro `CALIBRATION_SAMPLES` times (default `1000`, defined in `config/config.h`).
4. It calculates the average drift on the X, Y, and Z axes and subtracts this bias from all future readings.
5. **CRITICAL:** Do not move the drone until the startup buzzer melody finishes playing. If you bump the drone during boot, the EKF will drift constantly in flight.

---

## 2. Covariance Tuning (Q and R Matrices)

The EKF uses covariance matrices to determine how much it "trusts" the physics model (Gyro) versus the absolute measurements (Accelerometer).

These defaults are set in `config/config.h`:

```c
// =============================================================================
// EKF DEFAULTS
// =============================================================================
#define DEFAULT_Q_GYRO   0.001f
#define DEFAULT_Q_BIAS   0.00001f
#define DEFAULT_R_ACCEL  1.5f
```

### Tuning the Measurement Noise ($R$ Matrix)
**`DEFAULT_R_ACCEL`** determines how noisy the accelerometer is.
- **Lower $R$**: The EKF trusts the accelerometer *more*. The 3D model will snap quickly to the true horizon, but it may twitch violently in response to motor vibrations.
- **Higher $R$**: The EKF trusts the accelerometer *less*. It will heavily smooth out motor vibrations, but if the gyro drifts, it will take longer to correct itself back to the true horizon.
- **How to tune**: For a high‑vibration drone frame, increase this value (e.g., `2.0f`). For a very smooth frame with soft‑mounted flight controllers, you can decrease it (e.g., `1.0f`).

### Tuning the Process Noise ($Q$ Matrix)
**`DEFAULT_Q_GYRO`** and **`DEFAULT_Q_BIAS`** determine how much confidence the EKF has in the gyroscope integration math.
- **`Q_GYRO`**: State noise. If you increase this, you are telling the EKF "my gyro integration is probably drifting, please look at the accelerometer more often to fix it."
- **`Q_BIAS`**: Gyro drift rate. Very small number. Represents how fast the gyro bias changes over time due to temperature.

### Tuning Methodology
1. If the drone orientation looks **"twitchy"** or shakes when motors spin up: The filter is trusting the noisy accelerometer too much. **Increase $R$** (`DEFAULT_R_ACCEL`).
2. If the drone orientation looks **"smooth" but slowly floats/drifts** past the actual angle before settling: The filter is ignoring the accelerometer and trusting the gyro too much. **Increase $Q$** (`DEFAULT_Q_GYRO`) or **decrease $R$**.

---

## 3. Using UART Telemetry for EKF Testing

When tuning the EKF, you need to visualize the 3D orientation (roll/pitch/yaw) in real‑time on a ground station or 3D dashboard.

Because you are usually testing this on the bench (often without the NRF24 radio connected or needed), the flight controller supports a dedicated **UART Telemetry Link**.

### UART Configuration

Defined in `config/config.h`:
```c
#define TELEM_UART_ID       uart0
#define TELEM_UART_TX_PIN   12 // Shared with RADIO_MISO
#define TELEM_UART_RX_PIN   13 // Shared with RADIO_CSN
#define TELEM_BAUD_RATE     38400
```

### Important Pin Conflicts
To ensure the BMI160 IMU is always available for the EKF, the UART telemetry pins (`12`, `13`) share GPIO lines with the **NRF24 Radio**.
> ⚠️ **You cannot use UART telemetry and the NRF24 radio at the same time.** The UART telemetry is strictly intended for bench‑testing the EKF filter while the drone is disarmed and plugged into a serial adapter. When you are ready to fly and tune PIDs, rely on the NRF24 radio link instead.

### Testing Steps
1. Connect a USB‑to‑Serial (FTDI) adapter to `GPIO 12 (TX)` and `GPIO 13 (RX)`.
2. Open your ground station dashboard at `38400` baud.
3. The drone will stream binary `TelemetryPacket`s containing the EKF `roll`, `pitch`, and `yaw` at 10 Hz (as long as `send_telemetry` is triggered in the main loop).
4. Move the drone by hand and watch the 3D model. Adjust $Q$ and $R$ in `config.h`, recompile, and flash until the 3D representation perfectly matches your physical hand movements.

---
*Last updated: 2026‑07‑14*
