# PID Calibration Guide

This document explains how to calibrate the **PID controllers** on the CROX flight controller using the **NRF24L01+ radio link**.

> **Note:** The UART telemetry system (`telemetry.h` / `telemetry.cpp`) is only for testing and tuning the **EKF filter**. All PID calibration is done over the NRF24 radio.

---

## Overview

The flight controller runs three independent PID loops:

| Axis  | PID instance | Measures        | Controls          |
|-------|-------------|-----------------|-------------------|
| Roll  | `roll_pid`  | EKF roll angle  | Differential motor thrust |
| Pitch | `pitch_pid` | EKF pitch angle | Differential motor thrust |
| Yaw   | `yaw_pid`   | Gyro Z rate (°/s) | Differential motor torque |

Each PID has three tunable gains (**P**, **I**, **D**), plus an angle **bias** offset. Compile‑time defaults live in `config/config.h`:

```c
// Roll & Pitch (shared gains)
#define ROLL_P_DEFAULT   2.5f
#define ROLL_I_DEFAULT   0.8f
#define ROLL_D_DEFAULT   0.0f
#define PITCH_P_DEFAULT  2.5f
#define PITCH_I_DEFAULT  0.8f
#define PITCH_D_DEFAULT  0.0f

// Yaw
#define YAW_P_DEFAULT    2.0f
#define YAW_I_DEFAULT    0.0f
#define YAW_D_DEFAULT    0.0f

// Angle biases
#define BIAS_ROLL_DEFAULT   0.0f
#define BIAS_PITCH_DEFAULT  0.0f
#define BIAS_YAW_DEFAULT    0.0f
```

---

## How the Radio Tuning Loop Works

### Architecture

PID tuning runs entirely over the NRF24 radio. **Core 1** handles all radio I/O:

```
Ground Station ──[PIDTuningPacket]──► Drone (Core 1)
                                         │
                                         ├─ Updates roll_pid / pitch_pid / yaw_pid gains
                                         └─ Updates bias_roll / bias_pitch / bias_yaw

Drone (Core 0 flight loop) ──[shared vars]──► Core 1 ──[TelemetryPacket]──► Ground Station
```

### Packets

**TelemetryPacket** (Drone → Ground, sent at **10 Hz**):

| Field        | Type     | Content                         |
|-------------|----------|---------------------------------|
| `roll`      | int16    | EKF roll × 100                  |
| `pitch`     | int16    | EKF pitch × 100                 |
| `yaw`       | int16    | Gyro Z rate × 100               |
| `rc_roll`   | int16    | RC stick roll input × 100       |
| `rc_pitch`  | int16    | RC stick pitch input × 100      |
| `rc_yaw`    | int16    | RC stick yaw input × 100        |
| `pid_roll`  | int16    | Roll PID output × 100           |
| `pid_pitch` | int16    | Pitch PID output × 100          |
| `pid_yaw`   | int16    | Yaw PID output × 100            |
| `dt_s`      | uint16   | Timestamp                       |

**PIDTuningPacket** (Ground → Drone):

| Field             | Type   | Content                        |
|------------------|--------|---------------------------------|
| `kp_roll_pitch`  | int16  | P gain (roll & pitch) × 1000   |
| `ki_roll_pitch`  | int16  | I gain (roll & pitch) × 1000   |
| `kd_roll_pitch`  | int16  | D gain (roll & pitch) × 1000   |
| `kp_yaw`         | int16  | P gain (yaw) × 1000            |
| `ki_yaw`         | int16  | I gain (yaw) × 1000            |
| `kd_yaw`         | int16  | D gain (yaw) × 1000            |
| `bias_roll`      | int16  | Roll bias × 1000               |
| `bias_pitch`     | int16  | Pitch bias × 1000              |
| `bias_yaw`       | int16  | Yaw bias × 1000                |

When a valid `PIDTuningPacket` arrives, the drone:
1. Decodes gains by dividing by 1000 (e.g., `kp = kp_roll_pitch / 1000.0f`).
2. Calls `roll_pid.set_pid(kp, ki, kd)`, `pitch_pid.set_pid(kp, ki, kd)`, `yaw_pid.set_pid(...)`.
3. Updates `bias_roll`, `bias_pitch`, `bias_yaw`.
4. Plays a confirmation tone via the buzzer.

---

## Calibration Procedure

### 1. Setup

1. Power on the drone — the startup melody confirms hardware is OK.
2. Open your ground station software and verify the NRF24 radio link is active (you should see telemetry packets arriving at ~10 Hz).
3. Keep the drone **unarmed** initially (throttle low, arm switch off).

### 2. Set Angle Biases First

Before tuning PID gains, correct for any mechanical tilt:

1. Place the drone on a flat, level surface.
2. Arm and raise throttle just enough to see the EKF roll/pitch values in telemetry.
3. If `roll` or `pitch` are not near zero, send a `PIDTuningPacket` with the bias values to compensate. For example, if the drone reads `roll = +2.3°`, set `bias_roll = 2.3` (i.e., send `2300` in the packet).
4. Disarm and verify the corrected readings.

### 3. Tune Proportional Gain (P)

> Roll & pitch share the same P/I/D gains in the tuning packet.

1. Start with a low P (e.g., `1.0`). Set I and D to `0.0`.
2. Send the gains via `PIDTuningPacket` — the buzzer will beep to confirm.
3. Arm the drone and hover. Watch the telemetry:
   - `roll` / `pitch` — actual angles
   - `rc_roll` / `rc_pitch` — your stick commands
   - `pid_roll` / `pid_pitch` — controller outputs
4. Increase P gradually until the drone responds crisply to stick inputs without oscillating.
5. **Too low P**: Drone feels sluggish, drifts.
6. **Too high P**: Fast oscillations, drone wobbles.

### 4. Tune Integral Gain (I)

1. With a good P, slowly increase I (e.g., `0.3` → `0.5` → `0.8`).
2. I eliminates steady‑state error — the drone should hold its angle without drift.
3. **Too high I**: Slow oscillations, the drone "hunts" around the setpoint, or the PID output saturates.

### 5. Tune Derivative Gain (D)

1. Only add D if you see high‑frequency oscillations after tuning P and I.
2. Start very small (e.g., `0.01`). D dampens rapid changes.
3. **Too high D**: Amplifies sensor noise, motors buzz/vibrate.
4. For many builds, `D = 0.0` works fine.

### 6. Tune Yaw Separately

Yaw has its own P/I/D fields in the `PIDTuningPacket`. Repeat steps 3–5 for yaw:
- Yaw typically needs lower P than roll/pitch.
- The yaw loop controls **rate** (°/s from gyro Z), not angle.

### 7. Validate

1. Perform gentle stick movements in all axes.
2. Check that the `pid_*` telemetry tracks the `rc_*` commands smoothly.
3. Look for any overshoot, oscillation, or drift in the telemetry data.
4. Use the blackbox to record a flight and analyze the motor responses offline.

---

## Quick Tuning Reference

| Step | What to adjust | Start value | Watch for |
|------|---------------|-------------|-----------|
| 1    | Bias roll/pitch/yaw | Measure at rest | Level readings at 0° |
| 2    | P (roll/pitch) | 1.0 | Crisp response, no oscillation |
| 3    | I (roll/pitch) | 0.3 | No drift, no wind‑up |
| 4    | D (roll/pitch) | 0.0 | Dampen only if needed |
| 5    | P (yaw)        | 1.0 | Responsive yaw rate |
| 6    | I (yaw)        | 0.0 | Usually not needed |
| 7    | D (yaw)        | 0.0 | Usually not needed |

---

## Summary Table (Defaults)

| Axis  | P     | I     | D     | Bias  |
|-------|-------|-------|-------|-------|
| Roll  | 2.5   | 0.8   | 0.0   | 0.0   |
| Pitch | 2.5   | 0.8   | 0.0   | 0.0   |
| Yaw   | 2.0   | 0.0   | 0.0   | 0.0   |

---

## Disarm Behaviour

When the drone is disarmed, the main loop automatically:
- Resets all three PID controllers (`roll_pid.reset()`, etc.), clearing integral state.
- Stops motors.
- Writes any buffered blackbox data to flash.

This means you get a clean PID state every time you re‑arm — no leftover integral wind‑up from a previous flight.

---

*Last updated: 2026‑07‑14*
