# Configuration Reference (`config/config.h`)

All compile‑time constants for the CROX flight controller are centralised in a single header: [`config/config.h`](../config/config.h).

---

## System

| Macro | Default | Description |
|-------|---------|-------------|
| `CPU_FREQ_KHZ` | `250000` | RP2350 system clock in kHz (250 MHz overclock). |
| `DEBUG_MODE` | *(commented out)* | Uncomment to enable `DEBUG_PRINT` over USB serial. Adds overhead — disable for flight. |
| `CALIBRATION_SAMPLES` | `1000` | Number of gyro samples averaged during startup calibration. |

---

## IMU (BMI160) — SPI0

| Macro | Default | Description |
|-------|---------|-------------|
| `IMU_SPI_PORT` | `spi0` | Hardware SPI peripheral used by the IMU. |
| `IMU_PIN_MISO` | `16` | SPI MISO (data from IMU). |
| `IMU_PIN_CS` | `17` | SPI chip‑select. |
| `IMU_PIN_SCK` | `18` | SPI clock. |
| `IMU_PIN_MOSI` | `19` | SPI MOSI (data to IMU). |
| `IMU_INT_PIN` | `20` | IMU data‑ready interrupt (active‑low). |

---

## NRF24 Radio — SPI1

| Macro | Default | Description |
|-------|---------|-------------|
| `RADIO_SPI_PORT` | `spi1` | Hardware SPI peripheral used by the radio. |
| `RADIO_CE` | `14` | Chip Enable (TX/RX toggle). |
| `RADIO_CSN` | `13` | SPI chip‑select. |
| `RADIO_SCK` | `10` | SPI clock. |
| `RADIO_MOSI` | `11` | SPI MOSI. |
| `RADIO_MISO` | `12` | SPI MISO. |

### Radio Addresses

| Macro | Default | Description |
|-------|---------|-------------|
| `DRONE_TX_ADDR` | `{'G','R','N','D','1'}` | 5‑byte TX address — must match the ground station's RX address. |
| `DRONE_RX_ADDR` | `{'D','R','O','N','1'}` | 5‑byte RX address — must match the ground station's TX address. |

### Packet Headers

| Macro | Value | Used by |
|-------|-------|---------|
| `TELEMETRY_HEADER_1` | `0xAA` | `TelemetryPacket` (Drone → Ground). |
| `TELEMETRY_HEADER_2` | `0x55` | |
| `PID_HEADER_1` | `0xBB` | `PIDTuningPacket` (Ground → Drone). |
| `PID_HEADER_2` | `0x66` | |

---

## Buzzer

| Macro | Default | Description |
|-------|---------|-------------|
| `BUZZER_PIN` | `15` | GPIO for the piezo buzzer. Used for startup melody, error tones, and PID‑update confirmation. |

---

## Motor Pins & ESC

Motor layout (X‑config quadcopter):

```
    Front
  M1(CW)  M2(CCW)
      \  /
       \/
       /\
      /  \
  M3(CCW) M4(CW)
     Rear
```

| Macro | Default | Description |
|-------|---------|-------------|
| `MOTOR1_PWM_PIN` | `2` | Front‑Left (CW). |
| `MOTOR2_PWM_PIN` | `3` | Front‑Right (CCW). |
| `MOTOR3_PWM_PIN` | `5` | Rear‑Left (CCW). |
| `MOTOR4_PWM_PIN` | `4` | Rear‑Right (CW). |
| `PWM_FREQUENCY_HZ` | `490` | ESC PWM frequency in Hz. |
| `PWM_WRAP_VALUE` | `2040` | PWM counter wrap value (period ≈ 1,000,000 / 490 ≈ 2040 µs). |
| `ESC_MIN_PULSE_US` | `1000` | Minimum ESC pulse width (motors off / idle). |
| `ESC_MAX_PULSE_US` | `2000` | Maximum ESC pulse width (full throttle). |

---

## Receiver (PWM Input)

| Macro | Default | Description |
|-------|---------|-------------|
| `NUM_CHANNELS` | `4` | Number of RC PWM channels. |
| `RC_CH1_PIN` | `9` | Roll input. |
| `RC_CH2_PIN` | `8` | Pitch input. |
| `RC_CH3_PIN` | `7` | Throttle input. |
| `RC_CH4_PIN` | `6` | Yaw input. |
| `CONTROL_OUT_MIN` | `-30.0f` | Minimum mapped output (°/s or °) from RC stick. |
| `CONTROL_OUT_MAX` | `30.0f` | Maximum mapped output. |
| `DEADBAND_US` | `8.0f` | ± µs around `RC_CENTER_US` treated as zero input. |
| `RC_CENTER_US` | `1500.0f` | RC stick center pulse width in µs. |

---

## PID Defaults

### Roll & Pitch (shared gains)

| Macro | Default | Description |
|-------|---------|-------------|
| `DEFAULT_PID_P_ROLL_PITCH` | `1.75f` | Proportional gain. |
| `DEFAULT_PID_I_ROLL_PITCH` | `0.04f` | Integral gain. |
| `DEFAULT_PID_D_ROLL_PITCH` | `0.47f` | Derivative gain. |

### Yaw

| Macro | Default | Description |
|-------|---------|-------------|
| `DEFAULT_PID_P_YAW` | `1.0f` | Proportional gain. |
| `DEFAULT_PID_I_YAW` | `0.0f` | Integral gain. |
| `DEFAULT_PID_D_YAW` | `0.0f` | Derivative gain. |

### PID Constraints

| Macro | Default | Description |
|-------|---------|-------------|
| `PID_INTEGRAL_LIMIT` | `400.0f` | Anti‑windup clamp on the integral term. |
| `PID_D_CUTOFF_HZ` | `25.0f` | Low‑pass filter cutoff for the derivative term (reduces noise). |

> These gains can be overridden at runtime via the NRF24 radio. See [PID Calibration Guide](pid_calibration.md).

---

## EKF Defaults

| Macro | Default | Description |
|-------|---------|-------------|
| `DEFAULT_Q_GYRO` | `0.001f` | Process noise for gyro (how much you trust the gyroscope). |
| `DEFAULT_Q_BIAS` | `0.00001f` | Process noise for gyro bias drift. |
| `DEFAULT_R_ACCEL` | `1.5f` | Measurement noise for accelerometer (how much you trust the accelerometer). |

Lower `Q` → trust the gyro more. Lower `R` → trust the accelerometer more.

---

## Angle Bias Defaults

| Macro | Default | Description |
|-------|---------|-------------|
| `DEFAULT_BIAS_ROLL` | `-3.95f` | Mechanical roll offset in degrees. |
| `DEFAULT_BIAS_PITCH` | `1.87f` | Mechanical pitch offset in degrees. |
| `DEFAULT_BIAS_YAW` | `0.0f` | Yaw offset in degrees. |

These compensate for physical tilt of the IMU relative to the frame. Can be updated at runtime via the NRF24 `PIDTuningPacket`.

---

## Blackbox

| Macro | Default | Description |
|-------|---------|-------------|
| `BLACKBOX_RAM_BYTES` | `256 * 1024` (256 KB) | RAM buffer size for in‑flight logging. |
| `FLASH_TARGET_OFFSET` | `2048 * 1024` (2 MB) | Flash offset where blackbox data is persisted. Must not overlap with firmware. |

> Flash writes are page‑aligned (256 bytes). The blackbox flushes to flash when the drone is disarmed.

---

## Telemetry (UART — EKF Testing Only)

> **This section is for EKF testing only.** PID calibration uses the NRF24 radio.

| Macro | Default | Description |
|-------|---------|-------------|
| `TELEM_UART_ID` | `uart0` | UART peripheral. |
| `TELEM_UART_TX_PIN` | `12` | TX GPIO (shared with `RADIO_MISO`). |
| `TELEM_UART_RX_PIN` | `13` | RX GPIO (shared with `RADIO_CSN`). |
| `TELEM_BAUD_RATE` | `38400` | Baud rate. |

> ⚠️ The UART TX/RX pins (`12`, `13`) overlap with the NRF24 radio (`RADIO_MISO` and `RADIO_CSN`). The UART telemetry and NRF24 radio **cannot be used simultaneously**. This is intentional — UART telemetry is only for bench testing the EKF with the IMU connected, when the radio is not needed.

---

*Last updated: 2026‑07‑14*
