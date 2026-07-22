#ifndef CONFIG_H
#define CONFIG_H

#include "hardware/spi.h"

// =============================================================================
// SYSTEM
// =============================================================================
#define CPU_FREQ_KHZ 250000
#define DEBUG_MODE // Uncomment to enable debug printf
#define CALIBRATION_SAMPLES 1000

// =============================================================================
// IMU (BMI160) — SPI0
// =============================================================================
#define IMU_SPI_PORT spi0
#define IMU_PIN_MISO 16
#define IMU_PIN_CS 17
#define IMU_PIN_SCK 18
#define IMU_PIN_MOSI 19
#define IMU_INT_PIN 20

// =============================================================================
// NRF24 RADIO — SPI1
// =============================================================================
#define RADIO_SPI_PORT spi1
#define RADIO_CE 14
#define RADIO_CSN 13
#define RADIO_SCK 10
#define RADIO_MOSI 11
#define RADIO_MISO 12

// 5-byte radio addresses (Drone ↔ Ground Station must be flipped)
#define DRONE_TX_ADDR {'G', 'R', 'N', 'D', '1'} // Sending to Ground
#define DRONE_RX_ADDR {'D', 'R', 'O', 'N', '1'} // Receiving from Ground

// =============================================================================
// NRF24 PACKET HEADERS
// =============================================================================
#define TELEMETRY_HEADER_1 0xAA
#define TELEMETRY_HEADER_2 0x55
#define CMD_HEADER_1 0xCC
#define CMD_HEADER_2 0xDD
#define RESP_HEADER_1 0xDD
#define RESP_HEADER_2 0xCC

// =============================================================================
// BUZZER
// =============================================================================
#define BUZZER_PIN 15

// =============================================================================
// MOTOR PINS & ESC
// =============================================================================
// Motor 1 = Front Left  (CW)
// Motor 2 = Front Right (CCW)
// Motor 3 = Rear Left   (CCW)
// Motor 4 = Rear Right  (CW)
#define MOTOR1_PWM_PIN 2
#define MOTOR2_PWM_PIN 3
#define MOTOR3_PWM_PIN 5
#define MOTOR4_PWM_PIN 4
#define PWM_FREQUENCY_HZ 490
#define PWM_WRAP_VALUE 2040 // Period = 1,000,000 / 490 ≈ 2040 µs
#define ESC_MIN_PULSE_US 1000
#define ESC_MAX_PULSE_US 2000

// =============================================================================
// RECEIVER (PWM INPUT)
// =============================================================================
#define NUM_CHANNELS 4
#define RC_CH1_PIN 9 // Roll
#define RC_CH2_PIN 8 // Pitch
#define RC_CH3_PIN 7 // Throttle
#define RC_CH4_PIN 6 // Yaw
#define CONTROL_OUT_MIN -30.0f
#define CONTROL_OUT_MAX 30.0f
#define DEADBAND_US 8.0f // ± 8 µs from center
#define RC_CENTER_US 1500.0f

// =============================================================================
// FLIGHT MODES
// =============================================================================
enum FlightMode {
    MODE_ANGLE,
    MODE_ACRO
};

#define ACRO_RATE_SCALING 6.6666f // Maps RC -30..30 to -200..200 deg/s

// =============================================================================
// ANGLE MODE TUNING (Outer Loop)
// =============================================================================
#define DEFAULT_ANGLE_STRENGTH 5.0f
#define DEFAULT_ANGLE_MAX_DEG 45.0f
#define DEFAULT_ANGLE_MAX_RATE 300.0f

// =============================================================================
// RATE PID DEFAULTS (Inner Loop)
// =============================================================================
#define DEFAULT_PID_P_ROLL_PITCH_ACRO 3.50000f
#define DEFAULT_PID_I_ROLL_PITCH_ACRO 0.10000f
#define DEFAULT_PID_D_ROLL_PITCH_ACRO 0.15000f

// PID DEFAULTS (Yaw)
#define DEFAULT_PID_P_YAW 4.00000f
#define DEFAULT_PID_I_YAW 0.10000f
#define DEFAULT_PID_D_YAW 0.00000f

// Feed-Forward Gains (Betaflight style)
#define DEFAULT_FF_ROLL 0.0f
#define DEFAULT_FF_PITCH 0.0f
#define DEFAULT_FF_YAW 0.0f

// PID Constraints
#define PID_INTEGRAL_LIMIT 400.0f
#define PID_D_CUTOFF_HZ 25.0f

// =============================================================================
// EKF DEFAULTS
// =============================================================================
#define DEFAULT_Q_GYRO 0.001f
#define DEFAULT_Q_BIAS 0.00001f
#define DEFAULT_R_ACCEL 30.0f
#define DYNAMIC_R_MAX 100.0f

// =============================================================================
// TPA SETTINGS
// =============================================================================
#define TPA_BREAKPOINT 1350.0f
#define TPA_FACTOR 0.30000f

// =============================================================================
// ANGLE BIAS DEFAULTS
// =============================================================================
#define DEFAULT_BIAS_ROLL -3.95f
#define DEFAULT_BIAS_PITCH 1.87f
#define DEFAULT_BIAS_YAW 0.0f

// =============================================================================
// RECEIVER TUNING
// =============================================================================
#define DEFAULT_RC_EXPO 0.0f
#define DEFAULT_RC_DEADBAND 8.0f
#define DEFAULT_RC_YAW_DEADBAND 5.0f

#define DEFAULT_RC_ROLL_REVERSE false
#define DEFAULT_RC_PITCH_REVERSE false
#define DEFAULT_RC_YAW_REVERSE false

// Centering offsets
#define DEFAULT_RC_ROLL_CENTER 1500.0f
#define DEFAULT_RC_PITCH_CENTER 1499.0f
#define DEFAULT_RC_YAW_CENTER 1502.0f

// =============================================================================
// BLACKBOX
// =============================================================================
#define BLACKBOX_RAM_BYTES (256 * 1024)   // 256 KB of RAM
#define FLASH_TARGET_OFFSET (2048 * 1024) // Save at the 2 MB mark
#define FLASH_MAX_SIZE (2048 * 1024)      // 2 MB of Flash available for ring buffer

// SD Card configuration (shares SPI1 with NRF24)
#define USE_SD_CARD_LOGGING false  // Set to false if physically swapping to NRF24
#define USE_NRF24_RADIO (!USE_SD_CARD_LOGGING)
#define SD_SPI_PORT spi1
#define SD_PIN_MISO 12
#define SD_PIN_CS 13              // Same as RADIO_CSN since they are physically swapped
#define SD_PIN_SCK 10
#define SD_PIN_MOSI 11

// =============================================================================
// TELEMETRY & MSP
// =============================================================================

// UART Telemetry (Shares GPIO 12/13 with NRF24)
// while UART telemetry is active. IMU remains available.
#define TELEM_UART_ID uart0
#define TELEM_UART_TX_PIN 12 // Shared with RADIO_MISO
#define TELEM_UART_RX_PIN 13 // Shared with RADIO_CSN
#define TELEM_BAUD_RATE 38400

#include <stdio.h>
#ifndef DEBUG_PRINT
#ifdef DEBUG_MODE
#define DEBUG_PRINT(...) printf(__VA_ARGS__)
#else
#define DEBUG_PRINT(...)
#endif
#endif

#endif // CONFIG_H
