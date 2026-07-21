#ifndef BLACKBOX_H
#define BLACKBOX_H

#include "config.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#pragma pack(push, 1) // Prevent memory padding
struct BlackboxPacket {
  uint32_t loop_iteration; // Increments every main loop iteration
  uint32_t dt_us;          // Loop time delta (us) since last iteration
  
  int32_t setpoint_roll;   // deg/s (acro) or deg (angle) * 100
  int32_t setpoint_pitch;  // deg/s (acro) or deg (angle) * 100
  int32_t setpoint_yaw;    // deg/s * 100
  
  int32_t roll;            // degrees * 100
  int32_t pitch;           // degrees * 100
  int32_t yaw_rate;        // degrees/sec * 100 (changed from 10 to 100 for consistency)
  
  // Raw sensors
  int32_t gyro_x;
  int32_t gyro_y;
  int32_t gyro_z;
  int32_t accel_x;
  int32_t accel_y;
  int32_t accel_z;
  
  int32_t pid_roll;
  int32_t pid_pitch;
  int32_t pid_yaw;
  int32_t rc_roll;      // 1000 - 2000
  int32_t rc_pitch;
  int32_t rc_yaw;
  uint16_t rc_throttle; // 1000 - 2000
  uint16_t motor1;      // 1000 - 2000
  uint16_t motor2;
  uint16_t motor3;
  uint16_t motor4;
};
#pragma pack(pop)

// Calculate exactly how many packets fit into 256KB for flash backup
#define MAX_BLACKBOX_PACKETS (BLACKBOX_RAM_BYTES / sizeof(BlackboxPacket))

class Blackbox {
public:
  Blackbox();

  // Pass a fully formed struct, not raw bytes
  void write_packet(const BlackboxPacket &packet);

  // Core 1 SD Logging Thread
  void sd_logging_task();

  // Burn to flash (Call ONLY when disarmed, if SD is not used)
  void write_blackbox_to_flash();

  // Dump to USB in chronological order
  void dump_flash_to_usb();

  void send_to_usb();

  void clear_blackbox_data();
  
  // Helper for configurator download
  void send_binary_info();
  
private:
  void write_betaflight_header();

  BlackboxPacket *packet_buffer;
  size_t head;  // Where we write the next packet in RAM
  size_t tail;  // Where the oldest valid packet is in RAM
  size_t count; // Total packets currently in RAM

  // Flash Ring Buffer Tracking
  uint32_t flash_write_offset; // Byte offset into FLASH_MAX_SIZE where next packet will be written
  uint32_t flash_read_offset;  // Byte offset into FLASH_MAX_SIZE where the oldest valid packet is
  
  bool sd_initialized;
};

#endif // BLACKBOX_H