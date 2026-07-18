#include "blackbox.h"
#include "config.h"
#include <string.h>
#include "pico/multicore.h"
#include "core/globals.h"

Blackbox::Blackbox() {
  packet_buffer = new BlackboxPacket[MAX_BLACKBOX_PACKETS];
  clear_blackbox_data();

  // Find the ring buffer head and tail in the 2MB flash
  const uint8_t *flash_data = (const uint8_t *)(XIP_BASE + FLASH_TARGET_OFFSET);
  
  flash_write_offset = 0;
  flash_read_offset = 0;

  // Check the very last page to know the wrap-around state
  bool last_page_empty = true;
  for (int i = 0; i < 256; i++) {
    if (flash_data[FLASH_MAX_SIZE - 256 + i] != 0xFF) { 
        last_page_empty = false; 
        break; 
    }
  }
  
  bool prev_empty = last_page_empty;

  // Scan all 256-byte pages in the 2MB block to find transitions
  for (uint32_t offset = 0; offset < FLASH_MAX_SIZE; offset += 256) {
    bool curr_empty = true;
    for (int i = 0; i < 256; i++) {
      if (flash_data[offset + i] != 0xFF) { 
          curr_empty = false; 
          break; 
      }
    }
    
    if (!prev_empty && curr_empty) {
      // Data -> Empty transition! This is the write head.
      flash_write_offset = offset;
    } else if (prev_empty && !curr_empty) {
      // Empty -> Data transition! This is the read tail.
      flash_read_offset = offset;
    }
    prev_empty = curr_empty;
  }
}

void Blackbox::clear_blackbox_data() {
  head = 0;
  tail = 0;
  count = 0;
  memset(packet_buffer, 0xFF, sizeof(BlackboxPacket) * MAX_BLACKBOX_PACKETS);
}

void Blackbox::write_packet(const BlackboxPacket &packet) {
  packet_buffer[head] = packet;
  head = (head + 1) % MAX_BLACKBOX_PACKETS;
  if (count < MAX_BLACKBOX_PACKETS) {
    count++;
  } else {
    tail = (tail + 1) % MAX_BLACKBOX_PACKETS;
  }
}

void Blackbox::write_blackbox_to_flash() {
  if (count == 0) return;

  DEBUG_PRINT("Saving Blackbox to Flash... Do not unplug!\n");

  multicore_lockout_start_blocking();
  uint32_t interrupts = save_and_disable_interrupts();

  uint8_t page_buffer[256];
  size_t page_idx = 0;

  // --- PREPEND METADATA ---
  BlackboxPacket metadata[5];
  memset(metadata, 0, sizeof(metadata));

  // 1. Flight Separator
  metadata[0].dt_us = 0xFFFE;
  
  // 2. PID RP
  metadata[1].dt_us = 0xFFFD;
  metadata[1].pid_roll = (int16_t)(pid_p_roll_pitch * 1000.0f);
  metadata[1].pid_pitch = (int16_t)(pid_i_roll_pitch * 1000.0f);
  metadata[1].pid_yaw = (int16_t)(pid_d_roll_pitch * 1000.0f);

  // 3. PID YAW
  metadata[2].dt_us = 0xFFFC;
  metadata[2].pid_roll = (int16_t)(pid_p_yaw * 1000.0f);
  metadata[2].pid_pitch = (int16_t)(pid_i_yaw * 1000.0f);
  metadata[2].pid_yaw = (int16_t)(pid_d_yaw * 1000.0f);

  // 4. BIAS
  metadata[3].dt_us = 0xFFFB;
  metadata[3].pid_roll = (int16_t)(bias_roll * 1000.0f);
  metadata[3].pid_pitch = (int16_t)(bias_pitch * 1000.0f);
  metadata[3].pid_yaw = (int16_t)(bias_yaw * 1000.0f);

  // 5. EKF
  metadata[4].dt_us = 0xFFFA;
  metadata[4].pid_roll = (int16_t)(q_gyro * 100000.0f);
  metadata[4].pid_pitch = (int16_t)(q_bias * 100000.0f);
  metadata[4].pid_yaw = (int16_t)(r_accel * 1000.0f);

  for (int m = 0; m < 5; m++) {
      uint8_t *packet_ptr = (uint8_t *)&metadata[m];
      for (size_t b = 0; b < sizeof(BlackboxPacket); b++) {
          page_buffer[page_idx++] = packet_ptr[b];
          if (page_idx == 256) {
              if (flash_write_offset % 4096 == 0) flash_range_erase(FLASH_TARGET_OFFSET + flash_write_offset, 4096);
              flash_range_program(FLASH_TARGET_OFFSET + flash_write_offset, page_buffer, 256);
              flash_write_offset += 256;
              if (flash_write_offset >= FLASH_MAX_SIZE) flash_write_offset = 0;
              page_idx = 0;
          }
      }
  }
  // --- END METADATA ---

  for (size_t i = 0; i < count; i++) {
    size_t read_index = (tail + i) % MAX_BLACKBOX_PACKETS;
    uint8_t *packet_ptr = (uint8_t *)&packet_buffer[read_index];

    for (size_t b = 0; b < sizeof(BlackboxPacket); b++) {
      page_buffer[page_idx++] = packet_ptr[b];

      if (page_idx == 256) {
        // Erase sector if we are crossing into a new one
        if (flash_write_offset % 4096 == 0) {
          flash_range_erase(FLASH_TARGET_OFFSET + flash_write_offset, 4096);
        }
        
        flash_range_program(FLASH_TARGET_OFFSET + flash_write_offset, page_buffer, 256);
        
        flash_write_offset += 256;
        if (flash_write_offset >= FLASH_MAX_SIZE) flash_write_offset = 0;
        
        page_idx = 0;
      }
    }
  }

  // Pad the final page
  if (page_idx > 0) {
    while (page_idx < 256) {
      page_buffer[page_idx++] = 0xFF;
    }
    if (flash_write_offset % 4096 == 0) {
      flash_range_erase(FLASH_TARGET_OFFSET + flash_write_offset, 4096);
    }
    flash_range_program(FLASH_TARGET_OFFSET + flash_write_offset, page_buffer, 256);
    
    flash_write_offset += 256;
    if (flash_write_offset >= FLASH_MAX_SIZE) flash_write_offset = 0;
  }

  restore_interrupts(interrupts);
  multicore_lockout_end_blocking();
  
  // Clear RAM for next flight
  clear_blackbox_data();
  
  DEBUG_PRINT("Blackbox Appended Successfully!\n");
}

void Blackbox::dump_flash_to_usb() {
  printf("\n--- BEGIN BLACKBOX DUMP ---\n");
  printf("Roll,Pitch,YawRate,PID_R,PID_P,PID_Y,RC_Roll,RC_Pitch,RC_Yaw,RC_"
         "Throttle,M1,M2,M3,M4,dt_us\n");

  const uint8_t *flash_data = (const uint8_t *)(XIP_BASE + FLASH_TARGET_OFFSET);
  uint32_t current_read = flash_read_offset;

  while (current_read != flash_write_offset) {
    BlackboxPacket p;
    // Read byte-by-byte to handle wrap-around
    for (size_t b = 0; b < sizeof(BlackboxPacket); b++) {
        uint32_t addr = (current_read + b) % FLASH_MAX_SIZE;
        ((uint8_t*)&p)[b] = flash_data[addr];
    }

    if (p.dt_us == 0xFFFF) {
      // Hit padding! Jump to the next page boundary
      uint32_t next_page = ((current_read / 256) + 1) * 256;
      current_read = next_page % FLASH_MAX_SIZE;
      continue;
    }

    if (p.dt_us == 0xFFFE) {
      printf("--- FLIGHT SEPARATOR ---\n");
    } else if (p.dt_us == 0xFFFD) {
      printf("--- TUNING_UPDATE: PID_RP,%.3f,%.3f,%.3f ---\n", p.pid_roll / 1000.0f, p.pid_pitch / 1000.0f, p.pid_yaw / 1000.0f);
    } else if (p.dt_us == 0xFFFC) {
      printf("--- TUNING_UPDATE: PID_YAW,%.3f,%.3f,%.3f ---\n", p.pid_roll / 1000.0f, p.pid_pitch / 1000.0f, p.pid_yaw / 1000.0f);
    } else if (p.dt_us == 0xFFFB) {
      printf("--- TUNING_UPDATE: BIAS,%.3f,%.3f,%.3f ---\n", p.pid_roll / 1000.0f, p.pid_pitch / 1000.0f, p.pid_yaw / 1000.0f);
    } else if (p.dt_us == 0xFFFA) {
      printf("--- TUNING_UPDATE: EKF,%.5f,%.5f,%.3f ---\n", p.pid_roll / 100000.0f, p.pid_pitch / 100000.0f, p.pid_yaw / 1000.0f);
    } else {
      printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%u,%u,%u,%u,%u,%u\n", p.roll, p.pitch,
             p.yaw_rate, p.pid_roll, p.pid_pitch, p.pid_yaw, p.rc_roll,
             p.rc_pitch, p.rc_yaw, p.rc_throttle, p.motor1, p.motor2, p.motor3,
             p.motor4, p.dt_us);
    }

    current_read = (current_read + sizeof(BlackboxPacket)) % FLASH_MAX_SIZE;
  }
  printf("--- END BLACKBOX DUMP ---\n");
}