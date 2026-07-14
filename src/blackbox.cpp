#include "blackbox.h"
#include <string.h>

Blackbox::Blackbox() {
  packet_buffer = new BlackboxPacket[MAX_BLACKBOX_PACKETS];
  clear_blackbox_data();
}

void Blackbox::clear_blackbox_data() {
  head = 0;
  tail = 0;
  count = 0;

  // Wipe RAM clean with 0xFF.
  // 0xFF (-1) is the standard value for "Empty Flash".
  // This guarantees your dump_flash_to_usb() function stops correctly!
  memset(packet_buffer, 0xFF, sizeof(BlackboxPacket) * MAX_BLACKBOX_PACKETS);
}

void Blackbox::write_packet(const BlackboxPacket &packet) {
  // 1. Write the packet as a whole object to the current head position
  packet_buffer[head] = packet;

  // 2. Advance the head, wrapping back to 0 if it hits the end
  head = (head + 1) % MAX_BLACKBOX_PACKETS;

  // 3. Update count and tail
  if (count < MAX_BLACKBOX_PACKETS) {
    count++; // RAM is not full yet
  } else {
    // RAM is full! The head just overwrote the oldest data.
    // We must push the tail forward to point to the new "oldest" data.
    tail = (tail + 1) % MAX_BLACKBOX_PACKETS;
  }
}

void Blackbox::write_blackbox_to_flash() {
  if (count == 0)
    return;

  printf("Saving Blackbox to Flash... Do not unplug!\n");

  // Calculate total bytes and strictly aligned Erase/Write sizes
  size_t total_bytes = count * sizeof(BlackboxPacket);
  size_t erase_size =
      ((total_bytes + FLASH_SECTOR_SIZE - 1) / FLASH_SECTOR_SIZE) *
      FLASH_SECTOR_SIZE;

  uint32_t interrupts = save_and_disable_interrupts();

  // Erase the required flash sectors
  flash_range_erase(FLASH_TARGET_OFFSET, erase_size);

  // Because Flash must be written in exact 256-byte chunks (FLASH_PAGE_SIZE),
  // we use a small local buffer to construct perfect 256-byte pages before
  // writing.
  uint8_t page_buffer[FLASH_PAGE_SIZE];
  size_t page_idx = 0;
  uint32_t current_flash_address = FLASH_TARGET_OFFSET;

  // Linearize the ring buffer: Read from oldest (tail) to newest
  for (size_t i = 0; i < count; i++) {
    size_t read_index = (tail + i) % MAX_BLACKBOX_PACKETS;
    uint8_t *packet_ptr = (uint8_t *)&packet_buffer[read_index];

    // Feed bytes into the 256-byte page buffer
    for (size_t b = 0; b < sizeof(BlackboxPacket); b++) {
      page_buffer[page_idx++] = packet_ptr[b];

      // When the page buffer is perfectly full, burn it to flash!
      if (page_idx == FLASH_PAGE_SIZE) {
        flash_range_program(current_flash_address, page_buffer,
                            FLASH_PAGE_SIZE);
        current_flash_address += FLASH_PAGE_SIZE;
        page_idx = 0; // Reset for the next page
      }
    }
  }

  // If there are leftover bytes that didn't perfectly fill a 256-byte page,
  // pad the rest of the page with 0xFF and flush it.
  if (page_idx > 0) {
    while (page_idx < FLASH_PAGE_SIZE) {
      page_buffer[page_idx++] = 0xFF;
    }
    flash_range_program(current_flash_address, page_buffer, FLASH_PAGE_SIZE);
  }

  restore_interrupts(interrupts);
  printf("Blackbox Saved Successfully! (%d packets)\n", count);
}

void Blackbox::dump_flash_to_usb() {
  printf("\n--- BEGIN BLACKBOX DUMP ---\n");
  printf("Roll,Pitch,YawRate,PID_R,PID_P,PID_Y,RC_Roll,RC_Pitch,RC_Yaw,RC_"
         "Throttle,M1,M2,M3,M4,dt_us\n");

  // Point directly to the physical flash memory
  const uint8_t *flash_data = (const uint8_t *)(XIP_BASE + FLASH_TARGET_OFFSET);
  size_t byte_index = 0;

  // Loop through the maximum possible packets that could fit in our sector
  for (size_t i = 0; i < MAX_BLACKBOX_PACKETS; i++) {
    BlackboxPacket p;
    memcpy(&p, &flash_data[byte_index], sizeof(BlackboxPacket));

    // --- POWER LOSS FIX ---
    // Empty flash memory defaults to all 1s (0xFF).
    // If the dt_us (timestamp) reads as 65535 (0xFFFF), it means
    // we have reached the end of the recorded flight data!
    if (p.dt_us == 0xFFFF) {
      printf("--- REACHED END OF FLIGHT DATA (%d packets) ---\n", i);
      break;
    }

    // write the packet to USB
    printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%u,%u,%u,%u,%u,%u\n", p.roll, p.pitch,
           p.yaw_rate, p.pid_roll, p.pid_pitch, p.pid_yaw, p.rc_roll,
           p.rc_pitch, p.rc_yaw, p.rc_throttle, p.motor1, p.motor2, p.motor3,
           p.motor4, p.dt_us);

    byte_index += sizeof(BlackboxPacket);
  }
  printf("--- END BLACKBOX DUMP ---\n");
}