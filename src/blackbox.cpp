#include "blackbox.h"
#include "config.h"
#include <string.h>
#include "pico/multicore.h"
#include "core/globals.h"
#include "pico/util/queue.h"

// Check if SD logging is enabled
#if USE_SD_CARD_LOGGING
#include "ff.h" // FatFS header
#include "hw_config.h" // Pico FatFS hardware config
#endif

// Queue for passing packets from Core 0 to Core 1
queue_t blackbox_queue;
#define BLACKBOX_QUEUE_SIZE 100 // Buffer 100 packets

Blackbox::Blackbox() {
  packet_buffer = new BlackboxPacket[MAX_BLACKBOX_PACKETS];
  clear_blackbox_data();
  sd_initialized = false;

  // Initialize the lock-free queue for Core 0 -> Core 1 communication
  queue_init(&blackbox_queue, sizeof(BlackboxPacket), BLACKBOX_QUEUE_SIZE);

#if USE_SD_CARD_LOGGING
  // Initialize SD Card
  FRESULT fr;
  FATFS fs;
  
  // NOTE: The hardware configuration (hw_config.c) must be defined to use the SD_PIN_* from config.h
  
  fr = f_mount(&fs, "0:", 1);
  if (fr == FR_OK) {
    sd_initialized = true;
    DEBUG_PRINT("SD Card initialized successfully.\n");
  } else {
    DEBUG_PRINT("SD Card initialization failed. Falling back to Flash.\n");
    sd_initialized = false;
  }
#endif

  if (!sd_initialized) {
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
}

void Blackbox::clear_blackbox_data() {
  head = 0;
  tail = 0;
  count = 0;
  memset(packet_buffer, 0xFF, sizeof(BlackboxPacket) * MAX_BLACKBOX_PACKETS);
}

void Blackbox::write_packet(const BlackboxPacket &packet) {
  if (sd_initialized) {
      // If SD card is ready, enqueue packet for Core 1 to process
      // queue_try_add is non-blocking. If queue is full, packet is dropped (avoids freezing Core 0)
      if (!queue_try_add(&blackbox_queue, &packet)) {
          // DEBUG_PRINT("Blackbox queue full! Dropped packet.\n");
      }
  } else {
      // Fallback to internal RAM/Flash buffering
      packet_buffer[head] = packet;
      head = (head + 1) % MAX_BLACKBOX_PACKETS;
      if (count < MAX_BLACKBOX_PACKETS) {
        count++;
      } else {
        tail = (tail + 1) % MAX_BLACKBOX_PACKETS;
      }
  }
}

#if USE_SD_CARD_LOGGING
// We use the same binary format on the SD card as we do in Flash.
// This simplifies the code and avoids the incomplete BFL implementation.
#endif

void Blackbox::sd_logging_task() {
#if USE_SD_CARD_LOGGING
    if (!sd_initialized) return;
    
    static FIL log_file;
    static bool file_open = false;
    
    // Check if armed
    extern bool is_armed;
    
    if (is_armed && !file_open) {
        // Create new log file when armed
        FRESULT fr = f_open(&log_file, "LOG001.BFL", FA_WRITE | FA_CREATE_ALWAYS | FA_OPEN_APPEND);
        if (fr == FR_OK) {
            file_open = true;
            DEBUG_PRINT("Opened SD log file.\n");
        }
    } else if (!is_armed && file_open) {
        // Write explicit EOF packet
        BlackboxPacket eof;
        memset(&eof, 0, sizeof(eof));
        eof.dt_us = 0xFFFFFFFF;
        UINT bw;
        f_write(&log_file, &eof, sizeof(BlackboxPacket), &bw);
        
        f_close(&log_file);
        file_open = false;
        DEBUG_PRINT("Closed SD log file.\n");
    }
    
    if (file_open) {
        BlackboxPacket p;
        while (queue_try_remove(&blackbox_queue, &p)) {
            UINT bw;
            f_write(&log_file, &p, sizeof(BlackboxPacket), &bw);
        }
        
        // Sync to SD card periodically
        f_sync(&log_file);
    }
#endif
}

void Blackbox::write_blackbox_to_flash() {
  if (sd_initialized || count == 0) return; // Don't write to flash if SD is working

  DEBUG_PRINT("Saving Blackbox to Flash... Do not unplug!\n");

  multicore_lockout_start_blocking();
  uint32_t interrupts = save_and_disable_interrupts();

  uint8_t page_buffer[256];
  size_t page_idx = 0;

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

  // Write an explicit End-Of-Flight packet before padding
  BlackboxPacket eof_packet;
  memset(&eof_packet, 0, sizeof(eof_packet));
  eof_packet.dt_us = 0xFFFFFFFF;
  
  uint8_t *eof_ptr = (uint8_t *)&eof_packet;
  for (size_t b = 0; b < sizeof(BlackboxPacket); b++) {
    page_buffer[page_idx++] = eof_ptr[b];
    if (page_idx == 256) {
      if (flash_write_offset % 4096 == 0) flash_range_erase(FLASH_TARGET_OFFSET + flash_write_offset, 4096);
      flash_range_program(FLASH_TARGET_OFFSET + flash_write_offset, page_buffer, 256);
      flash_write_offset += 256;
      if (flash_write_offset >= FLASH_MAX_SIZE) flash_write_offset = 0;
      page_idx = 0;
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

void Blackbox::send_binary_info() {
#if USE_SD_CARD_LOGGING
    if (sd_initialized) {
        FIL file;
        if (f_open(&file, "LOG001.BFL", FA_READ) == FR_OK) {
            uint32_t fsize = f_size(&file);
            uint32_t num_packets = fsize / sizeof(BlackboxPacket);
            printf("BLACKBOX_INFO,%lu,%zu\n", num_packets, sizeof(BlackboxPacket));
            f_close(&file);
            return;
        }
    }
#endif

    // Count flash packets
    uint32_t num_packets = 0;
    const uint8_t *flash_data = (const uint8_t *)(XIP_BASE + FLASH_TARGET_OFFSET);
    uint32_t current_read = flash_read_offset;
    
    while (current_read != flash_write_offset) {
        // Read dt_us to check for EOF marker
        uint32_t dt_us = 0;
        for (size_t b = 0; b < 4; b++) {
            uint32_t addr = (current_read + 4 /* offset of dt_us */ + b) % FLASH_MAX_SIZE;
            ((uint8_t*)&dt_us)[b] = flash_data[addr];
        }
        
        uint32_t packet_start = current_read;
        current_read = (current_read + sizeof(BlackboxPacket)) % FLASH_MAX_SIZE;
        
        if (dt_us == 0xFFFFFFFF) {
            uint32_t next_page = ((packet_start / 256) + 1) * 256;
            current_read = next_page % FLASH_MAX_SIZE;
            continue;
        }
        num_packets++;
    }
    
    printf("BLACKBOX_INFO,%lu,%zu\n", num_packets, sizeof(BlackboxPacket));
    fflush(stdout);
}

void Blackbox::dump_flash_to_usb() {
    const uint8_t *flash_data = (const uint8_t *)(XIP_BASE + FLASH_TARGET_OFFSET);
    uint32_t current_read = flash_read_offset;

    while (current_read != flash_write_offset) {
        BlackboxPacket p;
        for (size_t b = 0; b < sizeof(BlackboxPacket); b++) {
            uint32_t addr = (current_read + b) % FLASH_MAX_SIZE;
            ((uint8_t*)&p)[b] = flash_data[addr];
        }

        uint32_t packet_start = current_read;
        current_read = (current_read + sizeof(BlackboxPacket)) % FLASH_MAX_SIZE;

        if (p.dt_us == 0xFFFFFFFF) {
            uint32_t next_page = ((packet_start / 256) + 1) * 256;
            current_read = next_page % FLASH_MAX_SIZE;
            
            // Still write the EOF packet so configurator can separate flights
            fwrite(&p, 1, sizeof(BlackboxPacket), stdout);
            continue;
        }

        fwrite(&p, 1, sizeof(BlackboxPacket), stdout);
    }
}

void Blackbox::send_to_usb() {
#if USE_SD_CARD_LOGGING
    if (sd_initialized) {
        FIL file;
        if (f_open(&file, "LOG001.BFL", FA_READ) == FR_OK) {
            uint32_t fsize = f_size(&file);
            uint32_t num_packets = fsize / sizeof(BlackboxPacket);
            
            printf("BLACKBOX_BIN_START,%lu,%zu\n", num_packets, sizeof(BlackboxPacket));
            fflush(stdout);
            
            // Read and send the file in chunks
            uint8_t buffer[256];
            UINT br;
            while (f_read(&file, buffer, sizeof(buffer), &br) == FR_OK && br > 0) {
                fwrite(buffer, 1, br, stdout);
            }
            f_close(&file);
            
            printf("\nBLACKBOX_BIN_END\n");
            fflush(stdout);
            return;
        }
    }
#endif

    // Flash fallback
    send_binary_info(); // Recalculates but it's fast enough
    
    // Get num packets again
    uint32_t num_packets = 0;
    const uint8_t *flash_data = (const uint8_t *)(XIP_BASE + FLASH_TARGET_OFFSET);
    uint32_t current_read = flash_read_offset;
    while (current_read != flash_write_offset) {
        uint32_t dt_us = 0;
        for (size_t b = 0; b < 4; b++) {
            uint32_t addr = (current_read + 4 + b) % FLASH_MAX_SIZE;
            ((uint8_t*)&dt_us)[b] = flash_data[addr];
        }
        uint32_t packet_start = current_read;
        current_read = (current_read + sizeof(BlackboxPacket)) % FLASH_MAX_SIZE;
        if (dt_us == 0xFFFFFFFF) {
            uint32_t next_page = ((packet_start / 256) + 1) * 256;
            current_read = next_page % FLASH_MAX_SIZE;
            continue;
        }
        num_packets++;
    }

    printf("BLACKBOX_BIN_START,%lu,%zu\n", num_packets, sizeof(BlackboxPacket));
    fflush(stdout);
    
    dump_flash_to_usb();
    
    printf("\nBLACKBOX_BIN_END\n");
    fflush(stdout);
}