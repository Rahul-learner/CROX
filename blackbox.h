#ifndef BLACKBOX_H
#define BLACKBOX_H

#include <stdint.h>
#include <stddef.h>
#include "hardware/flash.h"
#include "hardware/sync.h"
#include <stdio.h>

// 256KB of RAM dedicated to Blackbox
#define BLACKBOX_RAM_BYTES (256 * 1024) 
#define FLASH_TARGET_OFFSET (2048 * 1024) // Save at the 2MB mark in Flash

#pragma pack(push, 1) // Prevent memory padding
struct BlackboxPacket {
    int16_t roll;         // degrees * 100
    int16_t pitch;        // degrees * 100
    int16_t yaw_rate;     // degrees/sec * 10
    int16_t pid_roll;     
    int16_t pid_pitch;
    int16_t pid_yaw;
    uint16_t rc_roll;      // 1000 - 2000
    uint16_t rc_pitch;
    uint16_t rc_yaw;
    uint16_t rc_throttle;  // 1000 - 2000
    uint16_t motor1;      // 1000 - 2000
    uint16_t motor2;
    uint16_t motor3;
    uint16_t motor4;
    uint16_t dt_us;       // Loop time
};
#pragma pack(pop)

// Calculate exactly how many packets fit into 256KB
// With a 22-byte packet, this holds ~11,915 packets! (Almost 10 minutes at 20Hz)
#define MAX_BLACKBOX_PACKETS (BLACKBOX_RAM_BYTES / sizeof(BlackboxPacket))

class Blackbox {
public:
    Blackbox();
    
    // Pass a fully formed struct, not raw bytes
    void write_packet(const BlackboxPacket& packet);
    
    // Burn to flash (Call ONLY when disarmed)
    void write_blackbox_to_flash();
    
    // Dump to USB in chronological order
    void dump_flash_to_usb();
    
    void clear_blackbox_data();

private:
    BlackboxPacket* packet_buffer;
    size_t head;  // Where we write the next packet
    size_t tail;  // Where the oldest valid packet is
    size_t count; // Total packets currently in RAM
};

#endif // BLACKBOX_H