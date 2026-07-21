#pragma once

void core1_entry();
void process_command(char* buffer);
void check_serial_commands();

struct RadioCommandPacket;
struct RadioResponsePacket;
void process_radio_command(RadioCommandPacket* cmd, RadioResponsePacket* resp);
