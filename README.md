# RP2350 Drone Flight Controller

A hobby flight controller firmware project for the RP2350 platform.

## Overview

This project contains firmware and support materials for a hobby drone flight controller based on the RP2350 microcontroller. The goal is to provide a lightweight, modular codebase for sensor integration, stabilization, and payload control.

## Features

- Core flight control loop for multirotor stabilization
- Support for IMU sensors (accelerometer, gyroscope)
- PID-based attitude control
- Basic motor output management
- Modular architecture for sensor, control, and hardware layers

## Getting Started

### Requirements

- RP2350-compatible development board
- RP2350 SDK or toolchain
- USB programming/debug interface

### Build

1. Install the RP2350 toolchain and any required SDKs.
2. Clone or copy this repository onto your development machine.
3. Navigate to the project directory.
4. Run the build command for your toolchain.

Example:

```bash
make
```

### Flash

Use the RP2350 programming tools or your board-specific flasher to write the compiled firmware to the device.

## Project Structure

- `src/` - firmware source code
- `include/` - headers and shared definitions
- `config/` - build or hardware configuration files
- `docs/` - project notes and reference materials

## Notes

This is a hobby project intended for learning and experimentation. Adjust the firmware and hardware integration to match your specific drone frame and components.

## License

This repository is released under a permissive license. Update this section with the actual license used for the project.
