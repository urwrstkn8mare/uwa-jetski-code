# Main Controller

## Overview

ESP32-based main controller that reads attitude (pitch/roll) from an MPU9250 IMU via I2C and broadcasts it over CAN bus.

## Hardware

- **MCU**: ESP32
- **IMU**: MPU9250 (Grove IMU 10DOF v2.0)
  - **I2C Pins**: SCL=33, SDA=32
  - **I2C Address**: 0x68
- **CAN (TWAI)**:
  - **TX Pin**: GPIO 26
  - **RX Pin**: GPIO 36
  - **Bitrate**: 200 kbps

## Software Architecture

### IMU Component (`components/imu`)

- Initializes the MPU9250 and its on-chip DMP (Digital Motion Processor)
- Performs gyro bias calibration on startup
- Pre-converges the DMP filter for 8 seconds (keep sensor still during boot)
- Captures a zero-reference quaternion so subsequent pitch/roll are relative to the boot orientation
- Runs a background FreeRTOS task that reads DMP FIFO at ~200 Hz
- Stores the latest pitch/roll in a mutex-protected buffer

### CAN Component (`shared_components/can`)

- Uses the ESP-IDF TWAI driver (`esp_driver_twai`)
- Configured for standard 11-bit IDs at 200 kbps
- Provides `can_init()` and `can_tx()` APIs

### Main Application (`main/main.c`)

- Initializes IMU and CAN
- Reads pitch/roll from the IMU component every 100 ms
- Transmits a CAN frame with ID `0x100` containing:
  - Bytes 0-1: Pitch (int16, degrees, rounded)
  - Bytes 2-3: Roll (int16, degrees, rounded)

## Startup Sequence

1. **I2C bus init**
2. **MPU9250 DMP init**
3. **Gyro bias calibration** (~1 second, keep sensor still)
4. **DMP pre-convergence** (8 seconds, keep sensor still)
5. **Zero-reference capture** (sensor orientation at this point becomes "zero")
6. **CAN init**
7. **Main loop** begins

## Build/Flash/Monitor

```bash
../../scripts/build.sh
../../scripts/flash.sh
python ../../scripts/monitor.py -t <timeout>
```

Or using the activated aliases (after `source ../../activate_scripts.sh`):

```bash
build
flash
monitor
```
