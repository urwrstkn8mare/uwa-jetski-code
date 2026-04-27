# Main Controller

## Overview

ESP32-based main controller that reads attitude (pitch/roll) from an MPU9250 IMU via I2C, reads ride height from an A02YYUW ultrasonic sensor via UART, and broadcasts both over CAN bus.

## Hardware

- **MCU**: ESP32
- **IMU**: MPU9250 (Grove IMU 10DOF v2.0)
  - **I2C Pins**: SCL=33, SDA=32
  - **I2C Address**: 0x68
- **Height Sensor**: A02YYUW (or A0221AU) ultrasonic distance sensor
  - **UART Port**: configurable (default UART_NUM_2)
  - **RX Pin**: configurable (default GPIO 13)
  - **TX Pin**: configurable (default GPIO 14)
  - **Baud Rate**: 9600
  - **Trigger**: sends `0xFF` trigger byte to support both auto-output and controlled sensor variants
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

### Height Component (`components/height`)

- Initializes UART for the A02YYUW sensor
- Sends a `0xFF` trigger byte on TX to support both auto-output and controlled sensor variants
- Runs a background FreeRTOS task that parses 4-byte sensor frames
- Validates frame checksum before accepting a reading
- Stores the latest height (in centimetres) in a mutex-protected buffer
- Configurable UART port, RX pin and TX pin via `menuconfig`

### CAN Component (`shared_components/can`)

- Uses the ESP-IDF TWAI driver (`esp_driver_twai`)
- Configured for standard 11-bit IDs at 200 kbps
- Provides `can_init()` and `can_tx()` APIs

### Main Application (`main/main.c`)

- Initializes IMU, height sensor and CAN
- Reads pitch/roll from the IMU component every 100 ms
- Reads height from the height sensor every 100 ms
- Transmits CAN frames:
  - ID `0x100` – Attitude:
    - Bytes 0-1: Pitch (int16, degrees, rounded)
    - Bytes 2-3: Roll (int16, degrees, rounded)
  - ID `0x101` – Height:
    - Bytes 0-1: Height (uint16, centimetres)

## Startup Sequence

1. **I2C bus init**
2. **MPU9250 DMP init**
3. **Gyro bias calibration** (~1 second, keep sensor still)
4. **DMP pre-convergence** (8 seconds, keep sensor still)
5. **Zero-reference capture** (sensor orientation at this point becomes "zero")
6. **Height sensor UART init**
7. **CAN init**
8. **Main loop** begins

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
