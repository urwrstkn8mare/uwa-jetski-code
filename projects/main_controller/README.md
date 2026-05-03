# Main controller (T-Display-S3)

## Overview

ESP32-S3 board (LilyGO T-Display-S3) that:

- Reads **pitch/roll** from an **ICM-20948** (I2C + DMP, Quat9 orientation) and sends `CAN_ID_ATTITUDE` (0x100).
- Reads **ride height** from the **A02YYUW** ultrasonic module (UART) and sends `CAN_ID_HEIGHT` (0x101).
- Drives **two 50 Hz PWM servos** (LEDC) from the **potentiometer** value received over CAN (`CAN_ID_POTENTIOMETER`, 0x102) and sends `CAN_ID_SERVO_POS` (0x103) with two `int16` degrees.
- Receives **GPS** snapshots from the **auxiliary controller** (`CAN_ID_GPS_POSITION` 0x104, `CAN_ID_GPS_VELOCITY` 0x105) for the on-panel status text.
- Shows a **debug LVGL** screen (local + CAN) on the built-in display (`tdisplays3` from the example repo; see `main/idf_component.yml`).

## Kconfig (Menuconfig)

- **CAN** — `CANTX`, `CANRX`, bitrate (`shared_components/can/Kconfig.projbuild`).
- **Height (A02YYUW)** — UART port, RX, TX (`components/height/Kconfig.projbuild`).
- **ICM-20948** — I2C port, SDA, SCL, 0x68 vs 0x69 (`main/Kconfig.projbuild`).
- **Servos** — GPIO per channel, LEDC timer and channels (`main/Kconfig.projbuild`).

## Components

- `components/imu` — `cybergear-robotics/icm20948` (registry) + legacy I2C driver. **Attitude is from the DMP only** (sensor `INV_ICM20948_SENSOR_ORIENTATION`, Quat9 from `inv_icm20948_read_dmp_data`), not from raw register reads.
- `components/height` — A02YYUW driver (git dep unchanged).
- `components/can` — symlink to `shared_components/can` (TWAI on-chip).
- `main/idf_component.yml` — pulls **icm20948** and **tdisplays3** (git).

## Build

From the repository root (see top-level `README.md` for scripts):

```bash
source ./activate_scripts.sh
cd projects/main_controller
build
```

## Note

The managed `icm20948` package still compiles an SPI file that triggers a pointer-type warning on ESP-IDF 5.5; the project CMake turns that into a non-fatal warning. Only the I2C path is used.
