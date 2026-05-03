# Main controller (T-Display-S3)

## Overview

ESP32-S3 board (LilyGO T-Display-S3) that:

- Reads **pitch/roll** from an **ICM-20948** (I2C + DMP, Quat9 orientation) and sends `CAN_ID_ATTITUDE` (0x100).
- Reads **ride height** from the **A02YYUW** ultrasonic module (UART) and sends `CAN_ID_HEIGHT` (0x101).
- **Rudder position** is the **POT %** from the auxiliary controller on CAN (`CAN_ID_POTENTIOMETER`, 0x102). That demand drives **two mirrored 50 Hz LEDC outputs** and **`CAN_ID_SERVO_POS` (0x103)** carries echoed **surface** angles (e.g. elevons — not rudder; rudder on the HUD uses POT).
- Receives **GPS** snapshots from the **auxiliary controller** (`CAN_ID_GPS_POSITION` 0x104, `CAN_ID_GPS_VELOCITY` 0x105) for internal state/can integration.
- Shows a compact **debug LVGL** screen with only local essentials (link status, pitch/roll, height, rudder demand) on the built-in display.

## Kconfig (Menuconfig)

- **CAN** — `CANTX`, `CANRX`, bitrate (`shared_components/can/Kconfig.projbuild`).
- **Height (A02YYUW)** — UART port, RX, TX (`components/height/Kconfig.projbuild`).
- **ICM-20948** — I2C port, SDA, SCL, 0x68 vs 0x69 (`components/imu/Kconfig.projbuild`; top-level menu in `menuconfig`, not under *Component config*).
- **PWM outputs** — GPIO per channel, LEDC timer and channels (`components/servo_drive/Kconfig.projbuild`; menu *UWA Jetski — Rudder PWM*, top-level).

## Components

- `components/imu` — `cybergear-robotics/icm20948` (registry) + I2C driver. **Attitude is from the DMP only** (sensor `INV_ICM20948_SENSOR_ORIENTATION`, Quat9 from `inv_icm20948_read_dmp_data`), not from raw register reads.
- `components/height` — A02YYUW driver (git dep unchanged).
- `components/can` — symlink to `shared_components/can` (TWAI on-chip).
- `main/idf_component.yml` — pulls **icm20948** and upstream **tdisplays3** from GitHub (`hiruna/esp-idf-t-display-s3`, `components/tdisplays3`).

## Build

From the repository root (see top-level `README.md` for scripts):

```bash
source ./activate_scripts.sh
cd projects/main_controller
build
```

## Note

The managed `icm20948` package still compiles an SPI file that triggers a pointer-type warning on ESP-IDF 5.5; the project CMake turns that into a non-fatal warning. Only the I2C path is used.
