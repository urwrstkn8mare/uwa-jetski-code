# Main controller (T-Display-S3)

## Overview

ESP32-S3 on the LilyGO **T-Display-S3**:

- **Debug LVGL** text (link, IMU, height, servo, rudder demand from CAN) assembled by **`lvgl_status_display`** from per-subsystem callbacks; hardware components expose compact `*_status_line_write` helpers in `main`.
- **Two 50 Hz LEDC servo outputs** driven by **pot %** from the auxiliary controller on **CAN 0x102**; **0x103** echoes commanded surface angles.
- **CAN TWAI** shared with the rest of the vehicle (attitude **0x100**, height **0x101**, etc.) — **`shared_components/can`** wired via `EXTRA_COMPONENT_DIRS`.
- **ICM-20948** (I2C / DMP attitude) and **A02YYUW** ultrasonic height (UART).

Failures to bring up CAN / IMU / height are **non-fatal**; the panel shows what is working.

## Components

- **`main`** — pulls in **`lvgl_status_display`**, **`tdisplays3`**, wires status providers and supervisor loop.
- **`components/app_state`** — subsystem flags + rudder POT from CAN (mutex).
- **`components/servo_drive`**, **`components/imu`**, **`components/height`** — hardware drivers; each exposes a one-line **`_*_status_line_write`** for debug text.
- **`main/idf_component.yml`** — **tdisplays3** BSP (and LVGL/Button helpers).

## Shared code

TWAI **`can`** driver lives under **`shared_components/can/`** (`EXTRA_COMPONENT_DIRS` in this project’s root `CMakeLists.txt`).

## Build

From the repository root (top-level `README.md` — `source ./activate_scripts.sh`):

```bash
cd projects/main_controller
build
```

## Note

The managed **icm20948** package triggers a pointer-type warning under ESP-IDF 5.5; project CMake lowers it to non-fatal. Only the I2C path is used.
