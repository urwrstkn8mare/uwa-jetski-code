# Main controller (T-Display-S3)

## Overview

ESP32-S3 on the LilyGO **T-Display-S3**:

- **Debug LVGL** strip (link, IMU, height, servo, rudder demand from CAN).
- **Two 50 Hz LEDC servo outputs** driven by **pot %** from the auxiliary controller on **CAN 0x102**; **0x103** echoes commanded surface angles.
- **CAN TWAI** shared with the rest of the vehicle (attitude **0x100**, height **0x101**, etc.).
- **ICM-20948** (I2C / DMP attitude) and **A02YYUW** ultrasonic height (UART).

Failures to bring up CAN / IMU / height are **non-fatal**; the panel shows what is working.

## Components

- `components/app_state` — subsystem flags + rudder POT from CAN (mutex).
- `components/main_panel_ui` — `tdisplays3` + LVGL status UI.
- `components/servo_drive`, `components/can`, `components/imu`, `components/height` — hardware drivers.
- `main/idf_component.yml` — pulls **icm20948** and **tdisplays3** from `urwrstkn8mare/esp-idf-t-display-s3` (`components/tdisplays3`).

## Build

From the repository root (top-level `README.md` — `source ./activate_scripts.sh`):

```bash
cd projects/main_controller
build
```

## Note

The managed **icm20948** package triggers a pointer-type warning under ESP-IDF 5.5; project CMake lowers it to non-fatal. Only the I2C path is used.
