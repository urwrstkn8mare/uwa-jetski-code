# Tab5 hydrofoil dashboard

M5Stack Tab5 target for the hydrofoil dashboard UI.

The dashboard feed is selected via `menuconfig` → *Dashboard feed*:

- **CAN**: TWAI RX directly patches `dashboard_ui` widgets (throttled).
- **DEMO**: an LVGL timer runs `dashboard_demo_update_ui()`.

Board bring-up (EXT5V, LVGL/BSP tuning, backlight, unused blocks disabled) lives in **`main/main.c`** next to the dashboard wiring.
