# WS LCD hydrofoil dashboard

Waveshare LCD target for the hydrofoil dashboard UI.

**Feed selection** is via `menuconfig` → *Dashboard feed*:

- **CAN**: TWAI RX directly patches `dashboard_ui` widgets (throttled).
- **DEMO**: an LVGL timer runs `dashboard_demo_update_ui()`.

The bottom strip uses `lvgl_status_display` to show either CAN status or the demo rate.
