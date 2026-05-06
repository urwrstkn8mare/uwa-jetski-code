# WS LCD hydrofoil dashboard

Waveshare LCD target for the hydrofoil dashboard UI.

**Feed selection** is via `menuconfig` → *Dashboard feed*:

- **CAN**: TWAI RX directly patches `dashboard_ui` widgets (throttled).
- **DEMO**: an LVGL timer runs `dashboard_demo_fill()` and applies the `dashboard_ui_set_*()` setters.

The bottom strip uses `lvgl_status_display` to show either CAN status or the demo rate.
