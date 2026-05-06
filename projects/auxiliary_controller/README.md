# Auxiliary controller

T-Display-S3 that handles **operator inputs** and navigation data: rudder pot (ADC), NEO-6M GPS (NMEA RMC), and CAN to the rest of the vehicle.

Build:

```bash
source ../../activate_scripts.sh
cd projects/auxiliary_controller
build
```

Manifest: `main/idf_component.yml` pulls **tdisplays3** from the `esp-idf-t-display-s3` example repo. Pinout: `menuconfig` → top-level *UWA Jetski — …* entries from the `gps`, `rudder_pot`, and **`shared_components/can`** (`EXTRA_COMPONENT_DIRS` in this project).
