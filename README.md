# UWA Jetski Code

This is the code for both the new and old hydrfoil jetski at UWA.

## Directories explained

- `esp-bsp`: A git submodule of `esp-bsp`'s `bsp/update_tab5` branch. Needed
for a compatible version of the `m5stack_tab5` component until
<https://github.com/espressif/esp-bsp/pull/765> is merged and released.

- `shared_components`: Stores components that may be used in one or more
projects. These must be ***symlinked*** into the projects' `components` directory.

- `projects`: A bunch of ESP-IDF projects that each run on a specfic ESP32
with a specifc purpose. Can simply look at the project's README to learn more.

- `scripts`: Contains utility scripts for development. Run these directly or via the aliases provided by `activate_scripts.sh`:
  - `build.sh`: Wrapper for `idf.py build`. Accepts same arguments as `idf.py build`.
  - `clean.py`: Removes build artifacts. Accepts optional `-p/--project` argument to specify project path (defaults to current directory).
  - `flash.sh`: Wrapper for `idf.py flash` but uses serial port defined in `.serial_port`
  - `monitor.py`: PTY wrapper for `idf.py monitor` but uses serial port defined in `.serial_port`.
  - `choose_port.py`: Interactive serial port selector that saves choice to `.serial_port` of project (also can be non-interactive).
- `activate_scripts.sh`: Source this script (do NOT run it directly!) to add the `scripts` directory to your PATH and create convenient shell aliases. After sourcing, you can use `build`, `clean`, `flash`, `monitor`, and `choose_port` directly without the `.sh` or `.py` extensions. Example: `source ./activate_scripts.sh`
