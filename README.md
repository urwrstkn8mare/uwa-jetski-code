# UWA Jetski Code

This is the code for both the new and old hydrofoil jetski at UWA.

## TODO

- [x] Make `status_ui` auto horizontal-scroll when text exceeds the display width.
- [ ] Implement CAN bulk data logging via the display projects (`tab5` and `ws_lcd`) to an SD card (CSV or better alternative).
- [x] Remove the rudder component — rudder is now handled by a CAN-enabled encoder, so the dashboard and main controller should read directly from CAN.
- [x] Enable reading of rudder angle from CAN-enabled Briterencoder absolute rotary encoder. Encoder is configured in auto-return mode at startup; dashboards read encoder frames directly from the CAN bus.
- [x] Make aux controller accept two potentiometers as a joystick input: left/right controls bank (via elevons), up/down controls pitch (via elevons). This should be disabled when the control task takes over (i.e. armed), with a webui option to disable the height part of the control loop so the joystick controls the pitch loop directly.
- [x] Make all custom components have consistent Kconfig exposure: each should be a top-level menu item prefixed with `UWA Jetski - ...`. This requires `Kconfig.projbuild`, not just `Kconfig`.
- [x] Add control loop performance monitoring (iteration time min/avg/max, and iterations/sec in Hz). Tracked separately for armed (full PID) and disarmed (joystick) loops. Appears in `status_ui` and `dashboard_ui`.
- [x] Refactor `ctrl_task` to a single loop with separate armed/disarmed perf tracking. Armed and disarmed loops use independent `perf_stats_t` accumulators that never reset on transition.
- [x] Show PID pitch and roll setpoints on the attitude display so the desired vs actual attitude can be compared. Setpoints are merged into `CAN_ID_CTRL_STATUS` and shown as an orange ghost aircraft pointer (flight-director style) offset from the actual pointer by the pitch/roll error — overlaps when on setpoint.
- [ ] Remove `dashboard_demo` and replace it with a unified dummy-data injection mechanism at the project level: `dashboard_ui` should only consume CAN data, and projects that need test/demo data push it via the same CAN path. This ensures future data-logging work can share the same dummy-data path without a separate demo codepath in the UI component.

## Directories

- `shared_components`: Components shared across firmware projects. Each firmware's top-level `CMakeLists.txt` adds this path with `list(APPEND EXTRA_COMPONENT_DIRS ".../shared_components")` so **`main`'s transitive `REQUIRES` (with `MINIMAL_BUILD`)** pulls only what's needed — no symlink farm under `projects/*/components`.

- `projects`: ESP-IDF projects, each targeting a specific ESP32 with a specific purpose. See each project's README for details.

- `scripts`: Utility scripts for development. Run directly or via the aliases provided by `activate_scripts.sh`:
  - `build.py`: Wrapper for `idf.py build`.
  - `save_defconfig.py`: Wrapper for `idf.py save-defconfig`.
  - `clean.py`: Removes build artifacts.
  - `flash.py`: Wrapper for `idf.py flash`, using the serial port defined in `.serial_port`.
  - `monitor.py`: PTY wrapper for `idf.py monitor`, using the serial port defined in `.serial_port`.
  - `choose_port.py`: Interactive serial port selector that saves the choice to a project's `.serial_port` (also supports non-interactive use).
  - `setup_clangd.py`: Sets up clangd (LSP) support. Runs `idf.py reconfigure` with the clang toolchain into `build.clang` for each project, then merges all `compile_commands.json` files into the repo root. No compilation is performed — purely for IDE language-server support.
  - `activate_scripts.sh`: Source this script (do **not** run it directly) to add the `scripts` directory to your `PATH` and create convenient aliases. Use `source ./activate_scripts.sh` from the repo root. After sourcing, you can use `build`, `save_defconfig`, `clean`, `flash`, `monitor`, `choose_port`, and `setup_clangd` directly without the `.py` extension.
  - Shell tab-completion is installed by `activate_scripts.sh` for all script commands/aliases, including dynamic completion for `-p/--project` values.
  - All scripts support `-p/--project` with a project name. Scripts that support multiple projects (`build`, `save_defconfig`, `clean`, `flash`, `setup_clangd`) accept repeated flags or comma-separated values.
  - Script target selection rules:
    - From repo root: multi-project scripts run for **all** projects when `-p/--project` is omitted.
    - From a project root (`projects/<name>`): scripts run for that project when `-p/--project` is omitted.
    - Otherwise: scripts error unless `-p/--project` is provided.
    - `monitor` and `choose_port` are limited to one project at a time.
  - `choose_port.py` uses `-P/--port` to specify a serial port (`-p` is reserved for project selection).
  - Scripts discover project names dynamically from directories under `projects` that contain `CMakeLists.txt`.
  - In `monitor.py`, `Ctrl+C` exits immediately; all other key input is passed through normally.

## Development Guidelines

These are more requirements than guidelines but 🤷‍♂️.

- Use scripts from the `scripts` directory instead of `idf.py` where possible.
  - Run from repo root to target all projects, or from a project root to target just that project.
  - Use `-p/--project` when running from other directories or to select specific projects.
- Always modify `sdkconfig`, not `sdkconfig.defaults`. Run `save_defconfig` before committing — `sdkconfig` is gitignored.
- Do not modify `managed_components`; it is autogenerated.
- If a component is used in more than one project, move it to `shared_components`.
- For editor/IDE support (clangd LSP), run `setup_clangd` after initial setup and again after adding/removing components or running `clean`. This reconfigures all projects with the clang toolchain and merges `compile_commands.json` files into the repo root — no compilation involved.
- No legacy code or backwards-compatibility shims. When replacing functionality, delete the old code entirely — do not rename unused variables, re-export removed types, or leave commented-out blocks. The goal is a final codebase that is simple, readable, maintainable, and modular; sweeping changes are preferred over incremental patches that leave dead weight behind.
- Do not add features, abstractions, or error handling beyond what the task requires. Three similar lines is better than a premature abstraction. Only validate at system boundaries (user input, external APIs, CAN frames) — trust internal code and framework guarantees.
