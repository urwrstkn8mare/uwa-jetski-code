# UWA Jetski Code

This is the code for both the new and old hydrfoil jetski at UWA.

## TODO

- [ ] make status_ui auto horizontal scrolling when text exceeds width of displays.
- [ ] impl CAN bulk data logging via the display projects (tab5 and ws_lcd). this would be to a SD card via a CSV file maybe (unless better alternative).
- [ ] no more rudder componenet as rudder is now handled by a CAN-enabled encoder (dashboard and main controller should just read off CAN directly for this).
- [ ] make aux controller take 2 potentiometers as inputs representing a joystick (a joystick compoennt). left and right should control bank (via elevons) and up and down should control pitch (via elevons as well). while this should essentially be disabled when control task takes over (i.e armed) there should a be an option via the webui to disable the height part of the control loop (so joystick would be used to control the pitch control loop.)
- [ ] make all of 'our' components have consisent configuration exposure (i.e. in kconfig they should all be toplevel menu items starting with UWA Jetski - ...). <- this requires Kconfig.projbuild not just Kconfig.
- [ ] add control loop performance monitoring (i.e. how long each iteration takes (min/avg/max) and on average how many interations per second (min/avg/max) in Hz) - this should show on status_ui and dashboard_ui (via status_ui part of dashboard_ui).

## Directories explained

- `shared_components`: Components shared across firmware projects. Each firmware’s top-level `CMakeLists.txt` adds this path with `list(APPEND EXTRA_COMPONENT_DIRS ".../shared_components")` so **`main`’s transitive `REQUIRES` (with `MINIMAL_BUILD`)** pulls only what’s needed—no symlink farm under `projects/*/components`.

- `projects`: A bunch of ESP-IDF projects that each run on a specfic ESP32
with a specifc purpose. Can simply look at the project's README to learn more.

- `scripts`: Contains utility scripts for development. Run these directly or via the aliases provided by `activate_scripts.sh`:
  - `build.py`: Wrapper for `idf.py build`.
  - `save_defconfig.py`: Wrapper for `idf.py save-defconfig`.
  - `clean.py`: Removes build artifacts.
  - `flash.py`: Wrapper for `idf.py flash` but uses serial port defined in `.serial_port`.
  - `monitor.py`: PTY wrapper for `idf.py monitor` but uses serial port defined in `.serial_port`.
  - `choose_port.py`: Interactive serial port selector that saves choice to a project's `.serial_port` (also can be non-interactive).
  - `setup_clangd.py`: Sets up clangd (LSP) support. Runs `idf.py reconfigure` with the clang toolchain into `build.clang` for each project, then merges all `compile_commands.json` files into the repo root. **No compilation is performed** — this is purely for IDE language-server support. Run this instead of a full clang build whenever you need to refresh clangd after adding/removing components or changing Kconfig. See [clangd setup](#clangd-setup) below.
  - `activate_scripts.sh`: Source this script (do NOT run it directly) to add the `scripts` directory to your PATH and create convenient aliases. Use `source ./activate_scripts.sh` from repo root. After this, you can use `build`, `save_defconfig`, `clean`, `flash`, `monitor`, `choose_port`, and `setup_clangd` directly without the `.py` extension.
  - Shell tab-completion is installed by `activate_scripts.sh` for all script commands/aliases, including dynamic completion for `-p/--project` values.
  - All scripts support `-p/--project` with a project name. For scripts that support multiple projects (`build`, `save_defconfig`, `clean`, `flash`, `setup_clangd`), pass multiple projects with repeated flags or comma-separated values.
  - Script target selection rules:
    - From repo root (this directory): scripts that support multiple projects run for **all** projects when `-p/--project` is omitted.
    - From a project root (`projects/<name>`): scripts run for that project when `-p/--project` is omitted.
    - Otherwise: scripts error unless `-p/--project` is provided.
    - `monitor` and `choose_port` are limited to one project at a time.
  - `choose_port.py` uses `-P/--port` for explicitly selecting a serial port (because `-p/--project` is reserved for project selection).
  - Scripts discover project names dynamically from directories under `projects` that contain `CMakeLists.txt`.
  - In `monitor.py`, `Ctrl+C` now exits monitor/script immediately, while other key input is passed through normally.

## clangd setup

clangd requires a `compile_commands.json` built with the clang toolchain (so the include paths and flags match what clang actually understands). The normal `build` script produces a GCC-based binary for flashing — it does not update the clangd database.

**Why a separate step is needed:** clangd, when opened at the repo root, reads the repo-root `compile_commands.json`. This file is a merged view of all projects' `build.clang/compile_commands.json` files. It is only regenerated by `setup_clangd` (or previously by `build --clang`). Without this merge, clangd uses a stale database and cannot resolve Kconfig macros (like `CONFIG_*` from `sdkconfig.h`) — even though `sdkconfig.h` itself exists after a plain `reconfigure`, the paths it reads are wrong.

**Initial setup or after adding/removing components:**

```sh
source ./activate_scripts.sh   # if not already done
setup_clangd                   # reconfigures all projects and merges compile_commands.json
```

Or for a single project:

```sh
setup_clangd -p main_controller
```

**When to re-run `setup_clangd`:**

- After `clean` that removed `build.clang` directories
- After adding or removing components (changes to `CMakeLists.txt` or `idf_component.yml`)
- After changing Kconfig values that affect which components are included

**clangd config:** The repo-root `.clangd` points `CompilationDatabase` to `.` (the merged `compile_commands.json`), so all files — project sources and `shared_components` — are served from that single database. The `setup_clangd` script is what keeps it up to date.

## Development Guidelines

These are more requirements than guidelines but 🤷‍♂️.

- Use scripts from the `scripts` directory instead of `idf.py` when possible
  - Run from repo root to target all projects, or from project root to target just that project.
  - Use `-p/--project` when running from other directories or when selecting specific projects.
- Always modify `sdkconfig` - not `sdkcofnig.defaults`. Run the `save_defconfig` script before committing as `sdkconfig` is gitignored.
- Do not modify `managed_components` as this is autogenerated.
- If a component is to be used in more than 1 project, move to `shared_components`.
