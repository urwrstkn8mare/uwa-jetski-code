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

- `scripts`: Useful scripts
