# MPU9250 IMU - Main Controller

## Overview

MPU9250 IMU (Grove IMU 10DOF v2.0) connected via I2C, computing pitch, roll, and yaw using a 9-axis Madgwick filter with magnetometer calibration.

## Hardware

- **Sensor**: Seeed Studio Grove IMU 10DOF v2.0 (MPU-9250 + BMP280)
- **I2C Pins**: SCL=39, SDA=38
- **I2C Address**: 0x68 (MPU9250), 0x0C (AK8963 magnetometer)

## Magnetometer Calibration

Calibration is required once per physical installation to compensate for hard-iron (permanent magnetic offsets) and soft-iron (elliptical distortion) effects.

### How to Calibrate

1. Set `MAG_OFFSET_X`, `MAG_OFFSET_Y`, `MAG_OFFSET_Z` to `0.0f` in `main/main.cpp`
2. Build and flash
3. On boot, the calibration sequence runs:
   - **5s countdown** - Get ready
   - **15s rotation phase** - Rotate the sensor slowly in all directions (figure-8, flip, spin)
   - **Halts** - Copy the calibration results from the serial output
4. Update the constants in `main/main.cpp` with the printed values
5. Reflash - calibration will be skipped on subsequent boots

### Calibration Constants

Current calibrated values (as of last calibration):

```cpp
static const float MAG_OFFSET_X = -17.96f;
static const float MAG_OFFSET_Y = -14.92f;
static const float MAG_OFFSET_Z = -0.09f;
static const float MAG_SCALE_X = 0.972f;
static const float MAG_SCALE_Y = 1.003f;
static const float MAG_SCALE_Z = 1.026f;
```

### When to Recalibrate

- Sensor is moved to a new physical location or mount
- Nearby metal objects or electronics are added/removed
- Yaw drift becomes excessive (>1°/s when stationary)

## Concerns and Recommendations

### Magnetometer Reliability in Electromagnetically Noisy Environments

The magnetometer (AK8963) is used to prevent yaw drift by referencing Earth's magnetic field. However, **it is fundamentally unreliable in environments with electromagnetic interference** for the following reasons:

1. **Electromagnetic interference from motors and electrical systems**: Any wire carrying current generates a magnetic field proportional to the current. Motors, solenoids, ignition coils, high-current cables, and speakers all produce significant stray magnetic fields that distort magnetometer readings ([Pacific Yacht Systems](https://www.pysystems.com/how-to/articles/electromagnetic-fields-on-your-boat/), [boats.com](https://www.boats.com/how-to/magnetic-interference-explained/)).

2. **Ferrous materials**: Steel components, stainless steel fasteners, welded joints, and structural metal can become magnetized over time, creating permanent hard-iron offsets that vary with vibration and temperature ([CPT Autopilot](https://www.cptautopilot.com/magnetic_interference_check.php)).

3. **Dynamic interference**: Unlike static calibration offsets, motor-generated magnetic fields change with RPM, load, and electrical activity. These cannot be calibrated out. Professional marine magnetometers are towed far behind vessels specifically to avoid the host platform's magnetic field ([WHOI](https://www.whoi.edu/what-we-do/explore/instruments/instruments-sensors-samplers/marine-magnetometer/)).

4. **Madgwick filter behavior**: While the Madgwick 9-axis implementation is designed to limit magnetic disturbance effects primarily to yaw, strong magnetic perturbations can still temporarily corrupt roll and pitch estimates until the disturbance is removed ([Valenti et al., 2015](https://www.mdpi.com/1424-8220/15/8/19302)).

### Recommendation: 6-Axis Mode for Production

For applications where **pitch and roll are the primary concern** and yaw is secondary:

- **Pitch and roll are computed from the accelerometer (gravity vector) and gyroscope** — the magnetometer contributes nothing to these angles
- Running the Madgwick filter in 6-axis mode (accelerometer + gyroscope only) eliminates all magnetic interference risk
- Yaw will drift over time from gyro integration, but this is acceptable when yaw is not critical
- If heading is needed, GPS course-over-ground provides reliable yaw when the platform is moving

To switch to 6-axis mode, pass `0, 0, 0` for magnetometer values to `madgwick.update()` and set `mag_enabled = 0` in the MPU config.

## Filter Comparison

| Filter | Pitch/Roll Accuracy | Yaw Accuracy | CPU (ESP32) | Complexity | Notes |
|--------|-------------------|-------------|-------------|------------|-------|
| **Madgwick** (current) | Excellent | Good (with mag) | ~160μs | Low | Best overall 9-axis accuracy; gradient descent approach |
| **Mahony** | Very Good | Good (with mag) | ~120μs | Low | PI controller; better gyro bias compensation; used in flight controllers (Betaflight, ArduPilot) |
| **Complementary** | Good | N/A (6-axis) | ~10μs | Lowest | Simple weighted average; no mag needed; stable for pitch/roll only |
| **Kalman (EKF)** | Best | Best | High | High | Optimal but complex to tune; overkill for simple pitch/roll |

**Why Madgwick was chosen**: Best balance of accuracy and simplicity for 9-axis fusion. The ESP32-S3 has ample headroom so computational cost is negligible. Research shows Madgwick handles external acceleration effects better than alternatives (Parikh et al., 2021).

**When to consider alternatives**:
- **Mahony**: If faster response to dynamic movements is needed or if gyro bias drift is problematic (its integral term handles this better)
- **Complementary**: If only pitch/roll matter and you want minimal code with zero magnetometer dependency (already available in `espp/filters`)
- **Kalman**: Only if maximum accuracy is critical and you have resources for complex tuning

## Output

Serial output at ~1Hz shows:
- Raw accelerometer and magnetometer values
- Normalized accel (g) and mag (uT) values
- Pitch, Roll, Yaw in degrees

## Build/Flash/Monitor

```bash
../../scripts/build.sh
../../scripts/flash.sh
python ../../scripts/monitor.py -t <timeout>
```
