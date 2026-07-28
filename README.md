# bike-sense

Firmware for a bike-mounted sensor built on the **Seeed XIAO nRF54L15**.

## Objective

The only objective of this project is to **collect accelerometer and gyroscope data from a
bicycle** and use it to:

1. **Figure out ride patterns** — how the bike is actually ridden: road surface, cadence,
   cornering and lean, braking events, impacts and potholes, idle vs. moving.
2. **Do predictive maintenance** — infer wear and developing faults from how the vibration and
   rotation signature of the bike drifts over time, so a component can be serviced before it
   fails rather than after.

Everything in this repository exists to serve that: get clean, correctly scaled, correctly timed
6-axis motion data off the bike. Nothing else is in scope.

## Hardware

| Item | Detail |
|---|---|
| Board | Seeed XIAO nRF54L15 (`xiao_nrf54l15/nrf54l15/cpuapp`) |
| IMU | On-board LSM6DS3TR-C, 6-axis accel + gyro |

## Current stage: `p1_imu_bringup`

Streams live accel + gyro samples to the serial console as CSV.

| Parameter | Value | Why |
|---|---|---|
| Hardware ODR | 104 Hz | Set at runtime — the driver powers the sensor down when the ODR Kconfigs are left at 0 |
| Sample / print rate | 50 Hz | Half the ODR, so every read returns a fresh sample and the stream fits the 115200 budget (~3.8 kB/s of ~11.5 kB/s) |
| Accel full scale | ±4 g | Covers road vibration and pothole impacts without clipping |
| Gyro full scale | ±500 dps | Covers lean and steering rates with headroom |
| Gyro bias | 200-sample average at startup | The driver applies only a sensitivity scale and has no offset support, so a stationary gyro reads a small nonzero rate |

**Hold the board still for the first ~4 seconds after power-on.** That is the gyro zero-rate
calibration. If any axis moves more than 0.02 rad/s peak-to-peak during it, the firmware logs a
warning that the bias is contaminated — power-cycle while still to redo it.

Sampling uses absolute deadlines rather than a fixed sleep, so print time does not accumulate as
drift in the sample interval. If the console applies backpressure the loop resyncs instead of
spinning to catch up.

### Data format

```
# XIAO nRF54L15 IMU stream: accel m/s^2 (+/-4 g), gyro rad/s (+/-500 dps, bias-corrected), ODR 104 Hz, sampled 50 Hz
timestamp_ms,ax,ay,az,gx,gy,gz,die_temp_c
```

| Column | Unit | Notes |
|---|---|---|
| `timestamp_ms` | ms | Uptime since boot |
| `ax`, `ay`, `az` | m/s² | 3 decimal places |
| `gx`, `gy`, `gz` | rad/s | 4 decimal places, **bias-corrected** |
| `die_temp_c` | °C | IMU die temperature; present only while `CONFIG_LSM6DSL_ENABLE_TEMP=y` |

A status line (`N samples streamed, M read errors`) is logged every 10 seconds.

## Build and flash

```sh
west build -b xiao_nrf54l15/nrf54l15/cpuapp -p
west flash
```

Then open a serial terminal on the board's USB serial port at **115200** to capture the CSV.

Measured footprint of the current firmware: **59,044 B flash** of 1428 KB (4.0 %) and
**10,624 B RAM** of 188 KB (5.5 %).

## Deliberately out of scope, for now

These are excluded on purpose, not pending oversight:

- **No flash partitioning.** The app links across the whole 1428 KB region; the partition table
  in the board devicetree is inert because `CONFIG_USE_DT_CODE_PARTITION` is not set.
- **No on-device storage.** No NVS, no LittleFS, no filesystem, and the flash driver is not even
  enabled. Data leaves the board over serial only.
- **No TF-M and no cryptography.** The build targets `cpuapp`, not `cpuapp/ns`, so there is no
  secure/non-secure split and nothing encrypts or authenticates stored data.
- **No changes to any SDK or driver source.** All configuration is done from the application:
  full-scale ranges and ODR are set at runtime through the public `sensor_attr_set()` API, and
  gyro bias correction lives in the app because the LSM6DSL driver has no offset support.

## Layout

```
p1_imu_bringup/
├── CMakeLists.txt
├── prj.conf          # IMU + sensor + logging config; ODR/FS left at 0 so the app owns them
└── src/main.c        # configure -> calibrate gyro bias -> stream CSV at 50 Hz
```
