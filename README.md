# MicroMouse NRW 8.0

A complete micromouse project: an ESP32-based maze-solving robot with
ultrasonic mapping, plus a browser simulator for testing the search
algorithm without hardware.

- **Firmware**: ESP32 DevKit V1, Arduino framework (PlatformIO)
- **Simulator**: standalone HTML/JS web application
- **Hardware specs**: full BOM, wiring, and measured parameters in
  [`HARDWARE_SPECS.md`](HARDWARE_SPECS.md)

## Repository Layout

```
firmware/          PlatformIO firmware project (ESP32)
simulator/         Browser-based maze simulator (single web page)
HARDWARE_SPECS.md  BOM, wiring, electrical and mechanical parameters
```

## Features

- Flood-fill search with trapezoidal-acceleration-aware step mapping
  (`MazeLib`)
- Trajectory control module (`MouseControl`): straight, slalom, and PID
  feedback with feedforward
- Wall detection with 3× HC-SR04 ultrasonic sensors
- IMU heading hold (MPU6050) and encoder odometry
- Battery monitoring and LittleFS maze persistence
- Web simulator mirrors the firmware parameters so tuning can be done
  first in the browser

## Hardware Overview

| Component | Model | Qty |
|-----------|-------|-----|
| Microcontroller | ESP32 DevKit V1 (CP2102) | 1 |
| Motors | N20 12V 1000 RPM + quadrature encoder | 2 |
| Motor driver | TB6612FNG (Pololu #713) | 1 |
| Ultrasonic | HC-SR04 | 3 |
| IMU | GY-521 (MPU6050) | 1 |
| Buck converter | XL4015 5A | 1 |
| Battery | 3S LiPo (11.1V) | 1 |

See [`HARDWARE_SPECS.md`](HARDWARE_SPECS.md) for the full BOM, wiring
diagram, pin map, and measurement procedures.

## Firmware

### Prerequisites

- [PlatformIO Core](https://platformio.org/install)
- ESP32 Arduino platform (auto-installed on first build)

### Build, Upload, Monitor

```bash
cd firmware
pio run                 # compile
pio run -t upload       # compile + flash (SELECT the COM port first, see below)
pio device monitor      # serial console
```

### Uploading

On Windows, the ESP32 usually appears as a COM port (CP210x or CH340).
If the board is not detected:

1. Use a USB **data** cable (not charge-only).
2. Install the CP210x / CH340 driver matching your board.
3. Press and hold `BOOT`, plug in USB, then release — the board enters
   bootloader mode.
4. Select the new COM port in PlatformIO before uploading.

### Configuration

All robot-specific parameters live in `include/config.h`. Adjust them to
match your physical build:

| Parameter | Where to measure |
|-----------|------------------|
| `MACHINE_GEAR_RATIO` | Motor label (e.g., 30:1) |
| `MACHINE_ENCODER_CPR` | Encoder spec (counts per output-shaft revolution) |
| `MACHINE_WHEEL_DIAMETER` | Caliper on the wheel |
| `MACHINE_TRACK` | Distance between wheel contact patches |
| `MACHINE_TAIL_LENGTH` | Drive axle to front HC-SR04 face |
| `USE_SLALOM_TURNS` | 1 = smooth slalom, 0 = in-place pivot turns |

A full reference of the constants is in the *Firmware Configuration*
section of [`HARDWARE_SPECS.md`](HARDWARE_SPECS.md).

### Operation

1. Power on -> IMU calibrates (keep the robot still).
2. Short press `BOOT` -> start exploration.
3. Long press `BOOT` (>= 1 s) -> fast run along the shortest path.

## Simulator

The simulator runs the same search and wall-following logic in a
browser, using the parameters from `engine.js` (which mirror
`config.h`). It runs entirely client-side in a Web Worker — no server
required.

```bash
cd simulator
python -m http.server 8000
# open http://localhost:8000
```

On Windows without Python, any static file server works (e.g. VS Code
Live Server).

## License

MIT

- Components `firmware/lib/MazeLib` and `firmware/lib/MouseControl` are
  derived from kerikun11's open-source micromouse libraries (MIT).
  See the credits in the library source files.