# MicroMouse Firmware — NRW 8.0 (Tunisia IEEE INSAT)

PlatformIO project for ESP32 DevKit V1 + custom hardware.

Full BOM, wiring, and measured parameters: see
[`HARDWARE_SPECS.md`](../HARDWARE_SPECS.md). Simulator and project
overview: see [`README.md`](../README.md).

- 2× N20 12V 1000RPM with quadrature encoders (TB6612FNG driver)
- 3× HC-SR04 ultrasonic sensors (left, front, right)
- 1× MPU6050 IMU (I2C)
- 1× Buzzer, 4× LEDs, 1× Button (BOOT)

## Build & Upload

```bash
cd firmware
pio run -t upload
```

## Configuration

Edit `include/config.h` to match your physical robot after building:

| Parameter                | Where to measure                                      |
| ------------------------ | ----------------------------------------------------- |
| `MACHINE_GEAR_RATIO`     | Motor label (e.g., 30:1, 50:1)                        |
| `MACHINE_ENCODER_CPR`    | Encoder datasheet or measure (3 PPR × 4 = 12 typical) |
| `MACHINE_WHEEL_DIAMETER` | Caliper on wheel (Pololu N20 wheel ≈ 32 mm)           |
| `MACHINE_TRACK`          | Distance between wheel contact patches (caliper)      |
| `MACHINE_TAIL_LENGTH`    | Front axle to front HC-SR04 face                      |
| Pin assignments          | Your wiring diagram                                   |

## Pin Map (default, change in config.h)

| Function           | GPIO |
| ------------------ | ---- |
| HC-SR04 Left Trig  | 13   |
| HC-SR04 Left Echo  | 14   |
| HC-SR04 Front Trig | 27   |
| HC-SR04 Front Echo | 26   |
| HC-SR04 Right Trig | 33   |
| HC-SR04 Right Echo | 34   |
| Motor L IN1        | 25   |
| Motor L IN2        | 32   |
| Motor L PWM        | 18   |
| Motor R IN1        | 19   |
| Motor R IN2        | 23   |
| Motor R PWM        | 5    |
| Encoder L A        | 36   |
| Encoder L B        | 39   |
| Encoder R A        | 22   |
| Encoder R B        | 21   |
| MPU6050 SDA        | 21   |
| MPU6050 SCL        | 22   |
| Battery ADC        | 35   |
| Buzzer             | 4    |
| LED 0 (onboard)    | 2    |
| LED 1              | 5    |
| LED 2              | 18   |
| LED 3              | 19   |
| BOOT Button        | 0    |

## Competition Workflow

1. **Power on** → IMU calibrates (keep robot still), buzzer plays BOOT tone
2. **Short press BOOT** → Start maze exploration (flood-fill search)
3. **Long press BOOT (≥1s)** → Start fast run (shortest path with slalom)
4. During fast run: robot returns to start after reaching center, ready for next run

## Slalom Toggle

In `config.h`:

```cpp
#define USE_SLALOM_TURNS 1   // 1 = smooth slalom, 0 = in-place pivot turns
```

If slalom is unstable, flip to `0` and recompile.

## Persistence

- Maze map + wall calibrations saved to **LittleFS** (survives power cycle)
- Automatic backup after each search run
- Manual restore via UI (not implemented in minimal UI — use serial if needed)

## Libraries (bundled in lib/)

- **MazeLib** — based on `https://github.com/kerikun11/micromouse-maze-library` (MIT)
- **MouseControl** — based on `https://github.com/kerikun11/micromouse-control-module` (MIT)

Both are C++17, PlatformIO-native. License credit is kept inline in the
library sources.

## Troubleshooting

- **Build fails**: Ensure PlatformIO Core/IDE is installed and ESP32 platform is added
- **Upload fails**: Hold BOOT button while plugging USB, or check COM port
- **IMU not detected**: Check I2C wiring (SDA=21, SCL=22), 4.7k pullups on DevKit V1 are onboard
- **Encoder counts wrong**: Verify `MACHINE_ENCODER_CPR` and quadrature wiring (A/B not swapped)
- **HC-SR04 noisy**: Add 100nF capacitor across VCC/GND at sensor, 1k series resistor on ECHO line

## License

MIT
