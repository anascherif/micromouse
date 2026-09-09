# Hardware Specifications — MicroMouse NRW 8.0

## Overview
This document captures all hardware specifications, datasheet extracts, and measured values for the IEEE MicroMouse competition robot (National Robotics Week 8.0, INSAT Tunisia).

---

## Bill of Materials (BOM)

| Component | Model | Qty | Notes |
|-----------|-------|-----|-------|
| Microcontroller | ESP32 DevKit V1 (CP2102) | 1 | Arduino framework |
| Motors | N20 12V 1000 RPM + Encoder | 2 | 30:1 gearbox, 6-wire (VCC/GND/A/B/M+/M-) |
| Wheels | Pololu N20 32×7 mm | 2 | 32 mm diameter, 3 mm bore |
| Caster | W420 metal ball | 1 | 42 mm diameter |
| Motor Driver | TB6612FNG (Pololu #713) | 1 | Dual H-bridge, 1.2A cont / 3A peak |
| IMU | GY-521 (MPU6050) | 1 | I²C 0x68, 400 kHz |
| Ultrasonic | HC-SR04 | 3 | Left, Front, Right |
| Buck Converter | XL4015 5A 75W | 1 | 12V → 5V (voltage pot only) |
| Power Switch | KCD11 2-pin | 1 | 10×15 mm |
| Battery | 3S LiPo | 1 | 11.1V nominal, 12.6V max |
| Voltage Divider | 100kΩ / 10kΩ | 1 | 11× division, ADC pin 35 |

---

## Mechanical Parameters

| Parameter | Symbol | Value | Unit | Source |
|-----------|--------|-------|------|--------|
| Gear Ratio | `MACHINE_GEAR_RATIO` | 30:1 | — | Motor label |
| Encoder CPR (motor shaft) | — | 12 | CPR | Pololu encoder spec |
| Encoder CPR (output shaft) | — | 360 | CPR | 12 × 30 |
| Encoder counts (quadrature) | `MACHINE_ENCODER_CPR` | 1440 | counts/rev | 48 × 30 |
| Wheel Diameter | `MACHINE_WHEEL_DIAMETER` | 32.0 | mm | Pololu wheel spec |
| Track Width | `MACHINE_TRACK` | 125.0 | mm | **Measured** (wheel center-to-center) |
| Rotation Radius | `MACHINE_ROTATION_RADIUS` | 62.5 | mm | Track / 2 |
| Tail Length | `MACHINE_TAIL_LENGTH` | 83.0 | mm | **Measured** (drive axle → front HC-SR04) |
| Wheelbase (robot length) | — | 100 | mm | Chassis |
| Robot Width | — | 100 | mm | Chassis |

### Derived Constants
```
ENC_MM_PER_COUNT = (32.0 * π * 30) / 1440 ≈ 2.094 mm/count
```

---

## Electrical Parameters

### Battery
- **Type**: 3S LiPo (3 cells in series)
- **Nominal Voltage**: 11.1 V
- **Max Voltage**: 12.6 V
- **Min Voltage**: 9.0 V (3.0 V/cell)
- **Capacity**: TBD mAh

### Voltage Divider (Battery Monitor)
- **R1**: 100 kΩ (to battery +)
- **R2**: 10 kΩ (to GND)
- **Ratio**: 11× (`BAT_DIVIDER_RATIO = 11.0f`)
- **ADC Pin**: GPIO 35 (ADC1_CH7)
- **ADC Reference**: 3.548 V (ESP32 ADC 11 dB attenuation)
- **Formula**: `Vbat = ADC * 3.548 / 4095 * 11`

### Power Rails
| Rail | Voltage | Source | Consumers | Current Limit |
|------|---------|--------|-----------|---------------|
| **VMOT** (Motor) | 11.1–12.6 V | 3S LiPo direct | TB6612FNG VMOT → N20 motors | 3A peak / ch (TB6612) |
| **VCC** (Logic) | 5.0 V | XL4015 buck | ESP32 5V, MPU6050, HC-SR04×3, TB6612 VCC, LEDs, buzzer | XL4015 5A (thermal), TB6612 3A overcurrent |

### XL4015 Buck Converter
- **Input**: 12V battery
- **Output**: 5.0 V (adjust voltage pot to 5.0V under load)
- **Max Current**: 5 A (thermal shutdown self-resetting)
- **Current Limit Pot**: **Not present on this board** — relies on thermal shutdown
- **Protection**: Add 470 µF electrolytic + 10 µF ceramic on output

### TB6612FNG Motor Driver (Pololu #713)
| Parameter | Value |
|-----------|-------|
| Motor Voltage (VMOT) | 12 V (battery) |
| Logic Voltage (VCC) | 5 V (XL4015) |
| Continuous Current | 1.2 A / channel |
| Peak Current | 3 A / channel |
| Max PWM Frequency | 100 kHz |
| Configured PWM | 20 kHz (`LEDC_MOTOR_FREQ_HZ`) |
| Standby Pin | Must be HIGH to enable |

---

## Sensors

### HC-SR04 Ultrasonic (×3)
| Parameter | Value |
|-----------|-------|
| Operating Voltage | 5 V |
| Range | 2 cm – 400 cm |
| Resolution | 3 mm |
| Beam Angle | 15° half-angle (30° full cone) |
| Trigger Pulse | 10 µs |
| Echo Timeout | 23200 µs (4 m round-trip) |
| Speed of Sound | 0.343 mm/µs (20°C) |
| Mounting | Left (13/14), Front (27/26), Right (33/34) |

### MPU6050 IMU (GY-521)
| Parameter | Value |
|-----------|-------|
| I²C Address | 0x68 (AD0 = GND) |
| Accel Range | ±2g / ±4g / ±8g / ±16g |
| Gyro Range | ±250 / ±500 / ±1000 / ±2000 °/s |
| Accel Noise | ~400 µg/√Hz |
| Gyro Noise | ~0.05 °/s/√Hz |
| I²C Speed | 400 kHz |
| SDA / SCL | GPIO 21 / 22 |

### Encoder (Hall Effect, built into N20)
| Parameter | Value |
|-----------|-------|
| Type | 2-channel quadrature Hall effect |
| PPR (motor shaft) | 12 CPR |
| Output | Digital A/B, 2.7–18V, 10 kΩ pull-ups |
| Cable | 6-wire (VCC, GND, A, B, M+, M-) |
| Interface | ESP32 PCNT (GPIO 36/39, 22/21) |
| PCNT Units | 0 (left), 1 (right) |

---

## Pin Mapping

| Function | GPIO | Notes |
|----------|------|-------|
| US_LEFT_TRIG | 13 | HC-SR04 Left |
| US_LEFT_ECHO | 14 | |
| US_FRONT_TRIG | 27 | HC-SR04 Front |
| US_FRONT_ECHO | 26 | |
| US_RIGHT_TRIG | 33 | HC-SR04 Right |
| US_RIGHT_ECHO | 34 | |
| MOTOR_L_IN1 | 25 | TB6612 AIN1 |
| MOTOR_L_IN2 | 32 | TB6612 AIN2 |
| MOTOR_L_PWM | 18 | TB6612 PWMA (LEDC ch0) |
| MOTOR_R_IN1 | 19 | TB6612 BIN1 |
| MOTOR_R_IN2 | 23 | TB6612 BIN2 |
| MOTOR_R_PWM | 5 | TB6612 PWMB (LEDC ch1) |
| ENC_L_A | 36 | PCNT unit 0 |
| ENC_L_B | 39 | PCNT unit 0 |
| ENC_R_A | 22 | PCNT unit 1 |
| ENC_R_B | 21 | PCNT unit 1 |
| IMU_SDA | 21 | I²C |
| IMU_SCL | 22 | I²C |
| BAT_ADC | 35 | ADC1_CH7 |
| BUZZER | 4 | LEDC ch2 |
| LED_0 (onboard) | 2 | |
| LED_1 | 5 | Shared with MOTOR_R_PWM |
| LED_2 | 18 | Shared with MOTOR_L_PWM |
| LED_3 | 19 | Shared with MOTOR_R_IN1 |
| BUTTON (BOOT) | 0 | Active LOW |

---

## Firmware Configuration (`config.h`)

```c
// Mechanical
#define MACHINE_GEAR_RATIO       (30.0f / 1.0f)
#define MACHINE_ENCODER_CPR      (12 * 30 * 4)   // = 1440
#define MACHINE_WHEEL_DIAMETER   32.0f
#define MACHINE_TRACK            125.0f
#define MACHINE_ROTATION_RADIUS  (MACHINE_TRACK / 2.0f)  // 62.5
#define MACHINE_TAIL_LENGTH      83.0f
#define SEGMENT_WIDTH            180.0f
#define WALL_THICKNESS           12.0f
#define USE_SLALOM_TURNS         1

// Power
#define BAT_DIVIDER_RATIO        11.0f
#define BAT_ADC_REF_VOLTAGE      3.548f

// Motor Driver
#define LEDC_MOTOR_FREQ_HZ       20000

// Sensors
#define US_ECHO_TIMEOUT_US       23200
#define US_SPEED_OF_SOUND_MM_US  0.343
#define WALL_THRESHOLD_SIDE_MM   70
#define WALL_THRESHOLD_FRONT_MM  120
#define WALL_HYSTERESIS_FACTOR   1.15f

// PID (tune after first run)
#define SPEED_CONTROLLER_KP      1.0f
#define SPEED_CONTROLLER_KI      0.1f
#define SPEED_CONTROLLER_KD      0.01f
```

### Derived Constants
```c
ENC_MM_PER_COUNT = (32.0 * π * 30) / 1440 ≈ 2.094 mm/count
```

---

## Simulator Configuration (`engine.js`)

```javascript
_defaults() {
  return {
    cell: 180, wallT: 12, rlen: 100, rwid: 100, track: 125, turnR: 60,
    s_vstr: 300, s_vcur: 240, s_vmax: 600, s_acc: 3000, s_dec: 2000, s_fbg: 20,
    s_preferUnknown: 0.9, s_revisit: 4,
    f_vmax: 800, f_vcur: 450, f_acc: 4000, f_dec: 4000, f_turnR: 50, f_v90: false,
    c_kp: 1.0, c_ki: 0.1, c_kd: 0.01, c_bat: 11.1, c_batsag: 0.15,
    u_sThr: 70, u_fThr: 120, u_hyst: 1.15, u_max: 4000, u_beam: 8,
    u_noise: 3, u_miss: 0.005, u_glitch: 0.0,
    o_drift: 0.02, o_slip: 0.02, o_dead: 8,
  };
}
```

---

## Measurement Procedures

### Track Width
1. Place robot on flat surface
2. Measure distance between **wheel contact patches** (center of tread)
3. Use digital calipers
4. Expected: ~125 mm

### Tail Length
1. Identify drive wheel axle centerline (rear wheels)
2. Measure along robot centerline to front face of HC-SR04 sensor
3. Expected: 83 mm

### Gear Ratio Verification (Optional)
1. Upload `gear_ratio_test` sketch
2. Place robot on blocks (wheels free)
3. Run test → observe Serial output
4. Formula: `ratio = encoder_counts / 48`

### Battery Divider Calibration
1. Measure battery voltage with DMM
2. Read ADC value via firmware
3. Adjust `BAT_DIVIDER_RATIO` if needed

### Battery ADC Calibration
1. Measure 3.3V rail with DMM
2. Adjust `BAT_ADC_REF_VOLTAGE` if different from 3.548V

---

## Wiring Diagram (Text)

```
BATTERY (3S LiPo)
├── (+) ──────────────────────────────────┬── TB6612 VMOT (12V)
├── (+) ──────────────────────────────────┼── XL4015 IN+
├── (-) ──────────────────────────────────┼── XL4015 IN-
│                                         │
XL4015 OUT+ (5V) ────────────────────────┼── 5V Rail
│                                         │
├── ESP32 5V pin
├── TB6612 VCC (logic)
├── HC-SR04 ×3 VCC
├── MPU6050 VCC
├── LEDs, Buzzer
└── Encoder VCC (both)

XL4015 OUT- ──────────────────────────────┼── GND Rail
├── Battery (-)
├── ESP32 GND
├── TB6612 GND
├── HC-SR04 ×3 GND
├── MPU6050 GND
└── Encoder GND (both)

ESP32 GPIO:
  13/14  ── US_LEFT_TRIG/ECHO
  27/26  ── US_FRONT_TRIG/ECHO
  33/34  ── US_RIGHT_TRIG/ECHO
  25/32/18 ── MOTOR_L_IN1/IN2/PWM (LEDC ch0)
  19/23/5  ── MOTOR_R_IN1/IN2/PWM (LEDC ch1)
  36/39  ── ENC_L_A/B (PCNT unit 0)
  22/21  ── ENC_R_A/B (PCNT unit 1)
  21/22  ── I2C SDA/SCL (MPU6050)
  35     ── BAT_ADC (voltage divider)
  4      ── BUZZER (LEDC ch2)
  2,5,18,19 ── LEDs
  0      ── BUTTON (BOOT)
```

---

## Safety Notes

1. **LiPo Handling**: Use fireproof bag, never over-discharge (< 3.0V/cell)
2. **Current Protection**: 
   - Motors: TB6612FNG internal 3A peak overcurrent
   - Logic rail: XL4015 thermal shutdown (add 470µF cap)
   - Consider 2A polyfuse on 5V rail if available
3. **Brownout Prevention**: 470µF electrolytic + 10µF ceramic on 5V rail near ESP32
4. **Reverse Polarity**: XL4015 has no reverse protection — double-check battery polarity
5. **Voltage Setting**: Set XL4015 to **exactly 5.0V** under load (measure with DMM)

---

## Competition Checklist

- [ ] Gear ratio verified (30:1)
- [ ] Track width measured (125 mm)
- [ ] Tail length measured (83 mm)
- [ ] XL4015 set to 5.0V under load
- [ ] Battery divider calibrated
- [ ] ADC reference voltage confirmed
- [ ] PID gains tuned (KP=1.0, KI=0.1, KD=0.01 starting)
- [ ] Wall thresholds tested (side 70mm, front 120mm)
- [ ] Battery monitor working
- [ ] Emergency stop functional
- [ ] Backup/restore maze working
- [ ] All sensors responding

---

## Revision History

| Date | Revision | Changes |
|------|----------|---------|
| 2026-09-08 | 1.0 | Initial release with 30:1 gear ratio, 125mm track, 83mm tail |