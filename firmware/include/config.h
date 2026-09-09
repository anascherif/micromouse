/**
 * @file config.h
 * @brief All hardware pin mappings and mechanical parameters.
 * EDIT THIS FILE after building the robot to match your wiring and measurements.
 */
#pragma once

/* Type definitions needed for typed pin macros (GPIO + PCNT hardware) */
#include <driver/gpio.h>  //< gpio_num_t
#include <driver/pcnt.h>  //< pcnt_unit_t

/* ============================================================
 *  USER EDIT SECTION — FILL IN AFTER BUILD
 * ============================================================ */

// ---- Mechanical parameters (measure with calipers) ----
// Gear ratio of N20 motor (e.g., 30.0f / 1.0f for 30:1)
#define MACHINE_GEAR_RATIO       (30.0f / 1.0f)

// Encoder counts per output shaft revolution (motor CPR × gear ratio × 4 for quadrature)
// 12 CPR motor shaft × 30:1 gear ratio × 4 (quadrature) = 1440 counts/rev output shaft
#define MACHINE_ENCODER_CPR      (12 * 30 * 4)  // = 1440

// Wheel diameter in mm (Pololu N20 wheel ≈ 32 mm)
#define MACHINE_WHEEL_DIAMETER   32.0f

// Distance between wheel contact patches (track width) in mm — MEASURE THIS
#define MACHINE_TRACK            125.0f

// Rotation radius = track / 2
#define MACHINE_ROTATION_RADIUS  (MACHINE_TRACK / 2.0f)

// Distance from drive wheel axle to front HC-SR04 sensor face in mm — MEASURE THIS
#define MACHINE_TAIL_LENGTH      83.0f

// ---- Maze geometry (from competition spec) ----
// Cell size: 18 cm = 180 mm
#define SEGMENT_WIDTH            180.0f
// Wall thickness: 1.2 cm = 12 mm
#define WALL_THICKNESS           12.0f

// ---- Slalom toggle ----
// 1 = use smooth slalom trajectories (S90, F45) from MouseControl
// 0 = use simple in-place pivot turns (safer if slalom is unstable)
#define USE_SLALOM_TURNS         1

/* ============================================================
 *  PIN MAPPINGS (change to match your wiring)
 * ============================================================ */

// --- HC-SR04 Ultrasonic sensors ---
// Left sensor
#define US_LEFT_TRIG_PIN         ((gpio_num_t)13)
#define US_LEFT_ECHO_PIN         ((gpio_num_t)14)
// Front sensor
#define US_FRONT_TRIG_PIN        ((gpio_num_t)27)
#define US_FRONT_ECHO_PIN        ((gpio_num_t)26)
// Right sensor
#define US_RIGHT_TRIG_PIN        ((gpio_num_t)33)
#define US_RIGHT_ECHO_PIN        ((gpio_num_t)34)

// HC-SR04 timing constants
#define US_TRIGGER_PULSE_US      10
#define US_ECHO_TIMEOUT_US       25000  // ~4.3 m max range
#define US_SPEED_OF_SOUND_MM_US  0.343  // mm/µs at 20°C

// --- TB6612FNG Motor Driver ---
// Left motor: IN1, IN2, PWM
#define MOTOR_L_IN1_PIN          25
#define MOTOR_L_IN2_PIN          32
#define MOTOR_L_PWM_PIN          18
// Right motor: IN1, IN2, PWM
#define MOTOR_R_IN1_PIN          19
#define MOTOR_R_IN2_PIN          23
#define MOTOR_R_PWM_PIN          5

// LEDC channels for motor PWM (0-15 available)
#define LEDC_CH_MOTOR_L          0
#define LEDC_CH_MOTOR_R          1
#define LEDC_MOTOR_FREQ_HZ       20000
#define LEDC_MOTOR_RES_BITS      10
#define MOTOR_DUTY_MAX           ((1 << LEDC_MOTOR_RES_BITS) - 1)  // 1023

// --- Quadrature Encoders (PCNT) ---
// Left encoder: A, B
#define ENC_L_A_PIN              ((gpio_num_t)36)
#define ENC_L_B_PIN              ((gpio_num_t)39)
// Right encoder: A, B
#define ENC_R_A_PIN              ((gpio_num_t)22)
#define ENC_R_B_PIN              ((gpio_num_t)21)

// PCNT units (ESP32 has 8 units, 0-7)
#define ENC_L_PCNT_UNIT          ((pcnt_unit_t)0)
#define ENC_R_PCNT_UNIT          ((pcnt_unit_t)1)

// --- MPU6050 IMU (I2C) ---
#define IMU_SDA_PIN              21
#define IMU_SCL_PIN              22
#define IMU_I2C_ADDR             0x68
#define IMU_I2C_FREQ_HZ          400000

// --- Battery Voltage Divider ---
// Voltage divider: R1=100k, R2=10k → 11x division
// ESP32 ADC: 12-bit, 3.3V ref (actually ~3.548V with 11dB atten)
// Formula: Vbat = ADC * 3.548 / 4095 * 11
#define BAT_ADC_PIN              35
#define BAT_DIVIDER_RATIO        11.0f
#define BAT_ADC_REF_VOLTAGE      3.548f

// --- Buzzer ---
#define BUZZER_PIN               4
#define LEDC_CH_BUZZER           2
#define LEDC_BUZZER_FREQ_HZ      440
#define LEDC_BUZZER_RES_BITS     8

// --- Status LEDs ---
#define LED_PINS                 { 2, 5, 18, 19 }  // LED0 = onboard blue

// --- Button (BOOT button on DevKit V1) ---
#define BUTTON_PIN               ((gpio_num_t)0)
#define BUTTON_ACTIVE_LOW        1  // BOOT button pulls GPIO0 low when pressed

// Button timing (ms). Task samples at BUTTON_SAMPLING_MS intervals.
#define BUTTON_SAMPLING_MS       1
#define BUTTON_PRESS_LEVEL       30    // normal press: >= 30 ms
#define BUTTON_LONG_PRESS_LEVEL_1 200  // short-long press: >= 200 ms
#define BUTTON_LONG_PRESS_LEVEL_2 1000 // long press: >= 1000 ms
#define BUTTON_LONG_PRESS_LEVEL_3 5000 // very long press: >= 5000 ms

// --- Buzzer task queue depth ---
#define BUZZER_QUEUE_SIZE        8

// --- SpeedController PID gains (in mm-based units) ---
// Tune these after first motor test; they convert wheel-speed error to PWM duty.
#define SPEED_CONTROLLER_KP      1.0f
#define SPEED_CONTROLLER_KI      0.1f
#define SPEED_CONTROLLER_KD      0.01f

// --- Wall detection thresholds (mm) ---
// Tune these after testing HC-SR04 readings in the maze
#define WALL_THRESHOLD_SIDE_MM   70   // wall present if side distance < this
#define WALL_THRESHOLD_FRONT_MM  120  // wall present if front distance < this
#define WALL_HYSTERESIS_FACTOR   1.15f  // off threshold = on * factor

/* ============================================================
 *  DERIVED CONSTANTS (do not edit)
 * ============================================================ */

// Distance per encoder count (mm)
#define ENC_MM_PER_COUNT \
  (MACHINE_WHEEL_DIAMETER * M_PI * MACHINE_GEAR_RATIO / MACHINE_ENCODER_CPR)

// Control loop period (µs) — 1 kHz
#define CONTROL_PERIOD_US        1000

// FreeRTOS task priorities (higher = more urgent)
#define TASK_PRIO_IMU            5
#define TASK_PRIO_ENCODER        5
#define TASK_PRIO_WALL_DETECTOR  4
#define TASK_PRIO_SPEED_CTRL     4
#define TASK_PRIO_SEARCH_RUN     3
#define TASK_PRIO_FAST_RUN       3
#define TASK_PRIO_MAZE_SOLVER    2
#define TASK_PRIO_ULTRASONIC     1
#define TASK_PRIO_BUTTON         1
#define TASK_PRIO_BUZZER         1
#define TASK_PRIO_EMERGENCY      4

// Task stack sizes (bytes)
#define STACK_SIZE_SMALL         2048
#define STACK_SIZE_MEDIUM        4096
#define STACK_SIZE_LARGE         8192