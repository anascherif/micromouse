/**
 * @file global.cpp
 * @brief Global singleton definitions.
 */
#include "config.h"
#include "global.h"
#include "drivers/motor_tb6612.h"
#include "drivers/buzzer.h"
#include "drivers/led.h"
#include "drivers/button.h"
#include "drivers/imu_mpu6050.h"
#include "drivers/encoder_pcnt.h"
#include "drivers/ultrasonic.h"
#include "SpeedController.h"
#include "WallDetector.h"
#include "SearchRun.h"
#include "FastRun.h"
#include "MazeSolver.h"
#include "Emergency.h"
#include "UserInterface.h"

/* Drivers */
Motor mt;
Buzzer bz(BUZZER_PIN, LEDC_CH_BUZZER);
LED led(LED_PINS);
Button btn(BUTTON_PIN);
IMU imu;
Encoder enc;
Ultrasonic us;

/* Core modules */
SpeedController sc;
WallDetector wd;
SearchRun sr;
FastRun fr;
MazeSolver ms;
Emergency em;
UserInterface ui;