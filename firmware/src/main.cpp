/**
 * @file main.cpp
 * @brief Main entry point for ESP32 Arduino + FreeRTOS.
 */
#include <Arduino.h>
#include <WiFi.h>
#include <LittleFS.h>
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
#include "MazeSolver.h"
#include "Emergency.h"
#include "UserInterface.h"

void setup() {
  // Disable WiFi/BT (competition rules)
  WiFi.mode(WIFI_OFF);
  btStop();

  printf("\n**************** MICROMOUSE NRW 8.0 ****************\n");

  // Initialize LittleFS
  if (!LittleFS.begin(true)) {
    printf("LittleFS mount failed!\n");
  }

  // Initialize drivers
  if (!bz.begin()) bz.play(Buzzer::ERROR);
  if (!led.begin()) bz.play(Buzzer::ERROR);
  if (!btn.begin()) bz.play(Buzzer::ERROR);

  // Battery check
  ui.batteryCheck();

  // Boot sound
  bz.play(Buzzer::BOOT);

  // Initialize sensors
  if (!imu.begin()) bz.play(Buzzer::ERROR);
  if (!enc.begin()) bz.play(Buzzer::ERROR);
  if (!us.begin()) bz.play(Buzzer::ERROR);
  if (!wd.begin()) bz.play(Buzzer::ERROR);

  // Emergency monitor
  em.begin();

  printf("All systems initialized. Ready.\n");
  printf("Short press BOOT = Search run\n");
  printf("Long press BOOT  = Fast run\n");
}

void loop() {
  // Handle UI (button press detection)
  ui.update();

  // Periodic status print
  static uint32_t last_print = 0;
  if (millis() - last_print > 2000) {
    last_print = millis();
    if (!ms.isRunning()) {
      printf("Idle. Battery: %.2fV  Enc: L=%.0f R=%.0f\n",
             2 * 1.1f * 3.548f * analogRead(BAT_ADC_PIN) / 4095.0f * BAT_DIVIDER_RATIO,
             enc.position(0), enc.position(1));
    }
  }

  vTaskDelay(10 / portTICK_PERIOD_MS);
}