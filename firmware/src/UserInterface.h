/**
 * @file UserInterface.h
 * @brief Simplified UI: short press = search, long press = fast run.
 * No menu system - minimal for competition.
 */
#pragma once

#include "config.h"
#include "drivers/button.h"
#include "drivers/buzzer.h"
#include "drivers/led.h"
#include "drivers/imu_mpu6050.h"
#include "drivers/encoder_pcnt.h"

// Externs
class Buzzer; extern Buzzer bz;
class Button; extern Button btn;
class LED; extern LED led;
class IMU; extern IMU imu;
class Encoder; extern Encoder enc;
class MazeSolver; extern MazeSolver ms;
class SearchRun; extern SearchRun sr;
class FastRun; extern FastRun fr;

class UserInterface {
public:
  UserInterface() {}

  // Call this in main loop to handle button events
  void update() {
    static bool was_pressed = false;
    static uint32_t press_start = 0;

    bool pressed = btn.pressing;

    if (pressed && !was_pressed) {
      // Press started
      press_start = millis();
      bz.play(Buzzer::SELECT);
    } else if (!pressed && was_pressed) {
      // Press released
      uint32_t duration = millis() - press_start;
      if (duration >= 1000) {
        // Long press: start fast run
        bz.play(Buzzer::CONFIRM);
        startFastRun();
      } else {
        // Short press: start search
        bz.play(Buzzer::CONFIRM);
        startSearch();
      }
    } else if (pressed) {
      // Still pressing - check for long press feedback
      uint32_t duration = millis() - press_start;
      if (duration >= 1000 && duration < 1100) {
        bz.play(Buzzer::SHORT); // Confirm long press registered
      }
    }
    was_pressed = pressed;
  }

  // Battery check at boot
  void batteryCheck() {
    float voltage = 2 * 1.1f * 3.54813389f * analogRead(BAT_ADC_PIN) / 4095.0f * BAT_DIVIDER_RATIO;
    printf("Battery: %.2f V\n", voltage);
    if (voltage < 3.8f) {
      bz.play(Buzzer::LOW_BATTERY);
      while (!btn.pressed) vTaskDelay(100);
      btn.flags = 0;
    }
    // LED battery indicator
    led = 0;
    if (voltage < 4.0f) led = 0x01;
    else if (voltage < 4.1f) led = 0x03;
    else if (voltage < 4.2f) led = 0x07;
    else led = 0x0F;
  }

private:
  void startSearch() {
    if (ms.isRunning()) return;
    bz.play(Buzzer::SUCCESSFUL);
    ms.start(false); // false = don't force search if map exists
  }

  void startFastRun() {
    if (ms.isRunning()) return;
    bz.play(Buzzer::SUCCESSFUL);
    // Use current fast run parameters (can be tuned via serial if needed)
    ms.start(true); // force new search to update map, then fast run
    // Actually, MazeSolver task handles search->fast run automatically
    // Just ensure we have a map
    if (!ms.isComplete()) {
      // Need to search first
      ms.start(false);
    } else {
      // We have a map, run fast
      // The MazeSolver task will do fast runs in a loop
    }
  }
};