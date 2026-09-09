/**
 * @file button.h
 * @brief Button with debouncing and long-press detection (FreeRTOS task).
 * Ported from kerise-v3 for GPIO0 BOOT button.
 */
#pragma once

#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "config.h"

class Button {
public:
  Button(gpio_num_t pin = BUTTON_PIN) : pin_(pin) {}

  bool begin() {
    gpio_config_t cfg = {};
    cfg.pin_bit_mask = 1ULL << pin_;
    cfg.mode = GPIO_MODE_INPUT;
    cfg.pull_up_en = GPIO_PULLUP_ENABLE;
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    cfg.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&cfg);

    flags = 0;
    counter_ = 0;

    xTaskCreatePinnedToCore(
        taskEntry, "Button", STACK_SIZE_SMALL, this,
        TASK_PRIO_BUTTON, &task_handle_, 1);
    return true;
  }

  union {
    uint8_t flags = 0;
    struct {
      uint8_t pressed : 1;         // single press detected
      uint8_t long_pressed_1 : 1;  // > 200 ms
      uint8_t long_pressed_2 : 1;  // > 1000 ms
      uint8_t long_pressed_3 : 1;  // > 5000 ms
      uint8_t pressing : 1;        // currently pressed
      uint8_t long_pressing_1 : 1; // currently held > 200 ms
      uint8_t long_pressing_2 : 1; // currently held > 1000 ms
      uint8_t long_pressing_3 : 1; // currently held > 5000 ms
    };
  };

private:
  gpio_num_t pin_;
  TaskHandle_t task_handle_ = nullptr;
  int counter_ = 0;

  static void taskEntry(void* pv) {
    static_cast<Button*>(pv)->task();
  }

  void task() {
    TickType_t last = xTaskGetTickCount();
    while (1) {
      vTaskDelayUntil(&last, BUTTON_SAMPLING_MS / portTICK_PERIOD_MS);
      last = xTaskGetTickCount();

      int level = gpio_get_level(pin_);
      bool active = (BUTTON_ACTIVE_LOW) ? (level == 0) : (level == 1);

      if (active) {
        if (counter_ < BUTTON_LONG_PRESS_LEVEL_3 + 1) counter_++;
        if (counter_ == BUTTON_LONG_PRESS_LEVEL_3) long_pressing_3 = 1;
        if (counter_ == BUTTON_LONG_PRESS_LEVEL_2) long_pressing_2 = 1;
        if (counter_ == BUTTON_LONG_PRESS_LEVEL_1) long_pressing_1 = 1;
        if (counter_ == BUTTON_PRESS_LEVEL) pressing = 1;
      } else {
        if (counter_ >= BUTTON_LONG_PRESS_LEVEL_3) long_pressed_3 = 1;
        else if (counter_ >= BUTTON_LONG_PRESS_LEVEL_2) long_pressed_2 = 1;
        else if (counter_ >= BUTTON_LONG_PRESS_LEVEL_1) long_pressed_1 = 1;
        else if (counter_ >= BUTTON_PRESS_LEVEL) pressed = 1;
        counter_ = 0;
        flags &= 0x0F; // clear pressing flags
      }
    }
  }
};