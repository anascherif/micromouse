/**
 * @file ultrasonic.h
 * @brief 3-channel HC-SR04 ultrasonic sensors with staggered triggering.
 * Each sensor fired sequentially every ~33 ms, giving ~100 ms update per sensor.
 * Uses GPIO ISR for precise echo timing.
 */
#pragma once

#include <driver/gpio.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "config.h"

class Ultrasonic {
public:
  enum Channel { LEFT = 0, FRONT = 1, RIGHT = 2, NUM_CHANNELS = 3 };

  Ultrasonic() {
    s_instance = this;
    for (int i = 0; i < NUM_CHANNELS; ++i) {
      distance_mm_[i] = 0;
      last_update_ms_[i] = 0;
      echo_start_us_[i] = 0;
    }
    trigger_sem_ = xSemaphoreCreateBinary();
    data_mutex_ = xSemaphoreCreateMutex();
  }

  bool begin() {
    // Configure trigger pins as output
    gpio_config_t trig_cfg = {};
    trig_cfg.pin_bit_mask = (1ULL << US_LEFT_TRIG_PIN) |
                            (1ULL << US_FRONT_TRIG_PIN) |
                            (1ULL << US_RIGHT_TRIG_PIN);
    trig_cfg.mode = GPIO_MODE_OUTPUT;
    trig_cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    trig_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&trig_cfg);

    // Configure echo pins as input with interrupt
    gpio_config_t echo_cfg = {};
    echo_cfg.pin_bit_mask = (1ULL << US_LEFT_ECHO_PIN) |
                            (1ULL << US_FRONT_ECHO_PIN) |
                            (1ULL << US_RIGHT_ECHO_PIN);
    echo_cfg.mode = GPIO_MODE_INPUT;
    echo_cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    echo_cfg.pull_down_en = GPIO_PULLDOWN_ENABLE;
    echo_cfg.intr_type = GPIO_INTR_ANYEDGE;
    gpio_config(&echo_cfg);

    // Install ISR service
    gpio_install_isr_service(0);
    gpio_isr_handler_add(US_LEFT_ECHO_PIN, echoISR, (void*)LEFT);
    gpio_isr_handler_add(US_FRONT_ECHO_PIN, echoISR, (void*)FRONT);
    gpio_isr_handler_add(US_RIGHT_ECHO_PIN, echoISR, (void*)RIGHT);

    // Set initial trigger pins low
    gpio_set_level((gpio_num_t)US_LEFT_TRIG_PIN, 0);
    gpio_set_level((gpio_num_t)US_FRONT_TRIG_PIN, 0);
    gpio_set_level((gpio_num_t)US_RIGHT_TRIG_PIN, 0);

    // Create staggered trigger task (runs every ~33 ms)
    xTaskCreatePinnedToCore(
        triggerTaskEntry, "US_Trigger", STACK_SIZE_SMALL, this,
        TASK_PRIO_ULTRASONIC, &trigger_task_handle_, 1);

    return true;
  }

  // Get latest distance in mm (0 if no valid reading yet)
  uint16_t getDistance(Channel ch) const {
    if (ch >= NUM_CHANNELS) return 0;
    uint16_t d = 0;
    xSemaphoreTake(data_mutex_, portMAX_DELAY);
    d = distance_mm_[ch];
    xSemaphoreGive(data_mutex_);
    return d;
  }

  // Get time since last valid reading in ms
  uint32_t getStalenessMs(Channel ch) const {
    if (ch >= NUM_CHANNELS) return UINT32_MAX;
    uint32_t t = 0;
    xSemaphoreTake(data_mutex_, portMAX_DELAY);
    t = (esp_timer_get_time() / 1000) - last_update_ms_[ch];
    xSemaphoreGive(data_mutex_);
    return t;
  }

  // Check if reading is fresh (updated within max_age_ms)
  bool isFresh(Channel ch, uint32_t max_age_ms = 200) const {
    return getStalenessMs(ch) <= max_age_ms;
  }

  void print() const {
    printf("US L:%d F:%d R:%d (mm)\n",
           getDistance(LEFT), getDistance(FRONT), getDistance(RIGHT));
  }

private:
  static constexpr gpio_num_t trig_pins_[NUM_CHANNELS] = {
    US_LEFT_TRIG_PIN, US_FRONT_TRIG_PIN, US_RIGHT_TRIG_PIN
  };
  static constexpr gpio_num_t echo_pins_[NUM_CHANNELS] = {
    US_LEFT_ECHO_PIN, US_FRONT_ECHO_PIN, US_RIGHT_ECHO_PIN
  };

  uint16_t distance_mm_[NUM_CHANNELS];
  uint32_t last_update_ms_[NUM_CHANNELS];
  int64_t echo_start_us_[NUM_CHANNELS];
  SemaphoreHandle_t trigger_sem_;
  SemaphoreHandle_t data_mutex_;
  TaskHandle_t trigger_task_handle_ = nullptr;

  static void echoISR(void* arg) {
    Channel ch = static_cast<Channel>(reinterpret_cast<intptr_t>(arg));
    int level = gpio_get_level(echo_pins_[ch]);
    int64_t now = esp_timer_get_time();
    if (s_instance) {
      if (level == 1) {
        s_instance->echo_start_us_[ch] = now;
      } else {
        int64_t pulse_us = now - s_instance->echo_start_us_[ch];
        if (pulse_us > 0 && pulse_us < US_ECHO_TIMEOUT_US) {
          uint16_t dist_mm = (uint16_t)(pulse_us * US_SPEED_OF_SOUND_MM_US / 2.0f);
          s_instance->distance_mm_[ch] = dist_mm;
          s_instance->last_update_ms_[ch] = now / 1000;
        }
      }
    }
  }

  static void triggerTaskEntry(void* pv) {
    static_cast<Ultrasonic*>(pv)->triggerTask();
  }

  void triggerTask() {
    const TickType_t interval = 33 / portTICK_PERIOD_MS; // ~30 Hz per sensor
    TickType_t last = xTaskGetTickCount();
    Channel next_ch = LEFT;

    while (1) {
      vTaskDelayUntil(&last, interval);
      last = xTaskGetTickCount();

      // Fire the next sensor
      triggerSingle(next_ch);
      next_ch = static_cast<Channel>((next_ch + 1) % NUM_CHANNELS);
    }
  }

  void triggerSingle(Channel ch) {
    gpio_set_level(trig_pins_[ch], 1);
    esp_rom_delay_us(US_TRIGGER_PULSE_US);
    gpio_set_level(trig_pins_[ch], 0);
  }

  static inline Ultrasonic* s_instance = nullptr;
};