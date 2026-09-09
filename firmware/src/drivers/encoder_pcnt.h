/**
 * @file encoder_pcnt.h
 * @brief Quadrature encoder using ESP32 PCNT peripheral.
 * 1 kHz sampling task with semaphore notification.
 */
#pragma once

#include <driver/pcnt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "config.h"

class Encoder {
public:
  Encoder() {
    sampling_sem_ = xSemaphoreCreateBinary();
  }

  bool begin() {
    // Left encoder (PCNT unit 0)
    pcnt_config_t pcnt_cfg_l = {};
    pcnt_cfg_l.pulse_gpio_num = ENC_L_A_PIN;
    pcnt_cfg_l.ctrl_gpio_num = ENC_L_B_PIN;
    pcnt_cfg_l.lctrl_mode = PCNT_MODE_REVERSE;
    pcnt_cfg_l.hctrl_mode = PCNT_MODE_KEEP;
    pcnt_cfg_l.pos_mode = PCNT_COUNT_INC;
    pcnt_cfg_l.neg_mode = PCNT_COUNT_DEC;
    pcnt_cfg_l.counter_h_lim = 32767;
    pcnt_cfg_l.counter_l_lim = -32768;
    pcnt_cfg_l.unit = ENC_L_PCNT_UNIT;
    pcnt_cfg_l.channel = PCNT_CHANNEL_0;
    ESP_ERROR_CHECK(pcnt_unit_config(&pcnt_cfg_l));
    ESP_ERROR_CHECK(pcnt_counter_pause(ENC_L_PCNT_UNIT));
    ESP_ERROR_CHECK(pcnt_counter_clear(ENC_L_PCNT_UNIT));
    ESP_ERROR_CHECK(pcnt_filter_enable(ENC_L_PCNT_UNIT));
    ESP_ERROR_CHECK(pcnt_set_filter_value(ENC_L_PCNT_UNIT, 250)); // ~1 µs filter
    ESP_ERROR_CHECK(pcnt_counter_resume(ENC_L_PCNT_UNIT));

    // Right encoder (PCNT unit 1)
    pcnt_config_t pcnt_cfg_r = {};
    pcnt_cfg_r.pulse_gpio_num = ENC_R_A_PIN;
    pcnt_cfg_r.ctrl_gpio_num = ENC_R_B_PIN;
    pcnt_cfg_r.lctrl_mode = PCNT_MODE_REVERSE;
    pcnt_cfg_r.hctrl_mode = PCNT_MODE_KEEP;
    pcnt_cfg_r.pos_mode = PCNT_COUNT_INC;
    pcnt_cfg_r.neg_mode = PCNT_COUNT_DEC;
    pcnt_cfg_r.counter_h_lim = 32767;
    pcnt_cfg_r.counter_l_lim = -32768;
    pcnt_cfg_r.unit = ENC_R_PCNT_UNIT;
    pcnt_cfg_r.channel = PCNT_CHANNEL_0;
    ESP_ERROR_CHECK(pcnt_unit_config(&pcnt_cfg_r));
    ESP_ERROR_CHECK(pcnt_counter_pause(ENC_R_PCNT_UNIT));
    ESP_ERROR_CHECK(pcnt_counter_clear(ENC_R_PCNT_UNIT));
    ESP_ERROR_CHECK(pcnt_filter_enable(ENC_R_PCNT_UNIT));
    ESP_ERROR_CHECK(pcnt_set_filter_value(ENC_R_PCNT_UNIT, 250));
    ESP_ERROR_CHECK(pcnt_counter_resume(ENC_R_PCNT_UNIT));

    // Create 1 kHz task
    xTaskCreatePinnedToCore(
        taskEntry, "Encoder", STACK_SIZE_MEDIUM, this,
        TASK_PRIO_ENCODER, &task_handle_, 1);

    return true;
  }

  // Distance in mm (cumulative)
  float position(uint8_t ch) const {
    int32_t counts = (ch == 0) ? total_counts_l_ : total_counts_r_;
    float mm = counts * ENC_MM_PER_COUNT;
    return (ch == 1) ? -mm : mm; // right wheel reversed
  }

  // Raw encoder counts (cumulative, signed)
  int32_t getPulses(uint8_t ch) const {
    int32_t counts = (ch == 0) ? total_counts_l_ : total_counts_r_;
    return (ch == 1) ? -counts : counts;
  }

  // Current PCNT counter value (not cumulative)
  int16_t getRaw(uint8_t ch) const {
    int16_t val = 0;
    pcnt_get_counter_value((ch == 0) ? ENC_L_PCNT_UNIT : ENC_R_PCNT_UNIT, &val);
    return (ch == 1) ? -val : val;
  }

  void print() const {
    printf("Encoder L: %.2f mm  R: %.2f mm\n", position(0), position(1));
  }

  void samplingSemaphoreTake(TickType_t timeout = portMAX_DELAY) {
    xSemaphoreTake(sampling_sem_, timeout);
  }

private:
  SemaphoreHandle_t sampling_sem_;
  TaskHandle_t task_handle_ = nullptr;
  int32_t total_counts_l_ = 0;
  int32_t total_counts_r_ = 0;
  int16_t prev_l_ = 0;
  int16_t prev_r_ = 0;

  static void taskEntry(void* pv) {
    static_cast<Encoder*>(pv)->task();
  }

  void task() {
    TickType_t last = xTaskGetTickCount();
    while (1) {
      vTaskDelayUntil(&last, CONTROL_PERIOD_US / portTICK_PERIOD_MS);
      last = xTaskGetTickCount();

      int16_t cnt_l = 0, cnt_r = 0;
      pcnt_get_counter_value(ENC_L_PCNT_UNIT, &cnt_l);
      pcnt_get_counter_value(ENC_R_PCNT_UNIT, &cnt_r);

      // Handle overflow/underflow
      int32_t diff_l = cnt_l - prev_l_;
      int32_t diff_r = cnt_r - prev_r_;
      if (diff_l > 30000) diff_l -= 65536;
      if (diff_l < -30000) diff_l += 65536;
      if (diff_r > 30000) diff_r -= 65536;
      if (diff_r < -30000) diff_r += 65536;

      total_counts_l_ += diff_l;
      total_counts_r_ += diff_r;
      prev_l_ = cnt_l;
      prev_r_ = cnt_r;

      xSemaphoreGive(sampling_sem_);
    }
  }
};