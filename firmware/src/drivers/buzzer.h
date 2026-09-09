/**
 * @file buzzer.h
 * @brief Buzzer with tone queue (FreeRTOS task).
 * Ported from kerise-v3.
 */
#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "config.h"

class Buzzer {
public:
  Buzzer(uint8_t pin = BUZZER_PIN, uint8_t ch = LEDC_CH_BUZZER)
    : pin_(pin), channel_(ch) {}

  bool begin() {
    ledcSetup(channel_, LEDC_BUZZER_FREQ_HZ, LEDC_BUZZER_RES_BITS);
    ledcAttachPin(pin_, channel_);
    queue_ = xQueueCreate(BUZZER_QUEUE_SIZE, sizeof(Music));
    xTaskCreatePinnedToCore(
        taskEntry, "Buzzer", STACK_SIZE_MEDIUM, this,
        TASK_PRIO_BUZZER, &task_handle_, 1);
    return true;
  }

  enum Music {
    SELECT,
    CANCEL,
    CONFIRM,
    SUCCESSFUL,
    ERROR,
    SHORT,
    BOOT,
    LOW_BATTERY,
    EMERGENCY,
    COMPLETE,
    MAZE_BACKUP,
    MAZE_RESTORE,
  };

  void play(Music music) {
    xQueueSendToBack(queue_, &music, 0);
  }

private:
  uint8_t pin_;
  uint8_t channel_;
  QueueHandle_t queue_ = nullptr;
  TaskHandle_t task_handle_ = nullptr;

  static void taskEntry(void* pv) {
    static_cast<Buzzer*>(pv)->task();
  }

  void sound(uint8_t note, uint8_t octave, uint32_t ms) {
    // note: 0=C, 1=C#, 2=D, 3=D#, 4=E, 5=F, 6=F#, 7=G, 8=G#, 9=A, 10=A#, 11=B
    static const uint16_t freq[12] = {
      262, 277, 294, 311, 330, 349, 370, 392, 415, 440, 466, 494
    };
    uint32_t f = freq[note] * (1 << octave);
    ledcSetup(channel_, f, LEDC_BUZZER_RES_BITS);
    ledcWrite(channel_, 1 << (LEDC_BUZZER_RES_BITS - 1)); // 50% duty
    vTaskDelay(pdMS_TO_TICKS(ms));
  }

  void mute(uint32_t ms = 400) {
    ledcWrite(channel_, 0);
    vTaskDelay(pdMS_TO_TICKS(ms));
  }

  void task() {
    while (1) {
      Music music;
      if (xQueueReceive(queue_, &music, portMAX_DELAY)) {
        switch (music) {
          case SELECT:     sound(0, 6, 100); mute(100); break;          // C6
          case CANCEL:     sound(4, 6, 100); sound(0, 6, 100); mute(100); break; // E6 C6
          case CONFIRM:    sound(0, 6, 100); sound(4, 6, 100); mute(100); break; // C6 E6
          case SUCCESSFUL: sound(0, 6, 100); sound(4, 6, 100); sound(7, 6, 100); mute(100); break; // C6 E6 G6
          case ERROR:      for (int i=0;i<6;i++) {sound(0,7,100);sound(4,7,100);} mute(); break;
          case BOOT:       sound(11,5,200);sound(4,6,400);sound(6,6,200);sound(11,6,600); mute(); break; // B5 E6 F#6 B6
          case LOW_BATTERY:sound(0,7,400);mute(200);sound(0,7,400);mute(200);sound(0,7,400);mute(200); break;
          case EMERGENCY:  sound(0,6,100);sound(5,6,100);mute(100);sound(5,6,75);mute(25);sound(5,6,176);sound(4,6,176);sound(2,6,176);sound(0,6,200);mute(100); break;
          case COMPLETE:   sound(0,6,100);sound(2,6,100);sound(4,6,100);sound(5,6,100);sound(7,6,100);sound(9,6,100);sound(11,6,100);sound(0,7,100);mute(100); break;
          case SHORT:      sound(0,7,50);mute(50); break;
          case MAZE_BACKUP:sound(7,7,100);sound(4,7,100);sound(0,7,100);mute(100); break;
          case MAZE_RESTORE:sound(0,7,100);sound(4,7,100);sound(7,7,100);mute(100); break;
          default:         sound(0,4,1000); mute(); break;
        }
      }
    }
  }
};