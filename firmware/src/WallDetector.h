/**
 * @file WallDetector.h
 * @brief Wall detection using 3× HC-SR04 ultrasonic sensors.
 * Reads from Ultrasonic driver, applies thresholds and hysteresis.
 */
#pragma once

#include "config.h"
#include "drivers/ultrasonic.h"
#include "utils/Accumulator.h"
#include "utils/TaskBase.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// Global ultrasonic instance (defined in global.cpp)
extern Ultrasonic us;

class WallDetector : public TaskBase {
public:
  WallDetector() {}
  bool begin() {
    cal_start_sem_ = xSemaphoreCreateBinary();
    cal_end_sem_ = xSemaphoreCreateBinary();
    cal_front_start_sem_ = xSemaphoreCreateBinary();
    cal_front_end_sem_ = xSemaphoreCreateBinary();
    return createTask("WallDetector", TASK_PRIO_WALL_DETECTOR, STACK_SIZE_MEDIUM);
  }

  bool backup() { return false; } // Placeholder for LittleFS
  bool restore() { return false; } // Placeholder

  void calibrationSide() {
    xSemaphoreTake(cal_end_sem_, 0);
    xSemaphoreGive(cal_start_sem_);
    xSemaphoreTake(cal_end_sem_, portMAX_DELAY);
  }

  void calibrationFront() {
    xSemaphoreTake(cal_front_end_sem_, 0);
    xSemaphoreGive(cal_front_start_sem_);
    xSemaphoreTake(cal_front_end_sem_, portMAX_DELAY);
  }

  void print() const {
    printf("Wall: L=%d F=%d R=%d  [%c %c %c]\n",
           distance_mm[Ultrasonic::LEFT], distance_mm[Ultrasonic::FRONT],
           distance_mm[Ultrasonic::RIGHT],
           wall[Ultrasonic::LEFT] ? 'X' : '.', wall[Ultrasonic::FRONT] ? 'X' : '.',
           wall[Ultrasonic::RIGHT] ? 'X' : '.');
  }

  // Public data for other modules
  uint16_t distance_mm[3] = {0, 0, 0}; // LEFT, FRONT, RIGHT
  bool wall[3] = {false, false, false};

private:
  SemaphoreHandle_t cal_start_sem_, cal_end_sem_;
  SemaphoreHandle_t cal_front_start_sem_, cal_front_end_sem_;
  Ultrasonic& us_ = us;

  // Median filter buffers
  Accumulator<uint16_t, 5> side_buf[2]; // LEFT, RIGHT
  Accumulator<uint16_t, 5> front_buf;

  void task() override {
    TickType_t last = xTaskGetTickCount();
    while (1) {
      vTaskDelayUntil(&last, CONTROL_PERIOD_US / portTICK_PERIOD_MS);
      last = xTaskGetTickCount();

      // --- Side walls (LEFT=0, RIGHT=2) ---
      for (int i = 0; i < 2; ++i) {
        Ultrasonic::Channel ch = (i == 0) ? Ultrasonic::LEFT : Ultrasonic::RIGHT;
        if (us_.isFresh(ch)) {
          side_buf[i].push(us_.getDistance(ch));
        }
        uint16_t median = side_buf[i].full() ? side_buf[i].average() : side_buf[i].average();

        if (!wall[i] && median < WALL_THRESHOLD_SIDE_MM) {
          wall[i] = true;
        } else if (wall[i] && median > WALL_THRESHOLD_SIDE_MM * WALL_HYSTERESIS_FACTOR) {
          wall[i] = false;
        }
        distance_mm[i] = median;
      }

      // --- Front wall ---
      if (us_.isFresh(Ultrasonic::FRONT)) {
        front_buf.push(us_.getDistance(Ultrasonic::FRONT));
      }
      uint16_t front_median = front_buf.full() ? front_buf.average() : front_buf.average();

      if (!wall[Ultrasonic::FRONT] && front_median < WALL_THRESHOLD_FRONT_MM) {
        wall[Ultrasonic::FRONT] = true;
      } else if (wall[Ultrasonic::FRONT] && front_median > WALL_THRESHOLD_FRONT_MM * WALL_HYSTERESIS_FACTOR) {
        wall[Ultrasonic::FRONT] = false;
      }
      distance_mm[Ultrasonic::FRONT] = front_median;

      // Stale detection: if no update for > 300 ms, clear wall
      for (int i = 0; i < 3; ++i) {
        Ultrasonic::Channel ch = (i == 0) ? Ultrasonic::LEFT :
                                 (i == 1) ? Ultrasonic::FRONT : Ultrasonic::RIGHT;
        if (us_.getStalenessMs(ch) > 300) {
          wall[i] = false;
        }
      }

      // Calibration handlers
      if (xSemaphoreTake(cal_start_sem_, 0) == pdTRUE) {
        calibrateSide();
        xSemaphoreGive(cal_end_sem_);
      }
      if (xSemaphoreTake(cal_front_start_sem_, 0) == pdTRUE) {
        calibrateFront();
        xSemaphoreGive(cal_front_end_sem_);
      }
    }
  }

  void calibrateSide() {
    float sum[2] = {0, 0};
    const int n = 500;
    TickType_t last = xTaskGetTickCount();
    for (int i = 0; i < n; ++i) {
      vTaskDelayUntil(&last, 1 / portTICK_PERIOD_MS);
      last = xTaskGetTickCount();
      if (us_.isFresh(Ultrasonic::LEFT)) sum[0] += us_.getDistance(Ultrasonic::LEFT);
      if (us_.isFresh(Ultrasonic::RIGHT)) sum[1] += us_.getDistance(Ultrasonic::RIGHT);
    }
    // Store reference distances (no wall) for potential future use
    printf("Wall cal side: L=%.1f R=%.1f\n", sum[0]/n, sum[1]/n);
  }

  void calibrateFront() {
    float sum = 0;
    const int n = 500;
    TickType_t last = xTaskGetTickCount();
    for (int i = 0; i < n; ++i) {
      vTaskDelayUntil(&last, 1 / portTICK_PERIOD_MS);
      last = xTaskGetTickCount();
      if (us_.isFresh(Ultrasonic::FRONT)) sum += us_.getDistance(Ultrasonic::FRONT);
    }
    printf("Wall cal front: %.1f\n", sum/n);
  }
};