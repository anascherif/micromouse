/**
 * @file Emergency.h
 * @brief Emergency stop monitor (crash detection via IMU).
 */
#pragma once

#include "config.h"
#include "utils/TaskBase.h"
#include "drivers/motor_tb6612.h"
#include "drivers/imu_mpu6050.h"

// Externs
class Motor; extern Motor mt;
class Buzzer; extern Buzzer bz;
class MazeSolver; extern MazeSolver ms;
class IMU; extern IMU imu;

class Emergency : public TaskBase {
public:
  Emergency() {}
  void begin() {
    createTask("Emergency", TASK_PRIO_EMERGENCY, STACK_SIZE_SMALL);
  }

private:
  void task() override {
    TickType_t last = xTaskGetTickCount();
    while (1) {
      vTaskDelayUntil(&last, CONTROL_PERIOD_US / portTICK_PERIOD_MS);
      last = xTaskGetTickCount();

      // Crash detection: high lateral accel or excessive yaw rate
      if (fabs(imu.accel.y) > 10 * 9806.65f ||
          fabs(imu.gyro.z) > 1800.0f / 180.0f * M_PI) {
        mt.emergency_stop();
        bz.play(Buzzer::EMERGENCY);
        ms.terminate();
        vTaskDelay(500 / portTICK_PERIOD_MS);
        mt.emergency_release();
        last = xTaskGetTickCount();
      }
    }
  }
};