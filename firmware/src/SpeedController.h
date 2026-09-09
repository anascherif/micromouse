/**
 * @file SpeedController.h
 * @brief PID speed controller with sensor fusion (encoder + IMU).
 * Ported from kerise-v3, adapted for N20 motors.
 */
#pragma once

#include "utils/Accumulator.h"
#include "config.h"
#include "drivers/encoder_pcnt.h"
#include "drivers/imu_mpu6050.h"
#include "drivers/motor_tb6612.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <cmath>

struct WheelParameter {
  float trans = 0;    // translational [mm/s]
  float rot = 0;      // rotational [rad/s]
  float wheel[2] = {0, 0}; // left, right wheel speed [mm/s]

  void pole2wheel() {
    wheel[0] = trans - MACHINE_ROTATION_RADIUS * rot;
    wheel[1] = trans + MACHINE_ROTATION_RADIUS * rot;
  }
  void wheel2pole() {
    rot = (wheel[1] - wheel[0]) / 2.0f / MACHINE_ROTATION_RADIUS;
    trans = (wheel[1] + wheel[0]) / 2.0f;
  }
  void clear() { trans = rot = wheel[0] = wheel[1] = 0; }
};

struct Position {
  float x = 0, y = 0, theta = 0;
  Position() {}
  Position(float x_, float y_, float t_ = 0) : x(x_), y(y_), theta(t_) {}
  void reset() { x = y = theta = 0; }
  void set(float x_, float y_, float t_) { x = x_; y = y_; theta = t_; }
  Position operator+(const Position& o) const {
    return Position(x + o.x, y + o.y, theta + o.theta);
  }
  Position operator-(const Position& o) const {
    return Position(x - o.x, y - o.y, theta - o.theta);
  }
  Position operator-() const { return Position(-x, -y, -theta); }
  Position rotate(float angle) const {
    return Position(x * cos(angle) - y * sin(angle),
                    x * sin(angle) + y * cos(angle), theta);
  }
  Position mirror_x() const { return Position(x, -y, -theta); }
};

class SpeedController {
public:
  WheelParameter target, actual, enconly, acconly;
  WheelParameter proportional, integral, differential;
  float Kp = SPEED_CONTROLLER_KP;
  float Ki = SPEED_CONTROLLER_KI;
  float Kd = SPEED_CONTROLLER_KD;
  Position position;
  int ave_num = 0;

  SpeedController() : enabled_(false) {
    xTaskCreatePinnedToCore(
        taskEntry, "SpeedCtrl", STACK_SIZE_MEDIUM, this,
        TASK_PRIO_SPEED_CTRL, &task_handle_, 1);
  }

  void enable(bool reset_pos = true) {
    reset();
    if (reset_pos) position.reset();
    enabled_ = true;
    printf("Speed Controller Enabled\n");
  }

  void disable() {
    enabled_ = false;
    vTaskDelay(2 / portTICK_PERIOD_MS);
    mt.free();
    printf("Speed Controller Disabled\n");
  }

  void set_target(float trans, float rot) {
    target.trans = trans;
    target.rot = rot;
    target.pole2wheel();
  }

private:
  bool enabled_ = false;
  TaskHandle_t task_handle_ = nullptr;
  WheelParameter actual_prev_, target_prev_;
  Accumulator<float, 16> wheel_pos[2];
  Accumulator<float, 16> accel_buf;
  Accumulator<float, 16> gyro_buf;

  static void taskEntry(void* pv) {
    static_cast<SpeedController*>(pv)->task();
  }

  void reset() {
    target.clear(); actual.clear(); integral.clear(); differential.clear();
    actual_prev_.clear(); target_prev_.clear();
    for (int i = 0; i < 2; ++i) wheel_pos[i].clear(enc.position(i));
    accel_buf.clear(imu.accel.y);
    gyro_buf.clear(imu.gyro.z);
  }

  void task() {
    TickType_t last = xTaskGetTickCount();
    while (1) {
      vTaskDelayUntil(&last, CONTROL_PERIOD_US / portTICK_PERIOD_MS);
      last = xTaskGetTickCount();

      if (!enabled_) continue;

      // Wait for sensor updates
      enc.samplingSemaphoreTake();
      imu.samplingSemaphoreTake();

      // Push latest readings
      for (int i = 0; i < 2; ++i) wheel_pos[i].push(enc.position(i));
      accel_buf.push(imu.accel.y);
      gyro_buf.push(imu.gyro.z);

      // Use fixed averaging window
      ave_num = 16;

      // Calculate actual wheel speeds from encoder (mm/s)
      float sum_accel = 0;
      for (int j = 0; j < ave_num - 1; ++j) sum_accel += accel_buf[j];

      for (int i = 0; i < 2; ++i) {
        actual.wheel[i] = (wheel_pos[i][0] - wheel_pos[i][ave_num - 1])
                          / (ave_num - 1) * 1000000.0f / CONTROL_PERIOD_US;
        enconly.wheel[i] = (wheel_pos[i][0] - wheel_pos[i][1])
                           * 1000000.0f / CONTROL_PERIOD_US;
      }
      acconly.trans = sum_accel * CONTROL_PERIOD_US / 1000000.0f / 2.0f;
      enconly.wheel2pole();

      // Sensor fusion: encoder + IMU accel
      actual.wheel2pole();
      actual.trans += sum_accel * CONTROL_PERIOD_US / 1000000.0f / 2.0f;
      actual.rot = imu.gyro.z;
      actual.pole2wheel();

      // PID
      for (int i = 0; i < 2; ++i) {
        integral.wheel[i] += (target.wheel[i] - actual.wheel[i])
                             * CONTROL_PERIOD_US / 1000000.0f;
        proportional.wheel[i] = target.wheel[i] - actual.wheel[i];
      }
      integral.wheel2pole();
      proportional.wheel2pole();
      differential.trans = (target.trans - target_prev_.trans)
                           / CONTROL_PERIOD_US * 1000000.0f - accel_buf[0];
      differential.rot = (target.rot - target_prev_.rot)
                         / CONTROL_PERIOD_US * 1000000.0f
                         - (gyro_buf[0] - gyro_buf[1])
                         / CONTROL_PERIOD_US * 1000000.0f;
      differential.pole2wheel();

      // Motor output
      float pwm[2];
      for (int i = 0; i < 2; ++i) {
        pwm[i] = Kp * proportional.wheel[i]
               + Ki * integral.wheel[i]
               + Kd * differential.wheel[i];
      }
      mt.drive((int16_t)pwm[0], (int16_t)pwm[1]);

      // Fail-safe: PWM saturation
      const float PWM_EMERGENCY = MOTOR_DUTY_MAX * 2;
      if (fabs(pwm[0]) > PWM_EMERGENCY || fabs(pwm[1]) > PWM_EMERGENCY) {
        mt.free();
        while (enabled_) vTaskDelay(1);
      }

      // Odometry integration
      position.theta += (actual_prev_.rot + actual.rot) / 2.0f
                        * CONTROL_PERIOD_US / 1000000.0f;
      position.x += (actual_prev_.trans + actual.trans) / 2.0f
                    * cos(position.theta) * CONTROL_PERIOD_US / 1000000.0f;
      position.y += (actual_prev_.trans + actual.trans) / 2.0f
                    * sin(position.theta) * CONTROL_PERIOD_US / 1000000.0f;
      actual_prev_ = actual;
      target_prev_ = target;
    }
  }
};