/**
 * @file imu_mpu6050.h
 * @brief MPU6050 6-axis IMU (I2C) driver with 1 kHz sampling task.
 * Compatible with kerise-v3 IMU interface.
 */
#pragma once

#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "config.h"

struct MotionParameter {
  float x = 0, y = 0, z = 0;
  MotionParameter() = default;
  MotionParameter(float x, float y, float z) : x(x), y(y), z(z) {}
  MotionParameter operator+(const MotionParameter& o) const {
    return {x + o.x, y + o.y, z + o.z};
  }
  MotionParameter operator-(const MotionParameter& o) const {
    return {x - o.x, y - o.y, z - o.z};
  }
  MotionParameter operator*(float m) const {
    return {x * m, y * m, z * m};
  }
  MotionParameter operator/(float d) const {
    return {x / d, y / d, z / d};
  }
  MotionParameter& operator+=(const MotionParameter& o) {
    x += o.x; y += o.y; z += o.z; return *this;
  }
  MotionParameter& operator/=(float d) {
    x /= d; y /= d; z /= d; return *this;
  }
};

class IMU {
public:
  IMU() {
    sampling_sem_ = xSemaphoreCreateBinary();
    cal_start_sem_ = xSemaphoreCreateBinary();
    cal_end_sem_ = xSemaphoreCreateBinary();
  }

  bool begin() {
    Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
    Wire.setClock(IMU_I2C_FREQ_HZ);

    // Wake up and configure
    writeReg(0x6B, 0x00);  // PWR_MGMT_1: wake, internal 8MHz osc
    delay(100);
    writeReg(0x1A, 0x03);  // CONFIG: DLPF 44 Hz
    writeReg(0x1B, 0x18);  // GYRO_CONFIG: ±2000 dps (0x18 = FS_SEL=3)
    writeReg(0x1C, 0x00);  // ACCEL_CONFIG: ±2 g (0x00 = AFS_SEL=0)
    writeReg(0x19, 0x00);  // SMPLRT_DIV: 1 kHz sample rate
    writeReg(0x6B, 0x01);  // PWR_MGMT_1: PLL with X gyro ref

    // Verify WHO_AM_I
    uint8_t who = readReg(0x75);
    if (who != 0x68) {
      return false;
    }

    // Create 1 kHz task
    xTaskCreatePinnedToCore(
        taskEntry, "IMU", STACK_SIZE_MEDIUM, this,
        TASK_PRIO_IMU, &task_handle_, 1);

    return true;
  }

  MotionParameter accel = {0, 0, 0};
  MotionParameter gyro = {0, 0, 0};
  float angle = 0;  // integrated yaw (rad)

  void print() const {
    printf("IMU angle: %.3f  gyro: %.2f %.2f %.2f  accel: %.2f %.2f %.2f\n",
           angle, gyro.x, gyro.y, gyro.z, accel.x, accel.y, accel.z);
  }

  void calibration(bool wait = true) {
    xSemaphoreTake(cal_end_sem_, 0);
    xSemaphoreGive(cal_start_sem_);
    if (wait) calibrationWait();
  }

  void calibrationWait() {
    xSemaphoreTake(cal_end_sem_, portMAX_DELAY);
  }

  void samplingSemaphoreTake(TickType_t timeout = portMAX_DELAY) {
    xSemaphoreTake(sampling_sem_, timeout);
  }

private:
  SemaphoreHandle_t sampling_sem_;
  SemaphoreHandle_t cal_start_sem_;
  SemaphoreHandle_t cal_end_sem_;
  TaskHandle_t task_handle_ = nullptr;
  MotionParameter accel_offset_{0, 0, 0};
  MotionParameter gyro_offset_{0, 0, 0};
  static constexpr float GYRO_SENS = 16.4f;     // LSB/(°/s) at ±2000 dps
  static constexpr float ACCEL_SENS = 16384.0f; // LSB/g at ±2g
  static constexpr float G_TO_MM_S2 = 9806.65f;

  static void taskEntry(void* pv) {
    static_cast<IMU*>(pv)->task();
  }

  void task() {
    TickType_t last = xTaskGetTickCount();
    while (1) {
      vTaskDelayUntil(&last, CONTROL_PERIOD_US / portTICK_PERIOD_MS);
      last = xTaskGetTickCount();

      update();
      xSemaphoreGive(sampling_sem_);

      if (xSemaphoreTake(cal_start_sem_, 0) == pdTRUE) {
        calibrate();
        xSemaphoreGive(cal_end_sem_);
      }
    }
  }

  void update() {
    Wire.beginTransmission(IMU_I2C_ADDR);
    Wire.write(0x3B);  // ACCEL_XOUT_H
    Wire.endTransmission(false);
    Wire.requestFrom(IMU_I2C_ADDR, 14);

    int16_t ax = (Wire.read() << 8) | Wire.read();
    int16_t ay = (Wire.read() << 8) | Wire.read();
    int16_t az = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read(); // temp
    int16_t gx = (Wire.read() << 8) | Wire.read();
    int16_t gy = (Wire.read() << 8) | Wire.read();
    int16_t gz = (Wire.read() << 8) | Wire.read();

    // Convert to physical units
    // Accel: LSB/g = 16384, 1g = 9806.65 mm/s²
    accel.x = (ax / ACCEL_SENS) * G_TO_MM_S2 - accel_offset_.x;
    accel.y = (ay / ACCEL_SENS) * G_TO_MM_S2 - accel_offset_.y;
    accel.z = (az / ACCEL_SENS) * G_TO_MM_S2 - accel_offset_.z;

    // Gyro: LSB/(°/s) = 16.4, convert to rad/s
    gyro.x = (gx / GYRO_SENS) * M_PI / 180.0f - gyro_offset_.x;
    gyro.y = (gy / GYRO_SENS) * M_PI / 180.0f - gyro_offset_.y;
    gyro.z = (gz / GYRO_SENS) * M_PI / 180.0f - gyro_offset_.z;

    // Integrate yaw
    angle += gyro.z * (CONTROL_PERIOD_US / 1000000.0f);
  }

  void calibrate() {
    MotionParameter accel_sum{0, 0, 0}, gyro_sum{0, 0, 0};
    const int samples = 500;
    TickType_t last = xTaskGetTickCount();
    for (int i = 0; i < samples; ++i) {
      vTaskDelayUntil(&last, 1 / portTICK_PERIOD_MS);
      last = xTaskGetTickCount();
      // Read raw without offset subtraction
      Wire.beginTransmission(IMU_I2C_ADDR);
      Wire.write(0x3B);
      Wire.endTransmission(false);
      Wire.requestFrom(IMU_I2C_ADDR, 14);
      int16_t ax = (Wire.read() << 8) | Wire.read();
      int16_t ay = (Wire.read() << 8) | Wire.read();
      int16_t az = (Wire.read() << 8) | Wire.read();
      Wire.read(); Wire.read();
      int16_t gx = (Wire.read() << 8) | Wire.read();
      int16_t gy = (Wire.read() << 8) | Wire.read();
      int16_t gz = (Wire.read() << 8) | Wire.read();
      accel_sum.x += (ax / ACCEL_SENS) * G_TO_MM_S2;
      accel_sum.y += (ay / ACCEL_SENS) * G_TO_MM_S2;
      accel_sum.z += (az / ACCEL_SENS) * G_TO_MM_S2;
      gyro_sum.x += (gx / GYRO_SENS) * M_PI / 180.0f;
      gyro_sum.y += (gy / GYRO_SENS) * M_PI / 180.0f;
      gyro_sum.z += (gz / GYRO_SENS) * M_PI / 180.0f;
    }
    accel_offset_ += accel_sum / samples;
    gyro_offset_ += gyro_sum / samples;
    // Gravity compensation: Z accel should read +1g when flat
    accel_offset_.z -= G_TO_MM_S2;
  }

  void writeReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(IMU_I2C_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
  }

  uint8_t readReg(uint8_t reg) {
    Wire.beginTransmission(IMU_I2C_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(IMU_I2C_ADDR, 1);
    return Wire.read();
  }
};