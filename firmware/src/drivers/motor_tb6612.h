/**
 * @file motor_tb6612.h
 * @brief TB6612FNG dual H-bridge motor driver.
 * 3 pins per motor: IN1 (dir), IN2 (dir), PWM (speed).
 */
#pragma once

#include <Arduino.h>
#include "config.h"

class Motor {
public:
  Motor() : emergency_(false) {
    pinMode(MOTOR_L_IN1_PIN, OUTPUT);
    pinMode(MOTOR_L_IN2_PIN, OUTPUT);
    pinMode(MOTOR_L_PWM_PIN, OUTPUT);
    pinMode(MOTOR_R_IN1_PIN, OUTPUT);
    pinMode(MOTOR_R_IN2_PIN, OUTPUT);
    pinMode(MOTOR_R_PWM_PIN, OUTPUT);

    ledcSetup(LEDC_CH_MOTOR_L, LEDC_MOTOR_FREQ_HZ, LEDC_MOTOR_RES_BITS);
    ledcSetup(LEDC_CH_MOTOR_R, LEDC_MOTOR_FREQ_HZ, LEDC_MOTOR_RES_BITS);
    ledcAttachPin(MOTOR_L_PWM_PIN, LEDC_CH_MOTOR_L);
    ledcAttachPin(MOTOR_R_PWM_PIN, LEDC_CH_MOTOR_R);

    free();
  }

  void left(int duty) {
    if (emergency_) return;
    driveMotor(MOTOR_L_IN1_PIN, MOTOR_L_IN2_PIN, LEDC_CH_MOTOR_L, duty);
  }

  void right(int duty) {
    if (emergency_) return;
    driveMotor(MOTOR_R_IN1_PIN, MOTOR_R_IN2_PIN, LEDC_CH_MOTOR_R, duty);
  }

  void drive(int16_t left_duty, int16_t right_duty) {
    left(left_duty);
    right(right_duty);
  }

  void free() {
    digitalWrite(MOTOR_L_IN1_PIN, LOW);
    digitalWrite(MOTOR_L_IN2_PIN, LOW);
    digitalWrite(MOTOR_R_IN1_PIN, LOW);
    digitalWrite(MOTOR_R_IN2_PIN, LOW);
    ledcWrite(LEDC_CH_MOTOR_L, 0);
    ledcWrite(LEDC_CH_MOTOR_R, 0);
  }

  void emergency_stop() {
    emergency_ = true;
    free();
  }

  void emergency_release() {
    emergency_ = false;
    free();
  }

  bool isEmergency() const { return emergency_; }

private:
  bool emergency_;

  void driveMotor(uint8_t in1, uint8_t in2, uint8_t ch, int duty) {
    if (duty > MOTOR_DUTY_MAX) duty = MOTOR_DUTY_MAX;
    if (duty < -MOTOR_DUTY_MAX) duty = -MOTOR_DUTY_MAX;

    if (duty >= 0) {
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      ledcWrite(ch, duty);
    } else {
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      ledcWrite(ch, -duty);
    }
  }
};