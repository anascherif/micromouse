/**
 * @file led.h
 * @brief Simple LED array control.
 * Ported from kerise-v3.
 */
#pragma once

#include <Arduino.h>
#include <vector>
#include "config.h"

class LED {
public:
  LED(const std::vector<int>& pins = LED_PINS) : pins_(pins), value_(0) {}

  bool begin() {
    for (int pin : pins_) pinMode(pin, OUTPUT);
    return true;
  }

  operator uint8_t() const { return value_; }

  uint8_t operator=(uint8_t new_value) {
    value_ = new_value;
    for (size_t i = 0; i < pins_.size(); ++i) {
      digitalWrite(pins_[i], (value_ & (1 << i)) ? HIGH : LOW);
    }
    return value_;
  }

private:
  const std::vector<int> pins_;
  uint8_t value_;
};