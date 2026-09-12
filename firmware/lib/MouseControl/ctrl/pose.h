/**
 * @file pose.h
 * @brief Pose coordinates on a plane.
 *
 * Portions derived from micromouse-mouse-control (MIT License)
 * Copyright (c) Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#pragma once

#include <cmath>
#include <ostream>

namespace ctrl {

/**
 * @brief Position and orientation.
 */
struct Pose {
  float x;  /**< @brief x component [m] */
  float y;  /**< @brief y component [m] */
  float th; /**< @brief theta component [rad] */

 public:
  constexpr Pose(const float x = 0, const float y = 0, const float th = 0)
      : x(x), y(y), th(th) {}
  void clear() { x = y = th = 0; }
  Pose mirror_x() const { return Pose(x, -y, -th); }
  Pose rotate(const float angle) const {
    const float cos_angle = std::cos(angle);
    const float sin_angle = std::sin(angle);
    return {x * cos_angle - y * sin_angle, x * sin_angle + y * cos_angle, th};
  }
  Pose homogeneous(const Pose& offset) const {
    return offset + this->rotate(offset.th);
  }
  Pose& operator+=(const Pose& o) {
    return x += o.x, y += o.y, th += o.th, *this;
  }
  Pose& operator-=(const Pose& o) {
    return x -= o.x, y -= o.y, th -= o.th, *this;
  }
  Pose operator+(const Pose& o) const { return {x + o.x, y + o.y, th + o.th}; }
  Pose operator-(const Pose& o) const { return {x - o.x, y - o.y, th - o.th}; }
  friend std::ostream& operator<<(std::ostream& os, const Pose& o) {
    return os << "(" << o.x << ", " << o.y << ", " << o.th << ")";
  }
};

}  // namespace ctrl