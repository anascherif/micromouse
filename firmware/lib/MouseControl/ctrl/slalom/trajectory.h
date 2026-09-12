/**
 * @file trajectory.h
 * @brief Slalom trajectory generation from constraints.
 *
 * Portions derived from micromouse-mouse-control (MIT License)
 * Copyright (c) Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#pragma once

#include <ctrl/slalom/slalom.h>

/**
 * @brief Control-related namespace.
 */
namespace ctrl {

/**
 * @brief Slalom-related namespace.
 */
namespace slalom {

/**
 * @brief slalom::Trajectory slalom trajectory generator.
 *
 * Provides the position and velocity at each time based on the slalom
 * shape and the translation velocity.
 */
class Trajectory {
 public:
  /**
   * @brief Constructor.
   *
   * @param[in] shape Slalom shape.
   * @param[in] mirror_x Mirror the shape about the x axis (left/right
   * relative to travel direction).
   */
  Trajectory(const Shape& shape, const bool mirror_x = false) : shape(shape) {
    if (mirror_x) {
      this->shape.curve = shape.curve.mirror_x();
      this->shape.total = shape.total.mirror_x();
    }
  }
  /**
   * @brief Set the translation velocity and initialize the trajectory.
   *
   * @param velocity Translation velocity [m/s].
   * @param th_start Initial orientation [rad] (optional).
   * @param t_start Initial time [s] (optional).
   */
  void reset(const float velocity, const float th_start = 0,
             const float t_start = 0) {
    this->velocity = velocity;
    const float gain = velocity / shape.v_ref;
    ad.reset(gain * gain * gain * shape.dddth_max, gain * gain * shape.ddth_max,
             gain * shape.dth_max, 0, 0, shape.total.th, th_start, t_start);
  }
  /**
   * @brief Update the trajectory.
   *
   * @param[inout] state Current state advanced to the next time.
   * @param[in] t Current time [s].
   * @param[in] Ts Integration period [s].
   * @param[in] k_slip Slip angle proportional constant.
   */
  void update(State& state, const float t, const float Ts,
              const float k_slip = 0) const {
    return Shape::integrate(ad, state, velocity, t, Ts, k_slip);
  }
  /**
   * @brief Get the translation velocity.
   */
  float getVelocity() const { return velocity; }
  /**
   * @brief Get the total turn time.
   */
  float getTimeCurve() const { return ad.t_end(); }
  /**
   * @brief Get the slalom shape.
   */
  const Shape& getShape() const { return shape; }
  /**
   * @brief Get the angular velocity designer.
   */
  const AccelDesigner& getAccelDesigner() const { return ad; }

 protected:
  Shape shape;      /**< @brief Slalom shape. */
  AccelDesigner ad; /**< @brief Curved acceleration generator for angular velocity. */
  float velocity;   /**< @brief Translation velocity. */
};

}  // namespace slalom
}  // namespace ctrl