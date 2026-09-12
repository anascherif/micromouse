/**
 * @file trajectory.h
 * @brief Straight-line trajectory generator.
 *
 * Portions derived from micromouse-mouse-control (MIT License)
 * Copyright (c) Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#pragma once

#include <ctrl/accel_designer.h>
#include <ctrl/state.h>

/**
 * @brief Control-related namespace.
 */
namespace ctrl {

/**
 * @brief Straight-line related namespace.
 */
namespace straight {

/**
 * @brief straight::Trajectory straight-line trajectory generator.
 *
 * Provided for use with ctrl::TrajectoryTracker.
 */
class Trajectory : public AccelDesigner {
 public:
  /**
   * @brief Empty constructor.
   * Must be initialized via the base class AccelDesigner::reset().
   */
  Trajectory() {}
  /**
   * @brief Update the state.
   *
   * @param[out] s State variables.
   * @param[in] t Current time.
   */
  void update(struct State& s, const float t) const {
    s.q = Pose(x(t), 0, 0);
    s.dq = Pose(v(t), 0, 0);
    s.ddq = Pose(a(t), 0, 0);
    s.dddq = Pose(j(t), 0, 0);
  }
};

}  // namespace straight
}  // namespace ctrl