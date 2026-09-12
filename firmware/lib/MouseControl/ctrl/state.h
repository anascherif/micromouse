/**
 * @file state.h
 * @brief State variables for trajectory control.
 *
 * Portions derived from micromouse-mouse-control (MIT License)
 * Copyright (c) Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#pragma once

#include "pose.h"

/**
 * @brief Control-related namespace.
 */
namespace ctrl {

/**
 * @brief State variables for trajectory control.
 */
struct State {
  Pose q;     //**< @brief Position.
  Pose dq;    //**< @brief Velocity.
  Pose ddq;   //**< @brief Acceleration.
  Pose dddq;  //**< @brief Jerk.
};

};  // namespace ctrl