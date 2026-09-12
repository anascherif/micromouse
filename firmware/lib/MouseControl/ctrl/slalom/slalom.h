/**
 * @file slalom.h
 * @brief Slalom trajectory generation from constraints.
 *
 * Portions derived from micromouse-mouse-control (MIT License)
 * Copyright (c) Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#pragma once

#include <ctrl/accel_designer.h>
#include <ctrl/pose.h>
#include <ctrl/state.h>

#include <array>
#include <cmath>
#include <ostream>

/**
 * @brief Control-related namespace.
 */
namespace ctrl {

/**
 * @brief Slalom-related namespace.
 */
namespace slalom {

/**
 * @brief Default maximum angular jerk [rad/s/s/s].
 */
static constexpr float dddth_max_default = 1200 * M_PI;
/**
 * @brief Default maximum angular acceleration [rad/s/s].
 */
static constexpr float ddth_max_default = 36 * M_PI;
/**
 * @brief Default maximum angular velocity [rad/s].
 */
static constexpr float dth_max_default = 3 * M_PI;

/**
 * @brief slalom::Shape describes a slalom shape.
 *
 * The members are determined interdependently, so modifying individual
 * values is not allowed. Use slalom::Trajectory to obtain a slalom
 * trajectory.
 */
struct Shape {
  Pose total; /**< @brief Displacement including the straight sections. */
  Pose curve; /**< @brief Displacement of the curved section. */
  float straight_prev; /**< @brief Straight distance before the curve [m]. */
  float straight_post; /**< @brief Straight distance after the curve [m]. */
  float v_ref;         /**< @brief Reference velocity on the curve [m/s]. */
  float dddth_max;     /**< @brief Maximum angular jerk magnitude [rad/s/s/s]. */
  float ddth_max;      /**< @brief Maximum angular acceleration [rad/s/s]. */
  float dth_max;       /**< @brief Maximum angular velocity [rad/s]. */

 public:
  /**
   * @brief Constructor generating a slalom shape from constraints.
   *
   * @param[in] total Displacement including straights [m, m, rad].
   * @param[in] y_curve_end Displacement along the y axis (perpendicular
   * to travel) [m]. Determines the curve size; this is the shape design
   * parameter.
   * @param[in] x_adv Straight length along the x axis (travel
   * direction) [m]. Only used for 180-degree turns.
   * @param[in] dddth_max Maximum angular jerk magnitude [rad/s/s/s].
   * @param[in] ddth_max Maximum angular acceleration [rad/s/s].
   * @param[in] dth_max Maximum angular velocity [rad/s].
   */
  Shape(const Pose& total, const float y_curve_end, const float x_adv = 0,
        const float dddth_max = dddth_max_default,
        const float ddth_max = ddth_max_default,
        const float dth_max = dth_max_default)
      : total(total),
        dddth_max(dddth_max),
        ddth_max(ddth_max),
        dth_max(dth_max) {
    /* preparation */
    const float Ts = 1.5e-3f;  //< simulation integration period
    float v = 600.0f;          //< initial value
    State s;                   //< simulation state
    AccelDesigner ad;
    ad.reset(dddth_max, ddth_max, dth_max, 0, 0, total.th);
    /* iterate a few times for accuracy */
    for (int i = 0; i < 3; ++i) {
      s.q.x = s.q.y = 0;
      /* simulation */
      float t = 0;
      while (t + Ts < ad.t_end()) integrate(ad, s, v, t, Ts), t += Ts;
      integrate(ad, s, v, t, ad.t_end() - t);  //< integrate the remainder
      /* update using the result */
      v *= y_curve_end / s.q.y;
    }
    curve = s.q;
    v_ref = v;
    const float sin_th = std::sin(total.th);
    const float cos_th = std::cos(total.th);
    /* determine the straight lengths */
    if (std::abs(sin_th) < 1e-3f) {
      /* 180-degree turn */
      straight_prev = x_adv;
      straight_post = x_adv;
      curve = total;
    } else {
      /* other turns */
      straight_prev = total.x - s.q.x - cos_th / sin_th * (total.y - s.q.y);
      straight_post = 1 / sin_th * (total.y - s.q.y);
    }
  }
  /**
   * @brief Constructor simply assigning an already generated shape.
   *
   * @param[in] total Displacement including straights [m, m, rad].
   * @param[in] curve Displacement of the curved section [m, m, rad].
   * @param[in] straight_prev Straight length before the curve [m].
   * @param[in] straight_post Straight length after the curve [m].
   * @param[in] v_ref Reference translation velocity [m/s].
   * @param[in] dddth_max Maximum angular jerk magnitude [rad/s/s/s].
   * @param[in] ddth_max Maximum angular acceleration [rad/s/s].
   * @param[in] dth_max Maximum angular velocity [rad/s].
   */
  Shape(const Pose& total, const Pose& curve, float straight_prev,
        const float straight_post, const float v_ref, const float dddth_max,
        const float ddth_max, const float dth_max)
      : total(total),
        curve(curve),
        straight_prev(straight_prev),
        straight_post(straight_post),
        v_ref(v_ref),
        dddth_max(dddth_max),
        ddth_max(ddth_max),
        dth_max(dth_max) {}
  /**
   * @brief Integrate the trajectory using the Runge-Kutta method.
   *
   * @param[in] ad Angular velocity profile.
   * @param[inout] s State variables.
   * @param[in] v Translation velocity [m/s].
   * @param[in] t Time [s].
   * @param[in] Ts Integration period [s].
   * @param[in] k_slip Slip angle constant.
   */
  static void integrate(const AccelDesigner& ad, State& s, const float v,
                        const float t, const float Ts, const float k_slip = 0) {
    /* Calculation */
    const std::array<float, 3> th{{ad.x(t), ad.x(t + Ts / 2), ad.x(t + Ts)}};
    const std::array<float, 3> w{{ad.v(t), ad.v(t + Ts / 2), ad.v(t + Ts)}};
    std::array<float, 3> cos_th;
    std::array<float, 3> sin_th;
    for (int i = 0; i < 3; ++i) {
      const auto th_slip = std::atan(-k_slip * v * w[i]);
      cos_th[i] = std::cos(th[i] + th_slip);
      sin_th[i] = std::sin(th[i] + th_slip);
    }
    /* Runge-Kutta Integral */
    s.q.x += v * Ts * (cos_th[0] + 4 * cos_th[1] + cos_th[2]) / 6;
    s.q.y += v * Ts * (sin_th[0] + 4 * sin_th[1] + sin_th[2]) / 6;
    /* Result */
    s.dq.x = v * cos_th[2];
    s.dq.y = v * sin_th[2];
    s.q.th = ad.x(t + Ts);
    s.dq.th = ad.v(t + Ts);
    s.ddq.th = ad.a(t + Ts);
    s.dddq.th = ad.j(t + Ts);
    s.ddq.x = -s.dq.y * s.dq.th;
    s.ddq.y = +s.dq.x * s.dq.th;
    s.dddq.x = -s.ddq.y * s.dq.th - s.dq.y * s.ddq.th;
    s.dddq.y = +s.ddq.x * s.dq.th + s.dq.x * s.ddq.th;
  }
  /**
   * @brief Print the object information.
   */
  friend std::ostream& operator<<(std::ostream& os, const Shape& obj) {
    os << "Slalom Shape" << std::endl;
    os << "\ttotal:\t" << obj.total << std::endl;
    os << "\tcurve:\t" << obj.curve << std::endl;
    os << "\tv_ref:\t" << obj.v_ref << std::endl;
    os << "\tstraight_prev:\t" << obj.straight_prev << std::endl;
    os << "\tstraight_post:\t" << obj.straight_post << std::endl;
    auto end = Pose(obj.straight_prev) + obj.curve +
               Pose(obj.straight_post).rotate(obj.curve.th);
    os << "\tintegral error:\t" << obj.total - end << std::endl;
    return os;
  }
};

}  // namespace slalom
}  // namespace ctrl