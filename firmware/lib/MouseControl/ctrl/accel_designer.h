/**
 * @file accel_designer.h
 * @brief Trajectory generator for acceleration/deceleration runs under
 * distance constraints.
 *
 * Portions derived from micromouse-mouse-control (MIT License)
 * Copyright (c) Ryotaro Onuki <kerikun11+github@gmail.com>
 * @see https://www.kerislab.jp/posts/2018-04-29-accel-designer4/
 */
#pragma once

#include <algorithm>  //< for std::max, std::min
#include <array>
#include <iostream>  //< for std::cout
#include <limits>    //< for std::numeric_limits
#include <ostream>

#include "accel_curve.h"

/**
 * @brief Control-related namespace.
 */
namespace ctrl {

/**
 * @brief Generates curved acceleration/deceleration trajectories
 * satisfying given constraints.
 *
 * - Generates a curved acceleration trajectory meeting constraints
 * like target velocity and travel distance.
 * - Provides continuous functions returning jerk $j(t)$, acceleration
 * $a(t)$, velocity $v(t)$, and position $x(t)$ at any time $t$.
 * - Depending on constraints such as max acceleration $a_{\\max}$ and
 * start velocity $v_s$, the target velocity $v_t$ may not be
 * reachable.
 */
class AccelDesigner {
 public:
  /**
   * @brief Constructor with initialization.
   *
   * @param[in] j_max     Magnitude of maximum jerk [m/s/s/s], must be
   * positive.
   * @param[in] a_max     Magnitude of maximum acceleration [m/s/s],
   * must be positive.
   * @param[in] v_max     Magnitude of maximum velocity [m/s], must be
   * positive.
   * @param[in] v_start   Start velocity [m/s].
   * @param[in] v_target  Target velocity [m/s].
   * @param[in] dist      Travel distance [m].
   * @param[in] x_start   Start position [m] (optional).
   * @param[in] t_start   Start time [s] (optional).
   */
  AccelDesigner(const float j_max, const float a_max, const float v_max,
                const float v_start, const float v_target, const float dist,
                const float x_start = 0, const float t_start = 0) {
    reset(j_max, a_max, v_max, v_start, v_target, dist, x_start, t_start);
  }
  /**
   * @brief Empty constructor for instantiation only.
   * @attention Must be initialized with reset() afterwards.
   */
  AccelDesigner() { t0 = t1 = t2 = t3 = x0 = x3 = 0; }
  /**
   * @brief Generate the curve from the given constraints.
   *
   * @details Initializes all internal variables.
   * @param[in] j_max     Magnitude of maximum jerk [m/s/s/s], must be
   * positive.
   * @param[in] a_max     Magnitude of maximum acceleration [m/s/s],
   * must be positive.
   * @param[in] v_max     Magnitude of maximum velocity [m/s], must be
   * positive.
   * @param[in] v_start   Start velocity [m/s].
   * @param[in] v_target  Target velocity [m/s].
   * @param[in] dist      Travel distance [m].
   * @param[in] x_start   Start position [m] (optional).
   * @param[in] t_start   Start time [s] (optional).
   */
  void reset(const float j_max, const float a_max, const float v_max,
             const float v_start, const float v_target, const float dist,
             const float x_start = 0, const float t_start = 0) {
    /* determine the end velocity from the travel distance */
    auto v_end = v_target;  //< tentative assignment
    /* handle the case where the distance forbids reaching v_target */
    const auto dist_min = AccelCurve::calcDistanceFromVelocityStartToEnd(
        j_max, a_max, v_start, v_end);
    if (std::abs(dist) < std::abs(dist_min)) {
      ctrl_logd << "vs -> ve != vt" << std::endl;
      /* end velocity $v_e$ reachable over distance $d$ toward $v_t$ */
      v_end = AccelCurve::calcReachableVelocityEnd(j_max, a_max, v_start,
                                                   v_target, dist);
    }
    /* tentative saturation velocity */
    auto v_sat = dist > 0 ? std::max({v_start, v_max, v_end})
                          : std::min({v_start, -v_max, v_end});
    /* generate the curve */
    ac.reset(j_max, a_max, v_start, v_sat);  //< acceleration section
    dc.reset(j_max, a_max, v_sat, v_end);    //< deceleration section
    /* handle the case where reaching v_max violates the distance */
    const auto d_sum = ac.x_end() + dc.x_end();
    if (std::abs(dist) < std::abs(d_sum)) {
      ctrl_logd << "vs -> vr -> ve" << std::endl;
      /* reachable velocity from constraints like travel distance */
      const auto v_rm = AccelCurve::calcReachableVelocityMax(
          j_max, a_max, v_start, v_end, dist);
      /* avoid unnecessary deceleration */
      v_sat = dist > 0 ? std::max({v_start, v_rm, v_end})
                       : std::min({v_start, v_rm, v_end});
      ac.reset(j_max, a_max, v_start, v_sat);  //< acceleration
      dc.reset(j_max, a_max, v_sat, v_end);    //< deceleration
    }
    /* avoid t23 = nan; occurs for vs = ve = d = 0 */
    if (std::abs(v_sat) < std::numeric_limits<float>::epsilon()) v_sat = 1;
    /* compute the constants */
    const auto t23 = (dist - ac.x_end() - dc.x_end()) / v_sat;
    x0 = x_start;
    x3 = x_start + dist;
    t0 = t_start;
    t1 = t0 + ac.t_end();                     //< end of curved accel
    t2 = t0 + ac.t_end() + t23;               //< end of constant speed
    t3 = t0 + ac.t_end() + t23 + dc.t_end();  //< end of curved decel
#if 0
    /* output check */
    const auto e = 0.01f;  //< numerical tolerance
    bool show_info = false;
    /* saturation time */
    if (t23 < 0) {
      ctrl_logd << t23 << std::endl;
      show_info = true;
    }
    /* end velocity */
    if (std::abs(v_start - v_end) > e + std::abs(v_start - v_target)) {
      std::cerr << "Error: Velocity Target!" << std::endl;
      show_info = true;
    }
    /* saturation velocity */
    if (std::abs(v_sat) >
        e + std::max({v_max, std::abs(v_start), std::abs(v_end)})) {
      std::cerr << "Error: Velocity Saturation!" << std::endl;
      show_info = true;
    }
    /* timestamps */
    if (!(t0 <= t1 + e && t1 <= t2 + e && t2 <= t3 + e)) {
      ctrl_loge << "Error: Time Point Relationship!" << std::endl;
      show_info = true;
    }
    /* print the inputs */
    if (show_info) {
      ctrl_loge << "Constraints:"
                << "\tj_max: " << j_max << "\ta_max: " << a_max
                << "\tv_max: " << v_max << "\tv_start: " << v_start
                << "\tv_target: " << v_target << "\tdist: " << dist
                << std::endl;
      ctrl_loge << "ad.reset(" << j_max << ", " << a_max << ", " << v_max
                << ", " << v_start << ", " << v_target << ", " << dist << ");"
                << std::endl;
      /* print */
      ctrl_loge << "Time Stamp: "
                << "\tt0: " << t0 << "\tt1: " << t1 << "\tt2: " << t2
                << "\tt3: " << t3 << std::endl;
      ctrl_loge << "Position:   "
                << "\tx0: " << x0 << "\tx1: " << x0 + ac.x_end()
                << "\tx2: " << x0 + (dist - dc.x_end()) << "\tx3: " << x3
                << std::endl;
      ctrl_loge << "Velocity:   "
                << "\tv0: " << v_start << "\tv1: " << v(t1) << "\tv2: " << v(t2)
                << "\tv3: " << v_end << std::endl;
    }
#endif
  }
  /**
   * @brief Jerk j [m/s/s/s] at time t [s].
   * @param[in] Time t [s].
   * @return Jerk [m/s/s/s].
   */
  float j(const float t) const {
    if (t < t2)
      return ac.j(t - t0);
    else
      return dc.j(t - t2);
  }
  /**
   * @brief Acceleration a [m/s/s] at time t [s].
   * @param[in] Time t [s].
   * @return Acceleration [m/s/s].
   */
  float a(const float t) const {
    if (t < t2)
      return ac.a(t - t0);
    else
      return dc.a(t - t2);
  }
  /**
   * @brief Velocity v [m/s] at time t [s].
   * @param[in] Time t [s].
   * @return Velocity [m/s].
   */
  float v(const float t) const {
    if (t < t2)
      return ac.v(t - t0);
    else
      return dc.v(t - t2);
  }
  /**
   * @brief Position x [m] at time t [s].
   * @param[in] Time t [s].
   * @return Position [m].
   */
  float x(const float t) const {
    if (t < t2)
      return x0 + ac.x(t - t0);
    else
      return x3 - dc.x_end() + dc.x(t - t2);
  }
  /**
   * @brief End time [s].
   */
  float t_end() const { return t3; }
  /**
   * @brief End velocity [m/s].
   */
  float v_end() const { return dc.v_end(); }
  /**
   * @brief End position [m].
   */
  float x_end() const { return x3; }
  /**
   * @brief Start time of the curved acceleration [s].
   */
  float t_0() const { return t0; }
  /**
   * @brief Time of maximum velocity [s].
   */
  float t_1() const { return t1; }
  /**
   * @brief Start time of the curved deceleration [s].
   */
  float t_2() const { return t2; }
  /**
   * @brief End time of the curved deceleration [s].
   */
  float t_3() const { return t3; }
  /**
   * @brief Boundary timestamps of the curved sections.
   */
  const std::array<float, 8> getTimeStamps() const {
    return {{
        t0 + ac.t_0(),
        t0 + ac.t_1(),
        t0 + ac.t_2(),
        t0 + ac.t_3(),
        t2 + dc.t_0(),
        t2 + dc.t_1(),
        t2 + dc.t_2(),
        t2 + dc.t_3(),
    }};
  }
  /**
   * @brief Print the trajectory as CSV to stdout.
   */
  void printCsv(const float t_interval = 1e-3f) const {
    printCsv(std::cout, t_interval);
  }
  /**
   * @brief Print the trajectory as CSV to a std::ostream.
   */
  void printCsv(std::ostream& os, const float t_interval = 1e-3f) const {
    for (float t = t0; t < t_end(); t += t_interval)
      os << t << "," << j(t) << "," << a(t) << "," << v(t) << "," << x(t)
         << std::endl;
  }
  /**
   * @brief Print the object information.
   */
  friend std::ostream& operator<<(std::ostream& os, const AccelDesigner& obj) {
    os << "AccelDesigner:";
    os << "\td: " << obj.x3 - obj.x0;
    os << "\tvs: " << obj.ac.v(0);
    os << "\tvm: " << obj.ac.v_end();
    os << "\tve: " << obj.dc.v_end();
    os << "\tt0: " << obj.t0;
    os << "\tt1: " << obj.t1;
    os << "\tt2: " << obj.t2;
    os << "\tt3: " << obj.t3;
    return os;
  }

 protected:
  float t0, t1, t2, t3; /**< @brief Boundary times [s]. */
  float x0, x3;         /**< @brief Boundary positions [m]. */
  AccelCurve ac;        /**< @brief Curved acceleration object. */
  AccelCurve dc;        /**< @brief Curved deceleration object. */
};

}  // namespace ctrl