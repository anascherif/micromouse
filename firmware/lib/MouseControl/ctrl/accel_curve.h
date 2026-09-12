/**
 * @file accel_curve.h
 * @brief Smooth acceleration using jerk-0, accel-1st, velocity-2nd, and
 * position-3rd order functions.
 *
 * Portions derived from micromouse-mouse-control (MIT License)
 * Copyright (c) Ryotaro Onuki <kerikun11+github@gmail.com>
 * @see https://www.kerislab.jp/posts/2018-04-29-accel-designer4/
 */
#pragma once

#include <array>
#include <cmath>     //< for std::sqrt, std::cbrt
#include <iostream>  //< for std::cout
#include <ostream>

/* log level definition */
#define CTRL_LOG_LEVEL_NONE 0
#define CTRL_LOG_LEVEL_ERROR 1
#define CTRL_LOG_LEVEL_WARNING 2
#define CTRL_LOG_LEVEL_INFO 3
#define CTRL_LOG_LEVEL_DEBUG 4
/* set log level */
#ifndef CTRL_LOG_LEVEL
#define CTRL_LOG_LEVEL CTRL_LOG_LEVEL_WARNING
#endif
/* Log Error */
#if CTRL_LOG_LEVEL >= CTRL_LOG_LEVEL_ERROR
#define ctrl_loge (std::cout << "[E][" __FILE__ ":" << __LINE__ << "]\t")
#else
#define ctrl_loge std::ostream(0)
#endif
/* Log Warning */
#if CTRL_LOG_LEVEL >= CTRL_LOG_LEVEL_WARNING
#define ctrl_logw (std::cout << "[W][" __FILE__ ":" << __LINE__ << "]\t")
#else
#define ctrl_logw std::ostream(0)
#endif
/* Log Info */
#if CTRL_LOG_LEVEL >= CTRL_LOG_LEVEL_INFO
#define ctrl_logi (std::cout << "[I][" __FILE__ ":" << __LINE__ << "]\t")
#else
#define ctrl_logi std::ostream(0)
#endif
/* Log Debug */
#if CTRL_LOG_LEVEL >= CTRL_LOG_LEVEL_DEBUG
#define ctrl_logd (std::cout "[D][" << __FILE__ ":" << __LINE__ << "]\t")
#else
#define ctrl_logd std::ostream(0)
#endif

/**
 * @brief Control-related namespace.
 */
namespace ctrl {

/**
 * @brief Generates a curved acceleration profile without a distance
 * constraint.
 *
 * - Builds an acceleration curve from the given constraints.
 * - Smoothly joins start and end velocities.
 * - No constraint on travel distance.
 * - Start and end velocities may be positive or negative.
 */
class AccelCurve {
 public:
  /**
   * @brief Constructor with initialization.
   * @param[in] j_max   Magnitude of maximum jerk [m/s/s/s], must be
   * positive.
   * @param[in] a_max   Magnitude of maximum acceleration [m/s/s], must
   * be positive.
   * @param[in] v_start Start velocity [m/s].
   * @param[in] v_end   End velocity [m/s].
   */
  AccelCurve(const float j_max, const float a_max, const float v_start,
             const float v_end) {
    reset(j_max, a_max, v_start, v_end);
  }
  /**
   * @brief Empty constructor for instantiation only.
   * @attention Must be initialized with reset() afterwards.
   */
  AccelCurve() {
    jm = am = t0 = t1 = t2 = t3 = v0 = v1 = v2 = v3 = x0 = x1 = x2 = x3 = 0;
  }
  /**
   * @brief Generate the curve from the given constraints.
   * @details Initializes all internal variables.
   * @param[in] j_max   Magnitude of maximum jerk [m/s/s/s], must be
   * positive.
   * @param[in] a_max   Magnitude of maximum acceleration [m/s/s], must
   * be positive.
   * @param[in] v_start Start velocity [m/s].
   * @param[in] v_end   End velocity [m/s].
   */
  void reset(const float j_max, const float a_max, const float v_start,
             const float v_end) {
    /* assign with sign */
    am = (v_end > v_start) ? a_max : -a_max;  //< sign of max acceleration
    jm = (v_end > v_start) ? j_max : -j_max;  //< sign of max jerk
    /* assign initial and final values */
    v0 = v_start;  //< assign
    v3 = v_end;    //< assign
    t0 = 0;        //< initial time is zero here
    x0 = 0;        //< initial position is zero here
    /* duration of the curved velocity section */
    const auto tc = a_max / j_max;
    /* duration of the constant-acceleration section */
    const auto tm = (v3 - v0) / am - tc;
    /* branch on whether constant acceleration occurs */
    if (tm > 0) {
      /* velocity: curve -> line -> curve */
      t1 = t0 + tc;
      t2 = t1 + tm;
      t3 = t2 + tc;
      v1 = v0 + am * tc / 2;                 //< integrate v(t)
      v2 = v1 + am * tm;                     //< integrate v(t)
      x1 = x0 + v0 * tc + am * tc * tc / 6;  //< integrate x(t)
      x2 = x1 + v1 * tm;                     //< integrate x(t)
      x3 = x0 + (v0 + v3) / 2 * (t3 - t0);  //< trapezoid area of v(t)
    } else {
      /* velocity: curve -> curve */
      const auto tcp = std::sqrt((v3 - v0) / jm);  //< time to inflection
      t1 = t2 = t0 + tcp;
      t3 = t2 + tcp;
      v1 = v2 = (v0 + v3) / 2;  //< midpoint by symmetry
      x1 = x2 = x0 + v1 * tcp + jm * tcp * tcp * tcp / 6;  //< integrate x(t)
      x3 = x0 + 2 * v1 * tcp;  //< area of the v(t) graph
    }
  }
  /**
   * @brief Jerk j [m/s/s/s] at time t [s].
   * @param[in] Time t [s].
   * @return Jerk [m/s/s/s].
   */
  float j(const float t) const {
    if (t <= t0)
      return 0;
    else if (t <= t1)
      return jm;
    else if (t <= t2)
      return 0;
    else if (t <= t3)
      return -jm;
    else
      return 0;
  }
  /**
   * @brief Acceleration a [m/s/s] at time t [s].
   * @param[in] Time t [s].
   * @return Acceleration [m/s/s].
   */
  float a(const float t) const {
    if (t <= t0)
      return 0;
    else if (t <= t1)
      return jm * (t - t0);
    else if (t <= t2)
      return am;
    else if (t <= t3)
      return -jm * (t - t3);
    else
      return 0;
  }
  /**
   * @brief Velocity v [m/s] at time t [s].
   * @param[in] Time t [s].
   * @return Velocity [m/s].
   */
  float v(const float t) const {
    if (t <= t0)
      return v0;
    else if (t <= t1)
      return v0 + jm / 2 * (t - t0) * (t - t0);
    else if (t <= t2)
      return v1 + am * (t - t1);
    else if (t <= t3)
      return v3 - jm / 2 * (t - t3) * (t - t3);
    else
      return v3;
  }
  /**
   * @brief Position x [m] at time t [s].
   * @param[in] Time t [s].
   * @return Position [m].
   */
  float x(const float t) const {
    if (t <= t0)
      return x0 + v0 * (t - t0);
    else if (t <= t1)
      return x0 + v0 * (t - t0) + jm / 6 * (t - t0) * (t - t0) * (t - t0);
    else if (t <= t2)
      return x1 + v1 * (t - t1) + am / 2 * (t - t1) * (t - t1);
    else if (t <= t3)
      return x3 + v3 * (t - t3) - jm / 6 * (t - t3) * (t - t3) * (t - t3);
    else
      return x3 + v3 * (t - t3);
  }
  /**
   * @brief End time [s].
   */
  float t_end() const { return t3; }
  /**
   * @brief End velocity [m/s].
   */
  float v_end() const { return v3; }
  /**
   * @brief End position [m].
   */
  float x_end() const { return x3; }
  /**
   * @brief Start time of the curved acceleration [s].
   */
  float t_0() const { return t0; }
  /**
   * @brief Start time of the constant-acceleration section [s].
   */
  float t_1() const { return t1; }
  /**
   * @brief End time of the constant-acceleration section [s].
   */
  float t_2() const { return t2; }
  /**
   * @brief End time of the curved acceleration [s].
   */
  float t_3() const { return t3; }
  /**
   * @brief All boundary timestamps.
   */
  const std::array<float, 4> getTimeStamps() const {
    return {{t0, t1, t2, t3}};
  }
  /**
   * @brief Print the trajectory as CSV to a std::ostream.
   */
  void printCsv(std::ostream& os, const float t_interval = 1e-3f) const {
    for (float t = t0; t < t_end(); t += t_interval) {
      os << t << "," << j(t) << "," << a(t) << "," << v(t) << "," << x(t)
         << std::endl;
    }
  }
  /**
   * @brief Print the object information.
   */
  friend std::ostream& operator<<(std::ostream& os, const AccelCurve& obj) {
    os << "AccelCurve ";
    os << "\tvs: " << obj.v0;
    os << "\tve: " << obj.v3;
    os << "\tt0: " << obj.t0;
    os << "\tt1: " << obj.t1;
    os << "\tt2: " << obj.t2;
    os << "\tt3: " << obj.t3;
    os << "\td: " << obj.x3 - obj.x0;
    return os;
  }

 public:
  /**
   * @brief End velocity reachable under the distance constraint.
   * @param[in] j_max Magnitude of maximum jerk [m/s/s/s], must be
   * positive.
   * @param[in] a_max Magnitude of maximum acceleration [m/s/s], must be
   * positive.
   * @param[in] vs    Start velocity [m/s].
   * @param[in] vt    Target velocity [m/s].
   * @param[in] d     Travel distance [m].
   * @return ve       End velocity [m/s].
   */
  static float calcReachableVelocityEnd(const float j_max, const float a_max,
                                        const float vs, const float vt,
                                        const float d) {
    /* duration of the curved velocity section */
    const auto tc = a_max / j_max;
    /* sign of max acceleration */
    const auto am = (vt > vs) ? a_max : -a_max;
    const auto jm = (vt > vs) ? j_max : -j_max;
    /* branch on whether constant acceleration occurs */
    const auto d_triangle = (vs + am * tc / 2) * tc;  //< distance @ tm == 0
    const auto v_triangle = jm / am * d - vs;         //< v_end @ tm == 0
    // ctrl_logd << "d_tri: " << d_triangle << std::endl;
    // ctrl_logd << "v_tri: " << v_triangle << std::endl;
    if (d * v_triangle > 0 && std::abs(d) > std::abs(d_triangle)) {
      /* curve - straight - curve */
      ctrl_logd << "v: curve - straight - curve" << std::endl;
      /* solve the quadratic equation */
      const auto amtc = am * tc;
      const auto D = amtc * amtc - 4 * (amtc * vs - vs * vs - 2 * am * d);
      const auto sqrtD = std::sqrt(D);
      return (-amtc + (d > 0 ? sqrtD : -sqrtD)) / 2;
    }
    /* curve - curve (travel distance too short) */
    /* solve the cubic equation for the end velocity;
     * for simplicity, convert all values to positive, then reapply
     * the sign to the result */
    const auto a = std::abs(vs);
    const auto b = (d > 0 ? 1 : -1) * jm * d * d;
    const auto aaa_27 = a * a * a / 27;
    const auto cr = 8 * aaa_27 + b / 2;
    const auto ci_b = 8 * aaa_27 / b + 1.0f / 4;
    if (ci_b >= 0) {
      /* non-negative radicand: solve via cube root */
      ctrl_logd << "v: curve - curve (accel)" << std::endl;
      const auto c = std::cbrt(cr + std::abs(b) * std::sqrt(ci_b));
      return (d > 0 ? 1 : -1) * (c + 4 * a * a / c / 9 - a / 3);
    } else {
      /* negative radicand: solve via polar conversion */
      ctrl_logd << "v: curve - curve (decel)" << std::endl;
      const auto ci = std::abs(b) * std::sqrt(-ci_b);
      const auto r = std::hypot(cr, ci);  //< = sqrt(cr^2 + ci^2)
      const auto th = std::atan2(ci, cr);
      return (d > 0 ? 1 : -1) * (2 * std::cbrt(r) * std::cos(th / 3) - a / 3);
    }
  }
  /**
   * @brief Maximum velocity reachable under the distance constraint.
   * @param[in] j_max Magnitude of maximum jerk [m/s/s/s], must be
   * positive.
   * @param[in] a_max Magnitude of maximum acceleration [m/s/s], must be
   * positive.
   * @param[in] vs    Start velocity [m/s].
   * @param[in] ve    End velocity [m/s].
   * @param[in] d     Travel distance [m].
   * @return vm       Maximum velocity [m/s].
   */
  static float calcReachableVelocityMax(const float j_max, const float a_max,
                                        const float vs, const float ve,
                                        const float d) {
    /* duration of the curved velocity section */
    const auto tc = a_max / j_max;
    const auto am = (d > 0) ? a_max : -a_max;  //< direction of travel
    /* solve the quadratic equation */
    const auto amtc = am * tc;
    const auto D = amtc * amtc - 2 * (vs + ve) * amtc + 4 * am * d +
                   2 * (vs * vs + ve * ve);
    if (D < 0) {
      /* inconsistent constraints */
      ctrl_loge << "Error! D = " << D << " < 0" << std::endl;
      /* input check */
      if (vs * ve < 0)
        ctrl_loge << "Invalid Input! vs: " << vs << ", ve: " << ve << std::endl;
      return vs;
    }
    const auto sqrtD = std::sqrt(D);
    return (-amtc + (d > 0 ? sqrtD : -sqrtD)) / 2;  //< quadratic solution
  }
  /**
   * @brief Displacement reachable under the velocity-difference
   * constraint.
   * @param[in] j_max   Magnitude of maximum jerk [m/s/s/s], must be
   * positive.
   * @param[in] a_max   Magnitude of maximum acceleration [m/s/s], must
   * be positive.
   * @param[in] v_start Start velocity [m/s].
   * @param[in] v_end   End velocity [m/s].
   * @return d          Displacement [m].
   */
  static float calcDistanceFromVelocityStartToEnd(const float j_max,
                                                  const float a_max,
                                                  const float v_start,
                                                  const float v_end) {
    /* cache */
    const auto ve_minus_vs = v_end - v_start;
    /* assign with sign */
    const auto am = (ve_minus_vs > 0) ? a_max : -a_max;
    const auto jm = (ve_minus_vs > 0) ? j_max : -j_max;
    /* duration of the curved velocity section */
    const auto tc = a_max / j_max;
    /* duration of the constant-acceleration section */
    const auto tm = ve_minus_vs / am - tc;
    /* duration from start to end */
    const auto t_all =
        (tm > 0) ? (tc + tm + tc) : (2 * std::sqrt(ve_minus_vs / jm));
    return (v_start + v_end) / 2 * t_all;  //< area of the velocity graph
  }

 protected:
  float jm;             /**< @brief Jerk constant [m/s/s/s]. */
  float am;             /**< @brief Acceleration constant [m/s/s]. */
  float t0, t1, t2, t3; /**< @brief Time constants [s]. */
  float v0, v1, v2, v3; /**< @brief Velocity constants [m/s]. */
  float x0, x1, x2, x3; /**< @brief Position constants [m]. */
};
}  // namespace ctrl