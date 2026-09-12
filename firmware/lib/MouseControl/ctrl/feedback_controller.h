/**
 * @file feedback_controller.h
 * @brief Feedback controller class.
 *
 * Portions derived from micromouse-mouse-control (MIT License)
 * Copyright (c) Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#pragma once

/**
 * @brief Control-related namespace.
 */
namespace ctrl {

/**
 * @brief Feedback controller with first-order feedforward compensation.
 * @tparam T State variable type.
 */
template <typename T>
class FeedbackController {
 public:
  /**
   * @brief First-order model used for the feedforward component.
   *
   * When unused, set $ K_1 = 1,~ T_1 = 0 $.
   * Transfer function $ y(s) = \\frac{K_1}{T_1s+1} u(s) $
   */
  struct Model {
    T K1; /**< @brief Steady-state gain (set to 1 when unused). */
    T T1; /**< @brief Time constant (set to 0 when unused). */
  };
  /**
   * @brief PID gains used for the feedback component.
   *         Set unused components to 0.
   *
   * Transfer function $ u(s) = K_p e(s) + K_i / s e(s) + K_d s e(s) $,
   * where $ e(s) := r(s) - y(s) $
   */
  struct Gain {
    T Kp; /**< @brief Proportional gain. */
    T Ki; /**< @brief Integral gain. */
    T Kd; /**< @brief Derivative gain. */
  };
  /**
   * @brief Breakdown of the control input components.
   * @details Used for visualizing gain tuning.
   */
  struct Breakdown {
    T ff;  /**< @brief Feedforward component. */
    T fb;  /**< @brief Feedback component. */
    T fbp; /**< @brief Proportional feedback component. */
    T fbi; /**< @brief Integral feedback component. */
    T fbd; /**< @brief Derivative feedback component. */
    T u;   /**< @brief Sum of all components. */
  };

 public:
  /**
   * @brief Constructor.
   *
   * @param[in] M Feedforward model.
   * @param[in] G Feedback gain.
   */
  FeedbackController(const Model& M, const Gain& G) : M(M), G(G) { reset(); }
  /**
   * @brief Reset the integral term.
   */
  void reset() {
    e_int = T();
    bd = Breakdown();
  }
  /**
   * @brief Update the state and compute the next control input.
   *
   * @param[in] r  Reference value.
   * @param[in] y  Measured value.
   * @param[in] dr Reference derivative.
   * @param[in] dy Measured derivative.
   * @param[in] Ts Discrete sampling period.
   * @return Control input for the next step.
   */
  const T& update(const T& r, const T& y, const T& dr, const T& dy,
                  const float Ts) {
    /* feedforward signal */
    bd.ff = (M.T1 * dr + r) / M.K1;
    /* feedback signal */
    bd.fbp = G.Kp * (r - y);
    bd.fbi = G.Ki * e_int;
    bd.fbd = G.Kd * (dr - dy);
    bd.fb = bd.fbp + bd.fbi + bd.fbd;
    /* calculate control input value */
    bd.u = bd.ff + bd.fb;
    /* integrate error */
    e_int += (r - y) * Ts;
    /* complete */
    return bd.u;
  }
  /**
   * @brief Get the error integral.
   */
  const T& getErrorIntegral() const { return e_int; }
  /**
   * @brief Get the feedforward model.
   */
  const Model& getModel() const { return M; }
  /**
   * @brief Set the feedforward model.
   */
  void setModel(const Model& model) { M = model; }
  /**
   * @brief Get the feedback gain.
   */
  const Gain& getGain() const { return G; }
  /**
   * @brief Set the feedback gain.
   */
  void setGain(const Gain& gain) { G = gain; }
  /**
   * @brief Get the control input breakdown.
   */
  const Breakdown& getBreakdown() const { return bd; }

 protected:
  Model M;      /**< @brief Feedforward model. */
  Gain G;       /**< @brief Feedback gain. */
  Breakdown bd; /**< @brief Control input breakdown. */
  T e_int;      /**< @brief Integrated tracking error. */
};

};  // namespace ctrl