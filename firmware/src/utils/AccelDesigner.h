/**
 * @file AccelDesigner.h
 * @brief Trapezoidal/curved acceleration profile generator.
 * Ported from kerise-v3, compatible with MouseControl::ctrl::AccelDesigner.
 */
#pragma once

#include <cmath>
#include <algorithm>
#include <complex>

class AccelCurve {
public:
  AccelCurve() : am_(0), t0_(0), t1_(0), t2_(0), t3_(0),
                 v0_(0), v1_(0), v2_(0), v3_(0),
                 x0_(0), x1_(0), x2_(0), x3_(0), tc_(0), tm_(0) {}

  AccelCurve(float a_max, float v_start, float v_end) {
    reset(a_max, v_start, v_end);
  }

  void reset(float a_max, float v_start, float v_end) {
    tc_ = calcTimeCurve(a_max);
    am_ = (v_end > v_start) ? a_max : -a_max;
    v0_ = v_start; v3_ = v_end;
    t0_ = 0; x0_ = 0;
    tm_ = (v3_ - v0_) / am_ - tc_;
    if (tm_ > 0) {
      t1_ = t0_ + tc_;
      t2_ = t1_ + tm_;
      t3_ = t2_ + tc_;
    } else {
      t1_ = t0_ + std::sqrt(tc_ / am_ * (v3_ - v0_));
      t2_ = t1_;
      t3_ = t2_ + (t1_ - t0_);
    }
    v1_ = v(t1_); v2_ = v(t2_);
    x1_ = x(t1_); x2_ = x(t2_);
    x3_ = x0_ + (v0_ + v3_) / 2 * (t3_ - t0_);
  }

  float a(float t) const {
    if (t <= t0_) return 0;
    else if (t <= t1_) return am_ / tc_ * (t - t0_);
    else if (t <= t2_) return am_;
    else if (t <= t3_) return -am_ / tc_ * (t - t3_);
    else return 0;
  }

  float v(float t) const {
    if (t <= t0_) return v0_;
    else if (t <= t1_) return v0_ + 0.5f / tc_ * am_ * (t - t0_) * (t - t0_);
    else if (t <= t2_) return v1_ + am_ * (t - t1_);
    else if (t <= t3_) return v3_ - 0.5f / tc_ * am_ * (t - t3_) * (t - t3_);
    else return v3_;
  }

  float x(float t) const {
    if (t <= t0_) return x0_ + v0_ * (t - t0_);
    else if (t <= t1_) return x0_ + v0_ * (t - t0_) + am_ / 6 / tc_ * (t - t0_) * (t - t0_) * (t - t0_);
    else if (t <= t2_) return x1_ + v1_ * (t - t1_) + am_ / 2 * (t - t1_) * (t - t1_);
    else if (t <= t3_) return x3_ + v3_ * (t - t3_) - am_ / 6 / tc_ * (t - t3_) * (t - t3_) * (t - t3_);
    else return x3_ + v3_ * (t - t3_);
  }

  float t_end() const { return t3_; }
  float v_end() const { return v3_; }
  float x_end() const { return x3_; }

  static float calcTimeCurve(float am) {
    return std::abs(am) / 50000.0f;
  }

  static float calcVelocityEnd(float am, float vs, float vt, float d) {
    const float tc = calcTimeCurve(am);
    am = (vt > vs) ? std::abs(am) : -std::abs(am);
    const float tm = (vt - vs) / am - tc;
    float ve;
    if (tm > 0) {
      ve = (std::sqrt(4*vs*vs - 4*vs*am*tc + am*(tc*tc*am + 8*d)) - am*tc) / 2;
    } else {
      const float a = vs;
      const float b = am*d*d/tc;
      const float aaa = a*a*a;
      const float c0 = 27*(32*aaa*b + 27*b*b);
      const float c1 = 16*aaa + 27*b;
      if (c0 >= 0) {
        const float c2 = std::cbrt(std::sqrt(c0) + c1) / 2;
        ve = (c2 + 4*a*a/c2 - a) / 3;
      } else {
        const auto c2 = std::pow(std::complex<float>(c1/2, std::sqrt(-c0)/2), 1.0f/3);
        ve = (c2.real()*2 - a) / 3;
      }
    }
    return (vt > vs) ? std::min(vt, ve) : std::max(vt, ve);
  }

  static float calcVelocityMax(float am, float vs, float va, float ve, float d) {
    const float tc = calcTimeCurve(am);
    const float D = am*am*tc*tc - 2*(vs+ve)*am*tc + 4*am*d + 2*(vs*vs+ve*ve);
    const float vm = (-am*tc + std::sqrt(D)) / 2;
    return std::max({vm, vs, ve});
  }

private:
  float am_, t0_, t1_, t2_, t3_;
  float v0_, v1_, v2_, v3_;
  float x0_, x1_, x2_, x3_;
  float tc_, tm_;
};

class AccelDesigner {
public:
  AccelDesigner() : t1_(0), t2_(0), t3_(0), x0_(0), x3_(0) {}

  AccelDesigner(float a_max, float v_start, float v_sat, float v_target,
                float distance, float x_start = 0) {
    reset(a_max, v_start, v_sat, v_target, distance, x_start);
  }

  void reset(float a_max, float v_start, float v_sat, float v_target,
             float distance, float x_start = 0) {
    const float v_end = AccelCurve::calcVelocityEnd(a_max, v_start, v_target, distance);
    const float v_max = std::min(v_sat, AccelCurve::calcVelocityMax(a_max, v_start, v_sat, v_end, distance));
    ac_.reset(a_max, v_start, v_max);
    dc_.reset(a_max, v_max, v_end);
    x0_ = x_start;
    x3_ = x_start + distance;
    t1_ = ac_.t_end();
    t2_ = ac_.t_end() + (distance - ac_.x_end() - dc_.x_end()) / v_max;
    t3_ = ac_.t_end() + (distance - ac_.x_end() - dc_.x_end()) / v_max + dc_.t_end();
  }

  float a(float t) const { return (t < t2_) ? ac_.a(t) : dc_.a(t - t2_); }
  float v(float t) const { return (t < t2_) ? ac_.v(t) : dc_.v(t - t2_); }
  float x(float t) const { return (t < t2_) ? x0_ + ac_.x(t) : x3_ - dc_.x_end() + dc_.x(t - t2_); }

  float t_end() const { return t3_; }
  float v_end() const { return dc_.v_end(); }
  float x_end() const { return x3_; }

private:
  float t1_, t2_, t3_;
  float x0_, x3_;
  AccelCurve ac_, dc_;
};