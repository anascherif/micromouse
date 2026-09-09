/**
 * @file FastRun.h
 * @brief Fast run executor with slalom trajectories for shortest path.
 * Uses MouseControl::ctrl::slalom for smooth turns.
 */
#pragma once

#include "utils/TaskBase.h"
#include "config.h"
#include "utils/AccelDesigner.h"
#include "SpeedController.h"
#include <memory>
#include <algorithm>
#include <queue>
#include <vector>
#include <string>
#include <cmath>

// Externs
class Motor; extern Motor mt;
class IMU; extern IMU imu;
class Encoder; extern Encoder enc;
class SpeedController; extern SpeedController sc;
class WallDetector; extern WallDetector wd;
class Buzzer; extern Buzzer bz;
class Button; extern Button btn;

#define FAST_END_REMAIN 6
#define FAST_ST_LOOK_AHEAD(v) (6.0f + 10.0f * v / 100.0f)
#define FAST_ST_FB_GAIN 10.0f
#define FAST_CURVE_FB_GAIN 3.0f

struct RunParameter {
  float curve_gain = 0.8f;
  float max_speed = 800;  // mm/s
  float accel = 4000;     // mm/s²
  float decel = 4000;     // mm/s²
  RunParameter() {}
  RunParameter(float cg, float ms, float a, float d) : curve_gain(cg), max_speed(ms), accel(a), decel(d) {}
};

class FastTrajectory {
public:
  FastTrajectory() { reset(); }
  virtual ~FastTrajectory() {}
  void reset() { last_index_ = -FAST_END_REMAIN; }
  virtual float velocity() const = 0;
  Position getNextDir(const Position& cur, float v) {
    int idx = getNextIndex(cur);
    Position dir = (getPosition(idx + 6) - cur).rotate(-cur.theta);
    float dt = 1.0f / v;
    float ff = (getPosition(last_index_ + 1).theta - getPosition(last_index_).theta) / dt;
    dir.theta = ff + FAST_CURVE_FB_GAIN * atan2f(dir.y, dir.x);
    return dir;
  }
  float getRemain() const { return (getSize() - last_index_) * interval_; }
  Position getEndPosition() { return getPosition(getSize()); }

protected:
  int last_index_ = 0;
  const float interval_ = 1.0f;
  virtual int getSize() const = 0;
  virtual Position getPosition(int idx) const = 0;
  int getNextIndex(const Position& pos) {
    for (int i = last_index_; ; ++i) {
      Position t = getPosition(i);
      Position d = (t - pos).rotate(-t.theta);
      if (d.x > 0) { last_index_ = i; return last_index_; }
    }
  }
};

// F45: 45° slalom (diagonal)
#if USE_SLALOM_TURNS
#include <ctrl/slalom/slalom.h>
#include <ctrl/slalom/trajectory.h>
#include <ctrl/pose.h>
#include <ctrl/state.h>

class F45 : public FastTrajectory {
public:
  F45(bool mirror = false) : mirror_(mirror) {
    using namespace ctrl::slalom;
    // 45° turn: total rotation = 45°
    ctrl::Pose total;
    total.x = 0; total.y = 0; total.th = M_PI / 4;
    float y_curve_end = 0.045f; // 45 mm lateral
    Shape shape(total, y_curve_end);
    trajectory_ = std::make_unique<ctrl::slalom::Trajectory>(shape, mirror_);
    trajectory_->reset(0.8f); // 800 mm/s = 0.8 m/s
    computePoints();
  }
  float velocity() const override { return 800; }

protected:
  int getSize() const override { return points_.size(); }
  Position getPosition(int idx) const override {
    if (idx < 0) return {interval_ * idx, 0, 0};
    if (idx >= (int)points_.size()) {
      Position e = points_.back();
      return {e.x + (idx - points_.size() + 1) * interval_ * cos(e.theta),
              e.y + (idx - points_.size() + 1) * interval_ * sin(e.theta), e.theta};
    }
    return points_[idx];
  }
private:
  bool mirror_;
  std::unique_ptr<ctrl::slalom::Trajectory> trajectory_;
  std::vector<Position> points_;
  void computePoints() {
    ctrl::State s{};
    float t = 0, Ts = 0.001f, v = trajectory_->getVelocity();
    while (t < trajectory_->getTimeCurve()) {
      trajectory_->update(s, t, Ts);
      points_.push_back({s.q.x, s.q.y, s.q.th});
      t += Ts;
    }
    if (mirror_) for (auto& p : points_) p = p.mirror_x();
  }
};

// D90: 90° slalom for fast run
class D90 : public FastTrajectory {
public:
  D90(bool mirror = false) : mirror_(mirror) {
    using namespace ctrl::slalom;
    ctrl::Pose total;
    total.x = 0; total.y = 0; total.th = M_PI / 2;
    float y_curve_end = 0.075f; // 75 mm lateral
    Shape shape(total, y_curve_end);
    trajectory_ = std::make_unique<ctrl::slalom::Trajectory>(shape, mirror_);
    trajectory_->reset(0.8f);
    computePoints();
  }
  float velocity() const override { return 800; }

protected:
  int getSize() const override { return points_.size(); }
  Position getPosition(int idx) const override {
    if (idx < 0) return {interval_ * idx, 0, 0};
    if (idx >= (int)points_.size()) {
      Position e = points_.back();
      return {e.x + (idx - points_.size() + 1) * interval_ * cos(e.theta),
              e.y + (idx - points_.size() + 1) * interval_ * sin(e.theta), e.theta};
    }
    return points_[idx];
  }
private:
  bool mirror_;
  std::unique_ptr<ctrl::slalom::Trajectory> trajectory_;
  std::vector<Position> points_;
  void computePoints() {
    ctrl::State s{};
    float t = 0, Ts = 0.001f, v = trajectory_->getVelocity();
    while (t < trajectory_->getTimeCurve()) {
      trajectory_->update(s, t, Ts);
      points_.push_back({s.q.x, s.q.y, s.q.th});
      t += Ts;
    }
    if (mirror_) for (auto& p : points_) p = p.mirror_x();
  }
};

// Straight trajectory for fast run
class StraightTraj : public FastTrajectory {
public:
  StraightTraj(float len) : len_(len) {}
  float velocity() const override { return 800; }
protected:
  int getSize() const override { return (int)(len_ / interval_) + 1; }
  Position getPosition(int idx) const override {
    if (idx < 0) return {interval_ * idx, 0, 0};
    if (idx >= getSize()) return {len_, 0, 0};
    return {idx * interval_, 0, 0};
  }
private:
  float len_;
};
#else
// Simple fallback trajectories
class F45 : public FastTrajectory { public: F45(bool){} float velocity() const override {return 0;} protected: int getSize() const override {return 1;} Position getPosition(int) const override {return {0,0,0};}};
class D90 : public FastTrajectory { public: D90(bool){} float velocity() const override {return 0;} protected: int getSize() const override {return 1;} Position getPosition(int) const override {return {0,0,0};}};
class StraightTraj : public FastTrajectory { public: StraightTraj(float){} float velocity() const override {return 0;} protected: int getSize() const override {return 1;} Position getPosition(int) const override {return {0,0,0};}};
#endif

class FastRun : public TaskBase {
public:
  enum Action {
    FAST_GO_STRAIGHT,
    FAST_TURN_LEFT_45,
    FAST_TURN_RIGHT_45,
    FAST_TURN_LEFT_90,
    FAST_TURN_RIGHT_90,
    FAST_TURN_BACK,
    FAST_STOP
  };
  struct Operation { Action action; int num; };

  FastRun() {}
  RunParameter runParameter;
  bool V90Enabled = false;
  bool wallAvoidFlag = true, wallAvoid45Flag = true, wallCutFlag = true;

  void enable() {
    printf("FastRun Enabled\n");
    deleteTask();
    createTask("FastRun", TASK_PRIO_FAST_RUN, STACK_SIZE_LARGE);
  }
  void disable() {
    deleteTask(); sc.disable();
    while (!q_.empty()) q_.pop();
    printf("FastRun Disabled\n");
  }
  void set_action(Action a, int n = 1) { q_.push({a, n}); }
  void set_path(const std::string& path) {
    while (!q_.empty()) q_.pop();
    for (char c : path) {
      switch (c) {
        case 's': set_action(FAST_GO_STRAIGHT); break;
        case 'l': set_action(V90Enabled ? FAST_TURN_LEFT_90 : FAST_TURN_LEFT_45); break;
        case 'r': set_action(V90Enabled ? FAST_TURN_RIGHT_90 : FAST_TURN_RIGHT_45); break;
        case 'b': set_action(FAST_TURN_BACK); break;
      }
    }
  }
  int actions() const { return q_.size(); }
  void waitForEnd() const { while (actions()) vTaskDelay(1); }

private:
  std::queue<Operation> q_;
  float target_speed_ = 0;

  void task() override {
    sc.enable();
    TickType_t last = xTaskGetTickCount();

    while (1) {
      // Wait for action
      while (q_.empty()) {
        vTaskDelayUntil(&last, 1 / portTICK_RATE_MS);
        straight_x(0, runParameter.max_speed, runParameter.max_speed);
      }

      Operation op = q_.front();
      q_.pop();

      switch (op.action) {
        case FAST_GO_STRAIGHT: {
          float dist = SEGMENT_WIDTH * op.num;
          straight_x(dist, runParameter.max_speed, runParameter.max_speed);
          break;
        }
        case FAST_TURN_LEFT_45: {
          F45 tr(false);
          straight_x(0, tr.velocity(), tr.velocity());
          trace(tr, tr.velocity());
          break;
        }
        case FAST_TURN_RIGHT_45: {
          F45 tr(true);
          straight_x(0, tr.velocity(), tr.velocity());
          trace(tr, tr.velocity());
          break;
        }
        case FAST_TURN_LEFT_90: {
          D90 tr(false);
          straight_x(0, tr.velocity(), tr.velocity());
          trace(tr, tr.velocity());
          break;
        }
        case FAST_TURN_RIGHT_90: {
          D90 tr(true);
          straight_x(0, tr.velocity(), tr.velocity());
          trace(tr, tr.velocity());
          break;
        }
        case FAST_TURN_BACK: {
          uturn();
          break;
        }
        case FAST_STOP:
          straight_x(SEGMENT_WIDTH/2, runParameter.max_speed, 0);
          sc.disable();
          while (!q_.empty()) q_.pop();
          while (1) vTaskDelay(1000);
      }
    }
  }

  void straight_x(float dist, float v_max, float v_end) {
    AccelDesigner ad(runParameter.accel, sc.actual.trans, v_max, v_end, dist - FAST_END_REMAIN);
    TickType_t last = xTaskGetTickCount();
    for (float t = 0; t < ad.t_end() + 0.1f; t += 0.001f) {
      vTaskDelayUntil(&last, 1 / portTICK_RATE_MS);
      float v = ad.v(t);
      float theta = atan2f(-sc.position.y, FAST_ST_LOOK_AHEAD(v)) - sc.position.theta;
      sc.set_target(v, FAST_ST_FB_GAIN * theta);
    }
    sc.set_target(v_end, 0);
    sc.position.x -= dist;
  }

  template<class T> void trace(T& tr, float v) {
    TickType_t last = xTaskGetTickCount();
    while (tr.getRemain() >= FAST_END_REMAIN) {
      vTaskDelayUntil(&last, 1 / portTICK_RATE_MS);
      Position dir = tr.getNextDir(sc.position, v);
      sc.set_target(v, dir.theta);
    }
    sc.set_target(v, 0);
    Position end = tr.getEndPosition();
    sc.position = (sc.position - end).rotate(-end.theta);
  }

  void uturn() {
    if (imu.angle > 0) { turn(-M_PI/2); turn(-M_PI/2); }
    else { turn(M_PI/2); turn(M_PI/2); }
  }

  void turn(float angle) {
    float speed = 4 * M_PI, accel = 48 * M_PI, decel = 36 * M_PI, back_gain = 1.5f;
    int ms = 0;
    TickType_t last = xTaskGetTickCount();
    while (1) {
      vTaskDelayUntil(&last, 1 / portTICK_RATE_MS);
      if (fabs(sc.actual.rot) > speed) break;
      float delta = sc.position.x * cos(-sc.position.theta) - sc.position.y * sin(-sc.position.theta);
      sc.set_target(-delta * back_gain, (angle > 0 ? 1 : -1) * ms / 1000.0f * accel);
      ms++;
    }
    while (1) {
      vTaskDelayUntil(&last, 1 / portTICK_RATE_MS);
      float extra = angle - sc.position.theta;
      if (fabs(sc.actual.rot) < 0.1 && fabs(extra) < 0.1) break;
      float ts = sqrt(2 * decel * fabs(extra));
      ts = std::min(ts, speed);
      float delta = sc.position.x * cos(-sc.position.theta) - sc.position.y * sin(-sc.position.theta);
      sc.set_target(-delta * back_gain, (extra > 0 ? 1 : -1) * ts);
    }
    sc.set_target(0, 0);
    sc.position.theta -= angle;
    sc.position = sc.position.rotate(-angle);
  }
};