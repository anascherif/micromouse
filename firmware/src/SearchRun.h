/**
 * @file SearchRun.h
 * @brief Search run executor with wall-following and slalom turns.
 * Uses MouseControl::ctrl::slalom for smooth 90° turns.
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
#include <cmath>

// Forward declarations for externs
class Motor; extern Motor mt;
class IMU; extern IMU imu;
class Encoder; extern Encoder enc;
class Ultrasonic; extern Ultrasonic us;
class SpeedController; extern SpeedController sc;
class WallDetector; extern WallDetector wd;
class Buzzer; extern Buzzer bz;
class LED; extern LED led;
class Button; extern Button btn;

#define SEARCH_END_REMAIN 3
#define SEARCH_ST_LOOK_AHEAD(v) (6.0f + 2.0f * v / 100.0f)
#define SEARCH_ST_FB_GAIN 20.0f
#define SEARCH_CURVE_FB_GAIN 5.0f

#define SEARCH_RUN_VELOCITY 300.0f   // mm/s
#define SEARCH_RUN_V_CURVE 240.0f    // mm/s
#define SEARCH_RUN_V_MAX 600.0f      // mm/s

// Search trajectory base class
class SearchTrajectory {
public:
  SearchTrajectory() { reset(); }
  virtual ~SearchTrajectory() {}
  void reset() { last_index_ = -SEARCH_END_REMAIN; }
  virtual float velocity() const = 0;
  virtual float straight() const = 0;

  Position getNextDir(const Position& cur, float v) {
    int idx = getNextIndex(cur);
    Position dir = (getPosition(idx + 3) - cur).rotate(-cur.theta);
    float dt = 1.0f / v;
    float ff = (getPosition(last_index_ + 1).theta - getPosition(last_index_).theta) / dt;
    dir.theta = ff + SEARCH_CURVE_FB_GAIN * atan2f(dir.y, dir.x);
    return dir;
  }
  float getRemain() const { return (getSize() - last_index_) * interval_; }
  Position getEndPosition() const { return getPosition(getSize()); }

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

// S90: 90-degree slalom turn using MouseControl
#if USE_SLALOM_TURNS
#include <ctrl/slalom/slalom.h>
#include <ctrl/slalom/trajectory.h>
#include <ctrl/pose.h>
#include <ctrl/state.h>

class S90 : public SearchTrajectory {
public:
  S90(bool mirror = false) : mirror_(mirror) {
    using namespace ctrl;
    using namespace ctrl::slalom;

    // Total pose for 90° turn: move 1 cell forward + 90° turn
    Pose total;
    total.x = 0;
    total.y = 0;
    total.th = M_PI / 2; // 90°

    // Curve end y-offset (lateral displacement during turn)
    // For 180 mm cell, typical slalom y-offset ≈ 60-80 mm
    float y_curve_end = 0.065f; // 65 mm in meters
    float x_adv = 0;

    Shape shape(total, y_curve_end, x_adv);
    trajectory_ = std::make_unique<Trajectory>(shape, mirror_);
    trajectory_->reset(SEARCH_RUN_V_CURVE / 1000.0f); // m/s

    // Pre-compute trajectory points for SearchTrajectory interface
    computePoints();
  }

  float velocity() const override { return SEARCH_RUN_V_CURVE; }
  float straight() const override { return straight_; }

protected:
  int getSize() const override { return points_.size(); }
  Position getPosition(int idx) const override {
    if (idx < 0) return {interval_ * idx, 0, 0};
    if (idx >= (int)points_.size()) {
      Position end = points_.back();
      return {end.x + (idx - points_.size() + 1) * interval_ * cos(end.theta),
              end.y + (idx - points_.size() + 1) * interval_ * sin(end.theta),
              end.theta};
    }
    return points_[idx];
  }

private:
  bool mirror_;
  float straight_ = 0;
  std::unique_ptr<ctrl::slalom::Trajectory> trajectory_;
  std::vector<Position> points_;

  void computePoints() {
    using namespace ctrl;
    State s{};
    float t = 0;
    const float Ts = 0.001f; // 1 ms
    const float v = trajectory_->getVelocity();
    straight_ = trajectory_->getShape().straight_prev;

    // Add straight approach points
    for (float x = 0; x < straight_; x += interval_) {
      points_.push_back({x, 0, 0});
    }

    // Add curve points
    while (t < trajectory_->getTimeCurve()) {
      trajectory_->update(s, t, Ts);
      points_.push_back({s.q.x, s.q.y, s.q.th});
      t += Ts;
    }

    // Add straight exit points
    Pose end = trajectory_->getShape().total;
    float straight_post = trajectory_->getShape().straight_post;
    for (float d = interval_; d <= straight_post; d += interval_) {
      Position p = {end.x + d * cos(end.th), end.y + d * sin(end.th), end.th};
      points_.push_back(p);
    }

    if (mirror_) {
      for (auto& p : points_) p = p.mirror_x();
    }
  }
};
#else
// Simple in-place pivot turn (fallback)
class S90 : public SearchTrajectory {
public:
  S90(bool mirror = false) : mirror_(mirror) {}
  float velocity() const override { return 0; }
  float straight() const override { return 0; }
protected:
  int getSize() const override { return 1; }
  Position getPosition(int) const override { return {0, 0, mirror_ ? -M_PI/2 : M_PI/2}; }
  bool mirror_;
};
#endif

// SearchRun task
class SearchRun : public TaskBase {
public:
  enum Action {
    START_STEP, START_INIT, GO_STRAIGHT, GO_HALF,
    TURN_LEFT_90, TURN_RIGHT_90, TURN_BACK, RETURN, STOP
  };
  struct Operation { Action action; int num; };

  SearchRun() {}
  void enable() {
    printf("SearchRun Enabled\n");
    deleteTask();
    createTask("SearchRun", TASK_PRIO_SEARCH_RUN, STACK_SIZE_LARGE);
  }
  void disable() {
    deleteTask();
    sc.disable();
    while (!q_.empty()) q_.pop();
    printf("SearchRun Disabled\n");
  }
  void set_action(Action a, int n = 1) {
    q_.push({a, n});
  }
  int actions() const { return q_.size(); }
  void waitForEnd() const {
    while (actions()) vTaskDelay(1);
  }

private:
  std::queue<Operation> q_;
  Position origin_;
  bool prev_wall_[2] = {false, false};

  void wall_attach() {
#if SEARCH_WALL_ATTACH_ENABLED
    if (wd.distance_mm[1] < 90) { // front wall close
      TickType_t last = xTaskGetTickCount();
      while (1) {
        vTaskDelayUntil(&last, 1 / portTICK_RATE_MS);
        float gain = 0.2f, satu = 60.0f, end = 5.0f;
        WheelParameter wp;
        wp.wheel[0] = -std::max(std::min(wd.distance_mm[0] * gain, satu), -satu);
        wp.wheel[1] = -std::max(std::min(wd.distance_mm[2] * gain, satu), -satu);
        wp.wheel2pole();
        if (fabs(wp.wheel[0]) + fabs(wp.wheel[1]) < end) break;
        sc.set_target(wp.trans, wp.rot);
      }
      sc.set_target(0, 0);
      sc.position.x = 0; sc.position.theta = 0;
      bz.play(Buzzer::SHORT);
    }
#endif
  }

  void wall_avoid(float distance) {
#if SEARCH_WALL_AVOID_ENABLED
    if (fabs(sc.position.theta) < 0.05 * M_PI) {
      float gain = 0.0002f, satu = 0.2f;
      if (wd.wall[0]) sc.position.y += std::max(std::min((wd.distance_mm[0] - 90) * gain, satu), -satu);
      if (wd.wall[2]) sc.position.y -= std::max(std::min((wd.distance_mm[2] - 90) * gain, satu), -satu);
    }
#endif
#if SEARCH_WALL_CUT_ENABLED
    for (int i = 0; i < 2; ++i) {
      if (prev_wall_[i] && !wd.wall[i] && sc.position.x > 30) {
        if (distance > 89) sc.position.x = sc.position.x - fmod(sc.position.x, 90) + 76;
      }
      if (!prev_wall_[i] && wd.wall[i] && sc.position.x > 30) {
        if (distance > 89) sc.position.x = sc.position.x - fmod(sc.position.x, 90) + 64;
      }
      prev_wall_[i] = wd.wall[i];
    }
#endif
  }

  void wall_calib(float velocity) {
#if SEARCH_WALL_FRONT_ENABLED
    if (wd.wall[1]) {
      float value = wd.distance_mm[1] - (10 + 0) / 1000.0f * velocity;
      if (value > 60 && value < 120) sc.position.x = 90 - value;
      if (sc.position.x > 0) sc.position.x = 0;
    }
#endif
  }

  void turn(float angle) {
    float speed = 3 * M_PI, accel = 36 * M_PI, decel = 24 * M_PI, back_gain = 2.0f;
    int ms = 0;
    TickType_t last = xTaskGetTickCount();
    // Accelerate
    while (1) {
      vTaskDelayUntil(&last, 1 / portTICK_RATE_MS);
      if (fabs(sc.actual.rot) > speed) break;
      float delta = sc.position.x * cos(-sc.position.theta) - sc.position.y * sin(-sc.position.theta);
      sc.set_target(-delta * back_gain, (angle > 0 ? 1 : -1) * ms / 1000.0f * accel);
      ms++;
    }
    // Decelerate
    while (1) {
      vTaskDelayUntil(&last, 1 / portTICK_RATE_MS);
      float extra = angle - sc.position.theta;
      if (fabs(sc.actual.rot) < 0.1 && fabs(extra) < 0.1) break;
      float target_speed = sqrt(2 * decel * fabs(extra));
      target_speed = std::min(target_speed, speed);
      float delta = sc.position.x * cos(-sc.position.theta) - sc.position.y * sin(-sc.position.theta);
      sc.set_target(-delta * back_gain, (extra > 0 ? 1 : -1) * target_speed);
    }
    sc.set_target(0, 0);
    sc.position.theta -= angle;
    sc.position = sc.position.rotate(-angle);
  }

  void straight_x(float dist, float v_max, float v_end) {
    float accel = 3000, decel = 2000;
    int ms = 0;
    float v_start = sc.actual.trans;
    float T = 1.5f * (v_max - v_start) / accel;
    TickType_t last = xTaskGetTickCount();
    prev_wall_[0] = wd.wall[0]; prev_wall_[1] = wd.wall[2];

    while (1) {
      vTaskDelayUntil(&last, 1 / portTICK_RATE_MS);
      Position cur = sc.position;
      if (v_end >= 1 && cur.x > dist - SEARCH_END_REMAIN) break;
      if (v_end < 1 && cur.x > dist - 1) break;
      float extra = dist - cur.x - SEARCH_END_REMAIN;
      float v_a = v_start + (v_max - v_start) * 6.0f *
                  (-1.0f/3 * pow(ms/1000.0f/T, 3) + 0.5f * pow(ms/1000.0f/T, 2));
      float v_d = sqrt(2 * decel * fabs(extra) + v_end * v_end);
      float v = std::min(v_max, std::min(v_a, v_d));
      float theta = atan2f(-cur.y, SEARCH_ST_LOOK_AHEAD(v)) - cur.theta;
      sc.set_target(v, SEARCH_ST_FB_GAIN * theta);
      wall_avoid(dist);
      ms++;
    }
    sc.set_target(v_end, 0);
    sc.position.x -= dist;
  }

  template<class T> void trace(T& tr, float v) {
    TickType_t last = xTaskGetTickCount();
    while (tr.getRemain() >= SEARCH_END_REMAIN) {
      vTaskDelayUntil(&last, 1 / portTICK_RATE_MS);
      Position dir = tr.getNextDir(sc.position, v);
      sc.set_target(v, dir.theta);
    }
    sc.set_target(v, 0);
    Position end = tr.getEndPosition();
    sc.position = (sc.position - end).rotate(-end.theta);
  }

  void put_back() {
    for (int i = 0; i < 150; ++i) {
      sc.set_target(-i, -sc.position.theta * 200); vTaskDelay(1);
    }
    for (int i = 0; i < 100; ++i) { sc.set_target(-150, -sc.position.theta * 200); vTaskDelay(1); }
    sc.disable(); mt.drive(-100, -100); vTaskDelay(200); sc.enable(true);
  }

  void uturn() {
    if (imu.angle > 0) { wall_attach(); turn(-M_PI/2); wall_attach(); turn(-M_PI/2); }
    else { wall_attach(); turn(M_PI/2); wall_attach(); turn(M_PI/2); }
  }

  void task() override {
    sc.enable();
    TickType_t last = xTaskGetTickCount();
    while (1) {
      // Wait for action
      while (q_.empty()) {
        vTaskDelayUntil(&last, 1 / portTICK_RATE_MS);
        Position cur = sc.position;
        float theta = atan2f(-cur.y, SEARCH_ST_LOOK_AHEAD(SEARCH_RUN_VELOCITY)) - cur.theta;
        sc.set_target(SEARCH_RUN_VELOCITY, SEARCH_ST_FB_GAIN * theta);
        wall_avoid(0);
      }

      Operation op = q_.front();
      q_.pop();
      printf("Action: %d\n", op.action);

      switch (op.action) {
        case START_STEP:
          sc.position.reset();
          straight_x(SEGMENT_WIDTH - MACHINE_TAIL_LENGTH - WALL_THICKNESS/2, SEARCH_RUN_VELOCITY, SEARCH_RUN_VELOCITY);
          break;
        case START_INIT:
          straight_x(SEGMENT_WIDTH/2, SEARCH_RUN_VELOCITY, 0);
          wall_attach(); turn(M_PI/2); wall_attach(); turn(M_PI/2);
          put_back(); mt.free(); while(!q_.empty()) q_.pop(); while(1) vTaskDelay(1000);
        case GO_STRAIGHT:
          straight_x(SEGMENT_WIDTH * op.num, SEARCH_RUN_V_MAX, SEARCH_RUN_VELOCITY); break;
        case GO_HALF:
          straight_x(SEGMENT_WIDTH/2 * op.num, SEARCH_RUN_VELOCITY, SEARCH_RUN_VELOCITY); break;
        case TURN_LEFT_90:
          for (int i=0;i<op.num;++i) {
            S90 tr(false);
            wall_calib(SEARCH_RUN_VELOCITY);
            straight_x(tr.straight(), SEARCH_RUN_VELOCITY, tr.velocity());
            trace(tr, tr.velocity());
            straight_x(tr.straight(), SEARCH_RUN_VELOCITY, SEARCH_RUN_VELOCITY);
          } break;
        case TURN_RIGHT_90:
          for (int i=0;i<op.num;++i) {
            S90 tr(true);
            wall_calib(SEARCH_RUN_VELOCITY);
            straight_x(tr.straight(), SEARCH_RUN_VELOCITY, tr.velocity());
            trace(tr, tr.velocity());
            straight_x(tr.straight(), SEARCH_RUN_VELOCITY, SEARCH_RUN_VELOCITY);
          } break;
        case TURN_BACK:
          straight_x(SEGMENT_WIDTH/2, SEARCH_RUN_VELOCITY, 0);
          uturn();
          straight_x(SEGMENT_WIDTH/2, SEARCH_RUN_VELOCITY, SEARCH_RUN_VELOCITY); break;
        case RETURN: uturn(); break;
        case STOP:
          straight_x(SEGMENT_WIDTH/2, SEARCH_RUN_VELOCITY, 0);
          wall_attach(); sc.disable(); while(!q_.empty()) q_.pop(); while(1) vTaskDelay(1000);
      }
    }
  }
};