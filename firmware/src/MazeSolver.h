/**
 * @file MazeSolver.h
 * @brief High-level maze solver using MazeLib (Maze + StepMap).
 * Implements flood-fill exploration and shortest-path fast runs.
 * Persistence via LittleFS.
 */
#pragma once

#include <MazeLib/Maze.h>
#include <MazeLib/StepMap.h>
#include "utils/TaskBase.h"
#include "config.h"
#include "drivers/buzzer.h"
#include "drivers/imu_mpu6050.h"
#include "SpeedController.h"
#include "WallDetector.h"
#include "SearchRun.h"
#include "FastRun.h"
#include <LittleFS.h>
#include <vector>
#include <queue>
#include <string>

// Externs
class Button; extern Button btn;
class LED; extern LED led;
class Motor; extern Motor mt;
class Encoder; extern Encoder enc;
class Ultrasonic; extern Ultrasonic us;

using MazeLib::Maze;
using MazeLib::StepMap;
using MazeLib::Direction;
using MazeLib::Directions;
using MazeLib::WallRecord;
using MazeLib::Pose;

#define MAZE_GOAL_POS MazeLib::Positions{{7,7},{7,8},{8,7},{8,8}}
#define MAZE_BACKUP_PATH "/maze_backup.bin"

class MazeSolver : public TaskBase {
public:
  MazeSolver() : maze_(MAZE_GOAL_POS, MazeLib::Position(0, 0)), step_map_() {}

  void start(bool force_search = false) {
    force_search_ = force_search;
    terminate();
    running_ = true;
    createTask("MazeSolver", TASK_PRIO_MAZE_SOLVER, STACK_SIZE_LARGE);
  }

  void terminate() {
    deleteTask();
    sr.disable();
    fr.disable();
    running_ = false;
  }

  void forceBackToStart() { /* ... */ }
  bool isRunning() const { return running_; }
  void set_goal(const std::vector<MazeLib::Position>& goal) { maze_.setGoals(goal); }
  bool isComplete() const { return state_ == REACHED; }

  void print() {
    maze_.print(std::cout, 16);
    step_map_.print(maze_, cur_pos_, cur_dir_, std::cout);
  }

  bool backup() {
    File file = LittleFS.open(MAZE_BACKUP_PATH, FILE_WRITE);
    if (!file) return false;
    const auto& logs = maze_.getWallRecords();
    for (size_t i = backup_counter_; i < logs.size(); ++i) {
      file.write((uint8_t*)&logs[i], sizeof(logs[i]));
    }
    backup_counter_ = logs.size();
    file.close();
    bz.play(Buzzer::MAZE_BACKUP);
    return true;
  }

  bool restore() {
    File file = LittleFS.open(MAZE_BACKUP_PATH, FILE_READ);
    if (!file) return false;
    maze_.reset();
    backup_counter_ = 0;
    while (file.available()) {
      WallRecord wl;
      file.read((uint8_t*)&wl, sizeof(wl));
      MazeLib::Position p = wl.getPosition();
      Direction d = wl.getDirection();
      bool b = wl.data & 0x1;
      maze_.updateWall(p, d, b);
      backup_counter_++;
    }
    file.close();
    return true;
  }

private:
  Maze maze_;
  StepMap step_map_;
  MazeLib::Position cur_pos_ = {0, 0};
  Direction cur_dir_ = Direction::North;
  Directions next_dirs_known_, next_dirs_candidates_;
  bool force_search_ = false;
  bool running_ = false;
  int backup_counter_ = 0;

  enum SearchState { SEARCHING = 0, SEARCHING_ADDITIONALLY = 1, BACKING_TO_START = 2, REACHED = 3 };
  SearchState state_ = SEARCHING;

  void queueActions(const Directions& dirs) {
    int straight = 0;
    for (auto d : dirs) {
      Direction rel = Direction(d - cur_dir_);
      switch ((int)rel) {
        case 0: straight++; break; // Front
        case 2: // Left
          if (straight) { sr.set_action(SearchRun::GO_STRAIGHT, straight); straight = 0; }
          sr.set_action(SearchRun::TURN_LEFT_90); break;
        case 4: // Back
          if (straight) { sr.set_action(SearchRun::GO_STRAIGHT, straight); straight = 0; }
          sr.set_action(SearchRun::STOP);
          sr.waitForEnd(); sr.disable();
          backup();
          imu.calibration(true);
          sr.set_action(SearchRun::RETURN);
          sr.set_action(SearchRun::GO_HALF);
          sr.enable();
          return;
        case 6: // Right
          if (straight) { sr.set_action(SearchRun::GO_STRAIGHT, straight); straight = 0; }
          sr.set_action(SearchRun::TURN_RIGHT_90); break;
      }
      cur_pos_ = cur_pos_.next(d);
      cur_dir_ = d;
    }
    if (straight) sr.set_action(SearchRun::GO_STRAIGHT, straight);
  }

  int calcNextDirs() {
    next_dirs_known_.clear(); next_dirs_candidates_.clear();
    Pose end = step_map_.calcNextDirections(maze_, Pose(cur_pos_, cur_dir_),
                                             next_dirs_known_, next_dirs_candidates_);
    cur_pos_ = end.p; cur_dir_ = end.d;
    state_ = maze_.isWall(cur_pos_, cur_dir_) ? REACHED : SEARCHING;
    return state_;
  }

  int calcShortestDirs(bool diag_enabled = false) {
    Directions path = step_map_.calcShortestDirections(maze_, true, false);
    if (path.empty()) return 0;
    shortest_dirs_ = path;
    return 1;
  }

  const Directions& getNextDirs() const { return next_dirs_known_; }
  const Directions& getShortestDirs() const { return shortest_dirs_; }
  bool findNextDir(const MazeLib::Position& p, const Direction& d, Direction& out) const {
    for (auto nd : next_dirs_candidates_) {
      if (maze_.canGo(p, nd)) { out = nd; return true; }
    }
    return false;
  }

  void updateWall(const MazeLib::Position& p, const Direction& d,
                  bool left, bool front, bool right) {
    maze_.updateWall(p, d + Direction::Left, left);
    maze_.updateWall(p, d, front);
    maze_.updateWall(p, d + Direction::Right, right);
  }

  Directions shortest_dirs_;

  bool searchRun(bool is_start = true,
                 const MazeLib::Position& start = MazeLib::Position(0, 0),
                 const Direction& dir = Direction::North) {
    if (state_ != REACHED) maze_.resetLastWalls(5);
    cur_pos_ = start; cur_dir_ = dir;
    int res = calcNextDirs();
    if (is_start) {
      if (res == REACHED) return true;
      sr.set_action(SearchRun::START_STEP);
      cur_pos_ = start.next(dir); cur_dir_ = dir;
      maze_.resetLastWalls(5);
    }
    bz.play(Buzzer::CONFIRM); imu.calibration(); bz.play(Buzzer::CANCEL);
    sr.enable();
    while (1) {
      SearchState prev = state_;
      res = calcNextDirs();
      if (res == REACHED) break;
      sr.waitForEnd();

      updateWall(cur_pos_, cur_dir_, wd.wall[0], wd.wall[1], wd.wall[2]);
      bz.play(Buzzer::SHORT);

      Direction next_dir;
      if (!findNextDir(cur_pos_, cur_dir_, next_dir)) {
        bz.play(Buzzer::ERROR); sr.set_action(SearchRun::STOP);
        sr.waitForEnd(); sr.disable(); return false;
      }
      queueActions({next_dir});
    }
    sr.set_action(SearchRun::START_INIT);
    cur_pos_ = {0,0}; cur_dir_ = Direction::North;
    calcNextDirs();
    sr.waitForEnd(); sr.disable(); backup(); bz.play(Buzzer::COMPLETE);
    return true;
  }

  bool fastRun() {
    if (!calcShortestDirs(fr.V90Enabled)) { bz.play(Buzzer::ERROR); return false; }
    auto path = shortest_dirs_; path.erase(path.begin());
    Direction d = Direction::North;
    for (auto nd : path) {
      Direction rel = Direction(nd - d);
      switch ((int)rel) {
        case 0: fr.set_action(FastRun::FAST_GO_STRAIGHT); break;
        case 2: fr.set_action(FastRun::FAST_TURN_LEFT_90); break;
        case 6: fr.set_action(FastRun::FAST_TURN_RIGHT_90); break;
        default: return false;
      }
      d = nd;
    }
    fr.enable(); fr.waitForEnd(); fr.disable();
    readyToStartWait();
    sc.position.reset();
    sr.set_action(SearchRun::RETURN);
    sr.set_action(SearchRun::GO_HALF);
    return searchRun(false, cur_pos_.next(d + 4), d + 4);
  }

  void readyToStartWait(int wait_ms = 2000) {
    vTaskDelay(200 / portTICK_PERIOD_MS);
    for (int ms = 0; ms < wait_ms; ++ms) {
      vTaskDelay(1 / portTICK_PERIOD_MS);
      if (fabs(imu.accel.z) > 2 * 9806.65f) { bz.play(Buzzer::CANCEL); while(1) vTaskDelay(1000); }
    }
  }

  void task() override {
    if (!calcShortestDirs()) {
      if (!searchRun()) { while(1) vTaskDelay(1000); }
      fr.V90Enabled = false;
      if (!fastRun()) { while(1) vTaskDelay(1000); }
      readyToStartWait();
      fr.V90Enabled = true;
    }
    while (1) {
      if (!fastRun()) { while(1) vTaskDelay(1000); }
      fr.runParameter.curve_gain *= 1.1f;
      fr.runParameter.max_speed *= 1.21f;
      fr.runParameter.accel *= 1.1f;
      fr.runParameter.decel *= 1.1f;
      readyToStartWait();
    }
  }
};