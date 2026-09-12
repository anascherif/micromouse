/**
 * @file StepMap.cpp
 * @brief Implementation of the StepMap class.
 *
 * Portions derived from micromouse-maze-library (MIT License)
 * Copyright (c) Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#include "MazeLib/StepMap.h"

#include <algorithm>  //< for std::sort
#include <cmath>      //< for std::sqrt
#include <iomanip>    //< for std::setw
#include <queue>

namespace MazeLib {

StepMap::StepMap() {
  calcStraightCostTable();
  reset();
}
void StepMap::print(const Maze& maze, const Position p, const Direction d,
                    std::ostream& os) const {
  return print(maze, {d}, p.next(d + Direction::Back), os);
}
void StepMap::print(const Maze& maze, const Directions& dirs,
                    const Position start, std::ostream& os) const {
  /* preparation */
  std::vector<Pose> path;
  path.reserve(dirs.size());
  Position p = start;
  for (const auto d : dirs) path.push_back({p, d}), p = p.next(d);
  const int mazeSize = MAZE_SIZE;
  step_t maxStep = 0;
  for (const auto step : stepMap)
    if (step != STEP_MAX) maxStep = std::max(maxStep, step);
  const bool simple = (maxStep < 999);
  const step_t scaler =
      stepTable[stepTableSize - 1] - stepTable[stepTableSize - 2];
  const auto find = [&](const WallIndex& i) {
    return std::find_if(path.cbegin(), path.cend(), [&](const Pose& pose) {
      return WallIndex(pose.p, pose.d) == i;
    });
  };
  /* start to draw maze */
  for (int8_t y = mazeSize; y >= 0; --y) {
    /* Vertical Wall Line */
    if (y != mazeSize) {
      for (uint8_t x = 0; x <= mazeSize; ++x) {
        /* Vertical Wall */
        const auto w = maze.isWall(x, y, Direction::West);
        const auto k = maze.isKnown(x, y, Direction::West);
        const auto it = find(WallIndex(Position(x, y), Direction::West));
        if (it != path.cend())
          os << C_YE "\e[1m" << it->d << C_NO;
        else
          os << (k ? (w ? "|" : " ") : (C_RE "." C_NO));
        /* Cell */
        if (x != mazeSize) {
          step_t step = getStep(x, y);
          step = std::min(999, simple ? step : step / scaler);
          os << (step == 0 ? C_YE : C_BL) << std::setw(3) << step << C_NO;
        }
      }
      os << "\e[0K" << std::endl;  // clear from cursor position to end of line
    }
    /* Horizontal Wall Line */
    for (uint8_t x = 0; x < mazeSize; ++x) {
      /* Pillar */
      os << '+';
      /* Horizontal Wall */
      const auto w = maze.isWall(x, y, Direction::South);
      const auto k = maze.isKnown(x, y, Direction::South);
      const auto it = find(WallIndex(Position(x, y), Direction::South));
      if (it != path.cend())
        os << C_YE "\e[1m " << it->d << " " C_NO;
      else
        os << (k ? (w ? "---" : "   ") : (C_RE " . " C_NO));
    }
    os << '+' << "\e[0K" << std::endl;
  }
}
void StepMap::printFull(const Maze& maze, const Position p, const Direction d,
                        std::ostream& os) const {
  return printFull(maze, {d}, p.next(d + Direction::Back), os);
}
void StepMap::printFull(const Maze& maze, const Directions& dirs,
                        const Position start, std::ostream& os) const {
  /* preparation */
  std::vector<Pose> path;
  path.reserve(dirs.size());
  Position p = start;
  for (const auto d : dirs) path.push_back({p, d}), p = p.next(d);
  const int mazeSize = MAZE_SIZE;
  const auto find = [&](const WallIndex& i) {
    return std::find_if(path.cbegin(), path.cend(), [&](const Pose& pose) {
      return WallIndex(pose.p, pose.d) == i;
    });
  };
  /* start to draw maze */
  for (int8_t y = mazeSize; y >= 0; --y) {
    /* Vertical Wall Line */
    if (y != mazeSize) {
      for (uint8_t x = 0; x <= mazeSize; ++x) {
        /* Vertical Wall */
        const auto w = maze.isWall(x, y, Direction::West);
        const auto k = maze.isKnown(x, y, Direction::West);
        const auto it = find(WallIndex(Position(x, y), Direction::West));
        if (it != path.cend())
          os << C_YE "\e[1m" << it->d << C_NO;
        else
          os << (k ? (w ? "|" : " ") : (C_RE "." C_NO));
        /* Cell */
        if (x != mazeSize) {
          auto step = std::min((step_t)99999, getStep(x, y));
          os << (step == 0 ? C_YE : C_BL) << std::setw(5) << step << C_NO;
        }
      }
      os << std::endl;
    }
    /* Horizontal Wall Line */
    for (uint8_t x = 0; x < mazeSize; ++x) {
      /* Pillar */
      os << '+';
      /* Horizontal Wall */
      const auto w = maze.isWall(x, y, Direction::South);
      const auto k = maze.isKnown(x, y, Direction::South);
      const auto it = find(WallIndex(Position(x, y), Direction::South));
      if (it != path.cend())
        os << C_YE "\e[1m  " << it->d << "  " C_NO;
      else
        os << (k ? (w ? "-----" : "     ") : (C_RE "  .  " C_NO));
    }
    os << '+' << std::endl;
  }
}
void StepMap::update(const Maze& maze, const Positions& dest,
                     const bool knownOnly, const bool simple) {
  MAZE_DEBUG_PROFILING_START(0)
  /* limit the search bounds for speed */
  int8_t min_x = maze.getMinX();
  int8_t max_x = maze.getMaxX();
  int8_t min_y = maze.getMinY();
  int8_t max_y = maze.getMaxY();
  for (const auto p : dest) {  //< goals must be included
    min_x = std::min(p.x, min_x);
    max_x = std::max(p.x, max_x);
    min_y = std::min(p.y, min_y);
    max_y = std::max(p.y, max_y);
  }
  min_x -= 1, min_y -= 1, max_x += 2, max_y += 2;  //< allow rim cells
  /* set every cell to the maximum step */
  reset();
  /* priority queue for step updates */
#define STEP_MAP_USE_PRIORITY_QUEUE 1
#if STEP_MAP_USE_PRIORITY_QUEUE
  struct Element {
    Position p;
    step_t s;
    bool operator<(const Element& e) const { return s > e.s; }
  };
  std::priority_queue<Element> q;
#else
  std::queue<Position> q;
#endif
  /* dest cells get step 0 */
  for (const auto p : dest)
    if (p.isInsideOfField())
#if STEP_MAP_USE_PRIORITY_QUEUE
      setStep(p, 0), q.push({p, 0});
#else
      setStep(p, 0), q.push(p);
#endif
  /* relax until no step improves */
  while (!q.empty()) {
#if MAZE_DEBUG_PROFILING
    queueSizeMax = std::max(queueSizeMax, static_cast<int>(q.size()));
#endif
    /* pop the focus cell */
#if STEP_MAP_USE_PRIORITY_QUEUE
    const auto focus = q.top().p;
    const auto focus_step_q = q.top().s;
#else
    const auto focus = q.front();
#endif
    q.pop();
    /* skip cells outside the limited range */
    if (focus.x > max_x || focus.y > max_y || focus.x < min_x ||
        focus.y < min_y)
      continue;
    const auto focus_step = stepMap[focus.getIndex()];
#if STEP_MAP_USE_PRIORITY_QUEUE
    /* stale queue entry */
    if (focus_step < focus_step_q) continue;
#endif
    /* scan the four directions */
    for (const auto d : Direction::Along4()) {
      /* update as far straight as possible */
      auto next = focus;
      for (int8_t i = 1;; ++i) {
        /* stop at a wall or (when knownOnly) an unknown wall */
        const auto next_wi = WallIndex(next, d);
        if (maze.isWall(next_wi) || (knownOnly && !maze.isKnown(next_wi)))
          break;
        next = next.next(d);  //< advance
        /* step value including straight-line acceleration */
        const step_t next_step = focus_step + (simple ? i : stepTable[i]);
        const auto next_index = next.getIndex();
        if (stepMap[next_index] <= next_step) break;  //< no improvement
        stepMap[next_index] = next_step;              //< update
        /* enqueue for further propagation */
#if STEP_MAP_USE_PRIORITY_QUEUE
        q.push({next, next_step});
#else
        q.push(next);
#endif
      }
    }
  }
  MAZE_DEBUG_PROFILING_END(0)
}
Directions StepMap::calcShortestDirections(const Maze& maze,
                                           const Position start,
                                           const Positions& dest,
                                           const bool knownOnly,
                                           const bool simple) {
  /* update the step map */
  update(maze, dest, knownOnly, simple);
  Pose end;
  const auto shortestDirections = getStepDownDirections(
      maze, {start, Direction::Max}, end, knownOnly, simple, false);
  /* goal check */
  return stepMap[end.p.getIndex()] == 0 ? shortestDirections : Directions{};
}
Pose StepMap::calcNextDirections(const Maze& maze, const Pose& start,
                                 Directions& nextDirectionsKnown,
                                 Directions& nextDirectionCandidates) const {
  Pose end;
  nextDirectionsKnown =
      getStepDownDirections(maze, start, end, false, false, true);
  nextDirectionCandidates = getNextDirectionCandidates(maze, end);
  return end;
}
Directions StepMap::getStepDownDirections(const Maze& maze, const Pose& start,
                                          Pose& end, const bool knownOnly,
                                          const bool simple,
                                          const bool breakUnknown) const {
#if 1
  /* shortest direction sequence from the start */
  Directions shortestDirections;
  auto& focus = end;
  /* follow the step gradient from start */
  focus = start;
  /* check */
  if (!start.p.isInsideOfField()) return {};
  /* scan around; find unknown walls and the minimum-step direction */
  while (1) {
    const auto focus_step = stepMap[focus.p.getIndex()];
    /* termination condition */
    if (focus_step == 0) break;
    /* scan around */
    auto min_p = focus.p;
    auto min_d = Direction::Max;
    for (const auto d : Direction::Along4()) {
      /* look as far straight as possible */
      auto next = focus.p;  //< adjacent
      for (int8_t i = 1;; ++i) {
        /* stop at a wall or (when knownOnly) an unknown wall */
        if (maze.isWall(next, d) || (knownOnly && !maze.isKnown(next, d)))
          break;
        next = next.next(d);  //< advance
        /* step value including straight-line acceleration */
        const step_t next_step = focus_step - (simple ? i : stepTable[i]);
        /* match against the edge cost */
        if (stepMap[next.getIndex()] == next_step) {
          min_p = next, min_d = d;
          goto loop_exit;
        }
      }
    }
  loop_exit:
    /* sanity: the new cell must hold a smaller step */
    if (focus_step <= stepMap[min_p.getIndex()]) break;
    /* append the move to the result */
    while (focus.p != min_p) {
      /* breakUnknown: stop when a cell with unknown walls is reached */
      if (breakUnknown && maze.unknownCount(focus.p)) return shortestDirections;
      focus = focus.next(min_d);
      shortestDirections.push_back(min_d);
    }
  }
  return shortestDirections;
#else
  /* build the known-section direction sequence from the step map */
  Directions shortestDirections;
  /* follow the step gradient from start */
  end = start;
  /* check */
  if (!start.p.isInsideOfField()) return {};
  while (1) {
    /* scan around; find unknown walls and the minimum-step direction */
    auto min_pose = end;
    auto min_step = STEP_MAX;
    for (const auto d : Direction::Along4()) {
      auto next = end.p;  //< adjacent
      for (int8_t i = 1; i < MAZE_SIZE; ++i) {
        /* stop at a wall or (when knownOnly) an unknown wall */
        if (maze.isWall(next, d) || (knownOnly && !maze.isKnown(next, d)))
          break;
        next = next.next(d);  //< advance
        /* update if the step is smaller than the current minimum */
        const auto next_step = stepMap[next.getIndex()];
        if (min_step <= next_step) break;
        min_step = next_step;
        min_pose = Pose{next, d};
      }
    }
    /* sanity: the new pose must hold a smaller step */
    if (stepMap[end.p.getIndex()] <= min_step) break;
    /* append the move to the result */
    while (end.p != min_pose.p) {
      /* breakUnknown: stop when a cell with unknown walls is reached */
      if (breakUnknown && maze.unknownCount(end.p)) return shortestDirections;
      end = end.next(min_pose.d);
      shortestDirections.push_back(min_pose.d);
    }
  }
  return shortestDirections;
#endif
}
Directions StepMap::getNextDirectionCandidates(const Maze& maze,
                                               const Pose& focus) const {
  /* pick candidates with straight priority; empty when all STEP_MAX */
  Directions dirs;
  dirs.reserve(4);
  for (const auto d : {focus.d + Direction::Front, focus.d + Direction::Left,
                       focus.d + Direction::Right, focus.d + Direction::Back})
    if (!maze.isWall(focus.p, d) && getStep(focus.p.next(d)) != STEP_MAX)
      dirs.push_back(d);
  /* sort by increasing cost */
  std::sort(dirs.begin(), dirs.end(),
            [&](const Direction d1, const Direction d2) {
              return getStep(focus.p.next(d1)) < getStep(focus.p.next(d2));
            });
#if 1
  /* prioritize cells with unknown walls (ties by cost) */
  std::sort(dirs.begin(), dirs.end(),
            [&](const Direction d1, const Direction d2) {
              return (maze.unknownCount(focus.p.next(d1)) &&
                      !maze.unknownCount(focus.p.next(d2)));
            });
#endif
#if 1
  /* prioritize straight travel */
  std::sort(dirs.begin(), dirs.end(),
            [&](const Direction d1, const Direction d2
                __attribute__((unused))) { return d1 == focus.d; });
#endif
  return dirs;
}
void StepMap::appendStraightDirections(const Maze& maze,
                                       Directions& shortestDirections,
                                       const bool knownOnly,
                                       const bool diagEnabled) {
  /* walk up to the goal cells */
  auto p = maze.getStart();
  for (const auto d : shortestDirections) p = p.next(d);
  if (shortestDirections.size() < 2) return;
  auto prev_dir = shortestDirections[shortestDirections.size() - 1 - 1];
  auto dir = shortestDirections[shortestDirections.size() - 1];
  /* go straight inside the goal cells as far as possible (with diagonals) */
  bool loop = true;
  while (loop) {
    loop = false;
    /* enumerate candidates with diagonals in mind */
    Directions dirs;
    const auto rel_dir = Direction(dir - prev_dir);
    if (diagEnabled && rel_dir == Direction::Left)
      dirs = {Direction(dir + Direction::Right), dir};
    else if (diagEnabled && rel_dir == Direction::Right)
      dirs = {Direction(dir + Direction::Left), dir};
    else
      dirs = {dir};
    /* take the first open candidate */
    for (const auto d : dirs) {
      if (!maze.isWall(p, d) && (!knownOnly || maze.isKnown(p, d))) {
        shortestDirections.push_back(d);
        p = p.next(d);
        prev_dir = dir;
        dir = d;
        loop = true;
        break;
      }
    }
  }
}
/**
 * @brief Cost of straight movement including trapezoidal acceleration.
 *
 * @param i Number of cells.
 * @param am Maximum acceleration.
 * @param vs Start velocity.
 * @param vm Saturation velocity.
 * @param seg Length of one cell.
 * @return StepMap::step_t Cost.
 */
static StepMap::step_t calcStraightCost(const int i, const float am,
                                        const float vs, const float vm,
                                        const float seg) {
  const auto d = seg * i;  //< distance of i cells
  /* solve for time using the graph area */
  const auto d_thr = (vm * vm - vs * vs) / am;  //< distance to reach vmax
  if (d < d_thr)
    return 2 * (std::sqrt(vs * vs + am * d) - vs) / am * 1000;  //< triangular
  else
    return (am * d + (vm - vs) * (vm - vs)) / (am * vm) * 1000;  //< trapezoid
}
void StepMap::calcStraightCostTable() {
  const float vs = 420.0f;      //< base velocity [mm/s]
  const float am_a = 4200.0f;   //< maximum acceleration [mm/s^2]
  const float vm_a = 1500.0f;   //< saturation velocity [mm/s]
  const float seg_a = 90.0f;    //< cell length [mm]
  const float t_turn = 287.0f;  //< tight 90-degree turn time [ms]
  stepTable[0] = 0;             //< [0] unused
  for (int i = 1; i < stepTableSize; ++i) {
    /* the first step counts as a 90-degree turn */
    stepTable[i] = t_turn + calcStraightCost(i - 1, am_a, vs, vm_a, seg_a);
  }
  /* keep the total cost below 65,535 [ms] by scaling */
  for (int i = 0; i < stepTableSize; ++i) {
    stepTable[i] /= scalingFactor;
#if 0
    MAZE_LOGI << "stepTable[" << i << "]:\t" << stepTable[i] << std::endl;
#endif
  }
}

}  // namespace MazeLib