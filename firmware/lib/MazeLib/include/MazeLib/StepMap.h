/**
 * @file StepMap.h
 * @brief Cell-based step map for micromouse maze search.
 *
 * Portions derived from micromouse-maze-library (MIT License)
 * Copyright (c) Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#pragma once

#include <limits>  //< for std::numeric_limits

#include "MazeLib/Maze.h"

namespace MazeLib {

/**
 * @brief Manages a cell-based step map.
 */
class StepMap {
 public:
  using step_t = uint16_t; /**< @brief Step value type. */
  static constexpr step_t STEP_MAX =
      std::numeric_limits<step_t>::max(); /**< @brief Maximum step value. */

 public:
  /**
   * @brief Default constructor.
   * @details Precomputes the trapezoidal-acceleration cost table.
   */
  StepMap();
  /**
   * @brief Initialize the step map.
   * @param[in] step Fill value for the whole map.
   */
  void reset(const step_t step = STEP_MAX) { stepMap.fill(step); }
  /**
   * @brief Get the step value.
   * @details Returns `STEP_MAX` for out-of-field positions.
   */
  step_t getStep(const int8_t x, const int8_t y) const {
    return getStep(Position(x, y));
  }
  /**
   * @brief Get the step value.
   * @details Returns `STEP_MAX` for out-of-field positions.
   */
  step_t getStep(const Position p) const {
    return p.isInsideOfField() ? stepMap[p.getIndex()] : STEP_MAX;
  }
  /**
   * @brief Set the step value.
   * @details Out-of-field writes are ignored.
   */
  void setStep(const int8_t x, const int8_t y, const step_t step) {
    return setStep(Position(x, y), step);
  }
  /**
   * @brief Set the step value.
   * @details Out-of-field writes are ignored.
   */
  void setStep(const Position p, const step_t step) {
    if (p.isInsideOfField()) stepMap[p.getIndex()] = step;
  }
  /**
   * @brief Read-only access to the raw step-map array.
   */
  const std::array<step_t, Position::SIZE>& getMapArray() const { return stepMap; }
  /**
   * @brief Multiply a step by this factor to convert to milliseconds.
   */
  constexpr float getScalingFactor() const { return scalingFactor; }
  /**
   * @brief Print the step map over the maze.
   * @param[in] maze Maze to draw.
   * @param[in] p Cell to highlight.
   * @param[in] d Direction to highlight.
   * @param[inout] os Output stream.
   */
  void print(const Maze& maze, const Position p = Position(-1, -1),
             const Direction d = Direction::Max,
             std::ostream& os = std::cout) const;
  void print(const Maze& maze, const Directions& dirs,
             const Position start = Position(0, 0),
             std::ostream& os = std::cout) const;
  void printFull(const Maze& maze, const Position p = Position(-1, -1),
                 const Direction d = Direction::Max,
                 std::ostream& os = std::cout) const;
  void printFull(const Maze& maze, const Directions& dirs,
                 const Position start = Position(0, 0),
                 std::ostream& os = std::cout) const;
  /**
   * @brief Update the step map.
   * @param[in] maze Maze information used for the update.
   * @param[in] dest Destination cells whose step becomes 0 (unordered).
   * @param[in] knownOnly true: unknown walls are impassable, false:
   * passable.
   * @param[in] simple Use cost 1 for every adjacent cell instead of
   * trapezoidal acceleration.
   */
  void update(const Maze& maze, const Positions& dest, const bool knownOnly,
              const bool simple);
  /**
   * @brief Compute the shortest path between the given cells.
   * @param[in] maze Maze to use.
   * @param[in] start Start cell.
   * @param[in] dest Destination cells (unordered).
   * @param[in] knownOnly Treat unknown walls as walls; follow only known
   * walls.
   * @param[in] simple Use cost 1 for every adjacent cell instead of
   * trapezoidal acceleration.
   * @return Direction sequence of the shortest path from start to dest;
   * empty if no path exists.
   */
  Directions calcShortestDirections(const Maze& maze, const Position start,
                                    const Positions& dest, const bool knownOnly,
                                    const bool simple);
  /**
   * @brief Compute the shortest path from start to the goals.
   * @param[in] maze Maze to use.
   * @param[in] knownOnly Treat unknown walls as walls; follow only known
   * walls.
   * @param[in] simple Use cost 1 for every adjacent cell instead of
   * trapezoidal acceleration.
   * @return Direction sequence from start to a goal; empty if no path
   * exists.
   */
  Directions calcShortestDirections(const Maze& maze, const bool knownOnly,
                                    const bool simple) {
    return calcShortestDirections(maze, maze.getStart(), maze.getGoals(),
                                  knownOnly, simple);
  }
  /**
   * @brief Compute the next directions from the step map.
   * @param[in] maze Maze to use.
   * @param[in] start Starting pose.
   * @param[out] nextDirectionsKnown Direction sequence over known cells.
   * @param[out] nextDirectionCandidates Candidate directions after the
   * known section.
   * @return Final cell of the known section.
   */
  Pose calcNextDirections(const Maze& maze, const Pose& start,
                          Directions& nextDirectionsKnown,
                          Directions& nextDirectionCandidates) const;
  /**
   * @brief Generate the direction sequence down the step gradient.
   * @param[in] maze Maze to use.
   * @param[in] start Starting pose.
   * @param[out] end Pose after the move.
   * @param[in] knownOnly Treat unknown walls as walls; follow only known
   * walls.
   * @param[in] simple Use cost 1 for every adjacent cell instead of
   * trapezoidal acceleration.
   * @param[in] breakUnknown Stop upon reaching a cell with unknown walls
   * (for exploration).
   */
  Directions getStepDownDirections(const Maze& maze, const Pose& start,
                                   Pose& end, const bool knownOnly,
                                   const bool simple,
                                   const bool breakUnknown) const;
  /**
   * @brief Prioritized unknown-wall checking order around a cell.
   * @param[in] maze Maze to use.
   * @param[in] focus The focus cell pose.
   * @return Prioritized directions to inspect.
   */
  Directions getNextDirectionCandidates(const Maze& maze,
                                        const Pose& focus) const;
  /**
   * @brief Append straight directions through the goal cells.
   * @param[in] maze Maze to use.
   * @param[inout] shortestDirections Sequence to append to.
   * @param[in] knownOnly Treat unknown walls as walls; follow only known
   * walls.
   * @param[in] diagEnabled Whether diagonal moves are allowed.
   */
  static void appendStraightDirections(const Maze& maze,
                                       Directions& shortestDirections,
                                       const bool knownOnly,
                                       const bool diagEnabled);

#if MAZE_DEBUG_PROFILING
  int queueSizeMax = 0;
#endif

 protected:
  /** @brief Step values for every cell. */
  std::array<step_t, Position::SIZE> stepMap;
  /** @brief Size of the cost table. */
  static constexpr int stepTableSize = MAZE_SIZE;
  /** @brief Scaling factor preventing cost overflow. */
  static constexpr float scalingFactor = 2;
  /** @brief Trapezoidal-acceleration cost table (along walls). */
  std::array<step_t, MAZE_SIZE> stepTable;

  /**
   * @brief Precompute the straight-move cost table for speed.
   */
  void calcStraightCostTable();
};

}  // namespace MazeLib