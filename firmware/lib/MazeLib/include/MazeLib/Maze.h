/**
 * @file Maze.h
 * @brief Maze representation and wall management for a micromouse.
 *
 * Portions derived from micromouse-maze-library (MIT License)
 * Copyright (c) Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#pragma once

#include <array>
#include <bitset>
#include <cmath>     //< for std::log2
#include <cstdint>   //< for uint8_t
#include <fstream>   //< for std::ifstream
#include <iostream>  //< for std::cout
#include <string>
#include <vector>

/* optional simple profiling aid */
#define MAZE_DEBUG_PROFILING 0
#if MAZE_DEBUG_PROFILING
#warning "this is debug mode!"
#include <chrono>
static int microseconds() {
  return std::chrono::duration_cast<std::chrono::microseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}
static int microseconds() __attribute__((unused));
#define MAZE_DEBUG_PROFILING_START(id) const auto t0_##id = microseconds();
#define MAZE_DEBUG_PROFILING_END(id)                                 \
  {                                                                  \
    const auto t1_##id = microseconds();                             \
    const auto dur = t1_##id - t0_##id;                              \
    static auto dur_max = 0;                                         \
    if (dur > dur_max) {                                             \
      dur_max = dur;                                                 \
      MAZE_LOGD << __func__ << "(" << #id << ")\t" << dur << " [us]" \
                << std::endl;                                        \
    }                                                                \
  }
#else
#define MAZE_DEBUG_PROFILING_START(id)
#define MAZE_DEBUG_PROFILING_END(id)
#endif

/*
 * Toggle colored console output for the maze printer.
 */
#ifdef MAZE_COLOR_DISABLED
#define C_RE ""
#define C_GR ""
#define C_YE ""
#define C_BL ""
#define C_MA ""
#define C_CY ""
#define C_NO ""
#else
#define C_RE "\e[31m" /**< @brief ANSI Escape Sequence RED */
#define C_GR "\e[32m" /**< @brief ANSI Escape Sequence GREEN */
#define C_YE "\e[33m" /**< @brief ANSI Escape Sequence YELLOW */
#define C_BL "\e[34m" /**< @brief ANSI Escape Sequence BLUE */
#define C_MA "\e[35m" /**< @brief ANSI Escape Sequence MAGENTA */
#define C_CY "\e[36m" /**< @brief ANSI Escape Sequence CYAN */
#define C_NO "\e[0m"  /**< @brief ANSI Escape Sequence RESET */
#endif

/**
 * @brief Logging verbosity selection.
 * @details 0: None, 1: Error, 2: Warn, 3: Info, 4: Debug
 */
#ifndef MAZE_LOG_LEVEL
#define MAZE_LOG_LEVEL 4
#endif
#define MAZE_LOG_STREAM_BASE(s, l, c) \
  (s << c "[" l "][" __FILE__ ":" << __LINE__ << "]" C_NO "\t")
#if MAZE_LOG_LEVEL >= 1
#define MAZE_LOGE MAZE_LOG_STREAM_BASE(std::cout, "E", C_RE)
#else
#define MAZE_LOGE std::ostream(0)
#endif
#if MAZE_LOG_LEVEL >= 2
#define MAZE_LOGW MAZE_LOG_STREAM_BASE(std::cout, "W", C_YE)
#else
#define MAZE_LOGW std::ostream(0)
#endif
#if MAZE_LOG_LEVEL >= 3
#define MAZE_LOGI MAZE_LOG_STREAM_BASE(std::cout, "I", C_GR)
#else
#define MAZE_LOGI std::ostream(0)
#endif
#if MAZE_LOG_LEVEL >= 4
#define MAZE_LOGD MAZE_LOG_STREAM_BASE(std::cout, "D", C_BL)
#else
#define MAZE_LOGD std::ostream(0)
#endif

/**
 * @brief All maze-searching library types live in this namespace.
 */
namespace MazeLib {

/**
 * @brief Number of cells along one side of the maze.
 */
static constexpr int MAZE_SIZE = 16;
/**
 * @brief Bits required to encode MAZE_SIZE. Used for bit shifts.
 */
static constexpr int MAZE_SIZE_BIT = std::ceil(std::log2(MAZE_SIZE));
/**
 * @brief Largest side length representable with MAZE_SIZE_BIT bits.
 */
static constexpr int MAZE_SIZE_MAX = std::pow(2, MAZE_SIZE_BIT);

/**
 * @brief A direction on the maze.
 * @details Backed by an 8-bit integer. Represents all 8 compass
 * directions, either absolute or relative. The constructor normalizes
 * the value into 0-7 so relative offsets can be computed by simple
 * addition and subtraction.
 * - e.g. Direction(Direction::East + Direction::Left) == Direction::North
 * - e.g. Direction(Direction::East - Direction::West) == Direction::Back
 * - e.g. Direction(-Direction::Left) == Direction::Right
 *
 * ```
 * AbsoluteDirection
 * +-----------+-------+-----------+
 * | NorthWest   North   NorthEast |
 * +           +       +           +
 * |      West     X          East |
 * +           +       +           +
 * | NorthWest   North   NorthEast |
 * +-----------+-------+-----------+
 * RelativeDirection
 * +-----------+-------+-----------+
 * |   Left135    Left      Left45 |
 * +           +       +           +
 * |      Back     X         Front |
 * +           +       +           +
 * |  Right135   Right     Right45 |
 * +-----------+-------+-----------+
 * ```
 */
class Direction {
 public:
  /**
   * @brief Absolute compass directions, 0-7.
   */
  enum AbsoluteDirection : int8_t {
    East,
    NorthEast,
    North,
    NorthWest,
    West,
    SouthWest,
    South,
    SouthEast,
  };
  /**
   * @brief Directions relative to the current heading, 0-7.
   */
  enum RelativeDirection : int8_t {
    Front,
    Left45,
    Left,
    Left135,
    Back,
    Right135,
    Right,
    Right45,
  };
  /**
   * @brief Total number of directions. Note this is an int8_t,
   * not a Direction.
   */
  static constexpr const int8_t Max = 8;

 public:
  /**
   * @brief Default constructor, stores the given absolute direction.
   */
  constexpr Direction(const AbsoluteDirection d = East) : d(d) {}
  /**
   * @brief Constructor from an integer. Result of relative-direction
   * arithmetic is normalized into 0-7.
   * @param d Result of relative-direction arithmetic.
   */
  constexpr Direction(const int8_t d) : d(d & 7) {}
  /**
   * @brief Conversion to int for relative-direction arithmetic.
   */
  constexpr operator int8_t() const { return d; }
  /**
   * @brief True if the direction is along an axis (not diagonal).
   */
  bool isAlong() const { return !(d & 1); }
  /**
   * @brief True if the direction is diagonal.
   */
  bool isDiag() const { return (d & 1); }
  /**
   * @brief Printable char representation.
   */
  char toChar() const { return ">'^`<,v.X"[d]; }
  /**
   * @brief Stream output.
   */
  friend std::ostream& operator<<(std::ostream& os, const Direction d) {
    return os << d.toChar();
  }
  /**
   * @brief The four non-diagonal directions (useful for loops).
   */
  static constexpr const std::array<Direction, 4> Along4() {
    return {
        Direction::East,
        Direction::North,
        Direction::West,
        Direction::South,
    };
  }
  /**
   * @brief The four diagonal directions (useful for loops).
   */
  static constexpr const std::array<Direction, 4> Diag4() {
    return {
        Direction::NorthEast,
        Direction::NorthWest,
        Direction::SouthWest,
        Direction::SouthEast,
    };
  }

 private:
  /**
   * @brief Direction value, always normalized into 0-7.
   */
  int8_t d;
};
static_assert(sizeof(Direction) == 1, "size error");

/**
 * @brief Dynamic array / set of Direction.
 */
using Directions = std::vector<Direction>;
/**
 * @brief Stream output for Directions, rendered as a sequence of
 * directional chars.
 */
std::ostream& operator<<(std::ostream& os, const Directions& obj);

/**
 * @brief A cell position in the maze.
 * @details Backed by a 16-bit integer. The bottom-left cell is (0,0)
 * on an (x, y) plane.
 *
 * ```
 * +--------+--------+
 * | (0, 1) | (1, 1) |
 * +--------+--------+
 * | (0, 0) | (1, 0) |
 * +--------+--------+
 * ```
 */
struct Position {
 public:
  /** @brief Total number of cells in the field; useful for allocation. */
  static constexpr int SIZE = MAZE_SIZE_MAX * MAZE_SIZE_MAX;

 public:
  union {
    struct {
      int8_t x; /**< @brief X component of the cell. */
      int8_t y; /**< @brief Y component of the cell. */
    };
    uint16_t data; /**< @brief Raw access to the whole value. */
  };

 public:
  /**
   * @brief Zero-initializing default constructor.
   */
  constexpr Position() : data(0) {}
  /**
   * @brief Constructor.
   * @param x,y Initial coordinates.
   */
  constexpr Position(const int8_t x, const int8_t y) : x(x), y(y) {}
  /**
   * @brief Unique sequential ID of a cell inside the maze.
   * @details Behavior is undefined for out-of-field cells. Check
   * Position::isInsideOfField() first.
   * @return uint16_t sequential ID
   */
  uint16_t getIndex() const { return (x << MAZE_SIZE_BIT) | y; }
  /**
   * @brief Reconstruct a Position from a sequential ID.
   * @param index Sequential ID.
   */
  static Position getPositionFromIndex(const uint16_t index) {
    return {int8_t(index >> MAZE_SIZE_BIT),
            int8_t(index & (MAZE_SIZE_MAX - 1))};
  }
  /** @brief Addition. */
  Position operator+(const Position p) const {
    return Position(x + p.x, y + p.y);
  }
  /** @brief Subtraction. */
  Position operator-(const Position p) const {
    return Position(x - p.x, y - p.y);
  }
  /** @brief Equality. */
  bool operator==(const Position p) const {
    // return x == p.x && y == p.y;
    return data == p.data;  //< faster
  }
  /** @brief Inequality. */
  bool operator!=(const Position p) const {
    // return x != p.x || y != p.y;
    return data != p.data;  //< faster
  }
  /**
   * @brief Position of the cell adjacent in the given direction.
   * @param d Adjacent direction.
   * @return Position of the adjacent cell.
   */
  Position next(const Direction d) const;
  /**
   * @brief True if the position is inside the field.
   * @return true Inside the field
   * @return false Outside the field
   */
  bool isInsideOfField() const {
    // return x >= 0 && x < MAZE_SIZE && y >= 0 && y < MAZE_SIZE;
    /* faster */
    return (static_cast<uint8_t>(x) < MAZE_SIZE) &&
           (static_cast<uint8_t>(y) < MAZE_SIZE);
  }
  /**
   * @brief Rotate the coordinate about the origin.
   * @param d Rotation amount; only 4 directions are supported.
   * @return Rotated position.
   */
  Position rotate(const Direction d) const;
  /**
   * @brief Rotate the coordinate about an arbitrary center.
   * @param d Rotation amount; only 4 directions are supported.
   * @param center Center of rotation.
   * @return Rotated position.
   */
  Position rotate(const Direction d, const Position center) const {
    return center + (*this - center).rotate(d);
  }
  /**
   * @brief Stream output in (  x,  y) form.
   */
  friend std::ostream& operator<<(std::ostream& os, const Position p);
  /**
   * @brief Printable string representation.
   */
  const char* toString() const {
    static char str[32];
    snprintf(str, sizeof(str), "(%02d, %02d)", x, y);
    return str;
  }
};
static_assert(sizeof(Position) == 2, "size error");

/**
 * @brief Dynamic array / set of Position.
 */
using Positions = std::vector<Position>;

/**
 * @brief A Position with a Direction, describing a pose.
 * @details Alignment constraints make this 4 bytes. A pose specifies a
 * cell and the direction of travel into that cell (not the direction out
 * of the current cell).
 *
 * ```
 * +---+---+---+  example:
 * |   <       | <--- (0, 2, West)
 * +   +---+ ^ + <--- (2, 2, North)
 * |   >       | <--- (1, 1, East)
 * +   +---+ v + <--- (2, 0, South)
 * | S |       | <--- (0, 0)
 * +---+---+---+
 * ```
 */
struct Pose {
 public:
  Position p;  /**< @brief Position. */
  Direction d; /**< @brief Direction. */

 public:
  Pose() {}
  Pose(const Position p, const Direction d) : p(p), d(d) {}
  /**
   * @brief Pose of the adjacent cell.
   * @param nextDirection Adjacent direction.
   * @return Pose Adjacent pose.
   */
  Pose next(const Direction nextDirection) const {
    return Pose(p.next(nextDirection), nextDirection);
  }
  /**
   * @brief Stream output.
   */
  friend std::ostream& operator<<(std::ostream& os, const Pose& pose);
  /**
   * @brief Printable string representation.
   */
  const char* toString() const {
    static char str[32];
    snprintf(str, sizeof(str), "(%02d, %02d, %c)", p.x, p.y, d.toChar());
    return str;
  }
};
static_assert(sizeof(Pose) == 4, "size error");

/**
 * @brief Wall-based management ID (as opposed to cell-based).
 * @details Casting to uint16_t yields a unique running index over all
 * walls. A WallIndex::SIZE-sized array indexed by that ID covers every
 * internal wall; check isInsideOfField() before indexing to avoid OOB.
 * IDs are only valid for internal walls, since the outer boundary has no
 * wall to represent.
 *
 * ```
 *      [x, y]    : Cell Position
 *             z  : Wall Distinction in the Cell; 0:East, 1:North
 *   => (x, y, z) : Wall Index
 * +-------------+-------------+-------------+
 * |             |             |             |
 * |     Cell   Wall           |             |
 * |             |             |             |
 * +--- z = 1 ---+- (x, y, 1) -+-------------+
 * |             |             |             |
 * |    (x-1, y, 0)  [ x, y]  (x, y, 0)      |
 * |             |             |             |
 * +--- z = 1 ---+- (x,y-1,1) -+-------------+
 * |             |             |             |
 * |           z = 0         z = 0           |
 * |             |             |             |
 * +-------------+-------------+-------------+
 * ```
 */
struct WallIndex {
  /**
   * @brief Total number of unique wall IDs in a field; useful for
   * allocation.
   */
  static constexpr int SIZE = MAZE_SIZE_MAX * MAZE_SIZE_MAX * 2;

 public:
  union {
    struct {
      int8_t x;      /**< @brief X component of the cell. */
      int8_t y : 7;  /**< @brief Y component of the cell. */
      uint8_t z : 1; /**< @brief Wall position in cell. 0:East, 1:North */
    };
    uint16_t data; /**< @brief Raw access to the whole value. */
  };
  static_assert(MAZE_SIZE < std::pow(2, 6), "MAZE_SIZE is too large!");

 public:
  /**
   * @brief Default constructor.
   */
  constexpr WallIndex() : data(0) {}
  /**
   * @brief Constructor storing the given raw components.
   */
  constexpr WallIndex(const int8_t x, const int8_t y, const uint8_t z)
      : x(x), y(y), z(z) {}
  /**
   * @brief Constructor that removes representation redundancy.
   * @param p Cell position.
   * @param d Direction within the cell; 4 directions.
   */
  WallIndex(const Position p, const Direction d) : x(p.x), y(p.y) {
    uniquify(d);
  }
  /**
   * @brief Constructor from a wall ID.
   * @param i Wall ID; must refer to an internal wall.
   * @attention Undefined behavior for walls outside the field.
   */
  constexpr WallIndex(const uint16_t i)
      : x(i & (MAZE_SIZE_MAX - 1)),
        y((i >> MAZE_SIZE_BIT) & (MAZE_SIZE_MAX - 1)),
        z(i >> (2 * MAZE_SIZE_BIT)) {}
  /** @brief Equality. */
  bool operator==(const WallIndex i) const {
    // return x == i.x && y == i.y && z == i.z;
    return data == i.data;  //< faster
  }
  /** @brief Inequality. */
  bool operator!=(const WallIndex i) const {
    // return x != i.x || y != i.y || z != i.z;
    return data != i.data;  //< faster
  }
  /**
   * @brief Unique running ID of this wall within the field.
   * @attention Undefined behavior for walls outside the field.
   * Check with WallIndex::isInsideOfField().
   * @return uint16_t ID
   */
  uint16_t getIndex() const {
    // return (z << (2 * MAZE_SIZE_BIT)) | (y << MAZE_SIZE_BIT) | x;
    return (z << (MAZE_SIZE_BIT << 1)) | (y << MAZE_SIZE_BIT) | x;  //< faster
  }
  /** @brief The cell position of this wall. */
  Position getPosition() const { return Position(x, y); }
  /** @brief The direction of this wall. */
  Direction getDirection() const {
    // return z == 0 ? Direction::East : Direction::North;
    return z << 1;  //< faster
  }
  /**
   * @brief Stream output in ( x, y, d) form.
   */
  friend std::ostream& operator<<(std::ostream& os, const WallIndex i);
  /**
   * @brief True if the wall lies inside the field.
   * @details (x, y) between (0, 0) and (MAZE_SIZE-1, MAZE_SIZE-1), and
   * z is not on the outer boundary.
   * @return true Inside the field
   * @return false Outside the field (including the boundary)
   */
  bool isInsideOfField() const {
    /* x and y inside the field and not on the boundary */
    // return !(x < 0 || y < 0 || x >= MAZE_SIZE || y >= MAZE_SIZE ||
    //          (z == 0 && (x == MAZE_SIZE - 1)) ||
    //          (z == 1 && (y == MAZE_SIZE - 1)));
    /* faster */
    return (static_cast<uint8_t>(x) < MAZE_SIZE - 1 + z) &&
           (static_cast<uint8_t>(y) < MAZE_SIZE - z);
  }
  /**
   * @brief WallIndex of the wall adjacent in the given direction.
   * @param d Adjacent direction.
   * @return WallIndex Adjacent wall.
   */
  WallIndex next(const Direction d) const;
  /**
   * @brief The six non-pillar directions adjacent to the current wall.
   * @return std::array<Direction, 6> Array of adjacent directions.
   */
  std::array<Direction, 6> getNextDirection6() const {
    const auto d = getDirection();
    return {{
        d + Direction::Front,
        d + Direction::Back,
        d + Direction::Left45,
        d + Direction::Right45,
        d + Direction::Left135,
        d + Direction::Right135,
    }};
  }

 private:
  /**
   * @brief Normalize the wall direction into a unique representation.
   * @details Mostly used by the constructors; end users rarely need it.
   * @param d Wall direction (4 directions).
   */
  void uniquify(const Direction d) {
    z = (d >> 1) & 1;  //< {East,West} => 0, {North,South} => 1
    switch (d) {
      case Direction::West:
        x--;
        break;
      case Direction::South:
        y--;
        break;
    }
  }
};
static_assert(sizeof(WallIndex) == 2, "size error");

/**
 * @brief Dynamic array / set of WallIndex.
 */
using WallIndexes = std::vector<WallIndex>;

/**
 * @brief Cell position, wall direction, and wall presence.
 * @details
 * - Backed by a 16-bit integer.
 * - Used to record exploration history.
 * - Uses a packed bit-field struct to stay small.
 */
struct WallRecord {
  /**
   * @brief Anonymous union of data members.
   */
  union {
    struct {
      int x : 6;          /**< @brief Cell X coordinate. */
      int y : 6;          /**< @brief Cell Y coordinate. */
      unsigned int d : 3; /**< @brief Wall direction. */
      unsigned int b : 1; /**< @brief Wall presence. */
    } __attribute__((__packed__));
    uint16_t data; /**< @brief Raw access to the whole value. */
  };
  static_assert(MAZE_SIZE < std::pow(2, 6), "MAZE_SIZE is too large!");
  /**
   * @brief Constructor.
   */
  WallRecord() {}
  WallRecord(const int8_t x, const int8_t y, const Direction d, const bool b)
      : x(x), y(y), d(d), b(b) {}
  WallRecord(const Position p, const Direction d, const bool b)
      : x(p.x), y(p.y), d(d), b(b) {}
  /** @brief The cell position. */
  const Position getPosition() const { return Position(x, y); }
  /** @brief The wall direction. */
  const Direction getDirection() const { return d; }
  /** @brief Stream output. */
  friend std::ostream& operator<<(std::ostream& os, const WallRecord& obj);
};
static_assert(sizeof(WallRecord) == 2, "size error");

/**
 * @brief Dynamic array of WallRecord.
 */
using WallRecords = std::vector<WallRecord>;

/**
 * @brief Keeps track of the maze wall map.
 * @details
 * - Stores the wall map plus the start and goal cells.
 * - Use isWall() to check wall presence.
 * - Use isKnown() to check whether a wall is explored.
 * - Use updateWall() to modify a wall.
 * - Maintains a WallRecords log for wall backup.
 */
class Maze {
 public:
  /**
   * @brief Default constructor.
   * @param goals Set of goal cells.
   * @param start Start cell.
   */
  Maze(const Positions& goals = Positions(),
       const Position start = Position(0, 0))
      : goals(goals), start(start) {
    reset();
  }
  /**
   * @brief Clear the maze; marks the start cell as explored.
   * @param set_start_wall Whether to set the start cell's East and North
   * walls.
   * @param set_range_full Whether to pre-expand the explored range to
   * the full field.
   */
  void reset(const bool set_start_wall = true,
             const bool set_range_full = false);
  /**
   * @brief Whether a wall exists at the given location.
   * @return true Wall exists, false no wall
   */
  bool isWall(const WallIndex i) const { return isWallBase(wall, i); }
  bool isWall(const Position p, const Direction d) const {
    return isWallBase(wall, WallIndex(p, d));
  }
  bool isWall(const int8_t x, const int8_t y, const Direction d) const {
    return isWallBase(wall, WallIndex(Position(x, y), d));
  }
  /**
   * @brief Update a wall.
   * @param i Wall location.
   * @param b Presence. true: wall, false: no wall.
   */
  void setWall(const WallIndex i, const bool b) {
    return setWallBase(wall, i, b);
  }
  void setWall(const Position p, const Direction d, const bool b) {
    return setWallBase(wall, WallIndex(p, d), b);
  }
  void setWall(const int8_t x, const int8_t y, const Direction d,
               const bool b) {
    return setWallBase(wall, WallIndex(Position(x, y), d), b);
  }
  /**
   * @brief Whether the wall has been explored.
   * @return true Explored, false not explored
   */
  bool isKnown(const WallIndex i) const { return isWallBase(known, i); }
  bool isKnown(const Position p, const Direction d) const {
    return isWallBase(known, WallIndex(p, d));
  }
  bool isKnown(const int8_t x, const int8_t y, const Direction d) const {
    return isWallBase(known, WallIndex(Position(x, y), d));
  }
  /**
   * @brief Mark a wall explored / unexplored.
   * @param i Wall location.
   * @param b Explored state. true: known, false: unknown.
   */
  void setKnown(const WallIndex i, const bool b) {
    return setWallBase(known, i, b);
  }
  void setKnown(const Position p, const Direction d, const bool b) {
    return setWallBase(known, WallIndex(p, d), b);
  }
  void setKnown(const int8_t x, const int8_t y, const Direction d,
                const bool b) {
    return setWallBase(known, WallIndex(Position(x, y), d), b);
  }
  /**
   * @brief Whether the wall can be traversed.
   * @return true Explored and wall-free
   * @return false Otherwise
   */
  bool canGo(const WallIndex i) const { return !isWall(i) && isKnown(i); }
  bool canGo(const Position p, const Direction d) const {
    return canGo(WallIndex(p, d));
  }
  bool canGo(const WallIndex& i, bool knownOnly) const {
    return !isWall(i) && (isKnown(i) || !knownOnly);
  }
  /**
   * @brief Update a wall, checking consistency with known walls.
   * @details If the reading contradicts a known wall, the wall is marked
   * unknown and this returns false.
   * @param p Cell position.
   * @param d Wall direction.
   * @param b Wall presence.
   * @param pushRecords Whether to append to the update record.
   * @return true Updated normally
   * @return false Contradicts known wall info
   */
  bool updateWall(const Position p, const Direction d, const bool b,
                  const bool pushRecords = true);
  /**
   * @brief Forget the most recently updated walls.
   * @param num Number of recent walls to clear.
   * @param set_start_wall Whether to set the start cell's East and North
   * walls.
   */
  void resetLastWalls(const int num, const bool set_start_wall = true);
  /**
   * @brief Number of walls around the given cell.
   * @param p Cell position.
   * @return Number of walls, 0-4.
   */
  int8_t wallCount(const Position p) const;
  /**
   * @brief Number of unknown walls adjacent to the given cell.
   * @param p Cell position.
   * @return Number of unknown walls, 0-4.
   */
  int8_t unknownCount(const Position p) const;
  /**
   * @brief Print the maze.
   */
  void print(std::ostream& os = std::cout,
             const int mazeSize = MAZE_SIZE) const;
  /**
   * @brief Print the maze with a path overlay.
   * @param start Start coordinate of the path.
   * @param dirs Array of travel directions.
   * @param os Output stream.
   * @param mazeSize Cells per side (square only).
   */
  void print(const Directions& dirs, const Position start = Position(0, 0),
             std::ostream& os = std::cout,
             const int mazeSize = MAZE_SIZE) const;
  /**
   * @brief Print the maze with a set of highlighted positions.
   * @param positions Positions to highlight.
   * @param os Output stream.
   * @param mazeSize Cells per side (square only).
   */
  void print(const Positions& positions, std::ostream& os = std::cout,
             const int mazeSize = MAZE_SIZE) const;
  /**
   * @brief Parse a maze from a text (*.maze) stream.
   * @details Text walls; S: start cell (single), G: goal cells (any).

   * ```
   * +---+---+
   * |     G |
   * +   +   +
   * | S | G |
   * +---+---+
   * ```
   *
   * @param is Input stream in *.maze format.
   */
  bool parse(std::istream& is);
  bool parse(const std::string& filepath) {
    std::ifstream ifs(filepath);
    return ifs ? parse(ifs) : false;
  }
  /**
   * @brief Parse maze data from an input stream.
   * @details Usage: Maze maze; maze << std::cin;
   * @param is Input stream containing text-format maze data.
   * @param maze Maze reference to parse into.
   * @return std::istream& The input argument, unchanged.
   */
  friend std::istream& operator>>(std::istream& is, Maze& maze) {
    maze.parse(is);
    return is;
  }
  /**
   * @brief Parse a maze from an array of hex strings.
   * @param data Array of per-cell hex strings, e.g. {"abaf", "1234",
   * "abab", "aaff"}.
   * @param mazeSize Cells per side (square only).
   */
  bool parse(const std::vector<std::string>& data, const int mazeSize);
  /**
   * @brief Set the goal cells.
   */
  void setGoals(const Positions& goals) { this->goals = goals; }
  /**
   * @brief Set the start cell.
   */
  void setStart(const Position start) { this->start = start; }
  /**
   * @brief Get the goal cells.
   */
  const Positions& getGoals() const { return goals; }
  /**
   * @brief Get the start cell.
   */
  const Position& getStart() const { return start; }
  /**
   * @brief Get the wall update log.
   */
  const WallRecords& getWallRecords() const { return wallRecords; }
  /**
   * @brief Bounds of the explored region; used to cut computation.
   */
  int8_t getMinX() const { return min_x; }
  int8_t getMinY() const { return min_y; }
  int8_t getMaxX() const { return max_x; }
  int8_t getMaxY() const { return max_y; }
  /**
   * @brief Append the wall record to a file.
   */
  bool backupWallRecordsToFile(const std::string& filepath,
                               const bool clear = false);
  /**
   * @brief Restore the wall record from a file.
   */
  bool restoreWallRecordsFromFile(const std::string& filepath);

 protected:
  std::bitset<WallIndex::SIZE> wall;  /**< @brief Wall map. */
  std::bitset<WallIndex::SIZE> known; /**< @brief Known/unknown walls. */
  Positions goals;                    /**< @brief Goal cells. */
  Position start;                     /**< @brief Start cell. */
  WallRecords wallRecords;            /**< @brief Wall update log. */
  int8_t min_x;                       /**< @brief Min explored X. */
  int8_t min_y;                       /**< @brief Min explored Y. */
  int8_t max_x;                       /**< @brief Max explored X. */
  int8_t max_y;                       /**< @brief Max explored Y. */
  int wallRecordsBackupCounter; /**< @brief Wall record backup counter. */

  /**
   * @brief Base wall lookup; out-of-field walls read as present.
   */
  bool isWallBase(const std::bitset<WallIndex::SIZE>& wall,
                  const WallIndex i) const {
    return !i.isInsideOfField() || wall[i.getIndex()];  //< out of field = wall
  }
  /**
   * @brief Base wall update; out-of-field writes are ignored.
   */
  void setWallBase(std::bitset<WallIndex::SIZE>& wall, const WallIndex i,
                   const bool b) const {
    if (i.isInsideOfField())  //< prevent out-of-range access
      wall[i.getIndex()] = b;
  }
};

}  // namespace MazeLib