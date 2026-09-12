/**
 * @file accumulator.h
 * @brief Ring buffer accumulating a fixed number of samples.
 *
 * Portions derived from micromouse-mouse-control (MIT License)
 * Copyright (c) Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#pragma once

#include <new>

namespace ctrl {

/**
 * @brief Data accumulator.
 * @tparam T Data type.
 * @tparam S Number of samples to hold.
 */
template <typename T, std::size_t S>
class Accumulator {
 public:
  /**
   * @brief Constructor.
   * @param[in] value Initial value assigned to every buffer slot.
   */
  Accumulator(const T& value = T()) {
    buffer = new T[S];
    head = 0;
    clear(value);
  }
  /**
   * @brief Destructor.
   */
  ~Accumulator() { delete[] buffer; }
  /**
   * @brief Reset the buffer.
   * @param[in] value Value to assign.
   */
  void clear(const T& value = T()) {
    for (int i = 0; i < S; i++) buffer[i] = value;
  }
  /**
   * @brief Append the latest sample.
   */
  void push(const T& value) {
    head = (head + 1) % S;
    buffer[head] = value;
  }
  /**
   * @brief Access the most recent sample at the given offset.
   * @details [0] is the newest sample, [size() - 1] the oldest.
   * @param[in] index Offset from the newest sample.
   * @return The requested sample.
   */
  const T& operator[](const std::size_t index) const {
    return buffer[(S + head - index) % S];
  }
  /**
   * @brief Average of the n most recent samples.
   * @param[in] n Number of samples to average.
   * @return Average value.
   */
  const T average(const int n = S) const {
    T sum = T();
    for (int i = 0; i < n; i++) {
      sum += buffer[(S + head - i) % S];
    }
    return sum / n;
  }
  /**
   * @brief Ring buffer size.
   */
  std::size_t size() const { return S; }

 private:
  T* buffer; /**< @brief Pointer to the ring buffer array. */
  std::size_t head; /**< @brief Head index of the ring buffer. */
};

}  // namespace ctrl