/**
 * @file Accumulator.h
 * @brief Circular buffer for moving average / history.
 * Ported from kerise-v3.
 */
#pragma once

#include <cstddef>

template <typename T, size_t N>
class Accumulator {
public:
  Accumulator() { clear(T{}); }

  void clear(const T& val = T{}) {
    head_ = 0;
    size_ = 0;
    for (size_t i = 0; i < N; ++i) buf_[i] = val;
  }

  void push(const T& val) {
    buf_[head_] = val;
    head_ = (head_ + 1) % N;
    if (size_ < N) ++size_;
  }

  T operator[](size_t idx) const {
    // idx=0 is newest, idx=size_-1 is oldest
    if (idx >= size_) return T{};
    size_t real_idx = (head_ + N - 1 - idx) % N;
    return buf_[real_idx];
  }

  T average() const {
    if (size_ == 0) return T{};
    T sum{};
    for (size_t i = 0; i < size_; ++i) sum += buf_[i];
    return sum / static_cast<T>(size_);
  }

  size_t size() const { return size_; }
  bool full() const { return size_ == N; }

private:
  T buf_[N];
  size_t head_ = 0;
  size_t size_ = 0;
};