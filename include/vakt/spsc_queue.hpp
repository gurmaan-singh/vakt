#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <new>
#include <optional>
#include <type_traits>

namespace vakt {

// Bounded single-producer / single-consumer ring buffer. Lock free and wait free: the
// producer only writes head_, the consumer only writes tail_. One slot is kept empty to
// tell full from empty. Capacity must be a power of two.
//
// Used between the receive, tracking and publish threads of vakt_live so a slow consumer
// drops data at a well-defined place (try_push fails) instead of stalling the radar input.
template <typename T, std::size_t Capacity>
class SpscQueue {
  static_assert(Capacity >= 2 && (Capacity & (Capacity - 1)) == 0, "capacity must be a power of two");
  static_assert(std::is_nothrow_move_assignable_v<T>);

 public:
  bool try_push(T value) {
    const std::size_t head = head_.load(std::memory_order_relaxed);
    const std::size_t next = (head + 1) & kMask;
    if (next == tail_.load(std::memory_order_acquire)) return false;  // full
    slots_[head] = std::move(value);
    head_.store(next, std::memory_order_release);
    return true;
  }

  std::optional<T> try_pop() {
    const std::size_t tail = tail_.load(std::memory_order_relaxed);
    if (tail == head_.load(std::memory_order_acquire)) return std::nullopt;  // empty
    T value = std::move(slots_[tail]);
    tail_.store((tail + 1) & kMask, std::memory_order_release);
    return value;
  }

  bool empty() const { return tail_.load(std::memory_order_acquire) == head_.load(std::memory_order_acquire); }

 private:
  static constexpr std::size_t kMask = Capacity - 1;
  // Separate cache lines so the two threads don't false-share the indices.
  static constexpr std::size_t kLine = 64;
  alignas(kLine) std::atomic<std::size_t> head_{0};
  alignas(kLine) std::atomic<std::size_t> tail_{0};
  alignas(kLine) std::array<T, Capacity> slots_{};
};

}  // namespace vakt
