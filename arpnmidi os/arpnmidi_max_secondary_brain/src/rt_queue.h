#pragma once

#include <stddef.h>
#include <stdint.h>

namespace arpnmidi3 {

// Lock-free single-producer/single-consumer queue. One slot remains unused so
// full and empty are distinguishable. Queue congestion is measured rather than
// ever blocking a real-time producer.
template <typename T, size_t Capacity>
class RtQueue {
 public:
  static_assert(Capacity >= 2, "RtQueue needs at least two slots");
  static_assert((Capacity & (Capacity - 1)) == 0,
                "RtQueue capacity must be a power of two");

  bool push(const T &value) {
    const uint32_t write = loadRelaxed(&writeIndex_);
    const uint32_t next = (write + 1U) & kMask;
    if (next == loadAcquire(&readIndex_)) {
      __atomic_add_fetch(&dropped_, 1U, __ATOMIC_RELAXED);
      return false;
    }
    entries_[write] = value;
    storeRelease(&writeIndex_, next);
    const uint32_t used = size();
    uint32_t high = loadRelaxed(&highWater_);
    while (used > high &&
           !__atomic_compare_exchange_n(&highWater_, &high, used, false,
                                        __ATOMIC_RELAXED, __ATOMIC_RELAXED)) {
    }
    return true;
  }

  bool pop(T &value) {
    const uint32_t read = loadRelaxed(&readIndex_);
    if (read == loadAcquire(&writeIndex_)) return false;
    value = entries_[read];
    storeRelease(&readIndex_, (read + 1U) & kMask);
    return true;
  }

  uint32_t size() const {
    return (loadAcquire(&writeIndex_) - loadAcquire(&readIndex_)) & kMask;
  }
  uint32_t dropped() const { return loadRelaxed(&dropped_); }
  uint32_t highWater() const { return loadRelaxed(&highWater_); }

 private:
  static constexpr uint32_t kMask = static_cast<uint32_t>(Capacity - 1U);
  static uint32_t loadRelaxed(const volatile uint32_t *value) {
    return __atomic_load_n(value, __ATOMIC_RELAXED);
  }
  static uint32_t loadAcquire(const volatile uint32_t *value) {
    return __atomic_load_n(value, __ATOMIC_ACQUIRE);
  }
  static void storeRelease(volatile uint32_t *target, uint32_t value) {
    __atomic_store_n(target, value, __ATOMIC_RELEASE);
  }

  T entries_[Capacity]{};
  volatile uint32_t writeIndex_ = 0;
  volatile uint32_t readIndex_ = 0;
  volatile uint32_t dropped_ = 0;
  volatile uint32_t highWater_ = 0;
};

}  // namespace arpnmidi3
