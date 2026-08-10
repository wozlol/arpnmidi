#pragma once

#include <stdint.h>

#include "four_track_looper.h"

namespace arpnmidi3 {

constexpr uint16_t kRollingHistoryCapacity = 2048;

struct HistorySnapshot {
  uint32_t nextSequence = 0;
  uint32_t endSequence = 0;
  uint64_t startUs = 0;
  uint64_t endUs = 0;
  uint16_t matchingEvents = 0;
  uint8_t target = 0;

  bool empty() const { return matchingEvents == 0; }
  uint16_t remaining() const { return matchingEvents; }
};

class RollingHistory {
 public:
  RollingHistory() = default;

  void reset();
  void push(uint64_t nowUs, uint8_t target, const LoopMidiEvent &event);
  HistorySnapshot snapshot(uint64_t endUs, uint32_t lengthUs, uint8_t target) const;
  bool readNext(HistorySnapshot &snapshot, LoopMidiEvent &event) const;

  uint16_t size() const { return count_; }
  uint32_t overwrittenCount() const { return overwrittenCount_; }

 private:
  struct Slot {
    uint64_t atUs = 0;
    uint32_t sequence = 0;
    uint8_t status = 0;
    uint8_t data1 = 0;
    uint8_t data2 = 0;
    uint8_t target = 0;
  };

  const Slot *slotForSequence(uint32_t sequence) const;

  Slot slots_[kRollingHistoryCapacity]{};
  uint32_t nextSequence_ = 0;
  uint32_t overwrittenCount_ = 0;
  uint16_t count_ = 0;
};

class HistoryRepeater {
 public:
  using EmitFn = void (*)(void *context, uint8_t target, const LoopMidiEvent &event);

  bool activate(const RollingHistory &history, uint64_t boundaryUs,
                uint32_t lengthUs, uint8_t target);
  void deactivate(EmitFn emit, void *context);
  void tick(uint64_t nowUs, const RollingHistory &history, EmitFn emit,
            void *context, uint16_t maxEvents = 48);

  bool active() const { return active_; }
  uint8_t target() const { return base_.target; }
  uint32_t lengthUs() const { return lengthUs_; }
  uint32_t completedCycles() const { return completedCycles_; }

 private:
  struct ActiveNote {
    uint8_t channel = 0;
    uint8_t note = 0;
  };

  int16_t findActiveNote(uint8_t channel, uint8_t note) const;
  void releaseActiveNotes(EmitFn emit, void *context);
  void emitNoteSafe(const LoopMidiEvent &event, EmitFn emit, void *context);

  HistorySnapshot base_{};
  HistorySnapshot cursor_{};
  LoopMidiEvent pending_{};
  ActiveNote activeNotes_[64]{};
  uint64_t cycleStartUs_ = 0;
  uint32_t lengthUs_ = 0;
  uint32_t completedCycles_ = 0;
  uint8_t activeNoteCount_ = 0;
  bool active_ = false;
  bool pendingValid_ = false;
};

}  // namespace arpnmidi3
