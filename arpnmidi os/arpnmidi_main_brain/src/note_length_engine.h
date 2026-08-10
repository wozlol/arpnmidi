#pragma once

#include <stdint.h>

#include "four_track_looper.h"

namespace arpnmidi3 {

class NoteLengthEngine {
 public:
  using EmitFn = void (*)(void *context, uint8_t target, uint8_t source,
                          const LoopMidiEvent &event);

  void reset(EmitFn emit = nullptr, void *context = nullptr);
  void process(uint64_t nowUs, uint8_t target, uint8_t source,
               const LoopMidiEvent &event, uint8_t percent,
               uint32_t fallbackDurationUs, EmitFn emit, void *context);
  void tick(uint64_t nowUs, EmitFn emit, void *context, uint16_t maxWork = 48);
  void stopTarget(uint8_t target, EmitFn emit, void *context);

  uint32_t overflowCount() const { return overflowCount_; }

 private:
  static constexpr uint8_t kTargetCount = 5;
  static constexpr uint8_t kGateCount = 128;

  struct Gate {
    uint64_t onUs = 0;
    uint64_t dueUs = 0;
    uint64_t staleUs = 0;
    uint8_t target = 0;
    uint8_t channel = 0;
    uint8_t note = 0;
    uint8_t percent = 100;
    bool used = false;
    bool sourceReleased = false;
    bool outputOffSent = false;
  };

  Gate *findGate(uint8_t target, uint8_t channel, uint8_t note);
  Gate *allocateGate();
  void emitOff(Gate &gate, EmitFn emit, void *context);
  void learnDuration(const Gate &gate, uint64_t nowUs);

  Gate gates_[kGateCount]{};
  uint16_t lastDurationMs_[kTargetCount][16][128]{};
  uint32_t overflowCount_ = 0;
};

}  // namespace arpnmidi3
