#pragma once

#include <stdint.h>

#include "four_track_looper.h"

namespace arpnmidi3 {

struct EchoConfig {
  uint32_t lengthUs = 0;
  uint32_t delayUs = 0;
  uint8_t wetPercent = 50;
  int8_t drift = 0;
};

class EchoEngine {
 public:
  using EmitFn = void (*)(void *context, uint8_t target, const LoopMidiEvent &event);

  void reset(EmitFn emit = nullptr, void *context = nullptr);
  void noteOn(uint64_t nowUs, uint8_t target, uint8_t status,
              uint8_t note, uint8_t velocity, const EchoConfig &config);
  void noteOff(uint64_t nowUs, uint8_t target, uint8_t status, uint8_t note);
  void tick(uint64_t nowUs, EmitFn emit, void *context, uint16_t maxWork = 48);
  void stopTarget(uint8_t target, EmitFn emit, void *context);

  uint32_t overflowCount() const { return overflowCount_; }
  uint8_t activeSequenceCount() const;

 private:
  static constexpr uint8_t kSequenceCount = 64;
  static constexpr uint8_t kActiveNoteCount = 96;
  static constexpr uint8_t kMaxRepeats = 64;

  struct Sequence {
    uint64_t sourceOnUs = 0;
    uint64_t firstRepeatUs = 0;
    uint64_t endUs = 0;
    uint64_t nextOnUs = 0;
    uint32_t spacingUs = 0;
    uint32_t sourceDurationUs = 0;
    uint32_t driftFactorQ16 = 65536;
    uint16_t id = 0;
    uint8_t target = 0;
    uint8_t channel = 0;
    uint8_t note = 0;
    uint8_t firstVelocity = 0;
    uint8_t repeatCount = 0;
    bool used = false;
    bool scheduling = false;
    bool sourceReleased = false;
  };

  struct ActiveNote {
    uint64_t offUs = 0;
    uint32_t maxGateUs = 0;
    uint16_t sequenceId = 0;
    uint8_t target = 0;
    uint8_t channel = 0;
    uint8_t note = 0;
    bool used = false;
  };

  uint32_t driftFactorQ16(int8_t drift) const;
  Sequence *allocateSequence();
  ActiveNote *allocateActiveNote();
  bool sequenceHasActiveNotes(uint16_t sequenceId) const;
  void reclaimSequences();
  void emitOff(ActiveNote &note, EmitFn emit, void *context);

  Sequence sequences_[kSequenceCount]{};
  ActiveNote activeNotes_[kActiveNoteCount]{};
  uint16_t nextSequenceId_ = 1;
  uint32_t overflowCount_ = 0;
};

}  // namespace arpnmidi3
