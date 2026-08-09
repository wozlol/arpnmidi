#pragma once

#include <stdint.h>

#include "musical_time.h"

namespace arpnmidi3 {

enum class TransportEvent : uint8_t {
  None = 0,
  Start,
  Continue,
  Stop
};

struct ClockConfig {
  bool followExternal = false;
  bool sendClock = false;
  bool threeFour = false;
  uint16_t manualBpm = 120;
};

class ClockEngine {
 public:
  void begin(const ClockConfig &config, uint64_t nowUs);
  void setConfig(const ClockConfig &config, uint64_t nowUs);

  // Handles F8, FA, FB, and FC. The caller remains responsible for forwarding
  // messages when Clock Out is enabled.
  TransportEvent receiveRealtime(uint8_t status, uint64_t nowUs);

  // Returns true once for every internally generated MIDI clock byte that is
  // due. Call repeatedly until it returns false.
  bool takeInternalClock(uint64_t nowUs);

  void start(uint64_t nowUs);
  void resume(uint64_t nowUs);
  void stop();

  bool running() const { return running_; }
  bool followingExternal() const { return config_.followExternal; }
  bool clockOutEnabled() const { return config_.sendClock; }
  bool externalClockPresent(uint64_t nowUs) const;
  bool synchronizedAdvanceAllowed(uint64_t nowUs) const;

  float bpm() const;
  uint32_t internalPulse() const { return internalPulse_; }
  uint32_t barPulse() const;
  uint64_t phaseOriginUs() const { return phaseOriginUs_; }
  uint32_t lastExternalClockAgeMs(uint64_t nowUs) const;

 private:
  void receiveClock(uint64_t nowUs);
  uint64_t midiClockIntervalUs() const;
  uint64_t externalTimeoutUs() const;
  void resetPhase(uint64_t nowUs);

  ClockConfig config_{};
  bool running_ = true;
  bool haveExternalClock_ = false;
  uint64_t phaseOriginUs_ = 0;
  uint64_t lastExternalClockUs_ = 0;
  uint64_t filteredExternalIntervalUs_ = 0;
  uint64_t nextInternalClockUs_ = 0;
  uint32_t internalPulse_ = 0;
};

}  // namespace arpnmidi3
