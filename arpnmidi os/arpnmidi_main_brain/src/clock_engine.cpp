#include "clock_engine.h"

namespace arpnmidi3 {
namespace {

constexpr uint8_t kMidiClockStatus = 0xF8;
constexpr uint8_t kMidiStartStatus = 0xFA;
constexpr uint8_t kMidiContinueStatus = 0xFB;
constexpr uint8_t kMidiStopStatus = 0xFC;
constexpr uint64_t kMinimumExternalTimeoutUs = 500000ULL;
constexpr uint8_t kMissingPulseLimit = 6;

uint16_t clampBpm(uint16_t bpm) {
  if (bpm < 20) return 20;
  if (bpm > 300) return 300;
  return bpm;
}

}  // namespace

void ClockEngine::begin(const ClockConfig &config, uint64_t nowUs) {
  config_ = config;
  config_.manualBpm = clampBpm(config_.manualBpm);
  running_ = true;
  haveExternalClock_ = false;
  filteredExternalIntervalUs_ = 0;
  lastExternalClockUs_ = 0;
  resetPhase(nowUs);
}

void ClockEngine::setConfig(const ClockConfig &config, uint64_t nowUs) {
  const bool timingSourceChanged = config.followExternal != config_.followExternal;
  const bool tempoChanged = clampBpm(config.manualBpm) != config_.manualBpm;
  config_ = config;
  config_.manualBpm = clampBpm(config_.manualBpm);
  if (timingSourceChanged || tempoChanged) resetPhase(nowUs);
}

TransportEvent ClockEngine::receiveRealtime(uint8_t status, uint64_t nowUs) {
  switch (status) {
    case kMidiClockStatus:
      receiveClock(nowUs);
      return TransportEvent::None;
    case kMidiStartStatus:
      if (config_.followExternal) start(nowUs);
      return TransportEvent::Start;
    case kMidiContinueStatus:
      if (config_.followExternal) resume(nowUs);
      return TransportEvent::Continue;
    case kMidiStopStatus:
      if (config_.followExternal) stop();
      return TransportEvent::Stop;
    default:
      return TransportEvent::None;
  }
}

void ClockEngine::receiveClock(uint64_t nowUs) {
  if (lastExternalClockUs_ != 0 && nowUs > lastExternalClockUs_) {
    const uint64_t interval = nowUs - lastExternalClockUs_;
    // Reject impossible clock intervals before they disturb the smoothing.
    if (interval >= 8000ULL && interval <= 150000ULL) {
      if (filteredExternalIntervalUs_ == 0) filteredExternalIntervalUs_ = interval;
      else filteredExternalIntervalUs_ =
          ((filteredExternalIntervalUs_ * 7ULL) + interval + 4ULL) / 8ULL;
    }
  }
  lastExternalClockUs_ = nowUs;
  haveExternalClock_ = true;
  if (config_.followExternal && running_) {
    internalPulse_ += kInternalPulsesPerMidiClock;
  }
}

bool ClockEngine::takeInternalClock(uint64_t nowUs) {
  if (!config_.sendClock || config_.followExternal || !running_) return false;
  if (nextInternalClockUs_ == 0) nextInternalClockUs_ = nowUs;
  if (nowUs < nextInternalClockUs_) return false;

  const uint64_t interval = midiClockIntervalUs();
  internalPulse_ += kInternalPulsesPerMidiClock;
  nextInternalClockUs_ += interval;

  // Bound catch-up work after a stall. Musical scheduling may report the stall,
  // but it must not flood the MIDI output with an unbounded clock burst.
  if (nowUs - nextInternalClockUs_ > interval * 4ULL) {
    nextInternalClockUs_ = nowUs + interval;
  }
  return true;
}

void ClockEngine::start(uint64_t nowUs) {
  running_ = true;
  resetPhase(nowUs);
}

void ClockEngine::resume(uint64_t nowUs) {
  if (running_) return;
  running_ = true;
  nextInternalClockUs_ = nowUs + midiClockIntervalUs();
}

void ClockEngine::stop() {
  running_ = false;
  nextInternalClockUs_ = 0;
}

bool ClockEngine::externalClockPresent(uint64_t nowUs) const {
  return haveExternalClock_ && nowUs >= lastExternalClockUs_ &&
         (nowUs - lastExternalClockUs_) <= externalTimeoutUs();
}

bool ClockEngine::synchronizedAdvanceAllowed(uint64_t nowUs) const {
  if (!running_) return false;
  return !config_.followExternal || externalClockPresent(nowUs);
}

float ClockEngine::bpm() const {
  if (config_.followExternal && filteredExternalIntervalUs_ != 0) {
    return 60000000.0f /
           (static_cast<float>(filteredExternalIntervalUs_) * kMidiClockPpqn);
  }
  return static_cast<float>(config_.manualBpm);
}

uint32_t ClockEngine::barPulse() const {
  const uint32_t pulses = barPulses(config_.threeFour);
  return pulses ? internalPulse_ % pulses : 0;
}

uint32_t ClockEngine::lastExternalClockAgeMs(uint64_t nowUs) const {
  if (!haveExternalClock_ || nowUs < lastExternalClockUs_) return UINT32_MAX;
  const uint64_t ageMs = (nowUs - lastExternalClockUs_) / 1000ULL;
  return ageMs > UINT32_MAX ? UINT32_MAX : static_cast<uint32_t>(ageMs);
}

uint64_t ClockEngine::midiClockIntervalUs() const {
  return quarterNoteUs(static_cast<float>(config_.manualBpm)) / kMidiClockPpqn;
}

uint64_t ClockEngine::externalTimeoutUs() const {
  const uint64_t pulseBased = filteredExternalIntervalUs_ * kMissingPulseLimit;
  return pulseBased > kMinimumExternalTimeoutUs ? pulseBased : kMinimumExternalTimeoutUs;
}

void ClockEngine::resetPhase(uint64_t nowUs) {
  phaseOriginUs_ = nowUs;
  internalPulse_ = 0;
  nextInternalClockUs_ = nowUs + midiClockIntervalUs();
}

}  // namespace arpnmidi3
