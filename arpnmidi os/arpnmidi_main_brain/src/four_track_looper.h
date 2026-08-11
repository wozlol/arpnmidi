#pragma once

#include <stdint.h>

namespace arpnmidi3 {

constexpr uint8_t kLoopTrackCount = 4;
constexpr uint16_t kLoopEventPoolSize = 3072;
constexpr uint16_t kNoLoopEvent = 0xFFFF;

enum class LoopTrackMode : uint8_t {
  Layers = 0,
  PartsAutoSolo,
  Manual
};

struct LoopMidiEvent {
  uint32_t atUs = 0;
  uint8_t status = 0;
  uint8_t data1 = 0;
  uint8_t data2 = 0;
};

struct LoopTrackState {
  uint16_t head = kNoLoopEvent;
  uint16_t tail = kNoLoopEvent;
  uint16_t playCursor = kNoLoopEvent;
  uint16_t count = 0;
  uint32_t lengthUs = 0;
  uint32_t storedLengthUs = 0;
  uint32_t generation = 0;
  uint64_t cycleStartUs = 0;
  uint32_t pausedPositionUs = 0;
  // Where this track's own cycle begins inside the shared transport cycle. A
  // layer that started partway through the others keeps that relationship
  // across stop, clear, undo, and play again.
  uint32_t startOffsetUs = 0;
  bool muted = false;
  bool solo = false;
  bool hidden = false;
};

class FourTrackLooper {
 public:
  using EmitFn = void (*)(void *context, uint8_t track, const LoopMidiEvent &event);
  using ReleaseFn = void (*)(void *context, uint8_t track);
  using VisitFn = bool (*)(void *context, uint8_t track, const LoopMidiEvent &event);

  FourTrackLooper();
  void reset();

  void setTrackMode(LoopTrackMode mode) { mode_ = mode; }
  LoopTrackMode trackMode() const { return mode_; }
  void selectTrack(uint8_t track);
  uint8_t selectedTrack() const { return selectedTrack_; }

  bool trackHasContent(uint8_t track) const;
  void armRecord(uint8_t track, uint32_t fixedLengthUs, bool overdub);
  bool beginArmedRecording(uint64_t nowUs);
  void setRecordQuantizeUs(uint32_t quantumUs) { recordQuantizeUs_ = quantumUs; }
  bool capture(uint64_t nowUs, const LoopMidiEvent &event);
  bool finishRecording(uint64_t nowUs);
  void cancelRecording();
  bool recordingArmed() const { return recordingArmed_; }
  bool recording() const { return recording_; }
  bool overdubbing() const { return overdubbing_; }
  uint8_t recordingTrack() const { return recordingTrack_; }

  void start(uint64_t nowUs);
  void pause(uint64_t nowUs, ReleaseFn release, void *context);
  void resume(uint64_t nowUs);
  void stop(ReleaseFn release, void *context);
  bool resizeTrack(uint8_t track, uint32_t lengthUs, uint64_t nowUs,
                   ReleaseFn release, void *context);
  void tick(uint64_t nowUs, EmitFn emit, ReleaseFn release, void *context,
            uint16_t maxEvents = 96);
  bool playing() const { return playing_; }

  void safeClear(uint8_t track, ReleaseFn release, void *context);
  void undoClear(uint8_t track);
  void permanentlyClear(uint8_t track);
  void clearAll(ReleaseFn release, void *context);
  void setMuted(uint8_t track, bool muted, ReleaseFn release, void *context);
  void setSolo(uint8_t track, bool solo, ReleaseFn release, void *context);
  bool audible(uint8_t track) const;

  const LoopTrackState &track(uint8_t index) const { return tracks_[index]; }
  uint16_t usedEvents() const { return usedEvents_; }
  uint16_t freeEvents() const { return kLoopEventPoolSize - usedEvents_; }
  uint32_t overflowCount() const { return overflowCount_; }
  bool hasAnyData() const;
  uint8_t oldestPopulatedTrack() const;
  uint8_t newestPopulatedTrack() const;
  bool visitEvents(VisitFn visitor, void *context) const;
  bool restoreEvent(uint8_t track, const LoopMidiEvent &event);
  void setRestoredTrackState(uint8_t track, uint32_t lengthUs,
                             uint32_t storedLengthUs, uint32_t generation,
                             bool muted, bool solo, bool hidden,
                             uint32_t startOffsetUs = 0);
  void beginImport(uint8_t track, uint32_t lengthUs, ReleaseFn release, void *context);
  bool importEvent(uint8_t track, const LoopMidiEvent &event);
  void finishImport(uint8_t track, uint64_t boundaryUs);

 private:
  struct EventSlot {
    LoopMidiEvent event{};
    uint16_t next = kNoLoopEvent;
  };

  uint16_t allocateSlot();
  void freeSlot(uint16_t slot);
  bool insertEvent(uint8_t track, const LoopMidiEvent &event);
  void captureTrackPhase(uint8_t track);
  void seekTrackTo(LoopTrackState &track, uint64_t nowUs, uint32_t positionUs,
                   bool skipEventsAtPosition);
  void releaseIfAudibilityChanged(uint8_t track, bool wasAudible,
                                  ReleaseFn release, void *context);
  void resetPlaybackCursors(uint64_t nowUs);
  void closeHeldRecordingNotes(uint32_t boundaryUs);

  EventSlot slots_[kLoopEventPoolSize]{};
  LoopTrackState tracks_[kLoopTrackCount]{};
  uint16_t freeHead_ = kNoLoopEvent;
  uint16_t usedEvents_ = 0;
  uint32_t overflowCount_ = 0;
  uint32_t generationCounter_ = 0;
  uint64_t transportStartUs_ = 0;
  uint64_t recordStartUs_ = 0;
  uint32_t recordFixedLengthUs_ = 0;
  uint32_t recordQuantizeUs_ = 0;
  uint8_t selectedTrack_ = 0;
  uint8_t recordingTrack_ = 0;
  LoopTrackMode mode_ = LoopTrackMode::Manual;
  bool playing_ = false;
  bool paused_ = false;
  bool recordingArmed_ = false;
  bool recording_ = false;
  bool overdubbing_ = false;
  bool importing_[kLoopTrackCount]{};
  bool recordHeld_[16][128]{};
};

}  // namespace arpnmidi3
