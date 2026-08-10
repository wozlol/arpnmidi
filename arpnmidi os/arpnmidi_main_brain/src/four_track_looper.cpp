#include "four_track_looper.h"

#include <string.h>

namespace arpnmidi3 {

FourTrackLooper::FourTrackLooper() { reset(); }

void FourTrackLooper::reset() {
  for (uint16_t i = 0; i < kLoopEventPoolSize; ++i) {
    slots_[i] = EventSlot{};
    slots_[i].next = (i + 1U < kLoopEventPoolSize) ? i + 1U : kNoLoopEvent;
  }
  for (LoopTrackState &track : tracks_) track = LoopTrackState{};
  freeHead_ = 0;
  usedEvents_ = 0;
  overflowCount_ = 0;
  generationCounter_ = 0;
  transportStartUs_ = 0;
  recordStartUs_ = 0;
  recordFixedLengthUs_ = 0;
  recordQuantizeUs_ = 0;
  selectedTrack_ = 0;
  recordingTrack_ = 0;
  mode_ = LoopTrackMode::Manual;
  playing_ = false;
  recordingArmed_ = false;
  recording_ = false;
  overdubbing_ = false;
  memset(recordHeld_, 0, sizeof(recordHeld_));
}

uint16_t FourTrackLooper::allocateSlot() {
  if (freeHead_ == kNoLoopEvent) {
    ++overflowCount_;
    return kNoLoopEvent;
  }
  const uint16_t slot = freeHead_;
  freeHead_ = slots_[slot].next;
  slots_[slot].next = kNoLoopEvent;
  ++usedEvents_;
  return slot;
}

void FourTrackLooper::freeSlot(uint16_t slot) {
  if (slot == kNoLoopEvent || slot >= kLoopEventPoolSize) return;
  slots_[slot] = EventSlot{};
  slots_[slot].next = freeHead_;
  freeHead_ = slot;
  if (usedEvents_ > 0) --usedEvents_;
}

bool FourTrackLooper::insertEvent(uint8_t trackIndex, const LoopMidiEvent &event) {
  if (trackIndex >= kLoopTrackCount) return false;
  const uint16_t slot = allocateSlot();
  if (slot == kNoLoopEvent) return false;
  slots_[slot].event = event;
  LoopTrackState &track = tracks_[trackIndex];
  if (track.head == kNoLoopEvent) {
    track.head = track.tail = slot;
  } else if (slots_[track.tail].event.atUs <= event.atUs) {
    slots_[track.tail].next = slot;
    track.tail = slot;
  } else if (event.atUs < slots_[track.head].event.atUs) {
    slots_[slot].next = track.head;
    track.head = slot;
  } else {
    uint16_t previous = track.head;
    while (slots_[previous].next != kNoLoopEvent &&
           slots_[slots_[previous].next].event.atUs <= event.atUs) {
      previous = slots_[previous].next;
    }
    slots_[slot].next = slots_[previous].next;
    slots_[previous].next = slot;
  }
  ++track.count;
  return true;
}

void FourTrackLooper::selectTrack(uint8_t track) {
  if (track < kLoopTrackCount) selectedTrack_ = track;
}

void FourTrackLooper::armRecord(uint8_t track, uint32_t fixedLengthUs, bool overdub) {
  if (track >= kLoopTrackCount) return;
  recordingTrack_ = track;
  selectedTrack_ = track;
  recordFixedLengthUs_ = fixedLengthUs;
  recordingArmed_ = !overdub;
  recording_ = overdub;
  overdubbing_ = overdub;
  recordStartUs_ = overdub ? transportStartUs_ : 0;
  memset(recordHeld_, 0, sizeof(recordHeld_));
  if (!overdub && tracks_[track].hidden) permanentlyClear(track);
  if (overdub) tracks_[track].generation = ++generationCounter_;
}

bool FourTrackLooper::capture(uint64_t nowUs, const LoopMidiEvent &input) {
  const uint8_t type = input.status & 0xF0;
  const bool noteOn = type == 0x90 && input.data2 > 0;
  if (recordingArmed_) {
    if (!noteOn) return false;
    recordingArmed_ = false;
    recording_ = true;
    overdubbing_ = false;
    recordStartUs_ = nowUs;
    permanentlyClear(recordingTrack_);
    tracks_[recordingTrack_].generation = ++generationCounter_;
  }
  if (!recording_) return false;

  LoopTrackState &track = tracks_[recordingTrack_];
  uint32_t atUs = 0;
  if (overdubbing_ && playing_ && track.lengthUs > 0) {
    atUs = static_cast<uint32_t>((nowUs - transportStartUs_) % track.lengthUs);
  } else {
    const uint64_t elapsed = nowUs - recordStartUs_;
    if (recordFixedLengthUs_ > 0 && elapsed >= recordFixedLengthUs_) return false;
    atUs = static_cast<uint32_t>(elapsed);
  }
  LoopMidiEvent event = input;
  if (recordQuantizeUs_ > 0) {
    atUs = static_cast<uint32_t>(
        ((static_cast<uint64_t>(atUs) + recordQuantizeUs_ / 2U) / recordQuantizeUs_) *
        recordQuantizeUs_);
    const uint32_t limit = overdubbing_ ? track.lengthUs : recordFixedLengthUs_;
    if (limit > 0 && atUs >= limit) atUs = limit - 1U;
  }
  event.atUs = atUs;
  if (!insertEvent(recordingTrack_, event)) return false;

  if (type == 0x90 || type == 0x80) {
    const uint8_t channel = input.status & 0x0F;
    recordHeld_[channel][input.data1] = noteOn;
  }
  return true;
}

void FourTrackLooper::closeHeldRecordingNotes(uint32_t boundaryUs) {
  for (uint8_t channel = 0; channel < 16; ++channel) {
    for (uint8_t note = 0; note < 128; ++note) {
      if (!recordHeld_[channel][note]) continue;
      insertEvent(recordingTrack_, LoopMidiEvent{boundaryUs,
          static_cast<uint8_t>(0x80 | channel), note, 0});
      recordHeld_[channel][note] = false;
    }
  }
}

bool FourTrackLooper::finishRecording(uint64_t nowUs) {
  if (!recording_ && !recordingArmed_) return false;
  if (recordingArmed_) {
    recordingArmed_ = false;
    return false;
  }
  LoopTrackState &track = tracks_[recordingTrack_];
  if (!overdubbing_) {
    const uint64_t elapsed = nowUs - recordStartUs_;
    track.lengthUs = recordFixedLengthUs_ ? recordFixedLengthUs_ :
        static_cast<uint32_t>(elapsed > UINT32_MAX ? UINT32_MAX : elapsed);
  }
  if (track.lengthUs == 0) track.lengthUs = 1;
  closeHeldRecordingNotes(track.lengthUs - 1U);
  recording_ = false;
  overdubbing_ = false;
  memset(recordHeld_, 0, sizeof(recordHeld_));
  return track.count > 0;
}

void FourTrackLooper::cancelRecording() {
  recordingArmed_ = false;
  recording_ = false;
  overdubbing_ = false;
  memset(recordHeld_, 0, sizeof(recordHeld_));
}

void FourTrackLooper::resetPlaybackCursors(uint64_t nowUs) {
  transportStartUs_ = nowUs;
  for (LoopTrackState &track : tracks_) {
    track.playCursor = track.head;
    track.cycleStartUs = nowUs;
  }
}

void FourTrackLooper::start(uint64_t nowUs) {
  if (!hasAnyData()) return;
  playing_ = true;
  resetPlaybackCursors(nowUs);
}

void FourTrackLooper::stop(ReleaseFn release, void *context) {
  if (release) {
    for (uint8_t track = 0; track < kLoopTrackCount; ++track) release(context, track);
  }
  playing_ = false;
  for (LoopTrackState &track : tracks_) track.playCursor = track.head;
}

bool FourTrackLooper::audible(uint8_t trackIndex) const {
  if (trackIndex >= kLoopTrackCount) return false;
  bool anySolo = false;
  for (const LoopTrackState &track : tracks_) anySolo |= track.solo && !track.hidden;
  const LoopTrackState &track = tracks_[trackIndex];
  return !track.hidden && !track.muted && (!anySolo || track.solo);
}

void FourTrackLooper::tick(uint64_t nowUs, EmitFn emit, ReleaseFn release,
                           void *context, uint16_t maxEvents) {
  if (recording_ && !overdubbing_ && recordFixedLengthUs_ > 0 &&
      nowUs - recordStartUs_ >= recordFixedLengthUs_) {
    finishRecording(recordStartUs_ + recordFixedLengthUs_);
    if (!playing_) start(recordStartUs_ + recordFixedLengthUs_);
  }
  if (!playing_) return;
  uint16_t emitted = 0;
  for (uint8_t trackIndex = 0; trackIndex < kLoopTrackCount; ++trackIndex) {
    LoopTrackState &track = tracks_[trackIndex];
    if (track.count == 0 || track.lengthUs == 0) continue;
    if (nowUs - track.cycleStartUs >= track.lengthUs) {
      if (release) release(context, trackIndex);
      const uint64_t cycles = (nowUs - track.cycleStartUs) / track.lengthUs;
      track.cycleStartUs += cycles * track.lengthUs;
      track.playCursor = track.head;
    }
    if (!audible(trackIndex)) continue;
    const uint32_t elapsed = static_cast<uint32_t>(nowUs - track.cycleStartUs);
    while (track.playCursor != kNoLoopEvent && emitted < maxEvents) {
      const EventSlot &slot = slots_[track.playCursor];
      if (slot.event.atUs > elapsed) break;
      if (emit) emit(context, trackIndex, slot.event);
      track.playCursor = slot.next;
      ++emitted;
    }
    if (emitted >= maxEvents) break;
  }
}

void FourTrackLooper::releaseIfAudibilityChanged(uint8_t track, bool wasAudible,
                                                  ReleaseFn release, void *context) {
  if (wasAudible && !audible(track) && release) release(context, track);
}

void FourTrackLooper::safeClear(uint8_t track, ReleaseFn release, void *context) {
  if (track >= kLoopTrackCount) return;
  const bool wasAudible = audible(track);
  tracks_[track].hidden = true;
  releaseIfAudibilityChanged(track, wasAudible, release, context);
}

void FourTrackLooper::undoClear(uint8_t track) {
  if (track < kLoopTrackCount && tracks_[track].count > 0) tracks_[track].hidden = false;
}

void FourTrackLooper::permanentlyClear(uint8_t trackIndex) {
  if (trackIndex >= kLoopTrackCount) return;
  uint16_t slot = tracks_[trackIndex].head;
  while (slot != kNoLoopEvent) {
    const uint16_t next = slots_[slot].next;
    freeSlot(slot);
    slot = next;
  }
  tracks_[trackIndex] = LoopTrackState{};
}

void FourTrackLooper::clearAll(ReleaseFn release, void *context) {
  if (release) {
    for (uint8_t track = 0; track < kLoopTrackCount; ++track) release(context, track);
  }
  for (uint8_t track = 0; track < kLoopTrackCount; ++track) permanentlyClear(track);
  cancelRecording();
  playing_ = false;
}

void FourTrackLooper::setMuted(uint8_t track, bool muted, ReleaseFn release, void *context) {
  if (track >= kLoopTrackCount) return;
  const bool wasAudible = audible(track);
  tracks_[track].muted = muted;
  releaseIfAudibilityChanged(track, wasAudible, release, context);
}

void FourTrackLooper::setSolo(uint8_t track, bool solo, ReleaseFn release, void *context) {
  if (track >= kLoopTrackCount) return;
  bool wasAudible[kLoopTrackCount];
  for (uint8_t i = 0; i < kLoopTrackCount; ++i) wasAudible[i] = audible(i);
  tracks_[track].solo = solo;
  for (uint8_t i = 0; i < kLoopTrackCount; ++i) {
    releaseIfAudibilityChanged(i, wasAudible[i], release, context);
  }
}

bool FourTrackLooper::hasAnyData() const {
  for (const LoopTrackState &track : tracks_) if (track.count > 0 && !track.hidden) return true;
  return false;
}

uint8_t FourTrackLooper::oldestPopulatedTrack() const {
  uint8_t result = selectedTrack_;
  uint32_t oldest = UINT32_MAX;
  for (uint8_t i = 0; i < kLoopTrackCount; ++i) {
    if (tracks_[i].count > 0 && tracks_[i].generation < oldest) {
      oldest = tracks_[i].generation;
      result = i;
    }
  }
  return result;
}

bool FourTrackLooper::visitEvents(VisitFn visitor, void *context) const {
  if (!visitor) return false;
  for (uint8_t track = 0; track < kLoopTrackCount; ++track) {
    uint16_t slot = tracks_[track].head;
    while (slot != kNoLoopEvent) {
      if (!visitor(context, track, slots_[slot].event)) return false;
      slot = slots_[slot].next;
    }
  }
  return true;
}

bool FourTrackLooper::restoreEvent(uint8_t track, const LoopMidiEvent &event) {
  return insertEvent(track, event);
}

void FourTrackLooper::setRestoredTrackState(uint8_t track, uint32_t lengthUs,
                                            uint32_t generation, bool muted,
                                            bool solo, bool hidden) {
  if (track >= kLoopTrackCount) return;
  tracks_[track].lengthUs = lengthUs;
  tracks_[track].generation = generation;
  tracks_[track].muted = muted;
  tracks_[track].solo = solo;
  tracks_[track].hidden = hidden;
  generationCounter_ = generation > generationCounter_ ? generation : generationCounter_;
}

}  // namespace arpnmidi3
