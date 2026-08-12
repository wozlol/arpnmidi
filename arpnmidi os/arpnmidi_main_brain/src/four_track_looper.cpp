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
  paused_ = false;
  recordingArmed_ = false;
  recording_ = false;
  overdubbing_ = false;
  recordStartCount_ = 0;
  memset(importing_, 0, sizeof(importing_));
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

bool FourTrackLooper::trackHasContent(uint8_t track) const {
  return track < kLoopTrackCount && tracks_[track].count > 0 && !tracks_[track].hidden;
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
  // Overdub starts recording immediately, right here, rather than waiting for
  // a first note the way a replacement arm does, so its start count has to be
  // captured now too.
  if (overdub) recordStartCount_ = tracks_[track].count;
  memset(recordHeld_, 0, sizeof(recordHeld_));
  // Arming stays non-destructive.  A cleared track keeps its undo material
  // until the first captured event actually replaces it, so the armed target
  // can still move to another track without losing anything.
  if (overdub) tracks_[track].generation = ++generationCounter_;
}

bool FourTrackLooper::beginArmedRecording(uint64_t nowUs) {
  if (!recordingArmed_) return false;
  recordingArmed_ = false;
  recording_ = true;
  // The armed target may have moved onto a populated track after arming, so
  // the kind of pass is decided here rather than at arm time.  Audible content
  // is layered onto; empty and cleared tracks take a replacement recording.
  LoopTrackState &track = tracks_[recordingTrack_];
  overdubbing_ = playing_ && track.lengthUs > 0 && track.count > 0 && !track.hidden;
  if (overdubbing_) {
    recordStartUs_ = transportStartUs_;
    recordStartCount_ = track.count;
  } else {
    recordStartUs_ = nowUs;
    permanentlyClear(recordingTrack_);
    recordStartCount_ = 0;
  }
  track.generation = ++generationCounter_;
  return true;
}

bool FourTrackLooper::capture(uint64_t nowUs, const LoopMidiEvent &input) {
  const uint8_t type = input.status & 0xF0;
  const bool noteOn = type == 0x90 && input.data2 > 0;
  if (recordingArmed_) {
    if (!noteOn) return false;
    beginArmedRecording(nowUs);
  }
  if (!recording_) return false;

  LoopTrackState &track = tracks_[recordingTrack_];
  uint32_t atUs = 0;
  if (overdubbing_ && playing_ && track.lengthUs > 0) {
    atUs = static_cast<uint32_t>((nowUs - track.cycleStartUs) % track.lengthUs);
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
    if (playing_) {
      // The take begins where the performer began it, which is usually not the
      // top of the transport cycle. That offset is the track's phase from now
      // on and is restored every time playback starts again.
      track.cycleStartUs = nowUs;
      track.playCursor = track.head;
      captureTrackPhase(recordingTrack_);
    } else {
      track.startOffsetUs = 0;
    }
  }
  if (track.lengthUs == 0) track.lengthUs = 1;
  if (!overdubbing_) track.storedLengthUs = track.lengthUs;
  uint32_t closeAtUs = track.lengthUs - 1U;
  if (overdubbing_ && playing_) {
    closeAtUs = static_cast<uint32_t>((nowUs - track.cycleStartUs) % track.lengthUs);
  }
  closeHeldRecordingNotes(closeAtUs);
  recording_ = false;
  overdubbing_ = false;
  memset(recordHeld_, 0, sizeof(recordHeld_));
  // Whether THIS pass captured anything, not just whether the track holds
  // anything at all: an overdub track already had content before the pass
  // began, so track.count alone cannot tell the two apart. A pass that added
  // nothing must not read as a completed take to whatever asked.
  return track.count > recordStartCount_;
}

void FourTrackLooper::cancelRecording() {
  recordingArmed_ = false;
  recording_ = false;
  overdubbing_ = false;
  memset(recordHeld_, 0, sizeof(recordHeld_));
}

// Where a track sits inside the shared transport cycle. Tracks are allowed to
// begin at different points, and that relationship is what has to survive a
// stop or a clear and undo, so it is stored rather than recomputed from zero.
void FourTrackLooper::captureTrackPhase(uint8_t trackIndex) {
  if (trackIndex >= kLoopTrackCount) return;
  LoopTrackState &track = tracks_[trackIndex];
  if (track.lengthUs == 0) {
    track.startOffsetUs = 0;
    return;
  }
  if (track.cycleStartUs >= transportStartUs_) {
    track.startOffsetUs =
        static_cast<uint32_t>((track.cycleStartUs - transportStartUs_) % track.lengthUs);
  } else {
    const uint32_t behind =
        static_cast<uint32_t>((transportStartUs_ - track.cycleStartUs) % track.lengthUs);
    track.startOffsetUs = behind == 0 ? 0 : track.lengthUs - behind;
  }
}

// Resume continues a cycle that was already running, so an event sitting
// exactly on the resume point has been played and is skipped. Starting a cycle
// places the transport on that point, so an event there is still due.
void FourTrackLooper::seekTrackTo(LoopTrackState &track, uint64_t nowUs,
                                  uint32_t positionUs, bool skipEventsAtPosition) {
  track.cycleStartUs = nowUs - positionUs;
  track.playCursor = track.head;
  while (track.playCursor != kNoLoopEvent) {
    const uint32_t atUs = slots_[track.playCursor].event.atUs;
    if (atUs > positionUs || (atUs == positionUs && !skipEventsAtPosition)) break;
    track.playCursor = slots_[track.playCursor].next;
  }
}

void FourTrackLooper::resetPlaybackCursors(uint64_t nowUs) {
  transportStartUs_ = nowUs;
  for (LoopTrackState &track : tracks_) {
    if (track.lengthUs == 0) {
      track.playCursor = track.head;
      track.cycleStartUs = nowUs;
      continue;
    }
    // Restore the track to the point in its own loop that belongs at the top of
    // the transport cycle, so every track comes back in the same alignment it
    // had when it was recorded.
    const uint32_t offset = track.startOffsetUs % track.lengthUs;
    const uint32_t positionUs = offset == 0 ? 0 : track.lengthUs - offset;
    seekTrackTo(track, nowUs, positionUs, false);
  }
}

void FourTrackLooper::start(uint64_t nowUs) {
  if (!hasAnyData()) return;
  playing_ = true;
  paused_ = false;
  for (LoopTrackState &track : tracks_) track.pausedPositionUs = 0;
  resetPlaybackCursors(nowUs);
}

void FourTrackLooper::pause(uint64_t nowUs, ReleaseFn release, void *context) {
  if (!playing_) return;
  if (release) {
    for (uint8_t track = 0; track < kLoopTrackCount; ++track) release(context, track);
  }
  for (LoopTrackState &track : tracks_) {
    track.pausedPositionUs = track.lengthUs > 0
        ? static_cast<uint32_t>((nowUs - track.cycleStartUs) % track.lengthUs) : 0;
    track.playCursor = track.head;
  }
  playing_ = false;
  paused_ = true;
}

void FourTrackLooper::resume(uint64_t nowUs) {
  if (playing_ || !hasAnyData()) return;
  if (!paused_) {
    start(nowUs);
    return;
  }
  playing_ = true;
  paused_ = false;
  transportStartUs_ = nowUs;
  for (uint8_t index = 0; index < kLoopTrackCount; ++index) {
    LoopTrackState &track = tracks_[index];
    if (track.lengthUs == 0) continue;
    const uint32_t positionUs = track.pausedPositionUs % track.lengthUs;
    seekTrackTo(track, nowUs, positionUs, true);
    track.pausedPositionUs = 0;
    // Resume keeps absolute positions, so the stored phase is re-expressed
    // against the new transport origin instead of drifting away from it.
    captureTrackPhase(index);
  }
}

void FourTrackLooper::stop(ReleaseFn release, void *context) {
  if (release) {
    for (uint8_t track = 0; track < kLoopTrackCount; ++track) release(context, track);
  }
  playing_ = false;
  paused_ = false;
  for (LoopTrackState &track : tracks_) {
    track.playCursor = track.head;
    track.pausedPositionUs = 0;
  }
}

bool FourTrackLooper::resizeTrack(uint8_t trackIndex, uint32_t lengthUs,
                                  uint64_t nowUs, ReleaseFn release,
                                  void *context) {
  if (trackIndex >= kLoopTrackCount || lengthUs == 0) return false;
  if ((recording_ || recordingArmed_) && recordingTrack_ == trackIndex) return false;
  LoopTrackState &track = tracks_[trackIndex];
  if (track.count == 0 || track.lengthUs == 0) return true;

  const uint32_t storedLength = track.storedLengthUs > 0
      ? track.storedLengthUs : track.lengthUs;
  if (lengthUs > storedLength) {
    uint32_t required = 0;
    for (uint64_t offset = storedLength; offset < lengthUs; offset += storedLength) {
      uint16_t slot = track.head;
      for (uint16_t visited = 0; visited < track.count && slot != kNoLoopEvent;
           ++visited, slot = slots_[slot].next) {
        if (static_cast<uint64_t>(slots_[slot].event.atUs) + offset < lengthUs) {
          ++required;
        }
      }
    }
    if (required > freeEvents()) {
      ++overflowCount_;
      return false;
    }

    const uint16_t sourceCount = track.count;
    for (uint64_t offset = storedLength; offset < lengthUs; offset += storedLength) {
      uint16_t slot = track.head;
      for (uint16_t visited = 0; visited < sourceCount && slot != kNoLoopEvent;
           ++visited) {
        const uint16_t next = slots_[slot].next;
        LoopMidiEvent copy = slots_[slot].event;
        const uint64_t copyTime = static_cast<uint64_t>(copy.atUs) + offset;
        if (copyTime < lengthUs) {
          copy.atUs = static_cast<uint32_t>(copyTime);
          if (!insertEvent(trackIndex, copy)) return false;
        }
        slot = next;
      }
    }
    track.storedLengthUs = lengthUs;
  }

  const bool wasAudible = playing_ && audible(trackIndex);
  const uint32_t oldLengthUs = track.lengthUs;
  track.lengthUs = lengthUs;
  if (playing_) {
    uint64_t positionUs = oldLengthUs > 0
        ? (nowUs - track.cycleStartUs) % oldLengthUs : 0;
    positionUs %= lengthUs;
    if (wasAudible && release) release(context, trackIndex);
    seekTrackTo(track, nowUs, static_cast<uint32_t>(positionUs), true);
    captureTrackPhase(trackIndex);
  } else {
    track.playCursor = track.head;
    track.startOffsetUs %= lengthUs;
  }
  return true;
}

bool FourTrackLooper::audible(uint8_t trackIndex) const {
  if (trackIndex >= kLoopTrackCount) return false;
  bool anySolo = false;
  for (const LoopTrackState &track : tracks_) anySolo |= track.solo && !track.hidden;
  const LoopTrackState &track = tracks_[trackIndex];
  return !importing_[trackIndex] && !track.hidden && !track.muted && (!anySolo || track.solo);
}

void FourTrackLooper::collectHeldNotes(uint8_t track, HeldNoteFn fn, void *context) const {
  if (track >= kLoopTrackCount || !fn) return;
  const LoopTrackState &state = tracks_[track];
  // Nothing between head and the cursor to replay, either because the track
  // is empty or because the cursor sits at the very top of its cycle.
  if (state.head == kNoLoopEvent || state.head == state.playCursor) return;

  bool held[16][128] = {};
  uint8_t velocity[16][128] = {};
  uint16_t slot = state.head;
  while (slot != kNoLoopEvent && slot != state.playCursor) {
    const LoopMidiEvent &event = slots_[slot].event;
    const uint8_t type = event.status & 0xF0;
    if ((type == 0x90 || type == 0x80) && event.data1 <= 127) {
      const uint8_t channel = event.status & 0x0F;
      const bool on = type == 0x90 && event.data2 > 0;
      held[channel][event.data1] = on;
      if (on) velocity[channel][event.data1] = event.data2;
    }
    slot = slots_[slot].next;
  }
  for (uint8_t channel = 0; channel < 16; ++channel) {
    for (uint8_t note = 0; note < 128; ++note) {
      if (held[channel][note]) fn(context, track, channel, note, velocity[channel][note]);
    }
  }
}

void FourTrackLooper::tick(uint64_t nowUs, EmitFn emit, ReleaseFn release,
                           void *context, uint16_t maxEvents) {
  if (recording_ && !overdubbing_ && recordFixedLengthUs_ > 0 &&
      nowUs - recordStartUs_ >= recordFixedLengthUs_) {
    finishRecording(recordStartUs_ + recordFixedLengthUs_);
    if (!playing_) start(recordStartUs_ + recordFixedLengthUs_);
  }
  if (!playing_) return;
  uint16_t processed = 0;
  for (uint8_t trackIndex = 0; trackIndex < kLoopTrackCount; ++trackIndex) {
    LoopTrackState &track = tracks_[trackIndex];
    if (track.count == 0 || track.lengthUs == 0) continue;
    if (nowUs - track.cycleStartUs >= track.lengthUs) {
      if (release) release(context, trackIndex);
      const uint64_t cycles = (nowUs - track.cycleStartUs) / track.lengthUs;
      track.cycleStartUs += cycles * track.lengthUs;
      track.playCursor = track.head;
    }
    const bool canEmit = audible(trackIndex);
    const uint32_t elapsed = static_cast<uint32_t>(nowUs - track.cycleStartUs);
    while (track.playCursor != kNoLoopEvent && processed < maxEvents) {
      const EventSlot &slot = slots_[track.playCursor];
      if (slot.event.atUs > elapsed) break;
      if (canEmit && emit) emit(context, trackIndex, slot.event);
      track.playCursor = slot.next;
      ++processed;
    }
    if (processed >= maxEvents) break;
  }
}

void FourTrackLooper::releaseIfAudibilityChanged(uint8_t track, bool wasAudible,
                                                  ReleaseFn release, void *context) {
  if (wasAudible && !audible(track) && release) release(context, track);
}

void FourTrackLooper::safeClear(uint8_t track, ReleaseFn release, void *context) {
  if (track >= kLoopTrackCount) return;
  // A pending arm survives its own track being cleared. There is nothing
  // captured yet, so tidying up old content underneath it is not a reason to
  // abandon the take: the performer stays ready to play, right where they
  // were, with the old material now out of the way. An armed-and-recording
  // pass is different: it already holds real captured events, so clearing its
  // own track out from under it still aborts it, the same as before.
  if (recording_ && recordingTrack_ == track) cancelRecording();
  // An empty track must not become hidden.  Undo can never bring it back, and a
  // phantom hidden track inverts every "is anything cleared" decision above.
  if (tracks_[track].count == 0) {
    if (release) release(context, track);
    return;
  }
  tracks_[track].hidden = true;
  if (release) release(context, track);
}

void FourTrackLooper::undoClear(uint8_t track) {
  if (track >= kLoopTrackCount || tracks_[track].count == 0) return;
  // Bringing the old content back means abandoning whatever take was pending
  // for this track: undo and a fresh replacement are opposite choices, so
  // choosing one cancels the other rather than leaving a stale arm behind.
  if ((recording_ || recordingArmed_) && recordingTrack_ == track) cancelRecording();
  tracks_[track].hidden = false;
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
  paused_ = false;
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

// The most recently recorded layer, cleared or not. Stepping back to the last
// thing played is what a performer means by the previous layer.
uint8_t FourTrackLooper::newestPopulatedTrack() const {
  uint8_t result = selectedTrack_;
  uint32_t newest = 0;
  bool found = false;
  for (uint8_t i = 0; i < kLoopTrackCount; ++i) {
    if (tracks_[i].count == 0) continue;
    if (!found || tracks_[i].generation >= newest) {
      newest = tracks_[i].generation;
      result = i;
      found = true;
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
                                            uint32_t storedLengthUs,
                                            uint32_t generation, bool muted,
                                            bool solo, bool hidden,
                                            uint32_t startOffsetUs) {
  if (track >= kLoopTrackCount) return;
  tracks_[track].startOffsetUs = lengthUs > 0 ? startOffsetUs % lengthUs : 0;
  tracks_[track].lengthUs = lengthUs;
  tracks_[track].storedLengthUs = storedLengthUs >= lengthUs
      ? storedLengthUs : lengthUs;
  tracks_[track].generation = generation;
  tracks_[track].muted = muted;
  tracks_[track].solo = solo;
  tracks_[track].hidden = hidden;
  generationCounter_ = generation > generationCounter_ ? generation : generationCounter_;
}

void FourTrackLooper::beginImport(uint8_t track, uint32_t lengthUs,
                                  ReleaseFn release, void *context) {
  if (track >= kLoopTrackCount || lengthUs == 0) return;
  if (release && audible(track)) release(context, track);
  if (recordingTrack_ == track) cancelRecording();
  permanentlyClear(track);
  selectedTrack_ = track;
  tracks_[track].lengthUs = lengthUs;
  tracks_[track].storedLengthUs = lengthUs;
  tracks_[track].generation = ++generationCounter_;
  importing_[track] = true;
}

bool FourTrackLooper::importEvent(uint8_t track, const LoopMidiEvent &event) {
  if (track >= kLoopTrackCount || !importing_[track] ||
      event.atUs >= tracks_[track].lengthUs) return false;
  return insertEvent(track, event);
}

void FourTrackLooper::finishImport(uint8_t track, uint64_t boundaryUs) {
  if (track >= kLoopTrackCount || !importing_[track]) return;
  importing_[track] = false;
  tracks_[track].playCursor = tracks_[track].head;
  tracks_[track].cycleStartUs = boundaryUs;
  captureTrackPhase(track);
}

}  // namespace arpnmidi3
