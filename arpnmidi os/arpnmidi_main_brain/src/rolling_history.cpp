#include "rolling_history.h"

namespace arpnmidi3 {

void RollingHistory::reset() {
  for (Slot &slot : slots_) slot = Slot{};
  nextSequence_ = 0;
  overwrittenCount_ = 0;
  count_ = 0;
}

void RollingHistory::push(uint64_t nowUs, uint8_t target, const LoopMidiEvent &event) {
  Slot &slot = slots_[nextSequence_ % kRollingHistoryCapacity];
  slot.atUs = nowUs;
  slot.sequence = nextSequence_;
  slot.status = event.status;
  slot.data1 = event.data1;
  slot.data2 = event.data2;
  slot.target = target;
  ++nextSequence_;
  if (count_ < kRollingHistoryCapacity) ++count_;
  else ++overwrittenCount_;
}

const RollingHistory::Slot *RollingHistory::slotForSequence(uint32_t sequence) const {
  const Slot &slot = slots_[sequence % kRollingHistoryCapacity];
  return slot.sequence == sequence ? &slot : nullptr;
}

HistorySnapshot RollingHistory::snapshot(uint64_t endUs, uint32_t lengthUs,
                                         uint8_t target) const {
  HistorySnapshot result;
  result.startUs = endUs >= lengthUs ? endUs - lengthUs : 0;
  result.endUs = endUs;
  result.target = target;
  result.endSequence = nextSequence_;

  uint32_t low = nextSequence_ - count_;
  uint32_t high = nextSequence_;
  while (low < high) {
    const uint32_t middle = low + (high - low) / 2U;
    const Slot *slot = slotForSequence(middle);
    if (!slot || slot->atUs < result.startUs) low = middle + 1U;
    else high = middle;
  }
  result.nextSequence = low;
  for (uint32_t sequence = low; sequence != result.endSequence; ++sequence) {
    const Slot *slot = slotForSequence(sequence);
    if (slot && slot->atUs <= endUs && slot->target == target) ++result.matchingEvents;
  }
  return result;
}

bool RollingHistory::readNext(HistorySnapshot &snapshot, LoopMidiEvent &event) const {
  while (snapshot.nextSequence != snapshot.endSequence) {
    const uint32_t sequence = snapshot.nextSequence++;
    const Slot *slot = slotForSequence(sequence);
    if (!slot) {
      snapshot.nextSequence = snapshot.endSequence;
      snapshot.matchingEvents = 0;
      return false;
    }
    if (slot->atUs < snapshot.startUs || slot->atUs > snapshot.endUs ||
        slot->target != snapshot.target) continue;
    const uint64_t relative = slot->atUs - snapshot.startUs;
    const uint64_t length = snapshot.endUs - snapshot.startUs;
    event.atUs = static_cast<uint32_t>(relative < length ? relative : (length ? length - 1U : 0));
    event.status = slot->status;
    event.data1 = slot->data1;
    event.data2 = slot->data2;
    if (snapshot.matchingEvents > 0) --snapshot.matchingEvents;
    return true;
  }
  snapshot.matchingEvents = 0;
  return false;
}

int16_t HistoryRepeater::findActiveNote(uint8_t channel, uint8_t note) const {
  for (uint8_t i = 0; i < activeNoteCount_; ++i) {
    if (activeNotes_[i].channel == channel && activeNotes_[i].note == note) return i;
  }
  return -1;
}

void HistoryRepeater::releaseActiveNotes(EmitFn emit, void *context) {
  if (emit) {
    for (uint8_t i = 0; i < activeNoteCount_; ++i) {
      const ActiveNote &note = activeNotes_[i];
      emit(context, base_.target,
           LoopMidiEvent{0, static_cast<uint8_t>(0x80 | note.channel), note.note, 0});
    }
  }
  activeNoteCount_ = 0;
}

void HistoryRepeater::emitNoteSafe(const LoopMidiEvent &event, EmitFn emit, void *context) {
  if (!emit) return;
  const uint8_t type = event.status & 0xF0;
  if ((type != 0x90 && type != 0x80) || event.data1 > 127) {
    emit(context, base_.target, event);
    return;
  }
  const uint8_t channel = event.status & 0x0F;
  const bool on = type == 0x90 && event.data2 > 0;
  const int16_t activeIndex = findActiveNote(channel, event.data1);
  if (on) {
    if (activeIndex >= 0) {
      emit(context, base_.target,
           LoopMidiEvent{event.atUs, static_cast<uint8_t>(0x80 | channel), event.data1, 0});
    } else if (activeNoteCount_ < sizeof(activeNotes_) / sizeof(activeNotes_[0])) {
      activeNotes_[activeNoteCount_++] = ActiveNote{channel, event.data1};
    } else {
      return;
    }
    emit(context, base_.target, event);
    return;
  }
  if (activeIndex < 0) return;
  emit(context, base_.target,
       LoopMidiEvent{event.atUs, static_cast<uint8_t>(0x80 | channel), event.data1, 0});
  activeNotes_[activeIndex] = activeNotes_[--activeNoteCount_];
}

bool HistoryRepeater::activate(const RollingHistory &history, uint64_t boundaryUs,
                               uint32_t lengthUs, uint8_t target) {
  if (lengthUs == 0) return false;
  HistorySnapshot snapshot = history.snapshot(boundaryUs, lengthUs, target);
  if (snapshot.empty()) return false;
  base_ = snapshot;
  cursor_ = snapshot;
  cycleStartUs_ = boundaryUs;
  lengthUs_ = lengthUs;
  completedCycles_ = 0;
  activeNoteCount_ = 0;
  pendingValid_ = false;
  active_ = true;
  return true;
}

void HistoryRepeater::deactivate(EmitFn emit, void *context) {
  if (active_) releaseActiveNotes(emit, context);
  active_ = false;
  pendingValid_ = false;
}

void HistoryRepeater::tick(uint64_t nowUs, const RollingHistory &history,
                           EmitFn emit, void *context, uint16_t maxEvents) {
  if (!active_ || lengthUs_ == 0) return;
  if (nowUs - cycleStartUs_ >= lengthUs_) {
    releaseActiveNotes(emit, context);
    const uint64_t cycles = (nowUs - cycleStartUs_) / lengthUs_;
    cycleStartUs_ += cycles * lengthUs_;
    completedCycles_ += static_cast<uint32_t>(cycles);
    cursor_ = base_;
    pendingValid_ = false;
  }
  const uint32_t elapsedUs = static_cast<uint32_t>(nowUs - cycleStartUs_);
  uint16_t emitted = 0;
  while (emitted < maxEvents) {
    if (!pendingValid_) {
      if (!history.readNext(cursor_, pending_)) break;
      pendingValid_ = true;
    }
    if (pending_.atUs > elapsedUs) break;
    emitNoteSafe(pending_, emit, context);
    pendingValid_ = false;
    ++emitted;
  }
}

}  // namespace arpnmidi3
