#include "note_length_engine.h"

namespace arpnmidi3 {

NoteLengthEngine::Gate *NoteLengthEngine::findGate(uint8_t target,
                                                    uint8_t channel,
                                                    uint8_t note) {
  Gate *newest = nullptr;
  for (Gate &gate : gates_) {
    if (!gate.used || gate.target != target || gate.channel != channel ||
        gate.note != note) continue;
    if (!newest || gate.onUs >= newest->onUs) newest = &gate;
  }
  return newest;
}

NoteLengthEngine::Gate *NoteLengthEngine::allocateGate() {
  for (Gate &gate : gates_) if (!gate.used) return &gate;
  ++overflowCount_;
  return nullptr;
}

void NoteLengthEngine::emitOff(Gate &gate, EmitFn emit, void *context) {
  if (gate.outputOffSent) return;
  if (emit) {
    emit(context, gate.target, 255,
         LoopMidiEvent{0, static_cast<uint8_t>(0x80 | gate.channel), gate.note, 0});
  }
  gate.outputOffSent = true;
  if (gate.sourceReleased) gate = Gate{};
}

void NoteLengthEngine::learnDuration(const Gate &gate, uint64_t nowUs) {
  const uint64_t durationUs = nowUs > gate.onUs ? nowUs - gate.onUs : 1000U;
  const uint64_t durationMs = (durationUs + 500U) / 1000U;
  lastDurationMs_[gate.target][gate.channel][gate.note] = static_cast<uint16_t>(
      durationMs > UINT16_MAX ? UINT16_MAX : (durationMs ? durationMs : 1));
}

void NoteLengthEngine::process(uint64_t nowUs, uint8_t target, uint8_t source,
                               const LoopMidiEvent &event, uint8_t percent,
                               uint32_t fallbackDurationUs, EmitFn emit,
                               void *context) {
  const uint8_t type = event.status & 0xF0;
  if (target >= kTargetCount || (type != 0x90 && type != 0x80) ||
      event.data1 > 127 || percent == 100) {
    if (emit) emit(context, target, source, event);
    return;
  }
  const uint8_t channel = event.status & 0x0F;
  const bool on = type == 0x90 && event.data2 > 0;
  if (on) {
    Gate *existing = findGate(target, channel, event.data1);
    if (existing) {
      emitOff(*existing, emit, context);
      *existing = Gate{};
    }
    Gate *gate = allocateGate();
    if (!gate) {
      if (emit) emit(context, target, source, event);
      return;
    }
    *gate = Gate{};
    gate->onUs = nowUs;
    gate->target = target;
    gate->channel = channel;
    gate->note = event.data1;
    gate->percent = percent;
    gate->used = true;
    gate->staleUs = nowUs + 600000000ULL;
    if (percent < 100) {
      uint64_t basisUs = static_cast<uint64_t>(
          lastDurationMs_[target][channel][event.data1]) * 1000ULL;
      if (basisUs == 0) basisUs = fallbackDurationUs ? fallbackDurationUs : 100000U;
      const uint64_t scaledDuration = basisUs * percent / 100U;
      gate->dueUs = nowUs + (scaledDuration < 1000U ? 1000U : scaledDuration);
    }
    if (emit) emit(context, target, source, event);
    return;
  }

  Gate *gate = findGate(target, channel, event.data1);
  if (!gate) {
    if (emit) emit(context, target, source, event);
    return;
  }
  learnDuration(*gate, nowUs);
  gate->sourceReleased = true;
  if (gate->percent > 100) {
    const uint64_t originalDuration = nowUs > gate->onUs ? nowUs - gate->onUs : 1000U;
    const uint64_t scaledDuration = originalDuration * gate->percent / 100U;
    gate->dueUs = gate->onUs + (scaledDuration < 1000U ? 1000U : scaledDuration);
  } else if (!gate->outputOffSent) {
    // The exact short duration is only known when the live Note Off arrives.
    // If the prediction has not already closed it, close it immediately now.
    gate->dueUs = nowUs;
  }
  if (gate->outputOffSent) *gate = Gate{};
}

void NoteLengthEngine::tick(uint64_t nowUs, EmitFn emit, void *context,
                            uint16_t maxWork) {
  uint16_t work = 0;
  for (Gate &gate : gates_) {
    if (work >= maxWork) break;
    if (!gate.used) continue;
    if (gate.dueUs != 0 && nowUs >= gate.dueUs && !gate.outputOffSent) {
      emitOff(gate, emit, context);
      ++work;
      continue;
    }
    if (nowUs >= gate.staleUs) {
      emitOff(gate, emit, context);
      gate = Gate{};
      ++work;
    }
  }
}

void NoteLengthEngine::stopTarget(uint8_t target, EmitFn emit, void *context) {
  for (Gate &gate : gates_) {
    if (!gate.used || gate.target != target) continue;
    emitOff(gate, emit, context);
    gate = Gate{};
  }
}

void NoteLengthEngine::reset(EmitFn emit, void *context) {
  for (Gate &gate : gates_) {
    if (gate.used) emitOff(gate, emit, context);
    gate = Gate{};
  }
  for (auto &target : lastDurationMs_) {
    for (auto &channel : target) {
      for (uint16_t &duration : channel) duration = 0;
    }
  }
  overflowCount_ = 0;
}

}  // namespace arpnmidi3
