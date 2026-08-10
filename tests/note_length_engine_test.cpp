#include <assert.h>

#include "../arpnmidi os/arpnmidi_main_brain/src/note_length_engine.h"

using arpnmidi3::LoopMidiEvent;
using arpnmidi3::NoteLengthEngine;

struct Probe {
  uint16_t count = 0;
  LoopMidiEvent events[16]{};
};

static void emit(void *context, uint8_t target, uint8_t,
                 const LoopMidiEvent &event) {
  assert(target == 0);
  Probe &probe = *static_cast<Probe *>(context);
  if (probe.count < 16) probe.events[probe.count] = event;
  ++probe.count;
}

int main() {
  NoteLengthEngine engine;
  Probe probe;
  engine.process(0, 0, 1, LoopMidiEvent{0, 0x90, 60, 100}, 50,
                 200000, emit, &probe);
  assert(probe.count == 1 && probe.events[0].status == 0x90);
  engine.tick(99999, emit, &probe);
  assert(probe.count == 1);
  engine.tick(100000, emit, &probe);
  assert(probe.count == 2 && probe.events[1].status == 0x80);
  engine.process(200000, 0, 1, LoopMidiEvent{0, 0x80, 60, 0}, 50,
                 200000, emit, &probe);
  assert(probe.count == 2);  // the source off is consumed

  engine.process(300000, 0, 1, LoopMidiEvent{0, 0x90, 60, 100}, 200,
                 200000, emit, &probe);
  engine.process(400000, 0, 1, LoopMidiEvent{0, 0x80, 60, 0}, 200,
                 200000, emit, &probe);
  engine.tick(499999, emit, &probe);
  assert(probe.count == 3);
  engine.tick(500000, emit, &probe);
  assert(probe.count == 4 && probe.events[3].status == 0x80);
  return 0;
}
