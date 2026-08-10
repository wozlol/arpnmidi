#include <assert.h>
#include <stdint.h>

#include "../arpnmidi os/arpnmidi_main_brain/src/rolling_history.h"

using arpnmidi3::HistorySnapshot;
using arpnmidi3::HistoryRepeater;
using arpnmidi3::LoopMidiEvent;
using arpnmidi3::RollingHistory;

struct Probe {
  uint16_t count = 0;
  LoopMidiEvent events[16]{};
};

static void emit(void *context, uint8_t target, const LoopMidiEvent &event) {
  Probe &probe = *static_cast<Probe *>(context);
  assert(target == 2);
  if (probe.count < 16) probe.events[probe.count] = event;
  ++probe.count;
}

int main() {
  RollingHistory history;
  history.push(1000, 0, LoopMidiEvent{0, 0x90, 60, 100});
  history.push(2000, 1, LoopMidiEvent{0, 0x80, 60, 0});
  history.push(3000, 0, LoopMidiEvent{0, 0x90, 64, 90});

  HistorySnapshot snapshot = history.snapshot(3500, 2000, 0);
  assert(snapshot.remaining() == 1);
  LoopMidiEvent event;
  assert(history.readNext(snapshot, event));
  assert(event.atUs == 1500 && event.status == 0x90 && event.data1 == 64);
  assert(!history.readNext(snapshot, event));

  for (uint32_t i = 0; i < arpnmidi3::kRollingHistoryCapacity + 10U; ++i) {
    history.push(4000 + i, 0,
                 LoopMidiEvent{0, 0xB0, 1, static_cast<uint8_t>(i & 0x7F)});
  }
  assert(history.size() == arpnmidi3::kRollingHistoryCapacity);
  assert(history.overwrittenCount() == 13);
  snapshot = history.snapshot(4000 + arpnmidi3::kRollingHistoryCapacity + 9U, 100, 0);
  uint16_t count = 0;
  while (history.readNext(snapshot, event)) ++count;
  assert(count == 101);

  RollingHistory repeatHistory;
  repeatHistory.push(10000, 2, LoopMidiEvent{0, 0x90, 60, 100});
  repeatHistory.push(10400, 2, LoopMidiEvent{0, 0x80, 60, 0});
  HistoryRepeater repeater;
  Probe probe;
  assert(repeater.activate(repeatHistory, 11000, 1000, 2));
  repeater.tick(11000, repeatHistory, emit, &probe);
  assert(probe.count == 1 && probe.events[0].status == 0x90);
  repeater.tick(11400, repeatHistory, emit, &probe);
  assert(probe.count == 2 && probe.events[1].status == 0x80);
  repeater.tick(12000, repeatHistory, emit, &probe);
  assert(probe.count == 3 && probe.events[2].status == 0x90);
  repeater.deactivate(emit, &probe);
  assert(probe.count == 4 && probe.events[3].status == 0x80);
  return 0;
}
