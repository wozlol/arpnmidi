#include <assert.h>
#include <stdint.h>

#include "../arpnmidi os/arpnmidi_main_brain/src/four_track_looper.h"

using arpnmidi3::FourTrackLooper;
using arpnmidi3::LoopMidiEvent;

struct Probe {
  uint16_t emitted = 0;
  uint16_t released = 0;
  uint8_t lastTrack = 0;
  LoopMidiEvent last{};
};

static void emit(void *context, uint8_t track, const LoopMidiEvent &event) {
  Probe &probe = *static_cast<Probe *>(context);
  ++probe.emitted;
  probe.lastTrack = track;
  probe.last = event;
}

static void release(void *context, uint8_t track) {
  Probe &probe = *static_cast<Probe *>(context);
  ++probe.released;
  probe.lastTrack = track;
}

int main() {
  FourTrackLooper looper;
  Probe probe;

  looper.armRecord(0, 1000000, false);
  assert(looper.recordingArmed());
  assert(looper.capture(100, LoopMidiEvent{0, 0x90, 60, 100}));
  assert(looper.recording());
  assert(looper.capture(500100, LoopMidiEvent{0, 0x80, 60, 0}));
  assert(looper.finishRecording(1000100));
  assert(looper.track(0).count == 2);
  assert(looper.track(0).lengthUs == 1000000);

  looper.start(2000000);
  looper.tick(2000000, emit, release, &probe);
  assert(probe.emitted == 1 && probe.last.status == 0x90);
  looper.tick(2500100, emit, release, &probe);
  assert(probe.emitted == 2 && probe.last.status == 0x80);

  looper.setMuted(0, true, release, &probe);
  assert(probe.released == 1);
  looper.setMuted(0, false, release, &probe);
  looper.safeClear(0, release, &probe);
  assert(!looper.hasAnyData());
  looper.undoClear(0);
  assert(looper.hasAnyData());

  looper.armRecord(1, 1000000, false);
  assert(looper.capture(3000000, LoopMidiEvent{0, 0x90, 64, 90}));
  assert(looper.finishRecording(4000000));
  assert(looper.track(1).count == 2);  // boundary Note Off is synthesized
  assert(looper.usedEvents() == 4);

  looper.clearAll(release, &probe);
  assert(looper.usedEvents() == 0);
  assert(!looper.hasAnyData());
  return 0;
}
