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
  looper.setMuted(0, true, release, &probe);
  looper.tick(2100000, emit, release, &probe);
  assert(probe.emitted == 0);
  looper.setMuted(0, false, release, &probe);
  looper.tick(2100001, emit, release, &probe);
  assert(probe.emitted == 0);  // unmuting does not replay stale events
  looper.stop(release, &probe);
  looper.start(2000000);
  looper.tick(2000000, emit, release, &probe);
  assert(probe.emitted == 1 && probe.last.status == 0x90);
  looper.tick(2500100, emit, release, &probe);
  assert(probe.emitted == 2 && probe.last.status == 0x80);

  const uint16_t releasesBeforeMute = probe.released;
  looper.setMuted(0, true, release, &probe);
  assert(probe.released == releasesBeforeMute + 1);
  looper.setMuted(0, false, release, &probe);
  looper.safeClear(0, release, &probe);
  assert(!looper.hasAnyData());
  looper.undoClear(0);
  assert(looper.hasAnyData());

  looper.beginImport(0, 500000, release, &probe);
  assert(!looper.audible(0));
  assert(looper.importEvent(0, LoopMidiEvent{0, 0x90, 67, 110}));
  assert(looper.importEvent(0, LoopMidiEvent{499999, 0x80, 67, 0}));
  assert(!looper.importEvent(0, LoopMidiEvent{500000, 0x90, 70, 100}));
  looper.finishImport(0, 2750000);
  assert(looper.track(0).count == 2);
  assert(looper.track(0).lengthUs == 500000);
  assert(looper.audible(0));

  looper.armRecord(1, 1000000, false);
  assert(looper.capture(3000000, LoopMidiEvent{0, 0x90, 64, 90}));
  assert(looper.finishRecording(4000000));
  assert(looper.track(1).count == 2);  // boundary Note Off is synthesized
  assert(looper.usedEvents() == 4);

  // A fixed-length take closes and starts playback at its boundary without a
  // UI action. Any still-held note is repaired with a boundary Note Off.
  looper.armRecord(2, 250000, false);
  assert(looper.capture(5000000, LoopMidiEvent{0, 0x90, 69, 100}));
  looper.tick(5250000, emit, release, &probe);
  assert(!looper.recording());
  assert(looper.playing());
  assert(looper.track(2).lengthUs == 250000);
  assert(looper.track(2).count == 2);

  // Ending an overdub closes its held note at the current loop phase, not at
  // the far end of the loop.
  looper.armRecord(2, 250000, true);
  assert(looper.capture(5300000, LoopMidiEvent{0, 0x90, 72, 100}));
  assert(looper.finishRecording(5350000));
  bool foundPhaseNoteOff = false;
  struct VisitContext {
    bool *found;
  } visitContext{&foundPhaseNoteOff};
  assert(looper.visitEvents([](void *context, uint8_t track, const LoopMidiEvent &event) {
    auto &visit = *static_cast<VisitContext *>(context);
    if (track == 2 && event.status == 0x80 && event.data1 == 72 && event.atUs == 100000) {
      *visit.found = true;
    }
    return true;
  }, &visitContext));
  assert(foundPhaseNoteOff);

  // Extending repeats the entire stored take. Shrinking changes only the
  // playback window, so extending again restores the already-built copies.
  FourTrackLooper resizeLooper;
  Probe resizeProbe;
  resizeLooper.armRecord(0, 250000, false);
  assert(resizeLooper.capture(1000000, LoopMidiEvent{0, 0x90, 60, 100}));
  assert(resizeLooper.capture(1100000, LoopMidiEvent{0, 0xB0, 74, 80}));
  assert(resizeLooper.capture(1125000, LoopMidiEvent{0, 0x80, 60, 0}));
  assert(resizeLooper.finishRecording(1250000));
  assert(resizeLooper.track(0).count == 3);
  assert(resizeLooper.track(0).storedLengthUs == 250000);
  assert(resizeLooper.resizeTrack(0, 500000, 1300000, release, &resizeProbe));
  assert(resizeLooper.track(0).count == 6);
  assert(resizeLooper.track(0).lengthUs == 500000);
  assert(resizeLooper.track(0).storedLengthUs == 500000);
  assert(resizeLooper.resizeTrack(0, 250000, 1400000, release, &resizeProbe));
  assert(resizeLooper.track(0).count == 6);
  assert(resizeLooper.track(0).storedLengthUs == 500000);
  assert(resizeLooper.resizeTrack(0, 500000, 1500000, release, &resizeProbe));
  assert(resizeLooper.track(0).count == 6);  // preserved copy, no duplicate copy
  assert(resizeLooper.resizeTrack(0, 1000000, 1600000, release, &resizeProbe));
  assert(resizeLooper.track(0).count == 12);

  bool foundCopiedCc = false;
  VisitContext copiedCcContext{&foundCopiedCc};
  assert(resizeLooper.visitEvents([](void *context, uint8_t track,
                                     const LoopMidiEvent &event) {
    auto &visit = *static_cast<VisitContext *>(context);
    if (track == 0 && event.status == 0xB0 && event.data1 == 74 &&
        event.data2 == 80 && event.atUs == 350000) {
      *visit.found = true;
    }
    return true;
  }, &copiedCcContext));
  assert(foundCopiedCc);

  FourTrackLooper transportLooper;
  Probe transportProbe;
  transportLooper.armRecord(0, 1000000, false);
  assert(transportLooper.capture(1000000, LoopMidiEvent{0, 0x90, 60, 100}));
  assert(transportLooper.capture(1500000, LoopMidiEvent{0, 0x80, 60, 0}));
  assert(transportLooper.finishRecording(2000000));
  transportLooper.start(3000000);
  transportLooper.tick(3000000, emit, release, &transportProbe);
  assert(transportProbe.emitted == 1);
  transportLooper.pause(3300000, release, &transportProbe);
  assert(!transportLooper.playing());
  transportLooper.resume(4000000);
  assert(transportLooper.playing());
  transportLooper.tick(4199999, emit, release, &transportProbe);
  assert(transportProbe.emitted == 1);
  transportLooper.tick(4200000, emit, release, &transportProbe);
  assert(transportProbe.emitted == 2);
  assert(transportProbe.last.status == 0x80);

  FourTrackLooper transportRecordLooper;
  transportRecordLooper.armRecord(0, 1000000, false);
  assert(transportRecordLooper.beginArmedRecording(5000000));
  assert(transportRecordLooper.recording());
  assert(!transportRecordLooper.recordingArmed());
  assert(transportRecordLooper.capture(
      5250000, LoopMidiEvent{0, 0x90, 69, 100}));
  assert(transportRecordLooper.track(0).count == 1);
  assert(!transportRecordLooper.beginArmedRecording(5500000));

  // A replacement take clears both the sounding window and retained copies.
  resizeLooper.armRecord(0, 250000, false);
  assert(resizeLooper.capture(2000000, LoopMidiEvent{0, 0x90, 67, 100}));
  assert(resizeLooper.capture(2100000, LoopMidiEvent{0, 0x80, 67, 0}));
  assert(resizeLooper.finishRecording(2250000));
  assert(resizeLooper.track(0).count == 2);
  assert(resizeLooper.track(0).storedLengthUs == 250000);

  // Clearing an empty track is a no-op. A hidden empty track could never be
  // brought back by Undo, and it would make "is anything cleared" answer yes
  // for a looper that holds nothing.
  FourTrackLooper clearLooper;
  Probe clearProbe;
  clearLooper.safeClear(2, release, &clearProbe);
  assert(!clearLooper.track(2).hidden);
  assert(!clearLooper.trackHasContent(2));

  // Arming is not destructive. A cleared track keeps its undo material until a
  // capture actually replaces it, so the armed target can still move away.
  clearLooper.armRecord(0, 250000, false);
  assert(clearLooper.capture(1000000, LoopMidiEvent{0, 0x90, 60, 100}));
  assert(clearLooper.finishRecording(1250000));
  assert(clearLooper.trackHasContent(0));
  clearLooper.safeClear(0, release, &clearProbe);
  assert(!clearLooper.trackHasContent(0));
  assert(clearLooper.track(0).count == 2);
  clearLooper.armRecord(0, 250000, false);
  assert(clearLooper.track(0).count == 2);  // still recoverable while armed
  clearLooper.cancelRecording();
  clearLooper.undoClear(0);
  assert(clearLooper.trackHasContent(0));

  // A pending record follows the working track. Landing on audible content
  // layers onto it instead of erasing it, and landing on a cleared track
  // replaces it.
  clearLooper.start(2000000);
  clearLooper.armRecord(1, 250000, false);
  assert(clearLooper.recordingArmed());
  clearLooper.armRecord(0, 250000, false);  // retargeted onto live content
  assert(clearLooper.recordingTrack() == 0);
  assert(clearLooper.track(1).count == 0);
  assert(clearLooper.capture(2100000, LoopMidiEvent{0, 0x90, 64, 100}));
  assert(clearLooper.overdubbing());
  assert(clearLooper.track(0).count == 3);  // the earlier take survived
  assert(clearLooper.finishRecording(2150000));

  clearLooper.safeClear(0, release, &clearProbe);
  clearLooper.armRecord(0, 250000, false);
  assert(clearLooper.capture(2200000, LoopMidiEvent{0, 0x90, 67, 100}));
  assert(!clearLooper.overdubbing());  // a cleared track takes a fresh take
  assert(clearLooper.track(0).count == 1);
  assert(!clearLooper.track(0).hidden);
  assert(clearLooper.finishRecording(2250000));

  // Tracks may begin at different points in the shared cycle. Stopping and
  // starting again, including a clear and undo in between, has to bring them
  // back in the same alignment rather than restarting every track at zero.
  FourTrackLooper syncLooper;
  Probe syncProbe;
  syncLooper.armRecord(0, 1000000, false);
  assert(syncLooper.capture(1000000, LoopMidiEvent{0, 0x90, 48, 100}));
  assert(syncLooper.capture(1100000, LoopMidiEvent{0, 0x80, 48, 0}));
  assert(syncLooper.finishRecording(2000000));
  syncLooper.start(2000000);

  // The second take begins a quarter of the way through the first loop.
  syncLooper.armRecord(1, 1000000, false);
  assert(syncLooper.capture(2250000, LoopMidiEvent{0, 0x90, 55, 100}));
  assert(syncLooper.capture(2350000, LoopMidiEvent{0, 0x80, 55, 0}));
  syncLooper.tick(3250000, emit, release, &syncProbe);
  assert(!syncLooper.recording());
  assert(syncLooper.track(1).lengthUs == 1000000);
  assert(syncLooper.track(0).startOffsetUs == 0);
  assert(syncLooper.track(1).startOffsetUs == 250000);

  syncLooper.stop(release, &syncProbe);
  syncLooper.safeClear(0, release, &syncProbe);
  syncLooper.safeClear(1, release, &syncProbe);
  syncLooper.undoClear(0);
  syncLooper.undoClear(1);
  syncLooper.start(9000000);
  assert(syncLooper.track(0).cycleStartUs == 9000000);
  // Track two is three quarters of the way through its loop at the top of the
  // transport cycle, so its own boundary still lands a quarter bar later.
  assert(syncLooper.track(1).cycleStartUs == 9000000 - 750000);
  assert(syncLooper.track(1).startOffsetUs == 250000);

  Probe syncPlayback;
  syncLooper.tick(9000000, emit, release, &syncPlayback);
  assert(syncPlayback.emitted == 1 && syncPlayback.lastTrack == 0);
  syncLooper.tick(9240000, emit, release, &syncPlayback);
  assert(syncPlayback.emitted == 2);  // track one's Note Off only
  syncLooper.tick(9260000, emit, release, &syncPlayback);
  assert(syncPlayback.emitted == 3 && syncPlayback.lastTrack == 1 &&
         syncPlayback.last.status == 0x90);

  looper.clearAll(release, &probe);
  assert(looper.usedEvents() == 0);
  assert(!looper.hasAnyData());
  return 0;
}
