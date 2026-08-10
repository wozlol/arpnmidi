#include <assert.h>

#include "../arpnmidi os/arpnmidi_main_brain/src/echo_engine.h"

using arpnmidi3::EchoConfig;
using arpnmidi3::EchoEngine;
using arpnmidi3::LoopMidiEvent;

struct Probe {
  uint16_t count = 0;
  LoopMidiEvent events[32]{};
};

static void emit(void *context, uint8_t target, const LoopMidiEvent &event) {
  assert(target == 3);
  Probe &probe = *static_cast<Probe *>(context);
  if (probe.count < 32) probe.events[probe.count] = event;
  ++probe.count;
}

int main() {
  EchoEngine echo;
  Probe probe;
  const EchoConfig config{1000000, 200000, 50, 0};
  echo.noteOn(0, 3, 0x91, 60, 100, config);
  echo.tick(199999, emit, &probe);
  assert(probe.count == 0);
  echo.tick(200000, emit, &probe);
  assert(probe.count == 1 && probe.events[0].status == 0x91 &&
         probe.events[0].data1 == 60 && probe.events[0].data2 == 50);
  echo.noteOff(250000, 3, 0x81, 60);
  echo.tick(350000, emit, &probe);
  assert(probe.count == 2 && probe.events[1].status == 0x81);
  echo.tick(400000, emit, &probe);
  assert(probe.count == 3 && probe.events[2].status == 0x91);
  echo.stopTarget(3, emit, &probe);
  assert(probe.count == 4 && probe.events[3].status == 0x81);

  echo.reset();
  probe = Probe{};
  echo.noteOn(0, 3, 0x90, 64, 100, EchoConfig{1000000, 200000, 100, 1});
  echo.tick(200000, emit, &probe);
  echo.tick(300000, emit, &probe);  // +100 ms after Drift +1 halves spacing
  assert(probe.count >= 2);
  return 0;
}
