/*
  max3421e_debug.ino
  MAX3421E USB host <-> RP2040 USB device MIDI bridge — debug/sanity-check build.

  Routes MAX3421E host MIDI directly to/from the RP2040's own USB device MIDI port.
  No main brain, no serial links. Plug a raw RP2040-Zero into USB and use this to
  verify that the MAX3421E SPI host is actually working end-to-end.

  MAX3421E host RX  →  USB device MIDI TX  (keyboard notes appear on USB)
  USB device MIDI RX  →  MAX3421E host TX  (send MIDI back out to host device)

  Pins (ARPnMIDI PCB):
    GP0   MAX3421E MISO  (SPI0 RX — GP0 is a valid SPI0 RX pin, selected via SPI.setRX(0))
    GP1   MAX3421E CS    (library-controlled, not hardware SPI CS)
    GP2   MAX3421E SCK   (SPI0 SCK)
    GP3   MAX3421E MOSI  (SPI0 TX)
    GP26  MAX3421E INT

  Arduino setup:
    Board: Waveshare RP2040-Zero  |  USB Stack: Adafruit TinyUSB
*/

#include <Arduino.h>
#include <SPI.h>

// RP2040 CMSIS defines USB as a hardware peripheral macro — clear it before
// including USB Host Shield 2.0 which defines a class named USB.
#ifdef USB
#undef USB
#endif
#ifdef _usb_h_
#undef _usb_h_
#endif
#ifdef USBCORE_H
#undef USBCORE_H
#endif

#include <usbhub.h>
#include <usbh_midi.h>
#include <Adafruit_TinyUSB.h>

constexpr uint8_t PIN_MAX_SPI_MISO        = 0;  // ARPnMIDI PCB: SPI0 MISO on GP0 (valid SPI0 RX)
constexpr uint8_t PIN_MAX_CS              = 1;
constexpr uint8_t PIN_MAX_SPI_SCK         = 2;
constexpr uint8_t PIN_MAX_SPI_MOSI        = 3;
constexpr uint8_t PIN_MAX_INT             = 26;
constexpr uint8_t MAX_HOST_MIDI_DEVICES   = 4;

// Subclass to expose whether a hosted device actually has a host->device (OUT)
// endpoint. Sending to a device without one targets endpoint 0 and wedges the
// host stack — that's why routing to an IN-only keyboard killed everything after
// a couple seconds. hasOut() lets us skip devices that can't receive.
class RoutableMidi : public USBH_MIDI {
public:
  RoutableMidi(USB *p) : USBH_MIDI(p) {}
  bool hasOut() { return epInfo[epDataOutIndex].epAddr != 0; }
};

Adafruit_USBD_MIDI usb_midi;
USB     maxUsb;
USBHub  maxHub(&maxUsb);
RoutableMidi maxMidi0(&maxUsb);
RoutableMidi maxMidi1(&maxUsb);
RoutableMidi maxMidi2(&maxUsb);
RoutableMidi maxMidi3(&maxUsb);
RoutableMidi *maxMidiDevices[MAX_HOST_MIDI_DEVICES] = {
  &maxMidi0, &maxMidi1, &maxMidi2, &maxMidi3
};

bool    maxHostReady  = false;
int8_t  maxInitResult = 99;
uint8_t maxRevision   = 0;

// host->host send-routing diagnostics
uint32_t routeSendOk    = 0;
uint32_t routeSendErr   = 0;
uint8_t  routeLastRcode = 0;
uint32_t hostRecoveries = 0;   // times the watchdog re-inited a wedged host

struct SerialMidiParser {
  uint8_t runningStatus = 0;
  uint8_t data[2]       = {0, 0};
  uint8_t needed        = 0;
  uint8_t have          = 0;
};
SerialMidiParser maxParser;

// ── MIDI helpers ─────────────────────────────────────────────────────────────

uint8_t midiDataLength(uint8_t status) {
  if (status >= 0xF8) return 0;
  if (status >= 0xF0) {
    switch (status) {
      case 0xF1: case 0xF3: return 1;
      case 0xF2: return 2;
      default:   return 0;
    }
  }
  switch (status & 0xF0) {
    case 0x80: case 0x90: case 0xA0: case 0xB0: case 0xE0: return 2;
    case 0xC0: case 0xD0: return 1;
    default: return 0;
  }
}

uint8_t cinFromStatus(uint8_t status, uint8_t len) {
  if (status >= 0xF8) return 0x0F;
  if (status >= 0xF0) {
    switch (status) {
      case 0xF1: return 0x02;
      case 0xF2: return 0x03;
      case 0xF3: return 0x02;
      case 0xF6: return 0x05;
      default:   return (len == 1) ? 0x05 : ((len == 2) ? 0x06 : 0x04);
    }
  }
  switch (status & 0xF0) {
    case 0x80: return 0x08;
    case 0x90: return 0x09;
    case 0xA0: return 0x0A;
    case 0xB0: return 0x0B;
    case 0xC0: return 0x0C;
    case 0xD0: return 0x0D;
    case 0xE0: return 0x0E;
    default:   return 0x00;
  }
}

uint8_t midiPacketDataLength(uint8_t cin) {
  switch (cin & 0x0F) {
    case 0x2: case 0x6: case 0xC: case 0xD: return 2;
    case 0x3: case 0x4: case 0x7: case 0x8:
    case 0x9: case 0xA: case 0xB: case 0xE: return 3;
    case 0x5: case 0xF: return 1;
    default: return 0;
  }
}

// ── routing ──────────────────────────────────────────────────────────────────

void sendUsbMidi(uint8_t status, uint8_t data1, uint8_t data2, uint8_t len) {
  uint8_t packet[4] = { cinFromStatus(status, len), status, data1, data2 };
  usb_midi.writePacket(packet);
}

void processSerialMidiByte(SerialMidiParser &parser, uint8_t b,
                           void (*cb)(uint8_t, uint8_t, uint8_t, uint8_t)) {
  if (b >= 0xF8) { cb(b, 0, 0, 1); return; }
  if (b & 0x80) {
    parser.runningStatus = b;
    parser.needed = midiDataLength(b);
    parser.have = 0;
    if (parser.needed == 0) {
      cb(b, 0, 0, 1);
      if (b >= 0xF0) parser.runningStatus = 0;
    }
    return;
  }
  if (!parser.runningStatus || !parser.needed) return;
  parser.data[parser.have++] = b;
  if (parser.have < parser.needed) return;
  const uint8_t st  = parser.runningStatus;
  const uint8_t d1  = parser.data[0];
  const uint8_t d2  = (parser.needed > 1) ? parser.data[1] : 0;
  cb(st, d1, d2, parser.needed + 1);
  parser.have = 0;
  if (st >= 0xF0) parser.runningStatus = 0;
}

// SysEx is intentionally DROPPED for now. This device doesn't process SysEx,
// and forwarding it raw is actively harmful: SendData() only frames short
// channel messages, so a multi-packet SysEx gets truncated/mis-framed and can
// hang a receiving device's MIDI parser — the KO II froze and reset when the
// keyboard's ch10 pads sent SysEx. Dropping it here also keeps the RP from
// spending time shuffling SysEx it will never use. (Proprietary SysEx may be
// added later, handled explicitly rather than blindly forwarded.)
// Returns true if this USB-MIDI message is part of a SysEx stream. Tracks the
// in-progress SysEx state per source device so continuation packets are caught.
static bool isSysExAndTrack(uint8_t srcIndex, const uint8_t *msg, uint8_t len) {
  static bool inSysEx[MAX_HOST_MIDI_DEVICES] = { false };
  const uint8_t s = msg[0];
  if (s == 0xF0) { inSysEx[srcIndex] = true;  return true; }  // SysEx start
  if (s == 0xF7) { inSysEx[srcIndex] = false; return true; }  // SysEx end (EOX)
  if (s >= 0xF8) return false;                                 // real-time: always pass
  if (inSysEx[srcIndex]) {                                     // mid-SysEx data bytes
    for (uint8_t b = 0; b < len; ++b) if (msg[b] == 0xF7) inSysEx[srcIndex] = false;
    return true;
  }
  if (s < 0x80) return true;   // stray data byte, no status -> junk, drop
  return false;                // normal channel / system-common message
}

// ── Per-destination output throttle (emulate a 31250-baud MIDI cable) ─────────
// USB MIDI has no baud limit, so a burst of host MIDI — a held pad's continuous
// aftertouch/pressure especially — blasts a receiver faster than any cable could
// and crashes it (the KO II). Each host device gets a small FIFO drained at
// ~320us per MIDI byte, exactly a TRS/DIN cable's pace. Aftertouch/pressure is
// fully preserved, just smoothed; only a sustained over-rate (which a real cable
// couldn't carry either) overflows and drops the oldest queued message.
constexpr uint8_t  MIDI_OUT_QUEUE_LEN = 64;
constexpr uint32_t MIDI_BYTE_MICROS   = 320;   // one byte at 31250 baud, 8N1

struct MidiOutQueue {
  uint8_t  msg[MIDI_OUT_QUEUE_LEN][3];
  uint8_t  len[MIDI_OUT_QUEUE_LEN];
  uint8_t  head, tail, count;
  uint32_t nextSendMicros;
};
MidiOutQueue midiOutQ[MAX_HOST_MIDI_DEVICES];

// Note-on/off must never be dropped — losing a note-off leaves a stuck note
// (the "pulsing stuck pads" symptom). 0x80 = note off, 0x90 = note on.
static inline bool isNoteMsg(uint8_t status) {
  const uint8_t hi = status & 0xF0;
  return hi == 0x80 || hi == 0x90;
}

void enqueueMidiOut(uint8_t dev, const uint8_t *m, uint8_t l) {
  MidiOutQueue &q = midiOutQ[dev];
  const bool note = isNoteMsg(m[0]);
  // Overflow protection that TOSSES before it can hurt: once the queue is half
  // full, shed incoming NON-note traffic (aftertouch/pressure/CC) so a held-pad
  // flood can never crowd out notes. Notes are always admitted; only a queue that
  // is somehow entirely notes drops its oldest to keep moving.
  if (!note && q.count >= (MIDI_OUT_QUEUE_LEN / 2)) return;
  if (q.count >= MIDI_OUT_QUEUE_LEN) {
    q.head = (q.head + 1) % MIDI_OUT_QUEUE_LEN;
    q.count--;
  }
  q.msg[q.tail][0] = m[0];
  q.msg[q.tail][1] = (l > 1) ? m[1] : 0;
  q.msg[q.tail][2] = (l > 2) ? m[2] : 0;
  q.len[q.tail] = l;
  q.tail = (q.tail + 1) % MIDI_OUT_QUEUE_LEN;
  q.count++;
}

void drainMidiOutQueues() {
  const uint32_t now = micros();
  for (uint8_t d = 0; d < MAX_HOST_MIDI_DEVICES; ++d) {
    MidiOutQueue &q = midiOutQ[d];
    if (q.count == 0) continue;
    if ((int32_t)(now - q.nextSendMicros) < 0) continue;   // still within the cable-rate gap
    RoutableMidi *dst = maxMidiDevices[d];
    if (!dst || !*dst || !dst->hasOut()) { q.head = q.tail = q.count = 0; continue; } // gone: flush
    const uint8_t l = q.len[q.head];
    const uint8_t rc = dst->SendData(q.msg[q.head]);
    if (rc) { routeSendErr++; routeLastRcode = rc; } else routeSendOk++;
    q.head = (q.head + 1) % MIDI_OUT_QUEUE_LEN;
    q.count--;
    q.nextSendMicros = now + (uint32_t)l * MIDI_BYTE_MICROS;
  }
}

constexpr uint32_t HOST_STUCK_TIMEOUT_MS  = 4000;  // no state change while not RUNNING -> wedged
constexpr uint32_t HOST_FROZEN_TIMEOUT_MS = 3000;  // RUNNING but queued sends won't drain -> frozen

// Recovery watchdog: re-init the MAX3421E host when it wedges so it self-heals
// instead of dying permanently. Two signals: (a) stuck in a non-RUNNING, non-idle
// state with no progress (wedged enumeration/error), or (b) RUNNING but queued
// MIDI won't drain (frozen host / device stopped accepting). Any state change or
// any successful send counts as progress, so normal (even slow) enumeration and
// normal traffic never trip it.
void serviceHostWatchdog() {
  static uint8_t  lastState      = 0xFF;
  static uint32_t lastProgressMs = 0;
  static uint32_t lastOk         = 0;
  static uint32_t lastOkMs       = 0;
  static bool     inited         = false;
  const uint32_t  nowMs = millis();
  if (!inited) { inited = true; lastProgressMs = nowMs; lastOkMs = nowMs; }

  const uint8_t st = maxUsb.getUsbTaskState();
  if (st != lastState)       { lastState = st;        lastProgressMs = nowMs; }
  if (routeSendOk != lastOk) { lastOk = routeSendOk;  lastOkMs = nowMs; }

  const bool running = (st == USB_STATE_RUNNING);
  const bool idle    = (st == USB_DETACHED_SUBSTATE_WAIT_FOR_DEVICE);

  uint16_t queued = 0;
  for (uint8_t d = 0; d < MAX_HOST_MIDI_DEVICES; ++d) queued += midiOutQ[d].count;

  bool wedged = false;
  if (!running && !idle && (nowMs - lastProgressMs > HOST_STUCK_TIMEOUT_MS)) wedged = true;   // (a)
  if (running && queued > 0 && (nowMs - lastOkMs > HOST_FROZEN_TIMEOUT_MS))  wedged = true;   // (b)

  if (wedged) {
    hostRecoveries++;
    Serial.printf("[%lu ms] host WEDGED (state=%d queued=%u) -> re-init #%lu\n",
                  nowMs, (int)st, queued, (unsigned long)hostRecoveries);
    for (uint8_t d = 0; d < MAX_HOST_MIDI_DEVICES; ++d) midiOutQ[d] = MidiOutQueue{};
    maxInitResult = maxUsb.Init();
    maxHostReady  = (maxInitResult == 0);
    lastProgressMs = nowMs;
    lastOkMs = nowMs;
    lastState = 0xFF;
  }
}

void pumpMaxHostToUsb() {
  if (!maxHostReady) return;
  uint8_t msg[3];
  for (uint8_t i = 0; i < MAX_HOST_MIDI_DEVICES; ++i) {
    RoutableMidi *src = maxMidiDevices[i];
    if (!src || !*src) continue;
    uint8_t len = 0;
    while ((len = src->RecvData(msg)) > 0) {
      if (isSysExAndTrack(i, msg, len)) continue;   // drop SysEx entirely
      // mirror to the USB device port so we can still monitor traffic
      for (uint8_t b = 0; b < len; ++b)
        processSerialMidiByte(maxParser, msg[b], sendUsbMidi);
      // Forward to every OTHER host device that can receive, THROUGH a per-device
      // throttle that paces sends to 31250 baud (a MIDI cable's rate). Aftertouch
      // and pressure are fully preserved — just smoothed, so the USB-speed burst no
      // longer overwhelms/crashes the receiver (the KO II). See enqueueMidiOut().
      for (uint8_t j = 0; j < MAX_HOST_MIDI_DEVICES; ++j) {
        if (j == i) continue;
        RoutableMidi *dst = maxMidiDevices[j];
        if (dst && *dst && dst->hasOut()) enqueueMidiOut(j, msg, len);
      }
    }
  }
}

void pumpUsbToMaxHost() {
  if (!maxHostReady) return;
  uint8_t packet[4];
  while (usb_midi.readPacket(packet)) {
    const uint8_t cin = packet[0] & 0x0F;
    if (cin >= 0x04 && cin <= 0x07) continue;  // drop SysEx from the computer too
    const uint8_t len = midiPacketDataLength(packet[0]);
    uint8_t msg[3] = { packet[1], packet[2], packet[3] };
    for (uint8_t i = 0; i < MAX_HOST_MIDI_DEVICES; ++i) {
      RoutableMidi *midi = maxMidiDevices[i];
      if (midi && *midi && midi->hasOut()) enqueueMidiOut(i, msg, len);
    }
  }
}

// ── init ─────────────────────────────────────────────────────────────────────

void setupMaxHost() {
  // Blink CS pin before SPI starts — confirm RP2040 can drive it independently.
  // Measure GP1 during this window (5 × 300ms HIGH, 300ms LOW = 3 seconds).
  pinMode(PIN_MAX_CS, OUTPUT);
  for (uint8_t i = 0; i < 5; ++i) {
    digitalWrite(PIN_MAX_CS, HIGH); delay(300);
    digitalWrite(PIN_MAX_CS, LOW);  delay(300);
  }
  digitalWrite(PIN_MAX_CS, HIGH);

  SPI.end();  // reset _begun so setRX/SCK/TX take effect even if SPI was already started
  SPI.setRX(PIN_MAX_SPI_MISO);
  SPI.setSCK(PIN_MAX_SPI_SCK);
  SPI.setTX(PIN_MAX_SPI_MOSI);
  SPI.begin();
  pinMode(PIN_MAX_INT, INPUT_PULLUP);

  // ── Raw SPI loopback self-test ─────────────────────────────────────────────
  // transfer(x) is full-duplex: it shifts x out on MOSI (GP3) and simultaneously
  // shifts MISO (GP0) into the RETURN value of the SAME call. So we must read the
  // return of the byte we send — not a following transfer(0).
  //
  //   Jumper GP3 -> GP0, no chip: every byte must echo back. Expect 90 55 AA.
  //     - got 90 55 AA  -> SPI0 is alive on GP2/GP3/GP0, GP0-as-MISO works.
  //     - got 00 00 00  -> SPI not initialized (_initted false) OR jumper/pin wrong.
  //     - got FF FF AA.. -> MISO floating / wrong RX pin.
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  uint8_t lb0 = SPI.transfer(0x90);
  uint8_t lb1 = SPI.transfer(0x55);
  uint8_t lb2 = SPI.transfer(0xAA);
  SPI.endTransaction();
  const bool loopbackOk = (lb0 == 0x90 && lb1 == 0x55 && lb2 == 0xAA);
  Serial.printf("SPI loopback (GP3->GP0): sent 90 55 AA  got %02X %02X %02X  -> %s\n",
                lb0, lb1, lb2,
                loopbackOk ? "OK: GP0-as-MISO works" : "FAIL: MOSI->MISO not echoing");

  // ── Direct chip probe (bypasses the library) ───────────────────────────────
  // REMOVE the GP3->GP4 jumper and connect the MAX3421E first. This reads the
  // REVISION register with manual CS toggling at a slow 1 MHz clock, so the chip
  // is tested independent of the library's clock/CS handling. Expect 0x13.
  //   0x13 (lib says 0x00) -> library timing/CS issue.
  //   0x00                 -> chip silent: held in RESET, SO not on GP4, or CS not reaching chip.
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_MAX_CS, LOW);
  delayMicroseconds(1);
  SPI.transfer(rREVISION);              // command byte (register + read)
  uint8_t chipRev = SPI.transfer(0x00); // chip drives REVISION onto MISO here
  delayMicroseconds(1);
  digitalWrite(PIN_MAX_CS, HIGH);
  SPI.endTransaction();
  Serial.printf("Direct chip REVISION read @1MHz (manual CS): 0x%02X (expect 0x13)\n", chipRev);

  // ── Oscillator bring-up probe ──────────────────────────────────────────────
  // REV reads 0x13 (SPI to the chip works), but Init returns -1. In this library
  // Init only returns -1 when OSCOKIRQ never asserts — i.e. the on-module 12 MHz
  // crystal isn't oscillating. Probe it directly and time how long it takes.
  //   oscOk=1 fast     -> oscillator fine; Init failure is elsewhere / transient.
  //   oscOk=0 (65535)  -> crystal not starting: bad solder, missing load caps,
  //                       or dead crystal on the module.
  maxUsb.regWr(rUSBCTL, bmCHIPRES);   // assert internal chip reset
  maxUsb.regWr(rUSBCTL, 0x00);        // release reset
  uint32_t oscStart = micros();
  uint32_t oscTries = 0;
  bool     oscOk    = false;
  while (oscTries < 65535UL) {         // same budget the library's reset() uses
    oscTries++;
    if (maxUsb.regRd(rUSBIRQ) & bmOSCOKIRQ) { oscOk = true; break; }
  }
  Serial.printf("OSC probe: oscOk=%d  tries=%lu/65535  elapsed=%luus  rUSBIRQ=0x%02X\n",
                (int)oscOk, (unsigned long)oscTries,
                (unsigned long)(micros() - oscStart),
                maxUsb.regRd(rUSBIRQ));

  // Retry Init a few times in case the oscillator just needs to settle.
  for (uint8_t attempt = 0; attempt < 5 && maxInitResult != 0; ++attempt) {
    maxInitResult = maxUsb.Init();
    if (maxInitResult != 0) delay(50);
  }
  maxRevision = maxUsb.regRd(rREVISION);
  Serial.printf("MAX3421E Init: %d (%s)  REVISION=0x%02X (expect 0x13 if SPI OK)\n",
                maxInitResult, maxInitResult == 0 ? "OK" : "FAIL", maxRevision);
  maxHostReady = (maxInitResult == 0);
}

void setup() {
  if (!TinyUSBDevice.isInitialized()) TinyUSBDevice.begin(0);
  usb_midi.setStringDescriptor("MAX3421E Debug");
  usb_midi.begin();

  if (TinyUSBDevice.mounted()) {
    TinyUSBDevice.detach();
    delay(10);
    TinyUSBDevice.attach();
  }

  setupMaxHost();
}

void loop() {
#ifdef TINYUSB_NEED_POLLING_TASK
  TinyUSBDevice.task();
#endif

  if (maxHostReady) maxUsb.Task();

  pumpMaxHostToUsb();
  pumpUsbToMaxHost();
  drainMidiOutQueues();   // paces queued host sends to 31250-baud cable rate
  serviceHostWatchdog();  // auto-recover the host if it wedges

  static uint32_t lastDiagMs = 0;
  const uint32_t  now        = millis();
  if (now - lastDiagMs >= 3000) {
    lastDiagMs = now;
    Serial.printf("[%lu ms] ready=%d state=%d REV=0x%02X routeOk=%lu routeErr=%lu lastRc=0x%02X recov=%lu\n",
                  now, (int)maxHostReady, (int)maxUsb.getUsbTaskState(), maxRevision,
                  (unsigned long)routeSendOk, (unsigned long)routeSendErr, routeLastRcode,
                  (unsigned long)hostRecoveries);
    for (uint8_t i = 0; i < MAX_HOST_MIDI_DEVICES; ++i) {
      if (maxMidiDevices[i] && maxMidiDevices[i]->GetAddress() != 0)
        Serial.printf("  MIDI[%d] addr=%d\n", i, maxMidiDevices[i]->GetAddress());
    }
  }
}
