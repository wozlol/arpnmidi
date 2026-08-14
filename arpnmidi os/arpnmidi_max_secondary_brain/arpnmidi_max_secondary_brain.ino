/*
  rp2040_usb_din_midi_bridge.txt
  RP2040 / RP2040-Zero USB MIDI plus serial MIDI router

  Purpose:
  - MAX3421E-secondary version for the next PCB direction.
  - MAX3421E will own USB host MIDI through SPI0 and the USB2514B hub when
    ARPNMIDI_SECONDARY_PIN_PROFILE_NEW_MAX_PCB is selected below.
  - GP4/GP5 are the main-brain MIDI serial link.
  - The pin profile below selects old/no-MAX bench wiring or new MAX PCB wiring.

  Special routing:
  - USB device MIDI in from the computer goes only to GP4 serial TX, into the main brain.
  - GP5 serial RX from the main brain goes to USB device MIDI out and the
    profile-selected external serial MIDI TX.
  - GP5 serial RX from the main brain also goes to all MAX3421E-hosted USB MIDI outs.
  - MAX3421E USB host MIDI merges only to GP4 serial TX, into the main brain.
  - The profile-selected external serial MIDI RX goes only to GP4 serial TX,
    into the main brain.
  - Nothing received by the secondary is routed directly to another secondary output.

  Wiring, matching the ARPnMIDI PCB pin convention:
  - GP0 = MAX3421E SPI0 MISO  (new MAX PCB profile)
  - GP1 = MAX3421E CS          (new MAX PCB profile)
  - GP2 = MAX3421E SPI0 SCK    (new MAX PCB profile) or external MIDI TX (old/no-MAX profile)
  - GP3 = MAX3421E SPI0 MOSI   (new MAX PCB profile) or external MIDI RX (old/no-MAX profile)
  - GP4 = main-brain MIDI serial TX from this RP2040
  - GP5 = main-brain MIDI serial RX into this RP2040
  - GP20 = external MIDI serial TX from this RP2040 (new MAX PCB profile)
  - GP21 = external MIDI serial RX into this RP2040 (new MAX PCB profile)
  - GP22 = optional MAX3421E RESET_N hardware line (new MAX PCB profile; not driven by firmware)
  - GP26 = MAX3421E INT
  - GP24 = USB2514B RESET_N control for the external hub chip

  USB2514B RESET_N handling:
  - RESET_N is active low. The board already has a 10k pullup to 3.3 V and 1 uF to ground.
  - In this MAX3421E-secondary build, the secondary owns host timing and hub reset locally.
  - The old GP4/GP5 command bus is not used because GP4/GP5 are now MIDI serial.

  Arduino setup:
  - Board: RP2040 / RP2040-Zero using Earle Philhower Arduino-Pico core
  - Tools -> USB Stack = Adafruit TinyUSB

  Notes:
  - Main-brain MIDI uses hardware UART Serial2/UART1 at 1 Mbps on GP4/GP5.
  - External MIDI uses SerialPIO at 31250 baud.
  - MAX3421E host support uses USB Host Shield 2.0's hub and MIDI classes.
  - It forwards channel voice, system common, and real-time MIDI.
  - It does not implement full streaming SysEx parsing from serial inputs.
*/

#include <Arduino.h>
#include <SPI.h>

// RP2040 CMSIS headers define USB as a hardware register macro. USB Host
// Shield 2.0 also names its MAX3421E host controller class USB.
#ifdef USB
#undef USB
#endif
#ifdef _usb_h_
#undef _usb_h_
#endif
#ifdef USBCORE_H
#undef USBCORE_H
#endif

// Vendored, PATCHED USB Host Shield 2.0 in this sketch's src/ folder — the exact
// working library travels with the code (self-contained, immune to global-library
// updates). Do NOT switch back to <angle-bracket> or the global lib would be used.
#include "src/USB_Host_Shield_Library_2.0/usbhub.h"
#include "src/USB_Host_Shield_Library_2.0/usbh_midi.h"
#include "src/rt_queue.h"
#include <Adafruit_TinyUSB.h>

// Select one pin profile:
// - OLD_NO_MAX: current bench wiring, external DIN on GP2/GP3, MAX/SPI disabled.
// - NEW_MAX_PCB: new PCB wiring, MAX on SPI0 GP0-GP3, external DIN on GP20/GP21.
#define ARPNMIDI_SECONDARY_PIN_PROFILE_OLD_NO_MAX 0
#define ARPNMIDI_SECONDARY_PIN_PROFILE_NEW_MAX_PCB 1
#ifndef ARPNMIDI_SECONDARY_PIN_PROFILE
#define ARPNMIDI_SECONDARY_PIN_PROFILE ARPNMIDI_SECONDARY_PIN_PROFILE_OLD_NO_MAX
#endif

constexpr uint8_t PIN_MAIN_BRAIN_MIDI_TX = 4;
constexpr uint8_t PIN_MAIN_BRAIN_MIDI_RX = 5;

#if ARPNMIDI_SECONDARY_PIN_PROFILE == ARPNMIDI_SECONDARY_PIN_PROFILE_NEW_MAX_PCB
constexpr bool ENABLE_MAX3421E_HOST = true;
constexpr uint8_t PIN_MAX_SPI_MISO = 0;  // SPI0 RX
constexpr uint8_t PIN_MAX_CS = 1;
constexpr uint8_t PIN_MAX_SPI_SCK = 2;
constexpr uint8_t PIN_MAX_SPI_MOSI = 3;
constexpr uint8_t PIN_EXTERNAL_MIDI_TX = 20;
constexpr uint8_t PIN_EXTERNAL_MIDI_RX = 21;
constexpr uint8_t PIN_USB_HUB_RESET_N = 24;
constexpr uint8_t PIN_MAX_INT = 26;
#else
constexpr bool ENABLE_MAX3421E_HOST = false;
constexpr uint8_t PIN_MAX_SPI_MISO = 0;
constexpr uint8_t PIN_MAX_CS = 1;
constexpr uint8_t PIN_MAX_SPI_SCK = 2;
constexpr uint8_t PIN_MAX_SPI_MOSI = 3;
constexpr uint8_t PIN_EXTERNAL_MIDI_TX = 2;
constexpr uint8_t PIN_EXTERNAL_MIDI_RX = 3;
constexpr uint8_t PIN_USB_HUB_RESET_N = 24;
constexpr uint8_t PIN_MAX_INT = 26;
#endif

constexpr uint8_t MAX_HOST_MIDI_DEVICE_COUNT = 4;
constexpr uint32_t INTER_BRAIN_MIDI_BAUD = 1000000UL;
constexpr uint32_t EXTERNAL_MIDI_BAUD = 31250UL;
constexpr uint32_t USB_HUB_RESET_PULSE_MS = 25;
constexpr uint32_t USB_HUB_RESET_RELEASE_SETTLE_MS = 5;
constexpr size_t CORE_MIDI_QUEUE_CAPACITY = 128;

// The GP4/GP5 link to the main brain carries plain MIDI bytes, so these two
// values are picked from the handful of status bytes the MIDI spec leaves
// permanently undefined/reserved: no compliant sender, hosted device, or
// external gear will ever produce them, so there is no message they could
// ever collide with, and each is its own complete one-byte message, no
// running status or data bytes to keep in sync. Both directions intercept
// their own byte before it reaches externalMidi/sendSecondaryOutputs or the
// main brain's own MIDI parser, so neither one ever leaks out as if it were
// real MIDI. Must match arpnmidi_main_brain.ino exactly.
//
// MAIN_BRAIN_BOOT_SYNC_BYTE: sent once by the main brain right after its own
// setup(), so a main-brain-only reboot (flashing, watchdog, brownout) always
// gets a clean handshake even if the reset glitched a partial byte onto the
// wire first. This side resets mainBrainParser to a fresh state on receipt,
// same as this side's own setup() already leaves it, discarding whatever
// stray status/data bytes were mid-flight when the reset happened rather
// than leaving them to desync the next real message. A secondary-only
// reboot needs no matching signal in the other direction: this side's own
// mainBrainParser already starts fresh on its own setup(), and the main
// brain's MIDI library resyncs on the next valid status byte the same way
// any compliant MIDI receiver does.
constexpr uint8_t MAIN_BRAIN_BOOT_SYNC_BYTE = 0xF4;
// SECONDARY_BACK_COMMAND_BYTE: sent once per clean press of the GP26 button,
// see PIN_MENU_BACK below. The main brain treats receiving it as though Back
// or Cancel had just been selected out of whatever menu is open, one level
// up, and does nothing if already at the top.
constexpr uint8_t SECONDARY_BACK_COMMAND_BYTE = 0xF5;

// A momentary button to GP26, idle low (external pulldown or the internal
// one enabled below), that asks the main brain to back out of whatever menu
// it has open. Only meaningful on the OLD_NO_MAX pin profile: GP26 is
// PIN_MAX_INT, the MAX3421E's own interrupt line, on the NEW_MAX_PCB
// profile, so this stays off there rather than fight over the pin.
constexpr uint8_t PIN_MENU_BACK = 26;
constexpr bool ENABLE_MENU_BACK_BUTTON = !ENABLE_MAX3421E_HOST;
constexpr uint32_t MENU_BACK_DEBOUNCE_MS = 30;

// Subclass to expose whether a hosted device has a host->device (OUT) endpoint.
// Sending to a device without one targets endpoint 0 and wedges the entire
// MAX3421E host stack after a couple seconds. Guard every SendData() with hasOut().
class RoutableMidi : public USBH_MIDI {
public:
  RoutableMidi(USB *p) : USBH_MIDI(p) {}
  bool hasOut() { return epInfo[epDataOutIndex].epAddr != 0; }
};

Adafruit_USBD_MIDI usb_midi;
SerialPIO externalMidi(PIN_EXTERNAL_MIDI_TX, PIN_EXTERNAL_MIDI_RX);
USB maxUsb;
USBHub maxHub(&maxUsb);
RoutableMidi maxMidi0(&maxUsb);
RoutableMidi maxMidi1(&maxUsb);
RoutableMidi maxMidi2(&maxUsb);
RoutableMidi maxMidi3(&maxUsb);
RoutableMidi *maxMidiDevices[MAX_HOST_MIDI_DEVICE_COUNT] = {
  &maxMidi0,
  &maxMidi1,
  &maxMidi2,
  &maxMidi3
};

struct SerialMidiParser {
  uint8_t runningStatus = 0;
  uint8_t data[2] = {0, 0};
  uint8_t needed = 0;
  uint8_t have = 0;
};

struct CoreMidiPacket {
  uint8_t status = 0;
  uint8_t data1 = 0;
  uint8_t data2 = 0;
  uint8_t len = 0;
};

SerialMidiParser mainBrainParser;
SerialMidiParser externalParser;
arpnmidi3::RtQueue<CoreMidiPacket, CORE_MIDI_QUEUE_CAPACITY> maxToIoQueue;
arpnmidi3::RtQueue<CoreMidiPacket, CORE_MIDI_QUEUE_CAPACITY> ioToMaxQueue;
uint32_t usbHubLastResetMs = 0;
bool menuBackPinWasHigh = false;
uint32_t menuBackPinChangeMs = 0;
volatile bool ioCoreReady = false;
volatile bool maxHostReady = false;
int8_t maxInitResult = 99;
uint8_t maxRevision = 0;

uint8_t midiDataLength(uint8_t status) {
  if (status >= 0xF8) return 0;

  if (status >= 0xF0) {
    switch (status) {
      case 0xF1:
      case 0xF3:
        return 1;
      case 0xF2:
        return 2;
      default:
        return 0;
    }
  }

  switch (status & 0xF0) {
    case 0x80:
    case 0x90:
    case 0xA0:
    case 0xB0:
    case 0xE0:
      return 2;
    case 0xC0:
    case 0xD0:
      return 1;
    default:
      return 0;
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
    case 0x2:
    case 0x6:
    case 0xC:
    case 0xD:
      return 2;
    case 0x3:
    case 0x4:
    case 0x7:
    case 0x8:
    case 0x9:
    case 0xA:
    case 0xB:
    case 0xE:
      return 3;
    case 0x5:
    case 0xF:
      return 1;
    default:
      return 0;
  }
}

void sendUsbMidi(uint8_t status, uint8_t data1, uint8_t data2, uint8_t len) {
  uint8_t packet[4] = {
    cinFromStatus(status, len),
    status,
    data1,
    data2
  };
  usb_midi.writePacket(packet);
}

void sendMaxHostMidi(uint8_t status, uint8_t data1, uint8_t data2, uint8_t len) {
  if (!ENABLE_MAX3421E_HOST || len == 0) return;
  ioToMaxQueue.push(CoreMidiPacket{status, data1, data2, len});
}

void sendMaxHostMidiNow(const CoreMidiPacket &packet) {
  if (!maxHostReady || packet.len == 0) return;
  uint8_t msg[3] = {packet.status, packet.data1, packet.data2};
  for (uint8_t i = 0; i < MAX_HOST_MIDI_DEVICE_COUNT; ++i) {
    RoutableMidi *midi = maxMidiDevices[i];
    // Only send to devices that actually have an OUT endpoint — sending to an
    // IN-only controller targets endpoint 0 and wedges the host stack.
    if (midi && *midi && midi->hasOut()) {
      midi->SendData(msg);
    }
  }
}

void sendSecondaryOutputs(uint8_t status, uint8_t data1, uint8_t data2, uint8_t len) {
  sendUsbMidi(status, data1, data2, len);
  sendMaxHostMidi(status, data1, data2, len);
}

void writeSerialMidi(Print &out, uint8_t status, uint8_t data1, uint8_t data2, uint8_t len) {
  if (len >= 1) out.write(status);
  if (len >= 2) out.write(data1);
  if (len >= 3) out.write(data2);
}

void sendMainBrainMidi(uint8_t status, uint8_t data1, uint8_t data2, uint8_t len) {
  writeSerialMidi(Serial2, status, data1, data2, len);
}

void holdUsbHubInReset() {
  digitalWrite(PIN_USB_HUB_RESET_N, LOW);
  pinMode(PIN_USB_HUB_RESET_N, OUTPUT);
}

void releaseUsbHubReset() {
  pinMode(PIN_USB_HUB_RESET_N, INPUT);
}

void releaseUsbHubForLocalHost() {
  releaseUsbHubReset();
  usbHubLastResetMs = millis();
  delay(USB_HUB_RESET_RELEASE_SETTLE_MS);
}

void configureMax3421ePinsIdle() {
  pinMode(PIN_MAX_CS, OUTPUT);
  digitalWrite(PIN_MAX_CS, HIGH);
  pinMode(PIN_MAX_INT, INPUT_PULLUP);
}

void setupMaxHost() {
  if (!ENABLE_MAX3421E_HOST) {
    maxHostReady = false;
    maxInitResult = -1;
    maxRevision = 0;
    return;
  }

  SPI.setRX(PIN_MAX_SPI_MISO);
  SPI.setSCK(PIN_MAX_SPI_SCK);
  SPI.setTX(PIN_MAX_SPI_MOSI);
  SPI.begin();  // no setCS — USB Host Shield 2.0 drives CS manually via SS

  pinMode(PIN_MAX_CS, OUTPUT);
  digitalWrite(PIN_MAX_CS, HIGH);
  pinMode(PIN_MAX_INT, INPUT_PULLUP);

  maxInitResult = maxUsb.Init();
  maxRevision = maxUsb.regRd(rREVISION);
  maxHostReady = (maxInitResult == 0);
}

void processSerialMidiByte(SerialMidiParser &parser, uint8_t b,
                           void (*sendMessage)(uint8_t, uint8_t, uint8_t, uint8_t)) {
  if (b >= 0xF8) {
    sendMessage(b, 0, 0, 1);
    return;
  }

  if (b & 0x80) {
    parser.runningStatus = b;
    parser.needed = midiDataLength(b);
    parser.have = 0;

    if (parser.needed == 0) {
      sendMessage(b, 0, 0, 1);
      if (b >= 0xF0) parser.runningStatus = 0;
    }
    return;
  }

  if (!parser.runningStatus || !parser.needed) return;

  parser.data[parser.have++] = b;
  if (parser.have < parser.needed) return;

  const uint8_t status = parser.runningStatus;
  const uint8_t data1 = parser.data[0];
  const uint8_t data2 = (parser.needed > 1) ? parser.data[1] : 0;
  sendMessage(status, data1, data2, parser.needed + 1);
  parser.have = 0;

  if (status >= 0xF0) parser.runningStatus = 0;
}

void pumpMainBrainToUsbAndExternal() {
  while (Serial2.available() > 0) {
    const uint8_t b = static_cast<uint8_t>(Serial2.read());
    if (b == MAIN_BRAIN_BOOT_SYNC_BYTE) {
      // The main brain just booted and may have glitched a stray byte onto
      // the wire during its own reset. Drop whatever this parser thought it
      // was mid-message on rather than let it desync the next real one.
      mainBrainParser = SerialMidiParser{};
      continue;
    }
    externalMidi.write(b);
    processSerialMidiByte(mainBrainParser, b, sendSecondaryOutputs);
  }
}

void pollMenuBackButton() {
  if (!ENABLE_MENU_BACK_BUTTON) return;
  const bool nowHigh = digitalRead(PIN_MENU_BACK) == HIGH;
  const uint32_t now = millis();
  if (nowHigh != menuBackPinWasHigh && (now - menuBackPinChangeMs) > MENU_BACK_DEBOUNCE_MS) {
    menuBackPinChangeMs = now;
    menuBackPinWasHigh = nowHigh;
    if (nowHigh) writeSerialMidi(Serial2, SECONDARY_BACK_COMMAND_BYTE, 0, 0, 1);
  }
}

void pumpUsbToMainBrain() {
  uint8_t packet[4];
  while (usb_midi.readPacket(packet)) {
    const uint8_t len = midiPacketDataLength(packet[0]);
    for (uint8_t i = 0; i < len; ++i) {
      Serial2.write(packet[i + 1]);
    }
  }
}

void pumpExternalToMainBrain() {
  while (externalMidi.available() > 0) {
    processSerialMidiByte(externalParser, static_cast<uint8_t>(externalMidi.read()), sendMainBrainMidi);
  }
}

// SysEx is intentionally DROPPED for now. This machine doesn't process SysEx,
// and passing it on is harmful: raw SysEx forwarded to a device can be truncated
// and hang its parser (the KO II froze/reset on the keyboard's ch10 pad SysEx),
// and streaming it wastes RP time for data we never use. Track SysEx state per
// source device so continuation packets are caught. (Proprietary SysEx may be
// added later, handled explicitly rather than blindly forwarded.)
static bool isSysExAndTrack(uint8_t srcIndex, const uint8_t *msg, uint8_t len) {
  static bool inSysEx[MAX_HOST_MIDI_DEVICE_COUNT] = { false };
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

void pumpMaxHostToMainBrain() {
  if (!maxHostReady) return;
  uint8_t msg[3];
  for (uint8_t i = 0; i < MAX_HOST_MIDI_DEVICE_COUNT; ++i) {
    RoutableMidi *midi = maxMidiDevices[i];
    if (!midi || !*midi) continue;
    uint8_t len = 0;
    while ((len = midi->RecvData(msg)) > 0) {
      if (isSysExAndTrack(i, msg, len)) continue;   // drop SysEx entirely
      // Queue the complete message so the MAX host core never blocks on UART.
      // The direct inter-brain link runs much faster than a physical MIDI cable.
      const CoreMidiPacket packet{msg[0], static_cast<uint8_t>(len > 1 ? msg[1] : 0),
                                  static_cast<uint8_t>(len > 2 ? msg[2] : 0), len};
      maxToIoQueue.push(packet);
    }
  }
}

void pumpQueuedMaxInputToMainBrain() {
  static CoreMidiPacket pending{};
  static bool hasPending = false;
  while (true) {
    if (!hasPending) hasPending = maxToIoQueue.pop(pending);
    if (!hasPending) return;
    // Arduino-Pico reports SerialUART::availableForWrite() as a writable
    // flag (0/1), not the number of free bytes in the UART FIFO.
    if (Serial2.availableForWrite() == 0) return;
    writeSerialMidi(Serial2, pending.status, pending.data1, pending.data2, pending.len);
    hasPending = false;
  }
}

void pumpQueuedMainOutputToMax() {
  CoreMidiPacket packet;
  while (ioToMaxQueue.pop(packet)) sendMaxHostMidiNow(packet);
}

void setup() {
  if (!TinyUSBDevice.isInitialized()) {
    TinyUSBDevice.begin(0);
  }

  TinyUSBDevice.setManufacturerDescriptor("WozAction2");
  TinyUSBDevice.setProductDescriptor("WozAction2");
  usb_midi.setStringDescriptor("WozAction2");
  usb_midi.begin();

  if (TinyUSBDevice.mounted()) {
    TinyUSBDevice.detach();
    delay(10);
    TinyUSBDevice.attach();
  }

  Serial2.setTX(PIN_MAIN_BRAIN_MIDI_TX);
  Serial2.setRX(PIN_MAIN_BRAIN_MIDI_RX);
  Serial2.begin(INTER_BRAIN_MIDI_BAUD);

  if (ENABLE_MENU_BACK_BUTTON) {
    pinMode(PIN_MENU_BACK, INPUT_PULLDOWN);
    // Read the real starting level rather than assume low, so a button
    // already held at boot cannot read as a fresh press the instant the
    // debounce window opens.
    menuBackPinWasHigh = digitalRead(PIN_MENU_BACK) == HIGH;
    menuBackPinChangeMs = millis();
  }

  externalMidi.begin(EXTERNAL_MIDI_BAUD);
  __atomic_store_n(&ioCoreReady, true, __ATOMIC_RELEASE);
}

void loop() {
#ifdef TINYUSB_NEED_POLLING_TASK
  TinyUSBDevice.task();
#endif
  pumpUsbToMainBrain();
  pumpQueuedMaxInputToMainBrain();
  pumpMainBrainToUsbAndExternal();
  pumpExternalToMainBrain();
  pollMenuBackButton();
}

void setup1() {
  while (!__atomic_load_n(&ioCoreReady, __ATOMIC_ACQUIRE)) delay(1);
  if (ENABLE_MAX3421E_HOST) {
    // Hub reset and all MAX/SPI ownership remain entirely on the host core.
    if (PIN_USB_HUB_RESET_N != PIN_MAX_INT) {
      holdUsbHubInReset();
      configureMax3421ePinsIdle();
      delay(USB_HUB_RESET_PULSE_MS);
      releaseUsbHubForLocalHost();
    } else {
      configureMax3421ePinsIdle();
    }
    setupMaxHost();
  } else {
    maxHostReady = false;
    maxInitResult = -1;
    maxRevision = 0;
  }
}

void loop1() {
  if (maxHostReady) {
    maxUsb.Task();
    pumpMaxHostToMainBrain();
    pumpQueuedMainOutputToMax();
  }
}
