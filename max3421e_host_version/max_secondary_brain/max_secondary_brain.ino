/*
  rp2040_usb_din_midi_bridge.txt
  RP2040 / RP2040-Zero USB MIDI plus serial MIDI router

  Purpose:
  - MAX3421E-secondary version for the next PCB direction.
  - MAX3421E will own USB host MIDI through SPI0 and the USB2514B hub.
  - GP4/GP5 are the main-brain MIDI serial link.
  - GP23/GP25 are the external MIDI serial port for this accessible-pin draft.

  Special routing:
  - USB device MIDI in from the computer goes only to GP4 serial TX, into the main brain.
  - GP5 serial RX from the main brain goes to USB device MIDI out and GP23 external serial TX.
  - GP5 serial RX from the main brain also goes to all MAX3421E-hosted USB MIDI outs.
  - MAX3421E USB host MIDI merges only to GP4 serial TX, into the main brain.
  - GP25 external serial RX goes only to GP4 serial TX, into the main brain.
  - Nothing received by the secondary is routed directly to another secondary output.

  Wiring, matching the ARPnMIDI PCB pin convention:
  - GP0 = MAX3421E SPI0 MISO  (valid SPI0 RX pin, selected via SPI.setRX(0))
  - GP1 = MAX3421E CS
  - GP2 = MAX3421E SPI0 SCK
  - GP3 = MAX3421E SPI0 MOSI
  - GP4 = main-brain MIDI serial TX from this RP2040
  - GP5 = main-brain MIDI serial RX into this RP2040
  - GP23 = external MIDI serial TX from this RP2040
  - GP25 = external MIDI serial RX into this RP2040 (TRS MIDI IN from opto)
  - GP26 = MAX3421E INT
  - GP24 = USB2514B RESET_N control for the external hub board

  USB2514B RESET_N handling:
  - RESET_N is active low. The board already has a 10k pullup to 3.3 V and 1 uF to ground.
  - In this MAX3421E-secondary build, the secondary owns host timing and hub reset locally.
  - The old GP4/GP5 command bus is not used because GP4/GP5 are now MIDI serial.

  Arduino setup:
  - Board: RP2040 / RP2040-Zero using Earle Philhower Arduino-Pico core
  - Tools -> USB Stack = Adafruit TinyUSB

  Notes:
  - Main-brain MIDI uses hardware UART Serial2/UART1 at 31250 baud on GP4/GP5.
  - External MIDI uses SerialPIO at 31250 baud on GP23/GP25.
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

#include <usbhub.h>
#include <usbh_midi.h>
#include <Adafruit_TinyUSB.h>

constexpr uint8_t PIN_MAX_SPI_MISO = 0;  // ARPnMIDI PCB: SPI0 MISO on GP0 (valid SPI0 RX)
constexpr uint8_t PIN_MAX_CS = 1;
constexpr uint8_t PIN_MAX_SPI_SCK = 2;
constexpr uint8_t PIN_MAX_SPI_MOSI = 3;
constexpr uint8_t PIN_MAIN_BRAIN_MIDI_TX = 4;
constexpr uint8_t PIN_MAIN_BRAIN_MIDI_RX = 5;
constexpr uint8_t PIN_EXTERNAL_MIDI_TX = 23;
constexpr uint8_t PIN_EXTERNAL_MIDI_RX = 25;
constexpr uint8_t PIN_USB_HUB_RESET_N = 22; //24 normally, testing
constexpr uint8_t PIN_MAX_INT = 26;
constexpr uint8_t MAX_HOST_MIDI_DEVICE_COUNT = 4;
constexpr uint32_t MIDI_BAUD = 31250;
constexpr uint32_t USB_HUB_RESET_PULSE_MS = 25;
constexpr uint32_t USB_HUB_RESET_RELEASE_SETTLE_MS = 5;

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

SerialMidiParser mainBrainParser;
SerialMidiParser externalParser;
uint32_t usbHubLastResetMs = 0;
bool maxHostReady = false;
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
  if (!maxHostReady || len == 0) return;
  uint8_t msg[3] = {status, data1, data2};
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
  SPI.setRX(PIN_MAX_SPI_MISO);
  SPI.setSCK(PIN_MAX_SPI_SCK);
  SPI.setTX(PIN_MAX_SPI_MOSI);
  SPI.begin();  // no setCS — USB Host Shield 2.0 drives CS manually via SS

  pinMode(PIN_MAX_CS, OUTPUT);
  digitalWrite(PIN_MAX_CS, HIGH);
  pinMode(PIN_MAX_INT, INPUT_PULLUP);

  maxInitResult = maxUsb.Init();
  maxRevision = maxUsb.regRd(rREVISION);
  Serial.printf("MAX3421E Init: %d (%s)  REVISION=0x%02X (expect 0x13 if SPI OK)\n",
                maxInitResult, maxInitResult == 0 ? "OK" : "FAIL", maxRevision);
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
    externalMidi.write(b);
    processSerialMidiByte(mainBrainParser, b, sendSecondaryOutputs);
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
      // Serial2 runs at 31250 baud — a real MIDI-cable rate — so it inherently
      // throttles the stream, aftertouch/pressure included (kept, not dropped).
      // Only write when the TX buffer has room for the whole message, so a
      // USB-speed pad burst can't block here and starve the host Task. Sustained
      // over-rate (which a cable couldn't carry either) is dropped, not blocked.
      if (Serial2.availableForWrite() >= (int)len) {
        for (uint8_t b = 0; b < len; ++b) {
          Serial2.write(msg[b]);
        }
      }
    }
  }
}

void setup() {
  // Only drive hub reset if the reset pin is separate from the MAX INT pin.
  // When PIN_USB_HUB_RESET_N == PIN_MAX_INT (testing mode), skip hub reset
  // entirely so we don't corrupt the INT pullup.
  if (PIN_USB_HUB_RESET_N != PIN_MAX_INT) {
    holdUsbHubInReset();
    configureMax3421ePinsIdle();
    delay(USB_HUB_RESET_PULSE_MS);
    releaseUsbHubForLocalHost();
  } else {
    configureMax3421ePinsIdle();
    Serial.println("Hub reset skipped (PIN_USB_HUB_RESET_N == PIN_MAX_INT, testing mode)");
  }

  if (!TinyUSBDevice.isInitialized()) {
    TinyUSBDevice.begin(0);
  }

  usb_midi.setStringDescriptor("RP2040 DIN MIDI Bridge");
  usb_midi.begin();

  if (TinyUSBDevice.mounted()) {
    TinyUSBDevice.detach();
    delay(10);
    TinyUSBDevice.attach();
  }

  Serial2.setTX(PIN_MAIN_BRAIN_MIDI_TX);
  Serial2.setRX(PIN_MAIN_BRAIN_MIDI_RX);
  Serial2.begin(MIDI_BAUD);

  externalMidi.begin(MIDI_BAUD);
  setupMaxHost();
}

void loop() {
#ifdef TINYUSB_NEED_POLLING_TASK
  TinyUSBDevice.task();
#endif
  if (maxHostReady) maxUsb.Task();

  pumpUsbToMainBrain();
  pumpMaxHostToMainBrain();
  pumpMainBrainToUsbAndExternal();
  pumpExternalToMainBrain();

  static uint32_t lastDiagMs = 0;
  uint32_t now = millis();
  if (now - lastDiagMs >= 3000) {
    lastDiagMs = now;
    Serial.printf("[%lu ms] maxHostReady=%d  USB state=%d  init=%d  REV=0x%02X\n",
                  now, (int)maxHostReady, (int)maxUsb.getUsbTaskState(),
                  (int)maxInitResult, maxRevision);
    for (uint8_t i = 0; i < MAX_HOST_MIDI_DEVICE_COUNT; ++i) {
      if (maxMidiDevices[i] && maxMidiDevices[i]->GetAddress() != 0) {
        Serial.printf("  MIDI[%d] addr=%d\n", i, maxMidiDevices[i]->GetAddress());
      }
    }
  }
}
