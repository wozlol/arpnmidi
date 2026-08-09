/*
  rp2040_usb_din_midi_bridge.txt
  RP2040 / RP2040-Zero USB MIDI plus serial MIDI router

  Purpose:
  - Route USB MIDI and two serial MIDI ports around the ARPnMIDI main brain.
  - GP0/GP1 are the main-brain MIDI serial link.
  - GP2/GP3 are the external MIDI serial port.

  Special routing:
  - USB MIDI in from the computer goes only to GP0 serial TX, into the main brain.
  - GP1 serial RX from the main brain goes to USB MIDI out and GP2 external serial TX.
  - USB MIDI is deliberately not routed directly to GP2. It must pass through the main brain first.
  - GP3 external serial RX goes only to GP0 serial TX, into the main brain.
  - GP3 external serial RX is deliberately not routed to USB.

  Wiring, matching the ARPnMIDI pin convention:
  - GP0 / board pin 0 = main-brain MIDI serial TX from this RP2040
  - GP1 / board pin 1 = main-brain MIDI serial RX into this RP2040
  - GP2 / board pin 2 = external MIDI serial TX from this RP2040
  - GP3 / board pin 3 = external MIDI serial RX into this RP2040
  - GP4 / board pin 4 = proprietary command TX from this RP2040 to main brain GP5
  - GP5 / board pin 5 = proprietary command RX into this RP2040 from main brain GP4
  - GP24 = USB2514B RESET_N control for the external hub board

  USB2514B RESET_N handling:
  - RESET_N is active low. The board already has a 10k pullup to 3.3 V and 1 uF to ground.
  - This firmware drives RESET_N low at boot and waits for the main brain to send AHR1 on GP5.
  - On AHR1, it releases RESET_N to high impedance so the external pullup/capacitor set the
    rising edge, waits a short datasheet-safe settle, then replies AHD1 to the main brain.
  - The main brain starts PIO-USB host only after AHD1, so the hub is freshly out of reset when
    the host begins enumeration.
  - Module-version default uses a short timeout so this bridge will still boot
    when the main brain is running the older RP2040 PIO-host module build and
    does not send the later PCB-hub release command.

  Arduino setup:
  - Board: RP2040 / RP2040-Zero using Earle Philhower Arduino-Pico core
  - Tools -> USB Stack = Adafruit TinyUSB

  Notes:
  - Main-brain MIDI uses hardware UART Serial1 at 31250 baud.
  - External MIDI uses SerialPIO at 31250 baud so GP2/GP3 can be used.
  - It forwards channel voice, system common, and real-time MIDI.
  - It does not implement full streaming SysEx parsing from serial inputs.
*/

#include <Arduino.h>
#include <Adafruit_TinyUSB.h>

constexpr uint8_t PIN_MAIN_BRAIN_MIDI_TX = 0;
constexpr uint8_t PIN_MAIN_BRAIN_MIDI_RX = 1;
constexpr uint8_t PIN_EXTERNAL_MIDI_TX = 2;
constexpr uint8_t PIN_EXTERNAL_MIDI_RX = 3;
constexpr uint8_t PIN_COMMAND_TX_TO_MAIN = 4;
constexpr uint8_t PIN_COMMAND_RX_FROM_MAIN = 5;
constexpr uint8_t PIN_USB_HUB_RESET_N = 24;
constexpr uint32_t MIDI_BAUD = 31250;
constexpr uint32_t COMMAND_BAUD = 115200;
constexpr uint32_t USB_HUB_RELEASE_COMMAND_TIMEOUT_MS = 500;
constexpr uint32_t USB_HUB_RESET_PULSE_MS = 25;
constexpr uint32_t USB_HUB_RESET_RELEASE_SETTLE_MS = 5;
constexpr uint32_t USB_HUB_RESET_COMMAND_IGNORE_MS = 250;
constexpr uint32_t USB_HUB_READY_SIGNAL_MS = 120;
constexpr uint8_t USB_HUB_RELEASE_COMMAND[] = {'A', 'H', 'R', '1'};
constexpr uint8_t USB_HUB_READY_RESPONSE[] = {'A', 'H', 'D', '1'};

Adafruit_USBD_MIDI usb_midi;
SerialPIO externalMidi(PIN_EXTERNAL_MIDI_TX, PIN_EXTERNAL_MIDI_RX);

struct SerialMidiParser {
  uint8_t runningStatus = 0;
  uint8_t data[2] = {0, 0};
  uint8_t needed = 0;
  uint8_t have = 0;
};

SerialMidiParser mainBrainParser;
SerialMidiParser externalParser;
uint32_t usbHubLastResetMs = 0;
uint32_t usbHubIgnoreCommandsUntilMs = 0;

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

void writeSerialMidi(Print &out, uint8_t status, uint8_t data1, uint8_t data2, uint8_t len) {
  if (len >= 1) out.write(status);
  if (len >= 2) out.write(data1);
  if (len >= 3) out.write(data2);
}

void sendMainBrainMidi(uint8_t status, uint8_t data1, uint8_t data2, uint8_t len) {
  writeSerialMidi(Serial1, status, data1, data2, len);
}

void holdUsbHubInReset() {
  digitalWrite(PIN_USB_HUB_RESET_N, LOW);
  pinMode(PIN_USB_HUB_RESET_N, OUTPUT);
}

void releaseUsbHubReset() {
  pinMode(PIN_USB_HUB_RESET_N, INPUT);
}

void setupCommandSerial() {
  Serial2.setTX(PIN_COMMAND_TX_TO_MAIN);
  Serial2.setRX(PIN_COMMAND_RX_FROM_MAIN);
  Serial2.begin(COMMAND_BAUD);
}

bool hubReleaseCommandSeen(uint8_t &matched) {
  while (Serial2.available() > 0) {
    const uint8_t b = static_cast<uint8_t>(Serial2.read());
    if (b == USB_HUB_RELEASE_COMMAND[matched]) {
      matched++;
      if (matched >= sizeof(USB_HUB_RELEASE_COMMAND)) {
        matched = 0;
        return true;
      }
    } else {
      matched = (b == USB_HUB_RELEASE_COMMAND[0]) ? 1 : 0;
    }
  }
  return false;
}

bool waitForHubReleaseCommand() {
  uint8_t matched = 0;
  const uint32_t startMs = millis();
  while (USB_HUB_RELEASE_COMMAND_TIMEOUT_MS == 0 ||
         (millis() - startMs) < USB_HUB_RELEASE_COMMAND_TIMEOUT_MS) {
    if (hubReleaseCommandSeen(matched)) return true;
    delay(1);
  }
  return false;
}

void sendHubReadyResponse() {
  const uint32_t startMs = millis();
  do {
    Serial2.write(USB_HUB_READY_RESPONSE, sizeof(USB_HUB_READY_RESPONSE));
    Serial2.flush();
    delay(5);
  } while ((millis() - startMs) < USB_HUB_READY_SIGNAL_MS);
}

void markUsbHubReleased() {
  usbHubLastResetMs = millis();
  usbHubIgnoreCommandsUntilMs = usbHubLastResetMs + USB_HUB_RESET_COMMAND_IGNORE_MS;
  delay(USB_HUB_RESET_RELEASE_SETTLE_MS);
  sendHubReadyResponse();
}

void releaseUsbHubForMain() {
  releaseUsbHubReset();
  markUsbHubReleased();
}

void pulseUsbHubReset() {
  const uint32_t now = millis();
  if (static_cast<int32_t>(now - usbHubIgnoreCommandsUntilMs) < 0) return;
  if (usbHubLastResetMs != 0 && (now - usbHubLastResetMs) < USB_HUB_RESET_COMMAND_IGNORE_MS) return;
  holdUsbHubInReset();
  delay(USB_HUB_RESET_PULSE_MS);
  releaseUsbHubForMain();
}

void serviceCommandSerial() {
  static uint8_t matched = 0;
  if (hubReleaseCommandSeen(matched)) pulseUsbHubReset();
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
  while (Serial1.available() > 0) {
    const uint8_t b = static_cast<uint8_t>(Serial1.read());
    externalMidi.write(b);
    processSerialMidiByte(mainBrainParser, b, sendUsbMidi);
  }
}

void pumpUsbToMainBrain() {
  uint8_t packet[4];
  while (usb_midi.readPacket(packet)) {
    const uint8_t len = midiPacketDataLength(packet[0]);
    for (uint8_t i = 0; i < len; ++i) {
      Serial1.write(packet[i + 1]);
    }
  }
}

void pumpExternalToMainBrain() {
  while (externalMidi.available() > 0) {
    processSerialMidiByte(externalParser, static_cast<uint8_t>(externalMidi.read()), sendMainBrainMidi);
  }
}

void setup() {
  holdUsbHubInReset();
  setupCommandSerial();
  waitForHubReleaseCommand();
  releaseUsbHubForMain();

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

  Serial1.setTX(PIN_MAIN_BRAIN_MIDI_TX);
  Serial1.setRX(PIN_MAIN_BRAIN_MIDI_RX);
  Serial1.begin(MIDI_BAUD);

  externalMidi.begin(MIDI_BAUD);
}

void loop() {
#ifdef TINYUSB_NEED_POLLING_TASK
  TinyUSBDevice.task();
#endif

  serviceCommandSerial();
  pumpUsbToMainBrain();
  pumpMainBrainToUsbAndExternal();
  pumpExternalToMainBrain();
}
