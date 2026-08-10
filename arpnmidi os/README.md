# ARPnMIDI OS firmware set

This folder contains the two matching Arduino sketches for the current
dual-RP2040 ARPnMIDI prototype.

## Firmware folders

### Main brain

[`arpnmidi_main_brain/arpnmidi_main_brain.ino`](arpnmidi_main_brain/arpnmidi_main_brain.ino)

The main brain owns:

- MIDI input parsing, routing, and transformations
- Musical clock, swing, arpeggiator, and Drum Magic
- Four-track looper, Time Travel, Stutter, Echo, and Note Length scheduling
- Note ownership and safe note-off handling
- Encoder, four physical buttons, and pressure sensor
- OLED and VL53L0X user interface
- Sixteen presets and global loop storage
- Native USB MIDI device input and output

### Secondary brain

[`arpnmidi_max_secondary_brain/arpnmidi_max_secondary_brain.ino`](arpnmidi_max_secondary_brain/arpnmidi_max_secondary_brain.ino)

The secondary brain owns:

- External DIN/TRS MIDI input and output
- A second native USB MIDI device input and output
- The 1 Mbps UART connection to the main brain
- USB2514B hub reset control
- Optional MAX3421E USB MIDI host and output fan-out

Flash both sketches from the same revision. Both ends of the GPIO4/5 link must
use the current 1 Mbps inter-brain setting.

## MIDI data flow

Every secondary-side input goes to the main brain before it reaches an output:

- USB device input goes to the main brain.
- External DIN/TRS input goes to the main brain.
- MAX3421E-hosted USB MIDI input goes to the main brain.
- Processed main-brain output returns to the secondary.
- The secondary fans returned output to its USB device, external DIN/TRS, and
  hosted USB MIDI outputs that provide a MIDI OUT endpoint.

There is no direct secondary input-to-output thru path. This keeps routing,
recording, transformations, and note ownership centralized in the main brain.

## Serial speeds

- Main-to-secondary UART: 1,000,000 baud
- Secondary-to-main UART: 1,000,000 baud
- Physical external DIN/TRS MIDI: 31,250 baud

The inter-brain path drains bounded batches and uses bounded queues so dense CC
traffic cannot monopolize either musical core. The main queue reserves room for
critical note offs.

## Main-brain prototype connections

- GPIO0: serial TX to the planned ESP32-C3
- GPIO1: serial RX from the planned ESP32-C3
- GPIO2: shared I2C SDA for SSD1306 and VL53L0X
- GPIO3: shared I2C SCL for SSD1306 and VL53L0X
- GPIO4: 1 Mbps UART TX to secondary GPIO5
- GPIO5: 1 Mbps UART RX from secondary GPIO4
- GPIO6: rotary encoder A
- GPIO7: rotary encoder B
- GPIO8: rotary encoder push switch
- GPIO9: physical button 1
- GPIO12: physical button 2
- GPIO10: physical button 3
- GPIO13: physical button 4
- GPIO16: optional RGB status LED, disabled by default
- GPIO26: pressure sensor analog input

GPIO11 is intentionally unused for mechanical clearance. The four buttons are
implemented in the order GPIO9, GPIO12, GPIO10, GPIO13. The firmware uses
active-high inputs with internal pulldowns.

GPIO0 and GPIO1 are reserved for a normal two-wire UART connection to the
planned ESP32-C3 wireless-MIDI processor. Standard BLE MIDI controllers,
including BLE MIDI foot controllers, can be supported through that processor.
There is no dedicated wired foot-pedal subsystem.

## Main-brain core allocation

Core 0 handles the timing-sensitive path:

- Encoder, buttons, and pressure input
- Inter-brain and native USB MIDI input
- Clock, arp, drums, routing, four-track looper, and live effects
- Deferred persistence only when the musical engine is idle

Core 1 handles slower peripheral work:

- OLED redraws from current state
- VL53L0X polling
- Outgoing inter-brain UART queue

The display is marked dirty only when visible state changes. OLED traffic does
not run continuously in the main musical loop.

## Secondary-brain core allocation

Core 0 handles:

- Native USB MIDI device traffic
- Physical DIN/TRS MIDI
- Inter-brain UART
- Bounded queues to and from the host core

Core 1 handles:

- MAX3421E host servicing when the MAX profile is selected
- Hosted-device MIDI input and output

## Secondary pin profiles

Select the profile with `ARPNMIDI_SECONDARY_PIN_PROFILE` near the top of the
secondary sketch.

### `ARPNMIDI_SECONDARY_PIN_PROFILE_OLD_NO_MAX`

This is the default for the current prototype:

- GPIO2: external serial MIDI TX
- GPIO3: external serial MIDI RX
- GPIO4: 1 Mbps UART TX to main GPIO5
- GPIO5: 1 Mbps UART RX from main GPIO4
- GPIO24: USB2514B `RESET_N`
- MAX3421E host disabled

### `ARPNMIDI_SECONDARY_PIN_PROFILE_NEW_MAX_PCB`

This profile builds the MAX3421E path:

- GPIO0: MAX3421E SPI0 MISO
- GPIO1: MAX3421E chip select
- GPIO2: MAX3421E SPI0 SCK
- GPIO3: MAX3421E SPI0 MOSI
- GPIO4: 1 Mbps UART TX to main GPIO5
- GPIO5: 1 Mbps UART RX from main GPIO4
- GPIO20: external serial MIDI TX
- GPIO21: external serial MIDI RX
- GPIO24: USB2514B `RESET_N`
- GPIO26: MAX3421E interrupt

The optional external MAX3421E hardware-reset line is not driven by this
firmware profile.

## Arduino setup

Use for both sketches:

- Earle Philhower Arduino-Pico board package
- Board: Waveshare RP2040 Zero
- USB stack: Adafruit TinyUSB
- CPU speed: 120 MHz or 240 MHz

The main brain must reserve LittleFS space. Use the 2 MB flash layout with
512 KB filesystem:

```sh
arduino-cli compile --warnings all \
  -b rp2040:rp2040:waveshare_rp2040_zero \
  --board-options freq=120,usbstack=tinyusb,flash=2097152_524288 \
  arpnmidi_main_brain
```

Build the secondary with:

```sh
arduino-cli compile --warnings all \
  -b rp2040:rp2040:waveshare_rp2040_zero \
  --board-options freq=120,usbstack=tinyusb \
  arpnmidi_max_secondary_brain
```

Open and flash each `.ino` from its matching sketch folder.

## Libraries and bundled host code

The main sketch uses:

- MIDI Library
- Adafruit GFX Library
- Adafruit SSD1306
- VL53L0X library

The secondary carries its patched USB Host Shield 2.0 source at
[`arpnmidi_max_secondary_brain/src/USB_Host_Shield_Library_2.0/`](arpnmidi_max_secondary_brain/src/USB_Host_Shield_Library_2.0/).

Keep the quoted local includes. Arduino-Pico also supplies a file named
`USB.h`, so replacing the bundled includes with a global `<Usb.h>` include can
select the wrong library on a case-insensitive filesystem.

Patch notes are in [`../patches/README.md`](../patches/README.md).

## Storage behavior

- The main brain uses flash-backed EEPROM for compact preset data.
- LittleFS stores extended preset records and the four global loop tracks.
- Use the required filesystem flash layout or extended settings and loops
  cannot persist.
- Automatic writes wait for a musically idle window.
- Time Travel and Stutter rolling history remain RAM-only.
- A preset-schema mismatch installs factory defaults; no prototype migration
  path is run.

## Current hardware status

- Both sketches compile without warnings at 120 MHz.
- The default no-MAX secondary profile is the current prototype configuration.
- The MAX3421E profile compiles and includes endpoint guards, but still needs
  validation on its target PCB.
- Full streaming SysEx parsing is not implemented on every serial path.
- ESP32-C3 wireless MIDI is planned and is not implemented here yet.

The planned dual-RP2354A and RP2350-Zero module pin assignment is documented in
[`../max3421e_pins_for_next_pcb.txt`](../max3421e_pins_for_next_pcb.txt). It is
not the pin map for this RP2040 prototype.
