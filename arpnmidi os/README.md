# ARPnMIDI OS firmware set

This folder contains the two Arduino sketches used by the current two-RP2040 ARPnMIDI system.

## Firmware folders

### Main brain

[`arpnmidi_main_brain/arpnmidi_main_brain.ino`](arpnmidi_main_brain/arpnmidi_main_brain.ino)

The main brain owns:

- MIDI routing and channel transformation
- Arpeggiator and musical timing
- Note looper and saved loop data
- Scale correction and note visualization
- SSD1306 display and rotary encoder interface
- Distance and pressure sensors
- Local button actions
- Sixteen flash-backed presets
- USB MIDI device connection

MIDI from external DIN/TRS and optional hosted USB devices arrives over the GPIO4/5 serial link from the secondary brain.

### Secondary brain

[`arpnmidi_max_secondary_brain/arpnmidi_max_secondary_brain.ino`](arpnmidi_max_secondary_brain/arpnmidi_max_secondary_brain.ino)

The secondary brain owns:

- USB MIDI device input and output
- External DIN/TRS MIDI input and output
- Serial MIDI transport to and from the main brain
- USB2514B hub reset control
- Optional MAX3421E USB MIDI host

## MIDI data flow

All secondary-side inputs are sent to the main brain before routing:

- USB MIDI device input goes through the secondary and GPIO4 to the main brain.
- DIN/TRS MIDI input goes through the secondary and GPIO4 to the main brain.
- MAX3421E USB-host input goes through the secondary and GPIO4 to the main brain.
- Main-brain output returns through GPIO5 to the secondary.
- The secondary sends returned MIDI to USB MIDI device output, DIN/TRS MIDI output, and available MAX3421E-hosted USB MIDI outputs.

The secondary does not route an input directly to another secondary-side output. This keeps the main brain in control of channel routing, transformations, loop capture, and note ownership.

## Arduino setup

Use these settings for both sketches:

- Board package: Earle Philhower Arduino-Pico
- Board: Waveshare RP2040 Zero
- USB stack: Adafruit TinyUSB
- CPU speed: 120 MHz or 240 MHz

Open and flash each sketch separately:

1. Flash `arpnmidi_main_brain.ino` to the main-brain RP2040.
2. Flash `arpnmidi_max_secondary_brain.ino` to the secondary RP2040.

The folder and `.ino` names already match, so both sketches open directly in the Arduino IDE.

## Main-brain connections

- GPIO0: serial TX to ESP32-C3
- GPIO1: serial RX from ESP32-C3
- GPIO2: I2C SDA for SSD1306 and VL53L0X
- GPIO3: I2C SCL for SSD1306 and VL53L0X
- GPIO4: serial MIDI TX to secondary brain
- GPIO5: serial MIDI RX from secondary brain
- GPIO6: rotary encoder A
- GPIO7: rotary encoder B
- GPIO8: rotary encoder push switch
- GPIO9: local button 1; currently `REMOTE 1`
- GPIO12: local button 2; currently `REMOTE 2`
- GPIO10: reserved for local button 3
- GPIO13: reserved for local button 4
- GPIO16: optional RGB LED output; disabled by default
- GPIO26: push/pressure analog sensor input

GPIO11 is intentionally unused for mechanical clearance.

The current firmware implements local buttons 1 and 2. Actions for buttons 3 and 4 still need to be added.

## Wireless MIDI direction

A future ESP connection to the main brain will provide wireless MIDI. It is intended to accept standard BLE MIDI devices, which may include keyboards, controllers, or BLE MIDI foot pedals. There is no dedicated wired foot-pedal subsystem in the current ARPnMIDI firmware.

## Secondary-brain pin profiles

The secondary sketch contains two compile-time profiles selected by `ARPNMIDI_SECONDARY_PIN_PROFILE`.

### `ARPNMIDI_SECONDARY_PIN_PROFILE_OLD_NO_MAX`

This is the current default profile. It disables the MAX3421E host and uses:

- GPIO2: external serial MIDI TX
- GPIO3: external serial MIDI RX
- GPIO4: serial MIDI TX to main brain
- GPIO5: serial MIDI RX from main brain
- GPIO24: USB2514B `RESET_N`

### `ARPNMIDI_SECONDARY_PIN_PROFILE_NEW_MAX_PCB`

This profile enables the MAX3421E USB host and uses:

- GPIO0: MAX3421E SPI0 MISO
- GPIO1: MAX3421E chip select
- GPIO2: MAX3421E SPI0 SCK
- GPIO3: MAX3421E SPI0 MOSI
- GPIO4: serial MIDI TX to main brain
- GPIO5: serial MIDI RX from main brain
- GPIO20: external serial MIDI TX
- GPIO21: external serial MIDI RX
- GPIO24: USB2514B `RESET_N`
- GPIO26: MAX3421E interrupt

The optional MAX3421E reset hardware line is on GPIO22 and is not driven by the current firmware.

## Secondary routing details

- Main-brain and external serial MIDI run at 31,250 baud.
- USB-device MIDI input is sent only to the main brain.
- External DIN/TRS MIDI input is sent only to the main brain.
- MAX3421E-hosted USB MIDI input is sent only to the main brain.
- MIDI returned by the main brain fans out to USB-device output, DIN/TRS output, and available MAX3421E-hosted USB MIDI outputs.
- The MAX3421E output path checks that a hosted device actually has a MIDI OUT endpoint before transmitting.
- The secondary uses non-blocking serial writes so dense continuous-controller traffic cannot indefinitely stall USB-host servicing.
- Full streaming SysEx parsing is not implemented for every serial path.

## Libraries

The main-brain sketch requires the normal installed Arduino libraries listed in the project-level [README](../README.md#arduino-build-setup).

The secondary sketch is self-contained for MAX3421E support. Its patched USB Host Shield 2.0 source is stored at:

[`arpnmidi_max_secondary_brain/src/USB_Host_Shield_Library_2.0/`](arpnmidi_max_secondary_brain/src/USB_Host_Shield_Library_2.0/)

The sketch includes this copy with quoted includes. Do not replace those includes with a global USB Host Shield installation. Arduino-Pico also provides a `USB.h`, so directly including `<Usb.h>` can select the wrong library on case-insensitive filesystems.

Patch notes for the bundled library are in [`../patches/README.md`](../patches/README.md).

## Current implementation status

- Both Arduino sketches compile for Waveshare RP2040 Zero with Adafruit TinyUSB.
- The no-MAX secondary profile provides the current external serial MIDI bridge configuration.
- The MAX3421E PCB profile is present in firmware but still requires validation on the target PCB hardware.
- Local buttons 3 and 4 are reserved in the hardware map but do not have firmware actions yet.
- ESP-based wireless MIDI is planned and is not implemented yet.
