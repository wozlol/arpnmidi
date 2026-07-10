# MAX3421E Secondary Host Version

Moves USB host MIDI to the secondary brain with a MAX3421E upstream of the
USB2514B hub. The MAX3421E owns USB host MIDI over SPI0; the secondary routes it
to the main brain over serial and fans the main brain's serial MIDI back out.

- `max_main_brain/max_main_brain.ino` — main ARPnMIDI firmware; RP2040 PIO-USB
  host disabled; also a USB MIDI device to a computer; serial link to secondary on GP4/GP5.
- `max_secondary_brain/max_secondary_brain.ino` — secondary bridge: USB device
  MIDI + MAX3421E USB host MIDI.

## Status — NOT yet tested in situ (2026-07-10)

- USB-host MIDI protections ported from `max3421e_debug` and in place in
  `max_secondary_brain`:
  - **hasOut() guard** on `sendMaxHostMidi()` (don't send to endpoint 0 → host wedge).
  - **SysEx dropped** on the host→main-brain path.
  - **Non-blocking Serial2 write** (`availableForWrite()` guard) so a pad
    aftertouch burst can't block the UART and starve the host Task. Serial2's
    31250 baud is itself the cable-rate throttle, so aftertouch is preserved.
- **TODO:** also port the note-safe overflow protection and the recovery watchdog
  from the debug sketch.
- **Blocked on hardware:** the PCB's onboard RP2040 does not yet talk SPI to the
  MAX over the patched traces, so this can only be validated once that SPI link
  works. For now only `max3421e_debug` (RP module + MAX module on the PCB's
  working hub) is testable.

## Secondary-brain pins (RP2040-Zero, ARPnMIDI PCB)

| Pin  | Function                                             |
|------|------------------------------------------------------|
| GP0  | MAX3421E MISO (SPI0 RX, via SPI.setRX(0))            |
| GP1  | MAX3421E CS (software CS)                             |
| GP2  | MAX3421E SCK (SPI0)                                   |
| GP3  | MAX3421E MOSI (SPI0 TX)                               |
| GP4  | serial MIDI TX to main brain (Serial2 / UART1)       |
| GP5  | serial MIDI RX from main brain (Serial2 / UART1)     |
| GP23 | external serial MIDI TX (SerialPIO)                  |
| GP25 | external serial MIDI RX — TRS MIDI IN from opto      |
| GP26 | MAX3421E INT                                          |
| GP24 | USB2514B RESET_N (active low) — currently GP22 in code for testing |
| RST  | MAX3421E RST → 3.3V (no on-module pullup)            |

Routing: every secondary-side MIDI input goes only to GP4 (into the main brain);
GP5 from the main brain fans out to USB device MIDI, external serial TX, and all
MAX-hosted USB MIDI outs. No secondary input is routed directly to another
secondary output.

## Library patches (see ../patches/)

- `USB_Host_Shield_2.0_RP2040_pins.patch` — UsbCore.h `MAX3421e<P1, P26>` (CS GP1, INT GP26).
- `USB_Host_Shield_2.0_OutTransfer_timeout.patch` — bounds two unguarded
  completion-wait spins in `usb.cpp OutTransfer()` that hard-freeze the RP2040
  under bus stress. Required for durability under load.
- Older combined notes also captured in `../uhs2_rp2040_compat.patch`.

macOS note: don't `#include <Usb.h>` directly — Arduino-Pico also ships a `USB.h`
and the filesystem may resolve the wrong file. Include `usbhub.h` / `usbh_midi.h`,
which pull in the correct library-local `Usb.h`.

## Build

- Board: RP2040 / RP2040-Zero (Earle Philhower arduino-pico core)
- USB Stack: Adafruit TinyUSB
- Main-brain + external MIDI at 31250 baud.
