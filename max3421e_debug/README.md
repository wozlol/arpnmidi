# max3421e_debug

Isolated bench test for the MAX3421E USB host on an RP2040-Zero. Routes MAX3421E
host MIDI directly to/from the RP2040's own USB device MIDI port — no main brain,
no serial links — so the USB host path can be validated on its own.

## Status — WORKING (2026-07-10)

- Enumerates multiple USB MIDI devices through a hub and routes host↔host.
- Host→host forwarding is hardened:
  - **hasOut() guard** — only sends to devices with a real OUT endpoint (sending
    to endpoint 0 wedged the host).
  - **31250-baud throttle** — per-device FIFO paced to a MIDI cable's rate, so a
    held pad's aftertouch flood no longer crashes the receiver (e.g. KO II).
    Aftertouch/pressure is preserved, just smoothed.
  - **Note-safe overflow** — never drops note-on/off (no stuck notes); sheds
    aftertouch/CC first under load.
  - **Recovery watchdog** — re-inits a wedged host automatically (`recov=` in the
    diagnostic).
  - **SysEx dropped** for now (not needed yet; raw forwarding truncates it).
- `maxHostReady` gates all host code — the sketch runs fine as a plain USB MIDI
  device if no MAX answers.

## Pins (RP2040-Zero, ARPnMIDI PCB)

| Pin  | Function                                   |
|------|--------------------------------------------|
| GP0  | MAX3421E MISO (SPI0 RX, via SPI.setRX(0))  |
| GP1  | MAX3421E CS (software CS)                   |
| GP2  | MAX3421E SCK (SPI0)                         |
| GP3  | MAX3421E MOSI (SPI0 TX)                     |
| GP26 | MAX3421E INT                               |
| RST  | tie to 3.3V (no on-module pullup)          |

- USB device port = the RP2040-Zero's own USB-C (to the computer).
- USB host port = the MAX3421E's USB-A (plug controllers/hub here).

## Library — vendored, self-contained (no global-library dependency)

The patched USB Host Shield 2.0 is bundled in `src/USB_Host_Shield_Library_2.0/`
and included with quotes, so this sketch compiles from its own copy and is immune
to global Arduino-library updates. Verified: it builds (exit 0) with the global
library moved aside. See `../patches/README.md` for what was changed vs. stock
(RP2040 pins, RP2040 compat layer, and the critical `OutTransfer` freeze fix).

## Build

- Board: Waveshare RP2040-Zero (Earle Philhower arduino-pico core)
- USB Stack: Adafruit TinyUSB
- Serial monitor: 115200 baud
