# patches/ — library change record

**The patched library is now VENDORED** into each MAX3421E sketch's `src/` folder
(`<sketch>/src/USB_Host_Shield_Library_2.0/`). That vendored copy is the live,
self-contained source of truth — the sketches include it with quote-includes
(`#include "src/USB_Host_Shield_Library_2.0/usbhub.h"`), so they compile from
their own bundled library and are immune to global Arduino-library updates.

Verified: each sketch compiles with the **global** `USB_Host_Shield_Library_2.0`
moved aside (exit 0), proving the vendored copy is complete and standalone.

The `.patch` / `.txt` files here are the **record** of what was changed vs. the
stock USB Host Shield Library 2.0 — for reference, review, and re-deriving the
edits if the library is ever re-vendored from a newer upstream:

- `USB_Host_Shield_2.0_RP2040_pins.patch` — `UsbCore.h` RP2040 entry
  `typedef MAX3421e<P1, P26>` (CS = GP1, INT = GP26). Without it the library
  falls back to Uno defaults (CS pin 10, INT pin 9).
- `USB_Host_Shield_2.0_OutTransfer_timeout.patch` — bounds two unguarded
  transfer-completion spins in `usb.cpp OutTransfer()` with a 50 ms deadline.
  **Critical:** without it the RP2040 hard-freezes (no serial, no recovery) when
  a host OUT transfer hangs under bus stress (e.g. a held pad's aftertouch flood).
- `uhs2_rp2040_compat.patch` (repo root) — the RP2040 compatibility layer:
  `avrpins.h` pin classes for GP0..GP29 and the `USB` macro cleanup. Required for
  RP2040 support at all (stock UHS 2.0 has none).
- `arduino-rp2040-5.6.1-tinyusb-midi-*.patch` — earlier core/TinyUSB notes.

## Re-vendoring from a fresh library (if ever needed)

1. Apply the changes above to a clean USB Host Shield Library 2.0.
2. Copy it into each sketch's `src/` (excluding `examples/`, `doc/`, `*.orig`).
3. Confirm the sketch still uses quote-includes to `src/...`.
4. Verify: temporarily move the global lib aside and `arduino-cli compile` —
   it must build with exit 0 from the vendored copy alone.
