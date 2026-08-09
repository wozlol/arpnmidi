# Module RP2040 Host Version

This pair keeps the main brain as the RP2040 PIO-USB host on GPIO14/GPIO15.

- `module_main_brain/module_main_brain.ino`
  - Main ARPnMIDI firmware.
  - SMD/DIP panel config is present.
  - RP2040 PIO-USB host remains enabled on the main brain.
  - Later PCB USB2514B reset-handshake and stability-test switches are disabled for this module baseline.

- `module_secondary_brain/module_secondary_brain.ino`
  - USB device plus serial MIDI bridge.
  - Main-brain serial MIDI link remains on GP0/GP1.
  - External serial MIDI remains on GP2/GP3.
  - Uses a short hub-release timeout so it does not block if the module main brain does not send the later reset command.
