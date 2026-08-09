# ARPnMIDI

ARPnMIDI is a two-RP2040 MIDI router, arpeggiator, looper, performance controller, scale tool, and USB/DIN MIDI hub designed by Joseph Wozniak — [woz.lol](https://woz.lol).

The main brain runs the musical engine and user interface. The secondary brain handles external MIDI connections and optional MAX3421E USB-host MIDI. MIDI entering through the secondary is sent to the main brain for processing before being returned to the secondary for output.

## Current firmware

The current two-sketch firmware set is in [`arpnmidi os/`](<arpnmidi os/>):

- [`arpnmidi_main_brain.ino`](<arpnmidi os/arpnmidi_main_brain/arpnmidi_main_brain.ino>) — router, arpeggiator, looper, display, sensors, local controls, presets, and USB MIDI device.
- [`arpnmidi_max_secondary_brain.ino`](<arpnmidi os/arpnmidi_max_secondary_brain/arpnmidi_max_secondary_brain.ino>) — USB MIDI device, external DIN/TRS MIDI bridge, USB2514B hub control, and optional MAX3421E USB MIDI host.

Open each `.ino` from its matching Arduino sketch folder and flash it to the corresponding RP2040. Detailed two-board wiring and build information is in the [firmware-set README](<arpnmidi os/README.md>).

Archived experiments and previous firmware are kept in [`legacy/`](legacy/). They are not required to build the current system.

## Main features

- MIDI routing between USB, DIN/TRS, and hosted USB MIDI devices
- 16-channel remapping with per-channel transposition
- Arpeggiator with classic and fixed musical patterns
- Independent arp, thru, bass, and control-message output channels
- Round-robin distribution across selected MIDI channels
- Dedicated channel-10 drum lane and note-range remapping
- Note-only looper with fixed-bar and free-length recording
- Overdub, automatic overdub, playback, stop, replace, and delete controls
- Key and scale correction
- Piano and guitar note visualization
- Optical distance and pressure-sensor performance control
- CC-learn assignments for live parameters
- Flash-backed presets and saved loop storage
- Two currently implemented local remote buttons, with four local button GPIOs reserved in the hardware design
- Planned ESP connection for wireless MIDI, including compatibility with standard BLE MIDI controllers such as BLE MIDI foot pedals

## Main-brain hardware

The main brain uses a Waveshare RP2040 Zero, a 128×64 SSD1306 OLED, a rotary encoder, optional distance and pressure sensors, and local buttons.

### Main-brain pin map

Use the GPIO numbers printed on the RP2040 Zero:

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

GPIO11 is intentionally unused so a mounting screw can pass through that area.

Buttons 3 and 4 are part of the hardware direction but do not have firmware actions yet. Wireless MIDI will be added through an ESP connected to the main brain; standard BLE MIDI devices, including BLE MIDI foot pedals, can then be used as ordinary MIDI input devices.

## Arduino build setup

- Board package: Earle Philhower Arduino-Pico
- Board: Waveshare RP2040 Zero
- USB stack: Adafruit TinyUSB
- CPU speed: 120 MHz or 240 MHz

The main-brain sketch also uses:

- MIDI Library
- Adafruit GFX Library
- Adafruit SSD1306
- VL53L0X library

The secondary sketch carries its patched USB Host Shield 2.0 source inside its own `src/` directory.

## Runtime controls

- Turn the encoder in select mode to choose a setting screen.
- Click the encoder to switch between select and edit modes.
- Turn the encoder in edit mode to change the current value.
- Hold the encoder while turning for coarse changes where supported.
- Hold the encoder button for two seconds for panic and the recovery/reboot path.
- The first input while the screen saver is active wakes the display without also changing a parameter.

## Presets

- Sixteen flash-backed preset slots
- Parameter changes automatically save when leaving edit mode
- `LOAD` recalls a selected slot
- `SAVE` copies the current configuration to a selected slot
- MAP CC assignments are stored per preset
- Saved loop data is restored from flash

To perform a factory reset, hold the encoder button while powering on, release it when prompted, and press it again within five seconds to confirm.

## MIDI routing behavior

- Notes on `INPUT CH` feed the arp, thru, bass, scale, split, and loop processing paths.
- Notes on other channels pass through unless another routing feature claims that channel.
- `MONO RETRIG` converts one selected non-input channel to monophonic last-note-priority behavior.
- CC, Pitch Bend, Program Change, and Channel Aftertouch received on `INPUT CH` route to the `IN CC >` destination.
- Non-input-channel channel messages pass through unchanged.
- The sixteen-channel router can change output channel and transpose notes or poly-aftertouch note numbers.
- MIDI realtime messages are forwarded.
- Incoming MIDI clock does not currently set the internal arp BPM.

## Complete menu reference

### 1. `BPM`

- Range: `20–300`
- Sets the internal arp and fixed-bar loop tempo

### 2. `ARP MODE`

- `OFF`
- `UP`
- `DOWN`
- `UP-DOWN 1`
- `UP-DOWN 2`
- `TRIGGER`
- `RANDOM`
- `UP 1-OCT`
- `RHYTHM`
- `OSTINATO`
- `OCT WALK`
- `FIFTH`
- `BASS+CHORD`
- `CHORD+RUN`

### 3. `DIVISION`

`1/1`, `1/2`, `1/2T`, `1/4`, `1/4T`, `1/8`, `1/8T`, `1/16`, `1/16T`, `1/32`, `1/32T`, and `1/64`.

### 4. `VELOCITY`

- Range: `1–127`
- Sets arp output velocity
- Distance or pressure input can reduce it live with `VEL DOWN`

### 5. `LENGTH`

- Range: `1–100%` of the current step
- Distance or pressure input can reduce it live with `LEN DOWN`

### 6. `INPUT CH`

Selects the main processing input channel from `CH 1–16`.

### 7. `ARP CH`

- `OFF` or `CH 1–16`
- `1+10`: main arp plus a channel-10 drum arp lane
- `1+10-A`: channel aftertouch controls the drum-lane pulse velocity
- `1-10 24`, `1-10 36`, `1-10 48`: add an eight-note input split remapped to channel-10 notes 36–43

The channel-10 drum lane continues pulsing when the main arp mode is off.

### 8. `BASS CH`

- `OFF` or channel selections for `CH 1–12`
- Per-channel octave offsets: `-2`, `-1`, `0`, or `+1`
- Lowest-note-priority monophonic bass voice

### 9. `THRU OUT`

Selects an independent thru destination from `OFF` or `CH 1–16`.

### 10. `RNDRBN`

- Sends successive arp notes across selected MIDI channels
- Allows multiple monophonic instruments to act as a distributed polyphonic instrument
- Includes the `CH10-1+` and `CH10-2+` channel-10 remapping modes

### 11. `ROUTER`

- One mapping row per input channel
- Selectable output channel
- Note and poly-aftertouch transposition from `-24` to `+24` semitones
- `CLEAR` restores identity routing

### 12. `DIV NOTE`

- Learn a note and channel for each division from `1/4` through `1/64`
- Holding a learned note temporarily changes the arp division
- The learned trigger can be swallowed or replaced with a configured `+NOTE`

### 13. `MAP CC`

Learn incoming CC controls for BPM, arp mode, division, velocity, length, channel settings, sensor modes, key, scale, loop length, and preset loading. Mappings can require the learned MIDI channel or accept the CC from all channels.

### 14. `IN CC >`

Routes input-channel CC, Pitch Bend, Program Change, and Channel Aftertouch to one channel or to the de-duplicated arp/thru/bass destinations with `ALL3`.

### 15. `MONO RETRIG`

- `OFF` or `CH 1–16`
- Converts the selected non-input channel to monophonic last-note priority
- Pressing a newer note replaces the active note
- Releasing it recalls the newest note still held

### 16. `REMOTE`

Selects the output channel used by the local `REMOTE 1` and `REMOTE 2` actions.

### 17. `REMOTE 1`

Configures local button 1 to send a short Note or CC pulse.

### 18. `REMOTE 2`

Configures local button 2 to send a short Note or CC pulse.

### 19. `SCRNSVR`

- `OFF`
- `AUTO`
- `NOW`

### 20. `EYE/PUSH`

Selects the shared output channel used by the distance and pressure controllers.

### 21. `EYE MODE`

Distance-sensor modes include:

- Division changes: `DIV +2`, `DIV -2`, `DIV +3`, `DIV -3`, `DIV FULL`, `DIV3`
- `VEL DOWN` and `LEN DOWN`
- `ARP LATCH`, `ARP LATCH+`, `ARP FREEZE`, `ARP FREEZ+`
- `PITCH UP` and `PITCH DOWN`
- Quantized note ranges `NOTES C0–C7`
- `CC 1–19` and `CC103`
- Loop record/play/overdub and stop/delete actions

### 22. `PUSH`

Provides the same behavior families as `EYE MODE`, controlled by the pressure sensor.

### 23. `LOOP`

- Lengths: `1 BAR`, `2 BAR`, `4 BAR`, `8 BAR`, or `FREE`
- Fixed lengths begin playback automatically at the loop boundary
- Free recording continues until stopped
- Supports playback, overdub, automatic overdub, replace, stop, and delete
- The active loop length can be changed while performing

### 24. `KEY`

- `OFF` or roots `C–B`
- Standard mode moves off-key notes upward to the next scale note
- `CKEY C–B` maps white keys across the selected scale

### 25. `SCALE`

`OFF`, `MAJOR`, `MINOR`, `MAJ+MIN`, `BLUES`, `MAJ BLUES`, `BLUES+BOTH`, `HARM MIN`, and `MEL MIN`.

### 26. `GIT/KEYS`

Selects live guitar-fretboard or piano-key note visualization.

### 27. `LOAD`

Loads one of sixteen preset slots when leaving edit mode.

### 28. `SAVE`

Writes the current configuration to one of sixteen preset slots when leaving edit mode.

### 29. `PANIC`

Shows MIDI and USB diagnostics including device counts, queue depth, drop counters, overload warning, last received note, and receive timing.
