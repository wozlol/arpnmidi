# ARPnMIDI

ARPnMIDI is a two-processor MIDI performance instrument designed by Joseph
Wozniak — [woz.lol](https://woz.lol). It combines MIDI routing, arpeggiation,
four-track looping, scale and chord processing, live effects, controller
mapping, USB MIDI, DIN/TRS MIDI, and an optional USB MIDI host.

The current prototype uses two Waveshare RP2040 Zero boards. The firmware is
also organized for the planned dual-RP2354A hardware.

## Current firmware

The two matching Arduino sketches are in [`arpnmidi os/`](<arpnmidi os/>):

- [`arpnmidi_main_brain.ino`](<arpnmidi os/arpnmidi_main_brain/arpnmidi_main_brain.ino>)
  runs the musical engines, routing, controls, display, presets, loop storage,
  sensors, and a native USB MIDI device.
- [`arpnmidi_max_secondary_brain.ino`](<arpnmidi os/arpnmidi_max_secondary_brain/arpnmidi_max_secondary_brain.ino>)
  runs the external DIN/TRS MIDI bridge, a second native USB MIDI device, and
  the optional MAX3421E USB MIDI host.

Both boards must run firmware from the same revision. Their direct GPIO4/5
UART link runs at 1 Mbps; it is intentionally much faster than physical MIDI.

Detailed wiring and two-board build information is in the
[firmware-set README](<arpnmidi os/README.md>).

## Musical features

### Arpeggiator

- Up, Down, Up-Down 1, Up-Down 2, Trigger, Random, and Off modes
- Factory patterns: Up 1-Octave, Rhythm, Ostinato, Octave Walk, Fifth,
  Bass+Chord, and Chord+Run
- Custom pattern recording with note timing, gate length, velocity, and pitch
  offsets measured from the lowest note played in the take
- Custom lengths of 1/4 bar, 1/2 bar, 1, 2, 4, or 8 bars
- Up to 32 note events in each custom pattern
- One through four octaves
- In-order or as-played note ordering
- Key-press or clock-synchronized retriggering
- Straight, triplet, and dotted divisions through 1/64T
- Arp Division can follow Drum Division live
- Arp velocity and gate length are stored per preset

### Drum Magic and Drum Roll

- Independent drum engine with its own output channel and division
- Input from MIDI channel 10 or an eight-note split on Main Input
- Configurable split start and mapped drum-note start
- Optional channel-aftertouch-to-drum-velocity control
- Drum Division can follow Arp Division or run Free
- Drum Roll learns either notes or CCs for temporary drum divisions
- Generated drum-roll notes are recorded by the looper
- Mutual Arp/Drum follow settings cannot form a circular dependency

### Four-track looper

- Four tracks sharing a fixed 3,072-event pool
- Per-track lengths from 1/4 bar through 8 bars
- Track 1 can also establish a Free loop length and make it the musical bar
- Layers, Parts Auto Solo, and Manual track modes
- Overdub, replace, Auto Arm, mute, exclusive solo, safe clear, and undo
- Recovering audibility, an unmute, losing an exclusive solo, or an undo,
  retriggers whatever the track's own data would already have sounding at that
  instant, not just whatever note-on comes next
- When all Layers tracks are occupied, the next overdub goes onto the oldest
  layer
- Optional auto quantize, set per track: Off, or any straight, dotted, or
  triplet division from 1/4 down through 1/64T
- Optional CC recording with bounded smoothing and pruning
- Optional MIDI transport response, enabled by default
- MIDI Start, Continue, Stop, Song Select, and useful MMC transport commands
- Time Travel can turn the immediately preceding performance into a loop
- All four loop tracks survive reboot and are shared across presets
- Time Travel and Stutter history remain in RAM and do not survive reboot

Tracks keep their own place in the shared cycle:

- A layer may begin wherever the first note landed, so its loop boundary can sit
  anywhere inside the others
- Stop, clear all, undo, and play again restore every track to that same
  relationship, so the loop comes back in sync rather than restarting every
  track at its own beginning
- The stored phase survives reboot with the rest of the loop file

Recording always follows one working track:

- A cleared track counts as free space. Recording it takes a fresh replacement
  take rather than layering onto material that cannot be heard
- Arming is not destructive. A cleared track keeps its undo material until the
  first captured event replaces it
- An armed record that has not started yet follows the working track, so the
  track shown on screen is always the track about to be written
- A pass that is already recording keeps its track until it is stopped
- Only a layer that captured something advances the working track
- In Layers mode the global Clear gesture works on all four tracks at once and
  only undoes once nothing audible is left to clear, and returns the working
  track to Track 1
- The Loop Mix screen strikes through cleared tracks and dots the record target,
  and the looper summary shows cleared content as a hollow marker
- A loop status icon in the top right shows on every screen: filled circle for
  actively recording or overdubbing, open circle for armed and waiting on a
  first note, triangle for playing, two short bars for stopped with data
  still in it
- Auto Arm fires in exactly one place: a fixed-length pass concluding on its
  own because it reached its length. Nothing else ever triggers it, picking a
  different track by hand included. In Manual mode, where the working track
  never advances on its own, this continues straight into an overdub of the
  same track. In Layers and Parts Auto Solo, it arms the track the existing
  auto-advance just moved to, waiting for the first note with no button press

Changing a populated track's length preserves the instrument's special repeat
behavior:

- Lengthening duplicates the complete stored recording, including notes, note
  offs, and recorded CCs, until the longer track is filled.
- Shortening changes only the audible loop window. Material beyond that window
  remains stored.
- Lengthening again restores the retained material without creating duplicate
  copies.
- A true replacement recording or permanent overwrite discards the retained
  copies and establishes new source material.
- A resize is rejected cleanly if the shared event pool cannot hold the needed
  copies.

Ways to reach the looper without leaving the screen you are on:

- Hold the encoder and turn on either the LOOPER or LOOP MIX screen to change
  the working track
- Loop Mix applies one mode to whichever track is picked: Solo, Mute, Clear, or
  Arm, and opens on Arm. Clear also undoes a track that was cleared and not recorded over. Arm
  selects and arms the picked track, takes the arm back off if it is already the
  target, and starts a stopped or paused transport
- Clicking Solo or Mute while that mode is already in force resets the whole
  mix, unmuting and unsoloing all four tracks
- One step past Arm the back arrows appear without a box, and clicking there
  leaves the screen

The master rec/play trigger takes a double tap from any source, the push, the
sensor, a mapped CC, or a button:

- A double tap safe clears a track and arms it in the same motion, in every
  track mode, so the part can be played again with nothing further to press
- In Layers, that track steps back to the layer just recorded first, when the
  working track is neither cleared nor holding content, because that is the
  one a performer means
- In Manual and Parts Auto Solo, it always clears and arms whichever track is
  currently selected, since choosing tracks there belongs to the performer
- A single trigger before anything is played takes a pending arm back off
- Another double tap brings a cleared track back and plays it, armed or not,
  staying on that same working track since undo restores content, it never
  counts as a capture

Holding the encoder switch into a panic, or a quick tap on the PANIC screen,
is a deliberate emergency reset on top of the usual all-notes-off: it also
clears whatever is still live in the loop (undoable, never restores an
already-cleared track), and puts every mapped Feature Knob's value back to
whatever the current preset actually has saved, so a connected controller
sitting at a stale position can't leave tempo, swing, or any other mapped
parameter stuck somewhere the preset never asked for. A mapped Feature
Button panic and the panic a preset load fires on its way out stay
silence-only.

### Live transformations

Velocity, Notelength, Stutter, and Echo can target Main or any of the four
looper tracks independently. Stutter and Echo also have a SELECTD target,
its own independent settings that dynamically follow whichever loop track is
currently selected, so one stutter or echo setup can travel with whatever
track the performer is actively working rather than needing reconfiguring
per track. Their submenus lead with the length/repeat-size parameter and
end with the target picker.

- Velocity scales from 0 to 200 percent.
- Notelength scales from 1 to 200 percent.
- Stutter uses the shared rolling history for note-safe beat repeats. It
  defaults to `1/8`, tops out at `1 BAR`, and has a configurable timeout in
  bars.
- Echo provides Wet, Length, Delay division, and signed Drift. Drift accelerates
  or decelerates the repeats for a bouncing-object effect. Echo Length and
  Delay reach the looper's `8 BARS` maximum, longer than Stutter's, since a
  fixed-length repeat spanning several bars stays useful in a way
  stutter-repeating that much rarely is.
- Stutter and Echo-generated repeats are kept out of the recorded source loop.
- A live stutter warning, a thin border around the whole screen, shows on
  every screen whenever any target actually has a stutter repeat running,
  and clears the instant it stops, whether that's the timeout, the submenu's
  own toggle, a mapped knob returning to zero, or a momentary button's
  release.

### Routing and note processing

- Main Input, Arp Out, Thru output, and lowest-note-priority Bass output
- Bass highest-note split
- Quick Jump from a chosen input channel to a chosen output channel, with a
  Hold option for preserving held notes across channel changes
- Quick Jump's output can also be moved live, one CC Button per MIDI channel:
  pressing one points the output there and turns Quick Jump on, or, if it's
  already on and already there, turns it back off without losing that output
  channel. A live move only, never saved to the preset.
- Sixteen-channel router with output channel, low note, high note, and
  transposition for each channel
- Notes outside a router row's selected range continue on their original
  channel
- Round Robin across selected channels, with cycle or true-random selection
- Optional channel-10 remapping in Round Robin
- Mono Retrig last-note-priority processing on a selected channel
- Key and scale correction, including an editable twelve-note User Scale
- Four-position Chord generator using chromatic offsets or scale degrees
- Four post-processing chord memories on the physical buttons
- Per-note parameter locks for CC values on a selected channel
- Channel and poly aftertouch forwarding, aftertouch-to-CC, and Main
  aftertouch-to-arp-velocity options

### MIDI control and mapping

- Features menu divided into continuous CC Knobs and trigger CC Buttons (and
  Notes: a trigger can learn either)
- A click on a row in the list opens it and starts learning right away,
  showing the current mapping and the learn prompt together; a further click
  leaves for the main menu if it's fast, or just closes the row if it's not
- Holding the encoder and turning it on an open row clears that mapping
  instead, showing CLEARED! for a second before closing back to the list
- Browsing past Features without opening it shows whichever CC Knob or
  Button was mapped most recently
- Per-target mappings for live effects and looper controls
- Separate Arp and Drum division mappings
- Sixteen Main-input CC-to-channel/CC remap slots
- Sixteen Note-to-CC slots with momentary or toggle behavior
- Live CC screen for selecting and sending a CC with the encoder
- Four physical buttons can operate as custom notes/CCs, looper controls, or
  chord memories
- Custom button behaviors: Momentary, Latch, and Flappy Bird

In looper button mode each button owns one track and steps through its enabled
actions, Select, Arm, Mute, Solo, Delete, and Undo in that order:

- The first tap on a track always performs the first enabled action, so a single
  press can never reach Delete
- The step only advances while the same button keeps being tapped, and the
  gesture closes after 1.5 seconds or as soon as another button is used
- With the default Arm, Delete, Undo set (Select is off by default) that reads
  as tap to arm, tap again to delete, tap a third time to undo
- Arm selects the track it arms, the same as Select, and disarms it again on a
  second tap if it is already the pending arm. A pending arm survives a clear
  on its own unrecorded track, so clearing after arming leaves the performer
  still armed with the old content out of the way; undoing that clear cancels
  the arm instead, since bringing old content back and starting a fresh take
  are opposite choices

Holding more than one of the four buttons down at once, in looper mode, reaches
two whole-loop gestures no single button can: whichever count is reached first,
two or three, decides what fires, regardless of press order or timing.

- Two buttons down together stops the transport, the same stop the eye or pad
  sensor gives on its first press. Doing it again while already stopped plays
  instead, a plain play/stop toggle from the buttons alone.
- Three buttons down together reaches the same undoable whole-loop clear the
  eye/pad's second stop-then-clear press does: clears every track that has
  anything audible, or brings them all back if every track is already cleared.
  Four down behaves the same as three.

## Clock and transport

- Internal BPM range: 20–300
- Swing range: 0–75
- 4/4 or 3/4 meter
- Clock Input: Ignore or Follow/Client
- Clock Output: Off or Send/Host
- Incoming Start resets synchronized arp phase.
- Incoming Continue resumes a paused loop without returning to its beginning.
- Incoming Stop closes active recording, releases owned notes, and pauses the
  looper at its current position.
- Looper MIDI Transport is independent of whether incoming clock is followed.
- An armed looper take starts exactly on an incoming MIDI Start boundary.
- Tap tempo replaces the normal Eye/Push actions while the BPM screen is open.

Arp and drum timing follow the measured external clock when Clock Input is set
to Follow/Client. Looper transport follows Start, Continue, and Stop, but saved
loop event positions are currently microsecond-based and are not continuously
tempo-stretched if the external tempo changes after recording.

## Main menu

The current top-level screens are:

1. BPM
2. Swing
3. Quick Jump
4. Stutter
5. Echo
6. Arp
7. Velocity
8. Notelength
9. Main Input
10. Arp Out
11. Drumroll
12. DrumDiv
13. Bass
14. Mono
15. Thru Out
16. Round Robin
17. Router
18. Features
19. CC Map
20. Note to CC
21. In CC Out
22. Screen Saver
23. Eye/Push
24. Eye Mode
25. Push
26. 4Button
27. Looper
28. Loop Mix
29. Parameter Lock
30. Chord
31. Key
32. Scale
33. Guitar/Keys
34. Live CC
35. Global
36. Load
37. Save
38. Panic

Feature screens use submenus with a Back item. Holding the encoder switch while
turning changes supported numeric values in steps of ten.

## Main-brain prototype pins

Use the GPIO numbers printed on the RP2040 Zero:

- GPIO0: serial TX to the planned ESP32-C3
- GPIO1: serial RX from the planned ESP32-C3
- GPIO2: I2C SDA for the SSD1306 OLED and VL53L0X
- GPIO3: I2C SCL for the SSD1306 OLED and VL53L0X
- GPIO4: 1 Mbps UART TX to the secondary brain
- GPIO5: 1 Mbps UART RX from the secondary brain
- GPIO6: rotary encoder A
- GPIO7: rotary encoder B
- GPIO8: rotary encoder switch
- GPIO9: physical button 1
- GPIO12: physical button 2
- GPIO10: physical button 3
- GPIO13: physical button 4
- GPIO16: optional RGB status LED, disabled by default
- GPIO26: pressure sensor analog input

GPIO11 is intentionally unused for mechanical clearance. All four physical
buttons are implemented. The current firmware configures them as active-high
inputs with internal pulldowns.

GPIO0 and GPIO1 reserve a normal two-wire UART connection for an ESP32-C3. The
planned ESP firmware will add wireless MIDI, allowing ordinary BLE MIDI devices
such as a standard BLE MIDI foot controller to connect. There is no dedicated
wired foot-pedal subsystem.

The future RP2354A and RP2350-Zero pin assignment is kept separately in
[`max3421e_pins_for_next_pcb.txt`](max3421e_pins_for_next_pcb.txt).

## Build setup

Use:

- Earle Philhower Arduino-Pico board package
- Board: Waveshare RP2040 Zero
- USB stack: Adafruit TinyUSB
- CPU speed: 120 MHz or 240 MHz
- Flash Size: **2MB (Sketch: 1536KB, FS: 512KB)**

The Flash Size setting is not optional for the main brain. The board package
defaults to `2MB (no FS)`, which gives the sketch no filesystem partition at
all. All persistent state lives in the filesystem, so with the default setting
nothing is saved: no presets, no settings, no loops, and no remembered screen.
The firmware says so rather than failing quietly. It shows `NO FILESYSTEM` at
boot, and the diagnostics screen reads `FS NONE` instead of `FS OK` with the
free space.

The main sketch requires the MIDI Library, Adafruit GFX, Adafruit SSD1306, and
VL53L0X libraries. The secondary sketch includes its patched USB Host Shield
2.0 source in its own `src` directory.

The command-line build used for the main brain is:

```sh
arduino-cli compile --warnings all \
  -b rp2040:rp2040:waveshare_rp2040_zero \
  --board-options freq=120,usbstack=tinyusb,flash=2097152_524288 \
  "arpnmidi os/arpnmidi_main_brain"
```

In the Arduino IDE the same setting is Tools, Flash Size,
`2MB (Sketch: 1536KB, FS: 512KB)`. Changing that setting reformats the
filesystem, so presets and loops are lost when it changes.

The secondary build uses the same board, USB stack, and CPU setting. It does
not need the filesystem partition.

## Presets and persistence

- Sixteen preset slots
- Auto Save on or off; it is the device-global storage preference
- Main musical settings, mappings, custom arp, chord memories, and parameter
  locks are stored per preset
- Four loop tracks are stored globally, not per preset
- Loop mute, solo, hidden/undo state, active length, retained length, and events
  survive reboot
- Rolling Time Travel/Stutter history is intentionally RAM-only
- One LittleFS file, `/state.f3`, holds everything: a small header with the
  current preset, Auto Save, and the remembered screen, then one complete record
  per preset slot
- A second LittleFS file, `/loops.f3`, holds the global loop tracks, because
  they are not per preset and change on their own rhythm
- There is no EEPROM. The RP2040 has none, and the emulation rewrote a whole
  4 KB flash sector for any change, so a two-byte screen memory cost as much as
  a preset and every save wrote two stores instead of one
- A menu edit saves at the moment of commitment: the click that leaves the
  edited screen. The pause happens right there, once, whether or not a loop is
  playing, rather than on a delay afterwards
- Nothing changed means nothing written. Every save first compares against the
  stored bytes, so navigation, a no-op exit, and a cancelled edit never touch
  flash and never pause
- Selecting a track is navigation, not content, and does not trigger a save
- Recording is the one state a commit click still defers around, since a
  flash stall landing inside a take is a genuinely bad moment for one
- Changes made automatically while the engine is busy, mapped CCs and
  performance captures, wait for an idle moment rather than forcing themselves
  through. A filesystem write disables interrupts and parks the rendering
  core, so an automatic write never happens while notes, the arp, or a loop
  are running
- The busy hourglass appears before any write starts, and the S flags plus the
  PASS/LATE scheduler numbers on the diagnostics screen show what is going on
- A failed write waits five seconds before another attempt, and a missing
  filesystem stops all attempts, so storage trouble can never grind the
  instrument
- Loading a preset writes anything still pending for the preset being left
- The remembered screen is the exception to "a commit click saves it": just
  looking at a screen is not something the performer made, so it waits for
  the screen to sit still for five full seconds, and for the engine to be
  idle too, rather than saving on every visit

Firmware 3 uses explicit storage schema identities. If the installed preset
layout does not match, the firmware installs the current factory defaults
instead of running prototype preset migrations.

To request a complete factory reset, hold the encoder while powering on,
release it when prompted, and press it again within five seconds.

## Current validation status

- Both sketches compile for Waveshare RP2040 Zero at 120 MHz.
- The four looper, rolling-history, Echo, and Notelength host tests pass.
- The default secondary profile supports the current no-MAX prototype wiring.
- The MAX3421E profile compiles but still needs validation on its target PCB.
- ESP32-C3 wireless MIDI is planned and is not implemented in this firmware.

Project archives are in [`legacy/`](legacy/) and are not needed to build or use
the current firmware.
