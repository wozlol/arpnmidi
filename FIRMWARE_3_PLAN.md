# ARPnMIDI Firmware 3 architecture

Firmware 3 runs on the current dual-RP2040 prototype and keeps the musical hot
path compatible with the planned dual-RP2354A system.

The operating priorities are:

1. Deliver notes and note offs on time.
2. Keep clocked arp, drum, loop, and effect work bounded.
3. Keep USB, DIN/TRS, and inter-brain traffic moving without blocking.
4. Render the display and write flash only when those operations cannot disturb
   musical timing.

## Processor responsibilities

### Main-brain core 0

- Encoder, four physical buttons, and pressure input
- Native USB and inter-brain MIDI input parsing
- MIDI clock, swing, and transport
- Arpeggiator and Drum Magic scheduling
- Four-track looper playback, recording, and Time Travel imports
- Routing, scales, chords, parameter locks, and note ownership
- Velocity, Note Length, Stutter, and Echo engines
- EEPROM and LittleFS writes only during musically idle windows

Core 0 drains no more than 32 inter-brain UART bytes per pass and performs
bounded work in each scheduler. Automatic flash writes are deferred while
notes, arp, playback, recording, or pending UI work are active.

### Main-brain core 1

- OLED rendering when visible state is dirty
- VL53L0X distance-sensor polling
- Outgoing 1 Mbps inter-brain UART queue

The main-to-secondary queue is fixed at 256 messages. It reserves capacity for
critical note offs and counts normal and critical drops separately.

### Secondary-brain core 0

- Native USB MIDI device input and output
- External DIN/TRS MIDI at 31,250 baud
- Inter-brain UART at 1 Mbps
- Bounded transfer queues to and from the host core

### Secondary-brain core 1

- MAX3421E USB-host servicing when enabled
- Hosted USB MIDI input and output
- Hosted-device MIDI OUT endpoint checks

The two brains must be flashed as a matched pair because both ends must use the
same inter-brain UART speed and message behavior.

## Musical timing

- Internal resolution is 96 PPQN.
- Incoming MIDI Clock is measured with bounded smoothing.
- Straight, triplet, and dotted divisions are represented exactly through
  1/64T.
- External clock follow stops synchronized advancement when the clock source
  disappears instead of silently falling back to internal timing.
- Clock Input and Looper MIDI Transport are independent settings.
- MIDI Start resets synchronized arp phase and starts an armed loop take at the
  transport boundary.
- MIDI Continue preserves looper position.
- MIDI Stop closes recording, releases notes, and pauses the looper.
- Song Select 0 through 3 selects looper tracks 1 through 4.
- MMC Stop, Play, Deferred Play, Fast Forward, Rewind, Record Strobe, Record
  Exit, Record Pause, Pause, and Reset have bounded looper actions.

Arp and Drum schedulers use the measured external tempo when Clock Input is
Follow/Client. Recorded looper event positions remain microsecond-based. Start,
Continue, and Stop are synchronized, but recorded loops are not continuously
tempo-stretched when an external source changes tempo after capture.

## Event path

Secondary-side MIDI always enters the main brain before fan-out. Main input is
processed in this order:

1. Control learning, transport, and source accounting
2. Quick Jump, channel routing, and drum-input claiming
3. Key, scale, chord, arp, bass, and thru processing
4. Main-loop recording and rolling history capture
5. Per-target Velocity and Note Length
6. Stutter
7. Echo
8. Main native USB and queued secondary output

Loop playback re-enters the musical path with a track-specific source identity.
Generated Stutter and Echo repeats are not fed back into the looper or rolling
history, which prevents recursive growth.

## Arpeggiator and drums

The Arp submenu contains Mode, Division, Arp Velocity, Arp Length, Octave Range,
Retrig, Order, Length, Learn Custom, Clear Custom, and Back.

## OLED language conventions

- Prefer short yellow titles that fit the screen, with longer descriptions only
  on summary screens when the title alone is not clear.
- Mono Retrig uses yellow title `MONO` and summary description
  `Retrig last key`.
- Per-note parameter lock uses yellow title `PLOCK` and summary description
  `Per-Note Parameter Lock`.
- Submenu selector pages use `BACK` to leave the submenu. Keep `BACK` where it
  reads naturally as navigation; do not rename every exit to `CANCEL`.
- Parameter edit pages that need rollback should use `CANCEL`, restore the value
  from when the user entered that edit, and keep live editing behavior while the
  value is being changed.

Custom arp capture:

- Starts with the first played note after Learn is armed
- Ends at its selected musical boundary or on an encoder press
- Preserves silence after an early manual ending
- Stores up to 32 note events with start, gate, velocity, and pitch offset
- Measures pitch offsets from the lowest note in the take
- Maps offset zero to the lowest currently held note during playback
- Persists per preset

Drum Magic has an independent input mode, output channel, key split, drum-note
mapping, aftertouch-to-velocity option, and division. Arp can follow Drum, and
Drum can follow Arp, but setting one link breaks an opposing link by returning
the other division to 1/16.

Drum Roll learns notes or CCs for temporary drum-division changes. Generated
drum notes enter loop recording like other musical output.

## Four-track looper

The looper uses a fixed pool of 3,072 events shared by four tracks. Allocation,
playback, Time Travel imports, CC pruning, and resize copying are bounded.

Track modes:

- Layers records successive layers into available tracks. Once all four are
  occupied, overdub continues on the oldest track.
- Parts Auto Solo makes a newly recorded part the exclusive solo track.
- Manual leaves working-track selection to the performer.

Each track stores note ons, note offs, and optional pruned CC automation.
Quantize can be Off or a straight division from 1/64 through 1/4.

### Retained length behavior

Each track has an audible length and a retained-content length:

- Extending beyond retained content duplicates the complete retained recording
  until the requested length is filled.
- Shrinking changes the audible length but does not delete events beyond it.
- Re-extending within the retained length exposes the saved events without
  copying them again.
- Further extension copies the complete retained block.
- Replacement recording and Time Travel import intentionally establish new
  content and discard the old retained copies.
- The resize is rejected before mutation if the fixed event pool lacks room.
- A sounding resized track releases owned notes, preserves normalized playback
  position, and resumes from the correct event cursor.

The loop file stores both audible and retained length so this behavior survives
reboot.

### Time Travel and Stutter history

A fixed 2,048-event rolling buffer records tagged Main and looper-track output
in RAM. Time Travel copies the preceding selected-length Main window into the
working track. Import work and missing boundary note offs are processed in
fixed batches.

Stutter snapshots the same tagged history rather than allocating a second live
buffer. Activating a slice suppresses the underlying target until the effect is
released or its timeout expires.

## Live target engines

Main and each looper track have independent settings for:

- Velocity from 0 to 200 percent
- Note Length from 1 to 200 percent
- Stutter enable and division
- Echo enable, Wet, Length, Delay, and signed Drift

All note schedulers have fixed capacities and close their owned notes when
disabled, retargeted, muted, cleared, or stopped.

## Routing and mappings

- The sixteen Router rows store output channel, note-low, note-high, and
  transpose. Notes outside the selected range continue unchanged.
- Round Robin has cycle and hardware-RNG true-random modes.
- Quick Jump has input, output, and enable controls.
- Bass has channel/octave selection and a highest-note limit.
- Mono Retrig uses last-note priority and recalls the newest still-held note.
- Features provides continuous CC Knobs and CC-or-note Buttons.
- CC Map has sixteen Main-input CC remap slots.
- Note to CC has sixteen momentary or toggle slots.
- Parameter Lock stores the latest CC value for each note and CC pair on one
  selected channel.

## Physical buttons

The current prototype button order is GPIO9, GPIO12, GPIO10, and GPIO13.

The buttons can run in three modes:

- Custom Note or CC with Momentary, Latch, or Flappy Bird behavior
- Looper control with Select, Mute, Solo, Delete, and Undo actions
- Chord Memory with Learn and Clear actions

If multiple looper actions are enabled, successive presses advance through the
enabled actions in a fixed order. They are not all fired on one press.

## Presets and storage

- Sixteen preset slots
- Compact preset image in 4 KB flash-backed EEPROM emulation
- Fixed per-preset extended records in LittleFS
- One global four-track loop file in LittleFS
- Custom arp, mappings, chord memories, live targets, router ranges, and
  parameter locks stored per preset
- Four loop tracks stored globally rather than per preset
- Time Travel/Stutter rolling history stored only in RAM
- Auto Save as the one device-global storage preference

Firmware 3 does not migrate incompatible prototype preset layouts. A schema
mismatch installs current factory defaults across all slots. This keeps boot
and persistence code bounded and prevents a stale binary layout from being
interpreted as current settings.

The RP2040 main sketch must use the 2 MB flash layout with 512 KB filesystem.

## Diagnostics

The Panic screen shows:

- DIN and USB input counts
- Last incoming status and data
- Main-to-secondary queue depth, high-water mark, and drops
- Looper event-pool use and overflow count
- Rolling-history use and overwrite count
- External clock age or internal-clock state
- Pending preset, loop, and extended-record writes
- A visible overload-risk state

Panic and every state change that invalidates output use the centralized
note-release paths.

## Hardware direction

The current firmware pins are documented in the two READMEs. The separate
[`max3421e_pins_for_next_pcb.txt`](max3421e_pins_for_next_pcb.txt) file defines
the planned dual-RP2354A layout and its compatible dual-RP2350-Zero module
layout. It is not the RP2040 prototype pin map.
