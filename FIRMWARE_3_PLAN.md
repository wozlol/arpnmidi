# ARPnMIDI Firmware 3.0 Plan

Firmware 3.0 is the coordinated feature update following the tagged August 26
prototype baseline. It targets the current dual-RP2040 prototype first while
keeping the design portable to the planned dual-RP2354A hardware.

Live MIDI timing, correct note ownership, and reliable note-offs take priority
over display refreshes and immediate flash persistence.

## Processor responsibilities

### Main-brain core 0

- MIDI input parsing and output scheduling
- Musical clock, swing, and transport state
- Arpeggiator and Drum Magic scheduling
- Four-track looper playback and recording
- Note ownership, routing, transformations, and parameter locks
- Bounded queues to the UI core and secondary brain

Core 0 must not wait for the OLED, sensors, controls, EEPROM commits, or a mutex
held by another core.

### Main-brain core 1

- OLED rendering from snapshots
- Encoder, five buttons, distance sensor, and pressure sensor
- Menu state and UI action generation
- Deferred preset and loop persistence

Flash commits must be deferred until the real-time engine reports a musically
safe window. Loop playback must not be interrupted merely to guarantee immediate
persistence.

### Secondary brain

- One core owns native USB MIDI, DIN/TRS MIDI, and the inter-brain UART.
- One core services the MAX3421E USB host.
- Both sides communicate with bounded queues.
- Congestion must be counted and visible on the Panic screen.

## Prototype buttons

- GPIO9: button 1
- GPIO12: button 2
- GPIO10: button 3
- GPIO13: button 4

The future hardware adds a dedicated Back button as documented in the next-PCB
pin plan. Prototype navigation must continue to work without that fifth button.

## Menu conventions

- The blue title on the Input Channel screen says `MAIN INPUT`.
- Encoder press enters or leaves a submenu or edit control.
- Encoder press while turning changes numeric values in increments of ten.
- Every option screen includes `BACK`.
- Arp submenus use letters such as `A`, `B`, and `C` instead of main-menu numbers.
- Feature pages provide manual controls and assignable MIDI controls. MIDI
  mapping never becomes the only way to operate a feature.

## Arpeggiator

One top-level `ARP` entry contains:

- Mode
- Output
- Division
- Velocity
- Length
- Octaves from 1 through 4
- Retrigger: current behavior or clock-synchronized
- Note Order: sorted order or as played
- Custom Pattern
- Back

Arp Velocity and Arp Length retain the behavior of the current top-level
Velocity and Length screens. New top-level Velocity and Note Length entries are
separate live transformations.

### Arp division

Arp Division supports straight, triplet, and dotted divisions plus `DRUM`.
When set to `DRUM`, it follows the effective Drum Magic division live.

If Drum Division already follows Arp when Arp is changed to follow Drum, Drum
Division is reset to the default `1/16` before the new link is made. The reverse
operation follows the same rule. Cyclic links are never allowed.

### Custom arp pattern

- One custom pattern is stored per preset.
- Pattern lengths are 1/4 bar, 1/2 bar, 1 bar, 2 bars, 4 bars, or 8 bars.
- Selecting Learn arms capture; the first note starts it.
- Capture ends at the selected boundary or when the encoder is clicked.
- An early manual ending leaves silence to the selected boundary.
- Capture stores note-on timing, gate length, velocity, and semitone position.
- Pitch offsets are measured from the lowest note played during the take.
- During playback, offset zero maps to the lowest currently held input note.
- The pattern contains at most 32 note-on events.
- Clear removes the stored custom pattern.

## Drum Magic and Drum Roll

Channel-10 arp/split behavior moves out of the Arp Output setting and into a
separate `DRUM MAGIC` menu.

Drum Magic includes:

- On/Off or output channel
- Input from channel 10 or a configurable key split on Main Input
- Split position and mapped drum-note starting point
- Aftertouch-to-velocity On/Off
- Division
- Back

Drum Division supports straight, triplet, and dotted divisions plus:

- `ARP`: follow effective Arp Division live
- `FREE`: no scheduled drum pulse, but Drum Roll may set it temporarily

`DIV NOTE` is renamed `DRUM ROLL`. Its learned note or CC triggers temporarily
change the drum division. The looper records the resulting generated drum notes.

## Clock, transport, swing, and meter

Global settings include:

- Clock In: Ignore or Follow
- Clock Out: Off or Send
- Time signature: 4/4 or 3/4

These musical settings are stored per preset.

Clock behavior:

- MIDI Start resets synchronized arp and loop phase.
- MIDI Continue resumes without resetting phase.
- MIDI Stop stops the looper and synchronized arp and releases owned notes.
- When following external clock, missing clock halts synchronized advancement
  rather than switching unexpectedly to internal clock.
- The last measured BPM remains visible while external clock is absent.
- Clock Out may send internal or followed timing without echoing duplicate clock.

Looper Settings includes `MIDI TRANSPORT`, default On:

- Start begins playback from the beginning and starts recording if the selected
  track is armed.
- Continue resumes without resetting position.
- Stop finalizes active recording and stops playback.
- Song Select values 0 through 3 select tracks 1 through 4.
- MMC Record Strobe arms or starts recording on the selected track.
- MMC Record Exit finalizes recording and begins playback.

Swing is a main-menu entry after BPM and applies to compatible internal
subdivisions without moving bar boundaries. Incoming external clock remains the
authoritative pulse grid.

On the BPM screen, distance and pressure activation perform Tap Tempo instead of
their normal assignments. The screen shows `Tap` once a tap sequence begins.

## Four-track looper

Four loop tracks share a bounded RAM event pool. Tracks and recorded CC
automation survive reboot globally but are not stored per preset. Time-travel
history is RAM-only.

Compact legacy controls remain in the supported 4 KB EEPROM-emulation sector.
Firmware 3 settings and bounded per-preset payloads such as custom arp events,
live-effect targets, mappings, and parameter locks use one fixed record per
preset in LittleFS under the same schema identity. Global loop data also uses
LittleFS rather than EEPROM. The prototype must be built with a flash layout
that reserves filesystem space. The pre-3.0 request for 8 KB of emulated EEPROM
was capped to 4 KB by Arduino-Pico, so its loop image above offset 4096 was not
actually persistent.

Looper Settings includes:

- Auto Rec On/Off
- Time Travel On/Off
- Track Mode
- Auto Quantize: Off or 1/64 through 1/4
- Record CCs On/Off
- MIDI Transport On/Off, default On
- Back

Track modes:

- Layers: each overdub uses the next track; after all four are occupied, further
  overdubs merge onto the oldest layer. Clear/Stop affects all tracks.
- Parts, Auto Solo: recording a new track solos it and unsolos the previous
  track. Clear affects the selected track.
- Manual: the performer chooses the working track. Clear affects the selected
  track.

Each track has its own event count and length. Only Track 1 may initially be
Free. Once its free length is closed, that duration becomes one musical bar and
the displayed BPM changes to match. Other tracks use divisions or multiples of
that established length.

CC recording is optional. Automation is reduced before storage using value and
time thresholds while preserving direction changes, endpoints, and fast moves.
Overflow is reported explicitly and never causes unbounded work in the musical
hot path.

### Time travel

The rolling RAM buffer has 2,048 fixed event slots and tags events as Main or
Looper Track 1 through 4. Activating Time Travel copies Main events from the
immediately preceding selected track length into the chosen loop track, with
the activation point becoming the loop boundary. A Free Track 1 uses one bar
for retrospective capture because the required past window must already have a
known duration. Copying and boundary Note Off repair are divided into fixed
work batches so live scheduling is not blocked. The same tagged snapshots feed
per-target Stutter without allocating a second history buffer.

### Mute and solo

`MUTE/SOLO` resembles the preset grid:

- Track selection on the left
- Solo/Mute mode selection on the right
- Clear and Back
- Muted tracks use a lightweight faded/dotted rendering
- Solo tracks use solid rendering
- Muting, clearing, or switching a sounding track immediately releases its
  owned notes

## Live transformations

The following top-level entries operate on Main and Looper Tracks 1 through 4:

- Velocity
- Note Length
- Stutter
- Echo

Every target has manual On/Off and parameter controls plus separate Features
assignments.

Velocity ranges from 0 through 200 percent with 100 percent neutral. Note Length
ranges from 1 through 200 percent with 100 percent neutral. A continuous MIDI CC
value of approximately 64 selects neutral.

Velocity and Note Length are applied after the looper record point. Shortening a
live note is necessarily predictive because its future physical Note Off is not
known yet: the engine uses the last observed duration for that target, channel,
and note, with the current master division as the first-note fallback. It learns
the real duration when the Note Off arrives. Lengthening is exact because the
known Note Off can be delayed. The scheduler is fixed-size, and changing or
disabling the effect closes its owned notes.

### Stutter

- Works as a note-safe beat-repeat/loop-slicer effect.
- A knob moves from longer to shorter divisions with Off at the bottom.
- A button activates the configured default division momentarily.
- Separate button Features select useful divisions from 1/2 and shorter.
- Stutter Timeout belongs in Stutter Settings and limits continuous activation
  to a fixed number of measures. After timeout, the controlling CC must move
  again before stutter can reactivate.
- Stutter and Time Travel may share the rolling capture infrastructure.
- Stutter snapshots the post-routing, post-Velocity/Note-Length stream for its
  target. The underlying target is suppressed while the frozen slice repeats;
  generated repeats are not fed back into history or the looper.

### Echo

The UX calls the effect `ECHO`; only its repeat-spacing parameter is called
`Delay`.

- Wet: initial echo velocity from 0 through 100 percent of the source note
- Length: musical fade duration
- Delay: straight, triplet, and dotted musical division
- Drift: signed acceleration or deceleration

Echo Length uses musical durations. Drift zero keeps constant spacing. Positive
Drift accelerates and negative Drift slows. Its magnitude is the number of
repeats over which spacing reaches half or double the initial Delay. Spacing
changes exponentially to create a bouncing-marble effect. Echo occurs after the
looper and Stutter, so generated repeats are not printed into the loop. Wet sets
the first repeat level and the remaining repeats fade across Length. All queues,
repeat counts, and per-tick work are bounded.

## Features and CC Map

The current feature-control `MAP CC` screen is renamed `FEATURES`.

Features has two groups:

- CC Knobs for continuous controls
- CC Buttons for momentary, toggle, and discrete actions

Momentary Features can learn either a CC or a note. Playing a note while learning
selects that note instead of a CC. Drum Roll and other trigger maps likewise
learn either notes or CCs.

Button Features include looper record/arm, play/stop, safe clear/undo, combined
Eye/Push looper actions, stutter divisions, Quick Jump On/Off, and the four
physical-button actions.

The new `CC MAP` remaps a CC arriving on Main Input to a chosen output channel
and CC number. Sixteen fixed remap slots are stored per preset.

The runtime action catalog contains separate per-target controls for Velocity,
Note Length, Stutter, Echo Wet/Length/Delay/Drift, linked Arp and Drum divisions,
Quick Jump, looper transport, track selection, mute/solo, safe clear/undo, and
straight Stutter trigger divisions. Feature mappings and CC Map records live in
the fixed per-preset filesystem record.

## Note-to-CC mapping and physical buttons

A Note-to-CC screen learns or manually selects:

- Input channel and note
- Output channel and CC
- Momentary or toggle behavior
- Back

The knob may select values manually or listen to played notes and received CCs.
Sixteen fixed slots are stored per preset. Momentary sends 127 while held and 0
on release; Toggle changes state on each Note On.

Four-button modes include:

- Custom note or CC, with channel and momentary, latch, or Flappy Bird behavior
- Looper control: track select, mute, solo, safe delete, and undo
- Chord memory: Learn, Clear, and Back

Learned chords capture the result after key, scale, and chord processing. Chord
memory playback occurs after those processors so stored chords are not processed
again.

The current prototype button order is GPIO 9, 12, 10, and 13. Custom mode stores
an independent channel, Note/CC number, and behavior for each button. Looper
mode applies any selected combination to the corresponding track; Solo is
exclusive. The four chord memories persist with their preset.

## Parameter locks

Per-note Parameter Lock selects Off or a MIDI channel and includes Back.

- While a note on the selected channel is held, incoming CC changes are stored
  for that note.
- Replaying the note sends its stored CC values.
- A new value for the same CC while the note is held replaces the old value.
- The selected channel is used for both note learning and CC retransmission.

## Routing, Quick Jump, bass, chord, and scale

Round Robin keeps its normal cycling behavior by default. Its submenu adds a
saved Random toggle. When enabled, each new routed note independently selects
one of the enabled Round Robin output channels using the RP2040 hardware random
generator. Immediate repeats are valid random results; this is not a shuffled
or forced-no-repeat cycle.

Each Router channel receives a low and high note bound. Only notes in that range
are routed and transposed. Notes outside the range continue on their original
channel unchanged.

`QUICK JUMP` appears immediately before Main Input and contains:

- Input channel, default 1
- Output channel, default 2
- On/Off
- Back

All three controls are available in Features, with On/Off treated as a button.

Bass Channel adds a highest-note split setting.

`CHORD` appears before Key and Scale. It contains four chromatic or scale-degree
positions from -12 through +12. Position zero is not forced; a chord such as
`-3, 3, 5, 7` intentionally omits the played root. Chord processing follows
scale interpretation so positions use scale degrees when a key and scale are
active and chromatic semitones otherwise.

Scale Correction adds a User Scale editor using selectable intervals and Back.

## Global and preset behavior

Global-menu musical settings are stored per preset. This includes time
signature, aftertouch behavior, clock behavior, Quick Jump, and other musical
configuration.

Auto Save is the only device-global behavior setting. It appears first near
Save/Load. Reset Preset is an action, not stored state.

Global options include:

- Auto Save On/Off
- Channel and poly aftertouch forwarding On/Off
- Map channel aftertouch to a CC
- Main Input channel aftertouch to arp velocity
- Clock In and Clock Out
- Time signature
- Reset Preset
- Back

Firmware 3 intentionally does not migrate prototype-era presets. The preset
schema has an explicit identity; when it changes, all preset slots receive the
current factory defaults. This keeps storage handling lean while the product is
still in prototype development.
The four global loop tracks are stored separately from presets.

## Panic screen

The Panic screen must remain lightweight and show useful real-time diagnostics:

- MIDI/USB device counts
- Main and secondary queue depths and high-water marks
- Dropped event counters
- Looper and rolling-buffer utilization
- Storage-dirty and pending-commit state
- External clock state and age
- Overload risk
- Last incoming channel/message

Panic actions and every mode transition that invalidates sounding output must
release notes through the same centralized note-ownership system.
