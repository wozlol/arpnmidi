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
- LittleFS writes only during musically idle windows

Core 0 drains no more than 32 inter-brain UART bytes per pass and performs
bounded work in each scheduler. Automatic flash writes are deferred while
notes, arp, playback, recording, or pending UI work are active.

### Main-brain core 1

- Outgoing 1 Mbps inter-brain UART queue, which outranks everything else here
- OLED rendering only when something visible actually changed, capped at one
  frame per 50 ms and deferred while outgoing MIDI is queued, with a 100 ms
  starvation bound. There is no periodic refresh. Events that change nothing
  on the selected screen must not mark the display dirty, so a quiet screen
  costs zero frames during a performance
- A push is the full 1024-byte frame, about 20 ms at 400 kHz regardless of how
  little changed, but this core no longer sits through the transfer: it hands
  the frame to a DMA channel paced by the I2C peripheral's own DREQ (its "FIFO
  has room" signal) and returns immediately, so the wire time runs in the
  background while this core goes back to draining outgoing MIDI. Nothing may
  touch the frame buffer, or start any other transaction on that same I2C
  peripheral, while a push is still in flight, checked by polling both the
  DMA channel and the peripheral's own bus-activity bit, since DMA finishing
  only means every byte reached the FIFO, not that the peripheral is done
  shifting them onto the wire. renderDisplayIfNeeded checks this once at its
  own top, which covers everything reachable through it; the one path that
  runs outside it, the busy-hourglass draw before a flash write freezes core
  0, waits on it explicitly first. LOOPER and LOOP MIX still change on nearly
  every click while the performer is actively working the loop, so those two
  screens keep a 300 ms cap instead of the general 50 ms on top of this,
  coalescing a flurry of clicks into fewer, now background, pushes
- VL53L0X distance-sensor polling, data-ready peek first so a slow or wedged
  sensor costs one register read instead of a blocking wait

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

The arp and drum schedulers share one grid origin. Whichever engine starts the
phrase plants the origin and is the boss, and the other joins that grid at its
own division rather than planting a second origin that would drift against the
first. A new arp key phrase joins a rolling drum grid instead of resetting it.
Engine accessors read stored settings directly, never the composite menu
accessors, whose raw values follow the submenu cursor and are navigation state.

Releasing every held key already left the arp side of the grid exactly where
it was, just paused; releasing every held drum note did not, it force-forgot
the grid every time regardless of what else was going on, so the next hit
planted a brand new origin at that instant instead of rejoining where the
beat already was. That is the normal, wanted behavior for a fresh performance
note when Retrig isn't set to Clock Sync, but while the loop has the clock
locked, armed, recording, or playing, it must not happen: a captured take
with no quantize to mask it would show the seam as an audible timing jump
right where the drum notes happened to gap. Forgetting the grid on release
now only happens outside that locked state, matching what already happened
on the arp side.

"Playing" alone was too broad a reason to treat the clock as locked: an
empty loop, or one that had just been cleared down to nothing, still
reports `playing()` true with nothing left to actually stay in sync with, a
mystery clock in every sense the grid-preservation fix above exists to
avoid. `releaseSilencedMultitrackOutputs`, already called after every clear,
undo, mute, and solo change, now also stops the transport once every track
has lost its content, checked by track content and hidden state, not
audibility, so muting or soloing everything into silence never trips it,
only an actual clear does. `undoLoopTrackClear` resumes on its own whenever
an undo makes a track audible again while stopped, since retrigger itself
does nothing on a stopped loop, so bringing content back keeps working
exactly as before; this pairing is what lets a genuinely empty loop go
fully quiet, transport included, rather than sitting in a played-but-silent
state that fools every downstream "is there a real clock" check.

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

Thru output is shared, ref-counted per output note across every source, so a
note sounding from two places is turned off once, and remembers the channel
it actually went out on at the moment it started, not whatever the current
setting says, so a mid-note channel change can't strand it. That memory only
helps if the Note Off actually reaches it: `onInputNote` compares an
incoming message's channel against the current input channel setting before
anything else, and a message on any other channel is forwarded raw,
bypassing thru's claim-release step entirely. A loop note recorded while its
channel matched the input channel still owns a thru claim by the time its
release comes due, an arbitrary time later for a clear or undo forcing the
release rather than the note ending on its own, so if the input channel
setting had moved on by then, the release used to take the raw path and
never reach the claim, leaving the note stuck sounding on the channel it was
actually sent on. A Note Off now always tries to release a thru claim first,
regardless of which channel path it otherwise takes; harmless for a note
thru never claimed.

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
  value is being changed. `CANCEL` is a selectable item and should only act when
  pressed; rolling onto it must not cancel automatically.

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
Quantize is per track and can be Off or any straight, dotted, or triplet
division from 1/4 down through 1/64T, the same order and range the main
Division list uses on its short end, just without 1/2 and 1/1, the looper
never needed anything looser than a quarter note for. The track being
written owns the setting, so a drum part can land on a grid while a pad
stays free. The LOOPER summary shows each track's length and quantize
together, which is why lengths read as 2Br rather than 2 Bars; a straight
division there keeps its "q" prefix (q4, q8, q16...), but a dotted or
triplet division drops it, since the name already carries its own D or T
letter and dropping "q" is what keeps rows like 32T fitting. Its Q flag box
is a master indicator lit whenever any track has a quantize set, not just
the selected one. The LOOPER submenu itself, Track, Length, Track Quantize,
New Track's mode, Auto Arm, Time Travel, Rec CC, Transport, Back, groups the
per-track fields first.

### Working-track and note-ownership rules

A cleared track is free space, not material. Recording it replaces it, arming
never destroys it before the first captured event, and an armed record that has
not started yet follows the working track so the displayed track and the write
target cannot disagree. Only a layer that captured something advances the
working track.

Each track also stores where its own cycle begins inside the shared transport
cycle. A layer that started partway through the others keeps that relationship
through stop, clear, undo, and play again, so starting the loop restores the
ensemble rather than restarting every track at its own beginning. The phase is
stored in the loop file as a fraction of the track length, in space the format
already reserved, so older loop files load as zero and behave as before.

Track playback is gated so a track emits one Note Off per Note On for a given
channel and note. Stored material cannot guarantee that on its own: overlapping
duplicate notes from chords, the arp, or drum generation collapse into a single
synthesized boundary Note Off, and an overdub pass can store a Note Off whose
Note On belongs to an earlier pass. Without the gate the final output reference
count never returns to zero and the note stays latched. A track that stops being
playable, including one whose events are replaced underneath a sounding note,
releases what it owns instead of waiting for a loop boundary that never comes.

A synthesized boundary Note Off also clears that track's own claim on the
performer-facing held-note bookkeeping arp, bass, and legato all read from,
regardless of which channel the note happens to be on. The normal input
dispatch only updates that bookkeeping for notes on the currently selected
input channel, so a loop-recorded note whose channel does not match, whether
because it was recorded on another channel or because the input channel
setting changed after it was recorded, would otherwise leave that track
holding the note forever. Bass reads a stuck hold like that as a note that
never lets go.

A track's playback cursor keeps stepping through its own recorded events every
tick regardless of audibility: muted, soloed out, or hidden all still let time
pass inside that track's data, since the cursor only stops advancing while the
track is empty or the transport itself is not running. Recovering audibility,
an unmute, losing an exclusive solo, or an undo, therefore does not land back
at the top of the data; it lands wherever the cursor already is, mid-phrase.
Picking up only the next note-on from there would silently skip whatever was
already sounding. Recovery instead replays every event the cursor has already
stepped past this cycle and retriggers whatever is still held as of right now,
so it sounds like the part was playing the whole time even though every note
is technically a fresh trigger. The replay is a core method, `collectHeldNotes`,
that walks a track's own event list from its head up to its live cursor and
reports the net-held notes; the sketch wraps mute, exclusive solo, and undo so
each captures audibility before and after the change and retriggers only the
tracks that newly became audible, never ones that already were.

### Redundant control paths

The looper is reachable more than one way on purpose, so a performance never
depends on a single control:

- Holding the encoder and turning changes the working track from the LOOPER and
  LOOP MIX screens, in the summary as well as inside the menu.
- Loop Mix applies one mode, Solo, Mute, Clear, or Arm, to whichever track is
  picked, and opens on Arm. Clear doubles as undo for a track that was cleared and not recorded
  over. Arm selects and arms the picked track, takes the arm back off when it is
  already the target, and starts a stopped or paused transport. Clicking Solo or
  Mute again, while that mode already holds, resets the whole mix instead.
- The master rec/play trigger accepts a double tap from any source that drives
  it. A double tap safe clears a layer and arms it, a single trigger before
  anything is played takes that arm back off, and a further double tap restores
  the layer and plays it, staying on that same working track rather than
  moving on, since undo restores content, it does not capture anything. In
  Layers, a double tap that lands on a track that is neither cleared nor
  holding content steps back to the most recently recorded layer first;
  Manual and Parts Auto Solo always stay on the working track.
  Every double tap's first press, on its own, still runs the ordinary
  single-trigger logic, since there is no way to know in advance a second
  press is coming. If the working track already has content and is playing,
  that first press silently begins an overdub take, one that the second
  press's recognized double tap then finishes. `finishRecording` used to
  decide whether a pass captured anything by checking `track.count > 0`,
  correct for a replace, which always starts from zero, but wrong for an
  overdub onto existing content, where that count is never zero regardless of
  whether the accidental pass added a single event. A genuinely empty
  overdub pass therefore used to read as a completed take and silently
  auto-advance the working track, right as the second press's own logic ran,
  landing the clear or undo on whatever track it advanced to instead of the
  one actually selected. The fix tracks each pass's starting count and checks
  `track.count > recordStartCount_` instead, so an overdub that captured
  nothing is correctly recognized as empty and never triggers the advance.
- Auto Arm fires in exactly one place: the instant a fixed-length pass
  concludes on its own because it reached its length, inside the same check
  that already decides whether the pass captured anything. Nothing else
  triggers it, not picking a different track by hand, not any other action. In
  Manual mode, where the working track never advances on its own, this
  continues straight into an overdub of the same track, which is itself never
  time-limited so no repeated re-trigger is needed. In Layers and Parts Auto
  Solo it arms the track the existing auto-advance just moved to. An earlier,
  broader version polled continuously and re-armed on every idle pass
  regardless of history, which could never be turned off and, worse, kept
  restarting a capture pass mid-note, which is what was leaving notes stuck on
  the thru channel while playing over a loop.
- Holding the encoder switch into a panic is a deliberate emergency reset, not
  just a silence: on top of the usual panic, all notes off and the transport
  stopped, it also gives the loop a clean slate. Unlike the eye/pad's
  stop-then-clear press and holding three buttons together, this clear is
  one-directional on purpose: it only ever clears whatever is still live and
  leaves an already-cleared track exactly alone, it never undoes, so there is
  no ambiguity about which way a held panic goes. The clear runs first and
  panic last, since panic's all-channel sweep is the most exhaustive
  kill-everything pass there is, the one thing that has to get the final word
  so nothing the clear stirs up can slip past it and stick. This is scoped to
  the held gesture specifically; a quick tap on the PANIC screen, a mapped
  Feature Button panic, and the panic a preset load fires on its way out all
  stay silence-only.

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

Stutter and Echo have one further target beyond Main and the four fixed
tracks: SELECTD, its own independent settings rather than a share of any
track's own, whose real target dynamically follows whichever loop track is
currently selected. A note from that track feeds SELECTD's pipeline, tagged
with SELECTD's own target slot, alongside the note's own track's normal
processing, never in place of it: SELECTD never repeats the dry note a
second time, only whatever it stutters or echoes on top. The moment the
selected track changes, `selectLooperTrack` tears down SELECTD's stutter
repeater and echo tails and releases its held output before anything from
the new track can reach it, so nothing keeps replaying old, orphaned content
from a track that is no longer selected, and nothing is left stuck sounding
from the one that was. Velocity and Note Length do not have SELECTD.
Both submenus lead with their length/repeat-size parameter and end with the
target picker, right before Back.

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
- Looper control with Select, Arm, Mute, Solo, Delete, and Undo actions, in
  that fixed order, with Arm, Delete, and Undo on by default and Select off
- Chord Memory with Learn and Clear actions

If multiple looper actions are enabled, successive presses advance through the
enabled actions in a fixed order. They are not all fired on one press. The
sequence only advances while the same button is tapped repeatedly on the track
that is already selected, and it restarts after a short pause, so the first tap
on a track always means the first action and a single press cannot reach Clear.
Arm counts as locking to a track for this the same way Select does, since the
button names a specific track either way, so Arm alone still lets pressing a
different button restart the sequence for its own track.

Arm selects the track as part of arming it and disarms it again on a second
press if that track is already the pending arm, matching Loop Mix's Arm mode.
A pending arm survives a clear on its own, unrecorded track, so the sequence
Select, Arm, Clear leaves the performer armed and ready with the old content
now out of the way; Undo is the opposite choice and cancels that pending arm
when it restores the old content instead.

In Looper mode the buttons also respond to more than one held down at once: a
fresh press counts however many of the four are physically down at that
instant, no waiting, no window, since none is needed, holding a second or
third button down later still leaves the earlier ones reading as held. One
button still means its own per-track action, two down together means stop
or play, three down together means the whole-loop clear or undo.

- Two buttons down together is the same stop the eye or pad sensor gives on
  its first press: finish anything mid-capture and stop the transport. Doing
  it again while already stopped plays instead of chaining into that
  sensor's stop-then-clear escalation, so this is a plain play/stop toggle
  from the buttons alone.
- Three buttons down together is the same undoable whole-loop clear the
  eye/pad reaches on a second stop-then-clear press: clear every track that
  has anything audible, or if every track is already cleared, bring them all
  back instead. Four down behaves the same as three.

## Presets and storage

- Sixteen preset slots
- One LittleFS state file: a header followed by one complete record per slot
- One global four-track loop file in LittleFS
- No EEPROM. The RP2040 has none, and the emulation rewrites a whole 4 KB flash
  sector for any change, so it cost more wear and more stall time than the
  filesystem for the same data, and split one preset across two stores
- Custom arp, mappings, chord memories, live targets, router ranges, and
  parameter locks stored per preset
- Four loop tracks stored globally rather than per preset
- Time Travel/Stutter rolling history stored only in RAM
- Auto Save as the one device-global storage preference

A flash write is a musical event on this chip, not a background chore. The
filesystem disables interrupts and parks the rendering core for the whole erase
and program, which costs tens of milliseconds with no MIDI input, no display,
and no scheduling. Menu edits therefore save synchronously at the exit click,
where the performer expects the pause, once, whether or not a loop happens to
be playing: it is a one-shot deliberate action, not a repeating background
trigger, so it always attempts the write rather than waiting for the engine to
go quiet. Recording is the one state a commit click still defers around, since
a stall landing inside a take would corrupt it. Every save compares against
the stored bytes first, so nothing is written when nothing changed, which is
what makes navigation, no-op exits, and cancelled edits free regardless of
engine state. Changes made automatically rather than by a fresh click, mapped
CCs and performance captures, still wait for a genuinely quiet engine, since
those can repeat continuously during a performance. A failed write backs off
for five seconds and a missing filesystem disables attempts entirely, because
retrying every loop pass would grind the whole instrument. Loading a preset
flushes anything still pending for the preset being left.

The remembered screen is the one exception to "a commit click saves it,"
because merely looking at a screen is not something the performer made. There
is no way to make a flash write itself imperceptible: the erase and program
cost the same tens of milliseconds regardless of how few bytes changed, a
12-byte header the same as a full preset record. It instead waits for the
screen to sit still for a long window, five seconds, not just a short one, and
still requires the engine to be genuinely idle on top of that, so browsing
through several screens in a row does not attempt a write after every stop
along the way.

The arp and drum scheduling grid only used to clear reactively, on the next
note-off after everything went idle. Stopping the looper with no notes
currently held produced no such event, so the grid lingered at a nonzero
timestamp indefinitely, which the engine-idle check read as still running,
permanently blocking every automatic flash write until an unrelated note
happened to clear it. The idle release now runs every arp tick, so it
self-heals within one pass of the looper actually going idle, and panic
triggers it immediately rather than waiting for the next tick.

Firmware 3 does not migrate incompatible prototype preset layouts. A schema
mismatch installs current factory defaults across all slots. This keeps boot
and persistence code bounded and prevents a stale binary layout from being
interpreted as current settings.

The RP2040 main sketch must use the 2 MB flash layout with 512 KB filesystem.
The board package defaults to no filesystem partition, and with that default
nothing can be stored at all. The firmware reports the condition at boot and on
the diagnostics screen rather than failing quietly.

The PANIC screen shows scheduler health measured over the previous second:
P is the worst main-loop pass in microseconds and L is the worst arp or drum
step lateness in microseconds. A healthy instrument shows P under about two
thousand and L in the low thousands.

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
