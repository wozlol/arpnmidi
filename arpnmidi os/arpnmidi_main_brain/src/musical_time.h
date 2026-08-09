#pragma once

#include <stdint.h>

namespace arpnmidi3 {

// 96 internal pulses per quarter note represents straight, triplet, and dotted
// values exactly through dotted and triplet 1/64 notes.
constexpr uint16_t kInternalPpqn = 96;
constexpr uint8_t kMidiClockPpqn = 24;
constexpr uint8_t kInternalPulsesPerMidiClock = kInternalPpqn / kMidiClockPpqn;

enum class MusicalDivision : uint8_t {
  Whole = 0,
  HalfDotted,
  Half,
  QuarterDotted,
  HalfTriplet,
  Quarter,
  EighthDotted,
  QuarterTriplet,
  Eighth,
  SixteenthDotted,
  EighthTriplet,
  Sixteenth,
  ThirtySecondDotted,
  SixteenthTriplet,
  ThirtySecond,
  SixtyFourthDotted,
  ThirtySecondTriplet,
  SixtyFourth,
  SixtyFourthTriplet,
  Count
};

constexpr uint8_t kMusicalDivisionCount =
    static_cast<uint8_t>(MusicalDivision::Count);

const char *divisionName(MusicalDivision division);
uint16_t divisionPulses(MusicalDivision division);
uint64_t quarterNoteUs(float bpm);
uint64_t divisionDurationUs(MusicalDivision division, float bpm);
uint32_t barPulses(bool threeFour);
uint64_t barDurationUs(float bpm, bool threeFour);
bool isDotted(MusicalDivision division);
bool isTriplet(MusicalDivision division);

}  // namespace arpnmidi3
