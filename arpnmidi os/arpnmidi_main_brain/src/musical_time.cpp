#include "musical_time.h"

namespace arpnmidi3 {
namespace {

constexpr const char *kNames[kMusicalDivisionCount] = {
    "1/1",  "1/2D",  "1/2",  "1/4D",  "1/2T",  "1/4",  "1/8D",
    "1/4T", "1/8",   "1/16D", "1/8T",  "1/16", "1/32D", "1/16T",
    "1/32", "1/64D", "1/32T", "1/64",  "1/64T"};

constexpr uint16_t kPulses[kMusicalDivisionCount] = {
    384, 288, 192, 144, 128, 96, 72, 64, 48, 36,
    32,  24,  18,  16,  12,  9,  8,  6,  4};

constexpr bool valid(MusicalDivision division) {
  return static_cast<uint8_t>(division) < kMusicalDivisionCount;
}

}  // namespace

static_assert(kInternalPpqn % kMidiClockPpqn == 0,
              "Internal timing must divide MIDI clock exactly");
static_assert(kPulses[static_cast<uint8_t>(MusicalDivision::SixtyFourthDotted)] == 9,
              "96 PPQN must represent dotted 1/64 exactly");
static_assert(kPulses[static_cast<uint8_t>(MusicalDivision::SixtyFourthTriplet)] == 4,
              "96 PPQN must represent triplet 1/64 exactly");

const char *divisionName(MusicalDivision division) {
  return valid(division) ? kNames[static_cast<uint8_t>(division)] : "1/16";
}

uint16_t divisionPulses(MusicalDivision division) {
  return valid(division) ? kPulses[static_cast<uint8_t>(division)] : 24;
}

uint64_t quarterNoteUs(float bpm) {
  if (bpm < 1.0f) bpm = 1.0f;
  return static_cast<uint64_t>(60000000.0f / bpm + 0.5f);
}

uint64_t divisionDurationUs(MusicalDivision division, float bpm) {
  return (quarterNoteUs(bpm) * divisionPulses(division) + (kInternalPpqn / 2U)) /
         kInternalPpqn;
}

uint32_t barPulses(bool threeFour) {
  return static_cast<uint32_t>(kInternalPpqn) * (threeFour ? 3U : 4U);
}

uint64_t barDurationUs(float bpm, bool threeFour) {
  return quarterNoteUs(bpm) * (threeFour ? 3ULL : 4ULL);
}

bool isDotted(MusicalDivision division) {
  switch (division) {
    case MusicalDivision::HalfDotted:
    case MusicalDivision::QuarterDotted:
    case MusicalDivision::EighthDotted:
    case MusicalDivision::SixteenthDotted:
    case MusicalDivision::ThirtySecondDotted:
    case MusicalDivision::SixtyFourthDotted:
      return true;
    default:
      return false;
  }
}

bool isTriplet(MusicalDivision division) {
  switch (division) {
    case MusicalDivision::HalfTriplet:
    case MusicalDivision::QuarterTriplet:
    case MusicalDivision::EighthTriplet:
    case MusicalDivision::SixteenthTriplet:
    case MusicalDivision::ThirtySecondTriplet:
    case MusicalDivision::SixtyFourthTriplet:
      return true;
    default:
      return false;
  }
}

}  // namespace arpnmidi3
