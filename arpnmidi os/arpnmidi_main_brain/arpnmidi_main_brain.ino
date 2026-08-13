/*
  arpnmidi.ino
  RP2040 Zero MIDI router / arpeggiator / processor / multitool

  Pin plan using the board labels you can see on this RP2040 Zero:

  0   Serial TX to ESP32-C3
  1   Serial RX from ESP32-C3
  2   I2C SDA for SSD1306 + VL53L0X
  3   I2C SCL for SSD1306 + VL53L0X
  4   1 Mbps inter-brain UART TX to secondary brain
  5   1 Mbps inter-brain UART RX from secondary brain
  6   Encoder A
  7   Encoder B
  8   Encoder push
  9   Local button 1
  12  Local button 2
  10  Local button 3
  13  Local button 4
  26  Push/pressure sensor analog in

  11 intentionally avoided.

  Notes:
  - MAX3421E USB host lives on the secondary brain in this build.
  - Build this with the Arduino-Pico core, Tools->USB Stack = Adafruit TinyUSB,
    and CPU speed = 120 MHz or 240 MHz.
  - The display is only redrawn when something visible changes.
  - Settings persist in a single LittleFS state file with 16 preset slots.
*/

#include <Arduino.h>
#include <Wire.h>
#include <LittleFS.h>
#include <hardware/watchdog.h>
#include <hardware/dma.h>
#include <hardware/i2c.h>
#include <MIDI.h>
#if ARPNMIDI_ENABLE_RGB_LED
#include <Adafruit_NeoPixel.h>
#endif
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Fonts/FreeSans9pt7b.h>
#include <VL53L0X.h>
#include "src/clock_engine.h"
#include "src/echo_engine.h"
#include "src/four_track_looper.h"
#include "src/note_length_engine.h"
#include "src/rolling_history.h"

#ifndef ARPNMIDI_ENABLE_USB_DEVICE_MIDI
#define ARPNMIDI_ENABLE_USB_DEVICE_MIDI 1
#endif

#ifndef ARPNMIDI_ENABLE_RGB_LED
#define ARPNMIDI_ENABLE_RGB_LED 0
#endif

/*
  Front-panel hardware mode:
  - 0 = DIP encoder/display build: original encoder direction, two physical clicks
        per UI increment, display rotated 180 with the mode band at the bottom.
  - 1 = SMD encoder/display build: encoder direction reversed, one physical click
        per UI increment, display in normal orientation with the yellow mode band
        at the top and the blue per-mode settings area below it.
*/
#ifndef ARPNMIDI_SMD_PANEL_MODE
#define ARPNMIDI_SMD_PANEL_MODE 1
#endif

#if ARPNMIDI_ENABLE_USB_DEVICE_MIDI
#include <Adafruit_TinyUSB.h>
#endif

constexpr uint8_t PIN_DIN_MIDI_RX = 5;
constexpr uint8_t PIN_DIN_MIDI_TX = 4;
constexpr uint8_t PIN_I2C_SDA = 2;
constexpr uint8_t PIN_I2C_SCL = 3;
constexpr uint8_t PIN_ENC_A = 6;
constexpr uint8_t PIN_ENC_B = 7;
constexpr uint8_t PIN_ENC_SW = 8;
constexpr uint8_t PIN_BUTTON_1 = 9;
constexpr uint8_t PIN_BUTTON_2 = 12;
constexpr uint8_t PIN_BUTTON_3 = 10;
constexpr uint8_t PIN_BUTTON_4 = 13;
constexpr uint8_t PIN_PUSH = 26;
constexpr uint8_t PIN_RGB_LED = 16;
constexpr uint8_t USB_DEVICE_SOURCE_PORT = 253;
constexpr uint32_t INTER_BRAIN_MIDI_BAUD = 1000000UL;

constexpr uint8_t OLED_ADDR = 0x3C;
constexpr uint8_t SCREEN_W = 128;
constexpr uint8_t SCREEN_H = 64;
constexpr uint8_t MODE_INFO_H = 16;
constexpr uint8_t SETTING_AREA_H = 48;
#if ARPNMIDI_SMD_PANEL_MODE
constexpr uint8_t DISPLAY_ROTATION = 0;
constexpr uint8_t MODE_INFO_Y = 0;
constexpr uint8_t SETTING_AREA_Y = MODE_INFO_H;
constexpr int8_t ENCODER_DIRECTION = -1;
constexpr uint8_t ENCODER_COUNTS_PER_INCREMENT = 2;
#else
constexpr uint8_t DISPLAY_ROTATION = 2;
constexpr uint8_t MODE_INFO_Y = SETTING_AREA_H;
constexpr uint8_t SETTING_AREA_Y = 0;
constexpr int8_t ENCODER_DIRECTION = 1;
constexpr uint8_t ENCODER_COUNTS_PER_INCREMENT = 4;
#endif

constexpr uint8_t VL53_VALID_MIN_MM = 60;
constexpr uint16_t VL53_VALID_MAX_MM = 600;
constexpr uint16_t SENSOR_LED_MAX_MM = 588;
constexpr uint16_t SENSOR_ACTIVE_MAX_MM = VL53_VALID_MIN_MM + (((SENSOR_LED_MAX_MM - VL53_VALID_MIN_MM) * 9) / 10);
constexpr uint8_t SENSOR_NOTE_FAR_TRIM_PCT = 10;
constexpr uint8_t SENSOR_NOTE_CLOSE_TRIM_PCT = 4;
constexpr uint16_t SENSOR_POLL_MS = 20;
constexpr uint32_t SENSOR_TIMEOUT_MS = 300;
constexpr uint32_t SENSOR_LOOP_REARM_DEBOUNCE_MS = 50UL;
constexpr uint16_t PUSH_POLL_MS = 20;
constexpr uint16_t PUSH_RAW_NEAR = 150;   // closest press (max effect)
constexpr uint16_t PUSH_RAW_FAR = 682;    // lightest press that should still register
constexpr uint16_t PUSH_RAW_OFF = 1023;   // no-touch/off region starts here
constexpr uint8_t PUSH_CURVE_POWER = 2;   // >1 makes light presses less aggressive

constexpr uint32_t SCREEN_SAVER_REFRESH_MS = 4000UL;
constexpr uint8_t SCREEN_SAVER_CANCEL_SLOT = 11;
constexpr uint32_t LONG_HOLD_PANIC_MS = 2000UL;
constexpr uint32_t BUTTON_DEBOUNCE_MS = 25UL;

constexpr uint8_t MAX_HELD_NOTES = 32;
constexpr uint8_t MAX_ARP_OUTPUT_NOTES = 8;
constexpr uint16_t MAX_PARAMETER_LOCKS = 256;
constexpr uint8_t PRESET_COUNT = 16;
constexpr uint8_t DIV_NOTE_SLOT_COUNT = 15;
constexpr uint8_t DIV_NOTE_PLUS_SLOT = DIV_NOTE_SLOT_COUNT;
constexpr uint8_t DIV_NOTE_RESET_SLOT = DIV_NOTE_SLOT_COUNT + 1;
constexpr uint8_t DIV_NOTE_BACK_SLOT = DIV_NOTE_SLOT_COUNT + 2;
constexpr uint8_t RND_RBN_CH10_TO_1_SLOT = 16;
constexpr uint8_t RND_RBN_CH10_TO_2_SLOT = 17;
constexpr uint8_t RND_RBN_RANDOM_SLOT = 18;
constexpr uint8_t RND_RBN_CLEAR_SLOT = 19;
constexpr uint8_t RND_RBN_BACK_SLOT = 20;
constexpr uint8_t ROUTER_CLEAR_SLOT = 16;
constexpr uint8_t ROUTER_BACK_SLOT = 17;
constexpr int8_t ROUTER_TRANSPOSE_MIN = -24;
constexpr int8_t ROUTER_TRANSPOSE_MAX = 24;
constexpr uint8_t ROUND_ROBIN_CH10_TO_1_BIT = 0x01;
constexpr uint8_t ROUND_ROBIN_CH10_TO_2_BIT = 0x02;
constexpr uint8_t ROUND_ROBIN_RANDOM_BIT = 0x04;
constexpr uint8_t LOOP_TRACK_SOURCE_BASE = 240;
constexpr uint8_t STUTTER_SOURCE_BASE = 224;
constexpr uint8_t HISTORY_OUTPUT_TARGET_BASE = 16;
constexpr uint32_t ARP_KEY_SYNC_CAPTURE_MS = 6UL;
constexpr uint32_t UI_RESUME_MAGIC = 0x41524D44UL;  // "ARMD"
// All persistent state lives in the filesystem. There is no second store: the
// RP2040 has no real EEPROM, and its emulation rewrites a whole 4 KB flash
// sector for any change, so a two-byte screen memory cost as much as a preset.
//
// Firmware 3 is still prototype firmware, so an incompatible layout change
// deliberately receives a new schema identity instead of carrying migration
// code. A mismatch installs all factory presets. Increment this value whenever
// the persisted layout or meaning changes.
constexpr uint32_t DEVICE_STATE_MAGIC = 0xF3090101UL;
constexpr uint8_t MAX_CUSTOM_ARP_EVENTS = 32;
constexpr uint32_t LOOP_FILE_MAGIC = 0x4C503304UL;  // "LP3" file, schema 4
constexpr uint8_t DRUM_AFTERTOUCH_MIN_VELOCITY = 42;  // 33% floor.

decltype(Serial2) &DinSerial = Serial2;
struct InterBrainSerialSettings : public midi::DefaultSerialSettings {
  static const long BaudRate = INTER_BRAIN_MIDI_BAUD;
};
MIDI_CREATE_CUSTOM_INSTANCE(HardwareSerial, Serial2, DinMIDI,
                            InterBrainSerialSettings);
bool core1_separate_stack = true;
Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire1, -1);
VL53L0X tof;
#if ARPNMIDI_ENABLE_RGB_LED
Adafruit_NeoPixel onboardRgb(1, PIN_RGB_LED, NEO_GRB + NEO_KHZ800);
#endif
#if ARPNMIDI_ENABLE_USB_DEVICE_MIDI
Adafruit_USBD_MIDI usbDeviceMidi;
#endif

void showBootStage(const __FlashStringHelper *line1,
                   const __FlashStringHelper *line2 = nullptr);
void storeUiResumeHint(uint8_t settingId);
void restartArpTiming(bool immediate);
void renderDisplayIfNeeded();
void arpNoteOffs();
void drumArpNoteOffs();
bool saveStorage();
void saveStorageIfAuto();
bool writeDeviceStateHeader();
void stagePersistedUiSetting(uint8_t settingId);
void markLoopStorageDirty();
void markExtendedPresetDirty();
void onInputNote(uint8_t sourcePort, uint8_t channel1, uint8_t note, uint8_t velocity, bool on,
                 bool recordForLoop = true);
void routeIncomingChannelMessage(uint8_t sourcePort, uint8_t status, uint8_t data1, uint8_t data2);
void loopAllOff();
void tickLooper();
void clearSavedLoopStorage();
void saveLoopStorageIfAny();
void loadSavedLoopStorage();
bool initializeExtendedPresetStorage(bool forceFactoryDefaults);
bool savePresetLearnedContent(uint8_t slot);
void noteThrough(uint8_t sourcePort, uint8_t inNote, uint8_t velocity, bool on);
void noteArpOffPassthrough(uint8_t sourcePort, uint8_t inNote, uint8_t velocity, bool on);
void thruOutputRefOn(uint8_t sourcePort, uint8_t outNote, uint8_t velocity);
void thruOutputRefOff(uint8_t sourcePort, uint8_t outNote);
void updateBassVoice();
void syncMusicalClockConfig(bool resetPhase = false);
void handleRealtimeByte(uint8_t sourcePort, uint8_t status);
uint8_t nextRoundRobinChannel(uint8_t baseCh);
void handleMultitrackRecPlay();
void handleMultitrackStopDelete();
void releaseMultitrackOutput(void *context, uint8_t track);
void emitMultitrackEvent(void *context, uint8_t track, const arpnmidi3::LoopMidiEvent &event);
void refreshLoopUiState();
void tickTimeTravelImport();
void emitEchoEvent(void *context, uint8_t target, const arpnmidi3::LoopMidiEvent &event);
void emitStutterEvent(void *context, uint8_t historyTarget,
                      const arpnmidi3::LoopMidiEvent &event);
void emitNoteLengthEvent(void *context, uint8_t target, uint8_t sourcePort,
                         const arpnmidi3::LoopMidiEvent &event);
void deactivateStutter(uint8_t target);
void requestStutterState(uint8_t target, bool enabled, int16_t lengthSelection = -1);
void clearSplitNoteFromMainPaths(uint8_t sourcePort, uint8_t note);
void setQuickJumpEnabled(bool enabled);
uint8_t currentDivisionSetting();
void captureChordMemoryOutput(uint8_t sourcePort, uint8_t channel,
                              uint8_t note, uint8_t velocity);
void finishChordMemoryLearnIfReady();
void handleMmcCommand(uint8_t command);

enum MenuMode : uint8_t {
  MENU_SELECT = 0,
  MENU_EDIT = 1
};

enum RouterEditStage : uint8_t {
  ROUTER_STAGE_LIST = 0,
  ROUTER_STAGE_DEST,
  ROUTER_STAGE_LOW_NOTE,
  ROUTER_STAGE_HIGH_NOTE,
  ROUTER_STAGE_TRANSPOSE
};

enum FeaturesUiStage : uint8_t {
  FEATURES_UI_GROUPS = 0,
  FEATURES_UI_KNOBS,
  FEATURES_UI_BUTTONS
};

enum CcRemapUiStage : uint8_t {
  CC_REMAP_UI_LIST = 0,
  CC_REMAP_UI_INPUT,
  CC_REMAP_UI_OUTPUT_CHANNEL,
  CC_REMAP_UI_OUTPUT_CC
};

enum NoteCcUiStage : uint8_t {
  NOTE_CC_UI_LIST = 0,
  NOTE_CC_UI_SLOT_ACTION,
  NOTE_CC_UI_INPUT_CHANNEL,
  NOTE_CC_UI_INPUT_NOTE,
  NOTE_CC_UI_OUTPUT_CHANNEL,
  NOTE_CC_UI_OUTPUT_CC,
  NOTE_CC_UI_BEHAVIOR
};

enum FourButtonUiStage : uint8_t {
  FOUR_BUTTON_UI_MAIN = 0,
  FOUR_BUTTON_UI_MODE,
  FOUR_BUTTON_UI_CUSTOM_LIST,
  FOUR_BUTTON_UI_CUSTOM_CHANNEL,
  FOUR_BUTTON_UI_CUSTOM_KIND,
  FOUR_BUTTON_UI_CUSTOM_NUMBER,
  FOUR_BUTTON_UI_CUSTOM_BEHAVIOR,
  FOUR_BUTTON_UI_LOOPER,
  FOUR_BUTTON_UI_CHORD
};

struct SubmenuUiState {
  uint8_t cursor = 0;
  bool editing = false;
};

enum SettingId : uint8_t {
  SET_BPM = 0,
  SET_SWING,
  SET_QUICK_JUMP,
  SET_STUTTER,
  SET_ECHO,
  SET_ARP_MODE,
  SET_LIVE_VELOCITY,
  SET_LIVE_NOTE_LENGTH,
  SET_DIVISION,
  SET_VELOCITY,
  SET_LENGTH,
  SET_INPUT_CH,
  SET_ARP_OUT_CH,
  SET_DRUM_MAGIC,
  SET_DIV_NOTES,
  SET_BASS_CH,
  SET_LEGATO_CH,
  SET_THRU_OUT_CH,
  SET_RND_RBN,
  SET_ROUTER,
  SET_MAP_CC,
  SET_CC_MAP,
  SET_NOTE_CC,
  SET_CC_OUT_CH,
  SET_SCREEN_SAVER,
  SET_SENSOR_CH,
  SET_SENSOR_MODE,
  SET_PUSH_MODE,
  SET_FOUR_BUTTON,
  SET_LOOP_BARS,
  SET_MUTE_SOLO,
  SET_PARAMETER_LOCK,
  SET_CHORD,
  SET_FORCE_KEY,
  SET_FORCE_SCALE,
  SET_GUITAR_PIANO,
  SET_LIVE_CC,
  SET_GLOBAL,
  SET_LOAD_PRESET,
  SET_SAVE_PRESET,
  SET_PANIC,
  SETTING_COUNT
};

enum ArpMode : uint8_t {
  ARP_UP = 0,
  ARP_DOWN,
  ARP_UPDOWN1,
  ARP_UPDOWN2,
  ARP_TRIGGER,
  ARP_RANDOM,
  ARP_OFF,
  ARP_MODE_COUNT
};

enum ArpSelection : uint8_t {
  ARPSEL_OFF = 0,
  ARPSEL_UP,
  ARPSEL_DOWN,
  ARPSEL_UPDOWN1,
  ARPSEL_UPDOWN2,
  ARPSEL_TRIGGER,
  ARPSEL_RANDOM,
  ARPSEL_PAT_UP_1OCT,
  ARPSEL_PAT_RHYTHM,
  ARPSEL_PAT_OSTINATO,
  ARPSEL_PAT_OCTAVE_WALK,
  ARPSEL_PAT_FIFTH,
  ARPSEL_PAT_BASS_CHORD,
  ARPSEL_PAT_CHORD_RUN,
  ARPSEL_CUSTOM,
  ARP_SELECTION_COUNT
};

enum DivisionId : uint8_t {
  DIV_1_1 = 0,
  DIV_1_2D,
  DIV_1_2,
  DIV_1_4D,
  DIV_1_2T,
  DIV_1_4,
  DIV_1_8D,
  DIV_1_4T,
  DIV_1_8,
  DIV_1_16D,
  DIV_1_8T,
  DIV_1_16,
  DIV_1_32D,
  DIV_1_16T,
  DIV_1_32,
  DIV_1_64D,
  DIV_1_32T,
  DIV_1_64,
  DIV_1_64T,
  DIVISION_COUNT
};

constexpr uint8_t ARP_DIVISION_FOLLOW_DRUM = DIVISION_COUNT;
constexpr uint8_t DRUM_DIVISION_FOLLOW_ARP = DIVISION_COUNT;
constexpr uint8_t DRUM_DIVISION_FREE = DIVISION_COUNT + 1;
constexpr uint8_t LIVE_TARGET_COUNT = 5;  // Main plus Looper Tracks 1-4.
// Stutter and Echo get a sixth target, SELECTD, beyond the five above: it
// dynamically mirrors whichever loop track is currently selected, using its
// own independent settings and runtime state rather than sharing that
// track's own fixed slot. Velocity and Note Length do not have this option
// and stay bound to LIVE_TARGET_COUNT, untouched.
constexpr uint8_t SELECTD_LIVE_TARGET = LIVE_TARGET_COUNT;            // = 5
constexpr uint8_t STUTTER_ECHO_TARGET_COUNT = LIVE_TARGET_COUNT + 1;  // = 6
// Off, then every straight/dotted/triplet division from 1/4 down through
// 1/64T, the same order and range the main Division list uses on its short
// end, just without the 1/2 and 1/1 the looper never needed anything looser
// than a quarter note for.
constexpr uint8_t LOOP_QUANTIZE_DIVISION_COUNT = DIVISION_COUNT - DIV_1_4;  // 14
constexpr uint8_t STUTTER_BUTTON_DIVISION_COUNT = 6;
// Stutter shares the rolling capture engine with Time Travel. Its long choices
// are meter-aware bars, followed by the ordinary musical divisions from long
// to short so a mapped knob moves from 8 bars toward 1/64T.
constexpr uint8_t STUTTER_BAR_LENGTH_COUNT = 4;
constexpr uint8_t STUTTER_LENGTH_DIVISION_BASE = STUTTER_BAR_LENGTH_COUNT;
constexpr uint8_t STUTTER_LENGTH_COUNT =
    STUTTER_LENGTH_DIVISION_BASE + DIVISION_COUNT;
constexpr uint8_t STUTTER_LENGTH_DEFAULT =
    STUTTER_LENGTH_DIVISION_BASE + DIV_1_4;
// Stutter's own repeat size never needs anything longer than a single bar;
// only Echo's fixed-length repeats still reach the full multi-bar range.
// "1 BAR" (index 3 in the shared length list, the last bar option before the
// ordinary divisions begin) is Stutter's largest choice.
constexpr uint8_t STUTTER_LENGTH_MIN_SELECTION = STUTTER_BAR_LENGTH_COUNT - 1;
constexpr uint8_t CC_REMAP_SLOT_COUNT = 16;
constexpr uint8_t NOTE_CC_SLOT_COUNT = 16;
constexpr uint8_t NOTE_CC_CANCEL_CHANNEL = 17;
constexpr uint8_t NOTE_CC_CANCEL_VALUE = 128;
constexpr uint8_t NOTE_CC_CANCEL_BEHAVIOR = 2;
constexpr uint8_t FOUR_BUTTON_CANCEL_CHANNEL = 17;
constexpr uint8_t FOUR_BUTTON_CANCEL_KIND = 2;
constexpr uint8_t FOUR_BUTTON_CANCEL_NUMBER = 128;
constexpr uint8_t FOUR_BUTTON_CANCEL_BEHAVIOR = 3;
constexpr uint8_t DIRECT_CANCEL_CHANNEL = 17;
constexpr uint8_t DIRECT_CANCEL_CC_CHANNEL = 18;
constexpr uint8_t BASS_CANCEL_CHANNEL = 13;
constexpr uint8_t BASS_CANCEL_OCTAVE = 4;
constexpr uint8_t BASS_CANCEL_HIGH_NOTE = 128;
constexpr uint8_t FOUR_BUTTON_CUSTOM_DONE_SLOT = 4;
constexpr uint8_t FOUR_BUTTON_CUSTOM_BACK_SLOT = 5;
constexpr uint8_t FOUR_BUTTON_LOOPER_DONE_SLOT = 6;
constexpr uint8_t FOUR_BUTTON_LOOPER_BACK_SLOT = 7;
constexpr uint8_t FOUR_BUTTON_CHORD_DONE_SLOT = 2;
constexpr uint8_t FOUR_BUTTON_CHORD_BACK_SLOT = 3;

enum FeatureKnobId : uint8_t {
  FEATURE_KNOB_VELOCITY_BASE = 0,
  FEATURE_KNOB_NOTE_LENGTH_BASE = FEATURE_KNOB_VELOCITY_BASE + LIVE_TARGET_COUNT,
  FEATURE_KNOB_STUTTER_BASE = FEATURE_KNOB_NOTE_LENGTH_BASE + LIVE_TARGET_COUNT,
  FEATURE_KNOB_ECHO_WET_BASE = FEATURE_KNOB_STUTTER_BASE + LIVE_TARGET_COUNT,
  FEATURE_KNOB_ECHO_LENGTH_BASE = FEATURE_KNOB_ECHO_WET_BASE + LIVE_TARGET_COUNT,
  FEATURE_KNOB_ECHO_DELAY_BASE = FEATURE_KNOB_ECHO_LENGTH_BASE + LIVE_TARGET_COUNT,
  FEATURE_KNOB_ECHO_DRIFT_BASE = FEATURE_KNOB_ECHO_DELAY_BASE + LIVE_TARGET_COUNT,
  FEATURE_KNOB_ARP_DIVISION = FEATURE_KNOB_ECHO_DRIFT_BASE + LIVE_TARGET_COUNT,
  FEATURE_KNOB_DRUM_DIVISION,
  FEATURE_KNOB_QUICK_JUMP_INPUT,
  FEATURE_KNOB_QUICK_JUMP_OUTPUT,
  FEATURE_KNOB_BPM,
  FEATURE_KNOB_SWING,
  FEATURE_KNOB_ARP_MODE,
  FEATURE_KNOB_ARP_VELOCITY,
  FEATURE_KNOB_ARP_LENGTH,
  FEATURE_KNOB_ARP_OCTAVES,
  FEATURE_KNOB_LOOP_TRACK,
  FEATURE_KNOB_LOOP_LENGTH,
  FEATURE_KNOB_COUNT
};

enum FeatureButtonId : uint8_t {
  FEATURE_BUTTON_VELOCITY_BASE = 0,
  FEATURE_BUTTON_NOTE_LENGTH_BASE = FEATURE_BUTTON_VELOCITY_BASE + LIVE_TARGET_COUNT,
  FEATURE_BUTTON_STUTTER_BASE = FEATURE_BUTTON_NOTE_LENGTH_BASE + LIVE_TARGET_COUNT,
  FEATURE_BUTTON_ECHO_BASE = FEATURE_BUTTON_STUTTER_BASE + LIVE_TARGET_COUNT,
  FEATURE_BUTTON_LOOP_RECORD = FEATURE_BUTTON_ECHO_BASE + LIVE_TARGET_COUNT,
  FEATURE_BUTTON_LOOP_PLAY_STOP,
  FEATURE_BUTTON_LOOP_CLEAR_UNDO,
  FEATURE_BUTTON_LOOP_COMBO,
  FEATURE_BUTTON_TRACK_SELECT_BASE,
  FEATURE_BUTTON_TRACK_MUTE_BASE = FEATURE_BUTTON_TRACK_SELECT_BASE + LIVE_TARGET_COUNT - 1,
  FEATURE_BUTTON_TRACK_SOLO_BASE = FEATURE_BUTTON_TRACK_MUTE_BASE + LIVE_TARGET_COUNT - 1,
  FEATURE_BUTTON_QUICK_JUMP = FEATURE_BUTTON_TRACK_SOLO_BASE + LIVE_TARGET_COUNT - 1,
  FEATURE_BUTTON_STUTTER_DIV_BASE,
  FEATURE_BUTTON_STUTTER_DIV_END = FEATURE_BUTTON_STUTTER_DIV_BASE +
      LIVE_TARGET_COUNT * STUTTER_BUTTON_DIVISION_COUNT,
  FEATURE_BUTTON_ARP_RETRIGGER = FEATURE_BUTTON_STUTTER_DIV_END,
  FEATURE_BUTTON_ARP_NOTE_ORDER,
  FEATURE_BUTTON_DRUM_MAGIC,
  FEATURE_BUTTON_DRUM_AFTERTOUCH_VELOCITY,
  FEATURE_BUTTON_CHORD,
  FEATURE_BUTTON_LOOP_AUTO_REC,
  FEATURE_BUTTON_LOOP_TIME_TRAVEL,
  FEATURE_BUTTON_LOOP_RECORD_CC,
  FEATURE_BUTTON_LOOP_MIDI_TRANSPORT,
  FEATURE_BUTTON_CLOCK_INPUT,
  FEATURE_BUTTON_CLOCK_OUTPUT,
  FEATURE_BUTTON_PANIC,
  FEATURE_BUTTON_COUNT
};

enum TriggerBindingKind : uint8_t {
  TRIGGER_BINDING_OFF = 0,
  TRIGGER_BINDING_CC,
  TRIGGER_BINDING_NOTE
};

enum FourButtonMode : uint8_t {
  FOUR_BUTTON_CUSTOM = 0,
  FOUR_BUTTON_LOOPER,
  FOUR_BUTTON_CHORD_MEMORY,
  FOUR_BUTTON_MODE_COUNT
};

enum CustomButtonBehavior : uint8_t {
  CUSTOM_BUTTON_MOMENTARY = 0,
  CUSTOM_BUTTON_LATCH,
  CUSTOM_BUTTON_FLAPPY,
  CUSTOM_BUTTON_BEHAVIOR_COUNT
};

enum NoteCcBehavior : uint8_t {
  NOTE_CC_MOMENTARY = 0,
  NOTE_CC_TOGGLE
};

constexpr uint8_t LOOPER_BUTTON_SELECT = 0x01;
constexpr uint8_t LOOPER_BUTTON_MUTE = 0x02;
constexpr uint8_t LOOPER_BUTTON_SOLO = 0x04;
constexpr uint8_t LOOPER_BUTTON_DELETE = 0x08;
constexpr uint8_t LOOPER_BUTTON_UNDO = 0x10;
constexpr uint8_t LOOPER_BUTTON_ARM = 0x20;
// Two master rec/play triggers inside this window read as one double gesture.
constexpr uint32_t LOOP_MASTER_DOUBLE_TAP_MS = 1000UL;
// A looper button keeps stepping through its enabled actions only while it is
// tapped repeatedly. This is how long one gesture stays open.
constexpr uint32_t LOOPER_BUTTON_CYCLE_MS = 1500;

struct FeatureKnobBinding {
  uint8_t channel = 0;
  uint8_t cc = 0xFF;
};

struct FeatureButtonBinding {
  uint8_t channel = 0;
  uint8_t number = 0xFF;
  uint8_t kind = TRIGGER_BINDING_OFF;
};

struct CcRemapEntry {
  uint8_t inputCc = 0xFF;
  uint8_t outputChannel = 1;
  uint8_t outputCc = 0;
};

struct CustomButtonConfig {
  uint8_t channel = 1;
  uint8_t number = 60;
  uint8_t kind = TRIGGER_BINDING_NOTE;
  uint8_t behavior = CUSTOM_BUTTON_MOMENTARY;
};

struct ChordMemorySlot {
  uint8_t count = 0;
  uint8_t channels[16]{};
  uint8_t notes[16]{};
  uint8_t velocities[16]{};
};

struct NoteCcMapEntry {
  uint8_t inputChannel = 0;
  uint8_t inputNote = 0xFF;
  uint8_t outputChannel = 1;
  uint8_t outputCc = 0;
  uint8_t behavior = NOTE_CC_MOMENTARY;
};

struct FeatureControlSettings {
  FeatureKnobBinding knobs[FEATURE_KNOB_COUNT];
  FeatureButtonBinding buttons[FEATURE_BUTTON_COUNT];
  CcRemapEntry ccRemaps[CC_REMAP_SLOT_COUNT];
  uint8_t drumRollKinds[DIV_NOTE_SLOT_COUNT];
  uint8_t fourButtonMode = FOUR_BUTTON_CUSTOM;
  uint8_t looperButtonActions = LOOPER_BUTTON_SELECT;
  CustomButtonConfig customButtons[4];
  ChordMemorySlot chordMemories[4];
  NoteCcMapEntry noteCcMaps[NOTE_CC_SLOT_COUNT];
};

struct LiveTargetSettings {
  uint8_t velocityEnabled;
  uint8_t velocityPercent;
  uint8_t noteLengthEnabled;
  uint8_t noteLengthPercent;
  uint8_t stutterEnabled;
  uint8_t stutterLengthSelection;
  uint8_t echoEnabled;
  uint8_t echoWet;
  uint8_t echoLength;
  uint8_t echoDelay;
  int8_t echoDrift;
};

// The as-loaded/as-saved baseline for every field a mapped Feature Knob can
// touch, so a hold-to-panic or a PANIC screen click can put all of them back
// exactly where the preset left them, undoing whatever a connected
// controller's last CC value temporarily set there, without needing the
// controller itself to move. Captured fresh on every preset load and every
// save, so it always reflects the most recent thing actually on flash.
struct FeatureKnobDefaults {
  uint8_t division = 0;
  uint8_t drumDivision = 0;
  uint8_t quickJumpInputChannel = 1;
  uint8_t quickJumpOutputChannel = 2;
  uint16_t manualBpm = 120;
  uint8_t swing = 0;
  uint8_t arpMode = 0;
  uint8_t arpVelocity = 100;
  uint8_t arpLengthPct = 50;
  uint8_t arpOctaves = 1;
  LiveTargetSettings liveTargets[STUTTER_ECHO_TARGET_COUNT]{};
};
FeatureKnobDefaults featureKnobDefaults;

enum PatternId : uint8_t {
  PAT_MODE = 0,
  PAT_UP_1OCT,
  PAT_DOWN,
  PAT_UPDOWN1,
  PAT_UPDOWN2,
  PAT_RANDOM,
  PAT_TRIGGER,
  PAT_RHYTHM,
  PAT_OSTINATO,
  PAT_OCTAVE_WALK,
  PAT_FIFTH,
  PAT_BASS_CHORD,
  PAT_CHORD_RUN,
  PATTERN_COUNT
};

enum ForceScaleId : uint8_t {
  SCALE_OFF = 0,
  SCALE_MAJOR,
  SCALE_MINOR,
  SCALE_MAJOR_MINOR,
  SCALE_BLUES,
  SCALE_MAJOR_BLUES,
  SCALE_BLUES_BOTH,
  SCALE_HARM_MINOR,
  SCALE_MELODIC_MINOR,
  SCALE_USER,
  FORCE_SCALE_COUNT
};

enum SensorModeId : uint8_t {
  SENSOR_OFF = 0,
  SENSOR_PARAM_PLUS2,
  SENSOR_PARAM_MINUS2,
  SENSOR_PARAM_PLUS3,
  SENSOR_PARAM_MINUS3,
  SENSOR_PARAM_FULL,
  SENSOR_DIV3,
  SENSOR_VEL_DOWN,
  SENSOR_LEN_DOWN,
  SENSOR_ARP_LATCH,
  SENSOR_ARP_LATCH_PLUS,
  SENSOR_ARP_FREEZE,
  SENSOR_ARP_FREEZ_PLUS,
  SENSOR_PITCH_UP,
  SENSOR_PITCH_DOWN,
  SENSOR_NOTES_C0,
  SENSOR_NOTES_C1,
  SENSOR_NOTES_C2,
  SENSOR_NOTES_C3,
  SENSOR_NOTES_C4,
  SENSOR_NOTES_C5,
  SENSOR_NOTES_C6,
  SENSOR_NOTES_C7,
  SENSOR_CC1,
  SENSOR_CC2,
  SENSOR_CC3,
  SENSOR_CC4,
  SENSOR_CC5,
  SENSOR_CC6,
  SENSOR_CC7,
  SENSOR_CC8,
  SENSOR_CC9,
  SENSOR_CC10,
  SENSOR_CC11,
  SENSOR_CC12,
  SENSOR_CC13,
  SENSOR_CC14,
  SENSOR_CC15,
  SENSOR_CC16,
  SENSOR_CC17,
  SENSOR_CC18,
  SENSOR_CC19,
  SENSOR_LOOP_TRIGGER,
  SENSOR_LOOP_REC_PLAY,
  SENSOR_LOOP_STOP_DELETE,
  SENSOR_MODE_COUNT
};

enum TokenType : int8_t {
  TOK_REST = -3,
  TOK_ALL = -2,
  TOK_MODE = -1
};

struct PatternToken {
  int8_t noteIndex;
  int8_t octaveOffset;
  int8_t semitoneOffset;
};

struct Settings {
  uint16_t manualBpm;
  uint8_t arpMode;
  uint8_t division;
  uint8_t arpVelocity;
  uint8_t arpLengthPct;
  uint8_t inputChannel;
  uint8_t arpOutChannel;
  uint8_t bassMode;
  uint8_t thruOutChannel;
  uint16_t roundRobinMask;
  uint16_t routerActiveMask;
  uint8_t routerOutChannels[16];
  int8_t routerTranspose[16];
  uint8_t ccOutChannel;
  uint8_t sensorChannel;
  uint8_t sensorMode;
  uint8_t forceKey;
  uint8_t forceScale;
  uint8_t instrumentView;
  uint8_t loadPreset;
  uint8_t savePreset;
  uint8_t reserved;
  uint8_t screenSaver;
  uint8_t divNoteChannels[DIV_NOTE_SLOT_COUNT];
  uint8_t divNoteNotes[DIV_NOTE_SLOT_COUNT];
  uint8_t divNotePlusNote;
  uint8_t pushMode;
  uint8_t legatoChannel;
  uint8_t roundRobinOptions;
};

struct Firmware3Settings {
  uint8_t clockInFollow;
  uint8_t clockOutSend;
  uint8_t timeSignature;
  uint8_t swing;
  uint8_t looperMidiTransport;
  uint8_t looperAutoRec;
  uint8_t looperTimeTravel;
  uint8_t looperTrackMode;
  uint8_t looperQuantize[arpnmidi3::kLoopTrackCount];
  uint8_t looperRecordCc;
  uint8_t stutterTimeoutBars;
  uint8_t arpOctaves;
  uint8_t arpRetriggerSync;
  uint8_t arpNoteOrder;
  uint8_t customArpLength;
  uint8_t drumEnabled;
  uint8_t drumInputMode;
  uint8_t drumOutputChannel;
  uint8_t drumSplitNote;
  uint8_t drumMappedStart;
  uint8_t drumAftertouchVelocity;
  uint8_t drumDivision;
  uint8_t quickJumpEnabled;
  uint8_t quickJumpInputChannel;
  uint8_t quickJumpOutputChannel;
  uint8_t quickJumpHold;
  uint8_t bassHighestNote;
  uint8_t parameterLockChannel;
  uint8_t forwardChannelAftertouch;
  uint8_t forwardPolyAftertouch;
  uint8_t channelAftertouchCc;
  uint8_t mainAftertouchArpVelocity;
  uint8_t chordEnabled;
  int8_t chordPositions[4];
  uint16_t userScaleMask;
  uint8_t routerLowNotes[16];
  uint8_t routerHighNotes[16];
  LiveTargetSettings liveTargets[STUTTER_ECHO_TARGET_COUNT];
};

// The file is a small header followed by one complete record per preset slot.
// Nothing about a preset is split across two stores any more.
struct DeviceStateHeader {
  uint32_t magic = DEVICE_STATE_MAGIC;
  uint16_t recordSize = 0;
  uint8_t presetCount = PRESET_COUNT;
  uint8_t currentPreset = 0;
  uint8_t autoSave = 1;
  uint8_t lastScreen = 0xFF;
  uint8_t reserved[2] = {0, 0};
};

struct LoopFileTrack {
  uint32_t lengthUs = 0;
  uint32_t storedLengthUs = 0;
  uint32_t generation = 0;
  uint8_t flags = 0;
  uint8_t lengthSelection = 2;
  // Where this track's cycle begins inside the shared transport cycle, stored
  // as a fraction of its own length so the field fits the space the format
  // already reserved. An older file reads as zero, which is the top of the
  // cycle and matches how those loops used to restart.
  uint16_t startPhase = 0;
};
static_assert(sizeof(LoopFileTrack) == 16, "Loop file track layout changed");

struct LoopFileHeader {
  uint32_t magic = LOOP_FILE_MAGIC;
  uint16_t eventCount = 0;
  uint8_t selectedTrack = 0;
  uint8_t trackMode = 0;
  uint32_t checksum = 2166136261UL;
  LoopFileTrack tracks[arpnmidi3::kLoopTrackCount];
};

struct LoopFileEvent {
  uint32_t atUs = 0;
  uint8_t track = 0;
  uint8_t status = 0;
  uint8_t data1 = 0;
  uint8_t data2 = 0;
};

// The header is written raw at the front of the state file, so its layout is
// pinned. Any change to it needs a new schema identity.
static_assert(sizeof(DeviceStateHeader) == 12, "Device state header layout changed");

struct LoopCcPruneState {
  bool used = false;
  bool pending = false;
  uint8_t channel = 1;
  uint8_t cc = 0;
  uint8_t lastStored = 0;
  uint8_t lastSeen = 0;
  uint8_t filtered = 0;
  int8_t direction = 0;
  uint32_t lastStoredMs = 0;
  uint32_t lastSeenMs = 0;
};

struct TimeTravelImportJob {
  arpnmidi3::HistorySnapshot snapshot{};
  uint64_t boundaryUs = 0;
  uint32_t lengthUs = 0;
  uint16_t closeScan = 0;
  uint16_t importedEvents = 0;
  uint8_t track = 0;
  bool active = false;
  bool closingNotes = false;
  bool wasPlaying = false;
  bool overflowed = false;
  uint8_t heldNotes[16][16]{};
};

struct ParameterLockEntry {
  uint8_t note = 0;
  uint8_t cc = 0;
  uint8_t value = 0;
};

struct CustomArpEvent {
  uint16_t startPulse = 0;
  uint16_t gatePulses = 1;
  int8_t pitchOffset = 0;
  uint8_t velocity = 96;
};

struct CustomArpPattern {
  uint8_t count = 0;
  uint8_t lengthSelection = 2;
  CustomArpEvent events[MAX_CUSTOM_ARP_EVENTS];
};

struct CustomArpVoice {
  bool active = false;
  uint8_t note = 0;
  uint8_t channel = 1;
  uint64_t offUs = 0;
};

// Everything one preset slot owns, in one record.
struct PresetRecord {
  Settings settings;
  Firmware3Settings firmware3;
  FeatureControlSettings featureControls;
  CustomArpPattern customArp;
  uint16_t parameterLockCount = 0;
  ParameterLockEntry parameterLocks[MAX_PARAMETER_LOCKS];
};

struct EncoderState {
  uint8_t lastAB = 0;
  int8_t stepAccum = 0;
  bool lastSwitch = true;
  bool switchDown = false;
  bool turnWhilePressed = false;
  uint32_t lastTurnMs = 0;
  uint32_t switchIgnoreUntilMs = 0;
  uint32_t switchChangeMs = 0;
  uint32_t pressStartMs = 0;
};

struct SensorRuntime {
  bool ready = false;
  bool present = false;
  uint16_t mm = 0;
  bool inRange = false;
  uint32_t lastSeenMs = 0;
  uint32_t lastPollMs = 0;
  int8_t activeNote = -1;
  int16_t lastPitch = 0;
  int16_t lastCcValue = -1;
};

struct PushRuntime {
  uint16_t raw = 0;
  uint8_t pct = 0;
  bool inRange = false;
  uint32_t lastPollMs = 0;
  int8_t activeNote = -1;
  int16_t lastPitch = 0;
  int16_t lastCcValue = -1;
};

struct UiState {
  volatile bool dirty = true;
  bool inSaver = false;
  bool swallowWakeInput = false;
  bool hasPendingEdit = false;
  bool deferredExitWork = false;
  bool deferredLoadPreset = false;
  bool deferredSaveOnly = false;
  uint32_t lastActivityMs = 0;
  uint32_t lastRenderMs = 0;
  MenuMode menuMode = MENU_SELECT;
  uint8_t selectedSetting = 0;
  uint8_t pendingSetting = 0;
  int16_t pendingValue = 0;
};

volatile bool uiBusyRequest = false;
volatile bool uiBusyShown = false;
DeviceStateHeader storage;
bool storageError = false;
bool factoryResetRequested = false;
Settings settings;
Firmware3Settings firmware3Settings;
FeatureControlSettings featureControls;
arpnmidi3::ClockEngine musicalClock;
EncoderState encoder;
SensorRuntime sensorRt;
PushRuntime pushRt;
UiState ui;
uint8_t persistedUiSetting = SET_BPM;
// The remembered screen is a convenience, not something the performer made,
// so it never forces itself in ahead of real content the way a menu commit
// does. It waits for the screen itself to sit still, not just the engine, so
// browsing through several screens in a row does not attempt a write after
// every stop along the way, only once the performer has actually settled.
uint8_t observedUiSetting = SET_BPM;
bool uiScreenSavePending = false;
uint32_t uiScreenChangedMs = 0;
constexpr uint32_t UI_SCREEN_SAVE_IDLE_MS = 5000UL;
constexpr uint32_t UI_MIN_FRAME_MS = 50;
// LOOPER and LOOP MIX show state that changes with almost every click while
// the performer is actively working the loop, select, arm, mute, solo, clear,
// undo, all one dirty screen apiece. A push to this panel is a full-frame,
// blocking I2C transfer that runs about 20ms at 400kHz regardless of how
// little changed, and it holds the outgoing MIDI drain, on this same core,
// off the wire for the entire transfer. A flurry of clicks on an ordinary
// screen already only pushes one frame every 50ms; these two screens get a
// longer window so more of that flurry coalesces into one push, trading a
// slightly less instant icon for a looper that stays on its own time.
constexpr uint32_t UI_LOOPER_SCREEN_MIN_FRAME_MS = 300;
constexpr uint32_t RENDER_STARVED_MS = 100;

uint32_t uiMinFrameIntervalMs() {
  return (ui.selectedSetting == SET_LOOP_BARS || ui.selectedSetting == SET_MUTE_SOLO)
      ? UI_LOOPER_SCREEN_MIN_FRAME_MS : UI_MIN_FRAME_MS;
}
// A failed flash write waits this long before another attempt. Retrying every
// loop pass would grind the whole instrument, which is far worse than data
// waiting a few extra seconds.
uint32_t storageRetryHoldUntilMs = 0;
// Scheduler health, measured instead of guessed. Worst loop pass and worst
// step lateness over the previous one-second window, shown on the PANIC
// diagnostics screen.
uint32_t perfWindowStartMs = 0;
uint32_t perfLoopMaxUs = 0;
uint32_t perfLoopMaxUsShown = 0;
uint32_t perfLateMaxUs = 0;
uint32_t perfLateMaxUsShown = 0;
bool presetStorageDirty = false;
uint32_t presetStorageDirtyMs = 0;
uint32_t tapTempoLastMs = 0;
uint32_t tapTempoIntervals[4];
uint8_t tapTempoIntervalCount = 0;
uint8_t tapTempoIntervalCursor = 0;
uint32_t tapTempoVisibleUntilMs = 0;

uint8_t drumAftertouchPressure = 127;
uint8_t mainAftertouchPressure = 127;
ParameterLockEntry parameterLocks[MAX_PARAMETER_LOCKS];
uint16_t parameterLockCount = 0;
uint32_t parameterLockOverflowCount = 0;
bool parameterLockHeldNotes[128];
CustomArpPattern customArpPattern;
bool customArpLearning = false;
bool customArpWaitingForFirstNote = false;
uint64_t customArpLearnStartUs = 0;
uint64_t customArpLearnEndUs = 0;
uint8_t customArpLearnLowestNote = 127;
int8_t customArpLearnActiveEvent[128];
CustomArpVoice customArpVoices[64];
uint64_t customArpCycleStartUs = 0;
uint8_t customArpPlayIndex = 0;
bool littleFsReady = false;
arpnmidi3::FourTrackLooper multitrackLooper;
arpnmidi3::RollingHistory rollingHistory;
arpnmidi3::EchoEngine echoEngine;
arpnmidi3::NoteLengthEngine noteLengthEngine;
arpnmidi3::HistoryRepeater stutterRepeaters[STUTTER_ECHO_TARGET_COUNT];
TimeTravelImportJob timeTravelImport;
uint8_t finalOutputNoteRefs[STUTTER_ECHO_TARGET_COUNT][16][128];
bool stutterSettingWasEnabled[STUTTER_ECHO_TARGET_COUNT];
bool stutterTimedOut[STUTTER_ECHO_TARGET_COUNT];
bool noteLengthSettingWasEnabled[LIVE_TARGET_COUNT];
bool echoSettingWasEnabled[STUTTER_ECHO_TARGET_COUNT];
bool featureButtonCcHeld[FEATURE_BUTTON_COUNT];
uint8_t activeStutterLengthSelection[STUTTER_ECHO_TARGET_COUNT];
uint64_t stutterStopUs[STUTTER_ECHO_TARGET_COUNT];
uint8_t multitrackPlaybackHeld[arpnmidi3::kLoopTrackCount][16][16];
uint16_t multitrackPlaybackHeldCount[arpnmidi3::kLoopTrackCount];
bool loopStorageDirty = false;
uint32_t loopStorageDirtyMs = 0;
uint32_t loopMasterLastTriggerMs = 0;
bool loopStorageError = false;
uint8_t loopTrackLengthSelection[arpnmidi3::kLoopTrackCount] = {2, 3, 4, 5};
LoopCcPruneState loopCcPrune[64];
uint32_t loopCcPruneOverflowCount = 0;

bool heldInputNotes[128];
uint8_t heldVelocities[128];
uint32_t heldNoteOrder[128];
uint32_t heldNoteOrderCounter = 0;
bool physicalHeldInputNotes[128];
uint8_t physicalHeldVelocities[128];
bool loopHeldInputNotes[128];
uint8_t loopHeldVelocities[128];
bool loopTrackHeldInputNotes[arpnmidi3::kLoopTrackCount][128];
uint8_t loopTrackHeldVelocities[arpnmidi3::kLoopTrackCount][128];
bool arpLatchedNotes[128];
uint8_t arpLatchedVelocities[128];
bool thruLatchedNotes[128];
bool heldDrumNotes[128];
uint8_t heldDrumVelocities[128];
bool physicalHeldDrumNotes[128];
uint8_t physicalHeldDrumVelocities[128];
bool loopHeldDrumNotes[128];
uint8_t loopHeldDrumVelocities[128];
bool loopTrackHeldDrumNotes[arpnmidi3::kLoopTrackCount][128];
uint8_t loopTrackHeldDrumVelocities[arpnmidi3::kLoopTrackCount][128];
uint8_t mappedThruNotes[128];
uint8_t mappedLoopThruNotes[arpnmidi3::kLoopTrackCount][128];
uint8_t mappedThruChordNotes[128][3];
uint8_t mappedLoopThruChordNotes[arpnmidi3::kLoopTrackCount][128][3];
uint8_t mappedThruChordCount[128];
uint8_t mappedLoopThruChordCount[arpnmidi3::kLoopTrackCount][128];
uint8_t mappedArpOffNotes[128];
uint8_t mappedArpOffChannels[128];
uint8_t mappedLoopArpOffNotes[arpnmidi3::kLoopTrackCount][128];
uint8_t mappedLoopArpOffChannels[arpnmidi3::kLoopTrackCount][128];
uint8_t thruOutputRefCount[128];
uint8_t thruOutputRefChannel[128];
uint8_t arpOffOutputRefCount[128];
uint8_t arpOffOutputChannel[128];
uint8_t legatoHeldCount[128];
uint8_t legatoHeldVelocity[128];
uint8_t legatoHeldSource[128];
uint32_t legatoHeldOrder[128];
uint32_t legatoOrderCounter = 0;
bool legatoOutputActive = false;
uint8_t legatoOutputNote = 0xFF;
uint8_t legatoOutputVelocity = 0;
uint8_t legatoOutputSource = 255;
uint8_t legatoOutputChannel = 0;
uint8_t heldSorted[MAX_HELD_NOTES];
uint8_t heldCount = 0;
uint8_t arpHeldSorted[MAX_HELD_NOTES];
uint8_t arpHeldCount = 0;
uint8_t heldDrumCount = 0;
int8_t currentBassSource = -1;
int8_t currentBassOutNote = -1;
int8_t activeArpNotes[MAX_ARP_OUTPUT_NOTES];
uint8_t activeArpChannels[MAX_ARP_OUTPUT_NOTES];
uint8_t activeArpCount = 0;
uint8_t roundRobinCursor = 0;
uint8_t roundRobinMenuCursor = 0;
uint8_t routerMenuCursor = 0;
uint8_t routerEditChannel = 0;
uint8_t routerEditStage = ROUTER_STAGE_LIST;
int8_t activeDrumArpNotes[MAX_HELD_NOTES];
uint8_t activeDrumArpCount = 0;
uint64_t arpNextStepUs = 0;
uint32_t arpGateOffMs = 0;
uint64_t arpGridOriginUs = 0;
uint64_t drumNextStepUs = 0;
uint32_t drumGateOffMs = 0;
uint64_t drumGridOriginUs = 0;
uint32_t drumGlobalStep = 0;
// arpGlobalStep is a position in musical time: which grid boundary the next
// step lands on. It is recomputed whenever the grid moves, which happens on
// every division change, and a drum roll changes the division constantly.
//
// arpSequenceStep is a position in the arpeggio: how many steps have actually
// been played. Note order and octave come from this one, so re-gridding changes
// when the next note happens without changing which note it is. Sharing one
// counter for both is what made Up sound random while rolling drums.
uint32_t arpGlobalStep = 0;
uint32_t arpSequenceStep = 0;
uint8_t arpPatternStep = 0;
bool arpHadKeys = false;
bool arpLatchAwaitingNewPhrase = false;
bool arpLatchSensorClose = false;
bool arpLatchPushClose = false;
bool arpFreezeSensorClose = false;
bool arpFreezePushClose = false;
bool arpFreezeActive = false;
bool arpFreezePlusActive = false;
bool arpFrozenNotes[128];
bool thruFrozenNotes[128];
uint8_t thruFrozenMappedNotes[128];
uint8_t divNotesCursor = 0;
bool divNoteHeld[DIV_NOTE_SLOT_COUNT];
bool physicalDivNoteHeld[DIV_NOTE_SLOT_COUNT];
bool loopDivNoteHeld[DIV_NOTE_SLOT_COUNT];
uint32_t divNoteHeldStamp[DIV_NOTE_SLOT_COUNT];
uint32_t divNotePressCounter = 0;
uint8_t featuresUiStage = FEATURES_UI_GROUPS;
uint8_t featuresGroupCursor = 0;
uint8_t featuresItemCursor = 0;
// The list is the browsing view; opening one item shows the same detail
// view this screen always has, closing again the moment the encoder turns.
bool featuresItemOpen = false;
bool featuresLearnActive = false;
uint8_t ccRemapUiStage = CC_REMAP_UI_LIST;
uint8_t ccRemapCursor = 0;
bool ccRemapLearnActive = false;
uint8_t noteCcUiStage = NOTE_CC_UI_LIST;
uint8_t noteCcCursor = 0;
uint8_t noteCcSlotActionCursor = 0;
bool noteCcLearnActive = false;
bool noteCcToggleState[NOTE_CC_SLOT_COUNT];
NoteCcMapEntry noteCcEditBackup{};
uint8_t muteSoloCursor = 0;
// Loop Mix applies one action to whichever track is picked. The action itself
// is a mode chosen from the bottom row.
enum LoopMixMode : uint8_t {
  LOOP_MIX_SOLO = 0,
  LOOP_MIX_MUTE,
  LOOP_MIX_CLEAR,
  LOOP_MIX_ARM,
  LOOP_MIX_MODE_COUNT
};
constexpr uint8_t LOOP_MIX_MODE_BASE = arpnmidi3::kLoopTrackCount;
constexpr uint8_t LOOP_MIX_BACK_SLOT = LOOP_MIX_MODE_BASE + LOOP_MIX_MODE_COUNT;
// Arm is what the screen is reached for most, so it is where the screen opens.
uint8_t loopMixMode = LOOP_MIX_ARM;
uint8_t loopMixLastClickedMode = 0xFF;
uint32_t loopMixModeClickMs = 0;
uint8_t loopMixLastClickedTrack = 0xFF;
uint32_t loopMixTrackClickMs = 0;
constexpr uint32_t LOOP_MIX_DOUBLE_CLICK_MS = 700UL;
SubmenuUiState arpMenuUi;
SubmenuUiState liveVelocityUi;
SubmenuUiState liveNoteLengthUi;
SubmenuUiState stutterUi;
SubmenuUiState echoUi;
SubmenuUiState quickJumpUi;
SubmenuUiState drumMagicUi;
SubmenuUiState bassUi;
SubmenuUiState looperSettingsUi;
SubmenuUiState parameterLockUi;
SubmenuUiState chordUi;
SubmenuUiState scaleUi;
SubmenuUiState globalUi;
bool submenuEditBackupValid = false;
uint8_t submenuEditBackupSetting = 0;
uint8_t submenuEditBackupCursor = 0;
int16_t submenuEditBackupValue = 0;
bool editCancelSelected = false;
uint8_t editCancelSetting = 0;
uint8_t editCancelCursor = 0;
uint8_t liveVelocityTarget = 0;
uint8_t liveNoteLengthTarget = 0;
uint8_t stutterTarget = 0;
uint8_t echoTarget = 0;
uint8_t liveCcCursor = 0;
uint8_t liveCcNumber = 1;
uint8_t liveCcValue = 0;
bool liveCcEditing = false;
uint8_t fourButtonUiStage = FOUR_BUTTON_UI_MAIN;
uint8_t fourButtonUiCursor = 0;
uint8_t fourButtonEditButton = 0;
bool fourButtonLearnActive = false;
CustomButtonConfig fourButtonEditBackup{};
bool physicalButtonState[4];
uint32_t physicalButtonChangeMs[4];
bool customButtonLatch[4];
uint8_t customButtonFlappyValue[4];
uint32_t customButtonFlappyMs[4];
uint8_t looperButtonStep[4];
uint8_t lastLooperButton = 0xFF;
uint32_t looperButtonLastMs = 0;
uint32_t looperScreenClickMs = 0;
bool chordButtonPlaying[4];
bool chordLearnArmed = false;
bool chordClearArmed = false;
bool chordLearnActive = false;
uint8_t chordLearnSlot = 0;
bool extendedPresetDirty = false;
uint32_t extendedPresetDirtyMs = 0;
bool screenSaverSeeded = false;
bool screenSaverForceNow = false;
uint8_t screenSaverCursor = 0;
uint32_t panicConfirmedUntilMs = 0;
uint8_t usbSysexBuffer[16];
uint8_t usbSysexLength = 0;
volatile bool mainSetupComplete = false;
volatile bool secondaryTxQueueEnabled = false;
uint32_t sensorHardwareLastPollMs = 0;
volatile uint32_t sensorSampleSequence = 0;
volatile uint16_t sensorSampleMm = 0;
volatile bool sensorSampleTimedOut = false;
uint32_t consumedSensorSampleSequence = 0;

struct SecondaryMidiTxMessage {
  uint8_t status = 0;
  uint8_t data1 = 0;
  uint8_t data2 = 0;
  uint8_t length = 0;
};

constexpr uint16_t SECONDARY_TX_QUEUE_CAPACITY = 256;
SecondaryMidiTxMessage secondaryTxQueue[SECONDARY_TX_QUEUE_CAPACITY];
volatile uint16_t secondaryTxHead = 0;
volatile uint16_t secondaryTxTail = 0;
volatile uint16_t secondaryTxHighWater = 0;
volatile uint32_t secondaryTxDropped = 0;
volatile uint32_t secondaryTxCriticalDropped = 0;
volatile uint32_t secondaryTxSent = 0;
volatile uint32_t dinIncomingMessageCount = 0;
volatile uint32_t usbIncomingMessageCount = 0;
volatile uint8_t lastIncomingSource = 0xFF;
volatile uint8_t lastIncomingStatus = 0;
volatile uint8_t lastIncomingData1 = 0;
volatile uint8_t lastIncomingData2 = 0;

bool loopSafeClearArmed = false;
uint32_t sensorLoopReleaseStartMs = 0;
uint32_t pushLoopReleaseStartMs = 0;
const int8_t kEncoderTransitionTable[16] = {
  0,  1, -1,  0,
 -1,  0,  0,  1,
  1,  0,  0, -1,
  0, -1,  1,  0
};

const char *const kSettingNames[SETTING_COUNT] = {
  "1 BPM", "2 SWING", "3 QUICK JUMP", "4 STUTTER", "5 ECHO", "6 ARP",
  "7 VELOCITY", "8 NOTELENGT", "", "", "", "9 MAIN INPUT", "10 ARP OUT", "11 DRUMROLL",
  "12 DRUMDIV", "13 BASS", "14 MONO", "15 THRU OUT", "16 RNDRBN", "17 ROUTER",
  "18 FEATURES", "19 CC MAP", "20 NOTE>CC", "21 IN CC >", "22 SCRNSVR",
  "23 EYE/PUSH", "24 EYE MODE", "25 PUSH", "26 4BUTTON", "27 LOOPER",
  "28 LOOP MIX", "29 PLOCK", "30 CHORD", "31 KEY", "32 SCALE",
  "33 GIT/KEYS", "34 LIVE CC", "35 GLOBAL", "36 LOAD", "37 SAVE", "38 PANIC"
};

const char *const kArpModeNames[ARP_MODE_COUNT] = {
  "UP", "DOWN", "UP-DOWN 1", "UP-DOWN 2", "TRIGGER", "RANDOM", "OFF"
};

const char *const kArpSelectionNames[ARP_SELECTION_COUNT] = {
  "OFF", "UP", "DOWN", "UP-DOWN 1", "UP-DOWN 2", "TRIGGER", "RANDOM",
  "UP 1-OCT",
  "RHYTHM", "OSTINATO", "OCT WALK", "FIFTH", "BASS+CHORD", "CHORD+RUN", "CUSTOM"
};

const char *const kDivisionNames[DIVISION_COUNT] = {
  "1/1", "1/2D", "1/2", "1/4D", "1/2T", "1/4", "1/8D",
  "1/4T", "1/8", "1/16D", "1/8T", "1/16", "1/32D", "1/16T",
  "1/32", "1/64D", "1/32T", "1/64", "1/64T"
};

const float kDivisionQuarterSteps[DIVISION_COUNT] = {
  4.0f, 3.0f, 2.0f, 1.5f, 4.0f / 3.0f, 1.0f, 0.75f,
  2.0f / 3.0f, 0.5f, 0.375f, 1.0f / 3.0f, 0.25f, 0.1875f,
  1.0f / 6.0f, 0.125f, 0.09375f, 1.0f / 12.0f, 0.0625f, 1.0f / 24.0f
};

constexpr uint16_t MUSICAL_PPQN = arpnmidi3::kInternalPpqn;
const uint16_t kDivisionPulseSteps[DIVISION_COUNT] = {
  384, 288, 192, 144, 128, 96, 72, 64, 48, 36,
  32, 24, 18, 16, 12, 9, 8, 6, 4
};

const char *const kForceScaleNames[FORCE_SCALE_COUNT] = {
  "OFF", "MAJOR", "MINOR", "MAJ+MIN", "BLUES", "MAJ BLUES",
  "BLUES+BOTH", "HARM MIN", "MEL MIN", "USER"
};

const char *const kSensorModeNames[SENSOR_MODE_COUNT] = {
  "OFF", "DIV +2", "DIV -2", "DIV +3", "DIV -3", "DIV FULL", "DIV3",
  "VEL DOWN", "LEN DOWN", "ARP LATCH", "ARP LATCH+", "ARP FREEZE", "ARP FREEZ+",
  "PITCH UP", "PITCH DOWN", "NOTES C0", "NOTES C1", "NOTES C2", "NOTES C3",
  "NOTES C4", "NOTES C5", "NOTES C6", "NOTES C7",
  "CC 1", "CC 2", "CC 3", "CC 4", "CC 5", "CC 6", "CC 7", "CC 8", "CC 9",
  "CC 10", "CC 11", "CC 12", "CC 13", "CC 14", "CC 15", "CC 16", "CC 17", "CC 18", "CC 19",
  "CC103", "Loop Rec/\nPlay/Over", "Loop Stop/\nDelete"
};

const char *const kNoteNames[12] = {
  "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"
};

const uint8_t kMajor[7] = {0, 2, 4, 5, 7, 9, 11};
const uint8_t kMinor[7] = {0, 2, 3, 5, 7, 8, 10};
const uint8_t kBlues[7] = {0, 3, 5, 6, 7, 10, 0};
const uint8_t kMajorBlues[7] = {0, 2, 3, 4, 7, 9, 0};
const uint8_t kHarmMinor[7] = {0, 2, 3, 5, 7, 8, 11};
const uint8_t kMelMinor[7] = {0, 2, 3, 5, 7, 9, 11};

const PatternToken kPatterns[PATTERN_COUNT][16] = {
  { {TOK_MODE, 0, 0}, {TOK_MODE, 0, 0}, {TOK_MODE, 0, 0}, {TOK_MODE, 0, 0},
    {TOK_MODE, 0, 0}, {TOK_MODE, 0, 0}, {TOK_MODE, 0, 0}, {TOK_MODE, 0, 0},
    {TOK_MODE, 0, 0}, {TOK_MODE, 0, 0}, {TOK_MODE, 0, 0}, {TOK_MODE, 0, 0},
    {TOK_MODE, 0, 0}, {TOK_MODE, 0, 0}, {TOK_MODE, 0, 0}, {TOK_MODE, 0, 0} },
  { {0,0,0},{1,0,0},{2,0,0},{3,0,0},{0,0,0},{1,0,0},{2,0,0},{3,0,0},
    {0,0,0},{1,0,0},{2,0,0},{3,0,0},{0,0,0},{1,0,0},{2,0,0},{3,0,0} },
  { {3,0,0},{2,0,0},{1,0,0},{0,0,0},{3,0,0},{2,0,0},{1,0,0},{0,0,0},
    {3,0,0},{2,0,0},{1,0,0},{0,0,0},{3,0,0},{2,0,0},{1,0,0},{0,0,0} },
  { {0,0,0},{1,0,0},{2,0,0},{3,0,0},{2,0,0},{1,0,0},{0,0,0},{1,0,0},
    {2,0,0},{3,0,0},{2,0,0},{1,0,0},{0,0,0},{1,0,0},{2,0,0},{3,0,0} },
  { {0,0,0},{1,0,0},{2,0,0},{3,0,0},{3,0,0},{2,0,0},{1,0,0},{0,0,0},
    {0,0,0},{1,0,0},{2,0,0},{3,0,0},{3,0,0},{2,0,0},{1,0,0},{0,0,0} },
  { {TOK_MODE,0,0},{TOK_MODE,0,0},{TOK_MODE,0,0},{TOK_MODE,0,0},
    {TOK_MODE,0,0},{TOK_MODE,0,0},{TOK_MODE,0,0},{TOK_MODE,0,0},
    {TOK_MODE,0,0},{TOK_MODE,0,0},{TOK_MODE,0,0},{TOK_MODE,0,0},
    {TOK_MODE,0,0},{TOK_MODE,0,0},{TOK_MODE,0,0},{TOK_MODE,0,0} },
  { {TOK_ALL,0,0},{TOK_ALL,0,0},{TOK_ALL,0,0},{TOK_ALL,0,0},
    {TOK_ALL,0,0},{TOK_ALL,0,0},{TOK_ALL,0,0},{TOK_ALL,0,0},
    {TOK_ALL,0,0},{TOK_ALL,0,0},{TOK_ALL,0,0},{TOK_ALL,0,0},
    {TOK_ALL,0,0},{TOK_ALL,0,0},{TOK_ALL,0,0},{TOK_ALL,0,0} },
  { {0,0,0},{TOK_REST,0,0},{0,0,0},{TOK_REST,0,0},{1,0,0},{TOK_REST,0,0},{1,0,0},{TOK_REST,0,0},
    {0,0,0},{TOK_REST,0,0},{2,0,0},{TOK_REST,0,0},{0,0,0},{TOK_REST,0,0},{3,0,0},{TOK_REST,0,0} },
  { {0,0,0},{1,0,0},{2,0,0},{0,0,0},{1,0,0},{2,0,0},{0,0,0},{1,0,0},
    {2,0,0},{0,0,0},{1,0,0},{2,0,0},{0,0,0},{1,0,0},{2,0,0},{0,0,0} },
  { {0,0,0},{0,1,0},{0,2,0},{0,3,0},{1,0,0},{1,1,0},{1,2,0},{1,3,0},
    {2,0,0},{2,1,0},{2,2,0},{2,3,0},{3,0,0},{3,1,0},{3,2,0},{3,3,0} },
  { {0,0,0},{0,0,7},{1,0,0},{1,0,7},{2,0,0},{2,0,7},{3,0,0},{3,0,7},
    {0,0,0},{0,0,7},{1,0,0},{1,0,7},{2,0,0},{2,0,7},{3,0,0},{3,0,7} },
  { {0,0,0},{TOK_ALL,0,0},{0,0,0},{TOK_ALL,0,0},{1,0,0},{TOK_ALL,0,0},{1,0,0},{TOK_ALL,0,0},
    {2,0,0},{TOK_ALL,0,0},{2,0,0},{TOK_ALL,0,0},{3,0,0},{TOK_ALL,0,0},{3,0,0},{TOK_ALL,0,0} },
  { {TOK_ALL,0,0},{0,0,0},{1,0,0},{2,0,0},{3,0,0},{TOK_ALL,0,0},{3,0,0},{2,0,0},
    {1,0,0},{0,0,0},{TOK_ALL,0,0},{0,0,0},{1,0,0},{2,0,0},{3,0,0},{TOK_ALL,0,0} }
};

uint8_t wrapIndex(int value, uint8_t count) {
  while (value < 0) value += count;
  while (value >= count) value -= count;
  return static_cast<uint8_t>(value);
}

uint8_t clampU8(int value, int lo, int hi) {
  if (value < lo) return static_cast<uint8_t>(lo);
  if (value > hi) return static_cast<uint8_t>(hi);
  return static_cast<uint8_t>(value);
}

void markActivity(bool visible = true) {
  ui.lastActivityMs = millis();
  if (visible) {
    ui.inSaver = false;
    ui.dirty = true;
  }
}

bool wakeFromSaverIfNeeded() {
  if (!ui.inSaver && !screenSaverForceNow) return false;
  screenSaverForceNow = false;
  ui.inSaver = false;
  ui.swallowWakeInput = true;
  markActivity();
  return true;
}

bool channelEnabled(uint8_t chSetting) {
  return chSetting >= 1 && chSetting <= 16;
}

uint16_t channelBit(uint8_t ch1) {
  return channelEnabled(ch1) ? static_cast<uint16_t>(1U << (ch1 - 1)) : 0;
}

bool roundRobinCh10To1Enabled(const Settings &s) {
  return (s.roundRobinOptions & ROUND_ROBIN_CH10_TO_1_BIT) != 0;
}

bool roundRobinCh10To1Enabled() {
  return roundRobinCh10To1Enabled(settings);
}

bool roundRobinCh10To2Enabled(const Settings &s) {
  return (s.roundRobinOptions & ROUND_ROBIN_CH10_TO_2_BIT) != 0;
}

bool roundRobinCh10To2Enabled() {
  return roundRobinCh10To2Enabled(settings);
}

bool roundRobinRandomEnabled(const Settings &s) {
  return (s.roundRobinOptions & ROUND_ROBIN_RANDOM_BIT) != 0;
}

bool roundRobinRandomEnabled() {
  return roundRobinRandomEnabled(settings);
}

void setRoundRobinCh10To1(Settings &s, bool enabled) {
  if (enabled) {
    s.roundRobinOptions |= ROUND_ROBIN_CH10_TO_1_BIT;
    s.roundRobinOptions &= static_cast<uint8_t>(~ROUND_ROBIN_CH10_TO_2_BIT);
  } else {
    s.roundRobinOptions &= static_cast<uint8_t>(~ROUND_ROBIN_CH10_TO_1_BIT);
  }
}

void setRoundRobinCh10To2(Settings &s, bool enabled) {
  if (enabled) {
    s.roundRobinOptions |= ROUND_ROBIN_CH10_TO_2_BIT;
    s.roundRobinOptions &= static_cast<uint8_t>(~ROUND_ROBIN_CH10_TO_1_BIT);
  } else {
    s.roundRobinOptions &= static_cast<uint8_t>(~ROUND_ROBIN_CH10_TO_2_BIT);
  }
}

void setRoundRobinRandom(Settings &s, bool enabled) {
  if (enabled) s.roundRobinOptions |= ROUND_ROBIN_RANDOM_BIT;
  else s.roundRobinOptions &= static_cast<uint8_t>(~ROUND_ROBIN_RANDOM_BIT);
}

void updateRouterActiveBit(Settings &s, uint8_t idx) {
  if (idx >= 16) return;
  const bool active = (s.routerOutChannels[idx] != idx + 1) || (s.routerTranspose[idx] != 0);
  if (active) s.routerActiveMask |= static_cast<uint16_t>(1U << idx);
  else s.routerActiveMask &= static_cast<uint16_t>(~static_cast<uint16_t>(1U << idx));
}

void clearRouterMappings(Settings &s) {
  s.routerActiveMask = 0;
  for (uint8_t i = 0; i < 16; ++i) {
    s.routerOutChannels[i] = i + 1;
    s.routerTranspose[i] = 0;
  }
}

bool applyRouterToChannelMessage(uint8_t &status, uint8_t &data1) {
  if (settings.routerActiveMask == 0 || status >= 0xF0) return true;
  const uint8_t ch1 = (status & 0x0F) + 1;
  const uint16_t bit = channelBit(ch1);
  if ((settings.routerActiveMask & bit) == 0) return true;

  const uint8_t idx = ch1 - 1;
  uint8_t outCh = settings.routerOutChannels[idx];
  if (!channelEnabled(outCh)) outCh = ch1;
  const uint8_t type = status & 0xF0;
  if (type == 0x80 || type == 0x90 || type == 0xA0) {
    if (data1 < firmware3Settings.routerLowNotes[idx] ||
        data1 > firmware3Settings.routerHighNotes[idx]) {
      return true;
    }
    const int outNote = static_cast<int>(data1) + settings.routerTranspose[idx];
    if (outNote < 0 || outNote > 127) return false;
    data1 = static_cast<uint8_t>(outNote);
  }
  status = type | ((outCh - 1) & 0x0F);
  return true;
}

bool arpChannelSpecialMode() {
  return firmware3Settings.drumEnabled != 0;
}

bool arpChannelAftertouchMode() {
  return arpChannelSpecialMode() && firmware3Settings.drumAftertouchVelocity != 0;
}

bool arpChannelSplitMode() {
  return arpChannelSpecialMode() && firmware3Settings.drumInputMode != 0;
}

uint8_t splitDrumStartNote() {
  return firmware3Settings.drumSplitNote;
}

uint8_t splitDrumOutputNote(uint8_t note) {
  const int mapped = static_cast<int>(firmware3Settings.drumMappedStart) +
                     static_cast<int>(note) - splitDrumStartNote();
  return clampU8(mapped, 0, 127);
}

uint8_t mainArpOutChannel() {
  return settings.arpOutChannel;
}

bool splitDrumInputNote(uint8_t note) {
  const uint8_t start = splitDrumStartNote();
  return note >= start && note < static_cast<uint8_t>(start + 8);
}

bool selectableSetting(uint8_t settingId) {
  return settingId != SET_DIVISION && settingId != SET_VELOCITY &&
         settingId != SET_LENGTH;
}

uint8_t advanceSelectableSetting(uint8_t current, int delta) {
  uint8_t next = current;
  do {
    int value = static_cast<int>(next) + delta;
    while (value < 0) value += SETTING_COUNT;
    while (value >= SETTING_COUNT) value -= SETTING_COUNT;
    next = static_cast<uint8_t>(value);
  } while (!selectableSetting(next));
  return next;
}

bool liveNoteViewActive() {
  return ui.selectedSetting == SET_FORCE_KEY ||
         ui.selectedSetting == SET_FORCE_SCALE ||
         ui.selectedSetting == SET_GUITAR_PIANO;
}

uint16_t currentBpm() {
  if (firmware3Settings.clockInFollow) {
    const float followed = musicalClock.bpm();
    return constrain(static_cast<int>(followed + 0.5f), 20, 300);
  }
  return constrain(static_cast<int>(settings.manualBpm), 20, 300);
}

arpnmidi3::ClockConfig currentClockConfig() {
  arpnmidi3::ClockConfig config;
  config.followExternal = firmware3Settings.clockInFollow != 0;
  config.sendClock = firmware3Settings.clockOutSend != 0;
  config.threeFour = firmware3Settings.timeSignature != 0;
  config.manualBpm = constrain(static_cast<int>(settings.manualBpm), 20, 300);
  return config;
}

void syncMusicalClockConfig(bool resetPhase) {
  const uint64_t nowUs = time_us_64();
  if (resetPhase) musicalClock.begin(currentClockConfig(), nowUs);
  else musicalClock.setConfig(currentClockConfig(), nowUs);
}

uint8_t midiMessageDataLength(uint8_t status) {
  if (status >= 0xF8) return 0;
  switch (status & 0xF0) {
    case 0x80:
    case 0x90:
    case 0xA0:
    case 0xB0:
    case 0xE0:
      return 2;
    case 0xC0:
    case 0xD0:
      return 1;
    default:
      return 0;
  }
}

uint8_t usbDeviceCinFromStatus(uint8_t status, uint8_t len) {
  if (status >= 0xF8) return 0x0F;

  if (status >= 0xF0) {
    switch (status) {
      case 0xF1: return 0x02;
      case 0xF2: return 0x03;
      case 0xF3: return 0x02;
      case 0xF6: return 0x05;
      default:   return (len == 1) ? 0x05 : ((len == 2) ? 0x06 : 0x04);
    }
  }

  switch (status & 0xF0) {
    case 0x80: return 0x08;
    case 0x90: return 0x09;
    case 0xA0: return 0x0A;
    case 0xB0: return 0x0B;
    case 0xC0: return 0x0C;
    case 0xD0: return 0x0D;
    case 0xE0: return 0x0E;
    default:   return 0x00;
  }
}

uint8_t usbDevicePacketDataLength(uint8_t cin) {
  switch (cin & 0x0F) {
    case 0x2:
    case 0x6:
    case 0xC:
    case 0xD:
      return 2;
    case 0x3:
    case 0x4:
    case 0x7:
    case 0x8:
    case 0x9:
    case 0xA:
    case 0xB:
    case 0xE:
      return 3;
    case 0x5:
    case 0xF:
      return 1;
    default:
      return 0;
  }
}

void sendMidiToUsbDevice(uint8_t sourcePort, uint8_t status, uint8_t data1, uint8_t data2) {
#if ARPNMIDI_ENABLE_USB_DEVICE_MIDI
  if (sourcePort == USB_DEVICE_SOURCE_PORT) return;
  const uint8_t len = (status >= 0xF8) ? 1 : (midiMessageDataLength(status) + 1);
  if (len == 0) return;
  uint8_t packet[4] = {
    usbDeviceCinFromStatus(status, len),
    status,
    data1,
    data2
  };
  usbDeviceMidi.writePacket(packet);
#else
  (void)sourcePort;
  (void)status;
  (void)data1;
  (void)data2;
#endif
}

uint16_t secondaryTxDepth() {
  const uint16_t head = secondaryTxHead;
  const uint16_t tail = secondaryTxTail;
  return head >= tail ? head - tail : SECONDARY_TX_QUEUE_CAPACITY - tail + head;
}

bool criticalSecondaryMessage(uint8_t status, uint8_t data1, uint8_t data2) {
  const uint8_t type = status & 0xF0;
  return type == 0x80 || (type == 0x90 && data2 == 0) ||
         (type == 0xB0 && (data1 == 120 || data1 == 123)) || status == 0xFC;
}

void queueSecondaryMidi(uint8_t status, uint8_t data1, uint8_t data2, uint8_t length) {
  if (!secondaryTxQueueEnabled) {
    DinSerial.write(status);
    if (length > 1) DinSerial.write(data1);
    if (length > 2) DinSerial.write(data2);
    return;
  }

  const bool critical = criticalSecondaryMessage(status, data1, data2);
  const uint16_t depth = secondaryTxDepth();
  if (!critical && depth >= SECONDARY_TX_QUEUE_CAPACITY - 32U) {
    ++secondaryTxDropped;
    return;
  }
  const uint16_t head = secondaryTxHead;
  const uint16_t next = (head + 1U) % SECONDARY_TX_QUEUE_CAPACITY;
  if (next == secondaryTxTail) {
    ++secondaryTxDropped;
    if (critical) ++secondaryTxCriticalDropped;
    return;
  }
  secondaryTxQueue[head] = SecondaryMidiTxMessage{status, data1, data2, length};
  __dmb();
  secondaryTxHead = next;
  const uint16_t queued = secondaryTxDepth();
  if (queued > secondaryTxHighWater) secondaryTxHighWater = queued;
}

void drainSecondaryMidiTx() {
  while (secondaryTxTail != secondaryTxHead) {
    const uint16_t tail = secondaryTxTail;
    const SecondaryMidiTxMessage message = secondaryTxQueue[tail];
    // Arduino-Pico's SerialUART::availableForWrite() is a writable flag
    // (0/1), not a byte-count. Do not compare it with the MIDI message
    // length or every 2/3-byte message will remain queued forever.
    if (DinSerial.availableForWrite() == 0) return;
    DinSerial.write(message.status);
    if (message.length > 1) DinSerial.write(message.data1);
    if (message.length > 2) DinSerial.write(message.data2);
    __dmb();
    secondaryTxTail = (tail + 1U) % SECONDARY_TX_QUEUE_CAPACITY;
    ++secondaryTxSent;
  }
}

void sendDinRaw1(uint8_t status) {
  queueSecondaryMidi(status, 0, 0, 1);
}

void sendDinRaw2(uint8_t status, uint8_t data1) {
  queueSecondaryMidi(status, data1, 0, 2);
}

uint64_t musicalDurationUs(uint64_t pulses) {
  return (pulses * 60000000ULL) /
         (static_cast<uint64_t>(currentBpm()) * MUSICAL_PPQN);
}

uint64_t divisionStepUs() {
  return musicalDurationUs(kDivisionPulseSteps[currentDivisionSetting()]);
}

bool divisionSupportsSwing(uint8_t division) {
  return division == DIV_1_8 || division == DIV_1_16 ||
         division == DIV_1_32 || division == DIV_1_64;
}

uint64_t swungGridOffsetUs(uint32_t step, uint8_t division) {
  const uint64_t straight = musicalDurationUs(
      static_cast<uint64_t>(step) * kDivisionPulseSteps[division]);
  if ((step & 1U) == 0 || firmware3Settings.swing == 0 ||
      !divisionSupportsSwing(division)) return straight;
  // Swing is an amount from straight (0) to a 75/25 pair (75).
  const uint64_t stepUs = musicalDurationUs(kDivisionPulseSteps[division]);
  return straight + ((stepUs * firmware3Settings.swing + 75ULL) / 150ULL);
}

uint64_t swungGridTimeUs(uint64_t originUs, uint32_t step, uint8_t division) {
  return originUs + swungGridOffsetUs(step, division);
}

void registerTapTempo() {
  const uint32_t now = millis();
  const uint32_t elapsed = now - tapTempoLastMs;
  if (tapTempoLastMs == 0 || elapsed > 3000) {
    tapTempoIntervalCount = 0;
    tapTempoIntervalCursor = 0;
  } else if (elapsed >= 200) {
    tapTempoIntervals[tapTempoIntervalCursor] = elapsed;
    tapTempoIntervalCursor = (tapTempoIntervalCursor + 1U) % 4U;
    if (tapTempoIntervalCount < 4) ++tapTempoIntervalCount;

    uint32_t totalMs = 0;
    for (uint8_t i = 0; i < tapTempoIntervalCount; ++i) totalMs += tapTempoIntervals[i];
    const uint32_t averageMs = max<uint32_t>(1, totalMs / tapTempoIntervalCount);
    settings.manualBpm = constrain(static_cast<int>((60000UL + averageMs / 2U) / averageMs), 20, 300);
    syncMusicalClockConfig(false);
  } else {
    return;
  }
  tapTempoLastMs = now;
  tapTempoVisibleUntilMs = now + 900;
  ui.dirty = true;
  markActivity(false);
}

uint32_t divisionStepMs() {
  return static_cast<uint32_t>((divisionStepUs() + 999ULL) / 1000ULL);
}

bool settingNeedsPanic(uint8_t settingId) {
  return settingId == SET_INPUT_CH || settingId == SET_ARP_OUT_CH ||
         settingId == SET_THRU_OUT_CH ||
         settingId == SET_LEGATO_CH ||
         settingId == SET_FORCE_KEY ||
         settingId == SET_SENSOR_CH || settingId == SET_SENSOR_MODE ||
         settingId == SET_PUSH_MODE;
}

String midiChannelLabel(uint8_t value, bool allowOff = false) {
  if (allowOff && value == 0) return "OFF";
  return "CH " + String(value);
}

String ccChannelLabel(uint8_t value) {
  if (value == 17) return "ALL3";
  return midiChannelLabel(value);
}

uint32_t screenSaverTimeoutMs(uint8_t selection) {
  static constexpr uint32_t timeouts[] = {
    0UL, 30000UL, 60000UL, 120000UL, 300000UL, 600000UL,
    900000UL, 1800000UL, 3600000UL, 10800000UL, 18000000UL
  };
  if (selection >= SCREEN_SAVER_CANCEL_SLOT) return 0;
  return timeouts[selection];
}

String screenSaverLabel(uint8_t selection) {
  static const char *const labels[] = {
    "OFF", "30S", "1 MIN", "2 MIN", "5 MIN", "10 MIN",
    "15 MIN", "30 MIN", "1 HR", "3 HR", "5 HR", "CANCEL"
  };
  return labels[clampU8(selection, 0, SCREEN_SAVER_CANCEL_SLOT)];
}

uint8_t divNoteSlotToDivision(uint8_t slot) {
  static constexpr uint8_t kDrumRollDivisions[DIV_NOTE_SLOT_COUNT] = {
    DIV_1_4D, DIV_1_4, DIV_1_4T,
    DIV_1_8D, DIV_1_8, DIV_1_8T,
    DIV_1_16D, DIV_1_16, DIV_1_16T,
    DIV_1_32D, DIV_1_32, DIV_1_32T,
    DIV_1_64D, DIV_1_64, DIV_1_64T
  };
  return kDrumRollDivisions[min<uint8_t>(slot, DIV_NOTE_SLOT_COUNT - 1)];
}

bool arpLatchEnabled() {
  return settings.sensorMode == SENSOR_ARP_LATCH ||
         settings.sensorMode == SENSOR_ARP_LATCH_PLUS ||
         settings.pushMode == SENSOR_ARP_LATCH ||
         settings.pushMode == SENSOR_ARP_LATCH_PLUS;
}

bool arpLatchPlusEnabled() {
  return settings.sensorMode == SENSOR_ARP_LATCH_PLUS ||
         settings.pushMode == SENSOR_ARP_LATCH_PLUS;
}

bool arpFreezeMode(uint8_t mode) {
  return mode == SENSOR_ARP_FREEZE || mode == SENSOR_ARP_FREEZ_PLUS;
}

bool arpFreezeEnabled() {
  return arpFreezeMode(settings.sensorMode) || arpFreezeMode(settings.pushMode);
}

bool arpFreezePlusEnabled() {
  return settings.sensorMode == SENSOR_ARP_FREEZ_PLUS ||
         settings.pushMode == SENSOR_ARP_FREEZ_PLUS;
}

bool sensorModeIsDivisionOverlay(uint8_t mode) {
  return mode == SENSOR_PARAM_PLUS2 || mode == SENSOR_PARAM_MINUS2 ||
         mode == SENSOR_PARAM_PLUS3 || mode == SENSOR_PARAM_MINUS3 ||
         mode == SENSOR_PARAM_FULL || mode == SENSOR_DIV3;
}

bool sensorModeIsNotes(uint8_t mode) {
  return mode >= SENSOR_NOTES_C0 && mode <= SENSOR_NOTES_C7;
}

bool sensorModeIsCc(uint8_t mode) {
  return mode >= SENSOR_CC1 && mode <= SENSOR_CC19;
}

bool sensorModeIsPitch(uint8_t mode) {
  return mode == SENSOR_PITCH_UP || mode == SENSOR_PITCH_DOWN;
}

bool ckeyEnabledForValue(uint8_t keyValue) {
  return keyValue >= 13 && keyValue <= 24;
}

bool ckeyEnabled() {
  return ckeyEnabledForValue(settings.forceKey);
}

uint8_t rootPcFromKeyValue(uint8_t keyValue) {
  if (keyValue == 0) return 0;
  if (ckeyEnabledForValue(keyValue)) return keyValue - 13;
  return keyValue - 1;
}

bool scaleIsCombo(uint8_t scale) {
  return scale == SCALE_MAJOR_MINOR || scale == SCALE_BLUES_BOTH;
}

uint8_t effectiveScaleForKeyMode(uint8_t keyValue, uint8_t scaleValue) {
  if (!ckeyEnabledForValue(keyValue)) return scaleValue;
  if (!scaleIsCombo(scaleValue)) return scaleValue;
  return SCALE_MAJOR;
}

void clearFreezeState() {
  for (uint8_t note = 0; note < 128; ++note) {
    if (arpFrozenNotes[note]) {
      noteArpOffPassthrough(255, note, 0, false);
      noteThrough(255, note, 0, false);
    }
    if (!thruFrozenNotes[note]) continue;
    const uint8_t out = thruFrozenMappedNotes[note];
    if (out <= 127) thruOutputRefOff(255, out);
  }
  memset(arpFrozenNotes, 0, sizeof(arpFrozenNotes));
  memset(thruFrozenNotes, 0, sizeof(thruFrozenNotes));
  memset(thruFrozenMappedNotes, 0xFF, sizeof(thruFrozenMappedNotes));
  arpFreezeActive = false;
  arpFreezePlusActive = false;
  arpFreezeSensorClose = false;
  arpFreezePushClose = false;
}

void clearArpLatchNotes() {
  for (uint8_t note = 0; note < 128; ++note) {
    if (thruLatchedNotes[note]) {
      noteThrough(255, note, 0, false);
      thruLatchedNotes[note] = false;
    }
    if (arpLatchedNotes[note]) {
      noteArpOffPassthrough(255, note, 0, false);
      noteThrough(255, note, 0, false);
    }
  }
  memset(arpLatchedNotes, 0, sizeof(arpLatchedNotes));
  memset(arpLatchedVelocities, 0, sizeof(arpLatchedVelocities));
  arpLatchAwaitingNewPhrase = false;
  arpHeldCount = 0;
  arpNoteOffs();
  updateBassVoice();
}

uint8_t bassModeChannel(uint8_t value) {
  return (value == 0) ? 0 : (((value - 1) / 4) + 1);
}

int8_t bassModeOctaveOffset(uint8_t value) {
  if (value == 0) return 0;
  switch ((value - 1) % 4) {
    case 0: return -2;
    case 1: return -1;
    case 2: return 0;
    default: return 1;
  }
}

uint8_t bassModeFromChannelOctave(uint8_t channel, int8_t octaves) {
  if (channel == 0) return 0;
  channel = clampU8(channel, 1, 12);
  octaves = constrain(octaves, -2, 1);
  return static_cast<uint8_t>((channel - 1U) * 4U + (octaves + 2) + 1U);
}

String bassLabel(uint8_t value) {
  if (value == 0) return "OFF";
  const uint8_t channel = bassModeChannel(value);
  const int8_t octaves = bassModeOctaveOffset(value);
  if (octaves > 0) return "CH " + String(channel) + " +" + String(octaves) + " OCT";
  if (octaves == 0) return "CH " + String(channel) + " 0 OCT";
  return "CH " + String(channel) + " " + String(octaves) + " OCT";
}

void drawWrappedTopValue(const String &text) {
  display.setTextSize(2);
  String lines[3];
  uint8_t lineCount = 0;
  String remaining = text;

  while (remaining.length() && lineCount < 3) {
    int forcedBreak = remaining.indexOf('\n');
    if (forcedBreak >= 0) {
      lines[lineCount++] = remaining.substring(0, forcedBreak);
      remaining = remaining.substring(forcedBreak + 1);
      continue;
    }

    int split = -1;
    int lastSpace = -1;
    for (size_t i = 0; i < remaining.length(); ++i) {
      if (remaining[i] == ' ') lastSpace = i;
      String candidate = remaining.substring(0, i + 1);
      int16_t x1, y1;
      uint16_t w, h;
      display.getTextBounds(candidate, 0, 0, &x1, &y1, &w, &h);
      if (w > SCREEN_W - 2) {
        split = (lastSpace >= 0) ? lastSpace : (i - 1);
        break;
      }
    }

    if (split < 0) {
      lines[lineCount++] = remaining;
      break;
    }

    if (split <= 0) split = 1;
    lines[lineCount++] = remaining.substring(0, split);
    remaining = remaining.substring(split);
    while (remaining.length() && remaining[0] == ' ') remaining.remove(0, 1);
  }

  const int topY = (lineCount > 1) ? 4 : 12;
  for (uint8_t i = 0; i < lineCount; ++i) {
    display.setCursor(0, topY + i * 16);
    display.print(lines[i]);
  }
}

void sendDinNoteOn(uint8_t channel1, uint8_t note, uint8_t velocity) {
  queueSecondaryMidi(static_cast<uint8_t>(0x90 | ((channel1 - 1U) & 0x0F)),
                     note, velocity, 3);
}

void sendDinNoteOff(uint8_t channel1, uint8_t note, uint8_t velocity = 0) {
  queueSecondaryMidi(static_cast<uint8_t>(0x80 | ((channel1 - 1U) & 0x0F)),
                     note, velocity, 3);
}

void sendDinCc(uint8_t channel1, uint8_t cc, uint8_t value) {
  queueSecondaryMidi(static_cast<uint8_t>(0xB0 | ((channel1 - 1U) & 0x0F)),
                     cc, value, 3);
}

void sendDinPitchBend(uint8_t channel1, int bend) {
  const uint16_t bend14 = static_cast<uint16_t>(constrain(bend + 8192, 0, 16383));
  queueSecondaryMidi(static_cast<uint8_t>(0xE0 | ((channel1 - 1U) & 0x0F)),
                     bend14 & 0x7F, (bend14 >> 7) & 0x7F, 3);
}

uint8_t liveTargetForSource(uint8_t sourcePort) {
  if (sourcePort >= LOOP_TRACK_SOURCE_BASE &&
      sourcePort < LOOP_TRACK_SOURCE_BASE + arpnmidi3::kLoopTrackCount) {
    return sourcePort - LOOP_TRACK_SOURCE_BASE + 1U;
  }
  if (sourcePort >= STUTTER_SOURCE_BASE && sourcePort < STUTTER_SOURCE_BASE + LIVE_TARGET_COUNT) {
    return sourcePort - STUTTER_SOURCE_BASE;
  }
  return 0;
}

bool generatedLiveEffectSource(uint8_t sourcePort) {
  return sourcePort >= STUTTER_SOURCE_BASE && sourcePort < STUTTER_SOURCE_BASE + LIVE_TARGET_COUNT;
}

void sendFinalMidi(uint8_t sourcePort, uint8_t status, uint8_t data1, uint8_t data2) {
  const uint8_t type = status & 0xF0;
  const uint8_t ch1 = (status & 0x0F) + 1;
  if (type == 0x90) {
    if (data2 == 0) sendDinNoteOff(ch1, data1, 0);
    else sendDinNoteOn(ch1, data1, data2);
  } else if (type == 0x80) {
    sendDinNoteOff(ch1, data1, data2);
  } else if (type == 0xB0) {
    sendDinCc(ch1, data1, data2);
  } else if (type == 0xC0) {
    queueSecondaryMidi(status, data1, 0, 2);
  } else if (type == 0xD0) {
    queueSecondaryMidi(status, data1, 0, 2);
  } else if (type == 0xA0) {
    queueSecondaryMidi(status, data1, data2, 3);
  } else if (type == 0xE0) {
    const int bend = static_cast<int>(data1) | (static_cast<int>(data2) << 7);
    sendDinPitchBend(ch1, bend - 8192);
  } else if (status >= 0xF8) {
    sendDinRaw1(status);
  }
  sendMidiToUsbDevice(sourcePort, status, data1, data2);
}

void sendQuickJumpTransitionNoteOffs(uint8_t channel1) {
  const bool channelValid = channelEnabled(channel1);
  const uint8_t status = channelValid
      ? static_cast<uint8_t>(0x80 | ((channel1 - 1U) & 0x0F)) : 0;
  for (uint8_t note = 0; note < 128; ++note) {
    if (!heldInputNotes[note]) continue;

    // Clear ownership maps before changing the Quick Jump route.  Directly
    // sending a note-off here leaves mapped thru/chord/arp outputs latched,
    // especially when the thru channel is split or remapped.
    if (physicalHeldInputNotes[note]) {
      clearSplitNoteFromMainPaths(0, note);
    }
    for (uint8_t track = 0; track < arpnmidi3::kLoopTrackCount; ++track) {
      if (loopTrackHeldInputNotes[track][note]) {
        clearSplitNoteFromMainPaths(LOOP_TRACK_SOURCE_BASE + track, note);
      }
    }

    // Also release any direct Quick Jump output that was not represented by
    // one of the main-path ownership maps.
    if (channelValid) sendFinalMidi(255, status, note, 0);
  }
}

void setQuickJumpEnabled(bool enabled) {
  const bool wasEnabled = firmware3Settings.quickJumpEnabled != 0;
  if (wasEnabled == enabled) return;

  // Hold preserves notes during the transition into Quick Jump. Turning it
  // back off still releases those notes from the destination channel.
  // Without Hold, release notes from whichever channel is being left on both
  // sides of the transition so no note remains latched in the old path.
  const uint8_t releaseChannel = enabled
      ? firmware3Settings.quickJumpInputChannel
      : firmware3Settings.quickJumpOutputChannel;
  if (!enabled || !firmware3Settings.quickJumpHold) {
    sendQuickJumpTransitionNoteOffs(releaseChannel);
  }
  firmware3Settings.quickJumpEnabled = enabled ? 1 : 0;
}

void sendTargetFinal(uint8_t target, uint8_t sourcePort,
                     uint8_t status, uint8_t data1, uint8_t data2) {
  if (target >= STUTTER_ECHO_TARGET_COUNT) return;
  const uint8_t type = status & 0xF0;
  if ((type == 0x90 || type == 0x80) && data1 <= 127) {
    uint8_t &refs = finalOutputNoteRefs[target][status & 0x0F][data1];
    const bool on = type == 0x90 && data2 > 0;
    if (on) {
      if (refs < 0xFF) ++refs;
    } else {
      if (refs == 0) return;
      --refs;
      if (refs > 0) return;
      status = static_cast<uint8_t>(0x80 | (status & 0x0F));
      data2 = 0;
    }
  }
  sendFinalMidi(sourcePort, status, data1, data2);
}

void releaseFinalTarget(uint8_t target) {
  if (target >= STUTTER_ECHO_TARGET_COUNT) return;
  for (uint8_t channel = 0; channel < 16; ++channel) {
    for (uint8_t note = 0; note < 128; ++note) {
      uint8_t &refs = finalOutputNoteRefs[target][channel][note];
      if (refs == 0) continue;
      refs = 0;
      sendFinalMidi(255, static_cast<uint8_t>(0x80 | channel), note, 0);
    }
  }
}

arpnmidi3::EchoConfig echoConfigForTarget(uint8_t target) {
  const LiveTargetSettings &settings = firmware3Settings.liveTargets[target];
  arpnmidi3::EchoConfig config;
  config.lengthUs = static_cast<uint32_t>(min<uint64_t>(UINT32_MAX,
      musicalDurationUs(lengthSelectionPulses(settings.echoLength))));
  config.delayUs = static_cast<uint32_t>(min<uint64_t>(UINT32_MAX,
      musicalDurationUs(lengthSelectionPulses(settings.echoDelay))));
  config.wetPercent = settings.echoWet;
  config.drift = settings.echoDrift;
  return config;
}

void emitEchoEvent(void *, uint8_t target, const arpnmidi3::LoopMidiEvent &event) {
  sendTargetFinal(target, 255, event.status, event.data1, event.data2);
}

void processPostStutterEvent(uint8_t target, uint8_t sourcePort,
                             const arpnmidi3::LoopMidiEvent &event) {
  const uint8_t type = event.status & 0xF0;
  const LiveTargetSettings &targetSettings = firmware3Settings.liveTargets[target];
  if (targetSettings.echoEnabled && (type == 0x90 || type == 0x80)) {
    if (type == 0x90 && event.data2 > 0) {
      echoEngine.noteOn(time_us_64(), target, event.status, event.data1, event.data2,
                        echoConfigForTarget(target));
    } else {
      echoEngine.noteOff(time_us_64(), target, event.status, event.data1);
    }
  }
  sendTargetFinal(target, sourcePort, event.status, event.data1, event.data2);
}

void emitStutterEvent(void *, uint8_t historyTarget,
                      const arpnmidi3::LoopMidiEvent &event) {
  if (historyTarget < HISTORY_OUTPUT_TARGET_BASE) return;
  const uint8_t target = historyTarget - HISTORY_OUTPUT_TARGET_BASE;
  if (target < LIVE_TARGET_COUNT) processPostStutterEvent(target, 255, event);
}

// SELECTD's echo scheduling only, no dry pass-through: the note itself
// already goes out once through its own track's target, so SELECTD must
// never repeat that part, only add whatever repeats it generates on top.
void processSelectdEcho(const arpnmidi3::LoopMidiEvent &event) {
  const uint8_t type = event.status & 0xF0;
  const LiveTargetSettings &selectd = firmware3Settings.liveTargets[SELECTD_LIVE_TARGET];
  if (!selectd.echoEnabled || (type != 0x90 && type != 0x80)) return;
  if (type == 0x90 && event.data2 > 0) {
    echoEngine.noteOn(time_us_64(), SELECTD_LIVE_TARGET, event.status, event.data1,
                      event.data2, echoConfigForTarget(SELECTD_LIVE_TARGET));
  } else {
    echoEngine.noteOff(time_us_64(), SELECTD_LIVE_TARGET, event.status, event.data1);
  }
}

void emitNoteLengthEvent(void *, uint8_t target, uint8_t sourcePort,
                         const arpnmidi3::LoopMidiEvent &event) {
  rollingHistory.push(time_us_64(), HISTORY_OUTPUT_TARGET_BASE + target, event);
  if (!stutterRepeaters[target].active()) {
    processPostStutterEvent(target, sourcePort, event);
  }
  // SELECTD mirrors whichever loop track is currently selected. If this
  // note's own target is that track, and SELECTD has stutter or echo turned
  // on, the same event also feeds SELECTD's own independent pipeline, tagged
  // with its own target slot so its rolling history, stutter repeater, echo
  // state, and held-note refs never mix with the track's own. It never
  // passes the dry note through a second time, only whatever it repeats.
  const LiveTargetSettings &selectd = firmware3Settings.liveTargets[SELECTD_LIVE_TARGET];
  if (target >= 1 && target <= arpnmidi3::kLoopTrackCount &&
      target == multitrackLooper.selectedTrack() + 1U &&
      (selectd.stutterEnabled || selectd.echoEnabled)) {
    rollingHistory.push(time_us_64(), HISTORY_OUTPUT_TARGET_BASE + SELECTD_LIVE_TARGET, event);
    processSelectdEcho(event);
  }
}

void sendFanout(uint8_t sourcePort, uint8_t status, uint8_t data1, uint8_t data2) {
  uint8_t type = status & 0xF0;
  uint8_t ch1 = (status & 0x0F) + 1;
  if (roundRobinCh10To1Enabled() && ch1 == 10 && data1 >= 36 && data1 <= 51 &&
      (type == 0x90 || type == 0x80)) {
    const bool isOn = (type == 0x90 && data2 > 0);
    const uint8_t mappedCh = data1 - 35;
    sendFanout(sourcePort, (isOn ? 0x90 : 0x80) | ((mappedCh - 1) & 0x0F), 60, isOn ? 127 : 0);
    return;
  }
  if (roundRobinCh10To2Enabled() && ch1 == 10 && data1 >= 36 && data1 <= 49 &&
      (type == 0x90 || type == 0x80)) {
    const bool isOn = (type == 0x90 && data2 > 0);
    const uint8_t mappedCh = data1 - 34; // 36 -> 2, 49 -> 15
    uint8_t outNote = 60;
    if (mappedCh == 2) outNote = 12;
    else if (mappedCh == 3) outNote = 48;
    sendFanout(sourcePort, (isOn ? 0x90 : 0x80) | ((mappedCh - 1) & 0x0F), outNote, isOn ? 127 : 0);
    return;
  }
  if (!applyRouterToChannelMessage(status, data1)) return;
  type = status & 0xF0;
  const uint8_t target = liveTargetForSource(sourcePort);
  const bool generatedEffect = generatedLiveEffectSource(sourcePort);
  const LiveTargetSettings &targetSettings = firmware3Settings.liveTargets[target];
  if (!generatedEffect && type == 0x90 && data2 > 0 && targetSettings.velocityEnabled) {
    const uint16_t scaled = (static_cast<uint16_t>(data2) * targetSettings.velocityPercent + 50U) / 100U;
    data2 = static_cast<uint8_t>(min<uint16_t>(127, scaled));
    if (data2 == 0) return;
  }
  const arpnmidi3::LoopMidiEvent finalEvent{0, status, data1, data2};
  if (!generatedEffect && type < 0xF0 &&
      targetSettings.noteLengthEnabled && (type == 0x90 || type == 0x80)) {
    const uint32_t fallbackUs = static_cast<uint32_t>(min<uint64_t>(UINT32_MAX,
        musicalDurationUs(kDivisionPulseSteps[currentDivisionSetting()])));
    noteLengthEngine.process(time_us_64(), target, sourcePort, finalEvent,
                             targetSettings.noteLengthPercent, fallbackUs,
                             emitNoteLengthEvent, nullptr);
    return;
  }
  if (!generatedEffect && status < 0xF0) {
    emitNoteLengthEvent(nullptr, target, sourcePort, finalEvent);
    return;
  }
  processPostStutterEvent(target, sourcePort, finalEvent);
}

void sendAllNoteOffChannel(uint8_t ch1) {
  if (!channelEnabled(ch1)) return;
  // Exact active Note Offs are released through finalOutputNoteRefs first.
  // These channel-mode messages are the bounded catch-all for anything that
  // entered before ownership tracking or lives in an external device.
  sendFinalMidi(255, 0xB0 | ((ch1 - 1) & 0x0F), 123, 0);
  sendFinalMidi(255, 0xB0 | ((ch1 - 1) & 0x0F), 120, 0);
}

void storeUiResumeHint(uint8_t settingId) {
  watchdog_hw->scratch[0] = UI_RESUME_MAGIC;
  watchdog_hw->scratch[1] = settingId;
}

bool takeUiResumeHint(uint8_t &settingId) {
  if (watchdog_hw->scratch[0] != UI_RESUME_MAGIC) return false;
  settingId = static_cast<uint8_t>(watchdog_hw->scratch[1] & 0xFF);
  watchdog_hw->scratch[0] = 0;
  watchdog_hw->scratch[1] = 0;
  return true;
}

// Everything the output owns, forgotten together. After a silence the claims
// and the counters have to start from the same place, or a later Note Off finds
// a counter that no longer matches anything sounding.
void clearOutputOwnership() {
  memset(mappedThruNotes, 0xFF, sizeof(mappedThruNotes));
  memset(mappedLoopThruNotes, 0xFF, sizeof(mappedLoopThruNotes));
  memset(mappedThruChordNotes, 0xFF, sizeof(mappedThruChordNotes));
  memset(mappedLoopThruChordNotes, 0xFF, sizeof(mappedLoopThruChordNotes));
  memset(mappedThruChordCount, 0, sizeof(mappedThruChordCount));
  memset(mappedLoopThruChordCount, 0, sizeof(mappedLoopThruChordCount));
  memset(mappedArpOffNotes, 0xFF, sizeof(mappedArpOffNotes));
  memset(mappedArpOffChannels, 0, sizeof(mappedArpOffChannels));
  memset(mappedLoopArpOffNotes, 0xFF, sizeof(mappedLoopArpOffNotes));
  memset(mappedLoopArpOffChannels, 0, sizeof(mappedLoopArpOffChannels));
  memset(thruOutputRefCount, 0, sizeof(thruOutputRefCount));
  memset(thruOutputRefChannel, 0, sizeof(thruOutputRefChannel));
  memset(arpOffOutputRefCount, 0, sizeof(arpOffOutputRefCount));
  memset(arpOffOutputChannel, 0, sizeof(arpOffOutputChannel));
}

void panicMidiOnly() {
  for (uint8_t target = 0; target < STUTTER_ECHO_TARGET_COUNT; ++target) {
    deactivateStutter(target);
  }
  noteLengthEngine.reset(emitNoteLengthEvent, nullptr);
  echoEngine.reset(emitEchoEvent, nullptr);
  // Panic already stopped the looper transport here. loopAllOff also releases
  // the arp/drum scheduling clock immediately, rather than waiting for the
  // next arp tick to notice the looper went idle, so a hold-and-panic clears
  // the same state that gates flash writes without any delay.
  loopAllOff();
  multitrackLooper.cancelRecording();
  timeTravelImport.active = false;
  for (uint8_t target = 0; target < STUTTER_ECHO_TARGET_COUNT; ++target) {
    releaseFinalTarget(target);
  }
  memset(stutterSettingWasEnabled, 0, sizeof(stutterSettingWasEnabled));
  memset(stutterTimedOut, 0, sizeof(stutterTimedOut));
  memset(noteLengthSettingWasEnabled, 0, sizeof(noteLengthSettingWasEnabled));
  memset(echoSettingWasEnabled, 0, sizeof(echoSettingWasEnabled));
  const uint8_t panicArpCh = mainArpOutChannel();
  if (channelEnabled(panicArpCh)) sendAllNoteOffChannel(panicArpCh);
  for (uint8_t ch = 1; ch <= 16; ++ch) {
    if (settings.roundRobinMask & channelBit(ch)) sendAllNoteOffChannel(ch);
  }
  if (roundRobinCh10To1Enabled() || roundRobinCh10To2Enabled()) {
    for (uint8_t ch = 1; ch <= 16; ++ch) sendAllNoteOffChannel(ch);
  }
  if (arpChannelSpecialMode()) sendAllNoteOffChannel(10);
  if (channelEnabled(settings.thruOutChannel)) sendAllNoteOffChannel(settings.thruOutChannel);
  if (channelEnabled(settings.legatoChannel)) sendAllNoteOffChannel(settings.legatoChannel);
  if (settings.bassMode > 0) sendAllNoteOffChannel(bassModeChannel(settings.bassMode));
  if (channelEnabled(settings.sensorChannel)) sendAllNoteOffChannel(settings.sensorChannel);
  activeArpCount = 0;
  activeDrumArpCount = 0;
  loopSafeClearArmed = false;
  currentBassOutNote = -1;
  sensorRt.activeNote = -1;
  pushRt.activeNote = -1;
  sensorRt.lastPitch = 0;
  pushRt.lastPitch = 0;
  sensorRt.lastCcValue = -1;
  pushRt.lastCcValue = -1;
  clearOutputOwnership();
}

void panicAll() {
  panicMidiOnly();
  resetHeldState();
  ui.menuMode = MENU_SELECT;
  panicConfirmedUntilMs = millis() + 1200;
  ui.dirty = true;
  markActivity();
}

void resetHeldState() {
  memset(heldInputNotes, 0, sizeof(heldInputNotes));
  memset(heldVelocities, 0, sizeof(heldVelocities));
  memset(heldNoteOrder, 0, sizeof(heldNoteOrder));
  heldNoteOrderCounter = 0;
  memset(parameterLockHeldNotes, 0, sizeof(parameterLockHeldNotes));
  memset(physicalHeldInputNotes, 0, sizeof(physicalHeldInputNotes));
  memset(physicalHeldVelocities, 0, sizeof(physicalHeldVelocities));
  memset(loopHeldInputNotes, 0, sizeof(loopHeldInputNotes));
  memset(loopHeldVelocities, 0, sizeof(loopHeldVelocities));
  memset(loopTrackHeldInputNotes, 0, sizeof(loopTrackHeldInputNotes));
  memset(loopTrackHeldVelocities, 0, sizeof(loopTrackHeldVelocities));
  memset(arpLatchedNotes, 0, sizeof(arpLatchedNotes));
  memset(arpLatchedVelocities, 0, sizeof(arpLatchedVelocities));
  memset(thruLatchedNotes, 0, sizeof(thruLatchedNotes));
  memset(arpFrozenNotes, 0, sizeof(arpFrozenNotes));
  memset(thruFrozenNotes, 0, sizeof(thruFrozenNotes));
  memset(thruFrozenMappedNotes, 0xFF, sizeof(thruFrozenMappedNotes));
  memset(heldDrumNotes, 0, sizeof(heldDrumNotes));
  memset(heldDrumVelocities, 0, sizeof(heldDrumVelocities));
  memset(physicalHeldDrumNotes, 0, sizeof(physicalHeldDrumNotes));
  memset(physicalHeldDrumVelocities, 0, sizeof(physicalHeldDrumVelocities));
  memset(loopHeldDrumNotes, 0, sizeof(loopHeldDrumNotes));
  memset(loopHeldDrumVelocities, 0, sizeof(loopHeldDrumVelocities));
  memset(loopTrackHeldDrumNotes, 0, sizeof(loopTrackHeldDrumNotes));
  memset(loopTrackHeldDrumVelocities, 0, sizeof(loopTrackHeldDrumVelocities));
  clearOutputOwnership();
  memset(legatoHeldCount, 0, sizeof(legatoHeldCount));
  memset(legatoHeldVelocity, 0, sizeof(legatoHeldVelocity));
  memset(legatoHeldSource, 255, sizeof(legatoHeldSource));
  memset(legatoHeldOrder, 0, sizeof(legatoHeldOrder));
  legatoOrderCounter = 0;
  legatoOutputActive = false;
  legatoOutputNote = 0xFF;
  legatoOutputVelocity = 0;
  legatoOutputSource = 255;
  legatoOutputChannel = 0;
  heldCount = 0;
  arpHeldCount = 0;
  heldDrumCount = 0;
  drumAftertouchPressure = 127;
  mainAftertouchPressure = 127;
  activeArpCount = 0;
  activeDrumArpCount = 0;
  memset(activeArpNotes, 0xFF, sizeof(activeArpNotes));
  memset(activeArpChannels, 0, sizeof(activeArpChannels));
  for (CustomArpVoice &voice : customArpVoices) voice = CustomArpVoice{};
  customArpCycleStartUs = 0;
  customArpPlayIndex = 0;
  customArpLearning = false;
  customArpWaitingForFirstNote = false;
  roundRobinCursor = 0;
  arpGlobalStep = 0;
  arpSequenceStep = 0;
  arpPatternStep = 0;
  arpHadKeys = false;
  arpLatchAwaitingNewPhrase = false;
  arpLatchSensorClose = false;
  arpLatchPushClose = false;
  arpFreezeSensorClose = false;
  arpFreezePushClose = false;
  arpFreezeActive = false;
  arpFreezePlusActive = false;
  memset(divNoteHeld, 0, sizeof(divNoteHeld));
  memset(physicalDivNoteHeld, 0, sizeof(physicalDivNoteHeld));
  memset(loopDivNoteHeld, 0, sizeof(loopDivNoteHeld));
  memset(divNoteHeldStamp, 0, sizeof(divNoteHeldStamp));
  divNotePressCounter = 0;
  currentBassSource = -1;
  currentBassOutNote = -1;
  activeArpCount = 0;
  activeDrumArpCount = 0;
}

bool noteInScale(uint8_t pc, uint8_t key, uint8_t scale) {
  if (scale == SCALE_OFF || key == 0) return true;
  const uint8_t tonic = rootPcFromKeyValue(key);
  const uint8_t selectedScale = effectiveScaleForKeyMode(key, scale);
  auto has = [&](const uint8_t *table) {
    for (uint8_t i = 0; i < 7; ++i) {
      if (pc == ((table[i] + tonic) % 12)) return true;
    }
    return false;
  };
  switch (selectedScale) {
    case SCALE_MAJOR: return has(kMajor);
    case SCALE_MINOR: return has(kMinor);
    case SCALE_MAJOR_MINOR: return has(kMajor) || has(kMinor);
    case SCALE_BLUES: return has(kBlues);
    case SCALE_MAJOR_BLUES: return has(kMajorBlues);
    case SCALE_BLUES_BOTH: return has(kBlues) || has(kMajorBlues) || has(kMinor);
    case SCALE_HARM_MINOR: return has(kHarmMinor);
    case SCALE_MELODIC_MINOR: return has(kMelMinor);
    case SCALE_USER: {
      const uint8_t relative = (pc + 12 - tonic) % 12;
      return (firmware3Settings.userScaleMask & (1U << relative)) != 0;
    }
    default: return true;
  }
}

uint8_t ckeyScaleTypeFromForceScale(uint8_t forceScale) {
  switch (forceScale) {
    case SCALE_MAJOR: return 0;
    case SCALE_MINOR: return 1;
    case SCALE_HARM_MINOR: return 2;
    case SCALE_MELODIC_MINOR: return 3;
    case SCALE_BLUES: return 4;
    case SCALE_MAJOR_BLUES: return 5;
    default: return 0;
  }
}

uint8_t mapWhiteKeyToScale(uint8_t inNote, uint8_t rootPc, uint8_t scaleType) {
  static const uint8_t scales[6][7] = {
    {0, 2, 4, 5, 7, 9, 11},
    {0, 2, 3, 5, 7, 8, 10},
    {0, 2, 3, 5, 7, 8, 11},
    {0, 2, 3, 5, 7, 9, 11},
    {0, 3, 5, 6, 7, 10, 12},
    {0, 2, 3, 4, 7, 9, 12}
  };
  static const uint8_t pcToWhiteIndex[12] = {
    0, 0, 1, 1, 2, 3, 3, 4, 4, 5, 5, 6
  };

  const uint8_t whiteIndex = pcToWhiteIndex[inNote % 12];
  const int16_t octaveBase = (inNote / 12) * 12;
  int16_t outNote = octaveBase + rootPc + scales[scaleType][whiteIndex];
  if (outNote < 0) outNote = 0;
  if (outNote > 127) outNote = 127;
  return static_cast<uint8_t>(outNote);
}

uint8_t quantizeUp(uint8_t note) {
  if (settings.forceKey == 0 || settings.forceScale == SCALE_OFF) return note;
  const uint8_t scale = effectiveScaleForKeyMode(settings.forceKey, settings.forceScale);
  if (scale == SCALE_OFF) return note;
  if (ckeyEnabled()) {
    return mapWhiteKeyToScale(note, rootPcFromKeyValue(settings.forceKey),
                              ckeyScaleTypeFromForceScale(scale));
  }
  uint8_t candidate = note;
  for (uint8_t i = 0; i < 12; ++i) {
    if (noteInScale(candidate % 12, settings.forceKey, scale)) return candidate;
    candidate++;
  }
  return note;
}

uint8_t chordNoteForPosition(uint8_t root, int8_t position) {
  if (position == 0 || position == 1) return root;
  const int direction = position > 0 ? 1 : -1;
  uint8_t remaining = static_cast<uint8_t>(abs(position) - 1);
  int candidate = root;
  const uint8_t scale = effectiveScaleForKeyMode(settings.forceKey, settings.forceScale);
  const bool useScale = settings.forceKey != 0 && scale != SCALE_OFF;
  while (remaining > 0) {
    candidate += direction;
    if (candidate < 0 || candidate > 127) break;
    if (!useScale || noteInScale(candidate % 12, settings.forceKey, scale)) --remaining;
  }
  return clampU8(candidate, 0, 127);
}

uint8_t buildChordNotes(uint8_t root, uint8_t *notes) {
  const uint8_t correctedRoot = quantizeUp(root);
  if (!firmware3Settings.chordEnabled) {
    notes[0] = correctedRoot;
    return 1;
  }
  uint8_t count = 0;
  for (uint8_t i = 0; i < 4; ++i) {
    const uint8_t note = chordNoteForPosition(correctedRoot, firmware3Settings.chordPositions[i]);
    bool duplicate = false;
    for (uint8_t j = 0; j < count; ++j) duplicate |= notes[j] == note;
    if (!duplicate) notes[count++] = note;
  }
  return count;
}

uint16_t customArpLengthPulses(uint8_t selection) {
  const uint16_t bar = arpnmidi3::barPulses(firmware3Settings.timeSignature != 0);
  switch (selection) {
    case 0: return max<uint16_t>(1, bar / 4);
    case 1: return max<uint16_t>(1, bar / 2);
    case 2: return bar;
    case 3: return bar * 2U;
    case 4: return bar * 4U;
    default: return bar * 8U;
  }
}

uint16_t customArpElapsedPulses(uint64_t nowUs) {
  if (customArpLearnStartUs == 0 || nowUs <= customArpLearnStartUs) return 0;
  const uint64_t onePulseUs = max<uint64_t>(1, musicalDurationUs(1));
  return min<uint64_t>(UINT16_MAX, (nowUs - customArpLearnStartUs) / onePulseUs);
}

void startCustomArpLearn() {
  customArpPattern = CustomArpPattern{};
  customArpPattern.lengthSelection = firmware3Settings.customArpLength;
  memset(customArpLearnActiveEvent, 0xFF, sizeof(customArpLearnActiveEvent));
  customArpLearning = true;
  customArpWaitingForFirstNote = true;
  customArpLearnStartUs = 0;
  customArpLearnEndUs = 0;
  customArpLearnLowestNote = 127;
}

void finishCustomArpLearn() {
  if (!customArpLearning) return;
  const uint16_t lengthPulses = customArpLengthPulses(customArpPattern.lengthSelection);
  const uint16_t endPulse = customArpLearnStartUs ?
      min<uint16_t>(lengthPulses, customArpElapsedPulses(time_us_64())) : 0;
  for (uint8_t note = 0; note < 128; ++note) {
    const int8_t idx = customArpLearnActiveEvent[note];
    if (idx >= 0 && idx < customArpPattern.count) {
      CustomArpEvent &event = customArpPattern.events[idx];
      event.gatePulses = max<uint16_t>(1, endPulse > event.startPulse ?
          endPulse - event.startPulse : 1);
    }
  }
  if (customArpPattern.count > 0) {
    for (uint8_t i = 0; i < customArpPattern.count; ++i) {
      CustomArpEvent &event = customArpPattern.events[i];
      event.pitchOffset = static_cast<int8_t>(event.pitchOffset - customArpLearnLowestNote);
      const uint16_t remaining = max<uint16_t>(1, lengthPulses - event.startPulse);
      event.gatePulses = min<uint16_t>(max<uint16_t>(1, event.gatePulses), remaining);
    }
  }
  customArpLearning = false;
  customArpWaitingForFirstNote = false;
  memset(customArpLearnActiveEvent, 0xFF, sizeof(customArpLearnActiveEvent));
  markExtendedPresetDirty();
  ui.dirty = true;
}

void clearCustomArpPattern() {
  customArpPattern = CustomArpPattern{};
  customArpPattern.lengthSelection = firmware3Settings.customArpLength;
  markExtendedPresetDirty();
  ui.dirty = true;
}

void captureCustomArpNote(uint8_t note, uint8_t velocity, bool on, uint64_t nowUs) {
  if (!customArpLearning) return;
  if (on && velocity > 0) {
    if (customArpWaitingForFirstNote) {
      customArpWaitingForFirstNote = false;
      customArpLearnStartUs = nowUs;
      customArpLearnEndUs = nowUs + musicalDurationUs(
          customArpLengthPulses(customArpPattern.lengthSelection));
    }
    if (customArpPattern.count >= MAX_CUSTOM_ARP_EVENTS || nowUs >= customArpLearnEndUs) return;
    const uint8_t idx = customArpPattern.count++;
    CustomArpEvent &event = customArpPattern.events[idx];
    event.startPulse = min<uint16_t>(customArpLengthPulses(customArpPattern.lengthSelection) - 1,
                                     customArpElapsedPulses(nowUs));
    event.gatePulses = 1;
    event.pitchOffset = static_cast<int8_t>(note);
    event.velocity = max<uint8_t>(1, velocity);
    customArpLearnActiveEvent[note] = idx;
    customArpLearnLowestNote = min<uint8_t>(customArpLearnLowestNote, note);
  } else {
    const int8_t idx = customArpLearnActiveEvent[note];
    if (idx >= 0 && idx < customArpPattern.count) {
      CustomArpEvent &event = customArpPattern.events[idx];
      const uint16_t offPulse = min<uint16_t>(customArpLengthPulses(customArpPattern.lengthSelection),
                                              customArpElapsedPulses(nowUs));
      event.gatePulses = max<uint16_t>(1, offPulse > event.startPulse ?
          offPulse - event.startPulse : 1);
      customArpLearnActiveEvent[note] = -1;
    }
  }
}

void releaseCustomArpVoices() {
  for (CustomArpVoice &voice : customArpVoices) {
    if (voice.active) {
      sendFanout(255, 0x80 | ((voice.channel - 1) & 0x0F), voice.note, 0);
      voice.active = false;
    }
  }
}

void tickCustomArp(uint64_t nowUs) {
  for (CustomArpVoice &voice : customArpVoices) {
    if (voice.active && nowUs >= voice.offUs) {
      sendFanout(255, 0x80 | ((voice.channel - 1) & 0x0F), voice.note, 0);
      voice.active = false;
    }
  }
  if (arpHeldCount == 0 || customArpPattern.count == 0 || !channelEnabled(mainArpOutChannel())) {
    releaseCustomArpVoices();
    customArpCycleStartUs = 0;
    customArpPlayIndex = 0;
    return;
  }
  const uint16_t lengthPulses = customArpLengthPulses(customArpPattern.lengthSelection);
  const uint64_t lengthUs = max<uint64_t>(1, musicalDurationUs(lengthPulses));
  if (customArpCycleStartUs == 0) customArpCycleStartUs = nowUs;
  while (nowUs - customArpCycleStartUs >= lengthUs) {
    releaseCustomArpVoices();
    customArpCycleStartUs += lengthUs;
    customArpPlayIndex = 0;
  }
  uint8_t lowest = 127;
  for (uint8_t i = 0; i < arpHeldCount; ++i) lowest = min<uint8_t>(lowest, arpHeldSorted[i]);
  const uint64_t elapsedUs = nowUs - customArpCycleStartUs;
  while (customArpPlayIndex < customArpPattern.count) {
    const CustomArpEvent &event = customArpPattern.events[customArpPlayIndex];
    const uint64_t eventUs = musicalDurationUs(event.startPulse);
    if (eventUs > elapsedUs) break;
    const uint8_t root = clampU8(static_cast<int>(lowest) + event.pitchOffset, 0, 127);
    uint8_t notes[4];
    const uint8_t count = buildChordNotes(root, notes);
    for (uint8_t n = 0; n < count; ++n) {
      for (CustomArpVoice &voice : customArpVoices) {
        if (voice.active) continue;
        voice.active = true;
        voice.note = notes[n];
        voice.channel = nextRoundRobinChannel(mainArpOutChannel());
        voice.offUs = min<uint64_t>(customArpCycleStartUs + lengthUs,
            customArpCycleStartUs + eventUs + musicalDurationUs(event.gatePulses));
        sendFanout(255, 0x90 | ((voice.channel - 1) & 0x0F), voice.note, event.velocity);
        break;
      }
    }
    ++customArpPlayIndex;
  }
}

bool loopOwnsInput(uint8_t sourcePort) {
  return sourcePort >= LOOP_TRACK_SOURCE_BASE &&
         sourcePort < LOOP_TRACK_SOURCE_BASE + arpnmidi3::kLoopTrackCount;
}

uint8_t loopTrackForSource(uint8_t sourcePort) {
  if (sourcePort >= LOOP_TRACK_SOURCE_BASE &&
      sourcePort < LOOP_TRACK_SOURCE_BASE + arpnmidi3::kLoopTrackCount) {
    return sourcePort - LOOP_TRACK_SOURCE_BASE;
  }
  return 0;
}

bool inputOwnerHeld(uint8_t sourcePort, uint8_t note) {
  return loopOwnsInput(sourcePort)
      ? loopTrackHeldInputNotes[loopTrackForSource(sourcePort)][note]
      : physicalHeldInputNotes[note];
}

void setInputOwnerState(uint8_t sourcePort, uint8_t note, uint8_t velocity, bool on) {
  bool *ownerNotes = loopOwnsInput(sourcePort)
      ? loopTrackHeldInputNotes[loopTrackForSource(sourcePort)] : physicalHeldInputNotes;
  uint8_t *ownerVelocities = loopOwnsInput(sourcePort)
      ? loopTrackHeldVelocities[loopTrackForSource(sourcePort)] : physicalHeldVelocities;
  if (on && !ownerNotes[note]) heldNoteOrder[note] = ++heldNoteOrderCounter;
  ownerNotes[note] = on;
  ownerVelocities[note] = on ? velocity : 0;
  loopHeldInputNotes[note] = false;
  loopHeldVelocities[note] = 0;
  for (uint8_t track = 0; track < arpnmidi3::kLoopTrackCount; ++track) {
    if (loopTrackHeldInputNotes[track][note]) {
      loopHeldInputNotes[note] = true;
      loopHeldVelocities[note] = loopTrackHeldVelocities[track][note];
      break;
    }
  }
  heldInputNotes[note] = physicalHeldInputNotes[note] || loopHeldInputNotes[note];
  heldVelocities[note] = physicalHeldInputNotes[note] ? physicalHeldVelocities[note] : loopHeldVelocities[note];
}

void setDrumOwnerState(uint8_t sourcePort, uint8_t note, uint8_t velocity, bool on) {
  bool *ownerNotes = loopOwnsInput(sourcePort)
      ? loopTrackHeldDrumNotes[loopTrackForSource(sourcePort)] : physicalHeldDrumNotes;
  uint8_t *ownerVelocities = loopOwnsInput(sourcePort)
      ? loopTrackHeldDrumVelocities[loopTrackForSource(sourcePort)] : physicalHeldDrumVelocities;
  ownerNotes[note] = on;
  ownerVelocities[note] = on ? velocity : 0;
  loopHeldDrumNotes[note] = false;
  loopHeldDrumVelocities[note] = 0;
  for (uint8_t track = 0; track < arpnmidi3::kLoopTrackCount; ++track) {
    if (loopTrackHeldDrumNotes[track][note]) {
      loopHeldDrumNotes[note] = true;
      loopHeldDrumVelocities[note] = loopTrackHeldDrumVelocities[track][note];
      break;
    }
  }
  heldDrumNotes[note] = physicalHeldDrumNotes[note] || loopHeldDrumNotes[note];
  heldDrumVelocities[note] = physicalHeldDrumNotes[note] ? physicalHeldDrumVelocities[note] : loopHeldDrumVelocities[note];
}

void rebuildHeldSorted() {
  heldCount = 0;
  for (uint8_t n = 0; n < 128 && heldCount < MAX_HELD_NOTES; ++n) {
    if (heldInputNotes[n]) heldSorted[heldCount++] = n;
  }
}

void rebuildArpHeldSorted() {
  arpHeldCount = 0;
  for (uint8_t n = 0; n < 128 && arpHeldCount < MAX_HELD_NOTES; ++n) {
    bool include = false;
    if (arpFreezeActive) include = arpFrozenNotes[n] || heldInputNotes[n];
    else if (arpLatchEnabled()) include = arpLatchedNotes[n];
    else include = heldInputNotes[n];
    if (!include) continue;
    if (!firmware3Settings.arpNoteOrder) {
      arpHeldSorted[arpHeldCount++] = n;
      continue;
    }
    uint8_t insert = arpHeldCount;
    while (insert > 0 && heldNoteOrder[arpHeldSorted[insert - 1]] > heldNoteOrder[n]) {
      arpHeldSorted[insert] = arpHeldSorted[insert - 1];
      --insert;
    }
    arpHeldSorted[insert] = n;
    ++arpHeldCount;
  }
}

bool anyPhysicalInputNotesHeld() {
  for (uint8_t n = 0; n < 128; ++n) {
    if (physicalHeldInputNotes[n]) return true;
  }
  return false;
}

int8_t bassOutputNoteForSource(uint8_t sourceNote) {
  if (settings.bassMode == 0) return -1;
  const int octaveShift = static_cast<int>(bassModeOctaveOffset(settings.bassMode)) * 12;
  int out = static_cast<int>(sourceNote) + octaveShift;
  if (out < 0) out = 0;
  if (out > 127) out = 127;
  return static_cast<int8_t>(quantizeUp(static_cast<uint8_t>(out)));
}

void updateBassVoice() {
  const uint8_t bassChannel = bassModeChannel(settings.bassMode);
  if (!channelEnabled(bassChannel) || settings.arpOutChannel == bassChannel) {
    if (currentBassOutNote >= 0) {
      const bool legatoOwnsSame = legatoOutputActive &&
                                  legatoOutputChannel == bassChannel &&
                                  legatoOutputNote == static_cast<uint8_t>(currentBassOutNote);
      if (!legatoOwnsSame) {
        sendFanout(255, 0x80 | ((bassChannel - 1) & 0x0F), currentBassOutNote, 0);
      }
      currentBassOutNote = -1;
    }
    return;
  }

  rebuildHeldSorted();
  rebuildArpHeldSorted();
  const uint8_t *bassCandidates = (arpLatchPlusEnabled() || arpFreezePlusActive)
                                      ? arpHeldSorted : heldSorted;
  const uint8_t bassCandidateCount = (arpLatchPlusEnabled() || arpFreezePlusActive)
                                         ? arpHeldCount : heldCount;
  int8_t nextSource = -1;
  for (uint8_t i = 0; i < bassCandidateCount; ++i) {
    if (bassCandidates[i] <= firmware3Settings.bassHighestNote) {
      nextSource = bassCandidates[i];
      break;
    }
  }
  if (nextSource < 0) {
    if (currentBassOutNote >= 0) {
      const bool legatoOwnsSame = legatoOutputActive &&
                                  legatoOutputChannel == bassChannel &&
                                  legatoOutputNote == static_cast<uint8_t>(currentBassOutNote);
      if (!legatoOwnsSame) {
        sendFanout(255, 0x80 | ((bassChannel - 1) & 0x0F), currentBassOutNote, 0);
      }
      currentBassOutNote = -1;
    }
    currentBassSource = -1;
    return;
  }

  const int8_t nextOut = bassOutputNoteForSource(nextSource);
  if (nextOut == currentBassOutNote && nextSource == currentBassSource) return;

  if (currentBassOutNote >= 0) {
    const bool legatoOwnsSame = legatoOutputActive &&
                                legatoOutputChannel == bassChannel &&
                                legatoOutputNote == static_cast<uint8_t>(currentBassOutNote);
    if (!legatoOwnsSame) {
      sendFanout(255, 0x80 | ((bassChannel - 1) & 0x0F), currentBassOutNote, 0);
    }
  }

  currentBassSource = nextSource;
  currentBassOutNote = nextOut;
  if (currentBassOutNote >= 0) {
    sendFanout(255, 0x90 | ((bassChannel - 1) & 0x0F), currentBassOutNote, currentArpVelocitySetting());
  }
}

uint8_t effectiveThruChannel() {
  if (!channelEnabled(settings.thruOutChannel)) return 0;
  if (settings.bassMode > 0 && settings.thruOutChannel == bassModeChannel(settings.bassMode)) return 0;
  return settings.thruOutChannel;
}

uint8_t inputPerformanceOutChannel() {
  return arpChannelSpecialMode() ? 1 : effectiveThruChannel();
}

uint8_t bassOutputChannel() {
  return bassModeChannel(settings.bassMode);
}

template <typename Fn>
void forEachInputChannelOutput(Fn fn) {
  uint8_t sent[3];
  uint8_t sentCount = 0;

  const uint8_t candidates[3] = {
    mainArpOutChannel(),
    effectiveThruChannel(),
    bassOutputChannel()
  };

  for (uint8_t i = 0; i < 3; ++i) {
    const uint8_t ch = candidates[i];
    if (!channelEnabled(ch)) continue;
    if (arpChannelSpecialMode() && ch == 10) continue;

    bool duplicate = false;
    for (uint8_t j = 0; j < sentCount; ++j) {
      if (sent[j] == ch) {
        duplicate = true;
        break;
      }
    }
    if (duplicate) continue;
    sent[sentCount++] = ch;
    fn(ch);
  }
}

template <typename Fn>
void forEachCcOutput(Fn fn) {
  if (settings.ccOutChannel == 17) {
    forEachInputChannelOutput(fn);
    return;
  }
  if (settings.ccOutChannel >= 1 && settings.ccOutChannel <= 16) fn(settings.ccOutChannel);
}

uint8_t currentArpSelection() {
  // The stored mode, never the composite menu accessor. SET_ARP_MODE's raw
  // value follows the submenu cursor when nothing is being edited, and the
  // cursor is navigation. Reading it as the mode made the arp boot silent
  // (cursor 0 reads as Off), turn Up into Random after visiting item 6, and
  // walk octaves after leaving through Back. Edits inside the submenu write
  // settings.arpMode live, so live audition still works through this path.
  return clampU8(settings.arpMode, 0, ARP_SELECTION_COUNT - 1);
}

int8_t activeDivNoteSlot() {
  int8_t best = -1;
  uint32_t bestStamp = 0;
  for (uint8_t i = 0; i < DIV_NOTE_SLOT_COUNT; ++i) {
    if (divNoteHeld[i] && divNoteHeldStamp[i] >= bestStamp) {
      best = static_cast<int8_t>(i);
      bestStamp = divNoteHeldStamp[i];
    }
  }
  return best;
}

uint8_t currentDivisionSetting() {
  const uint8_t selected = clampU8(effectiveSettingValue(SET_DIVISION), 0, ARP_DIVISION_FOLLOW_DRUM);
  if (selected != ARP_DIVISION_FOLLOW_DRUM) return selected;
  const int8_t rollSlot = activeDivNoteSlot();
  if (rollSlot >= 0) return divNoteSlotToDivision(static_cast<uint8_t>(rollSlot));
  if (firmware3Settings.drumDivision < DIVISION_COUNT) return firmware3Settings.drumDivision;
  return DIV_1_16;
}

int8_t currentDrumDivisionSetting() {
  const int8_t rollSlot = activeDivNoteSlot();
  if (rollSlot >= 0) return divNoteSlotToDivision(static_cast<uint8_t>(rollSlot));
  if (firmware3Settings.drumDivision == DRUM_DIVISION_FREE) return -1;
  if (firmware3Settings.drumDivision == DRUM_DIVISION_FOLLOW_ARP) {
    return settings.division < DIVISION_COUNT
        ? static_cast<int8_t>(settings.division)
        : static_cast<int8_t>(DIV_1_16);
  }
  return clampU8(firmware3Settings.drumDivision, 0, DIVISION_COUNT - 1);
}

uint8_t sensorPercent() {
  if (!sensorRt.inRange) return 0;
  const int span = SENSOR_ACTIVE_MAX_MM - VL53_VALID_MIN_MM;
  const int rel = constrain(SENSOR_ACTIVE_MAX_MM - sensorRt.mm, 0, span);
  return map(rel, 0, span, 0, 100);
}

uint8_t applyDownByPercent(uint8_t base, uint8_t pct, uint8_t floorValue) {
  return constrain(map(pct, 0, 100, base, floorValue), floorValue, base);
}

uint8_t currentArpVelocitySetting() {
  const uint8_t base = constrain(getSettingValueRaw(SET_VELOCITY), 1, 127);
  if (firmware3Settings.mainAftertouchArpVelocity) {
    return max<uint8_t>(1, mainAftertouchPressure);
  }
  uint8_t value = base;
  if (sensorRt.inRange && settings.sensorMode == SENSOR_VEL_DOWN) {
    value = min<uint8_t>(value, applyDownByPercent(base, sensorPercent(), 1));
  }
  if (pushRt.inRange && settings.pushMode == SENSOR_VEL_DOWN) {
    value = min<uint8_t>(value, applyDownByPercent(base, pushRt.pct, 1));
  }
  return value;
}

uint8_t currentArpLengthPctSetting() {
  const uint8_t base = constrain(getSettingValueRaw(SET_LENGTH), 1, 100);
  uint8_t value = base;
  if (sensorRt.inRange && settings.sensorMode == SENSOR_LEN_DOWN) {
    value = min<uint8_t>(value, applyDownByPercent(base, sensorPercent(), 1));
  }
  if (pushRt.inRange && settings.pushMode == SENSOR_LEN_DOWN) {
    value = min<uint8_t>(value, applyDownByPercent(base, pushRt.pct, 1));
  }
  return value;
}

bool arpSelectionIsClassicMode(uint8_t sel) {
  return sel <= ARPSEL_RANDOM;
}

uint8_t classicArpModeFromSelection(uint8_t sel) {
  if (!arpSelectionIsClassicMode(sel)) return ARP_UP;
  switch (sel) {
    case ARPSEL_OFF: return ARP_OFF;
    case ARPSEL_UP: return ARP_UP;
    case ARPSEL_DOWN: return ARP_DOWN;
    case ARPSEL_UPDOWN1: return ARP_UPDOWN1;
    case ARPSEL_UPDOWN2: return ARP_UPDOWN2;
    case ARPSEL_TRIGGER: return ARP_TRIGGER;
    case ARPSEL_RANDOM: return ARP_RANDOM;
    default: return ARP_UP;
  }
}

uint8_t patternFromArpSelection(uint8_t sel) {
  if (arpSelectionIsClassicMode(sel)) return PAT_MODE;
  switch (sel) {
    case ARPSEL_PAT_UP_1OCT: return PAT_UP_1OCT;
    case ARPSEL_PAT_RHYTHM: return PAT_RHYTHM;
    case ARPSEL_PAT_OSTINATO: return PAT_OSTINATO;
    case ARPSEL_PAT_OCTAVE_WALK: return PAT_OCTAVE_WALK;
    case ARPSEL_PAT_FIFTH: return PAT_FIFTH;
    case ARPSEL_PAT_BASS_CHORD: return PAT_BASS_CHORD;
    case ARPSEL_PAT_CHORD_RUN: return PAT_CHORD_RUN;
    default: return PAT_MODE;
  }
}

void rebuildHeldDrumCount() {
  heldDrumCount = 0;
  for (uint8_t note = 0; note < 128; ++note) {
    if (heldDrumNotes[note]) heldDrumCount++;
  }
}

void restartArpTiming(bool sendNoteOffs = true) {
  if (sendNoteOffs) {
    arpNoteOffs();
    drumArpNoteOffs();
  }
  arpGateOffMs = 0;
  drumGateOffMs = 0;
  arpGridOriginUs = time_us_64();
  drumGridOriginUs = arpGridOriginUs;
  arpNextStepUs = arpGridOriginUs;
  drumNextStepUs = drumGridOriginUs;
  arpGlobalStep = 0;
  arpSequenceStep = 0;
  drumGlobalStep = 0;
  arpPatternStep = 0;
}

void restartArpFromNewKeyPhraseAt(uint64_t phraseStartUs) {
  arpGateOffMs = 0;
  arpGlobalStep = 0;
  arpSequenceStep = 0;
  arpPatternStep = 0;
  const uint64_t readyUs = phraseStartUs + (ARP_KEY_SYNC_CAPTURE_MS * 1000ULL);
  if (drumNextStepUs != 0) {
    // Drums own the running clock. The new arp phrase joins their grid at its
    // own division instead of resetting a rolling pattern underneath the
    // performer, which was audible as a hiccup on every played phrase.
    const uint8_t division = currentDivisionSetting();
    const uint64_t stepUs = max<uint64_t>(1, musicalDurationUs(kDivisionPulseSteps[division]));
    uint32_t boundary = readyUs > drumGridOriginUs
        ? static_cast<uint32_t>((readyUs - drumGridOriginUs) / stepUs) : 0;
    while (swungGridTimeUs(drumGridOriginUs, boundary, division) < readyUs) ++boundary;
    arpGridOriginUs = drumGridOriginUs;
    arpGlobalStep = boundary;
    arpNextStepUs = swungGridTimeUs(drumGridOriginUs, boundary, division);
    return;
  }
  drumGateOffMs = 0;
  drumGlobalStep = 0;
  arpGridOriginUs = readyUs;
  arpNextStepUs = arpGridOriginUs;
  drumGridOriginUs = arpGridOriginUs;
  drumNextStepUs = drumGridOriginUs;
}

void restartArpFromNewKeyPhrase() {
  const uint64_t nowUs = time_us_64();
  if (!firmware3Settings.arpRetriggerSync || drumNextStepUs != 0) {
    restartArpFromNewKeyPhraseAt(nowUs);
    return;
  }

  const uint8_t division = currentDivisionSetting();
  const uint64_t readyUs = nowUs + (ARP_KEY_SYNC_CAPTURE_MS * 1000ULL);
  uint64_t originUs = musicalClock.phaseOriginUs();
  if (originUs == 0 || originUs > readyUs) originUs = nowUs;
  const uint64_t stepUs = max<uint64_t>(1, musicalDurationUs(kDivisionPulseSteps[division]));
  uint32_t boundary = static_cast<uint32_t>((readyUs - originUs + stepUs - 1ULL) / stepUs);
  while (swungGridTimeUs(originUs, boundary, division) < readyUs) ++boundary;

  arpGateOffMs = 0;
  drumGateOffMs = 0;
  arpGlobalStep = 0;
  arpSequenceStep = 0;
  drumGlobalStep = 0;
  arpPatternStep = 0;
  arpGridOriginUs = swungGridTimeUs(originUs, boundary, division);
  arpNextStepUs = arpGridOriginUs;
  drumGridOriginUs = arpGridOriginUs;
  drumNextStepUs = drumGridOriginUs;
}

// Called whenever a division might have moved, which a drum roll does on every
// press and release. Each half only re-grids when its own next step actually
// lands somewhere new, so rolling drums leaves the arp's timing untouched.
void syncArpDivisionToGrid() {
  const uint64_t now = time_us_64();
  if (arpNextStepUs != 0) {
    if (now < arpGridOriginUs) {
      arpNextStepUs = arpGridOriginUs;
    } else {
      const uint8_t division = currentDivisionSetting();
      const uint64_t stepUs = max<uint64_t>(1, musicalDurationUs(kDivisionPulseSteps[division]));
      uint32_t boundary = static_cast<uint32_t>((now - arpGridOriginUs) / stepUs);
      while (swungGridTimeUs(arpGridOriginUs, boundary, division) < now) ++boundary;
      const uint64_t nextUs = swungGridTimeUs(arpGridOriginUs, boundary, division);
      if (nextUs != arpNextStepUs) {
        arpGlobalStep = boundary;
        arpNextStepUs = nextUs;
      }
    }
  }

  if (drumNextStepUs == 0) return;
  const int8_t drumDivision = currentDrumDivisionSetting();
  if (drumDivision < 0) {
    drumNextStepUs = 0;
    return;
  }
  if (now < drumGridOriginUs) {
    drumNextStepUs = drumGridOriginUs;
    return;
  }
  const uint64_t drumStepUs = max<uint64_t>(1,
      musicalDurationUs(kDivisionPulseSteps[drumDivision]));
  uint32_t drumBoundary = static_cast<uint32_t>((now - drumGridOriginUs) / drumStepUs);
  while (swungGridTimeUs(drumGridOriginUs, drumBoundary, drumDivision) < now) {
    ++drumBoundary;
  }
  const uint64_t drumNextUs = swungGridTimeUs(drumGridOriginUs, drumBoundary, drumDivision);
  if (drumNextUs != drumNextStepUs) {
    drumGlobalStep = drumBoundary;
    drumNextStepUs = drumNextUs;
  }
}

void releaseArpPassthroughClaim(uint8_t sourcePort, uint8_t inNote) {
  uint8_t *mappedNotes = loopOwnsInput(sourcePort)
      ? mappedLoopArpOffNotes[loopTrackForSource(sourcePort)] : mappedArpOffNotes;
  uint8_t *mappedChannels = loopOwnsInput(sourcePort)
      ? mappedLoopArpOffChannels[loopTrackForSource(sourcePort)] : mappedArpOffChannels;
  if (mappedNotes[inNote] > 127) return;
  arpOutputRefOff(sourcePort, mappedNotes[inNote]);
  mappedNotes[inNote] = 0xFF;
  mappedChannels[inNote] = 0;
}

void noteArpOffPassthrough(uint8_t sourcePort, uint8_t inNote, uint8_t velocity, bool on) {
  // A release always runs, whatever the routing or the arp mode looks like now.
  // Bailing out because the arp was switched on, or a channel was disabled or
  // moved, would leave the claim counted forever.
  if (!on) {
    releaseArpPassthroughClaim(sourcePort, inNote);
    return;
  }
  if (currentArpSelection() != ARPSEL_OFF) return;
  const bool drumSplit = arpChannelSplitMode() && splitDrumInputNote(inNote);
  const uint8_t baseOutCh = drumSplit ? 10 : mainArpOutChannel();
  if (!channelEnabled(baseOutCh)) return;
  if (baseOutCh == effectiveThruChannel()) return;
  // A second Note On for a note this source already holds replaces the old
  // claim rather than abandoning it.
  releaseArpPassthroughClaim(sourcePort, inNote);
  uint8_t *mappedNotes = loopOwnsInput(sourcePort)
      ? mappedLoopArpOffNotes[loopTrackForSource(sourcePort)] : mappedArpOffNotes;
  uint8_t *mappedChannels = loopOwnsInput(sourcePort)
      ? mappedLoopArpOffChannels[loopTrackForSource(sourcePort)] : mappedArpOffChannels;
  const uint8_t q = drumSplit ? inNote : quantizeUp(inNote);
  mappedNotes[inNote] = q;
  mappedChannels[inNote] = arpOutputRefOn(sourcePort, q, baseOutCh, !drumSplit, velocity);
}

// The scheduling grid only used to clear reactively, on the next note-off
// event after everything went idle. A performer who stops the looper with no
// notes currently held, the ordinary case, produced no such event, so the
// grid lingered at a nonzero timestamp indefinitely. arpAnyPlaybackActive()
// treats that as the engine still running, which permanently blocked every
// flash write, presets and the loop file alike, until an unrelated note
// happened to toggle it. Called every arp tick now, so it self-heals within
// one pass of the looper actually going idle, regardless of which of the
// many stop paths, push, sensor, CC, panic, or a menu action, got it there.
void releaseArpClockIfLooperIdle() {
  if (multitrackLooper.playing() || multitrackLooper.recording() ||
      multitrackLooper.recordingArmed()) return;
  if (anyPhysicalInputNotesHeld() || heldDrumCount > 0) return;
  arpHadKeys = false;
  arpGateOffMs = 0;
  arpNextStepUs = 0;
  drumGateOffMs = 0;
  drumNextStepUs = 0;
}

void loopAllOff() {
  multitrackLooper.stop(releaseMultitrackOutput, nullptr);
  releaseArpClockIfLooperIdle();
}

void handleLoopRecPlayTrigger() {
  handleMultitrackRecPlay();
}

void handleLoopStopDeleteTrigger() {
  handleMultitrackStopDelete();
}

uint32_t multitrackFixedLengthUs(uint8_t track) {
  if (track >= arpnmidi3::kLoopTrackCount) return 0;
  const uint8_t selection = loopTrackLengthSelection[track];
  if (track == 0 && selection == 6) return 0;  // Free
  uint64_t oneBarUs = musicalDurationUs(
      static_cast<uint64_t>(MUSICAL_PPQN) * (firmware3Settings.timeSignature ? 3ULL : 4ULL));
  if (loopTrackLengthSelection[0] == 6 && multitrackLooper.track(0).lengthUs > 0) {
    oneBarUs = multitrackLooper.track(0).lengthUs;
  }
  switch (min<uint8_t>(selection, 5)) {
    case 0: oneBarUs /= 4ULL; break;
    case 1: oneBarUs /= 2ULL; break;
    case 3: oneBarUs *= 2ULL; break;
    case 4: oneBarUs *= 4ULL; break;
    case 5: oneBarUs *= 8ULL; break;
    default: break;
  }
  return static_cast<uint32_t>(min<uint64_t>(UINT32_MAX, max<uint64_t>(1, oneBarUs)));
}

bool setLoopTrackLengthSelection(uint8_t track, uint8_t selection) {
  if (track >= arpnmidi3::kLoopTrackCount) return false;
  selection = clampU8(selection, 0, track == 0 ? 6 : 5);
  if (selection == loopTrackLengthSelection[track]) return true;
  if ((multitrackLooper.recording() || multitrackLooper.recordingArmed()) &&
      multitrackLooper.recordingTrack() == track) return false;

  const uint8_t previous = loopTrackLengthSelection[track];
  loopTrackLengthSelection[track] = selection;
  const uint32_t targetLengthUs = multitrackFixedLengthUs(track);
  if (multitrackLooper.track(track).count > 0 && targetLengthUs > 0 &&
      targetLengthUs != multitrackLooper.track(track).lengthUs &&
      !multitrackLooper.resizeTrack(track, targetLengthUs, time_us_64(),
                                    releaseMultitrackOutput, nullptr)) {
    loopTrackLengthSelection[track] = previous;
    return false;
  }
  if (multitrackLooper.track(track).count > 0) markLoopStorageDirty();
  // The length selection lives in the loop file, not the preset, so
  // markLoopStorageDirty above is what actually persists it, on its own
  // idle-gated schedule. A stray preset save here fired on every tick while
  // scrolling this field for a value the preset was never going to store.
  refreshLoopUiState();
  return true;
}

void adoptFreeTrackOneTempo() {
  if (loopTrackLengthSelection[0] != 6 || multitrackLooper.track(0).lengthUs == 0) return;
  const uint32_t beats = firmware3Settings.timeSignature ? 3U : 4U;
  const uint64_t numerator = 60000000ULL * beats;
  settings.manualBpm = constrain(static_cast<int>((numerator +
      multitrackLooper.track(0).lengthUs / 2ULL) /
      multitrackLooper.track(0).lengthUs), 20, 300);
  syncMusicalClockConfig(true);
}

uint8_t loopTrackQuantizeSelection(uint8_t track) {
  if (track >= arpnmidi3::kLoopTrackCount) return 0;
  return clampU8(firmware3Settings.looperQuantize[track], 0, LOOP_QUANTIZE_DIVISION_COUNT);
}

// Quantize belongs to the track being written, so a drum part can land on a
// grid while a pad stays free.
uint32_t multitrackQuantizeUs(uint8_t track) {
  const uint8_t selection = loopTrackQuantizeSelection(track);
  if (selection == 0) return 0;
  const uint8_t division = DIV_1_4 + (selection - 1);
  return static_cast<uint32_t>(min<uint64_t>(UINT32_MAX,
      musicalDurationUs(kDivisionPulseSteps[division])));
}

void refreshLoopUiState() {
  ui.dirty = true;
}

void releaseMultitrackOutput(void *, uint8_t track) {
  if (track >= arpnmidi3::kLoopTrackCount) return;
  const uint8_t sourcePort = LOOP_TRACK_SOURCE_BASE + track;
  bool anyReleased = false;
  for (uint8_t channel = 0; channel < 16; ++channel) {
    for (uint8_t noteByte = 0; noteByte < 16; ++noteByte) {
      const uint8_t held = multitrackPlaybackHeld[track][channel][noteByte];
      multitrackPlaybackHeld[track][channel][noteByte] = 0;
      for (uint8_t bit = 0; bit < 8; ++bit) {
        if ((held & (1U << bit)) == 0) continue;
        const uint8_t note = noteByte * 8 + bit;
        routeIncomingChannelMessage(sourcePort, 0x80 | channel, note, 0);
        // The message above only reaches setInputOwnerState (arp/bass/legato's
        // held-note bookkeeping) when the note's channel still matches the
        // current input channel setting. A note recorded on some other
        // channel, or one whose channel no longer matches because the
        // setting changed after it was recorded, would otherwise leave this
        // track's ownership of it stuck true forever: bass in particular
        // reads that as a note that never lets go. Clearing it here does not
        // depend on which branch the routed message happened to take.
        setInputOwnerState(sourcePort, note, 0, false);
        anyReleased = true;
      }
    }
  }
  multitrackPlaybackHeldCount[track] = 0;
  if (anyReleased) updateBassVoice();
}

// Every downstream owner of a loop note (thru mapping, chord extras, arp
// mapping, and the final output reference count) assumes one Note Off per Note
// On for a given track, channel, and note.  Recorded material cannot guarantee
// that on its own: overlapping duplicate notes from chords, the arp, or drum
// generation collapse into a single stored Note Off, and an overdub pass can
// store a Note Off whose Note On was recorded in an earlier pass.  This gate
// makes the emitted stream balanced no matter what the track holds.
void emitMultitrackEvent(void *, uint8_t track, const arpnmidi3::LoopMidiEvent &event) {
  if (track >= arpnmidi3::kLoopTrackCount) return;
  const uint8_t type = event.status & 0xF0;
  const uint8_t sourcePort = LOOP_TRACK_SOURCE_BASE + track;
  if ((type == 0x90 || type == 0x80) && event.data1 <= 127) {
    uint8_t &bits = multitrackPlaybackHeld[track][event.status & 0x0F][event.data1 >> 3];
    const uint8_t mask = static_cast<uint8_t>(1U << (event.data1 & 0x07));
    const bool sounding = (bits & mask) != 0;
    if (type == 0x90 && event.data2 > 0) {
      if (sounding) {
        // Retrigger: release the sounding copy first so ownership stays paired.
        routeIncomingChannelMessage(sourcePort,
                                    static_cast<uint8_t>(0x80 | (event.status & 0x0F)),
                                    event.data1, 0);
      } else {
        ++multitrackPlaybackHeldCount[track];
      }
      bits |= mask;
    } else {
      if (!sounding) return;  // Nothing is holding this note, so drop the orphan.
      bits &= static_cast<uint8_t>(~mask);
      if (multitrackPlaybackHeldCount[track] > 0) --multitrackPlaybackHeldCount[track];
    }
  }
  routeIncomingChannelMessage(sourcePort, event.status, event.data1, event.data2);
}

// A track's cursor keeps stepping through its own recorded events every tick
// no matter why it went quiet, muted, soloed out, or hidden, so by the time it
// regains audibility the data itself has moved on. Picking up only the next
// note-on from there would skip whatever was already mid-note. This replays
// the track's own held-note state through the normal emit path, so recovery
// sounds like the part was playing the whole time even though it is
// technically a fresh trigger, and everything downstream, thru claims, the
// held-note bookkeeping above, arp passthrough, sees an ordinary Note On.
void retriggerLoopTrackHeldNotes(uint8_t track) {
  if (!multitrackLooper.playing()) return;
  multitrackLooper.collectHeldNotes(
      track,
      [](void *, uint8_t trackIndex, uint8_t channel, uint8_t note, uint8_t velocity) {
        emitMultitrackEvent(nullptr, trackIndex,
            arpnmidi3::LoopMidiEvent{0, static_cast<uint8_t>(0x90 | channel), note, velocity});
      },
      nullptr);
}

void setLoopTrackMuted(uint8_t track, bool muted) {
  const bool wasAudible = multitrackLooper.audible(track);
  multitrackLooper.setMuted(track, muted, releaseMultitrackOutput, nullptr);
  if (!wasAudible && multitrackLooper.audible(track)) retriggerLoopTrackHeldNotes(track);
}

// Solo is exclusive: enabling it for one track changes every track's
// audibility at once, so every track is checked before and after, not just
// the one the performer touched.
void setExclusiveLoopSolo(uint8_t soloTrack, bool enable) {
  bool wasAudible[arpnmidi3::kLoopTrackCount];
  for (uint8_t i = 0; i < arpnmidi3::kLoopTrackCount; ++i) {
    wasAudible[i] = multitrackLooper.audible(i);
  }
  for (uint8_t i = 0; i < arpnmidi3::kLoopTrackCount; ++i) {
    multitrackLooper.setSolo(i, enable && i == soloTrack, releaseMultitrackOutput, nullptr);
  }
  for (uint8_t i = 0; i < arpnmidi3::kLoopTrackCount; ++i) {
    if (!wasAudible[i] && multitrackLooper.audible(i)) retriggerLoopTrackHeldNotes(i);
  }
}

// Undo shares the same mechanism as mute and solo: a hidden track's cursor
// keeps advancing the same as a muted one, so bringing it back benefits from
// the same retrigger.
void undoLoopTrackClear(uint8_t track) {
  const bool wasAudible = multitrackLooper.audible(track);
  multitrackLooper.undoClear(track);
  const bool nowAudible = multitrackLooper.audible(track);
  // A transport stopped because nothing was left audible (or for any other
  // reason) has to resume before retrigger can put anything back: retrigger
  // itself does nothing on a stopped loop. Only resume if this undo is what
  // actually made the track audible again, not on an undo of an empty or
  // still-inaudible one.
  if (nowAudible && !multitrackLooper.playing()) multitrackLooper.resume(time_us_64());
  if (!wasAudible && nowAudible) retriggerLoopTrackHeldNotes(track);
}

// A track that is no longer playable must not keep notes latched.  Clearing,
// replacing, muting, soloing, and stopping all release through the looper, but
// a track can also lose its events underneath a sounding note, and a silent
// track never reaches the loop boundary that would release it.
void releaseSilencedMultitrackOutputs() {
  for (uint8_t track = 0; track < arpnmidi3::kLoopTrackCount; ++track) {
    if (multitrackPlaybackHeldCount[track] == 0) continue;
    const arpnmidi3::LoopTrackState &state = multitrackLooper.track(track);
    if (multitrackLooper.playing() && state.count > 0 && state.lengthUs > 0 &&
        multitrackLooper.audible(track)) continue;
    releaseMultitrackOutput(nullptr, track);
  }
  // Nothing left to play back is nothing left to stay in sync with: an
  // empty or fully-cleared loop that still reports playing() only fools
  // every "is there a real clock running" check into protecting a grid
  // nobody can hear, so a fresh note quietly locks onto a mystery clock
  // instead of restarting the arp/drum grid like it should. This checks
  // loopTrackHasContent, not audible, on purpose: muting or soloing every
  // track never touches a track's own content or clear state, so this can
  // only ever fire from an actual clear, never strand a deliberately muted
  // performance stopped. undoLoopTrackClear already resumes on its own
  // whenever an undo makes a track audible again, so this never fights it.
  if (multitrackLooper.playing()) {
    bool anyContent = false;
    for (uint8_t track = 0; track < arpnmidi3::kLoopTrackCount; ++track) {
      anyContent |= loopTrackHasContent(track);
    }
    if (!anyContent) multitrackLooper.stop(releaseMultitrackOutput, nullptr);
  }
}

void configureMultitrackLooper() {
  multitrackLooper.setTrackMode(
      static_cast<arpnmidi3::LoopTrackMode>(firmware3Settings.looperTrackMode));
  // The armed or recording track owns the setting; only one track records at a
  // time, so the working track supplies it until a pass starts.
  const uint8_t quantizeTrack = (multitrackLooper.recording() || multitrackLooper.recordingArmed())
      ? multitrackLooper.recordingTrack() : multitrackLooper.selectedTrack();
  multitrackLooper.setRecordQuantizeUs(multitrackQuantizeUs(quantizeTrack));
}

void resetLoopCcPruning() {
  for (LoopCcPruneState &state : loopCcPrune) state = LoopCcPruneState{};
}

bool capturePrunedLoopCcEvent(uint8_t channel, uint8_t cc, uint8_t value) {
  const arpnmidi3::LoopMidiEvent event{0,
      static_cast<uint8_t>(0xB0 | ((channel - 1) & 0x0F)), cc, value};
  if (!multitrackLooper.capture(time_us_64(), event)) return false;
  markLoopStorageDirty();
  return true;
}

void flushLoopCcPruning(bool forceAll) {
  const uint32_t now = millis();
  for (LoopCcPruneState &state : loopCcPrune) {
    if (!state.used || !state.pending) continue;
    if (!forceAll && now - state.lastSeenMs < 80) continue;
    if (capturePrunedLoopCcEvent(state.channel, state.cc, state.lastSeen)) {
      state.lastStored = state.lastSeen;
      state.lastStoredMs = now;
      state.pending = false;
    }
  }
}

void recordLoopCc(uint8_t sourcePort, uint8_t channel, uint8_t cc, uint8_t value) {
  const bool fromLoop = loopOwnsInput(sourcePort);
  if (firmware3Settings.looperRecordCc) {
    const uint8_t target = fromLoop ? loopTrackForSource(sourcePort) + 1U : 0;
    rollingHistory.push(time_us_64(), target, arpnmidi3::LoopMidiEvent{0,
        static_cast<uint8_t>(0xB0 | ((channel - 1) & 0x0F)), cc, value});
  }
  if (fromLoop || !firmware3Settings.looperRecordCc || !multitrackLooper.recording()) return;
  LoopCcPruneState *state = nullptr;
  for (LoopCcPruneState &candidate : loopCcPrune) {
    if (candidate.used && candidate.channel == channel && candidate.cc == cc) {
      state = &candidate;
      break;
    }
    if (!state && !candidate.used) state = &candidate;
  }
  if (!state) {
    ++loopCcPruneOverflowCount;
    return;
  }
  const uint32_t now = millis();
  if (!state->used) {
    *state = LoopCcPruneState{};
    state->used = true;
    state->channel = channel;
    state->cc = cc;
    state->lastStored = state->lastSeen = state->filtered = value;
    state->lastStoredMs = state->lastSeenMs = now;
    capturePrunedLoopCcEvent(channel, cc, value);
    return;
  }
  const int delta = static_cast<int>(value) - state->lastSeen;
  const int8_t direction = delta > 0 ? 1 : (delta < 0 ? -1 : 0);
  const bool directionChanged = direction != 0 && state->direction != 0 && direction != state->direction;
  state->filtered = static_cast<uint8_t>((static_cast<uint16_t>(state->filtered) * 2U + value + 1U) / 3U);
  state->lastSeen = value;
  state->lastSeenMs = now;
  state->pending = value != state->lastStored;
  const uint8_t distance = static_cast<uint8_t>(abs(static_cast<int>(state->filtered) - state->lastStored));
  const bool timedSmoothPoint = distance >= 2 && now - state->lastStoredMs >= 25;
  const bool fastMove = abs(delta) >= 6 && now - state->lastStoredMs >= 8;
  if (directionChanged || timedSmoothPoint || fastMove) {
    const uint8_t storedValue = (directionChanged || fastMove) ? value : state->filtered;
    if (capturePrunedLoopCcEvent(channel, cc, storedValue)) {
      state->lastStored = storedValue;
      state->lastStoredMs = now;
      state->pending = value != storedValue;
    }
  }
  if (direction != 0) state->direction = direction;
}

// A cleared track counts as free space, never as material to layer onto. It is
// still distinct from an empty one, because Undo can bring it back.
bool loopTrackHasContent(uint8_t track) {
  return multitrackLooper.trackHasContent(track);
}

bool loopTrackIsCleared(uint8_t track) {
  return track < arpnmidi3::kLoopTrackCount &&
         multitrackLooper.track(track).count > 0 &&
         multitrackLooper.track(track).hidden;
}

void armSelectedMultitrack(bool overdub) {
  configureMultitrackLooper();
  const uint8_t track = multitrackLooper.selectedTrack();
  if (overdub && !multitrackLooper.playing()) {
    multitrackLooper.start(time_us_64());
  }
  multitrackLooper.armRecord(track, multitrackFixedLengthUs(track), overdub);
  resetLoopCcPruning();
  refreshLoopUiState();
  ui.dirty = true;
}

// Every working-track change goes through here.  A record that is armed but not
// yet started follows the selection, so the track shown on screen and the track
// about to be written can never disagree.  A pass that is already recording
// keeps its track: only stopping it can move the write target.
void selectLooperTrack(uint8_t track) {
  if (track >= arpnmidi3::kLoopTrackCount) return;
  const uint8_t previousTrack = multitrackLooper.selectedTrack();
  const bool retarget = multitrackLooper.recordingArmed() &&
                        multitrackLooper.recordingTrack() != track;
  multitrackLooper.selectTrack(track);
  // SELECTD's Stutter/Echo dynamically follows the working track, so a real
  // change here has to tear down whatever it was doing on the old one.
  if (track != previousTrack) releaseSelectdTarget();
  // Loop Mix shows the engine's working track, so its cursor follows every
  // selection change, including the auto-advance after a layer completes.
  if (ui.selectedSetting == SET_MUTE_SOLO && ui.menuMode == MENU_EDIT &&
      muteSoloCursor < arpnmidi3::kLoopTrackCount) {
    muteSoloCursor = track;
  }
  if (retarget) {
    configureMultitrackLooper();
    multitrackLooper.armRecord(track, multitrackFixedLengthUs(track), false);
    resetLoopCcPruning();
  }
  refreshLoopUiState();
  ui.dirty = true;
}

void selectNextAutoLooperTrackAfterCapture() {
  const auto mode = multitrackLooper.trackMode();
  if (mode == arpnmidi3::LoopTrackMode::Manual) return;

  uint8_t target = multitrackLooper.selectedTrack();
  bool foundEmpty = false;
  for (uint8_t i = 0; i < arpnmidi3::kLoopTrackCount; ++i) {
    if (loopTrackHasContent(i)) continue;
    target = i;
    foundEmpty = true;
    break;
  }
  if (!foundEmpty) target = multitrackLooper.oldestPopulatedTrack();
  selectLooperTrack(target);
}

// Layers treats the four tracks as one instrument, so its clear works on all of
// them at once.  The gesture only undoes when there is nothing left to clear,
// which keeps a single press from flipping the whole loop back after one track
// was cleared on its own.
void clearOrUndoAllLoopTracks() {
  bool anyLive = false;
  for (uint8_t track = 0; track < arpnmidi3::kLoopTrackCount; ++track) {
    anyLive |= loopTrackHasContent(track);
  }
  for (uint8_t track = 0; track < arpnmidi3::kLoopTrackCount; ++track) {
    if (anyLive) multitrackLooper.safeClear(track, releaseMultitrackOutput, nullptr);
    else undoLoopTrackClear(track);
  }
  // A whole-loop clear is a fresh start, so the working track resets to 1
  // along with it. Undo restores content, not the selection, so it leaves
  // wherever the working track already was alone.
  if (anyLive) selectLooperTrack(0);
}

// Hold-to-panic's clear is one-directional on purpose, unlike the eye/pad's
// stop-then-clear escalation above: it only ever clears whatever is still
// live and leaves any track that's already cleared exactly alone, so it can
// never restore an old take the performer meant to leave gone. safeClear is
// already a no-op on a track with nothing in it, cleared or truly empty.
void clearAllLiveLoopTracks() {
  for (uint8_t track = 0; track < arpnmidi3::kLoopTrackCount; ++track) {
    multitrackLooper.safeClear(track, releaseMultitrackOutput, nullptr);
  }
  selectLooperTrack(0);
}

bool finishActiveMultitrackRecording(uint64_t nowUs, bool startIfStopped) {
  if (!multitrackLooper.recording() && !multitrackLooper.recordingArmed()) return false;
  flushLoopCcPruning(true);
  const bool captured = multitrackLooper.finishRecording(nowUs);
  if (captured) markLoopStorageDirty();
  if (captured && multitrackLooper.recordingTrack() == 0) adoptFreeTrackOneTempo();
  if (captured && startIfStopped && !multitrackLooper.playing()) {
    multitrackLooper.start(nowUs);
  }
  if (captured) selectNextAutoLooperTrackAfterCapture();
  return captured;
}

// Loop Mix Arm mode. Picking a track selects it and arms a record on it, or
// takes the arm back off if that track is already the pending target. A stopped
// or paused transport also starts here, the same way the master trigger does,
// so the mix screen is a complete way to work.
void toggleLooperArmForTrack(uint8_t track) {
  if (track >= arpnmidi3::kLoopTrackCount) return;
  if ((multitrackLooper.recordingArmed() || multitrackLooper.recording()) &&
      multitrackLooper.recordingTrack() == track) {
    finishActiveMultitrackRecording(time_us_64(), true);
    return;
  }
  selectLooperTrack(track);
  if (!multitrackLooper.playing() && multitrackLooper.hasAnyData()) {
    multitrackLooper.resume(time_us_64());
  }
  armSelectedMultitrack(false);
}

// Clicking Solo or Mute when that mode is already in force resets the whole
// mix: every track comes back unmuted and unsoloed. It is the one action on
// this screen that is not about a single picked track.
void resetLoopMixMuteAndSolo() {
  bool wasAudible[arpnmidi3::kLoopTrackCount];
  for (uint8_t track = 0; track < arpnmidi3::kLoopTrackCount; ++track) {
    wasAudible[track] = multitrackLooper.audible(track);
  }
  for (uint8_t track = 0; track < arpnmidi3::kLoopTrackCount; ++track) {
    multitrackLooper.setMuted(track, false, releaseMultitrackOutput, nullptr);
    multitrackLooper.setSolo(track, false, releaseMultitrackOutput, nullptr);
  }
  for (uint8_t track = 0; track < arpnmidi3::kLoopTrackCount; ++track) {
    if (!wasAudible[track] && multitrackLooper.audible(track)) {
      retriggerLoopTrackHeldNotes(track);
    }
  }
  markLoopStorageDirty();
  releaseSilencedMultitrackOutputs();
  refreshLoopUiState();
}

void applyLoopMixModeToTrack(uint8_t track) {
  if (track >= arpnmidi3::kLoopTrackCount) return;
  if (loopMixMode == LOOP_MIX_SOLO) {
    const bool enable = !multitrackLooper.track(track).solo;
    setExclusiveLoopSolo(track, enable);
    markLoopStorageDirty();
  } else if (loopMixMode == LOOP_MIX_MUTE) {
    setLoopTrackMuted(track, !multitrackLooper.track(track).muted);
    markLoopStorageDirty();
  } else if (loopMixMode == LOOP_MIX_CLEAR) {
    // Clear and undo are the same gesture: a track that was cleared and not
    // recorded over comes back on the next press.
    if (multitrackLooper.track(track).hidden) undoLoopTrackClear(track);
    else multitrackLooper.safeClear(track, releaseMultitrackOutput, nullptr);
    selectLooperTrack(track);
    markLoopStorageDirty();
  } else {
    toggleLooperArmForTrack(track);
  }
  releaseSilencedMultitrackOutputs();
  refreshLoopUiState();
}

bool beginTimeTravelImport() {
  const uint8_t track = multitrackLooper.selectedTrack();
  uint32_t lengthUs = multitrackFixedLengthUs(track);
  if (lengthUs == 0) {
    // A retrospective loop needs a known window. Free Track 1 therefore uses
    // one bar for this capture and adopts the normal one-bar selection.
    loopTrackLengthSelection[track] = 2;
    lengthUs = multitrackFixedLengthUs(track);
  }
  const uint64_t boundaryUs = time_us_64();
  const arpnmidi3::HistorySnapshot snapshot = rollingHistory.snapshot(boundaryUs, lengthUs, 0);
  if (snapshot.empty()) return false;

  timeTravelImport = TimeTravelImportJob{};
  timeTravelImport.snapshot = snapshot;
  timeTravelImport.boundaryUs = boundaryUs;
  timeTravelImport.lengthUs = lengthUs;
  timeTravelImport.track = track;
  timeTravelImport.wasPlaying = multitrackLooper.playing();
  timeTravelImport.active = true;
  multitrackLooper.beginImport(track, lengthUs, releaseMultitrackOutput, nullptr);
  refreshLoopUiState();
  ui.dirty = true;
  return true;
}

bool timeTravelNoteHeld(uint8_t channel, uint8_t note) {
  return (timeTravelImport.heldNotes[channel][note >> 3] &
          static_cast<uint8_t>(1U << (note & 0x07))) != 0;
}

void setTimeTravelNoteHeld(uint8_t channel, uint8_t note, bool held) {
  uint8_t &bits = timeTravelImport.heldNotes[channel][note >> 3];
  const uint8_t mask = static_cast<uint8_t>(1U << (note & 0x07));
  if (held) bits |= mask;
  else bits &= static_cast<uint8_t>(~mask);
}

void finishTimeTravelImport() {
  multitrackLooper.finishImport(timeTravelImport.track, timeTravelImport.boundaryUs);
  if (!timeTravelImport.wasPlaying && multitrackLooper.hasAnyData()) {
    multitrackLooper.start(timeTravelImport.boundaryUs);
  }
  timeTravelImport.active = false;
  markLoopStorageDirty();
  refreshLoopUiState();
  ui.dirty = true;
}

void tickTimeTravelImport() {
  if (!timeTravelImport.active) return;
  constexpr uint16_t kMaxWorkPerTick = 48;
  uint16_t work = 0;
  while (!timeTravelImport.closingNotes && work < kMaxWorkPerTick) {
    arpnmidi3::LoopMidiEvent event;
    if (!rollingHistory.readNext(timeTravelImport.snapshot, event)) {
      timeTravelImport.closingNotes = true;
      break;
    }
    ++work;
    const uint8_t type = event.status & 0xF0;
    if ((type == 0x90 || type == 0x80) && event.data1 <= 127) {
      const uint8_t channel = event.status & 0x0F;
      const bool on = type == 0x90 && event.data2 > 0;
      if (!on && !timeTravelNoteHeld(channel, event.data1)) continue;
      setTimeTravelNoteHeld(channel, event.data1, on);
      if (!on) {
        event.status = static_cast<uint8_t>(0x80 | channel);
        event.data2 = 0;
      }
    }
    if (multitrackLooper.importEvent(timeTravelImport.track, event)) {
      ++timeTravelImport.importedEvents;
    } else {
      timeTravelImport.overflowed = true;
    }
  }

  while (timeTravelImport.closingNotes && work < kMaxWorkPerTick &&
         timeTravelImport.closeScan < 16U * 128U) {
    const uint16_t noteIndex = timeTravelImport.closeScan++;
    ++work;
    const uint8_t channel = noteIndex / 128U;
    const uint8_t note = noteIndex % 128U;
    if (!timeTravelNoteHeld(channel, note)) continue;
    const arpnmidi3::LoopMidiEvent noteOff{timeTravelImport.lengthUs - 1U,
        static_cast<uint8_t>(0x80 | channel), note, 0};
    if (multitrackLooper.importEvent(timeTravelImport.track, noteOff)) {
      ++timeTravelImport.importedEvents;
    } else {
      timeTravelImport.overflowed = true;
    }
    setTimeTravelNoteHeld(channel, note, false);
  }
  if (timeTravelImport.closingNotes && timeTravelImport.closeScan >= 16U * 128U) {
    finishTimeTravelImport();
  }
}

// Two triggers inside the window count as one double gesture, from whichever
// source drove them: the push, the sensor, a mapped CC, or a button.
bool loopMasterDoubleTap() {
  const uint32_t now = millis();
  const bool isDouble = loopMasterLastTriggerMs != 0 &&
                        (now - loopMasterLastTriggerMs) <= LOOP_MASTER_DOUBLE_TAP_MS;
  loopMasterLastTriggerMs = isDouble ? 0 : now;
  return isDouble;
}

// The double gesture is the retake control. It safe clears a layer and arms it
// so the part can be played again, and a second double brings the old layer
// back and plays it. Layers steps back to the layer just recorded to do
// either of those, because that is the one a performer means, but the working
// track goes no further than that: undo restores content rather than
// capturing it, so it must not also advance to another track afterward.
// Manual and Parts Auto Solo stay on the working track throughout: moving
// between tracks there is the performer's decision alone.
void handleMultitrackMasterDoubleTap() {
  const uint64_t nowUs = time_us_64();
  const bool layers = multitrackLooper.trackMode() == arpnmidi3::LoopTrackMode::Layers;
  if (multitrackLooper.recording()) finishActiveMultitrackRecording(nowUs, true);
  else if (multitrackLooper.recordingArmed()) multitrackLooper.cancelRecording();

  uint8_t target = multitrackLooper.selectedTrack();
  if (layers && !loopTrackIsCleared(target) && !loopTrackHasContent(target)) {
    target = multitrackLooper.newestPopulatedTrack();
  }

  if (loopTrackIsCleared(target)) {
    const bool wasAudible = multitrackLooper.audible(target);
    multitrackLooper.undoClear(target);
    selectLooperTrack(target);
    if (!multitrackLooper.playing()) multitrackLooper.resume(nowUs);
    // The retrigger check runs after resume, once the cursor is actually
    // positioned for right now rather than wherever it was left paused.
    if (!wasAudible && multitrackLooper.audible(target)) {
      retriggerLoopTrackHeldNotes(target);
    }
    // Undo restores content; it does not capture anything, so it must not
    // move the working track. Only a layer that actually captured something
    // does that, the same rule as everywhere else in the looper.
  } else {
    selectLooperTrack(target);
    multitrackLooper.safeClear(target, releaseMultitrackOutput, nullptr);
    armSelectedMultitrack(false);
  }
  releaseSilencedMultitrackOutputs();
  markLoopStorageDirty();
  refreshLoopUiState();
  ui.dirty = true;
}

void handleMultitrackRecPlay() {
  const uint64_t nowUs = time_us_64();
  configureMultitrackLooper();
  if (loopMasterDoubleTap()) {
    handleMultitrackMasterDoubleTap();
    return;
  }
  if (firmware3Settings.looperTimeTravel && !multitrackLooper.recordingArmed() &&
      !multitrackLooper.recording()) {
    beginTimeTravelImport();
    return;
  }
  if (multitrackLooper.recordingArmed() || multitrackLooper.recording()) {
    finishActiveMultitrackRecording(nowUs, true);
  } else if (!multitrackLooper.hasAnyData()) {
    armSelectedMultitrack(false);
  } else if (!multitrackLooper.playing()) {
    multitrackLooper.start(nowUs);
  } else {
    const auto mode = multitrackLooper.trackMode();
    const uint8_t target = multitrackLooper.selectedTrack();
    armSelectedMultitrack(loopTrackHasContent(target));
    if (mode == arpnmidi3::LoopTrackMode::PartsAutoSolo) {
      setExclusiveLoopSolo(target, true);
      markLoopStorageDirty();
    }
  }
  refreshLoopUiState();
  ui.dirty = true;
}

void handleMultitrackStopDelete() {
  const uint8_t track = multitrackLooper.selectedTrack();
  if (multitrackLooper.recording() || multitrackLooper.recordingArmed()) {
    finishActiveMultitrackRecording(time_us_64(), false);
  }
  if (multitrackLooper.playing()) {
    multitrackLooper.stop(releaseMultitrackOutput, nullptr);
    loopSafeClearArmed = true;
  } else if (loopSafeClearArmed && multitrackLooper.usedEvents() > 0) {
    if (multitrackLooper.trackMode() == arpnmidi3::LoopTrackMode::Layers) {
      clearOrUndoAllLoopTracks();
    } else if (multitrackLooper.track(track).hidden) {
      undoLoopTrackClear(track);
    } else {
      multitrackLooper.safeClear(track, releaseMultitrackOutput, nullptr);
    }
    loopSafeClearArmed = false;
    markLoopStorageDirty();
  } else {
    loopSafeClearArmed = multitrackLooper.trackMode() == arpnmidi3::LoopTrackMode::Layers
        ? multitrackLooper.usedEvents() > 0 : multitrackLooper.track(track).count > 0;
  }
  refreshLoopUiState();
  ui.dirty = true;
}

void recordLoopNote(uint8_t sourcePort, uint8_t channel1, uint8_t note, uint8_t velocity, bool on) {
  const uint64_t nowUs = time_us_64();
  const arpnmidi3::LoopMidiEvent event{0,
      static_cast<uint8_t>((on && velocity > 0 ? 0x90 : 0x80) | ((channel1 - 1) & 0x0F)),
      note, static_cast<uint8_t>(on ? velocity : 0)};
  const uint8_t historyTarget = loopOwnsInput(sourcePort) ? loopTrackForSource(sourcePort) + 1U : 0;
  rollingHistory.push(nowUs, historyTarget, event);
  if (!loopOwnsInput(sourcePort)) {
    // Auto Arm keeps the working track armed and waiting, so nothing special
    // happens here on the note that starts a take: it is captured by the same
    // path as any other armed recording, below.
    if (multitrackLooper.capture(nowUs, event)) markLoopStorageDirty();
    // No refresh here. A captured note changes nothing the screen shows, and
    // this runs once per played note while recording.
  }
}

void tickLooper() {
  tickTimeTravelImport();
  const bool wasRecording = multitrackLooper.recording();
  flushLoopCcPruning(false);
  configureMultitrackLooper();
  multitrackLooper.tick(time_us_64(), emitMultitrackEvent,
                        releaseMultitrackOutput, nullptr);
  if (wasRecording && !multitrackLooper.recording()) {
    // A fixed length ran out on its own.  Treat it exactly like a hand-stopped
    // pass: only a layer that actually captured something advances the working
    // track, so an empty timed pass leaves the performer where they were.
    // This is also the one and only moment Auto Arm acts: a track was
    // recording and reached its length, nothing else. Manual mode never
    // advances the selection, so arming the same track here is a continuous
    // auto-overdub. Layers and Parts Auto Solo already moved to the next
    // track above, so arming there is what makes that track waiting the
    // instant it becomes the selection.
    const uint8_t recorded = multitrackLooper.recordingTrack();
    const bool captured = multitrackLooper.track(recorded).count > 0;
    if (captured) markLoopStorageDirty();
    if (captured) {
      if (recorded == 0) adoptFreeTrackOneTempo();
      selectNextAutoLooperTrackAfterCapture();
      if (firmware3Settings.looperAutoRec) {
        const bool manual = multitrackLooper.trackMode() == arpnmidi3::LoopTrackMode::Manual;
        armSelectedMultitrack(manual);
      }
    }
    refreshLoopUiState();
  }
  releaseSilencedMultitrackOutputs();
}

void tickEcho() {
  for (uint8_t target = 0; target < STUTTER_ECHO_TARGET_COUNT; ++target) {
    const bool enabled = firmware3Settings.liveTargets[target].echoEnabled != 0;
    if (!enabled && echoSettingWasEnabled[target]) {
      echoEngine.stopTarget(target, emitEchoEvent, nullptr);
    }
    echoSettingWasEnabled[target] = enabled;
  }
  echoEngine.tick(time_us_64(), emitEchoEvent, nullptr);
}

void tickNoteLength() {
  for (uint8_t target = 0; target < LIVE_TARGET_COUNT; ++target) {
    const bool enabled = firmware3Settings.liveTargets[target].noteLengthEnabled != 0;
    if (!enabled && noteLengthSettingWasEnabled[target]) {
      noteLengthEngine.stopTarget(target, emitNoteLengthEvent, nullptr);
    }
    noteLengthSettingWasEnabled[target] = enabled;
  }
  noteLengthEngine.tick(time_us_64(), emitNoteLengthEvent, nullptr);
}

uint8_t lengthSelectionForDivision(uint8_t division) {
  return STUTTER_LENGTH_DIVISION_BASE + clampU8(division, 0, DIVISION_COUNT - 1);
}

String lengthSelectionName(uint8_t selection) {
  static const char *const barNames[STUTTER_BAR_LENGTH_COUNT] = {
    "8 BARS", "4 BARS", "2 BARS", "1 BAR"
  };
  selection = clampU8(selection, 0, STUTTER_LENGTH_COUNT - 1);
  if (selection < STUTTER_BAR_LENGTH_COUNT) return String(barNames[selection]);
  return String(kDivisionNames[selection - STUTTER_LENGTH_DIVISION_BASE]);
}

// A one-line summary has no room for the grammatically correct plural, so
// this always says BAR, never BARS, unlike lengthSelectionName above, which
// detail views with room to spare still use as-is.
String compactLengthSelectionName(uint8_t selection) {
  static const char *const compactBarNames[STUTTER_BAR_LENGTH_COUNT] = {
    "8 BAR", "4 BAR", "2 BAR", "1 BAR"
  };
  selection = clampU8(selection, 0, STUTTER_LENGTH_COUNT - 1);
  if (selection < STUTTER_BAR_LENGTH_COUNT) return String(compactBarNames[selection]);
  return String(kDivisionNames[selection - STUTTER_LENGTH_DIVISION_BASE]);
}

uint64_t lengthSelectionPulses(uint8_t selection) {
  selection = clampU8(selection, 0, STUTTER_LENGTH_COUNT - 1);
  if (selection < STUTTER_BAR_LENGTH_COUNT) {
    static constexpr uint8_t barMultipliers[STUTTER_BAR_LENGTH_COUNT] = {8, 4, 2, 1};
    const uint64_t pulsesPerBar = static_cast<uint64_t>(MUSICAL_PPQN) *
        (firmware3Settings.timeSignature ? 3ULL : 4ULL);
    return pulsesPerBar * barMultipliers[selection];
  }
  return kDivisionPulseSteps[selection - STUTTER_LENGTH_DIVISION_BASE];
}

uint32_t stutterLengthUs(uint8_t target) {
  const uint8_t selection = clampU8(
      firmware3Settings.liveTargets[target].stutterLengthSelection,
      0, STUTTER_LENGTH_COUNT - 1);
  return static_cast<uint32_t>(min<uint64_t>(UINT32_MAX,
      musicalDurationUs(lengthSelectionPulses(selection))));
}

bool activateStutter(uint8_t target, uint64_t nowUs) {
  if (target >= STUTTER_ECHO_TARGET_COUNT) return false;
  const uint32_t lengthUs = stutterLengthUs(target);
  if (!stutterRepeaters[target].activate(rollingHistory, nowUs, lengthUs,
                                        HISTORY_OUTPUT_TARGET_BASE + target)) return false;
  echoEngine.stopTarget(target, emitEchoEvent, nullptr);
  releaseFinalTarget(target);
  activeStutterLengthSelection[target] =
      firmware3Settings.liveTargets[target].stutterLengthSelection;
  const uint64_t barPulses = static_cast<uint64_t>(MUSICAL_PPQN) *
      (firmware3Settings.timeSignature ? 3ULL : 4ULL);
  stutterStopUs[target] = nowUs + musicalDurationUs(barPulses) *
      firmware3Settings.stutterTimeoutBars;
  stutterTimedOut[target] = false;
  return true;
}

void deactivateStutter(uint8_t target) {
  if (target >= STUTTER_ECHO_TARGET_COUNT) return;
  stutterRepeaters[target].deactivate(emitStutterEvent, nullptr);
}

// SELECTD's stutter/echo state is tied to whichever track is currently
// selected, not to a track of its own, so the moment that changes its old
// state has to be torn down: otherwise its repeater would keep replaying the
// old track's captured notes, or its echo tails and held output refs would
// just hang with no note-off ever coming from a track that is no longer
// selected. stutterSettingWasEnabled is reset too, not just deactivated,
// since tickStutter only reactivates a target on the false-to-true edge of
// its enabled setting, which never fires here, this stutter's own enabled
// flag never actually changed.
void releaseSelectdTarget() {
  deactivateStutter(SELECTD_LIVE_TARGET);
  stutterSettingWasEnabled[SELECTD_LIVE_TARGET] = false;
  stutterTimedOut[SELECTD_LIVE_TARGET] = false;
  echoEngine.stopTarget(SELECTD_LIVE_TARGET, emitEchoEvent, nullptr);
  releaseFinalTarget(SELECTD_LIVE_TARGET);
}

void tickStutter() {
  const uint64_t nowUs = time_us_64();
  for (uint8_t target = 0; target < STUTTER_ECHO_TARGET_COUNT; ++target) {
    const bool enabled = firmware3Settings.liveTargets[target].stutterEnabled != 0;
    if (!enabled) {
      if (stutterRepeaters[target].active()) deactivateStutter(target);
      stutterSettingWasEnabled[target] = false;
      stutterTimedOut[target] = false;
      continue;
    }
    if (!stutterSettingWasEnabled[target]) {
      if (!stutterTimedOut[target]) activateStutter(target, nowUs);
      stutterSettingWasEnabled[target] = true;
    } else if (stutterRepeaters[target].active() &&
               activeStutterLengthSelection[target] !=
                   firmware3Settings.liveTargets[target].stutterLengthSelection) {
      deactivateStutter(target);
      activateStutter(target, nowUs);
    }
    if (stutterRepeaters[target].active() && nowUs >= stutterStopUs[target]) {
      deactivateStutter(target);
      stutterTimedOut[target] = true;
    }
    stutterRepeaters[target].tick(nowUs, rollingHistory, emitStutterEvent, nullptr);
  }
}

// A flash write is a musical event, not a background chore. LittleFS disables
// interrupts and parks the second core for the whole erase and program, which
// costs tens of milliseconds with no MIDI input, no display, and no scheduling.
// Writes therefore wait for the engine to be genuinely quiet, however long that
// takes. The dirty markers on the diagnostics screen show what is still owed,
// and stopping the loop is what settles it.
bool storageWriteAlwaysBlocked() {
  return multitrackLooper.recording() || multitrackLooper.recordingArmed() ||
         timeTravelImport.active;
}

bool storageWriteEngineIdle() {
  return !multitrackLooper.playing() && !anyPhysicalInputNotesHeld() &&
         heldDrumCount == 0 && !arpAnyPlaybackActive();
}

bool storageWriteReady(uint32_t dirtySinceMs, uint32_t minimumPendingMs) {
  if (storageWriteAlwaysBlocked() || !storageWriteEngineIdle()) return false;
  return millis() - dirtySinceMs >= minimumPendingMs;
}

void markLoopStorageDirty() {
  loopStorageDirty = true;
  loopStorageDirtyMs = millis();
}

void markExtendedPresetDirty() {
  extendedPresetDirty = true;
  extendedPresetDirtyMs = millis();
}

void pollLoopStoragePersistence() {
  if (!loopStorageDirty || !littleFsReady) return;
  if (millis() < storageRetryHoldUntilMs) return;
  if (!storageWriteReady(loopStorageDirtyMs, 750UL)) return;
  showBusyHourglass();
  saveLoopStorageIfAny();
  endBusyHourglass();
  if (loopStorageDirty) storageRetryHoldUntilMs = millis() + 5000UL;
}

// There is no fast way to make a flash write imperceptible: it disables
// interrupts and parks the rendering core for the same tens of milliseconds
// regardless of how few bytes changed, so a 12-byte header costs the same
// pause as a full preset. The remembered screen is not worth that pause on
// every visit, so it only writes once the screen has sat still for a long
// while AND the engine has nothing going on, five full seconds of neither,
// not a menu commit's single deliberate click.
void pollUiScreenPersistence() {
  const uint32_t now = millis();
  if (ui.selectedSetting != observedUiSetting) {
    observedUiSetting = ui.selectedSetting;
    uiScreenSavePending = (ui.selectedSetting != persistedUiSetting);
    uiScreenChangedMs = now;
  }
  if (!uiScreenSavePending || ui.deferredExitWork || !littleFsReady) return;
  if (millis() < storageRetryHoldUntilMs) return;
  if (!storageWriteReady(uiScreenChangedMs, UI_SCREEN_SAVE_IDLE_MS)) return;
  showBusyHourglass();
  stagePersistedUiSetting(ui.selectedSetting);
  const bool ok = writeDeviceStateHeader();
  endBusyHourglass();
  if (!ok) storageRetryHoldUntilMs = millis() + 5000UL;
}

// The deferred polls exist for changes made while the engine is busy, mapped
// CCs and captures during a performance. A change made from the menu saves at
// the exit click instead and never reaches here.
void pollPresetStoragePersistence() {
  if (!presetStorageDirty || ui.deferredExitWork || !littleFsReady) return;
  if (millis() < storageRetryHoldUntilMs) return;
  if (!storageWriteReady(presetStorageDirtyMs, 750UL)) return;
  saveStorage();
}

void pollExtendedPresetPersistence() {
  if (!extendedPresetDirty || chordLearnActive || customArpLearning ||
      ui.deferredExitWork || !littleFsReady) return;
  if (millis() < storageRetryHoldUntilMs) return;
  if (!storageWriteReady(extendedPresetDirtyMs, 750UL)) return;
  if (savePresetLearnedContent(storage.currentPreset)) extendedPresetDirty = false;
}

void clearSavedLoopStorage() {
  if (littleFsReady) LittleFS.remove("/loops.f3");
  loopStorageDirty = false;
}

uint32_t loopChecksumUpdate(uint32_t checksum, const uint8_t *data, size_t length) {
  for (size_t i = 0; i < length; ++i) {
    checksum ^= data[i];
    checksum *= 16777619UL;
  }
  return checksum;
}

struct LoopSaveContext {
  File *file = nullptr;
  uint32_t checksum = 2166136261UL;
  bool ok = true;
};

bool writeLoopFileEvent(void *raw, uint8_t track, const arpnmidi3::LoopMidiEvent &event) {
  LoopSaveContext &context = *static_cast<LoopSaveContext *>(raw);
  const LoopFileEvent stored{event.atUs, track, event.status, event.data1, event.data2};
  context.checksum = loopChecksumUpdate(context.checksum,
      reinterpret_cast<const uint8_t *>(&stored), sizeof(stored));
  context.ok = context.ok && context.file->write(
      reinterpret_cast<const uint8_t *>(&stored), sizeof(stored)) == sizeof(stored);
  return context.ok;
}

void saveLoopStorageIfAny() {
  if (!littleFsReady) {
    loopStorageError = true;
    return;
  }
  if (multitrackLooper.usedEvents() == 0) {
    clearSavedLoopStorage();
    return;
  }
  File file = LittleFS.open("/loops.tmp", "w+");
  if (!file) {
    loopStorageError = true;
    return;
  }
  LoopFileHeader header{};
  header.eventCount = multitrackLooper.usedEvents();
  header.selectedTrack = multitrackLooper.selectedTrack();
  header.trackMode = static_cast<uint8_t>(multitrackLooper.trackMode());
  for (uint8_t track = 0; track < arpnmidi3::kLoopTrackCount; ++track) {
    const arpnmidi3::LoopTrackState &state = multitrackLooper.track(track);
    header.tracks[track].lengthUs = state.lengthUs;
    header.tracks[track].storedLengthUs = state.storedLengthUs;
    header.tracks[track].generation = state.generation;
    header.tracks[track].flags = (state.muted ? 0x01 : 0) |
                                (state.solo ? 0x02 : 0) |
                                (state.hidden ? 0x04 : 0);
    header.tracks[track].lengthSelection = loopTrackLengthSelection[track];
    header.tracks[track].startPhase = state.lengthUs > 0
        ? static_cast<uint16_t>((static_cast<uint64_t>(state.startOffsetUs) * 65536ULL) /
                                state.lengthUs)
        : 0;
  }
  header.checksum = 0;
  bool ok = file.write(reinterpret_cast<const uint8_t *>(&header), sizeof(header)) == sizeof(header);
  LoopSaveContext context{&file, 2166136261UL, ok};
  if (ok) ok = multitrackLooper.visitEvents(writeLoopFileEvent, &context) && context.ok;
  header.checksum = context.checksum;
  if (ok) ok = file.seek(0, SeekSet) &&
      file.write(reinterpret_cast<const uint8_t *>(&header), sizeof(header)) == sizeof(header);
  file.close();
  if (ok) {
    LittleFS.remove("/loops.f3");
    ok = LittleFS.rename("/loops.tmp", "/loops.f3");
  } else {
    LittleFS.remove("/loops.tmp");
  }
  loopStorageError = !ok;
  if (ok) loopStorageDirty = false;
}

void loadSavedLoopStorage() {
  multitrackLooper.reset();
  static constexpr uint8_t defaultLoopLengths[arpnmidi3::kLoopTrackCount] = {2, 3, 4, 5};
  for (uint8_t track = 0; track < arpnmidi3::kLoopTrackCount; ++track) {
    loopTrackLengthSelection[track] = defaultLoopLengths[track];
  }
  if (!littleFsReady) return;
  File file = LittleFS.open("/loops.f3", "r");
  LoopFileHeader header{};
  bool ok = file && file.read(reinterpret_cast<uint8_t *>(&header), sizeof(header)) == sizeof(header) &&
            header.magic == LOOP_FILE_MAGIC && header.eventCount <= arpnmidi3::kLoopEventPoolSize;
  uint32_t checksum = 2166136261UL;
  for (uint16_t i = 0; ok && i < header.eventCount; ++i) {
    LoopFileEvent stored{};
    ok = file.read(reinterpret_cast<uint8_t *>(&stored), sizeof(stored)) == sizeof(stored) &&
         stored.track < arpnmidi3::kLoopTrackCount;
    if (!ok) break;
    checksum = loopChecksumUpdate(checksum, reinterpret_cast<const uint8_t *>(&stored), sizeof(stored));
    ok = multitrackLooper.restoreEvent(stored.track,
        arpnmidi3::LoopMidiEvent{stored.atUs, stored.status, stored.data1, stored.data2});
  }
  file.close();
  ok = ok && checksum == header.checksum;
  if (!ok) {
    multitrackLooper.reset();
    loopStorageError = true;
    refreshLoopUiState();
    return;
  }
  for (uint8_t track = 0; track < arpnmidi3::kLoopTrackCount; ++track) {
    const LoopFileTrack &saved = header.tracks[track];
    const uint32_t startOffsetUs = static_cast<uint32_t>(
        (static_cast<uint64_t>(saved.startPhase) * saved.lengthUs) / 65536ULL);
    multitrackLooper.setRestoredTrackState(track, saved.lengthUs, saved.storedLengthUs,
        saved.generation,
        saved.flags & 0x01, saved.flags & 0x02, saved.flags & 0x04, startOffsetUs);
    loopTrackLengthSelection[track] = clampU8(saved.lengthSelection, 0, track == 0 ? 6 : 5);
  }
  multitrackLooper.selectTrack(clampU8(header.selectedTrack, 0,
      arpnmidi3::kLoopTrackCount - 1));
  multitrackLooper.setTrackMode(static_cast<arpnmidi3::LoopTrackMode>(
      clampU8(header.trackMode, 0, static_cast<uint8_t>(arpnmidi3::LoopTrackMode::Manual))));
  loopStorageDirty = false;
  loopStorageError = false;
  refreshLoopUiState();
}

// These counters are shared by every source, so a note sounding from two places
// is turned off once. That only holds if each claim is released exactly once.
// A claim is therefore always counted and always released, whatever the channel
// settings happen to be at the time: a decrement skipped because a channel was
// disabled or changed would strand the counter above zero, and from then on
// every Note Off for that output note would be swallowed, from any source and
// any track. The channel a note started on is remembered so it can still be
// released correctly after the setting moves.
void thruOutputRefOn(uint8_t sourcePort, uint8_t outNote, uint8_t velocity) {
  if (outNote > 127) return;
  const uint8_t outCh = effectiveThruChannel();
  const bool sendable = channelEnabled(outCh);
  if (sendable) captureChordMemoryOutput(sourcePort, outCh, outNote, velocity);
  if (thruOutputRefCount[outNote]++ == 0) {
    thruOutputRefChannel[outNote] = sendable ? outCh : 0;
    if (sendable) sendFanout(sourcePort, 0x90 | ((outCh - 1) & 0x0F), outNote, velocity);
  }
}

void thruOutputRefOff(uint8_t sourcePort, uint8_t outNote) {
  if (outNote > 127 || thruOutputRefCount[outNote] == 0) return;
  if (--thruOutputRefCount[outNote] > 0) return;
  const uint8_t outCh = thruOutputRefChannel[outNote];
  thruOutputRefChannel[outNote] = 0;
  if (channelEnabled(outCh)) {
    sendFanout(sourcePort, 0x80 | ((outCh - 1) & 0x0F), outNote, 0);
  }
}

// The arp passthrough owns its output notes the same way.
uint8_t arpOutputRefOn(uint8_t sourcePort, uint8_t outNote, uint8_t baseCh,
                       bool allowRoundRobin, uint8_t velocity) {
  if (outNote > 127) return baseCh;
  if (arpOffOutputRefCount[outNote]++ != 0) return arpOffOutputChannel[outNote];
  const uint8_t outCh = allowRoundRobin ? nextRoundRobinChannel(baseCh) : baseCh;
  const bool sendable = channelEnabled(outCh);
  arpOffOutputChannel[outNote] = sendable ? outCh : 0;
  if (sendable) sendFanout(sourcePort, 0x90 | ((outCh - 1) & 0x0F), outNote, velocity);
  return outCh;
}

void arpOutputRefOff(uint8_t sourcePort, uint8_t outNote) {
  if (outNote > 127 || arpOffOutputRefCount[outNote] == 0) return;
  if (--arpOffOutputRefCount[outNote] > 0) return;
  const uint8_t outCh = arpOffOutputChannel[outNote];
  arpOffOutputChannel[outNote] = 0;
  if (channelEnabled(outCh)) {
    sendFanout(sourcePort, 0x80 | ((outCh - 1) & 0x0F), outNote, 0);
  }
}

void noteThrough(uint8_t sourcePort, uint8_t inNote, uint8_t velocity, bool on) {
  uint8_t *mappedNotes = loopOwnsInput(sourcePort)
      ? mappedLoopThruNotes[loopTrackForSource(sourcePort)] : mappedThruNotes;
  uint8_t (*extraNotes)[3] = loopOwnsInput(sourcePort)
      ? mappedLoopThruChordNotes[loopTrackForSource(sourcePort)] : mappedThruChordNotes;
  uint8_t *extraCount = loopOwnsInput(sourcePort)
      ? mappedLoopThruChordCount[loopTrackForSource(sourcePort)] : mappedThruChordCount;
  if (on) {
    // Replace any claim this source still holds for the note, so the shared
    // output counters keep one release for every claim.
    if (mappedNotes[inNote] <= 127 || extraCount[inNote] > 0) {
      noteThrough(sourcePort, inNote, 0, false);
    }
    uint8_t notes[4];
    const uint8_t count = buildChordNotes(inNote, notes);
    mappedNotes[inNote] = notes[0];
    extraCount[inNote] = count - 1;
    for (uint8_t i = 1; i < count; ++i) extraNotes[inNote][i - 1] = notes[i];
    for (uint8_t i = 0; i < count; ++i) thruOutputRefOn(sourcePort, notes[i], velocity);
  } else {
    const uint8_t q = mappedNotes[inNote];
    if (q <= 127) thruOutputRefOff(sourcePort, q);
    for (uint8_t i = 0; i < extraCount[inNote]; ++i) {
      if (extraNotes[inNote][i] <= 127) thruOutputRefOff(sourcePort, extraNotes[inNote][i]);
      extraNotes[inNote][i] = 0xFF;
    }
    extraCount[inNote] = 0;
    mappedNotes[inNote] = 0xFF;
  }
}

void legatoNoteOffCurrent(uint8_t fallbackChannel = 0) {
  if (!legatoOutputActive || legatoOutputNote > 127) return;
  uint8_t outCh = legatoOutputChannel;
  if (!channelEnabled(outCh)) outCh = fallbackChannel;
  if (!channelEnabled(outCh)) outCh = settings.legatoChannel;
  if (channelEnabled(outCh)) {
    const uint8_t bassCh = bassModeChannel(settings.bassMode);
    const bool bassOwnsSame = channelEnabled(bassCh) &&
                              bassCh == outCh &&
                              currentBassOutNote >= 0 &&
                              static_cast<uint8_t>(currentBassOutNote) == legatoOutputNote;
    if (!bassOwnsSame) {
      sendFanout(legatoOutputSource, 0x80 | ((outCh - 1) & 0x0F), legatoOutputNote, 0);
    }
  }
  legatoOutputActive = false;
  legatoOutputNote = 0xFF;
  legatoOutputVelocity = 0;
  legatoOutputSource = 255;
  legatoOutputChannel = 0;
}

void clearLegatoState(bool sendCurrentOff = false, uint8_t fallbackChannel = 0) {
  if (sendCurrentOff) legatoNoteOffCurrent(fallbackChannel);
  else {
    legatoOutputActive = false;
    legatoOutputNote = 0xFF;
    legatoOutputVelocity = 0;
    legatoOutputSource = 255;
    legatoOutputChannel = 0;
  }
  memset(legatoHeldCount, 0, sizeof(legatoHeldCount));
  memset(legatoHeldVelocity, 0, sizeof(legatoHeldVelocity));
  memset(legatoHeldSource, 255, sizeof(legatoHeldSource));
  memset(legatoHeldOrder, 0, sizeof(legatoHeldOrder));
  legatoOrderCounter = 0;
}

int8_t legatoNewestHeldNote() {
  int8_t winner = -1;
  uint32_t newest = 0;
  for (uint8_t note = 0; note < 128; ++note) {
    if (legatoHeldCount[note] == 0) continue;
    if (winner < 0 || legatoHeldOrder[note] >= newest) {
      winner = static_cast<int8_t>(note);
      newest = legatoHeldOrder[note];
    }
  }
  return winner;
}

void handleLegatoInputNote(uint8_t sourcePort, uint8_t channel1, uint8_t note, uint8_t velocity, bool on) {
  if (!channelEnabled(channel1)) return;
  if (on && velocity > 0) {
    legatoHeldCount[note] = 1;
    legatoHeldVelocity[note] = velocity;
    legatoHeldSource[note] = sourcePort;
    legatoHeldOrder[note] = ++legatoOrderCounter;

    if (legatoOutputActive && legatoOutputChannel == channel1 &&
        legatoOutputNote == note) {
      legatoOutputVelocity = velocity;
      legatoOutputSource = sourcePort;
      return;
    }

    if (legatoOutputActive) legatoNoteOffCurrent(channel1);
    sendFanout(sourcePort, 0x90 | ((channel1 - 1) & 0x0F), note, max<uint8_t>(1, velocity));
    legatoOutputActive = true;
    legatoOutputNote = note;
    legatoOutputVelocity = velocity;
    legatoOutputSource = sourcePort;
    legatoOutputChannel = channel1;
    return;
  }

  // NoteOff path
  {
    legatoHeldCount[note] = 0;
    legatoHeldVelocity[note] = 0;
    legatoHeldSource[note] = sourcePort;
    legatoHeldOrder[note] = 0;
  }

  if (!legatoOutputActive || legatoOutputChannel != channel1 || legatoOutputNote != note) {
    return;
  }

  const int8_t winner = legatoNewestHeldNote();
  if (winner < 0) {
    legatoNoteOffCurrent(channel1);
    return;
  }

  legatoNoteOffCurrent(channel1);
  const uint8_t nextNote = static_cast<uint8_t>(winner);
  const uint8_t nextVel = max<uint8_t>(1, legatoHeldVelocity[nextNote]);
  const uint8_t nextSource = legatoHeldSource[nextNote];
  sendFanout(nextSource, 0x90 | ((channel1 - 1) & 0x0F), nextNote, nextVel);
  legatoOutputActive = true;
  legatoOutputNote = nextNote;
  legatoOutputVelocity = nextVel;
  legatoOutputSource = nextSource;
  legatoOutputChannel = channel1;
}

void clearSplitNoteFromMainPaths(uint8_t sourcePort, uint8_t note) {
  setInputOwnerState(sourcePort, note, 0, false);

  uint8_t *thruMap = loopOwnsInput(sourcePort)
      ? mappedLoopThruNotes[loopTrackForSource(sourcePort)] : mappedThruNotes;
  if (thruMap[note] <= 127) thruOutputRefOff(sourcePort, thruMap[note]);
  thruMap[note] = 0xFF;
  uint8_t (*extraNotes)[3] = loopOwnsInput(sourcePort)
      ? mappedLoopThruChordNotes[loopTrackForSource(sourcePort)] : mappedThruChordNotes;
  uint8_t *extraCount = loopOwnsInput(sourcePort)
      ? mappedLoopThruChordCount[loopTrackForSource(sourcePort)] : mappedThruChordCount;
  for (uint8_t i = 0; i < extraCount[note]; ++i) {
    if (extraNotes[note][i] <= 127) thruOutputRefOff(sourcePort, extraNotes[note][i]);
    extraNotes[note][i] = 0xFF;
  }
  extraCount[note] = 0;

  releaseArpPassthroughClaim(sourcePort, note);
}

uint8_t nextRoundRobinChannel(uint8_t baseCh) {
  if (!channelEnabled(baseCh) || settings.roundRobinMask == 0) return baseCh;
  if (roundRobinRandomEnabled()) {
    uint8_t candidates[16];
    uint8_t count = 0;
    for (uint8_t idx = 0; idx < 16; ++idx) {
      if (settings.roundRobinMask & static_cast<uint16_t>(1U << idx)) candidates[count++] = idx + 1;
    }
    // Use the RP2040 hardware RNG. Repeats are intentional: every enabled channel is an
    // independent random choice rather than a shuffled or no-repeat cycle. Reject the tiny
    // modulo remainder so channel probabilities stay exactly even for every candidate count.
    const uint32_t threshold = (0U - static_cast<uint32_t>(count)) % count;
    uint32_t randomValue;
    do {
      randomValue = rp2040.hwrand32();
    } while (randomValue < threshold);
    return candidates[randomValue % count];
  }
  for (uint8_t attempts = 0; attempts < 16; ++attempts) {
    const uint8_t idx = roundRobinCursor++ & 0x0F;
    if (settings.roundRobinMask & static_cast<uint16_t>(1U << idx)) return idx + 1;
  }
  return baseCh;
}

void applyIncomingTransport(arpnmidi3::TransportEvent event) {
  if (event == arpnmidi3::TransportEvent::None) return;

  const uint64_t nowUs = time_us_64();
  if (event == arpnmidi3::TransportEvent::Start) {
    if (firmware3Settings.clockInFollow) restartArpTiming(true);
    if (firmware3Settings.looperMidiTransport) {
      if (multitrackLooper.hasAnyData()) {
        // Restarting from the top rewinds every cursor, so anything already
        // sounding has to be released or it would never be closed.
        if (multitrackLooper.playing()) {
          multitrackLooper.stop(releaseMultitrackOutput, nullptr);
        }
        multitrackLooper.start(nowUs);
      }
      multitrackLooper.beginArmedRecording(nowUs);
    }
  } else if (event == arpnmidi3::TransportEvent::Continue) {
    if (firmware3Settings.looperMidiTransport && multitrackLooper.hasAnyData() &&
        !multitrackLooper.playing()) multitrackLooper.resume(nowUs);
  } else if (event == arpnmidi3::TransportEvent::Stop) {
    if (firmware3Settings.clockInFollow) {
      arpNoteOffs();
      drumArpNoteOffs();
      arpNextStepUs = 0;
    }
    if (firmware3Settings.looperMidiTransport) {
      if (multitrackLooper.recording() || multitrackLooper.recordingArmed()) {
        finishActiveMultitrackRecording(nowUs, false);
      }
      multitrackLooper.pause(nowUs, releaseMultitrackOutput, nullptr);
    }
  }
  refreshLoopUiState();
  ui.dirty = true;
}

void handleRealtimeByte(uint8_t sourcePort, uint8_t status) {
  if (sourcePort == USB_DEVICE_SOURCE_PORT) ++usbIncomingMessageCount;
  else if (sourcePort == 0) ++dinIncomingMessageCount;
  lastIncomingSource = sourcePort;
  lastIncomingStatus = status;
  lastIncomingData1 = 0;
  lastIncomingData2 = 0;
  const arpnmidi3::TransportEvent event =
      musicalClock.receiveRealtime(status, time_us_64());
  applyIncomingTransport(event);

  if (firmware3Settings.clockOutSend && firmware3Settings.clockInFollow) {
    sendFanout(sourcePort, status, 0, 0);
  }
}

void handleClockByte(bool fromDin) {
  handleRealtimeByte(fromDin ? 0 : USB_DEVICE_SOURCE_PORT, 0xF8);
}

bool sensorParamEligible(uint8_t settingId) {
  return settingId == SET_DIVISION;
}

int16_t settingRangeMax(uint8_t settingId) {
  switch (settingId) {
    case SET_BPM: return 300;
    case SET_SWING: return 75;
    case SET_ARP_MODE:
      if (!arpMenuUi.editing) return 10;
      if (arpMenuUi.cursor == 0) return ARP_SELECTION_COUNT;
      if (arpMenuUi.cursor == 1) return ARP_DIVISION_FOLLOW_DRUM + 1;
      if (arpMenuUi.cursor == 2) return 128;
      if (arpMenuUi.cursor == 3) return 101;
      if (arpMenuUi.cursor == 4) return 5;
      if (arpMenuUi.cursor == 7) return 6;
      return 2;
    case SET_LIVE_VELOCITY:
      if (!liveVelocityUi.editing) return 3;
      if (liveVelocityUi.cursor == 0) return LIVE_TARGET_COUNT - 1;
      if (liveVelocityUi.cursor == 1) return 1;
      return 200;
    case SET_LIVE_NOTE_LENGTH:
      if (!liveNoteLengthUi.editing) return 3;
      if (liveNoteLengthUi.cursor == 0) return LIVE_TARGET_COUNT - 1;
      if (liveNoteLengthUi.cursor == 1) return 1;
      return 200;
    case SET_STUTTER:
      if (!stutterUi.editing) return 4;
      if (stutterUi.cursor == 0) return STUTTER_LENGTH_COUNT - STUTTER_LENGTH_MIN_SELECTION;
      if (stutterUi.cursor == 1) return 1;
      if (stutterUi.cursor == 2) return 17;
      return STUTTER_ECHO_TARGET_COUNT;
    case SET_ECHO:
      if (!echoUi.editing) return 6;
      if (echoUi.cursor == 0) return STUTTER_LENGTH_COUNT;
      if (echoUi.cursor == 1) return 1;
      if (echoUi.cursor == 2) return 101;
      if (echoUi.cursor == 3) return STUTTER_LENGTH_COUNT;
      if (echoUi.cursor == 4) return 33;
      return STUTTER_ECHO_TARGET_COUNT;
    case SET_DIVISION: return ARP_DIVISION_FOLLOW_DRUM;
    case SET_VELOCITY: return 127;
    case SET_LENGTH: return 100;
    case SET_QUICK_JUMP:
      if (!quickJumpUi.editing) return 4;
      if (quickJumpUi.cursor < 2) return 17;
      return 2;
    case SET_INPUT_CH: return DIRECT_CANCEL_CHANNEL;
    case SET_ARP_OUT_CH: return DIRECT_CANCEL_CHANNEL;
    case SET_DRUM_MAGIC:
      if (!drumMagicUi.editing) return 7;
      if (drumMagicUi.cursor == 0 || drumMagicUi.cursor == 1 || drumMagicUi.cursor == 5) return 1;
      if (drumMagicUi.cursor == 2) return 16;
      if (drumMagicUi.cursor == 6) return DRUM_DIVISION_FREE;
      return 120;
    case SET_BASS_CH:
      if (!bassUi.editing) return 3;
      if (bassUi.cursor == 0) return BASS_CANCEL_CHANNEL;
      if (bassUi.cursor == 1) return BASS_CANCEL_OCTAVE;
      return BASS_CANCEL_HIGH_NOTE;
    case SET_THRU_OUT_CH: return DIRECT_CANCEL_CHANNEL;
    case SET_RND_RBN: return RND_RBN_BACK_SLOT;
    case SET_ROUTER:
      if (routerEditStage == ROUTER_STAGE_DEST) return 16;
      if (routerEditStage == ROUTER_STAGE_LOW_NOTE ||
          routerEditStage == ROUTER_STAGE_HIGH_NOTE) return 127;
      if (routerEditStage == ROUTER_STAGE_TRANSPOSE) return ROUTER_TRANSPOSE_MAX - ROUTER_TRANSPOSE_MIN;
      return ROUTER_BACK_SLOT;
    case SET_DIV_NOTES: return DIV_NOTE_BACK_SLOT;
    case SET_MAP_CC:
      if (featuresUiStage == FEATURES_UI_KNOBS) return FEATURE_KNOB_COUNT;
      if (featuresUiStage == FEATURES_UI_BUTTONS) return FEATURE_BUTTON_COUNT;
      return 3;
    case SET_CC_MAP:
      if (ccRemapUiStage == CC_REMAP_UI_LIST) return CC_REMAP_SLOT_COUNT + 1;
      if (ccRemapUiStage == CC_REMAP_UI_INPUT) return 128;
      if (ccRemapUiStage == CC_REMAP_UI_OUTPUT_CHANNEL) return 16;
      return 127;
    case SET_NOTE_CC:
      if (noteCcUiStage == NOTE_CC_UI_LIST) return NOTE_CC_SLOT_COUNT + 1;
      if (noteCcUiStage == NOTE_CC_UI_SLOT_ACTION) return 2;
      if (noteCcUiStage == NOTE_CC_UI_INPUT_CHANNEL ||
          noteCcUiStage == NOTE_CC_UI_OUTPUT_CHANNEL) return NOTE_CC_CANCEL_CHANNEL;
      if (noteCcUiStage == NOTE_CC_UI_BEHAVIOR) return NOTE_CC_CANCEL_BEHAVIOR;
      return NOTE_CC_CANCEL_VALUE;
    case SET_LEGATO_CH: return 16;
    case SET_CC_OUT_CH: return DIRECT_CANCEL_CC_CHANNEL;
    case SET_SENSOR_CH: return DIRECT_CANCEL_CHANNEL;
    case SET_SENSOR_MODE: return SENSOR_MODE_COUNT;
    case SET_PUSH_MODE: return SENSOR_MODE_COUNT;
    case SET_FOUR_BUTTON:
      if (fourButtonUiStage == FOUR_BUTTON_UI_MAIN) return 4;
      if (fourButtonUiStage == FOUR_BUTTON_UI_MODE) return FOUR_BUTTON_MODE_COUNT - 1;
      if (fourButtonUiStage == FOUR_BUTTON_UI_CUSTOM_LIST) return FOUR_BUTTON_CUSTOM_BACK_SLOT;
      if (fourButtonUiStage == FOUR_BUTTON_UI_CUSTOM_CHANNEL) return FOUR_BUTTON_CANCEL_CHANNEL;
      if (fourButtonUiStage == FOUR_BUTTON_UI_CUSTOM_KIND) return FOUR_BUTTON_CANCEL_KIND;
      if (fourButtonUiStage == FOUR_BUTTON_UI_CUSTOM_NUMBER) return FOUR_BUTTON_CANCEL_NUMBER;
      if (fourButtonUiStage == FOUR_BUTTON_UI_CUSTOM_BEHAVIOR) return FOUR_BUTTON_CANCEL_BEHAVIOR;
      if (fourButtonUiStage == FOUR_BUTTON_UI_LOOPER) return FOUR_BUTTON_LOOPER_BACK_SLOT;
      return FOUR_BUTTON_CHORD_BACK_SLOT;
    case SET_LOOP_BARS:
      if (!looperSettingsUi.editing) return 8;
      if (looperSettingsUi.cursor == 0) return arpnmidi3::kLoopTrackCount - 1;
      if (looperSettingsUi.cursor == 1) return multitrackLooper.selectedTrack() == 0 ? 6 : 5;
      if (looperSettingsUi.cursor == 2) return LOOP_QUANTIZE_DIVISION_COUNT;
      if (looperSettingsUi.cursor == 3) return static_cast<uint8_t>(arpnmidi3::LoopTrackMode::Manual);
      return 1;
    case SET_MUTE_SOLO: return LOOP_MIX_BACK_SLOT;
    case SET_PARAMETER_LOCK:
      if (!parameterLockUi.editing) return 2;
      return 17;
    case SET_CHORD:
      if (!chordUi.editing) return 5;
      if (chordUi.cursor == 0) return 1;
      return 24;
    case SET_FORCE_KEY: return 24;
    case SET_FORCE_SCALE:
      if (!scaleUi.editing) return 13;
      return FORCE_SCALE_COUNT;
    case SET_GUITAR_PIANO: return 1;
    case SET_LIVE_CC:
      if (!liveCcEditing) return 2;
      return liveCcCursor == 0 ? 127 : 127;
    case SET_GLOBAL:
      if (!globalUi.editing) return 9;
      if (globalUi.cursor <= 5 || globalUi.cursor == 7) return 1;
      if (globalUi.cursor == 6) return 128;
      return 0;
    case SET_LOAD_PRESET: return PRESET_COUNT - 1;
    case SET_SAVE_PRESET: return PRESET_COUNT - 1;
    case SET_SCREEN_SAVER: return SCREEN_SAVER_CANCEL_SLOT;
    default: return 0;
  }
}

int16_t getSettingValueRaw(uint8_t settingId) {
  if (cancelSelectedFor(settingId)) return settingRangeMax(settingId);
  switch (settingId) {
    case SET_BPM: return settings.manualBpm;
    case SET_SWING: return firmware3Settings.swing;
    case SET_ARP_MODE:
      if (!arpMenuUi.editing) return arpMenuUi.cursor;
      if (arpMenuUi.cursor == 0) return settings.arpMode;
      if (arpMenuUi.cursor == 1) return settings.division;
      if (arpMenuUi.cursor == 2) return settings.arpVelocity;
      if (arpMenuUi.cursor == 3) return settings.arpLengthPct;
      if (arpMenuUi.cursor == 4) return firmware3Settings.arpOctaves;
      if (arpMenuUi.cursor == 5) return firmware3Settings.arpRetriggerSync;
      if (arpMenuUi.cursor == 6) return firmware3Settings.arpNoteOrder;
      if (arpMenuUi.cursor == 7) return firmware3Settings.customArpLength;
      return 0;
    case SET_LIVE_VELOCITY:
      if (!liveVelocityUi.editing) return liveVelocityUi.cursor;
      if (liveVelocityUi.cursor == 0) return liveVelocityTarget;
      if (liveVelocityUi.cursor == 1) return firmware3Settings.liveTargets[liveVelocityTarget].velocityEnabled;
      return firmware3Settings.liveTargets[liveVelocityTarget].velocityPercent;
    case SET_LIVE_NOTE_LENGTH:
      if (!liveNoteLengthUi.editing) return liveNoteLengthUi.cursor;
      if (liveNoteLengthUi.cursor == 0) return liveNoteLengthTarget;
      if (liveNoteLengthUi.cursor == 1) return firmware3Settings.liveTargets[liveNoteLengthTarget].noteLengthEnabled;
      return firmware3Settings.liveTargets[liveNoteLengthTarget].noteLengthPercent;
    case SET_STUTTER:
      if (!stutterUi.editing) return stutterUi.cursor;
      if (stutterUi.cursor == 0) {
        // Edited as a local, 0-based index within Stutter's own shorter
        // range; the stored value stays in the full shared length space so
        // display and duration lookups elsewhere need no translation.
        return firmware3Settings.liveTargets[stutterTarget].stutterLengthSelection -
            STUTTER_LENGTH_MIN_SELECTION;
      }
      if (stutterUi.cursor == 1) return firmware3Settings.liveTargets[stutterTarget].stutterEnabled;
      if (stutterUi.cursor == 2) return firmware3Settings.stutterTimeoutBars;
      return stutterTarget;
    case SET_ECHO:
      if (!echoUi.editing) return echoUi.cursor;
      if (echoUi.cursor == 0) return firmware3Settings.liveTargets[echoTarget].echoLength;
      if (echoUi.cursor == 1) return firmware3Settings.liveTargets[echoTarget].echoEnabled;
      if (echoUi.cursor == 2) return firmware3Settings.liveTargets[echoTarget].echoWet;
      if (echoUi.cursor == 3) return firmware3Settings.liveTargets[echoTarget].echoDelay;
      if (echoUi.cursor == 4) return firmware3Settings.liveTargets[echoTarget].echoDrift + 16;
      return echoTarget;
    case SET_DIVISION: return settings.division;
    case SET_VELOCITY: return settings.arpVelocity;
    case SET_LENGTH: return settings.arpLengthPct;
    case SET_QUICK_JUMP:
      if (!quickJumpUi.editing) return quickJumpUi.cursor;
      if (quickJumpUi.cursor == 0) return firmware3Settings.quickJumpInputChannel;
      if (quickJumpUi.cursor == 1) return firmware3Settings.quickJumpOutputChannel;
      if (quickJumpUi.cursor == 2) return firmware3Settings.quickJumpEnabled;
      return firmware3Settings.quickJumpHold;
    case SET_INPUT_CH: return settings.inputChannel;
    case SET_ARP_OUT_CH: return settings.arpOutChannel;
    case SET_DRUM_MAGIC:
      if (!drumMagicUi.editing) return drumMagicUi.cursor;
      if (drumMagicUi.cursor == 0) return firmware3Settings.drumEnabled;
      if (drumMagicUi.cursor == 1) return firmware3Settings.drumInputMode;
      if (drumMagicUi.cursor == 2) return firmware3Settings.drumOutputChannel;
      if (drumMagicUi.cursor == 3) return firmware3Settings.drumSplitNote;
      if (drumMagicUi.cursor == 4) return firmware3Settings.drumMappedStart;
      if (drumMagicUi.cursor == 5) return firmware3Settings.drumAftertouchVelocity;
      return firmware3Settings.drumDivision;
    case SET_BASS_CH:
      if (!bassUi.editing) return bassUi.cursor;
      if (bassUi.cursor == 0) return bassModeChannel(settings.bassMode);
      if (bassUi.cursor == 1) return bassModeOctaveOffset(settings.bassMode) + 2;
      return firmware3Settings.bassHighestNote;
    case SET_THRU_OUT_CH: return settings.thruOutChannel;
    case SET_RND_RBN: return roundRobinMenuCursor;
    case SET_ROUTER:
      if (routerEditStage == ROUTER_STAGE_DEST) return settings.routerOutChannels[routerEditChannel];
      if (routerEditStage == ROUTER_STAGE_LOW_NOTE) return firmware3Settings.routerLowNotes[routerEditChannel];
      if (routerEditStage == ROUTER_STAGE_HIGH_NOTE) return firmware3Settings.routerHighNotes[routerEditChannel];
      if (routerEditStage == ROUTER_STAGE_TRANSPOSE) return settings.routerTranspose[routerEditChannel] - ROUTER_TRANSPOSE_MIN;
      return routerMenuCursor;
    case SET_DIV_NOTES: return divNotesCursor;
    case SET_MAP_CC:
      return featuresUiStage == FEATURES_UI_GROUPS ? featuresGroupCursor : featuresItemCursor;
    case SET_CC_MAP:
      if (ccRemapUiStage == CC_REMAP_UI_LIST) return ccRemapCursor;
      if (ccRemapCursor >= CC_REMAP_SLOT_COUNT) return 0;
      if (ccRemapUiStage == CC_REMAP_UI_INPUT) {
        return featureControls.ccRemaps[ccRemapCursor].inputCc <= 127
            ? featureControls.ccRemaps[ccRemapCursor].inputCc : 128;
      }
      if (ccRemapUiStage == CC_REMAP_UI_OUTPUT_CHANNEL) {
        return featureControls.ccRemaps[ccRemapCursor].outputChannel;
      }
      return featureControls.ccRemaps[ccRemapCursor].outputCc;
    case SET_NOTE_CC:
      if (noteCcUiStage == NOTE_CC_UI_LIST) return noteCcCursor;
      if (noteCcUiStage == NOTE_CC_UI_SLOT_ACTION) return noteCcSlotActionCursor;
      if (noteCcCursor >= NOTE_CC_SLOT_COUNT) return 0;
      if (noteCcUiStage == NOTE_CC_UI_INPUT_CHANNEL) {
        if (cancelSelectedFor(settingId)) return NOTE_CC_CANCEL_CHANNEL;
        return max<uint8_t>(1, featureControls.noteCcMaps[noteCcCursor].inputChannel);
      }
      if (noteCcUiStage == NOTE_CC_UI_INPUT_NOTE) {
        if (cancelSelectedFor(settingId)) return NOTE_CC_CANCEL_VALUE;
        return featureControls.noteCcMaps[noteCcCursor].inputNote <= 127
            ? featureControls.noteCcMaps[noteCcCursor].inputNote : 0;
      }
      if (noteCcUiStage == NOTE_CC_UI_OUTPUT_CHANNEL) {
        if (cancelSelectedFor(settingId)) return NOTE_CC_CANCEL_CHANNEL;
        return featureControls.noteCcMaps[noteCcCursor].outputChannel;
      }
      if (noteCcUiStage == NOTE_CC_UI_OUTPUT_CC) {
        if (cancelSelectedFor(settingId)) return NOTE_CC_CANCEL_VALUE;
        return featureControls.noteCcMaps[noteCcCursor].outputCc;
      }
      if (cancelSelectedFor(settingId)) return NOTE_CC_CANCEL_BEHAVIOR;
      return featureControls.noteCcMaps[noteCcCursor].behavior;
    case SET_LEGATO_CH: return settings.legatoChannel;
    case SET_CC_OUT_CH: return settings.ccOutChannel;
    case SET_SENSOR_CH: return settings.sensorChannel;
    case SET_SENSOR_MODE: return settings.sensorMode;
    case SET_PUSH_MODE: return settings.pushMode;
    case SET_FOUR_BUTTON:
      if (fourButtonUiStage == FOUR_BUTTON_UI_MAIN ||
          fourButtonUiStage == FOUR_BUTTON_UI_CUSTOM_LIST ||
          fourButtonUiStage == FOUR_BUTTON_UI_LOOPER ||
          fourButtonUiStage == FOUR_BUTTON_UI_CHORD) return fourButtonUiCursor;
      if (fourButtonUiStage == FOUR_BUTTON_UI_MODE) return featureControls.fourButtonMode;
      if (fourButtonUiStage == FOUR_BUTTON_UI_CUSTOM_CHANNEL) {
        if (cancelSelectedFor(settingId)) return FOUR_BUTTON_CANCEL_CHANNEL;
        return featureControls.customButtons[fourButtonEditButton].channel;
      }
      if (fourButtonUiStage == FOUR_BUTTON_UI_CUSTOM_KIND) {
        if (cancelSelectedFor(settingId)) return FOUR_BUTTON_CANCEL_KIND;
        return featureControls.customButtons[fourButtonEditButton].kind == TRIGGER_BINDING_NOTE ? 1 : 0;
      }
      if (fourButtonUiStage == FOUR_BUTTON_UI_CUSTOM_NUMBER) {
        if (cancelSelectedFor(settingId)) return FOUR_BUTTON_CANCEL_NUMBER;
        return featureControls.customButtons[fourButtonEditButton].number;
      }
      if (cancelSelectedFor(settingId)) return FOUR_BUTTON_CANCEL_BEHAVIOR;
      return featureControls.customButtons[fourButtonEditButton].behavior;
    case SET_LOOP_BARS:
      if (!looperSettingsUi.editing) return looperSettingsUi.cursor;
      if (looperSettingsUi.cursor == 0) return multitrackLooper.selectedTrack();
      if (looperSettingsUi.cursor == 1) return loopTrackLengthSelection[multitrackLooper.selectedTrack()];
      if (looperSettingsUi.cursor == 2) {
        return loopTrackQuantizeSelection(multitrackLooper.selectedTrack());
      }
      if (looperSettingsUi.cursor == 3) return firmware3Settings.looperTrackMode;
      if (looperSettingsUi.cursor == 4) return firmware3Settings.looperAutoRec;
      if (looperSettingsUi.cursor == 5) return firmware3Settings.looperTimeTravel;
      if (looperSettingsUi.cursor == 6) return firmware3Settings.looperRecordCc;
      return firmware3Settings.looperMidiTransport;
    case SET_MUTE_SOLO: return muteSoloCursor;
    case SET_PARAMETER_LOCK:
      if (!parameterLockUi.editing) return parameterLockUi.cursor;
      return firmware3Settings.parameterLockChannel;
    case SET_CHORD:
      if (!chordUi.editing) return chordUi.cursor;
      if (chordUi.cursor == 0) return firmware3Settings.chordEnabled;
      return firmware3Settings.chordPositions[chordUi.cursor - 1] + 12;
    case SET_FORCE_KEY: return settings.forceKey;
    case SET_FORCE_SCALE:
      if (!scaleUi.editing) return scaleUi.cursor;
      return settings.forceScale;
    case SET_GUITAR_PIANO: return settings.instrumentView;
    case SET_LIVE_CC:
      if (!liveCcEditing) return liveCcCursor;
      return liveCcCursor == 0 ? liveCcNumber : liveCcValue;
    case SET_GLOBAL:
      if (!globalUi.editing) return globalUi.cursor;
      if (globalUi.cursor == 0) return storage.autoSave;
      if (globalUi.cursor == 1) return firmware3Settings.clockInFollow;
      if (globalUi.cursor == 2) return firmware3Settings.clockOutSend;
      if (globalUi.cursor == 3) return firmware3Settings.timeSignature;
      if (globalUi.cursor == 4) return firmware3Settings.forwardChannelAftertouch;
      if (globalUi.cursor == 5) return firmware3Settings.forwardPolyAftertouch;
      if (globalUi.cursor == 6) return firmware3Settings.channelAftertouchCc <= 127
          ? firmware3Settings.channelAftertouchCc : 128;
      if (globalUi.cursor == 7) return firmware3Settings.mainAftertouchArpVelocity;
      return 0;
    case SET_LOAD_PRESET:
      if (ui.menuMode == MENU_SELECT && ui.selectedSetting == SET_LOAD_PRESET) return storage.currentPreset;
      return settings.loadPreset;
    case SET_SAVE_PRESET:
      if (ui.menuMode == MENU_SELECT && ui.selectedSetting == SET_SAVE_PRESET) return storage.currentPreset;
      return settings.savePreset;
    case SET_SCREEN_SAVER:
      return ui.menuMode == MENU_EDIT && ui.selectedSetting == SET_SCREEN_SAVER
          ? screenSaverCursor : settings.screenSaver;
    default: return 0;
  }
}

void cancelNoteCcEdit() {
  if (noteCcCursor < NOTE_CC_SLOT_COUNT) {
    featureControls.noteCcMaps[noteCcCursor] = noteCcEditBackup;
  }
  noteCcLearnActive = false;
  noteCcUiStage = NOTE_CC_UI_LIST;
  noteCcSlotActionCursor = 0;
  clearEditCancelSelection();
  ui.dirty = true;
}

void cancelFourButtonCustomEdit() {
  if (fourButtonEditButton < 4) {
    featureControls.customButtons[fourButtonEditButton] = fourButtonEditBackup;
  }
  fourButtonLearnActive = false;
  fourButtonUiStage = FOUR_BUTTON_UI_CUSTOM_LIST;
  fourButtonUiCursor = fourButtonEditButton;
  clearEditCancelSelection();
  ui.dirty = true;
}

uint8_t cancelContextCursor(uint8_t settingId) {
  if (settingId == SET_ARP_MODE) return arpMenuUi.cursor;
  if (settingId == SET_QUICK_JUMP) return quickJumpUi.cursor;
  if (settingId == SET_STUTTER) return stutterUi.cursor;
  if (settingId == SET_ECHO) return echoUi.cursor;
  if (settingId == SET_BASS_CH) return bassUi.cursor;
  if (settingId == SET_FOUR_BUTTON) return fourButtonUiStage;
  if (settingId == SET_PARAMETER_LOCK) return parameterLockUi.cursor;
  if (settingId == SET_FORCE_SCALE) return scaleUi.cursor;
  return 0;
}

bool cancelSelectedFor(uint8_t settingId) {
  return editCancelSelected && editCancelSetting == settingId &&
         editCancelCursor == cancelContextCursor(settingId);
}

void selectCancelFor(uint8_t settingId) {
  editCancelSelected = true;
  editCancelSetting = settingId;
  editCancelCursor = cancelContextCursor(settingId);
}

void clearEditCancelSelection() {
  editCancelSelected = false;
}

void setSettingValueRaw(uint8_t settingId, int16_t value) {
  switch (settingId) {
    case SET_BPM:
      settings.manualBpm = constrain(value, 20, 300);
      syncMusicalClockConfig(false);
      break;
    case SET_SWING:
      firmware3Settings.swing = clampU8(value, 0, 75);
      break;
    case SET_ARP_MODE:
      if (!arpMenuUi.editing) arpMenuUi.cursor = clampU8(value, 0, 10);
      else if (arpMenuUi.cursor == 0) settings.arpMode = clampU8(value, 0, ARP_SELECTION_COUNT - 1);
      else if (arpMenuUi.cursor == 1) {
        settings.division = clampU8(value, 0, ARP_DIVISION_FOLLOW_DRUM);
        if (settings.division == ARP_DIVISION_FOLLOW_DRUM &&
            firmware3Settings.drumDivision == DRUM_DIVISION_FOLLOW_ARP) {
          firmware3Settings.drumDivision = DIV_1_16;
        }
        syncArpDivisionToGrid();
      } else if (arpMenuUi.cursor == 2) settings.arpVelocity = clampU8(value, 1, 127);
      else if (arpMenuUi.cursor == 3) settings.arpLengthPct = clampU8(value, 1, 100);
      else if (arpMenuUi.cursor == 4) firmware3Settings.arpOctaves = clampU8(value, 1, 4);
      else if (arpMenuUi.cursor == 5) firmware3Settings.arpRetriggerSync = value ? 1 : 0;
      else if (arpMenuUi.cursor == 6) firmware3Settings.arpNoteOrder = value ? 1 : 0;
      else if (arpMenuUi.cursor == 7) {
        firmware3Settings.customArpLength = clampU8(value, 0, 5);
        customArpPattern.lengthSelection = firmware3Settings.customArpLength;
      }
      break;
    case SET_LIVE_VELOCITY:
      if (!liveVelocityUi.editing) liveVelocityUi.cursor = clampU8(value, 0, 3);
      else if (liveVelocityUi.cursor == 0) liveVelocityTarget = clampU8(value, 0, LIVE_TARGET_COUNT - 1);
      else if (liveVelocityUi.cursor == 1) firmware3Settings.liveTargets[liveVelocityTarget].velocityEnabled = value ? 1 : 0;
      else firmware3Settings.liveTargets[liveVelocityTarget].velocityPercent = clampU8(value, 0, 200);
      break;
    case SET_LIVE_NOTE_LENGTH:
      if (!liveNoteLengthUi.editing) liveNoteLengthUi.cursor = clampU8(value, 0, 3);
      else if (liveNoteLengthUi.cursor == 0) liveNoteLengthTarget = clampU8(value, 0, LIVE_TARGET_COUNT - 1);
      else if (liveNoteLengthUi.cursor == 1) firmware3Settings.liveTargets[liveNoteLengthTarget].noteLengthEnabled = value ? 1 : 0;
      else firmware3Settings.liveTargets[liveNoteLengthTarget].noteLengthPercent = clampU8(value, 1, 200);
      break;
    case SET_STUTTER:
      if (!stutterUi.editing) stutterUi.cursor = clampU8(value, 0, 4);
      else if (stutterUi.cursor == 0) requestStutterState(stutterTarget,
          firmware3Settings.liveTargets[stutterTarget].stutterEnabled != 0,
          clampU8(value, 0, STUTTER_LENGTH_COUNT - 1 - STUTTER_LENGTH_MIN_SELECTION) +
              STUTTER_LENGTH_MIN_SELECTION);
      else if (stutterUi.cursor == 1) requestStutterState(stutterTarget, value != 0);
      else if (stutterUi.cursor == 2) firmware3Settings.stutterTimeoutBars = clampU8(value, 1, 16);
      else stutterTarget = clampU8(value, 0, STUTTER_ECHO_TARGET_COUNT - 1);
      break;
    case SET_ECHO:
      if (!echoUi.editing) echoUi.cursor = clampU8(value, 0, 6);
      else if (echoUi.cursor == 0) firmware3Settings.liveTargets[echoTarget].echoLength = clampU8(value, 0, STUTTER_LENGTH_COUNT - 1);
      else if (echoUi.cursor == 1) firmware3Settings.liveTargets[echoTarget].echoEnabled = value ? 1 : 0;
      else if (echoUi.cursor == 2) firmware3Settings.liveTargets[echoTarget].echoWet = clampU8(value, 0, 100);
      else if (echoUi.cursor == 3) firmware3Settings.liveTargets[echoTarget].echoDelay = clampU8(value, 0, STUTTER_LENGTH_COUNT - 1);
      else if (echoUi.cursor == 4) firmware3Settings.liveTargets[echoTarget].echoDrift =
          constrain(static_cast<int>(value) - 16, -16, 16);
      else echoTarget = clampU8(value, 0, STUTTER_ECHO_TARGET_COUNT - 1);
      break;
    case SET_DIVISION:
      settings.division = clampU8(value, 0, ARP_DIVISION_FOLLOW_DRUM);
      if (settings.division == ARP_DIVISION_FOLLOW_DRUM &&
          firmware3Settings.drumDivision == DRUM_DIVISION_FOLLOW_ARP) {
        firmware3Settings.drumDivision = DIV_1_16;
      }
      break;
    case SET_VELOCITY: settings.arpVelocity = clampU8(value, 1, 127); break;
    case SET_LENGTH: settings.arpLengthPct = clampU8(value, 1, 100); break;
    case SET_QUICK_JUMP:
      if (!quickJumpUi.editing) quickJumpUi.cursor = clampU8(value, 0, 4);
      else if (quickJumpUi.cursor == 0) firmware3Settings.quickJumpInputChannel = clampU8(value, 1, 16);
      else if (quickJumpUi.cursor == 1) firmware3Settings.quickJumpOutputChannel = clampU8(value, 1, 16);
      else if (quickJumpUi.cursor == 2) setQuickJumpEnabled(value != 0);
      else firmware3Settings.quickJumpHold = value ? 1 : 0;
      break;
    case SET_INPUT_CH: settings.inputChannel = clampU8(value, 1, 16); break;
    case SET_ARP_OUT_CH:
      settings.arpOutChannel = clampU8(value, 0, 16);
      break;
    case SET_DRUM_MAGIC:
      if (!drumMagicUi.editing) drumMagicUi.cursor = clampU8(value, 0, 7);
      else if (drumMagicUi.cursor == 0) firmware3Settings.drumEnabled = value ? 1 : 0;
      else if (drumMagicUi.cursor == 1) firmware3Settings.drumInputMode = value ? 1 : 0;
      else if (drumMagicUi.cursor == 2) firmware3Settings.drumOutputChannel = clampU8(value, 1, 16);
      else if (drumMagicUi.cursor == 3) firmware3Settings.drumSplitNote = clampU8(value, 0, 120);
      else if (drumMagicUi.cursor == 4) firmware3Settings.drumMappedStart = clampU8(value, 0, 120);
      else if (drumMagicUi.cursor == 5) firmware3Settings.drumAftertouchVelocity = value ? 1 : 0;
      else {
        firmware3Settings.drumDivision = clampU8(value, 0, DRUM_DIVISION_FREE);
        if (firmware3Settings.drumDivision == DRUM_DIVISION_FOLLOW_ARP &&
            settings.division == ARP_DIVISION_FOLLOW_DRUM) settings.division = DIV_1_16;
        syncArpDivisionToGrid();
      }
      break;
    case SET_BASS_CH:
      if (!bassUi.editing) bassUi.cursor = clampU8(value, 0, 3);
      else if (bassUi.cursor == 0) {
        const uint8_t channel = clampU8(value, 0, 12);
        const int8_t octaves = settings.bassMode == 0 ? 0 : bassModeOctaveOffset(settings.bassMode);
        settings.bassMode = bassModeFromChannelOctave(channel, octaves);
      } else if (bassUi.cursor == 1) {
        const uint8_t channel = bassModeChannel(settings.bassMode);
        settings.bassMode = bassModeFromChannelOctave(channel, static_cast<int8_t>(clampU8(value, 0, 3)) - 2);
      } else {
        firmware3Settings.bassHighestNote = clampU8(value, 0, 127);
      }
      break;
    case SET_THRU_OUT_CH: settings.thruOutChannel = clampU8(value, 0, 16); break;
    case SET_RND_RBN: roundRobinMenuCursor = clampU8(value, 0, RND_RBN_BACK_SLOT); break;
    case SET_ROUTER:
      if (routerEditStage == ROUTER_STAGE_DEST) {
        settings.routerOutChannels[routerEditChannel] = clampU8(value, 1, 16);
        updateRouterActiveBit(settings, routerEditChannel);
      } else if (routerEditStage == ROUTER_STAGE_LOW_NOTE) {
        firmware3Settings.routerLowNotes[routerEditChannel] =
            clampU8(value, 0, firmware3Settings.routerHighNotes[routerEditChannel]);
      } else if (routerEditStage == ROUTER_STAGE_HIGH_NOTE) {
        firmware3Settings.routerHighNotes[routerEditChannel] =
            clampU8(value, firmware3Settings.routerLowNotes[routerEditChannel], 127);
      } else if (routerEditStage == ROUTER_STAGE_TRANSPOSE) {
        settings.routerTranspose[routerEditChannel] =
          static_cast<int8_t>(constrain(value + ROUTER_TRANSPOSE_MIN, ROUTER_TRANSPOSE_MIN, ROUTER_TRANSPOSE_MAX));
        updateRouterActiveBit(settings, routerEditChannel);
      } else {
        routerMenuCursor = clampU8(value, 0, ROUTER_BACK_SLOT);
      }
      break;
    case SET_DIV_NOTES: divNotesCursor = clampU8(value, 0, DIV_NOTE_BACK_SLOT); break;
    case SET_MAP_CC:
      if (featuresUiStage == FEATURES_UI_GROUPS) featuresGroupCursor = clampU8(value, 0, 3);
      else {
        // A turn always means "back to browsing the list," wherever it lands.
        featuresItemOpen = false;
        if (featuresUiStage == FEATURES_UI_KNOBS) {
          featuresItemCursor = clampU8(value, 0, FEATURE_KNOB_COUNT);
        } else {
          featuresItemCursor = clampU8(value, 0, FEATURE_BUTTON_COUNT);
        }
      }
      break;
    case SET_CC_MAP:
      if (ccRemapUiStage == CC_REMAP_UI_LIST) {
        ccRemapCursor = clampU8(value, 0, CC_REMAP_SLOT_COUNT + 1);
      } else if (ccRemapCursor < CC_REMAP_SLOT_COUNT) {
        CcRemapEntry &entry = featureControls.ccRemaps[ccRemapCursor];
        if (ccRemapUiStage == CC_REMAP_UI_INPUT) {
          entry.inputCc = value > 127 ? 0xFF : clampU8(value, 0, 127);
        } else if (ccRemapUiStage == CC_REMAP_UI_OUTPUT_CHANNEL) {
          entry.outputChannel = clampU8(value, 1, 16);
        } else {
          entry.outputCc = clampU8(value, 0, 127);
        }
      }
      break;
    case SET_NOTE_CC:
      if (noteCcUiStage == NOTE_CC_UI_LIST) {
        noteCcCursor = clampU8(value, 0, NOTE_CC_SLOT_COUNT + 1);
      } else if (noteCcUiStage == NOTE_CC_UI_SLOT_ACTION) {
        noteCcSlotActionCursor = clampU8(value, 0, 2);
      } else if (noteCcCursor < NOTE_CC_SLOT_COUNT) {
        NoteCcMapEntry &entry = featureControls.noteCcMaps[noteCcCursor];
        if (noteCcUiStage == NOTE_CC_UI_INPUT_CHANNEL) {
          entry.inputChannel = clampU8(value, 1, 16);
        } else if (noteCcUiStage == NOTE_CC_UI_INPUT_NOTE) {
          entry.inputNote = clampU8(value, 0, 127);
        } else if (noteCcUiStage == NOTE_CC_UI_OUTPUT_CHANNEL) {
          entry.outputChannel = clampU8(value, 1, 16);
        } else if (noteCcUiStage == NOTE_CC_UI_OUTPUT_CC) {
          entry.outputCc = clampU8(value, 0, 127);
        } else {
          entry.behavior = value ? NOTE_CC_TOGGLE : NOTE_CC_MOMENTARY;
        }
      }
      break;
    case SET_LEGATO_CH: {
      const uint8_t previous = settings.legatoChannel;
      settings.legatoChannel = clampU8(value, 0, 16);
      if (settings.legatoChannel != previous) clearLegatoState(true, previous);
      break;
    }
    case SET_CC_OUT_CH: settings.ccOutChannel = clampU8(value, 1, 17); break;
    case SET_SENSOR_CH: settings.sensorChannel = clampU8(value, 1, 16); break;
    case SET_SENSOR_MODE:
      settings.sensorMode = clampU8(value, 0, SENSOR_MODE_COUNT - 1);
      if (!arpLatchEnabled()) clearArpLatchNotes();
      if (!arpFreezeEnabled()) clearFreezeState();
      if (arpFreezeActive && arpFreezePlusActive && !arpFreezePlusEnabled()) {
        for (uint8_t note = 0; note < 128; ++note) {
          if (!thruFrozenNotes[note]) continue;
          const uint8_t out = thruFrozenMappedNotes[note];
          if (out <= 127) thruOutputRefOff(255, out);
        }
        memset(thruFrozenNotes, 0, sizeof(thruFrozenNotes));
        memset(thruFrozenMappedNotes, 0xFF, sizeof(thruFrozenMappedNotes));
        arpFreezePlusActive = false;
      }
      rebuildArpHeldSorted();
      updateBassVoice();
      break;
    case SET_PUSH_MODE:
      settings.pushMode = clampU8(value, 0, SENSOR_MODE_COUNT - 1);
      if (!arpLatchEnabled()) clearArpLatchNotes();
      if (!arpFreezeEnabled()) clearFreezeState();
      if (arpFreezeActive && arpFreezePlusActive && !arpFreezePlusEnabled()) {
        for (uint8_t note = 0; note < 128; ++note) {
          if (!thruFrozenNotes[note]) continue;
          const uint8_t out = thruFrozenMappedNotes[note];
          if (out <= 127) thruOutputRefOff(255, out);
        }
        memset(thruFrozenNotes, 0, sizeof(thruFrozenNotes));
        memset(thruFrozenMappedNotes, 0xFF, sizeof(thruFrozenMappedNotes));
        arpFreezePlusActive = false;
      }
      rebuildArpHeldSorted();
      updateBassVoice();
      break;
    case SET_FOUR_BUTTON:
      if (fourButtonUiStage == FOUR_BUTTON_UI_MAIN ||
          fourButtonUiStage == FOUR_BUTTON_UI_CUSTOM_LIST ||
          fourButtonUiStage == FOUR_BUTTON_UI_LOOPER ||
          fourButtonUiStage == FOUR_BUTTON_UI_CHORD) {
        fourButtonUiCursor = clampU8(value, 0, settingRangeMax(SET_FOUR_BUTTON));
      } else if (fourButtonUiStage == FOUR_BUTTON_UI_MODE) {
        featureControls.fourButtonMode = clampU8(value, 0, FOUR_BUTTON_MODE_COUNT - 1);
      } else if (fourButtonUiStage == FOUR_BUTTON_UI_CUSTOM_CHANNEL) {
        featureControls.customButtons[fourButtonEditButton].channel = clampU8(value, 1, 16);
      } else if (fourButtonUiStage == FOUR_BUTTON_UI_CUSTOM_KIND) {
        featureControls.customButtons[fourButtonEditButton].kind = value
            ? TRIGGER_BINDING_NOTE : TRIGGER_BINDING_CC;
      } else if (fourButtonUiStage == FOUR_BUTTON_UI_CUSTOM_NUMBER) {
        featureControls.customButtons[fourButtonEditButton].number = clampU8(value, 0, 127);
      } else {
        featureControls.customButtons[fourButtonEditButton].behavior =
            clampU8(value, 0, CUSTOM_BUTTON_BEHAVIOR_COUNT - 1);
      }
      break;
    case SET_LOOP_BARS:
      if (!looperSettingsUi.editing) looperSettingsUi.cursor = clampU8(value, 0, 8);
      else if (looperSettingsUi.cursor == 0) {
        // Selection is navigation, not content. It rides along in the loop
        // file whenever a real change writes it.
        selectLooperTrack(clampU8(value, 0, arpnmidi3::kLoopTrackCount - 1));
      }
      else if (looperSettingsUi.cursor == 1) {
        const uint8_t track = multitrackLooper.selectedTrack();
        setLoopTrackLengthSelection(track, value);
      } else if (looperSettingsUi.cursor == 2) {
        firmware3Settings.looperQuantize[multitrackLooper.selectedTrack()] =
            clampU8(value, 0, LOOP_QUANTIZE_DIVISION_COUNT);
      } else if (looperSettingsUi.cursor == 3) {
        firmware3Settings.looperTrackMode =
          clampU8(value, 0, static_cast<uint8_t>(arpnmidi3::LoopTrackMode::Manual));
      } else if (looperSettingsUi.cursor == 4) {
        firmware3Settings.looperAutoRec = value ? 1 : 0;
      } else if (looperSettingsUi.cursor == 5) {
        firmware3Settings.looperTimeTravel = value ? 1 : 0;
      } else if (looperSettingsUi.cursor == 6) {
        firmware3Settings.looperRecordCc = value ? 1 : 0;
      } else {
        firmware3Settings.looperMidiTransport = value ? 1 : 0;
      }
      // Every field here lives in the same submenu and commits together: the
      // click that leaves the edited field saves once, through
      // finishSubmenuOrEdit below. Saving per tick while scrolling a value,
      // quant included, wrote to flash on every detent instead of once at
      // commit.
      break;
    case SET_MUTE_SOLO:
      // Moving the cursor is browsing. The engine's working track changes
      // only when an action lands on a track: Arm and Clear select it, Mute
      // and Solo leave the selection alone. The cursor still follows the
      // engine when a button, a CC, or the auto-advance selects a track.
      muteSoloCursor = clampU8(value, 0, LOOP_MIX_BACK_SLOT);
      break;
    case SET_PARAMETER_LOCK:
      if (!parameterLockUi.editing) parameterLockUi.cursor = clampU8(value, 0, 2);
      else firmware3Settings.parameterLockChannel = clampU8(value, 0, 16);
      break;
    case SET_CHORD:
      if (!chordUi.editing) chordUi.cursor = clampU8(value, 0, 5);
      else if (chordUi.cursor == 0) firmware3Settings.chordEnabled = value ? 1 : 0;
      else firmware3Settings.chordPositions[chordUi.cursor - 1] =
          constrain(static_cast<int>(value) - 12, -12, 12);
      break;
    case SET_FORCE_KEY:
      settings.forceKey = clampU8(value, 0, 24);
      if (ckeyEnabled() && scaleIsCombo(settings.forceScale)) settings.forceScale = SCALE_MAJOR;
      break;
    case SET_FORCE_SCALE:
      if (!scaleUi.editing) scaleUi.cursor = clampU8(value, 0, 13);
      else settings.forceScale = clampU8(value, 0, FORCE_SCALE_COUNT - 1);
      break;
    case SET_GUITAR_PIANO: settings.instrumentView = clampU8(value, 0, 1); break;
    case SET_LIVE_CC:
      if (!liveCcEditing) liveCcCursor = clampU8(value, 0, 2);
      else if (liveCcCursor == 0) liveCcNumber = clampU8(value, 0, 127);
      else {
        liveCcValue = clampU8(value, 0, 127);
        const uint8_t channel = channelEnabled(mainArpOutChannel()) ? mainArpOutChannel() : 1;
        sendFanout(255, static_cast<uint8_t>(0xB0 | (channel - 1U)), liveCcNumber, liveCcValue);
      }
      break;
    case SET_GLOBAL:
      if (!globalUi.editing) globalUi.cursor = clampU8(value, 0, 9);
      else if (globalUi.cursor == 0) {
        const uint8_t enabled = value ? 1 : 0;
        if (storage.autoSave != enabled) {
          storage.autoSave = enabled;
          // Auto Save is global rather than part of the preset, so its own
          // change must survive even when it is being turned off.
          saveStorage();
        }
      }
      else if (globalUi.cursor == 1) firmware3Settings.clockInFollow = value ? 1 : 0;
      else if (globalUi.cursor == 2) firmware3Settings.clockOutSend = value ? 1 : 0;
      else if (globalUi.cursor == 3) firmware3Settings.timeSignature = value ? 1 : 0;
      else if (globalUi.cursor == 4) firmware3Settings.forwardChannelAftertouch = value ? 1 : 0;
      else if (globalUi.cursor == 5) firmware3Settings.forwardPolyAftertouch = value ? 1 : 0;
      else if (globalUi.cursor == 6) firmware3Settings.channelAftertouchCc =
          value > 127 ? 0xFF : clampU8(value, 0, 127);
      else if (globalUi.cursor == 7) firmware3Settings.mainAftertouchArpVelocity = value ? 1 : 0;
      syncMusicalClockConfig(false);
      break;
    case SET_LOAD_PRESET: settings.loadPreset = clampU8(value, 0, PRESET_COUNT - 1); break;
    case SET_SAVE_PRESET: settings.savePreset = clampU8(value, 0, PRESET_COUNT - 1); break;
    case SET_SCREEN_SAVER:
      if (ui.menuMode == MENU_EDIT && ui.selectedSetting == SET_SCREEN_SAVER) {
        screenSaverCursor = clampU8(value, 0, SCREEN_SAVER_CANCEL_SLOT);
      } else {
        settings.screenSaver = clampU8(value, 0, SCREEN_SAVER_CANCEL_SLOT - 1);
        screenSaverCursor = settings.screenSaver;
        screenSaverForceNow = false;
      }
      break;
    default: break;
  }
}

int16_t applyDivisionOverlayMode(int16_t base, int16_t maxValue, uint8_t mode,
                                 bool inRange, uint8_t pct) {
  if (!inRange || !sensorModeIsDivisionOverlay(mode)) return base;

  if (mode == SENSOR_PARAM_PLUS2 || mode == SENSOR_PARAM_MINUS2) {
    const int delta = (pct >= 50) ? 2 : 1;
    return wrapIndex(base + ((mode == SENSOR_PARAM_PLUS2) ? delta : -delta), maxValue + 1);
  }

  if (mode == SENSOR_PARAM_PLUS3 || mode == SENSOR_PARAM_MINUS3) {
    int delta = 1;
    if (pct >= 67) delta = 3;
    else if (pct >= 34) delta = 2;
    return wrapIndex(base + ((mode == SENSOR_PARAM_PLUS3) ? delta : -delta), maxValue + 1);
  }

  if (mode == SENSOR_PARAM_FULL) {
    return map(pct, 0, 100, base, maxValue);
  }

  if (mode == SENSOR_DIV3) {
    // Far zone is largest and slowest, middle is medium/slow, close zone is small/fast.
    int delta = -2;        // farthest range
    if (pct > 50) delta = -1;
    if (pct > 85) delta = 2;
    return wrapIndex(base + delta, maxValue + 1);
  }

  return base;
}

int16_t effectiveSettingValue(uint8_t settingId) {
  if (cancelSelectedFor(settingId)) return settingRangeMax(settingId);
  if (ui.hasPendingEdit && ui.pendingSetting == settingId) return ui.pendingValue;
  int16_t base = getSettingValueRaw(settingId);
  if (!sensorParamEligible(settingId)) return base;
  const int16_t maxValue = settingRangeMax(settingId);
  base = applyDivisionOverlayMode(base, maxValue, settings.sensorMode, sensorRt.inRange, sensorPercent());
  base = applyDivisionOverlayMode(base, maxValue, settings.pushMode, pushRt.inRange, pushRt.pct);
  return base;
}

void sanitizeSettings(Settings &s) {
  s.manualBpm = constrain(static_cast<int>(s.manualBpm), 20, 300);
  s.arpMode = clampU8(s.arpMode, 0, ARP_SELECTION_COUNT - 1);
  s.division = clampU8(s.division, 0, ARP_DIVISION_FOLLOW_DRUM);
  s.arpVelocity = clampU8(s.arpVelocity, 1, 127);
  s.arpLengthPct = clampU8(s.arpLengthPct, 1, 100);
  s.inputChannel = clampU8(s.inputChannel, 1, 16);
  s.arpOutChannel = clampU8(s.arpOutChannel, 0, 16);
  s.bassMode = clampU8(s.bassMode, 0, 48);
  s.thruOutChannel = clampU8(s.thruOutChannel, 0, 16);
  s.roundRobinMask &= 0xFFFF;
  s.routerActiveMask = 0;
  for (uint8_t i = 0; i < 16; ++i) {
    if (!channelEnabled(s.routerOutChannels[i])) s.routerOutChannels[i] = i + 1;
    s.routerTranspose[i] = constrain(static_cast<int>(s.routerTranspose[i]), ROUTER_TRANSPOSE_MIN, ROUTER_TRANSPOSE_MAX);
    updateRouterActiveBit(s, i);
  }
  s.legatoChannel = clampU8(s.legatoChannel, 0, 16);
  s.ccOutChannel = clampU8(s.ccOutChannel, 1, 17);
  s.sensorChannel = clampU8(s.sensorChannel, 1, 16);
  s.sensorMode = clampU8(s.sensorMode, 0, SENSOR_MODE_COUNT - 1);
  s.pushMode = clampU8(s.pushMode, 0, SENSOR_MODE_COUNT - 1);
  s.forceKey = clampU8(s.forceKey, 0, 24);
  s.forceScale = clampU8(s.forceScale, 0, FORCE_SCALE_COUNT - 1);
  if (ckeyEnabledForValue(s.forceKey) && scaleIsCombo(s.forceScale)) s.forceScale = SCALE_MAJOR;
  s.instrumentView = clampU8(s.instrumentView, 0, 1);
  s.loadPreset = clampU8(s.loadPreset, 0, PRESET_COUNT - 1);
  s.savePreset = clampU8(s.savePreset, 0, PRESET_COUNT - 1);
  s.reserved = 0;
  s.screenSaver = clampU8(s.screenSaver, 0, SCREEN_SAVER_CANCEL_SLOT - 1);
  for (uint8_t i = 0; i < DIV_NOTE_SLOT_COUNT; ++i) {
    if (s.divNoteChannels[i] > 16) s.divNoteChannels[i] = 0;
    if (s.divNoteNotes[i] > 127) s.divNoteNotes[i] = 0xFF;
  }
  if (s.divNotePlusNote > 127) s.divNotePlusNote = 0xFF;
  s.roundRobinOptions &=
      (ROUND_ROBIN_CH10_TO_1_BIT | ROUND_ROBIN_CH10_TO_2_BIT | ROUND_ROBIN_RANDOM_BIT);
}

Firmware3Settings defaultFirmware3Settings() {
  Firmware3Settings s{};
  s.clockInFollow = 0;
  s.clockOutSend = 0;
  s.timeSignature = 0;
  s.swing = 0;
  s.looperMidiTransport = 1;
  s.looperAutoRec = 0;
  s.looperTimeTravel = 0;
  s.looperTrackMode = static_cast<uint8_t>(arpnmidi3::LoopTrackMode::Layers);
  for (uint8_t track = 0; track < arpnmidi3::kLoopTrackCount; ++track) {
    s.looperQuantize[track] = 0;
  }
  s.looperRecordCc = 0;
  s.stutterTimeoutBars = 4;
  s.arpOctaves = 1;
  s.arpRetriggerSync = 0;
  s.arpNoteOrder = 0;
  s.customArpLength = 2;  // 1 bar
  s.drumEnabled = 1;
  s.drumInputMode = 0;  // channel 10
  s.drumOutputChannel = 10;
  s.drumSplitNote = 36;
  s.drumMappedStart = 36;
  s.drumAftertouchVelocity = 0;
  s.drumDivision = DIV_1_16;
  s.quickJumpEnabled = 0;
  s.quickJumpInputChannel = 1;
  s.quickJumpOutputChannel = 2;
  s.quickJumpHold = 0;
  s.bassHighestNote = 127;
  s.parameterLockChannel = 0;
  s.forwardChannelAftertouch = 1;
  s.forwardPolyAftertouch = 1;
  s.channelAftertouchCc = 0xFF;
  s.mainAftertouchArpVelocity = 0;
  s.chordEnabled = 0;
  s.chordPositions[0] = 1;
  s.chordPositions[1] = 3;
  s.chordPositions[2] = 5;
  s.chordPositions[3] = 7;
  s.userScaleMask = 0x0AB5;  // C major pitch classes
  for (uint8_t ch = 0; ch < 16; ++ch) {
    s.routerLowNotes[ch] = 0;
    s.routerHighNotes[ch] = 127;
  }
  for (LiveTargetSettings &target : s.liveTargets) {
    target.velocityEnabled = 0;
    target.velocityPercent = 100;
    target.noteLengthEnabled = 0;
    target.noteLengthPercent = 100;
    target.stutterEnabled = 0;
    target.stutterLengthSelection = STUTTER_LENGTH_DEFAULT;
    target.echoEnabled = 0;
    target.echoWet = 50;
    target.echoLength = lengthSelectionForDivision(DIV_1_2);
    target.echoDelay = lengthSelectionForDivision(DIV_1_8);
    target.echoDrift = 0;
  }
  return s;
}

FeatureControlSettings defaultFeatureControlSettings() {
  FeatureControlSettings controls{};
  controls.fourButtonMode = FOUR_BUTTON_LOOPER;
  controls.looperButtonActions = LOOPER_BUTTON_ARM |
      LOOPER_BUTTON_DELETE | LOOPER_BUTTON_UNDO;
  for (uint8_t button = 0; button < 4; ++button) {
    controls.customButtons[button].channel = 1;
    controls.customButtons[button].number = 60 + button;
    controls.customButtons[button].kind = TRIGGER_BINDING_NOTE;
    controls.customButtons[button].behavior = CUSTOM_BUTTON_MOMENTARY;
  }
  return controls;
}

void sanitizeFirmware3Settings(Firmware3Settings &s) {
  s.clockInFollow = s.clockInFollow ? 1 : 0;
  s.clockOutSend = s.clockOutSend ? 1 : 0;
  s.timeSignature = s.timeSignature ? 1 : 0;
  s.swing = clampU8(s.swing, 0, 75);
  s.looperMidiTransport = s.looperMidiTransport ? 1 : 0;
  s.looperAutoRec = s.looperAutoRec ? 1 : 0;
  s.looperTimeTravel = s.looperTimeTravel ? 1 : 0;
  s.looperTrackMode = clampU8(s.looperTrackMode, 0,
      static_cast<uint8_t>(arpnmidi3::LoopTrackMode::Manual));
  for (uint8_t track = 0; track < arpnmidi3::kLoopTrackCount; ++track) {
    s.looperQuantize[track] = clampU8(s.looperQuantize[track], 0, LOOP_QUANTIZE_DIVISION_COUNT);
  }
  s.looperRecordCc = s.looperRecordCc ? 1 : 0;
  s.stutterTimeoutBars = clampU8(s.stutterTimeoutBars, 1, 16);
  s.arpOctaves = clampU8(s.arpOctaves, 1, 4);
  s.arpRetriggerSync = s.arpRetriggerSync ? 1 : 0;
  s.arpNoteOrder = s.arpNoteOrder ? 1 : 0;
  s.customArpLength = clampU8(s.customArpLength, 0, 5);
  s.drumEnabled = s.drumEnabled ? 1 : 0;
  s.drumInputMode = s.drumInputMode ? 1 : 0;
  s.drumOutputChannel = clampU8(s.drumOutputChannel, 1, 16);
  s.drumSplitNote = clampU8(s.drumSplitNote, 0, 120);
  s.drumMappedStart = clampU8(s.drumMappedStart, 0, 120);
  s.drumAftertouchVelocity = s.drumAftertouchVelocity ? 1 : 0;
  s.drumDivision = clampU8(s.drumDivision, 0, DRUM_DIVISION_FREE);
  s.quickJumpEnabled = s.quickJumpEnabled ? 1 : 0;
  s.quickJumpInputChannel = clampU8(s.quickJumpInputChannel, 1, 16);
  s.quickJumpOutputChannel = clampU8(s.quickJumpOutputChannel, 1, 16);
  s.quickJumpHold = s.quickJumpHold ? 1 : 0;
  s.bassHighestNote = clampU8(s.bassHighestNote, 0, 127);
  s.parameterLockChannel = clampU8(s.parameterLockChannel, 0, 16);
  s.forwardChannelAftertouch = s.forwardChannelAftertouch ? 1 : 0;
  s.forwardPolyAftertouch = s.forwardPolyAftertouch ? 1 : 0;
  if (s.channelAftertouchCc > 127) s.channelAftertouchCc = 0xFF;
  s.mainAftertouchArpVelocity = s.mainAftertouchArpVelocity ? 1 : 0;
  s.chordEnabled = s.chordEnabled ? 1 : 0;
  for (uint8_t i = 0; i < 4; ++i) {
    s.chordPositions[i] = constrain(static_cast<int>(s.chordPositions[i]), -12, 12);
  }
  s.userScaleMask &= 0x0FFF;
  if (s.userScaleMask == 0) s.userScaleMask = 1;
  for (uint8_t ch = 0; ch < 16; ++ch) {
    s.routerLowNotes[ch] = clampU8(s.routerLowNotes[ch], 0, 127);
    s.routerHighNotes[ch] = clampU8(s.routerHighNotes[ch], s.routerLowNotes[ch], 127);
  }
  for (LiveTargetSettings &target : s.liveTargets) {
    target.velocityEnabled = target.velocityEnabled ? 1 : 0;
    target.velocityPercent = clampU8(target.velocityPercent, 0, 200);
    target.noteLengthEnabled = target.noteLengthEnabled ? 1 : 0;
    target.noteLengthPercent = clampU8(target.noteLengthPercent, 1, 200);
    target.stutterEnabled = target.stutterEnabled ? 1 : 0;
    target.stutterLengthSelection = clampU8(
        target.stutterLengthSelection, STUTTER_LENGTH_MIN_SELECTION, STUTTER_LENGTH_COUNT - 1);
    target.echoEnabled = target.echoEnabled ? 1 : 0;
    target.echoWet = clampU8(target.echoWet, 0, 100);
    target.echoLength = clampU8(target.echoLength, 0, STUTTER_LENGTH_COUNT - 1);
    target.echoDelay = clampU8(target.echoDelay, 0, STUTTER_LENGTH_COUNT - 1);
    target.echoDrift = constrain(static_cast<int>(target.echoDrift), -16, 16);
  }
}

constexpr const char *DEVICE_STATE_PATH = "/state.f3";

// One scratch record rather than a stack copy: a preset record is far too large
// to build on the musical core's stack.
PresetRecord presetScratch;
PresetRecord presetCompareScratch;

size_t presetRecordOffset(uint8_t slot) {
  return sizeof(DeviceStateHeader) + static_cast<size_t>(slot) * sizeof(PresetRecord);
}

void fillPresetRecordFromLiveState(PresetRecord &record) {
  record = PresetRecord{};  // deterministic padding so compares mean something
  record.settings = settings;
  record.firmware3 = firmware3Settings;
  record.featureControls = featureControls;
  record.customArp = customArpPattern;
  record.parameterLockCount = min<uint16_t>(parameterLockCount, MAX_PARAMETER_LOCKS);
  for (uint16_t i = 0; i < MAX_PARAMETER_LOCKS; ++i) {
    record.parameterLocks[i] = i < record.parameterLockCount
        ? parameterLocks[i] : ParameterLockEntry{};
  }
}

void fillPresetRecordWithDefaults(PresetRecord &record, uint8_t slot) {
  record = PresetRecord{};
  record.settings = defaultSettings();
  record.settings.loadPreset = slot;
  record.settings.savePreset = slot;
  record.firmware3 = defaultFirmware3Settings();
  record.featureControls = defaultFeatureControlSettings();
  record.customArp = CustomArpPattern{};
  record.customArp.lengthSelection = record.firmware3.customArpLength;
  record.parameterLockCount = 0;
}

// Anything read from flash is treated as untrusted until it has been clamped
// into range, so a damaged record cannot put the engine into an impossible
// state.
void applyPresetRecord(const PresetRecord &record) {
  settings = record.settings;
  sanitizeSettings(settings);
  firmware3Settings = record.firmware3;
  sanitizeFirmware3Settings(firmware3Settings);
  featureControls = record.featureControls;
  for (FeatureKnobBinding &binding : featureControls.knobs) {
    if (binding.channel > 16 || binding.cc > 127) binding = FeatureKnobBinding{};
  }
  for (FeatureButtonBinding &binding : featureControls.buttons) {
    if (binding.channel > 16 || binding.number > 127 ||
        binding.kind > TRIGGER_BINDING_NOTE) binding = FeatureButtonBinding{};
  }
  for (CcRemapEntry &entry : featureControls.ccRemaps) {
    if (entry.inputCc > 127) entry = CcRemapEntry{};
    entry.outputChannel = clampU8(entry.outputChannel, 1, 16);
    entry.outputCc = clampU8(entry.outputCc, 0, 127);
  }
  for (uint8_t &kind : featureControls.drumRollKinds) {
    if (kind > TRIGGER_BINDING_NOTE) kind = TRIGGER_BINDING_OFF;
  }
  featureControls.fourButtonMode = clampU8(featureControls.fourButtonMode, 0,
      FOUR_BUTTON_MODE_COUNT - 1);
  featureControls.looperButtonActions &= LOOPER_BUTTON_SELECT | LOOPER_BUTTON_MUTE |
      LOOPER_BUTTON_SOLO | LOOPER_BUTTON_DELETE | LOOPER_BUTTON_UNDO | LOOPER_BUTTON_ARM;
  if (featureControls.looperButtonActions ==
      (LOOPER_BUTTON_SELECT | LOOPER_BUTTON_DELETE)) {
    featureControls.looperButtonActions |= LOOPER_BUTTON_UNDO;
  }
  if (featureControls.looperButtonActions == 0) {
    featureControls.looperButtonActions = LOOPER_BUTTON_SELECT;
  }
  for (CustomButtonConfig &button : featureControls.customButtons) {
    button.channel = clampU8(button.channel, 1, 16);
    button.number = clampU8(button.number, 0, 127);
    if (button.kind != TRIGGER_BINDING_CC && button.kind != TRIGGER_BINDING_NOTE) {
      button.kind = TRIGGER_BINDING_NOTE;
    }
    button.behavior = clampU8(button.behavior, 0, CUSTOM_BUTTON_BEHAVIOR_COUNT - 1);
  }
  for (ChordMemorySlot &chord : featureControls.chordMemories) {
    chord.count = clampU8(chord.count, 0, 16);
    for (uint8_t note = 0; note < chord.count; ++note) {
      chord.channels[note] = clampU8(chord.channels[note], 1, 16);
      chord.notes[note] = clampU8(chord.notes[note], 0, 127);
      chord.velocities[note] = clampU8(chord.velocities[note], 1, 127);
    }
  }
  for (NoteCcMapEntry &entry : featureControls.noteCcMaps) {
    if (entry.inputChannel > 16 || entry.inputNote > 127) entry = NoteCcMapEntry{};
    entry.outputChannel = clampU8(entry.outputChannel, 1, 16);
    entry.outputCc = clampU8(entry.outputCc, 0, 127);
    entry.behavior = entry.behavior == NOTE_CC_TOGGLE ? NOTE_CC_TOGGLE : NOTE_CC_MOMENTARY;
  }
  memset(noteCcToggleState, 0, sizeof(noteCcToggleState));
  memset(featureButtonCcHeld, 0, sizeof(featureButtonCcHeld));
  customArpPattern = record.customArp;
  customArpPattern.lengthSelection = clampU8(customArpPattern.lengthSelection, 0, 5);
  parameterLockCount = record.parameterLockCount;
  memcpy(parameterLocks, record.parameterLocks,
         parameterLockCount * sizeof(ParameterLockEntry));
}

bool readPresetRecord(uint8_t slot, PresetRecord &record) {
  if (!littleFsReady || slot >= PRESET_COUNT) return false;
  File file = LittleFS.open(DEVICE_STATE_PATH, "r");
  if (!file) return false;
  const bool ok = file.seek(presetRecordOffset(slot), SeekSet) &&
                  file.read(reinterpret_cast<uint8_t *>(&record), sizeof(record)) == sizeof(record);
  file.close();
  if (!ok || record.customArp.count > MAX_CUSTOM_ARP_EVENTS ||
      record.parameterLockCount > MAX_PARAMETER_LOCKS) return false;
  return true;
}

bool writePresetRecord(uint8_t slot, const PresetRecord &record) {
  if (!littleFsReady || slot >= PRESET_COUNT) return false;
  File file = LittleFS.open(DEVICE_STATE_PATH, "r+");
  if (!file) return false;
  const bool ok = file.seek(presetRecordOffset(slot), SeekSet) &&
                  file.write(reinterpret_cast<const uint8_t *>(&record), sizeof(record)) == sizeof(record);
  file.close();
  return ok;
}

// Learned content, chord memories, custom arp, and four-button setup, is kept
// even when Auto Save is off. It is written without disturbing the settings
// stored in that slot, so Auto Save still governs knob changes alone. A slot
// that cannot be read has no settings worth preserving, so the whole live state
// is written instead and the learn is not lost to a retry loop.
bool savePresetLearnedContent(uint8_t slot) {
  if (!littleFsReady) {
    // Nothing can be stored without a filesystem. Reporting success stops the
    // caller from retrying forever, and the diagnostics screen already says
    // FS NONE.
    storageError = true;
    return true;
  }
  const bool readable = readPresetRecord(slot, presetCompareScratch);
  fillPresetRecordFromLiveState(presetScratch);
  if (readable) presetScratch.settings = presetCompareScratch.settings;
  if (readable &&
      memcmp(&presetScratch, &presetCompareScratch, sizeof(PresetRecord)) == 0) {
    return true;
  }
  showBusyHourglass();
  const bool ok = writePresetRecord(slot, presetScratch);
  endBusyHourglass();
  if (!ok) storageRetryHoldUntilMs = millis() + 5000UL;
  return ok;
}

// The header carries the device-global state: which preset is live, Auto Save,
// and the screen to return to. It is twelve bytes at the front of the file, so
// remembering a screen costs one small block write.
bool writeDeviceStateHeader() {
  if (!littleFsReady) return false;
  File file = LittleFS.open(DEVICE_STATE_PATH, "r+");
  if (!file) return false;
  storage.magic = DEVICE_STATE_MAGIC;
  storage.recordSize = sizeof(PresetRecord);
  storage.presetCount = PRESET_COUNT;
  const bool ok = file.seek(0, SeekSet) &&
                  file.write(reinterpret_cast<const uint8_t *>(&storage), sizeof(storage)) == sizeof(storage);
  file.close();
  return ok;
}

bool initializeDeviceState(bool forceFactoryDefaults) {
  if (!littleFsReady) return false;
  if (!forceFactoryDefaults) {
    File file = LittleFS.open(DEVICE_STATE_PATH, "r");
    DeviceStateHeader header{};
    bool valid = false;
    if (file && file.read(reinterpret_cast<uint8_t *>(&header), sizeof(header)) == sizeof(header)) {
      const size_t expected = sizeof(header) + PRESET_COUNT * sizeof(PresetRecord);
      valid = header.magic == DEVICE_STATE_MAGIC &&
              header.recordSize == sizeof(PresetRecord) &&
              header.presetCount == PRESET_COUNT && file.size() == expected;
    }
    if (file) file.close();
    if (valid) {
      storage = header;
      storage.currentPreset = clampU8(storage.currentPreset, 0, PRESET_COUNT - 1);
      storage.autoSave = storage.autoSave ? 1 : 0;
      return true;
    }
  }

  File file = LittleFS.open(DEVICE_STATE_PATH, "w");
  if (!file) return false;
  storage = DeviceStateHeader{};
  storage.recordSize = sizeof(PresetRecord);
  bool ok = file.write(reinterpret_cast<const uint8_t *>(&storage), sizeof(storage)) == sizeof(storage);
  for (uint8_t slot = 0; ok && slot < PRESET_COUNT; ++slot) {
    fillPresetRecordWithDefaults(presetScratch, slot);
    ok = file.write(reinterpret_cast<const uint8_t *>(&presetScratch),
                    sizeof(presetScratch)) == sizeof(presetScratch);
  }
  file.close();
  if (!ok) LittleFS.remove(DEVICE_STATE_PATH);
  return ok;
}

void captureFeatureKnobDefaults() {
  featureKnobDefaults.division = settings.division;
  featureKnobDefaults.drumDivision = firmware3Settings.drumDivision;
  featureKnobDefaults.quickJumpInputChannel = firmware3Settings.quickJumpInputChannel;
  featureKnobDefaults.quickJumpOutputChannel = firmware3Settings.quickJumpOutputChannel;
  featureKnobDefaults.manualBpm = settings.manualBpm;
  featureKnobDefaults.swing = firmware3Settings.swing;
  featureKnobDefaults.arpMode = settings.arpMode;
  featureKnobDefaults.arpVelocity = settings.arpVelocity;
  featureKnobDefaults.arpLengthPct = settings.arpLengthPct;
  featureKnobDefaults.arpOctaves = firmware3Settings.arpOctaves;
  for (uint8_t i = 0; i < STUTTER_ECHO_TARGET_COUNT; ++i) {
    featureKnobDefaults.liveTargets[i] = firmware3Settings.liveTargets[i];
  }
}

// A mapped Feature Knob's "temp change" is a direct, permanent-looking edit
// to the live setting; nothing marks it as an override or ever un-applies it
// on its own, so a controller sitting at a stale non-zero position leaves
// that value in place indefinitely. This puts every one of those fields
// back to captureFeatureKnobDefaults' snapshot, whatever the preset actually
// has saved, so a hold-to-panic or a PANIC screen click both let every
// mapped knob's effect sleep until the controller sends a fresh value,
// rather than pretending zero arrived, which is not the same thing for
// several of these (BPM's zero is 20, Arp Velocity's is 1, Echo Drift's is
// its most negative value, not "off").
void restoreFeatureKnobDefaults() {
  settings.division = featureKnobDefaults.division;
  firmware3Settings.drumDivision = featureKnobDefaults.drumDivision;
  firmware3Settings.quickJumpInputChannel = featureKnobDefaults.quickJumpInputChannel;
  firmware3Settings.quickJumpOutputChannel = featureKnobDefaults.quickJumpOutputChannel;
  settings.manualBpm = featureKnobDefaults.manualBpm;
  firmware3Settings.swing = featureKnobDefaults.swing;
  settings.arpMode = featureKnobDefaults.arpMode;
  settings.arpVelocity = featureKnobDefaults.arpVelocity;
  settings.arpLengthPct = featureKnobDefaults.arpLengthPct;
  firmware3Settings.arpOctaves = featureKnobDefaults.arpOctaves;
  for (uint8_t i = 0; i < STUTTER_ECHO_TARGET_COUNT; ++i) {
    firmware3Settings.liveTargets[i] = featureKnobDefaults.liveTargets[i];
  }
  syncArpDivisionToGrid();
  syncMusicalClockConfig(false);
  ui.dirty = true;
}

void loadCurrentPreset() {
  screenSaverForceNow = false;
  memset(parameterLockHeldNotes, 0, sizeof(parameterLockHeldNotes));
  if (!readPresetRecord(storage.currentPreset, presetScratch)) {
    fillPresetRecordWithDefaults(presetScratch, storage.currentPreset);
    storageError = littleFsReady;
  }
  applyPresetRecord(presetScratch);
  if (settings.division == ARP_DIVISION_FOLLOW_DRUM &&
      firmware3Settings.drumDivision == DRUM_DIVISION_FOLLOW_ARP) {
    firmware3Settings.drumDivision = DIV_1_16;
  }
  syncMusicalClockConfig(false);
  divNotesCursor = 0;
  featuresUiStage = FEATURES_UI_GROUPS;
  featuresGroupCursor = 0;
  featuresItemCursor = 0;
  featuresLearnActive = false;
  ccRemapUiStage = CC_REMAP_UI_LIST;
  ccRemapCursor = 0;
  ccRemapLearnActive = false;
  noteCcUiStage = NOTE_CC_UI_LIST;
  noteCcCursor = 0;
  noteCcLearnActive = false;
  memset(noteCcToggleState, 0, sizeof(noteCcToggleState));
  fourButtonUiStage = FOUR_BUTTON_UI_MAIN;
  fourButtonUiCursor = 0;
  fourButtonEditButton = 0;
  fourButtonLearnActive = false;
  memset(customButtonLatch, 0, sizeof(customButtonLatch));
  memset(customButtonFlappyValue, 0, sizeof(customButtonFlappyValue));
  memset(looperButtonStep, 0, sizeof(looperButtonStep));
  memset(chordButtonPlaying, 0, sizeof(chordButtonPlaying));
  chordLearnArmed = false;
  chordClearArmed = false;
  chordLearnActive = false;
  extendedPresetDirty = false;
  settings.loadPreset = storage.currentPreset;
  settings.savePreset = storage.currentPreset;
  captureFeatureKnobDefaults();
  ui.dirty = true;
}

void stagePersistedUiSetting(uint8_t settingId) {
  if (settingId >= SETTING_COUNT || !selectableSetting(settingId)) return;
  storage.lastScreen = settingId;
  persistedUiSetting = settingId;
  uiScreenSavePending = false;
}

bool loadPersistedUiSetting(uint8_t &settingId) {
  const uint8_t stored = storage.lastScreen;
  if (stored >= SETTING_COUNT || !selectableSetting(stored)) return false;
  settingId = stored;
  persistedUiSetting = stored;
  return true;
}

// The old EEPROM emulation had a property worth keeping: writing a byte that
// already held that value marked nothing dirty, so an exit with no edits never
// touched flash. These compares restore that property for the state file.
bool presetRecordDiffersFromStored(uint8_t slot, const PresetRecord &record) {
  if (!readPresetRecord(slot, presetCompareScratch)) return true;
  return memcmp(&record, &presetCompareScratch, sizeof(PresetRecord)) != 0;
}

// The remembered screen alone never justifies a write. Only the fields the
// performer actually owns can force one.
bool deviceHeaderCoreDiffers() {
  File file = LittleFS.open(DEVICE_STATE_PATH, "r");
  DeviceStateHeader stored{};
  const bool ok = file &&
      file.read(reinterpret_cast<uint8_t *>(&stored), sizeof(stored)) == sizeof(stored);
  if (file) file.close();
  if (!ok) return true;
  return stored.currentPreset != storage.currentPreset ||
         stored.autoSave != storage.autoSave;
}

// One save writes one preset record and the header, at the moment of
// commitment. Nothing changed means nothing written, no hourglass, no pause.
bool saveStorage() {
  if (!littleFsReady) {
    storageError = true;
    presetStorageDirty = false;
    extendedPresetDirty = false;
    return false;
  }
  settings.loadPreset = storage.currentPreset;
  settings.savePreset = storage.currentPreset;
  fillPresetRecordFromLiveState(presetScratch);
  const bool recordChanged =
      presetRecordDiffersFromStored(storage.currentPreset, presetScratch);
  if (!recordChanged && !deviceHeaderCoreDiffers()) {
    presetStorageDirty = false;
    extendedPresetDirty = false;
    captureFeatureKnobDefaults();
    return true;
  }
  stagePersistedUiSetting(ui.selectedSetting);
  showBusyHourglass();
  bool ok = true;
  if (recordChanged) ok = writePresetRecord(storage.currentPreset, presetScratch);
  ok = writeDeviceStateHeader() && ok;
  endBusyHourglass();
  storageError = !ok;
  if (!ok) {
    storageRetryHoldUntilMs = millis() + 5000UL;
    return false;
  }
  extendedPresetDirty = false;
  presetStorageDirty = false;
  // What just landed on flash is the new as-booted baseline: any live
  // Feature Knob value now saved with it is no longer a temp override, a
  // panic reset should keep it, not revert past it.
  captureFeatureKnobDefaults();
  return true;
}

void saveStorageIfAuto() {
  if (!storage.autoSave) return;
  presetStorageDirty = true;
  presetStorageDirtyMs = millis();
}

Settings defaultSettings() {
  Settings s{};
  s.manualBpm = 120;
  s.arpMode = ARPSEL_UP;
  s.division = DIV_1_16;
  s.arpVelocity = 96;
  s.arpLengthPct = 55;
  s.inputChannel = 1;
  s.arpOutChannel = 1;
  s.bassMode = 0;
  s.thruOutChannel = 2;
  s.roundRobinMask = 0;
  clearRouterMappings(s);
  s.legatoChannel = 0;
  s.ccOutChannel = 17;
  s.sensorChannel = 3;
  s.sensorMode = SENSOR_OFF;
  s.pushMode = SENSOR_OFF;
  s.forceKey = 0;
  s.forceScale = SCALE_OFF;
  s.instrumentView = 0;
  s.loadPreset = 0;
  s.savePreset = 0;
  s.reserved = 0;
  s.screenSaver = 2;
  for (uint8_t i = 0; i < DIV_NOTE_SLOT_COUNT; ++i) {
    s.divNoteChannels[i] = 0;
    s.divNoteNotes[i] = 0xFF;
  }
  s.divNotePlusNote = 0xFF;
  s.roundRobinOptions = 0;
  return s;
}

void initStorageIfNeeded() {
  storage = DeviceStateHeader{};
  storageError = !initializeDeviceState(factoryResetRequested);
  factoryResetRequested = false;
  loadCurrentPreset();
  loadSavedLoopStorage();
}

void applySettingDelta(int delta, bool fastStep) {
  const uint8_t id = ui.selectedSetting;
  if (id == SET_PANIC) return;
  if (id == SET_MAP_CC) featuresLearnActive = false;
  if (id == SET_CC_MAP && ccRemapUiStage == CC_REMAP_UI_INPUT) ccRemapLearnActive = false;
  if (id == SET_NOTE_CC && (noteCcUiStage == NOTE_CC_UI_INPUT_CHANNEL ||
                            noteCcUiStage == NOTE_CC_UI_INPUT_NOTE ||
                            noteCcUiStage == NOTE_CC_UI_OUTPUT_CHANNEL ||
                            noteCcUiStage == NOTE_CC_UI_OUTPUT_CC)) {
    noteCcLearnActive = false;
  }
  if (id == SET_FOUR_BUTTON && fourButtonUiStage == FOUR_BUTTON_UI_CUSTOM_NUMBER) {
    fourButtonLearnActive = false;
  }
  const int fast = fastStep ? 10 : 1;
  const int step = delta * fast;
  const int oldValue = cancelSelectedFor(id)
      ? settingRangeMax(id)
      : ((ui.hasPendingEdit && ui.pendingSetting == id) ? ui.pendingValue : getSettingValueRaw(id));
  int next = oldValue + step;
  const int maxValue = settingRangeMax(id);

  if (id == SET_BPM) next = constrain(next, 20, 300);
  else if (id == SET_INPUT_CH || id == SET_SENSOR_CH) next = wrapIndex(next - 1, maxValue) + 1;
  else if (id == SET_CC_OUT_CH) next = wrapIndex(next - 1, maxValue) + 1;
  else if (id == SET_ROUTER && routerEditStage == ROUTER_STAGE_DEST) next = wrapIndex(next - 1, 16) + 1;
  else if (id == SET_ROUTER && routerEditStage == ROUTER_STAGE_TRANSPOSE) {
    next = constrain(next, 0, ROUTER_TRANSPOSE_MAX - ROUTER_TRANSPOSE_MIN);
  }
  else if (id == SET_ROUTER && routerEditStage == ROUTER_STAGE_LOW_NOTE) {
    next = constrain(next, 0, firmware3Settings.routerHighNotes[routerEditChannel]);
  }
  else if (id == SET_ROUTER && routerEditStage == ROUTER_STAGE_HIGH_NOTE) {
    next = constrain(next, firmware3Settings.routerLowNotes[routerEditChannel], 127);
  }
  else if (id == SET_CC_MAP && ccRemapUiStage == CC_REMAP_UI_OUTPUT_CHANNEL) {
    next = wrapIndex(next - 1, 16) + 1;
  }
  else if (id == SET_FOUR_BUTTON && fourButtonUiStage == FOUR_BUTTON_UI_CUSTOM_CHANNEL) {
    next = wrapIndex(next - 1, FOUR_BUTTON_CANCEL_CHANNEL) + 1;
  }
  else if (id == SET_FOUR_BUTTON && fourButtonUiStage == FOUR_BUTTON_UI_CUSTOM_NUMBER) {
    next = wrapIndex(next, FOUR_BUTTON_CANCEL_NUMBER + 1);
  }
  else if (id == SET_NOTE_CC && (noteCcUiStage == NOTE_CC_UI_INPUT_CHANNEL ||
                                 noteCcUiStage == NOTE_CC_UI_OUTPUT_CHANNEL)) {
    next = wrapIndex(next - 1, NOTE_CC_CANCEL_CHANNEL) + 1;
  }
  else if (id == SET_NOTE_CC && (noteCcUiStage == NOTE_CC_UI_INPUT_NOTE ||
                                 noteCcUiStage == NOTE_CC_UI_OUTPUT_CC)) {
    next = wrapIndex(next, NOTE_CC_CANCEL_VALUE + 1);
  }
  else if (id == SET_NOTE_CC && noteCcUiStage == NOTE_CC_UI_BEHAVIOR) {
    next = wrapIndex(next, NOTE_CC_CANCEL_BEHAVIOR + 1);
  }
  else if ((id == SET_QUICK_JUMP && quickJumpUi.editing && quickJumpUi.cursor < 2) ||
           (id == SET_DRUM_MAGIC && drumMagicUi.editing && drumMagicUi.cursor == 2)) {
    next = wrapIndex(next - 1, maxValue) + 1;
  }
  else if ((id == SET_ARP_MODE && arpMenuUi.editing &&
            (arpMenuUi.cursor == 2 || arpMenuUi.cursor == 3 || arpMenuUi.cursor == 4)) ||
           (id == SET_LIVE_NOTE_LENGTH && liveNoteLengthUi.editing &&
            liveNoteLengthUi.cursor == 2) ||
           (id == SET_STUTTER && stutterUi.editing && stutterUi.cursor == 2)) {
    next = wrapIndex(next - 1, maxValue) + 1;
  }
  else if (id == SET_VELOCITY) next = constrain(next, 1, 127);
  else if (id == SET_LENGTH) next = constrain(next, 1, 100);
  else if (id == SET_FORCE_SCALE && scaleUi.editing) {
    if (ckeyEnabled()) {
      const int direction = (delta >= 0) ? 1 : -1;
      int candidate = oldValue;
      for (uint8_t i = 0; i < FORCE_SCALE_COUNT + 2; ++i) {
        candidate += direction;
        if (candidate < 1) candidate = maxValue;
        if (candidate > maxValue) candidate = 1;
        if (!scaleIsCombo(candidate)) break;
      }
      next = constrain(candidate, 1, maxValue);
    } else {
      next = constrain(next, 1, maxValue);
    }
  }
  else next = wrapIndex(next, maxValue + 1);

  if (next == oldValue) return;
  // Stutter and Echo's ON/OFF field is a plain two-way toggle: reserving a
  // third position for Cancel there would mean turning past ON before it
  // wraps back to OFF, so it is excluded here even though the rest of both
  // submenus keep Cancel.
  const bool onOffFieldExcludedFromCancel =
      (id == SET_STUTTER && stutterUi.cursor == 1) ||
      (id == SET_ECHO && echoUi.cursor == 1);
  const bool cancelSelectable =
      (!onOffFieldExcludedFromCancel && submenuCancelEnabled(id) &&
       ui.menuMode == MENU_EDIT &&
       submenuStateForSetting(id) && submenuStateForSetting(id)->editing) ||
      (directCancelEnabled(id) && ui.menuMode == MENU_EDIT) ||
      (id == SET_FOUR_BUTTON && ui.menuMode == MENU_EDIT &&
       fourButtonUiStage >= FOUR_BUTTON_UI_CUSTOM_CHANNEL &&
       fourButtonUiStage <= FOUR_BUTTON_UI_CUSTOM_BEHAVIOR) ||
      (id == SET_NOTE_CC && ui.menuMode == MENU_EDIT &&
       noteCcUiStage >= NOTE_CC_UI_INPUT_CHANNEL &&
       noteCcUiStage <= NOTE_CC_UI_BEHAVIOR) ||
      (id == SET_PARAMETER_LOCK && ui.menuMode == MENU_EDIT &&
       parameterLockUi.editing && parameterLockUi.cursor == 0) ||
      (id == SET_FORCE_SCALE && ui.menuMode == MENU_EDIT &&
       scaleUi.editing && scaleUi.cursor == 0);
  if (cancelSelectable && next == maxValue) {
    selectCancelFor(id);
    ui.dirty = true;
    markActivity();
    return;
  }
  clearEditCancelSelection();
  if (settingNeedsPanic(id) && ui.menuMode == MENU_EDIT) {
    ui.hasPendingEdit = true;
    ui.pendingSetting = id;
    ui.pendingValue = next;
  } else {
    if (settingNeedsPanic(id)) panicMidiOnly();
    setSettingValueRaw(id, next);
  }

  if (id == SET_SWING || id == SET_DIVISION || id == SET_LENGTH || id == SET_VELOCITY ||
      (id == SET_ARP_MODE && arpMenuUi.editing && arpMenuUi.cursor <= 4)) {
    restartArpTiming(true);
  }

  ui.dirty = true;
  markActivity();
}

void resetCurrentPresetToFactory() {
  panicAll();
  settings = defaultSettings();
  settings.loadPreset = storage.currentPreset;
  settings.savePreset = storage.currentPreset;
  firmware3Settings = defaultFirmware3Settings();
  featureControls = defaultFeatureControlSettings();
  customArpPattern = CustomArpPattern{};
  customArpPattern.lengthSelection = firmware3Settings.customArpLength;
  parameterLockCount = 0;
  for (ParameterLockEntry &entry : parameterLocks) entry = ParameterLockEntry{};
  saveStorage();
  syncMusicalClockConfig(true);
  ui.dirty = true;
}

bool submenuCancelEnabled(uint8_t settingId) {
  return settingId == SET_QUICK_JUMP || settingId == SET_ARP_MODE ||
         settingId == SET_STUTTER || settingId == SET_ECHO ||
         settingId == SET_BASS_CH;
}

bool directCancelEnabled(uint8_t settingId) {
  return settingId == SET_INPUT_CH || settingId == SET_ARP_OUT_CH ||
         settingId == SET_THRU_OUT_CH || settingId == SET_CC_OUT_CH ||
         settingId == SET_SENSOR_CH || settingId == SET_SENSOR_MODE ||
         settingId == SET_PUSH_MODE;
}

bool settingStoredInCurrentPreset(uint8_t settingId) {
  return settingId != SET_PANIC && settingId != SET_LOAD_PRESET &&
         settingId != SET_SAVE_PRESET;
}

void saveCurrentPresetIfSettingStored(uint8_t settingId) {
  if (settingStoredInCurrentPreset(settingId)) saveStorageIfAuto();
}

void beginSubmenuParameterEdit(SubmenuUiState &state) {
  state.editing = true;
  if (submenuCancelEnabled(ui.selectedSetting)) {
    submenuEditBackupSetting = ui.selectedSetting;
    submenuEditBackupCursor = state.cursor;
    submenuEditBackupValue = getSettingValueRaw(ui.selectedSetting);
    submenuEditBackupValid = true;
  }
}

void finishSubmenuParameterEdit() {
  submenuEditBackupValid = false;
  clearEditCancelSelection();
}

void cancelSubmenuParameterEdit(SubmenuUiState &state) {
  if (submenuEditBackupValid &&
      submenuEditBackupSetting == ui.selectedSetting &&
      submenuEditBackupCursor == state.cursor) {
    const int16_t restoreValue = submenuEditBackupValue;
    submenuEditBackupValid = false;
    clearEditCancelSelection();
    setSettingValueRaw(ui.selectedSetting, restoreValue);
  }
  state.editing = false;
  clearEditCancelSelection();
  ui.dirty = true;
}

SubmenuUiState *submenuStateForSetting(uint8_t settingId) {
  if (settingId == SET_QUICK_JUMP) return &quickJumpUi;
  if (settingId == SET_ARP_MODE) return &arpMenuUi;
  if (settingId == SET_STUTTER) return &stutterUi;
  if (settingId == SET_ECHO) return &echoUi;
  if (settingId == SET_BASS_CH) return &bassUi;
  return nullptr;
}

bool finishSubmenuOrEdit(SubmenuUiState &state, uint8_t backCursor) {
  if (state.editing) {
    if (cancelSelectedFor(ui.selectedSetting)) {
      cancelSubmenuParameterEdit(state);
      return true;
    }
    state.editing = false;
    finishSubmenuParameterEdit();
    clearEditCancelSelection();
    saveCurrentPresetIfSettingStored(ui.selectedSetting);
    return true;
  }
  if (state.cursor != backCursor) {
    beginSubmenuParameterEdit(state);
    return true;
  }
  ui.menuMode = MENU_SELECT;
  ui.deferredExitWork = true;
  return true;
}

void cancelDirectEdit() {
  ui.hasPendingEdit = false;
  ui.menuMode = MENU_SELECT;
  clearEditCancelSelection();
  ui.dirty = true;
}

bool handleFirmware3SubmenuClick() {
  switch (ui.selectedSetting) {
    case SET_ARP_MODE:
      if (arpMenuUi.editing) {
        if (cancelSelectedFor(SET_ARP_MODE)) {
          cancelSubmenuParameterEdit(arpMenuUi);
          return true;
        }
        arpMenuUi.editing = false;
        finishSubmenuParameterEdit();
        clearEditCancelSelection();
        saveCurrentPresetIfSettingStored(SET_ARP_MODE);
      }
      else if (arpMenuUi.cursor == 8) {
        if (customArpLearning) finishCustomArpLearn();
        else startCustomArpLearn();
      } else if (arpMenuUi.cursor == 9) {
        clearCustomArpPattern();
        saveCurrentPresetIfSettingStored(SET_ARP_MODE);
      } else if (arpMenuUi.cursor == 10) {
        ui.menuMode = MENU_SELECT;
        ui.deferredExitWork = true;
      } else beginSubmenuParameterEdit(arpMenuUi);
      return true;
    case SET_LIVE_VELOCITY: return finishSubmenuOrEdit(liveVelocityUi, 3);
    case SET_LIVE_NOTE_LENGTH: return finishSubmenuOrEdit(liveNoteLengthUi, 3);
    case SET_STUTTER: return finishSubmenuOrEdit(stutterUi, 4);
    case SET_ECHO: return finishSubmenuOrEdit(echoUi, 6);
    case SET_QUICK_JUMP: return finishSubmenuOrEdit(quickJumpUi, 4);
    case SET_DRUM_MAGIC: return finishSubmenuOrEdit(drumMagicUi, 7);
    case SET_BASS_CH: return finishSubmenuOrEdit(bassUi, 3);
    case SET_LOOP_BARS: {
      const bool wasEditing = looperSettingsUi.editing;
      const bool handled = finishSubmenuOrEdit(looperSettingsUi, 8);
      // Committing any value hops straight back to the LOOPER summary. The
      // submenu is a place to change one thing, not a place to live.
      if (wasEditing && !looperSettingsUi.editing && ui.menuMode == MENU_EDIT) {
        ui.menuMode = MENU_SELECT;
        ui.deferredExitWork = true;
      }
      return handled;
    }
    case SET_PARAMETER_LOCK:
      if (parameterLockUi.editing) {
        if (cancelSelectedFor(SET_PARAMETER_LOCK)) {
          firmware3Settings.parameterLockChannel =
              clampU8(submenuEditBackupValue, 0, 16);
        }
        parameterLockUi.editing = false;
        finishSubmenuParameterEdit();
        saveCurrentPresetIfSettingStored(SET_PARAMETER_LOCK);
      }
      else if (parameterLockUi.cursor == 0) beginSubmenuParameterEdit(parameterLockUi);
      else if (parameterLockUi.cursor == 1) {
        parameterLockCount = 0;
        for (ParameterLockEntry &entry : parameterLocks) entry = ParameterLockEntry{};
        saveCurrentPresetIfSettingStored(SET_PARAMETER_LOCK);
      } else {
        ui.menuMode = MENU_SELECT;
        ui.deferredExitWork = true;
      }
      return true;
    case SET_CHORD: return finishSubmenuOrEdit(chordUi, 5);
    case SET_FORCE_SCALE:
      if (scaleUi.editing) {
        if (cancelSelectedFor(SET_FORCE_SCALE)) {
          settings.forceScale = clampU8(submenuEditBackupValue, 0, FORCE_SCALE_COUNT - 1);
        }
        scaleUi.editing = false;
        finishSubmenuParameterEdit();
        saveCurrentPresetIfSettingStored(SET_FORCE_SCALE);
      } else if (scaleUi.cursor == 0) {
        beginSubmenuParameterEdit(scaleUi);
      } else if (scaleUi.cursor <= 12) {
        const uint16_t bit = static_cast<uint16_t>(1U << (scaleUi.cursor - 1U));
        if ((firmware3Settings.userScaleMask & bit) == 0 ||
            firmware3Settings.userScaleMask != bit) {
          firmware3Settings.userScaleMask ^= bit;
          saveCurrentPresetIfSettingStored(SET_FORCE_SCALE);
        }
      } else {
        ui.menuMode = MENU_SELECT;
        ui.deferredExitWork = true;
      }
      return true;
    case SET_LIVE_CC:
      if (liveCcEditing) {
        liveCcEditing = false;
        saveCurrentPresetIfSettingStored(SET_LIVE_CC);
      }
      else if (liveCcCursor < 2) liveCcEditing = true;
      else {
        ui.menuMode = MENU_SELECT;
        ui.deferredExitWork = true;
      }
      return true;
    case SET_SCREEN_SAVER:
      if (screenSaverCursor != SCREEN_SAVER_CANCEL_SLOT) {
        settings.screenSaver = screenSaverCursor;
        screenSaverForceNow = false;
        saveStorageIfAuto();
      }
      screenSaverCursor = settings.screenSaver;
      ui.menuMode = MENU_SELECT;
      return true;
    case SET_GLOBAL:
      if (globalUi.editing) {
        globalUi.editing = false;
        saveCurrentPresetIfSettingStored(SET_GLOBAL);
      }
      else if (globalUi.cursor == 8) resetCurrentPresetToFactory();
      else if (globalUi.cursor == 9) {
        ui.menuMode = MENU_SELECT;
        ui.deferredExitWork = true;
      } else globalUi.editing = true;
      return true;
    default: return false;
  }
}

void activateClickAction() {
  // A fast double click on the LOOPER summary arms the working track. The
  // first click of the pair enters the submenu as usual, and the second one
  // backs out and arms, so slower clicks keep their normal meaning.
  if (ui.selectedSetting == SET_LOOP_BARS) {
    const uint32_t clickMs = millis();
    const bool fastPair = (clickMs - looperScreenClickMs) <= 400UL;
    looperScreenClickMs = clickMs;
    if (fastPair && ui.menuMode == MENU_EDIT && !looperSettingsUi.editing &&
        looperSettingsUi.cursor == 0) {
      ui.menuMode = MENU_SELECT;
      toggleLooperArmForTrack(multitrackLooper.selectedTrack());
      releaseSilencedMultitrackOutputs();
      refreshLoopUiState();
      encoder.switchIgnoreUntilMs = millis() + 120;
      ui.dirty = true;
      markActivity();
      return;
    }
  }
  if (ui.selectedSetting == SET_PANIC) {
    // Same as the held encoder gesture: put every mapped Feature Knob's live
    // value back to what the preset has saved before the panic sweep itself.
    restoreFeatureKnobDefaults();
    panicAll();
    return;
  }
  if (ui.selectedSetting == SET_SAVE_PRESET) {
    if (ui.menuMode == MENU_EDIT) {
      ui.menuMode = MENU_SELECT;
      ui.deferredSaveOnly = true;
      ui.deferredExitWork = true;
    } else {
      ui.menuMode = MENU_EDIT;
    }
    encoder.switchIgnoreUntilMs = millis() + 120;
    ui.dirty = true;
    return;
  }
  if (ui.menuMode == MENU_EDIT) {
    if (directCancelEnabled(ui.selectedSetting) && cancelSelectedFor(ui.selectedSetting)) {
      cancelDirectEdit();
      encoder.switchIgnoreUntilMs = millis() + 120;
      ui.dirty = true;
      markActivity();
      return;
    }
    if (handleFirmware3SubmenuClick()) {
      encoder.switchIgnoreUntilMs = millis() + 120;
      ui.dirty = true;
      markActivity();
      return;
    }
    if (ui.selectedSetting == SET_MAP_CC) {
      if (featuresUiStage == FEATURES_UI_GROUPS) {
        if (featuresGroupCursor == 0) {
          featuresUiStage = FEATURES_UI_KNOBS;
          featuresItemCursor = 0;
          featuresItemOpen = false;
        } else if (featuresGroupCursor == 1) {
          featuresUiStage = FEATURES_UI_BUTTONS;
          featuresItemCursor = 0;
          featuresItemOpen = false;
        } else if (featuresGroupCursor == 2) {
          for (FeatureKnobBinding &binding : featureControls.knobs) {
            binding = FeatureKnobBinding{};
          }
          for (FeatureButtonBinding &binding : featureControls.buttons) {
            binding = FeatureButtonBinding{};
          }
          memset(featureButtonCcHeld, 0, sizeof(featureButtonCcHeld));
          saveStorageIfAuto();
        } else {
          // Back exits through the common edit completion below.
          featuresLearnActive = false;
          ui.menuMode = MENU_SELECT;
          ui.deferredExitWork = true;
        }
      } else {
        const uint8_t count = featuresUiStage == FEATURES_UI_KNOBS
            ? static_cast<uint8_t>(FEATURE_KNOB_COUNT)
            : static_cast<uint8_t>(FEATURE_BUTTON_COUNT);
        if (featuresItemCursor >= count) {
          featuresUiStage = FEATURES_UI_GROUPS;
          featuresGroupCursor = 0;
          featuresLearnActive = false;
          featuresItemOpen = false;
        } else if (!featuresItemOpen) {
          // First click on a list row opens its detail view; the row itself
          // was already reached by turning, not clicking.
          featuresItemOpen = true;
        } else {
          featuresLearnActive = !featuresLearnActive;
        }
      }
      encoder.switchIgnoreUntilMs = millis() + 120;
      ui.dirty = true;
      markActivity();
      return;
    } else if (ui.selectedSetting == SET_CC_MAP) {
      if (ccRemapUiStage == CC_REMAP_UI_LIST) {
        if (ccRemapCursor < CC_REMAP_SLOT_COUNT) {
          ccRemapUiStage = CC_REMAP_UI_INPUT;
          ccRemapLearnActive = true;
        } else if (ccRemapCursor == CC_REMAP_SLOT_COUNT) {
          for (CcRemapEntry &entry : featureControls.ccRemaps) entry = CcRemapEntry{};
          saveStorageIfAuto();
        } else {
          ui.menuMode = MENU_SELECT;
          ui.deferredExitWork = true;
        }
      } else if (ccRemapUiStage == CC_REMAP_UI_INPUT) {
        ccRemapLearnActive = false;
        ccRemapUiStage = CC_REMAP_UI_OUTPUT_CHANNEL;
      } else if (ccRemapUiStage == CC_REMAP_UI_OUTPUT_CHANNEL) {
        ccRemapUiStage = CC_REMAP_UI_OUTPUT_CC;
      } else {
        ccRemapUiStage = CC_REMAP_UI_LIST;
        saveStorageIfAuto();
      }
      encoder.switchIgnoreUntilMs = millis() + 120;
      ui.dirty = true;
      markActivity();
      return;
    } else if (ui.selectedSetting == SET_NOTE_CC) {
      if (noteCcUiStage == NOTE_CC_UI_LIST) {
        if (noteCcCursor < NOTE_CC_SLOT_COUNT) {
          noteCcEditBackup = featureControls.noteCcMaps[noteCcCursor];
          noteCcSlotActionCursor = 0;
          noteCcUiStage = NOTE_CC_UI_SLOT_ACTION;
        } else if (noteCcCursor == NOTE_CC_SLOT_COUNT) {
          for (NoteCcMapEntry &entry : featureControls.noteCcMaps) entry = NoteCcMapEntry{};
          memset(noteCcToggleState, 0, sizeof(noteCcToggleState));
          saveStorageIfAuto();
        } else {
          ui.menuMode = MENU_SELECT;
          ui.deferredExitWork = true;
        }
      } else if (noteCcUiStage == NOTE_CC_UI_SLOT_ACTION) {
        if (noteCcSlotActionCursor == 0) {
          noteCcUiStage = NOTE_CC_UI_INPUT_CHANNEL;
          noteCcLearnActive = true;
        } else if (noteCcSlotActionCursor == 1) {
          featureControls.noteCcMaps[noteCcCursor] = NoteCcMapEntry{};
          noteCcToggleState[noteCcCursor] = false;
          noteCcUiStage = NOTE_CC_UI_LIST;
          noteCcSlotActionCursor = 0;
          saveStorageIfAuto();
        } else {
          cancelNoteCcEdit();
        }
      } else if (noteCcUiStage == NOTE_CC_UI_INPUT_CHANNEL) {
        if (cancelSelectedFor(SET_NOTE_CC)) cancelNoteCcEdit();
        else {
          noteCcUiStage = NOTE_CC_UI_INPUT_NOTE;
          noteCcLearnActive = true;
        }
      } else if (noteCcUiStage == NOTE_CC_UI_INPUT_NOTE) {
        if (cancelSelectedFor(SET_NOTE_CC)) cancelNoteCcEdit();
        else {
          noteCcLearnActive = false;
          noteCcUiStage = NOTE_CC_UI_OUTPUT_CHANNEL;
          noteCcLearnActive = true;
        }
      } else if (noteCcUiStage == NOTE_CC_UI_OUTPUT_CHANNEL) {
        if (cancelSelectedFor(SET_NOTE_CC)) cancelNoteCcEdit();
        else {
          noteCcUiStage = NOTE_CC_UI_OUTPUT_CC;
          noteCcLearnActive = true;
        }
      } else if (noteCcUiStage == NOTE_CC_UI_OUTPUT_CC) {
        if (cancelSelectedFor(SET_NOTE_CC)) cancelNoteCcEdit();
        else {
          noteCcLearnActive = false;
          noteCcUiStage = NOTE_CC_UI_BEHAVIOR;
        }
      } else {
        if (cancelSelectedFor(SET_NOTE_CC)) cancelNoteCcEdit();
        else {
          noteCcUiStage = NOTE_CC_UI_LIST;
          noteCcSlotActionCursor = 0;
          saveStorageIfAuto();
        }
      }
      encoder.switchIgnoreUntilMs = millis() + 120;
      ui.dirty = true;
      markActivity();
      return;
    } else if (ui.selectedSetting == SET_DIV_NOTES) {
      if (divNotesCursor == DIV_NOTE_RESET_SLOT) {
        clearDivNoteAssignments();
        divNotesCursor = 0;
        saveStorageIfAuto();
        encoder.switchIgnoreUntilMs = millis() + 120;
        ui.dirty = true;
        markActivity();
        return;
      }
      if (divNotesCursor == DIV_NOTE_BACK_SLOT) {
        divNotesCursor = 0;
        ui.menuMode = MENU_SELECT;
        ui.deferredExitWork = true;
        encoder.switchIgnoreUntilMs = millis() + 120;
        ui.dirty = true;
        markActivity();
        return;
      }
    } else if (ui.selectedSetting == SET_FOUR_BUTTON) {
      if (fourButtonUiStage == FOUR_BUTTON_UI_MAIN) {
        if (fourButtonUiCursor == 0) fourButtonUiStage = FOUR_BUTTON_UI_MODE;
        else if (fourButtonUiCursor == 1) {
          fourButtonUiStage = FOUR_BUTTON_UI_CUSTOM_LIST;
          fourButtonUiCursor = 0;
        } else if (fourButtonUiCursor == 2) {
          fourButtonUiStage = FOUR_BUTTON_UI_LOOPER;
          fourButtonUiCursor = 0;
        } else if (fourButtonUiCursor == 3) {
          fourButtonUiStage = FOUR_BUTTON_UI_CHORD;
          fourButtonUiCursor = 0;
        } else {
          ui.menuMode = MENU_SELECT;
          ui.deferredExitWork = true;
        }
      } else if (fourButtonUiStage == FOUR_BUTTON_UI_MODE) {
        fourButtonUiStage = FOUR_BUTTON_UI_MAIN;
        fourButtonUiCursor = 0;
        saveStorageIfAuto();
      } else if (fourButtonUiStage == FOUR_BUTTON_UI_CUSTOM_LIST) {
        if (fourButtonUiCursor < 4) {
          fourButtonEditButton = fourButtonUiCursor;
          fourButtonEditBackup = featureControls.customButtons[fourButtonEditButton];
          fourButtonUiStage = FOUR_BUTTON_UI_CUSTOM_CHANNEL;
        } else if (fourButtonUiCursor == FOUR_BUTTON_CUSTOM_DONE_SLOT) {
          featureControls.fourButtonMode = FOUR_BUTTON_CUSTOM;
          fourButtonUiStage = FOUR_BUTTON_UI_MAIN;
          fourButtonUiCursor = 0;
          saveStorageIfAuto();
        } else {
          fourButtonUiStage = FOUR_BUTTON_UI_MAIN;
          fourButtonUiCursor = 0;
        }
      } else if (fourButtonUiStage == FOUR_BUTTON_UI_CUSTOM_CHANNEL) {
        if (cancelSelectedFor(SET_FOUR_BUTTON)) cancelFourButtonCustomEdit();
        else fourButtonUiStage = FOUR_BUTTON_UI_CUSTOM_KIND;
      } else if (fourButtonUiStage == FOUR_BUTTON_UI_CUSTOM_KIND) {
        if (cancelSelectedFor(SET_FOUR_BUTTON)) cancelFourButtonCustomEdit();
        else {
          fourButtonUiStage = FOUR_BUTTON_UI_CUSTOM_NUMBER;
          fourButtonLearnActive = true;
        }
      } else if (fourButtonUiStage == FOUR_BUTTON_UI_CUSTOM_NUMBER) {
        if (cancelSelectedFor(SET_FOUR_BUTTON)) cancelFourButtonCustomEdit();
        else {
          fourButtonLearnActive = false;
          fourButtonUiStage = FOUR_BUTTON_UI_CUSTOM_BEHAVIOR;
        }
      } else if (fourButtonUiStage == FOUR_BUTTON_UI_CUSTOM_BEHAVIOR) {
        if (cancelSelectedFor(SET_FOUR_BUTTON)) cancelFourButtonCustomEdit();
        else {
          fourButtonUiStage = FOUR_BUTTON_UI_CUSTOM_LIST;
          fourButtonUiCursor = fourButtonEditButton;
          saveStorageIfAuto();
        }
      } else if (fourButtonUiStage == FOUR_BUTTON_UI_LOOPER) {
        if (fourButtonUiCursor < 6) {
          static constexpr uint8_t masks[6] = {
            LOOPER_BUTTON_SELECT, LOOPER_BUTTON_ARM, LOOPER_BUTTON_MUTE,
            LOOPER_BUTTON_SOLO, LOOPER_BUTTON_DELETE, LOOPER_BUTTON_UNDO
          };
          featureControls.looperButtonActions ^= masks[fourButtonUiCursor];
          saveStorageIfAuto();
        } else if (fourButtonUiCursor == FOUR_BUTTON_LOOPER_DONE_SLOT) {
          featureControls.fourButtonMode = FOUR_BUTTON_LOOPER;
          fourButtonUiStage = FOUR_BUTTON_UI_MAIN;
          fourButtonUiCursor = 0;
          saveStorageIfAuto();
        } else {
          fourButtonUiStage = FOUR_BUTTON_UI_MAIN;
          fourButtonUiCursor = 0;
        }
      } else {
        if (fourButtonUiCursor == 0) {
          chordLearnArmed = true;
          chordClearArmed = false;
        } else if (fourButtonUiCursor == 1) {
          chordClearArmed = true;
          chordLearnArmed = false;
        } else if (fourButtonUiCursor == FOUR_BUTTON_CHORD_DONE_SLOT) {
          featureControls.fourButtonMode = FOUR_BUTTON_CHORD_MEMORY;
          chordLearnArmed = false;
          chordClearArmed = false;
          fourButtonUiStage = FOUR_BUTTON_UI_MAIN;
          fourButtonUiCursor = 0;
          saveStorageIfAuto();
        } else {
          chordLearnArmed = false;
          chordClearArmed = false;
          fourButtonUiStage = FOUR_BUTTON_UI_MAIN;
          fourButtonUiCursor = 0;
        }
      }
      encoder.switchIgnoreUntilMs = millis() + 120;
      ui.dirty = true;
      markActivity();
      return;
    } else if (ui.selectedSetting == SET_MUTE_SOLO) {
      if (muteSoloCursor < arpnmidi3::kLoopTrackCount) {
        const uint32_t trackClickMs = millis();
        const bool secondClick = loopMixLastClickedTrack == muteSoloCursor &&
            (trackClickMs - loopMixTrackClickMs) <= LOOP_MIX_DOUBLE_CLICK_MS;
        loopMixLastClickedTrack = muteSoloCursor;
        loopMixTrackClickMs = trackClickMs;
        if (secondClick) {
          // The fast second click on a track leaves the screen. The action
          // already landed on the first click, and re-applying it here would
          // just toggle it straight back.
          ui.menuMode = MENU_SELECT;
          ui.deferredExitWork = true;
        } else {
          applyLoopMixModeToTrack(muteSoloCursor);
        }
      } else if (muteSoloCursor < LOOP_MIX_BACK_SLOT) {
        const uint8_t picked = muteSoloCursor - LOOP_MIX_MODE_BASE;
        const uint32_t clickMs = millis();
        const bool secondClick = picked == loopMixMode &&
            loopMixLastClickedMode == picked &&
            (clickMs - loopMixModeClickMs) <= LOOP_MIX_DOUBLE_CLICK_MS;
        loopMixLastClickedMode = picked;
        loopMixModeClickMs = clickMs;
        if (picked == loopMixMode &&
            (picked == LOOP_MIX_SOLO || picked == LOOP_MIX_MUTE)) {
          resetLoopMixMuteAndSolo();
        } else if (secondClick && picked == LOOP_MIX_CLEAR) {
          // Double-clicking Clear is the full wipe: all four tracks gone for
          // good, undo material included.
          multitrackLooper.clearAll(releaseMultitrackOutput, nullptr);
          releaseSilencedMultitrackOutputs();
          loopSafeClearArmed = false;
          markLoopStorageDirty();
        } else if (secondClick && picked == LOOP_MIX_ARM) {
          // Double-clicking Arm leaves the screen, the same as Back.
          ui.menuMode = MENU_SELECT;
          ui.deferredExitWork = true;
        } else {
          loopMixMode = picked;
        }
      } else {
        ui.menuMode = MENU_SELECT;
        ui.deferredExitWork = true;
      }
      refreshLoopUiState();
      encoder.switchIgnoreUntilMs = millis() + 120;
      ui.dirty = true;
      markActivity();
      return;
    } else if (ui.selectedSetting == SET_RND_RBN) {
      if (roundRobinMenuCursor < 16) {
        settings.roundRobinMask ^= channelBit(roundRobinMenuCursor + 1);
        saveStorageIfAuto();
        ui.dirty = true;
        markActivity();
        encoder.switchIgnoreUntilMs = millis() + 120;
        return;
      }
      if (roundRobinMenuCursor == RND_RBN_CH10_TO_1_SLOT) {
        setRoundRobinCh10To1(settings, !roundRobinCh10To1Enabled());
        saveStorageIfAuto();
        ui.dirty = true;
        markActivity();
        encoder.switchIgnoreUntilMs = millis() + 120;
        return;
      }
      if (roundRobinMenuCursor == RND_RBN_CH10_TO_2_SLOT) {
        setRoundRobinCh10To2(settings, !roundRobinCh10To2Enabled());
        saveStorageIfAuto();
        ui.dirty = true;
        markActivity();
        encoder.switchIgnoreUntilMs = millis() + 120;
        return;
      }
      if (roundRobinMenuCursor == RND_RBN_RANDOM_SLOT) {
        setRoundRobinRandom(settings, !roundRobinRandomEnabled());
        saveStorageIfAuto();
        ui.dirty = true;
        markActivity();
        encoder.switchIgnoreUntilMs = millis() + 120;
        return;
      }
      if (roundRobinMenuCursor == RND_RBN_CLEAR_SLOT) {
        settings.roundRobinMask = 0;
        setRoundRobinCh10To1(settings, false);
        setRoundRobinCh10To2(settings, false);
        setRoundRobinRandom(settings, false);
        saveStorageIfAuto();
        ui.dirty = true;
        markActivity();
        encoder.switchIgnoreUntilMs = millis() + 120;
        return;
      }
    } else if (ui.selectedSetting == SET_ROUTER) {
      if (routerEditStage == ROUTER_STAGE_LIST) {
        if (routerMenuCursor < 16) {
          routerEditChannel = routerMenuCursor;
          routerEditStage = ROUTER_STAGE_DEST;
          ui.dirty = true;
          markActivity();
          encoder.switchIgnoreUntilMs = millis() + 120;
          return;
        }
        if (routerMenuCursor == ROUTER_CLEAR_SLOT) {
          panicMidiOnly();
          clearRouterMappings(settings);
          for (uint8_t channel = 0; channel < 16; ++channel) {
            firmware3Settings.routerLowNotes[channel] = 0;
            firmware3Settings.routerHighNotes[channel] = 127;
          }
          saveStorageIfAuto();
          ui.dirty = true;
          markActivity();
          encoder.switchIgnoreUntilMs = millis() + 120;
          return;
        }
      } else if (routerEditStage == ROUTER_STAGE_DEST) {
        routerEditStage = ROUTER_STAGE_LOW_NOTE;
        ui.dirty = true;
        markActivity();
        encoder.switchIgnoreUntilMs = millis() + 120;
        return;
      } else if (routerEditStage == ROUTER_STAGE_LOW_NOTE) {
        routerEditStage = ROUTER_STAGE_HIGH_NOTE;
        ui.dirty = true;
        markActivity();
        encoder.switchIgnoreUntilMs = millis() + 120;
        return;
      } else if (routerEditStage == ROUTER_STAGE_HIGH_NOTE) {
        routerEditStage = ROUTER_STAGE_TRANSPOSE;
        ui.dirty = true;
        markActivity();
        encoder.switchIgnoreUntilMs = millis() + 120;
        return;
      } else {
        routerEditStage = ROUTER_STAGE_LIST;
        ui.dirty = true;
        markActivity();
        encoder.switchIgnoreUntilMs = millis() + 120;
        return;
      }
    }
    ui.menuMode = MENU_SELECT;
    ui.deferredLoadPreset = (ui.selectedSetting == SET_LOAD_PRESET);
    ui.deferredSaveOnly = false;
    ui.deferredExitWork = true;
  } else if (settingNeedsPanic(ui.selectedSetting)) {
    ui.hasPendingEdit = true;
    ui.pendingSetting = ui.selectedSetting;
    ui.pendingValue = getSettingValueRaw(ui.selectedSetting);
    ui.menuMode = MENU_EDIT;
  } else {
    if (ui.selectedSetting == SET_ARP_MODE) arpMenuUi = SubmenuUiState{};
    if (ui.selectedSetting == SET_LIVE_VELOCITY) liveVelocityUi = SubmenuUiState{};
    if (ui.selectedSetting == SET_LIVE_NOTE_LENGTH) liveNoteLengthUi = SubmenuUiState{};
    if (ui.selectedSetting == SET_STUTTER) stutterUi = SubmenuUiState{};
    if (ui.selectedSetting == SET_ECHO) echoUi = SubmenuUiState{};
    if (ui.selectedSetting == SET_QUICK_JUMP) quickJumpUi = SubmenuUiState{};
    if (ui.selectedSetting == SET_DRUM_MAGIC) drumMagicUi = SubmenuUiState{};
    if (ui.selectedSetting == SET_BASS_CH) bassUi = SubmenuUiState{};
    if (ui.selectedSetting == SET_LOOP_BARS) looperSettingsUi = SubmenuUiState{};
    if (ui.selectedSetting == SET_PARAMETER_LOCK) parameterLockUi = SubmenuUiState{};
    if (ui.selectedSetting == SET_CHORD) chordUi = SubmenuUiState{};
    if (ui.selectedSetting == SET_FORCE_SCALE) scaleUi = SubmenuUiState{};
    if (ui.selectedSetting == SET_GLOBAL) globalUi = SubmenuUiState{};
    if (ui.selectedSetting == SET_SCREEN_SAVER) {
      screenSaverCursor = clampU8(settings.screenSaver, 0, SCREEN_SAVER_CANCEL_SLOT - 1);
      screenSaverForceNow = false;
    }
    if (ui.selectedSetting == SET_LIVE_CC) {
      liveCcCursor = 0;
      liveCcEditing = false;
    }
    if (ui.selectedSetting == SET_ROUTER) routerEditStage = ROUTER_STAGE_LIST;
    if (ui.selectedSetting == SET_MAP_CC) {
      featuresUiStage = FEATURES_UI_GROUPS;
      featuresGroupCursor = 0;
      featuresLearnActive = false;
      featuresItemOpen = false;
    }
    if (ui.selectedSetting == SET_CC_MAP) {
      ccRemapUiStage = CC_REMAP_UI_LIST;
      ccRemapCursor = 0;
      ccRemapLearnActive = false;
    }
    if (ui.selectedSetting == SET_FOUR_BUTTON) {
      fourButtonUiStage = FOUR_BUTTON_UI_MAIN;
      fourButtonUiCursor = 0;
      fourButtonLearnActive = false;
    }
    if (ui.selectedSetting == SET_NOTE_CC) {
      noteCcUiStage = NOTE_CC_UI_LIST;
      noteCcCursor = 0;
      noteCcSlotActionCursor = 0;
      noteCcLearnActive = false;
    }
    if (ui.selectedSetting == SET_MUTE_SOLO) {
      muteSoloCursor = 0;
      loopMixMode = LOOP_MIX_ARM;
    }
    ui.menuMode = MENU_EDIT;
  }
  encoder.switchIgnoreUntilMs = millis() + 120;
  ui.dirty = true;
}

void encoderTurn(int delta, bool pressed) {
  if (wakeFromSaverIfNeeded()) return;
  if (ui.swallowWakeInput) {
    ui.swallowWakeInput = false;
    return;
  }
  markActivity();
  // Hold and turn changes the working track from either looper screen, in the
  // summary as well as inside the menu, so the track can be moved without
  // walking through a submenu first.
  if (pressed && (ui.selectedSetting == SET_LOOP_BARS ||
                  ui.selectedSetting == SET_MUTE_SOLO)) {
    selectLooperTrack(static_cast<uint8_t>(wrapIndex(
        multitrackLooper.selectedTrack() + delta, arpnmidi3::kLoopTrackCount)));
    return;
  }
  if (ui.menuMode == MENU_SELECT) {
    ui.selectedSetting = advanceSelectableSetting(ui.selectedSetting, delta);
    ui.dirty = true;
    return;
  }
  applySettingDelta(delta, pressed);
}

void pollEncoder() {
  const bool a = digitalRead(PIN_ENC_A);
  const bool b = digitalRead(PIN_ENC_B);
  const uint8_t ab = (a << 1) | b;
  const uint8_t last = encoder.lastAB;

  if (ab != last) {
    const int8_t delta = kEncoderTransitionTable[(last << 2) | ab];
    encoder.lastAB = ab;
    if (delta != 0) {
      encoder.turnWhilePressed |= encoder.switchDown;
      encoder.lastTurnMs = millis();
      encoder.switchIgnoreUntilMs = encoder.lastTurnMs + 120;
      encoder.stepAccum += delta * ENCODER_DIRECTION;
      if (encoder.stepAccum >= ENCODER_COUNTS_PER_INCREMENT) {
        encoder.stepAccum -= ENCODER_COUNTS_PER_INCREMENT;
        encoderTurn(1, encoder.switchDown);
      } else if (encoder.stepAccum <= -ENCODER_COUNTS_PER_INCREMENT) {
        encoder.stepAccum += ENCODER_COUNTS_PER_INCREMENT;
        encoderTurn(-1, encoder.switchDown);
      }
    }
  }

  const bool sw = digitalRead(PIN_ENC_SW);
  const uint32_t now = millis();
  if (sw != encoder.lastSwitch && now >= encoder.switchIgnoreUntilMs &&
      (now - encoder.switchChangeMs) > 12) {
    encoder.switchChangeMs = now;
    encoder.lastSwitch = sw;
    if (!sw) {
      encoder.switchDown = true;
      encoder.turnWhilePressed = false;
      encoder.pressStartMs = now;
    } else {
      const bool shortClick = encoder.switchDown && !encoder.turnWhilePressed &&
                              ((now - encoder.pressStartMs) < LONG_HOLD_PANIC_MS);
      encoder.switchDown = false;
      if (shortClick) {
        if (wakeFromSaverIfNeeded()) {
          ui.swallowWakeInput = false;
        } else if (ui.swallowWakeInput) {
          ui.swallowWakeInput = false;
        } else {
          activateClickAction();
        }
      }
    }
  }

  if (encoder.switchDown && !encoder.turnWhilePressed &&
      (now - encoder.pressStartMs) >= LONG_HOLD_PANIC_MS) {
    // The held gesture is a deliberate emergency reset, not just a silence:
    // it also gives the loop a clean slate, so a held panic never leaves an
    // old take sitting there. Unlike the eye/pad's stop-then-clear press,
    // this is one-directional on purpose: it only ever clears whatever is
    // still live, never undoes an already-cleared track, so there is no
    // ambiguity about which way a held panic goes. Clear first and panic
    // last: panic's all-channel sweep is the most exhaustive kill-everything
    // pass there is, so it has to run after, not before, or anything the
    // clear stirs up could slip past it and stick.
    clearAllLiveLoopTracks();
    markLoopStorageDirty();
    refreshLoopUiState();
    // Every mapped Feature Knob's live value goes back to what the preset
    // actually has saved too, so a controller sitting at a stale non-zero
    // position can't keep BPM, swing, velocity, or anything else parked
    // somewhere the preset never asked for.
    restoreFeatureKnobDefaults();
    panicAll();
    encoder.turnWhilePressed = true;
  }
}

bool loopLocksArpClock() {
  return multitrackLooper.recording() || multitrackLooper.recordingArmed() ||
         multitrackLooper.playing();
}

void handleDrumInputNote(uint8_t sourcePort, uint8_t note, uint8_t velocity, bool on) {
  markActivity(false);
  if (liveNoteViewActive() || ui.selectedSetting == SET_PANIC) ui.dirty = true;

  const bool hadNoDrumKeys = (heldDrumCount == 0);
  setDrumOwnerState(sourcePort, note, velocity, on && velocity > 0);
  rebuildHeldDrumCount();
  if (on && velocity > 0 && hadNoDrumKeys && arpHeldCount == 0 && !loopLocksArpClock()) {
    restartArpFromNewKeyPhrase();
  } else if (!on && heldDrumCount == 0 && arpHeldCount == 0 && !loopLocksArpClock()) {
    arpGateOffMs = 0;
    arpNextStepUs = 0;
  }
}

bool arpAnyPlaybackActive() {
  return arpHeldCount > 0 || heldDrumCount > 0 ||
         activeArpCount > 0 || activeDrumArpCount > 0 ||
         arpGateOffMs != 0 || arpNextStepUs != 0;
}

bool translateSplitInputToDrum(uint8_t &channel1, uint8_t &note) {
  if (channel1 != settings.inputChannel) return false;
  if (!arpChannelSplitMode()) return false;
  if (!splitDrumInputNote(note)) return false;

  // Channel zero is an internal-only marker; MIDI channels remain 1..16.
  channel1 = 0;
  note = splitDrumOutputNote(note);
  return true;
}

bool captureDivNoteAssignment(uint8_t channel1, uint8_t note, bool on) {
  if (ui.selectedSetting != SET_DIV_NOTES || ui.menuMode != MENU_EDIT) return false;
  if (!on) return true;
  if (divNotesCursor < DIV_NOTE_SLOT_COUNT) {
    settings.divNoteChannels[divNotesCursor] = channel1;
    settings.divNoteNotes[divNotesCursor] = note;
    featureControls.drumRollKinds[divNotesCursor] = TRIGGER_BINDING_NOTE;
  } else if (divNotesCursor == DIV_NOTE_PLUS_SLOT) {
    settings.divNotePlusNote = note;
  }
  saveStorageIfAuto();
  ui.dirty = true;
  markActivity(false);
  return true;
}

void clearDivNoteAssignments() {
  for (uint8_t i = 0; i < DIV_NOTE_SLOT_COUNT; ++i) {
    settings.divNoteChannels[i] = 0;
    settings.divNoteNotes[i] = 0xFF;
    featureControls.drumRollKinds[i] = TRIGGER_BINDING_OFF;
    divNoteHeld[i] = false;
    physicalDivNoteHeld[i] = false;
    loopDivNoteHeld[i] = false;
    divNoteHeldStamp[i] = 0;
  }
  settings.divNotePlusNote = 0xFF;
  divNotePressCounter = 0;
}

uint8_t handleDivNoteOverride(uint8_t sourcePort, uint8_t channel1, uint8_t &note,
                              uint8_t velocity, bool on) {
  for (uint8_t i = 0; i < DIV_NOTE_SLOT_COUNT; ++i) {
    if (featureControls.drumRollKinds[i] == TRIGGER_BINDING_NOTE &&
        settings.divNoteChannels[i] == channel1 && settings.divNoteNotes[i] == note) {
      recordLoopNote(sourcePort, channel1, note, velocity, on);
      if (loopOwnsInput(sourcePort)) loopDivNoteHeld[i] = on;
      else physicalDivNoteHeld[i] = on;
      divNoteHeld[i] = physicalDivNoteHeld[i] || loopDivNoteHeld[i];
      if (on) divNoteHeldStamp[i] = ++divNotePressCounter;
      syncArpDivisionToGrid();
      markActivity(false);
      if (settings.divNotePlusNote != 0xFF) {
        note = settings.divNotePlusNote;
        return 2;
      }
      return 1;
    }
  }
  return 0;
}

void releaseDuplicateInputNote(uint8_t sourcePort, uint8_t note) {
  clearSplitNoteFromMainPaths(sourcePort, note);
  thruLatchedNotes[note] = false;
}

void onInputNote(uint8_t sourcePort, uint8_t channel1, uint8_t note, uint8_t velocity, bool on,
                 bool recordForLoop) {
  const bool splitDrumInput = channel1 == 0;
  const bool channel10DrumInput = arpChannelSpecialMode() && !arpChannelSplitMode() && channel1 == 10;
  if (splitDrumInput || channel10DrumInput) {
    if (recordForLoop) {
      recordLoopNote(sourcePort, firmware3Settings.drumOutputChannel, note, velocity, on);
    }
    handleDrumInputNote(sourcePort, note, velocity, on);
    return;
  }

  if (recordForLoop) recordLoopNote(sourcePort, channel1, note, velocity, on);

  if (channel1 != settings.inputChannel) {
    // A note that reached noteThrough while this WAS the input channel still
    // owns a thru claim by the time its Note Off arrives here, if the
    // setting moved on in between, an arbitrary time later for one a clear
    // or undo force-releases rather than one ending on its own. Releasing it
    // here is a harmless no-op for a note noteThrough never claimed.
    if (!on) noteThrough(sourcePort, note, 0, false);
    if (channelEnabled(settings.legatoChannel) && channel1 == settings.legatoChannel) {
      handleLegatoInputNote(sourcePort, channel1, note, velocity, on);
    } else {
      sendFanout(sourcePort, (on ? 0x90 : 0x80) | ((channel1 - 1) & 0x0F), note, velocity);
    }
    return;
  }

  markActivity(false);
  if (liveNoteViewActive()) ui.dirty = true;

  // Latch and Freeze belong to the performer's hands. Loop playback re-enters
  // here on the main channel, and letting it participate stranded thru claims:
  // a loop note latched itself, or collided with a latched pitch, and then its
  // note-off was treated as sustain-held and never released the thru output.
  // Safe clear could not fix it because clear releases through this same path.
  const bool fromLoop = loopOwnsInput(sourcePort);

  if (on && velocity > 0) {
    const bool hadNoPhysicalInputNotes = !anyPhysicalInputNotesHeld();
    if (inputOwnerHeld(sourcePort, note)) releaseDuplicateInputNote(sourcePort, note);

    if (arpLatchEnabled() && !fromLoop) {
      if (arpLatchAwaitingNewPhrase && !anyPhysicalInputNotesHeld()) {
        clearArpLatchNotes();
      }
      arpLatchedNotes[note] = true;
      arpLatchedVelocities[note] = velocity;
      arpLatchAwaitingNewPhrase = false;
      if (arpLatchPlusEnabled() && !thruLatchedNotes[note]) {
        noteThrough(sourcePort, note, velocity, true);
        thruLatchedNotes[note] = true;
      }
    }
    setInputOwnerState(sourcePort, note, velocity, true);
    const bool startsFreshPhysicalPhrase = hadNoPhysicalInputNotes && !arpLatchEnabled() && !arpFreezeActive;
    if ((!arpHadKeys || startsFreshPhysicalPhrase) && heldDrumCount == 0 && !loopLocksArpClock()) {
      restartArpFromNewKeyPhrase();
    }
    arpHadKeys = true;
    if (!arpLatchPlusEnabled() || fromLoop) {
      noteThrough(sourcePort, note, velocity, true);
      if (arpLatchEnabled() && !fromLoop) thruLatchedNotes[note] = true;
    }
    noteArpOffPassthrough(sourcePort, note, velocity, true);
  } else {
    setInputOwnerState(sourcePort, note, 0, false);
    if (arpLatchEnabled() && !fromLoop && !anyPhysicalInputNotesHeld()) {
      arpLatchAwaitingNewPhrase = true;
    }
    // A loop-sourced note-off always releases its own thru and arp claims,
    // whatever the performer has latched or frozen at the same pitch.
    const bool sustainHeld = !fromLoop &&
        ((arpLatchEnabled() && arpLatchedNotes[note]) ||
         (arpFreezeActive && arpFrozenNotes[note]));
    if ((!arpLatchPlusEnabled() || fromLoop) && !sustainHeld) {
      noteThrough(sourcePort, note, 0, false);
    }
    if (!sustainHeld) noteArpOffPassthrough(sourcePort, note, 0, false);
  }

  rebuildHeldSorted();
  rebuildArpHeldSorted();
  if (arpHeldCount == 0) {
    arpHadKeys = false;
    for (uint8_t i = 0; i < activeArpCount; ++i) {
      const uint8_t outCh = activeArpChannels[i] ? activeArpChannels[i] : mainArpOutChannel();
      if (activeArpNotes[i] >= 0 && channelEnabled(outCh)) {
        sendFanout(255, 0x80 | ((outCh - 1) & 0x0F), activeArpNotes[i], 0);
      }
    }
    activeArpCount = 0;
    if (heldDrumCount == 0 && !loopLocksArpClock()) {
      arpGateOffMs = 0;
      arpNextStepUs = 0;
    }
  }
  updateBassVoice();
  finishChordMemoryLearnIfReady();
}

void handleDinNoteOn(byte channel, byte pitch, byte velocity) {
  routeIncomingChannelMessage(0, 0x90 | ((channel - 1) & 0x0F), pitch, velocity);
}

void handleDinNoteOff(byte channel, byte pitch, byte velocity) {
  routeIncomingChannelMessage(0, 0x80 | ((channel - 1) & 0x0F), pitch, velocity);
}

void recallParameterLocks(uint8_t sourcePort, uint8_t channel, uint8_t note) {
  for (uint16_t i = 0; i < parameterLockCount; ++i) {
    const ParameterLockEntry &entry = parameterLocks[i];
    if (entry.note == note) {
      sendFanout(sourcePort, 0xB0 | ((channel - 1) & 0x0F), entry.cc, entry.value);
    }
  }
}

void captureParameterLockCc(uint8_t channel, uint8_t cc, uint8_t value) {
  if (!channelEnabled(firmware3Settings.parameterLockChannel) ||
      channel != firmware3Settings.parameterLockChannel) return;
  bool changed = false;
  for (uint8_t note = 0; note < 128; ++note) {
    if (!parameterLockHeldNotes[note]) continue;
    bool replaced = false;
    for (uint16_t i = 0; i < parameterLockCount; ++i) {
      if (parameterLocks[i].note == note && parameterLocks[i].cc == cc) {
        if (parameterLocks[i].value != value) {
          parameterLocks[i].value = value;
          changed = true;
        }
        replaced = true;
        break;
      }
    }
    if (replaced) continue;
    if (parameterLockCount >= MAX_PARAMETER_LOCKS) {
      ++parameterLockOverflowCount;
      continue;
    }
    parameterLocks[parameterLockCount++] = ParameterLockEntry{note, cc, value};
    changed = true;
  }
  if (changed) saveStorageIfAuto();
}

uint8_t featureBipolarPercent(uint8_t value, bool allowZero) {
  uint16_t result;
  if (value <= 64) result = (static_cast<uint16_t>(value) * 100U + 32U) / 64U;
  else result = 100U + (static_cast<uint16_t>(value - 64U) * 100U + 31U) / 63U;
  if (!allowZero && result == 0) result = 1;
  return static_cast<uint8_t>(min<uint16_t>(200, result));
}

uint8_t featureTargetFromBlock(uint8_t id, uint8_t base) {
  return id >= base && id < base + LIVE_TARGET_COUNT ? id - base : 0xFF;
}

void requestStutterState(uint8_t target, bool enabled, int16_t lengthSelection) {
  if (target >= STUTTER_ECHO_TARGET_COUNT) return;
  if (stutterRepeaters[target].active()) deactivateStutter(target);
  if (lengthSelection >= 0) {
    // Enforced here too, not just in the UI's own local-index translation,
    // so a mapped knob or button can't reach past Stutter's 1-bar ceiling
    // either.
    firmware3Settings.liveTargets[target].stutterLengthSelection = clampU8(
        lengthSelection, STUTTER_LENGTH_MIN_SELECTION, STUTTER_LENGTH_COUNT - 1);
  }
  firmware3Settings.liveTargets[target].stutterEnabled = enabled ? 1 : 0;
  stutterTimedOut[target] = false;
  stutterSettingWasEnabled[target] = false;
  ui.dirty = true;
}

void applyFeatureKnob(uint8_t id, uint8_t value) {
  uint8_t target = featureTargetFromBlock(id, FEATURE_KNOB_VELOCITY_BASE);
  if (target < LIVE_TARGET_COUNT) {
    LiveTargetSettings &live = firmware3Settings.liveTargets[target];
    live.velocityPercent = featureBipolarPercent(value, true);
    live.velocityEnabled = 1;
    ui.dirty = true;
    return;
  }
  target = featureTargetFromBlock(id, FEATURE_KNOB_NOTE_LENGTH_BASE);
  if (target < LIVE_TARGET_COUNT) {
    LiveTargetSettings &live = firmware3Settings.liveTargets[target];
    live.noteLengthPercent = featureBipolarPercent(value, false);
    live.noteLengthEnabled = 1;
    ui.dirty = true;
    return;
  }
  target = featureTargetFromBlock(id, FEATURE_KNOB_STUTTER_BASE);
  if (target < LIVE_TARGET_COUNT) {
    if (value == 0) {
      requestStutterState(target, false);
    } else {
      const uint8_t lengthSelection = static_cast<uint8_t>(
          (static_cast<uint16_t>(value - 1U) * (STUTTER_LENGTH_COUNT - 1U) + 63U) /
          126U);
      requestStutterState(target, true, lengthSelection);
    }
    return;
  }
  target = featureTargetFromBlock(id, FEATURE_KNOB_ECHO_WET_BASE);
  if (target < LIVE_TARGET_COUNT) {
    firmware3Settings.liveTargets[target].echoWet =
        static_cast<uint8_t>((static_cast<uint16_t>(value) * 100U + 63U) / 127U);
    firmware3Settings.liveTargets[target].echoEnabled = 1;
    ui.dirty = true;
    return;
  }
  target = featureTargetFromBlock(id, FEATURE_KNOB_ECHO_LENGTH_BASE);
  if (target < LIVE_TARGET_COUNT) {
    firmware3Settings.liveTargets[target].echoLength =
        static_cast<uint8_t>((static_cast<uint16_t>(value) * (STUTTER_LENGTH_COUNT - 1U) + 63U) / 127U);
    ui.dirty = true;
    return;
  }
  target = featureTargetFromBlock(id, FEATURE_KNOB_ECHO_DELAY_BASE);
  if (target < LIVE_TARGET_COUNT) {
    firmware3Settings.liveTargets[target].echoDelay =
        static_cast<uint8_t>((static_cast<uint16_t>(value) * (STUTTER_LENGTH_COUNT - 1U) + 63U) / 127U);
    ui.dirty = true;
    return;
  }
  target = featureTargetFromBlock(id, FEATURE_KNOB_ECHO_DRIFT_BASE);
  if (target < LIVE_TARGET_COUNT) {
    firmware3Settings.liveTargets[target].echoDrift = value <= 64
        ? -static_cast<int8_t>((static_cast<uint16_t>(64U - value) * 16U + 32U) / 64U)
        : static_cast<int8_t>((static_cast<uint16_t>(value - 64U) * 16U + 31U) / 63U);
    ui.dirty = true;
    return;
  }
  if (id == FEATURE_KNOB_ARP_DIVISION) {
    const uint8_t next = static_cast<uint8_t>(
        (static_cast<uint16_t>(value) * ARP_DIVISION_FOLLOW_DRUM + 63U) / 127U);
    setSettingValueRaw(SET_DIVISION, next);
    syncArpDivisionToGrid();
  } else if (id == FEATURE_KNOB_DRUM_DIVISION) {
    firmware3Settings.drumDivision = static_cast<uint8_t>(
        (static_cast<uint16_t>(value) * DRUM_DIVISION_FREE + 63U) / 127U);
    if (firmware3Settings.drumDivision == DRUM_DIVISION_FOLLOW_ARP &&
        settings.division == ARP_DIVISION_FOLLOW_DRUM) settings.division = DIV_1_16;
    syncArpDivisionToGrid();
  } else if (id == FEATURE_KNOB_QUICK_JUMP_INPUT) {
    firmware3Settings.quickJumpInputChannel = 1U +
        static_cast<uint8_t>((static_cast<uint16_t>(value) * 15U + 63U) / 127U);
  } else if (id == FEATURE_KNOB_QUICK_JUMP_OUTPUT) {
    firmware3Settings.quickJumpOutputChannel = 1U +
        static_cast<uint8_t>((static_cast<uint16_t>(value) * 15U + 63U) / 127U);
  } else if (id == FEATURE_KNOB_BPM) {
    settings.manualBpm = 20U + static_cast<uint16_t>(
        (static_cast<uint32_t>(value) * 280U + 63U) / 127U);
    syncMusicalClockConfig(false);
  } else if (id == FEATURE_KNOB_SWING) {
    firmware3Settings.swing = static_cast<uint8_t>(
        (static_cast<uint16_t>(value) * 75U + 63U) / 127U);
  } else if (id == FEATURE_KNOB_ARP_MODE) {
    settings.arpMode = static_cast<uint8_t>(
        (static_cast<uint16_t>(value) * (ARP_SELECTION_COUNT - 1U) + 63U) / 127U);
    restartArpTiming(true);
  } else if (id == FEATURE_KNOB_ARP_VELOCITY) {
    settings.arpVelocity = max<uint8_t>(1, value);
  } else if (id == FEATURE_KNOB_ARP_LENGTH) {
    settings.arpLengthPct = 1U + static_cast<uint8_t>(
        (static_cast<uint16_t>(value) * 99U + 63U) / 127U);
  } else if (id == FEATURE_KNOB_ARP_OCTAVES) {
    firmware3Settings.arpOctaves = 1U + static_cast<uint8_t>(
        (static_cast<uint16_t>(value) * 3U + 63U) / 127U);
    restartArpTiming(true);
  } else if (id == FEATURE_KNOB_LOOP_TRACK) {
    selectLooperTrack(static_cast<uint8_t>(
        (static_cast<uint16_t>(value) * (arpnmidi3::kLoopTrackCount - 1U) + 63U) / 127U));
  } else if (id == FEATURE_KNOB_LOOP_LENGTH) {
    const uint8_t track = multitrackLooper.selectedTrack();
    const uint8_t maximum = track == 0 ? 6 : 5;
    setLoopTrackLengthSelection(track, static_cast<uint8_t>(
        (static_cast<uint16_t>(value) * maximum + 63U) / 127U));
  }
  ui.dirty = true;
}

void featureLoopClearUndo() {
  const uint8_t selected = multitrackLooper.selectedTrack();
  if (multitrackLooper.trackMode() == arpnmidi3::LoopTrackMode::Layers) {
    clearOrUndoAllLoopTracks();
  } else if (multitrackLooper.track(selected).hidden) {
    undoLoopTrackClear(selected);
  } else {
    multitrackLooper.safeClear(selected, releaseMultitrackOutput, nullptr);
  }
  markLoopStorageDirty();
  refreshLoopUiState();
  ui.dirty = true;
}

void triggerFeatureButton(uint8_t id, bool pressed) {
  uint8_t target = featureTargetFromBlock(id, FEATURE_BUTTON_VELOCITY_BASE);
  if (target < LIVE_TARGET_COUNT) {
    if (pressed) firmware3Settings.liveTargets[target].velocityEnabled ^= 1U;
    ui.dirty = true;
    return;
  }
  target = featureTargetFromBlock(id, FEATURE_BUTTON_NOTE_LENGTH_BASE);
  if (target < LIVE_TARGET_COUNT) {
    if (pressed) firmware3Settings.liveTargets[target].noteLengthEnabled ^= 1U;
    ui.dirty = true;
    return;
  }
  target = featureTargetFromBlock(id, FEATURE_BUTTON_STUTTER_BASE);
  if (target < LIVE_TARGET_COUNT) {
    requestStutterState(target, pressed);
    return;
  }
  target = featureTargetFromBlock(id, FEATURE_BUTTON_ECHO_BASE);
  if (target < LIVE_TARGET_COUNT) {
    if (pressed) firmware3Settings.liveTargets[target].echoEnabled ^= 1U;
    ui.dirty = true;
    return;
  }
  if (id >= FEATURE_BUTTON_STUTTER_DIV_BASE && id < FEATURE_BUTTON_STUTTER_DIV_END) {
    static constexpr uint8_t divisions[STUTTER_BUTTON_DIVISION_COUNT] = {
      DIV_1_2, DIV_1_4, DIV_1_8, DIV_1_16, DIV_1_32, DIV_1_64
    };
    const uint8_t offset = id - FEATURE_BUTTON_STUTTER_DIV_BASE;
    target = offset / STUTTER_BUTTON_DIVISION_COUNT;
    const uint8_t division = divisions[offset % STUTTER_BUTTON_DIVISION_COUNT];
    requestStutterState(target, pressed,
        pressed ? lengthSelectionForDivision(division) : -1);
    return;
  }
  if (!pressed) return;
  if (id == FEATURE_BUTTON_LOOP_RECORD) {
    if (multitrackLooper.recording() || multitrackLooper.recordingArmed()) {
      finishActiveMultitrackRecording(time_us_64(), false);
    } else {
      const uint8_t track = multitrackLooper.selectedTrack();
      armSelectedMultitrack(loopTrackHasContent(track));
    }
  } else if (id == FEATURE_BUTTON_LOOP_PLAY_STOP) {
    if (multitrackLooper.playing()) multitrackLooper.stop(releaseMultitrackOutput, nullptr);
    else multitrackLooper.start(time_us_64());
  } else if (id == FEATURE_BUTTON_LOOP_CLEAR_UNDO) {
    featureLoopClearUndo();
  } else if (id == FEATURE_BUTTON_LOOP_COMBO) {
    handleMultitrackRecPlay();
  } else if (id >= FEATURE_BUTTON_TRACK_SELECT_BASE &&
             id < FEATURE_BUTTON_TRACK_SELECT_BASE + arpnmidi3::kLoopTrackCount) {
    selectLooperTrack(id - FEATURE_BUTTON_TRACK_SELECT_BASE);
  } else if (id >= FEATURE_BUTTON_TRACK_MUTE_BASE &&
             id < FEATURE_BUTTON_TRACK_MUTE_BASE + arpnmidi3::kLoopTrackCount) {
    const uint8_t track = id - FEATURE_BUTTON_TRACK_MUTE_BASE;
    setLoopTrackMuted(track, !multitrackLooper.track(track).muted);
    markLoopStorageDirty();
  } else if (id >= FEATURE_BUTTON_TRACK_SOLO_BASE &&
             id < FEATURE_BUTTON_TRACK_SOLO_BASE + arpnmidi3::kLoopTrackCount) {
    const uint8_t selected = id - FEATURE_BUTTON_TRACK_SOLO_BASE;
    setExclusiveLoopSolo(selected, !multitrackLooper.track(selected).solo);
    markLoopStorageDirty();
  } else if (id == FEATURE_BUTTON_QUICK_JUMP) {
    setQuickJumpEnabled(firmware3Settings.quickJumpEnabled == 0);
  } else if (id == FEATURE_BUTTON_ARP_RETRIGGER) {
    firmware3Settings.arpRetriggerSync ^= 1U;
  } else if (id == FEATURE_BUTTON_ARP_NOTE_ORDER) {
    firmware3Settings.arpNoteOrder ^= 1U;
    rebuildArpHeldSorted();
  } else if (id == FEATURE_BUTTON_DRUM_MAGIC) {
    firmware3Settings.drumEnabled ^= 1U;
  } else if (id == FEATURE_BUTTON_DRUM_AFTERTOUCH_VELOCITY) {
    firmware3Settings.drumAftertouchVelocity ^= 1U;
  } else if (id == FEATURE_BUTTON_CHORD) {
    firmware3Settings.chordEnabled ^= 1U;
  } else if (id == FEATURE_BUTTON_LOOP_AUTO_REC) {
    firmware3Settings.looperAutoRec ^= 1U;
  } else if (id == FEATURE_BUTTON_LOOP_TIME_TRAVEL) {
    firmware3Settings.looperTimeTravel ^= 1U;
  } else if (id == FEATURE_BUTTON_LOOP_RECORD_CC) {
    firmware3Settings.looperRecordCc ^= 1U;
  } else if (id == FEATURE_BUTTON_LOOP_MIDI_TRANSPORT) {
    firmware3Settings.looperMidiTransport ^= 1U;
  } else if (id == FEATURE_BUTTON_CLOCK_INPUT) {
    firmware3Settings.clockInFollow ^= 1U;
    syncMusicalClockConfig(true);
  } else if (id == FEATURE_BUTTON_CLOCK_OUTPUT) {
    firmware3Settings.clockOutSend ^= 1U;
    syncMusicalClockConfig(false);
  } else if (id == FEATURE_BUTTON_PANIC) {
    panicAll();
  }
  refreshLoopUiState();
  ui.dirty = true;
}

bool processFeatureCc(uint8_t channel, uint8_t cc, uint8_t value) {
  bool matched = false;
  for (uint8_t id = 0; id < FEATURE_KNOB_COUNT; ++id) {
    const FeatureKnobBinding &binding = featureControls.knobs[id];
    if (binding.channel == channel && binding.cc == cc) {
      applyFeatureKnob(id, value);
      matched = true;
    }
  }
  for (uint8_t id = 0; id < FEATURE_BUTTON_COUNT; ++id) {
    const FeatureButtonBinding &binding = featureControls.buttons[id];
    if (binding.kind != TRIGGER_BINDING_CC || binding.channel != channel ||
        binding.number != cc) continue;
    const bool pressed = value >= 64;
    if (featureButtonCcHeld[id] != pressed) {
      featureButtonCcHeld[id] = pressed;
      triggerFeatureButton(id, pressed);
    }
    matched = true;
  }
  if (matched) saveStorageIfAuto();
  return matched;
}

bool processDrumRollCc(uint8_t sourcePort, uint8_t channel, uint8_t cc,
                       uint8_t value) {
  bool matched = false;
  bool recordedControl = false;
  const bool pressed = value >= 64;
  for (uint8_t slot = 0; slot < DIV_NOTE_SLOT_COUNT; ++slot) {
    if (featureControls.drumRollKinds[slot] != TRIGGER_BINDING_CC ||
        settings.divNoteChannels[slot] != channel ||
        settings.divNoteNotes[slot] != cc) continue;
    if (!recordedControl && !loopOwnsInput(sourcePort) &&
        !firmware3Settings.looperRecordCc && multitrackLooper.recording()) {
      const arpnmidi3::LoopMidiEvent event{0,
          static_cast<uint8_t>(0xB0 | ((channel - 1U) & 0x0F)), cc, value};
      if (multitrackLooper.capture(time_us_64(), event)) markLoopStorageDirty();
      recordedControl = true;
    }
    if (loopOwnsInput(sourcePort)) loopDivNoteHeld[slot] = pressed;
    else physicalDivNoteHeld[slot] = pressed;
    divNoteHeld[slot] = physicalDivNoteHeld[slot] || loopDivNoteHeld[slot];
    if (pressed) divNoteHeldStamp[slot] = ++divNotePressCounter;
    syncArpDivisionToGrid();
    matched = true;
  }
  if (matched) {
    markActivity(false);
    // Playing roll notes is performance, not editing. Settings screens show
    // stored settings, and no stored setting changes here, so none of them
    // redraw. The live note views are the one place whose whole purpose is
    // showing keys as they happen.
    if (liveNoteViewActive()) ui.dirty = true;
  }
  return matched;
}

bool captureDrumRollCcAssignment(uint8_t channel, uint8_t cc) {
  if (ui.selectedSetting != SET_DIV_NOTES || ui.menuMode != MENU_EDIT ||
      divNotesCursor >= DIV_NOTE_SLOT_COUNT) return false;
  settings.divNoteChannels[divNotesCursor] = channel;
  settings.divNoteNotes[divNotesCursor] = cc;
  featureControls.drumRollKinds[divNotesCursor] = TRIGGER_BINDING_CC;
  saveStorageIfAuto();
  markActivity(false);
  ui.dirty = true;
  return true;
}

bool captureFeatureCcAssignment(uint8_t channel, uint8_t cc) {
  if (ui.selectedSetting != SET_MAP_CC || ui.menuMode != MENU_EDIT ||
      !featuresLearnActive) return false;
  if (featuresUiStage == FEATURES_UI_KNOBS && featuresItemCursor < FEATURE_KNOB_COUNT) {
    featureControls.knobs[featuresItemCursor] = FeatureKnobBinding{channel, cc};
  } else if (featuresUiStage == FEATURES_UI_BUTTONS &&
             featuresItemCursor < FEATURE_BUTTON_COUNT) {
    featureControls.buttons[featuresItemCursor] =
        FeatureButtonBinding{channel, cc, TRIGGER_BINDING_CC};
  } else {
    return false;
  }
  featuresLearnActive = false;
  markActivity(false);
  ui.dirty = true;
  return true;
}

bool captureFourButtonCcAssignment(uint8_t channel, uint8_t cc) {
  if (ui.selectedSetting != SET_FOUR_BUTTON || ui.menuMode != MENU_EDIT ||
      fourButtonUiStage != FOUR_BUTTON_UI_CUSTOM_NUMBER || !fourButtonLearnActive ||
      fourButtonEditButton >= 4) return false;
  CustomButtonConfig &button = featureControls.customButtons[fourButtonEditButton];
  button.channel = channel;
  button.number = cc;
  button.kind = TRIGGER_BINDING_CC;
  fourButtonLearnActive = false;
  ui.dirty = true;
  markActivity(false);
  return true;
}

bool captureNoteCcOutputAssignment(uint8_t channel, uint8_t cc) {
  if (ui.selectedSetting != SET_NOTE_CC || ui.menuMode != MENU_EDIT ||
      (noteCcUiStage != NOTE_CC_UI_OUTPUT_CHANNEL &&
       noteCcUiStage != NOTE_CC_UI_OUTPUT_CC) || !noteCcLearnActive ||
      noteCcCursor >= NOTE_CC_SLOT_COUNT) return false;
  NoteCcMapEntry &entry = featureControls.noteCcMaps[noteCcCursor];
  entry.outputChannel = channel;
  entry.outputCc = cc;
  noteCcLearnActive = false;
  noteCcUiStage = NOTE_CC_UI_OUTPUT_CC;
  ui.dirty = true;
  markActivity(false);
  return true;
}

bool captureFeatureNoteAssignment(uint8_t channel, uint8_t note, bool pressed) {
  if (!pressed || ui.selectedSetting != SET_MAP_CC || ui.menuMode != MENU_EDIT ||
      !featuresLearnActive || featuresUiStage != FEATURES_UI_BUTTONS ||
      featuresItemCursor >= FEATURE_BUTTON_COUNT) return false;
  featureControls.buttons[featuresItemCursor] =
      FeatureButtonBinding{channel, note, TRIGGER_BINDING_NOTE};
  featuresLearnActive = false;
  markActivity(false);
  ui.dirty = true;
  return true;
}

bool captureFourButtonNoteAssignment(uint8_t channel, uint8_t note, bool pressed) {
  if (!pressed || ui.selectedSetting != SET_FOUR_BUTTON || ui.menuMode != MENU_EDIT ||
      fourButtonUiStage != FOUR_BUTTON_UI_CUSTOM_NUMBER || !fourButtonLearnActive ||
      fourButtonEditButton >= 4) return false;
  CustomButtonConfig &button = featureControls.customButtons[fourButtonEditButton];
  button.channel = channel;
  button.number = note;
  button.kind = TRIGGER_BINDING_NOTE;
  fourButtonLearnActive = false;
  ui.dirty = true;
  markActivity(false);
  return true;
}

bool captureNoteCcInputAssignment(uint8_t channel, uint8_t note, bool pressed) {
  if (!pressed || ui.selectedSetting != SET_NOTE_CC || ui.menuMode != MENU_EDIT ||
      (noteCcUiStage != NOTE_CC_UI_INPUT_CHANNEL &&
       noteCcUiStage != NOTE_CC_UI_INPUT_NOTE) || !noteCcLearnActive ||
      noteCcCursor >= NOTE_CC_SLOT_COUNT) return false;
  NoteCcMapEntry &entry = featureControls.noteCcMaps[noteCcCursor];
  entry.inputChannel = channel;
  entry.inputNote = note;
  noteCcLearnActive = false;
  noteCcUiStage = NOTE_CC_UI_INPUT_NOTE;
  ui.dirty = true;
  markActivity(false);
  return true;
}

bool captureCcRemapInput(uint8_t channel, uint8_t cc) {
  if (ui.selectedSetting != SET_CC_MAP || ui.menuMode != MENU_EDIT ||
      ccRemapUiStage != CC_REMAP_UI_INPUT || !ccRemapLearnActive ||
      ccRemapCursor >= CC_REMAP_SLOT_COUNT || channel != settings.inputChannel) return false;
  featureControls.ccRemaps[ccRemapCursor].inputCc = cc;
  ccRemapLearnActive = false;
  markActivity(false);
  ui.dirty = true;
  return true;
}

bool processFeatureNote(uint8_t channel, uint8_t note, bool pressed) {
  bool matched = false;
  for (uint8_t id = 0; id < FEATURE_BUTTON_COUNT; ++id) {
    const FeatureButtonBinding &binding = featureControls.buttons[id];
    if (binding.kind == TRIGGER_BINDING_NOTE && binding.channel == channel &&
        binding.number == note) {
      triggerFeatureButton(id, pressed);
      matched = true;
    }
  }
  if (matched) saveStorageIfAuto();
  return matched;
}

bool processNoteCcMap(uint8_t sourcePort, uint8_t channel, uint8_t note,
                      bool pressed) {
  bool matched = false;
  for (uint8_t slot = 0; slot < NOTE_CC_SLOT_COUNT; ++slot) {
    const NoteCcMapEntry &entry = featureControls.noteCcMaps[slot];
    if (entry.inputChannel != channel || entry.inputNote != note) continue;
    if (entry.behavior == NOTE_CC_TOGGLE) {
      if (pressed) {
        noteCcToggleState[slot] = !noteCcToggleState[slot];
        sendFanout(sourcePort, static_cast<uint8_t>(0xB0 | (entry.outputChannel - 1U)),
                   entry.outputCc, noteCcToggleState[slot] ? 127 : 0);
      }
    } else {
      sendFanout(sourcePort, static_cast<uint8_t>(0xB0 | (entry.outputChannel - 1U)),
                 entry.outputCc, pressed ? 127 : 0);
    }
    matched = true;
  }
  return matched;
}

bool applyCcRemap(uint8_t sourcePort, uint8_t channel, uint8_t cc, uint8_t value) {
  if (channel != settings.inputChannel) return false;
  bool matched = false;
  for (const CcRemapEntry &entry : featureControls.ccRemaps) {
    if (entry.inputCc != cc) continue;
    sendFanout(sourcePort, static_cast<uint8_t>(0xB0 | (entry.outputChannel - 1U)),
               entry.outputCc, value);
    matched = true;
  }
  return matched;
}

void routeControlChange(uint8_t sourcePort, byte channel, byte control, byte value) {
  if (captureNoteCcOutputAssignment(channel, control)) return;
  if (captureFourButtonCcAssignment(channel, control)) return;
  if (captureFeatureCcAssignment(channel, control)) return;
  if (captureCcRemapInput(channel, control)) return;
  if (captureDrumRollCcAssignment(channel, control)) return;
  recordLoopCc(sourcePort, channel, control, value);
  captureParameterLockCc(channel, control, value);
  if (processDrumRollCc(sourcePort, channel, control, value)) return;
  if (processFeatureCc(channel, control, value)) return;
  if (applyCcRemap(sourcePort, channel, control, value)) return;
  if (control != 0 && control != 32) markActivity(false);
  if (channel != settings.inputChannel) {
    sendFanout(sourcePort, 0xB0 | ((channel - 1) & 0x0F), control, value);
    return;
  }
  forEachCcOutput([&](uint8_t outCh) {
    sendFanout(sourcePort, 0xB0 | ((outCh - 1) & 0x0F), control, value);
  });
}

void handleDinCc(byte channel, byte control, byte value) {
  routeIncomingChannelMessage(0, 0xB0 | ((channel - 1) & 0x0F), control, value);
}

void routePitchBend(uint8_t sourcePort, byte channel, int bend) {
  if (channel != settings.inputChannel) {
    int bend14 = bend + 8192;
    sendFanout(sourcePort, 0xE0 | ((channel - 1) & 0x0F), bend14 & 0x7F, (bend14 >> 7) & 0x7F);
    return;
  }
  forEachCcOutput([&](uint8_t outCh) {
    int bend14 = bend + 8192;
    sendFanout(sourcePort, 0xE0 | ((outCh - 1) & 0x0F), bend14 & 0x7F, (bend14 >> 7) & 0x7F);
  });
}

void routeProgramLikeMessage(uint8_t sourcePort, uint8_t status, uint8_t data1, uint8_t data2) {
  const uint8_t channel = (status & 0x0F) + 1;
  if (channel != settings.inputChannel) {
    sendFanout(sourcePort, status, data1, data2);
    return;
  }
  forEachCcOutput([&](uint8_t outCh) {
    sendFanout(sourcePort, (status & 0xF0) | ((outCh - 1) & 0x0F), data1, data2);
  });
}

void routeChannelAftertouch(uint8_t sourcePort, uint8_t channel, uint8_t pressure) {
  if (channel == 10) drumAftertouchPressure = pressure;
  if (channel == settings.inputChannel && firmware3Settings.mainAftertouchArpVelocity) {
    mainAftertouchPressure = pressure;
  }
  if (firmware3Settings.channelAftertouchCc <= 127) {
    if (channel == settings.inputChannel) {
      forEachCcOutput([&](uint8_t outCh) {
        sendFanout(sourcePort, 0xB0 | ((outCh - 1) & 0x0F),
                   firmware3Settings.channelAftertouchCc, pressure);
      });
    } else {
      sendFanout(sourcePort, 0xB0 | ((channel - 1) & 0x0F),
                 firmware3Settings.channelAftertouchCc, pressure);
    }
  }
  if (firmware3Settings.forwardChannelAftertouch) {
    routeProgramLikeMessage(sourcePort, 0xD0 | ((channel - 1) & 0x0F), pressure, 0);
  }
}

void handleDinPb(byte channel, int bend) {
  const int bend14 = bend + 8192;
  routeIncomingChannelMessage(0, 0xE0 | ((channel - 1) & 0x0F), bend14 & 0x7F,
                              (bend14 >> 7) & 0x7F);
}

void handleDinProgramChange(byte channel, byte number) {
  routeIncomingChannelMessage(0, 0xC0 | ((channel - 1) & 0x0F), number, 0);
}

void handleDinAfterTouchChannel(byte channel, byte pressure) {
  routeIncomingChannelMessage(0, 0xD0 | ((channel - 1) & 0x0F), pressure, 0);
}

void handleDinClock() {
  handleClockByte(true);
}

void handleDinStart() {
  handleRealtimeByte(0, 0xFA);
}

void handleDinContinue() {
  handleRealtimeByte(0, 0xFB);
}

void handleDinStop() {
  handleRealtimeByte(0, 0xFC);
}

void handleSongSelect(byte song) {
  if (!firmware3Settings.looperMidiTransport || song >= arpnmidi3::kLoopTrackCount) return;
  selectLooperTrack(song);
  refreshLoopUiState();
  ui.dirty = true;
}

void finishMidiTransportRecording() {
  finishActiveMultitrackRecording(time_us_64(), true);
}

void selectAdjacentLooperTrack(int8_t direction) {
  const int current = multitrackLooper.selectedTrack();
  selectLooperTrack(static_cast<uint8_t>((current + direction +
      arpnmidi3::kLoopTrackCount) % arpnmidi3::kLoopTrackCount));
}

void handleMmcCommand(uint8_t command) {
  if (!firmware3Settings.looperMidiTransport) return;
  const uint64_t nowUs = time_us_64();
  switch (command) {
    case 0x01:  // Stop
      finishMidiTransportRecording();
      multitrackLooper.pause(nowUs, releaseMultitrackOutput, nullptr);
      break;
    case 0x02:  // Play
    case 0x03:  // Deferred Play
      multitrackLooper.resume(nowUs);
      break;
    case 0x04:  // Fast Forward: next working track
      selectAdjacentLooperTrack(1);
      break;
    case 0x05:  // Rewind: previous working track
      selectAdjacentLooperTrack(-1);
      break;
    case 0x06: {  // Record Strobe
      if (!multitrackLooper.recording() && !multitrackLooper.recordingArmed()) {
        const uint8_t track = multitrackLooper.selectedTrack();
        armSelectedMultitrack(loopTrackHasContent(track));
      }
      break;
    }
    case 0x07:  // Record Exit
      finishMidiTransportRecording();
      break;
    case 0x08:  // Record Pause
      finishMidiTransportRecording();
      multitrackLooper.pause(nowUs, releaseMultitrackOutput, nullptr);
      break;
    case 0x09:  // Pause
      multitrackLooper.pause(nowUs, releaseMultitrackOutput, nullptr);
      break;
    case 0x0D:  // MMC Reset: stop safely and return to track 1
      finishMidiTransportRecording();
      multitrackLooper.stop(releaseMultitrackOutput, nullptr);
      selectLooperTrack(0);
      break;
    default:
      return;
  }
  refreshLoopUiState();
  ui.dirty = true;
}

void parseTransportSysex(const uint8_t *data, uint8_t length) {
  if (!data || length < 6) return;
  uint8_t start = 0;
  while (start < length && data[start] != 0xF0) ++start;
  if (start + 5 >= length || data[start + 1] != 0x7F || data[start + 3] != 0x06) return;
  handleMmcCommand(data[start + 4]);
}

void handleDinSystemExclusive(byte *data, unsigned size) {
  parseTransportSysex(data, static_cast<uint8_t>(min<unsigned>(size, 255U)));
}

void routeIncomingChannelMessage(uint8_t sourcePort, uint8_t status, uint8_t data1, uint8_t data2) {
  if (sourcePort == USB_DEVICE_SOURCE_PORT) ++usbIncomingMessageCount;
  else if (sourcePort == 0) ++dinIncomingMessageCount;
  if (sourcePort == USB_DEVICE_SOURCE_PORT || sourcePort == 0) {
    lastIncomingSource = sourcePort;
    lastIncomingStatus = status;
    lastIncomingData1 = data1;
    lastIncomingData2 = data2;
  }
  const uint8_t type = status & 0xF0;
  uint8_t channel = (status & 0x0F) + 1;
  if (firmware3Settings.quickJumpEnabled &&
      channel == firmware3Settings.quickJumpInputChannel) {
    channel = firmware3Settings.quickJumpOutputChannel;
    status = type | ((channel - 1) & 0x0F);
  }
  if ((type == 0x90 || type == 0x80) &&
      captureNoteCcInputAssignment(channel, data1, type == 0x90 && data2 > 0)) return;
  if ((type == 0x90 || type == 0x80) &&
      captureFourButtonNoteAssignment(channel, data1, type == 0x90 && data2 > 0)) return;
  if ((type == 0x90 || type == 0x80) &&
      captureFeatureNoteAssignment(channel, data1, type == 0x90 && data2 > 0)) return;
  if ((type == 0x90 || type == 0x80) &&
      processNoteCcMap(sourcePort, channel, data1, type == 0x90 && data2 > 0)) return;
  if ((type == 0x90 || type == 0x80) &&
      processFeatureNote(channel, data1, type == 0x90 && data2 > 0)) return;
  if (type == 0x90) {
    if (channel == settings.inputChannel) {
      captureCustomArpNote(data1, data2, data2 > 0, time_us_64());
    }
    if (channel == firmware3Settings.parameterLockChannel) {
      const bool noteOn = data2 > 0;
      parameterLockHeldNotes[data1] = noteOn;
      if (noteOn) recallParameterLocks(sourcePort, channel, data1);
    }
    if (captureDivNoteAssignment(channel, data1, data2 > 0)) return;
    const uint8_t divNoteAction = handleDivNoteOverride(sourcePort, channel, data1, data2, data2 > 0);
    if (divNoteAction == 1) return;
    translateSplitInputToDrum(channel, data1);
    onInputNote(sourcePort, channel, data1, data2, data2 > 0, divNoteAction != 2);
  } else if (type == 0x80) {
    if (channel == settings.inputChannel) captureCustomArpNote(data1, 0, false, time_us_64());
    if (channel == firmware3Settings.parameterLockChannel) parameterLockHeldNotes[data1] = false;
    if (captureDivNoteAssignment(channel, data1, false)) return;
    const uint8_t divNoteAction = handleDivNoteOverride(sourcePort, channel, data1, 0, false);
    if (divNoteAction == 1) return;
    translateSplitInputToDrum(channel, data1);
    onInputNote(sourcePort, channel, data1, 0, false, divNoteAction != 2);
  } else if (type == 0xB0) {
    routeControlChange(sourcePort, channel, data1, data2);
  } else if (type == 0xE0) {
    routePitchBend(sourcePort, channel, static_cast<int>(data1 | (data2 << 7)) - 8192);
  } else if (type == 0xD0) {
    routeChannelAftertouch(sourcePort, channel, data1);
  } else if (type == 0xA0) {
    if (firmware3Settings.forwardPolyAftertouch) {
      routeProgramLikeMessage(sourcePort, status, data1, data2);
    }
  } else if (type == 0xC0) {
    routeProgramLikeMessage(sourcePort, status, data1, data2);
  }
}


int8_t arpModeNextIndex() {
  if (arpHeldCount == 0) return -1;
  const uint8_t mode = classicArpModeFromSelection(currentArpSelection());
  const uint32_t phase = arpSequenceStep;
  switch (mode) {
    case ARP_UP:
      return phase % arpHeldCount;
    case ARP_DOWN:
      return arpHeldCount - 1 - (phase % arpHeldCount);
    case ARP_UPDOWN1: {
      if (arpHeldCount == 1) return 0;
      const uint8_t period = (arpHeldCount * 2) - 2;
      const uint8_t position = phase % period;
      return (position < arpHeldCount) ? position : period - position;
    }
    case ARP_UPDOWN2: {
      if (arpHeldCount == 1) return 0;
      const uint8_t period = arpHeldCount * 2;
      const uint8_t position = phase % period;
      return (position < arpHeldCount) ? position : period - 1 - position;
    }
    case ARP_TRIGGER:
      return 0;
    case ARP_RANDOM:
      return random(arpHeldCount);
    case ARP_OFF:
    default:
      return -1;
  }
}

void arpNoteOffs() {
  for (uint8_t i = 0; i < activeArpCount; ++i) {
    if (activeArpNotes[i] >= 0) {
      const uint8_t outCh = activeArpChannels[i] ? activeArpChannels[i] : mainArpOutChannel();
      if (!channelEnabled(outCh)) continue;
      sendFanout(255, 0x80 | ((outCh - 1) & 0x0F), activeArpNotes[i], 0);
    }
  }
  activeArpCount = 0;
}

void arpAddSingleOutput(uint8_t note) {
  const uint8_t outCh = mainArpOutChannel();
  if (!channelEnabled(outCh)) return;
  if (activeArpCount >= MAX_ARP_OUTPUT_NOTES) return;
  for (uint8_t i = 0; i < activeArpCount; ++i) {
    if (activeArpNotes[i] == static_cast<int8_t>(note)) return;
  }
  const uint8_t actualCh = nextRoundRobinChannel(outCh);
  activeArpNotes[activeArpCount] = note;
  activeArpChannels[activeArpCount] = actualCh;
  activeArpCount++;
  sendFanout(255, 0x90 | ((actualCh - 1) & 0x0F), note, currentArpVelocitySetting());
}

void arpAddOutput(uint8_t note) {
  uint8_t notes[4];
  const uint8_t count = buildChordNotes(note, notes);
  for (uint8_t i = 0; i < count && activeArpCount < MAX_ARP_OUTPUT_NOTES; ++i) {
    arpAddSingleOutput(notes[i]);
  }
}

void drumArpNoteOffs() {
  if (!arpChannelSpecialMode()) return;
  for (uint8_t i = 0; i < activeDrumArpCount; ++i) {
    if (activeDrumArpNotes[i] >= 0) {
      const uint8_t outCh = firmware3Settings.drumOutputChannel;
      sendFanout(255, 0x80 | ((outCh - 1) & 0x0F), activeDrumArpNotes[i], 0);
    }
  }
  activeDrumArpCount = 0;
}

uint8_t drumArpPulseVelocity() {
  if (!arpChannelAftertouchMode()) return 127;
  const long mapped = map(drumAftertouchPressure, 0, 127, DRUM_AFTERTOUCH_MIN_VELOCITY, 127);
  return static_cast<uint8_t>(constrain(mapped, DRUM_AFTERTOUCH_MIN_VELOCITY, 127));
}

void drumArpAddOutput(uint8_t note) {
  if (!arpChannelSpecialMode()) return;
  if (activeDrumArpCount >= MAX_HELD_NOTES) return;
  for (uint8_t i = 0; i < activeDrumArpCount; ++i) {
    if (activeDrumArpNotes[i] == static_cast<int8_t>(note)) return;
  }
  activeDrumArpNotes[activeDrumArpCount++] = note;
  const uint8_t outCh = firmware3Settings.drumOutputChannel;
  sendFanout(255, 0x90 | ((outCh - 1) & 0x0F), note, drumArpPulseVelocity());
}

void runArpStep() {
  const uint8_t arpSelection = currentArpSelection();
  const uint8_t arpPattern = patternFromArpSelection(arpSelection);
  const uint8_t arpMode = classicArpModeFromSelection(arpSelection);
  const bool mainArpEnabled = channelEnabled(mainArpOutChannel()) && arpMode != ARP_OFF && arpHeldCount > 0;
  if (!mainArpEnabled) {
    arpNoteOffs();
    return;
  }

  arpNoteOffs();
  const uint8_t step = arpPatternStep % 16;
  const PatternToken token = kPatterns[arpPattern][step];

  if (mainArpEnabled && token.noteIndex == TOK_REST) {
    arpGateOffMs = 0;
    arpGlobalStep++;
    arpSequenceStep++;
    arpPatternStep = (arpPatternStep + 1) % 16;
    return;
  }

  if (mainArpEnabled && (token.noteIndex == TOK_ALL || arpMode == ARP_TRIGGER)) {
    for (uint8_t octave = 0; octave < firmware3Settings.arpOctaves; ++octave) {
      for (uint8_t i = 0; i < arpHeldCount && activeArpCount < MAX_ARP_OUTPUT_NOTES; ++i) {
        const int note = static_cast<int>(arpHeldSorted[i]) + octave * 12;
        if (note <= 127) arpAddOutput(quantizeUp(note));
      }
    }
  } else if (mainArpEnabled) {
    int8_t idx = token.noteIndex;
    if (idx == TOK_MODE || arpPattern == PAT_MODE || arpPattern == PAT_RANDOM) {
      idx = arpModeNextIndex();
    }
    if (idx >= 0 && arpHeldCount > 0) {
      const uint8_t base = arpHeldSorted[idx % arpHeldCount];
      const uint8_t octave = (arpSequenceStep / max<uint8_t>(1, arpHeldCount)) %
                             max<uint8_t>(1, firmware3Settings.arpOctaves);
      int note = base + token.semitoneOffset + ((token.octaveOffset + octave) * 12);
      note = constrain(note, 0, 127);
      arpAddOutput(quantizeUp(note));
    }
  }

  const uint32_t gateMs = max<uint32_t>(15, (divisionStepMs() * currentArpLengthPctSetting()) / 100);
  arpGateOffMs = millis() + gateMs;
  arpGlobalStep++;
  arpSequenceStep++;
  arpPatternStep = (arpPatternStep + 1) % 16;
}

void runDrumStep(uint8_t division) {
  drumArpNoteOffs();
  if (!arpChannelSpecialMode() || heldDrumCount == 0) return;
  for (uint8_t note = 0; note < 128; ++note) {
    if (heldDrumNotes[note]) drumArpAddOutput(note);
  }
  const uint64_t stepUs = musicalDurationUs(kDivisionPulseSteps[division]);
  const uint32_t gateMs = max<uint32_t>(15, static_cast<uint32_t>(stepUs / 2000ULL));
  drumGateOffMs = millis() + gateMs;
}

void tickArp() {
  const uint32_t nowMs = millis();
  const uint64_t nowUs = time_us_64();
  if (customArpLearning && !customArpWaitingForFirstNote && nowUs >= customArpLearnEndUs) {
    finishCustomArpLearn();
  }
  const bool customMode = currentArpSelection() == ARPSEL_CUSTOM;
  if (customMode) {
    arpNoteOffs();
    arpGateOffMs = 0;
    arpNextStepUs = 0;
    tickCustomArp(nowUs);
  } else {
    releaseCustomArpVoices();
    customArpCycleStartUs = 0;
    customArpPlayIndex = 0;
    if (arpHeldCount == 0) arpNoteOffs();
  }
  if (arpGateOffMs && nowMs >= arpGateOffMs) {
    arpNoteOffs();
    arpGateOffMs = 0;
  }
  if (heldDrumCount == 0) drumArpNoteOffs();
  if (drumGateOffMs && nowMs >= drumGateOffMs) {
    drumArpNoteOffs();
    drumGateOffMs = 0;
  }
  releaseArpClockIfLooperIdle();
  if (!musicalClock.synchronizedAdvanceAllowed(nowUs)) return;
  if (!customMode && arpHeldCount > 0 && (arpNextStepUs == 0 || nowUs >= arpNextStepUs)) {
    const uint8_t division = currentDivisionSetting();
    if (arpNextStepUs == 0) {
      arpSequenceStep = 0;
      arpPatternStep = 0;
      if (drumNextStepUs != 0) {
        // Drums started this phrase, so they own the clock. The arp joins
        // their grid at its own division instead of planting a second origin
        // that would drift against the first.
        arpGridOriginUs = drumGridOriginUs;
        const uint64_t stepUs = max<uint64_t>(1, musicalDurationUs(kDivisionPulseSteps[division]));
        uint32_t boundary = static_cast<uint32_t>((nowUs - arpGridOriginUs) / stepUs);
        while (swungGridTimeUs(arpGridOriginUs, boundary, division) < nowUs) ++boundary;
        arpGlobalStep = boundary;
        arpNextStepUs = swungGridTimeUs(arpGridOriginUs, boundary, division);
      } else {
        arpGridOriginUs = nowUs;
        arpGlobalStep = 0;
        arpNextStepUs = arpGridOriginUs;
      }
    }
    const uint64_t stepUs = max<uint64_t>(1, musicalDurationUs(kDivisionPulseSteps[division]));
    if (nowUs > arpNextStepUs && nowUs - arpNextStepUs > stepUs * 4ULL) {
      uint32_t boundary = static_cast<uint32_t>((nowUs - arpGridOriginUs) / stepUs);
      while (boundary > 0 && swungGridTimeUs(arpGridOriginUs, boundary, division) > nowUs) --boundary;
      arpGlobalStep = boundary;
      arpNextStepUs = swungGridTimeUs(arpGridOriginUs, boundary, division);
    }
    if (nowUs > arpNextStepUs) {
      const uint32_t lateUs = static_cast<uint32_t>(
          min<uint64_t>(nowUs - arpNextStepUs, UINT32_MAX));
      if (lateUs > perfLateMaxUs) perfLateMaxUs = lateUs;
    }
    uint8_t catchUp = 0;
    while (nowUs >= arpNextStepUs && catchUp++ < 4) {
      runArpStep();
      arpNextStepUs = swungGridTimeUs(arpGridOriginUs, arpGlobalStep, division);
    }
  }

  const int8_t drumDivision = currentDrumDivisionSetting();
  if (drumDivision < 0) {
    drumNextStepUs = 0;
    return;
  }
  if (heldDrumCount == 0) {
    // Loop recording, armed, or playing must never let a momentary gap
    // between drum hits forget the grid: that's what was producing an
    // audible timing seam in a captured take with no quantize to mask it.
    // Outside of that, forgetting it here is what lets a fresh note plant a
    // brand new origin, the normal Key Press retrigger behavior when Retrig
    // isn't set to Clock Sync, so only skip the reset while the loop has the
    // clock locked.
    if (!loopLocksArpClock()) drumNextStepUs = 0;
    return;
  }
  if (drumNextStepUs == 0 || nowUs >= drumNextStepUs) {
    if (drumNextStepUs == 0) {
      if (arpNextStepUs != 0) {
        // The arp started this phrase and is the boss. Drums follow its
        // origin with their own division.
        drumGridOriginUs = arpGridOriginUs;
        const uint64_t stepUs = max<uint64_t>(1,
            musicalDurationUs(kDivisionPulseSteps[drumDivision]));
        uint32_t boundary = static_cast<uint32_t>((nowUs - drumGridOriginUs) / stepUs);
        while (swungGridTimeUs(drumGridOriginUs, boundary, drumDivision) < nowUs) ++boundary;
        drumGlobalStep = boundary;
        drumNextStepUs = swungGridTimeUs(drumGridOriginUs, boundary, drumDivision);
      } else {
        drumGridOriginUs = nowUs;
        drumGlobalStep = 0;
        drumNextStepUs = drumGridOriginUs;
      }
    }
    const uint64_t stepUs = max<uint64_t>(1, musicalDurationUs(kDivisionPulseSteps[drumDivision]));
    if (nowUs > drumNextStepUs && nowUs - drumNextStepUs > stepUs * 4ULL) {
      uint32_t boundary = static_cast<uint32_t>((nowUs - drumGridOriginUs) / stepUs);
      while (boundary > 0 && swungGridTimeUs(drumGridOriginUs, boundary, drumDivision) > nowUs) --boundary;
      drumGlobalStep = boundary;
      drumNextStepUs = swungGridTimeUs(drumGridOriginUs, boundary, drumDivision);
    }
    if (nowUs > drumNextStepUs) {
      const uint32_t lateUs = static_cast<uint32_t>(
          min<uint64_t>(nowUs - drumNextStepUs, UINT32_MAX));
      if (lateUs > perfLateMaxUs) perfLateMaxUs = lateUs;
    }
    uint8_t catchUp = 0;
    while (nowUs >= drumNextStepUs && catchUp++ < 4) {
      runDrumStep(drumDivision);
      ++drumGlobalStep;
      drumNextStepUs = swungGridTimeUs(drumGridOriginUs, drumGlobalStep, drumDivision);
    }
  }
}

constexpr uint8_t kButtonPins[4] = {
  PIN_BUTTON_1, PIN_BUTTON_2, PIN_BUTTON_3, PIN_BUTTON_4
};

void sendCustomButtonValue(uint8_t button, bool on, uint8_t ccValue = 127) {
  if (button >= 4) return;
  const CustomButtonConfig &config = featureControls.customButtons[button];
  if (config.kind == TRIGGER_BINDING_CC) {
    sendFanout(254, static_cast<uint8_t>(0xB0 | (config.channel - 1U)),
               config.number, on ? ccValue : 0);
  } else {
    sendFanout(254, static_cast<uint8_t>((on ? 0x90 : 0x80) | (config.channel - 1U)),
               config.number, on ? 127 : 0);
  }
}

void handleCustomButton(uint8_t button, bool pressed) {
  const CustomButtonConfig &config = featureControls.customButtons[button];
  if (config.behavior == CUSTOM_BUTTON_LATCH) {
    if (!pressed) return;
    customButtonLatch[button] = !customButtonLatch[button];
    sendCustomButtonValue(button, customButtonLatch[button]);
  } else if (config.behavior == CUSTOM_BUTTON_FLAPPY &&
             config.kind == TRIGGER_BINDING_CC) {
    customButtonFlappyMs[button] = millis();
  } else {
    sendCustomButtonValue(button, pressed);
  }
}

void handleLooperButton(uint8_t button) {
  const uint32_t now = millis();
  const uint8_t actions = featureControls.looperButtonActions;
  // Arm selects too, the same as Select itself: the button press already
  // names a specific track, so either action puts the working track there.
  const bool locksToTrack =
      (actions & (LOOPER_BUTTON_SELECT | LOOPER_BUTTON_ARM)) != 0;

  // The action cycle only continues while the performer keeps tapping the same
  // track in one gesture.  A different button, a pause, or a track that is not
  // already the working track restarts it, so the first tap on a track always
  // means the first action and a single press can never reach Clear.
  if (button != lastLooperButton ||
      now - looperButtonLastMs > LOOPER_BUTTON_CYCLE_MS ||
      (locksToTrack && multitrackLooper.selectedTrack() != button)) {
    memset(looperButtonStep, 0, sizeof(looperButtonStep));
  }
  lastLooperButton = button;
  looperButtonLastMs = now;

  static constexpr uint8_t orderedActions[6] = {
    LOOPER_BUTTON_SELECT, LOOPER_BUTTON_ARM, LOOPER_BUTTON_MUTE,
    LOOPER_BUTTON_SOLO, LOOPER_BUTTON_DELETE, LOOPER_BUTTON_UNDO
  };
  uint8_t selectedAction = 0;
  for (uint8_t offset = 0; offset < 6; ++offset) {
    const uint8_t index = (looperButtonStep[button] + offset) % 6U;
    if ((actions & orderedActions[index]) == 0) continue;
    selectedAction = orderedActions[index];
    looperButtonStep[button] = (index + 1U) % 6U;
    break;
  }
  if (selectedAction == LOOPER_BUTTON_SELECT) {
    selectLooperTrack(button);
  } else if (selectedAction == LOOPER_BUTTON_ARM) {
    // Selects the track as part of arming it, and disarms it again if this
    // same track is already the pending arm.
    toggleLooperArmForTrack(button);
  } else if (selectedAction == LOOPER_BUTTON_MUTE) {
    setLoopTrackMuted(button, !multitrackLooper.track(button).muted);
  } else if (selectedAction == LOOPER_BUTTON_SOLO) {
    setExclusiveLoopSolo(button, !multitrackLooper.track(button).solo);
  } else if (selectedAction == LOOPER_BUTTON_DELETE) {
    // Clearing the working track cancels a real in-progress recording aimed
    // at it, but a mere pending arm survives, so the track is taken first and
    // the selection follows afterwards either way.
    multitrackLooper.safeClear(button, releaseMultitrackOutput, nullptr);
    selectLooperTrack(button);
  } else if (selectedAction == LOOPER_BUTTON_UNDO) {
    undoLoopTrackClear(button);
    selectLooperTrack(button);
  }
  if (selectedAction != 0 && selectedAction != LOOPER_BUTTON_SELECT &&
      selectedAction != LOOPER_BUTTON_ARM) {
    markLoopStorageDirty();
  }
  releaseSilencedMultitrackOutputs();
  refreshLoopUiState();
  ui.dirty = true;
}

// Two physical buttons held together in Looper mode is the same stop the eye
// or pad sensor gives on its first press: finish anything mid-capture and
// stop the transport. Doing it again while already stopped plays instead of
// chaining into that sensor's stop-then-clear escalation, since holding
// three buttons together, below, is the dedicated way to reach a clear from
// the physical buttons.
void handleLooperTwoButtonHold() {
  const uint64_t nowUs = time_us_64();
  if (multitrackLooper.recording() || multitrackLooper.recordingArmed()) {
    finishActiveMultitrackRecording(nowUs, false);
  }
  if (multitrackLooper.playing()) {
    multitrackLooper.stop(releaseMultitrackOutput, nullptr);
    loopSafeClearArmed = true;
  } else {
    multitrackLooper.start(nowUs);
    loopSafeClearArmed = false;
  }
  releaseSilencedMultitrackOutputs();
  refreshLoopUiState();
  ui.dirty = true;
}

// Three physical buttons held together in Looper mode is the same undoable
// clear-all gesture the eye/pad reaches on a second stop-then-clear press:
// clear every track that has anything audible, or if every track is already
// cleared, bring them all back instead.
void handleLooperThreeButtonHold() {
  const uint64_t nowUs = time_us_64();
  if (multitrackLooper.recording() || multitrackLooper.recordingArmed()) {
    finishActiveMultitrackRecording(nowUs, false);
  }
  clearOrUndoAllLoopTracks();
  loopSafeClearArmed = false;
  markLoopStorageDirty();
  releaseSilencedMultitrackOutputs();
  refreshLoopUiState();
  ui.dirty = true;
}

void sendChordMemorySlot(uint8_t slot, bool on) {
  if (slot >= 4) return;
  ChordMemorySlot &chord = featureControls.chordMemories[slot];
  for (uint8_t i = 0; i < chord.count; ++i) {
    sendFanout(254, static_cast<uint8_t>((on ? 0x90 : 0x80) |
        ((chord.channels[i] - 1U) & 0x0F)), chord.notes[i], on ? chord.velocities[i] : 0);
  }
  chordButtonPlaying[slot] = on;
}

void handleChordMemoryButton(uint8_t button, bool pressed) {
  if (pressed && chordClearArmed) {
    if (chordButtonPlaying[button]) sendChordMemorySlot(button, false);
    featureControls.chordMemories[button] = ChordMemorySlot{};
    chordClearArmed = false;
    markExtendedPresetDirty();
    ui.dirty = true;
    return;
  }
  if (pressed && chordLearnArmed) {
    if (chordButtonPlaying[button]) sendChordMemorySlot(button, false);
    featureControls.chordMemories[button] = ChordMemorySlot{};
    chordLearnSlot = button;
    chordLearnActive = true;
    chordLearnArmed = false;
    ui.dirty = true;
    return;
  }
  sendChordMemorySlot(button, pressed);
}

void captureChordMemoryOutput(uint8_t sourcePort, uint8_t channel,
                              uint8_t note, uint8_t velocity) {
  if (!chordLearnActive || loopOwnsInput(sourcePort) || generatedLiveEffectSource(sourcePort) ||
      chordLearnSlot >= 4 || !channelEnabled(channel)) return;
  ChordMemorySlot &chord = featureControls.chordMemories[chordLearnSlot];
  for (uint8_t i = 0; i < chord.count; ++i) {
    if (chord.channels[i] == channel && chord.notes[i] == note) {
      chord.velocities[i] = max<uint8_t>(1, velocity);
      return;
    }
  }
  if (chord.count >= 16) return;
  const uint8_t index = chord.count++;
  chord.channels[index] = channel;
  chord.notes[index] = note;
  chord.velocities[index] = max<uint8_t>(1, velocity);
  markExtendedPresetDirty();
}

void finishChordMemoryLearnIfReady() {
  if (!chordLearnActive || anyPhysicalInputNotesHeld()) return;
  chordLearnActive = false;
  markExtendedPresetDirty();
  ui.dirty = true;
}

void tickFlappyButtons() {
  const uint32_t now = millis();
  for (uint8_t button = 0; button < 4; ++button) {
    const CustomButtonConfig &config = featureControls.customButtons[button];
    if (featureControls.fourButtonMode != FOUR_BUTTON_CUSTOM ||
        config.behavior != CUSTOM_BUTTON_FLAPPY || config.kind != TRIGGER_BINDING_CC ||
        now - customButtonFlappyMs[button] < 12) continue;
    customButtonFlappyMs[button] = now;
    const uint8_t previous = customButtonFlappyValue[button];
    if (physicalButtonState[button]) {
      customButtonFlappyValue[button] = min<uint8_t>(127, previous + 4U);
    } else {
      customButtonFlappyValue[button] = previous > 2U ? previous - 2U : 0;
    }
    if (customButtonFlappyValue[button] != previous) {
      sendCustomButtonValue(button, true, customButtonFlappyValue[button]);
    }
  }
}

uint8_t heldPhysicalButtonCount() {
  uint8_t count = 0;
  for (uint8_t button = 0; button < 4; ++button) {
    if (physicalButtonState[button]) ++count;
  }
  return count;
}

void pollButtons() {
  const uint32_t now = millis();
  for (uint8_t button = 0; button < 4; ++button) {
    const bool pressed = digitalRead(kButtonPins[button]);
    if (pressed == physicalButtonState[button] ||
        now - physicalButtonChangeMs[button] <= BUTTON_DEBOUNCE_MS) continue;
    physicalButtonChangeMs[button] = now;
    physicalButtonState[button] = pressed;
    markActivity(false);
    if (featureControls.fourButtonMode == FOUR_BUTTON_CUSTOM) {
      handleCustomButton(button, pressed);
    } else if (featureControls.fourButtonMode == FOUR_BUTTON_LOOPER) {
      // physicalButtonState[] above already reflects this press, so counting
      // it right here is however many buttons are physically down this
      // instant, no matter which order they went down in or how far apart:
      // one button still means its own per-track action, two down together
      // means stop or play, three down together means the whole-loop clear
      // or undo.
      if (pressed) {
        const uint8_t heldCount = heldPhysicalButtonCount();
        if (heldCount >= 3) {
          handleLooperThreeButtonHold();
        } else if (heldCount == 2) {
          handleLooperTwoButtonHold();
        } else {
          handleLooperButton(button);
        }
      }
    } else {
      handleChordMemoryButton(button, pressed);
    }
  }
  tickFlappyButtons();
}

void applyArpFreezeSnapshot(bool plusMode) {
  if (!anyPhysicalInputNotesHeld()) {
    clearFreezeState();
    rebuildArpHeldSorted();
    updateBassVoice();
    return;
  }

  bool frozenNow[128];
  memset(frozenNow, 0, sizeof(frozenNow));
  for (uint8_t note = 0; note < 128; ++note) {
    if (physicalHeldInputNotes[note]) frozenNow[note] = true;
  }

  if (!plusMode && arpFreezePlusActive) {
    for (uint8_t note = 0; note < 128; ++note) {
      if (!thruFrozenNotes[note]) continue;
      const uint8_t out = thruFrozenMappedNotes[note];
      if (out <= 127) thruOutputRefOff(255, out);
    }
    memset(thruFrozenNotes, 0, sizeof(thruFrozenNotes));
    memset(thruFrozenMappedNotes, 0xFF, sizeof(thruFrozenMappedNotes));
  }

  for (uint8_t note = 0; note < 128; ++note) {
    arpFrozenNotes[note] = frozenNow[note];
    if (!plusMode) continue;
    if (frozenNow[note] && !thruFrozenNotes[note]) {
      const uint8_t velocity = heldVelocities[note] ? heldVelocities[note] : currentArpVelocitySetting();
      const uint8_t out = quantizeUp(note);
      thruOutputRefOn(255, out, velocity);
      thruFrozenNotes[note] = true;
      thruFrozenMappedNotes[note] = out;
    } else if (!frozenNow[note] && thruFrozenNotes[note]) {
      const uint8_t out = thruFrozenMappedNotes[note];
      if (out <= 127) thruOutputRefOff(255, out);
      thruFrozenNotes[note] = false;
      thruFrozenMappedNotes[note] = 0xFF;
    }
  }

  arpFreezeActive = true;
  arpFreezePlusActive = plusMode;
  rebuildArpHeldSorted();
  updateBassVoice();
}

bool sensorLoopReleaseDebounced(uint32_t &releaseStartMs, bool released) {
  if (!released) {
    releaseStartMs = 0;
    return false;
  }
  const uint32_t now = millis();
  if (releaseStartMs == 0) {
    releaseStartMs = now;
    return false;
  }
  return (now - releaseStartMs) >= SENSOR_LOOP_REARM_DEBOUNCE_MS;
}

void updateControllerOutput(uint8_t sourcePort, uint8_t mode, bool inRange, uint8_t pct, uint8_t ch,
                            int8_t &activeNote, int16_t &lastPitch, int16_t &lastCcValue,
                            bool &latchCloseState, bool &freezeCloseState) {
  if (!channelEnabled(ch)) return;

  const bool noteMode = sensorModeIsNotes(mode);
  const bool ccMode = sensorModeIsCc(mode);
  const bool pitchMode = sensorModeIsPitch(mode);
  const bool latchMode = (mode == SENSOR_ARP_LATCH || mode == SENSOR_ARP_LATCH_PLUS);
  const bool freezeMode = arpFreezeMode(mode);
  const bool loopRpMode = (mode == SENSOR_LOOP_REC_PLAY);
  const bool loopSdMode = (mode == SENSOR_LOOP_STOP_DELETE);
  uint32_t *loopReleaseStartMs = nullptr;
  if (sourcePort == 253) loopReleaseStartMs = &sensorLoopReleaseStartMs;
  else if (sourcePort == 252) loopReleaseStartMs = &pushLoopReleaseStartMs;
  const bool debounceLoopRelease = loopReleaseStartMs && (loopRpMode || loopSdMode);

  if (loopReleaseStartMs && !loopRpMode && !loopSdMode) *loopReleaseStartMs = 0;

  if (!inRange) {
    const bool loopReleaseReady = !debounceLoopRelease ||
                                  sensorLoopReleaseDebounced(*loopReleaseStartMs, true);
    if (latchMode || (loopRpMode && loopReleaseReady)) latchCloseState = false;
    if (freezeMode || (loopSdMode && loopReleaseReady)) freezeCloseState = false;

    if (activeNote >= 0) {
      sendFanout(sourcePort, 0x80 | ((ch - 1) & 0x0F), activeNote, 0);
      activeNote = -1;
    }
    if (ccMode && lastCcValue >= 0) {
      sendFanout(sourcePort, 0xB0 | ((ch - 1) & 0x0F), static_cast<uint8_t>(mode - SENSOR_CC1 + 1), 0);
    }
    lastCcValue = -1;
    if (pitchMode && lastPitch != 0) {
      sendFanout(sourcePort, 0xE0 | ((ch - 1) & 0x0F), 0, 64);
    }
    lastPitch = 0;
    return;
  }

  if (!noteMode && activeNote >= 0) {
    sendFanout(sourcePort, 0x80 | ((ch - 1) & 0x0F), activeNote, 0);
    activeNote = -1;
  }

  if (latchMode) {
    const bool closeHalf = pct >= 50;
    if (closeHalf && !latchCloseState) {
      clearArpLatchNotes();
      rebuildArpHeldSorted();
      latchCloseState = true;
    } else if (!closeHalf) {
      latchCloseState = false;
    }
    return;
  }
  if (!loopRpMode) latchCloseState = false;

  if (freezeMode) {
    const bool closeHalf = pct >= 50;
    if (closeHalf && !freezeCloseState) {
      applyArpFreezeSnapshot(mode == SENSOR_ARP_FREEZ_PLUS);
      freezeCloseState = true;
    } else if (!closeHalf) {
      freezeCloseState = false;
    }
    return;
  }
  if (!loopSdMode) freezeCloseState = false;

  if (loopRpMode) {
    const bool closeHalf = pct >= 50;
    if (closeHalf && !latchCloseState) {
      if (debounceLoopRelease) sensorLoopReleaseDebounced(*loopReleaseStartMs, false);
      handleLoopRecPlayTrigger();
      latchCloseState = true;
    } else if (closeHalf) {
      if (debounceLoopRelease) sensorLoopReleaseDebounced(*loopReleaseStartMs, false);
    } else if (!debounceLoopRelease || sensorLoopReleaseDebounced(*loopReleaseStartMs, true)) {
      latchCloseState = false;
    }
    return;
  }

  if (loopSdMode) {
    const bool closeHalf = pct >= 50;
    if (closeHalf && !freezeCloseState) {
      if (debounceLoopRelease) sensorLoopReleaseDebounced(*loopReleaseStartMs, false);
      handleLoopStopDeleteTrigger();
      freezeCloseState = true;
    } else if (closeHalf) {
      if (debounceLoopRelease) sensorLoopReleaseDebounced(*loopReleaseStartMs, false);
    } else if (!debounceLoopRelease || sensorLoopReleaseDebounced(*loopReleaseStartMs, true)) {
      freezeCloseState = false;
    }
    return;
  }

  if (pitchMode) {
    const int bendPct = (mode == SENSOR_PITCH_UP) ? pct : -pct;
    if (bendPct != lastPitch) {
      const int bend14 = map(bendPct, -100, 100, 0, 16383);
      sendFanout(sourcePort, 0xE0 | ((ch - 1) & 0x0F), bend14 & 0x7F, (bend14 >> 7) & 0x7F);
      lastPitch = bendPct;
    }
    return;
  }
  if (lastPitch != 0) {
    sendFanout(sourcePort, 0xE0 | ((ch - 1) & 0x0F), 0, 64);
    lastPitch = 0;
  }

  if (noteMode) {
    const uint8_t rootOct = mode - SENSOR_NOTES_C0;
    const uint8_t noteSpan = 24;
    const uint8_t notePct = constrain(
      map(pct, SENSOR_NOTE_FAR_TRIM_PCT, 100 - SENSOR_NOTE_CLOSE_TRIM_PCT, 0, 100),
      0, 100
    );
    const int16_t paddedIdx = map(notePct, 0, 100, -2, noteSpan + 1);
    const uint8_t idx = constrain(paddedIdx, 0, noteSpan);
    uint8_t note = (rootOct * 12) + idx;
    note = quantizeUp(note);
    if (note != activeNote) {
      if (activeNote >= 0) sendFanout(sourcePort, 0x80 | ((ch - 1) & 0x0F), activeNote, 0);
      sendFanout(sourcePort, 0x90 | ((ch - 1) & 0x0F), note, currentArpVelocitySetting());
      activeNote = note;
    }
    return;
  }

  if (ccMode) {
    const uint8_t cc = mode - SENSOR_CC1 + 1;
    const uint8_t value = map(pct, 0, 100, 0, 127);
    if (value != lastCcValue) {
      sendFanout(sourcePort, 0xB0 | ((ch - 1) & 0x0F), cc, value);
      lastCcValue = value;
    }
    return;
  }
  lastCcValue = -1;

  if (mode == SENSOR_LOOP_TRIGGER) {
    static uint32_t lastLoopTrigMsSensor = 0;
    static uint32_t lastLoopTrigMsPush = 0;
    uint32_t &lastLoopTrigMs = (sourcePort == 253) ? lastLoopTrigMsSensor : lastLoopTrigMsPush;
    if ((millis() - lastLoopTrigMs) > 200) {
      sendFanout(sourcePort, 0xB0 | ((ch - 1) & 0x0F), 103, 127);
      sendFanout(sourcePort, 0xB0 | ((ch - 1) & 0x0F), 103, 0);
      lastLoopTrigMs = millis();
    }
  }
}

void updateSensorOutput() {
  if (!sensorRt.present) return;
  updateControllerOutput(253, settings.sensorMode, sensorRt.inRange, sensorPercent(), settings.sensorChannel,
                         sensorRt.activeNote, sensorRt.lastPitch, sensorRt.lastCcValue,
                         arpLatchSensorClose, arpFreezeSensorClose);
}

void updatePushOutput() {
  updateControllerOutput(252, settings.pushMode, pushRt.inRange, pushRt.pct, settings.sensorChannel,
                         pushRt.activeNote, pushRt.lastPitch, pushRt.lastCcValue,
                         arpLatchPushClose, arpFreezePushClose);
}

void updateSensorLed() {
#if !ARPNMIDI_ENABLE_RGB_LED
  return;
#else
  if (settings.sensorMode == SENSOR_OFF || !sensorRt.present || !sensorRt.inRange) {
    onboardRgb.setPixelColor(0, onboardRgb.Color(0, 0, 0));
    onboardRgb.show();
    return;
  }

  const int span = SENSOR_LED_MAX_MM - VL53_VALID_MIN_MM;
  const int rel = constrain(SENSOR_LED_MAX_MM - sensorRt.mm, 0, span);
  const uint8_t pct = map(rel, 0, span, 0, 100);

  uint8_t r = 0;
  uint8_t g = 0;
  uint8_t b = 0;

  if (pct > 0 && pct <= 50) {
    r = map(pct, 1, 50, 8, 255);
  } else if (pct <= 85) {
    r = 255;
    g = map(pct, 51, 85, 0, 255);
  } else {
    r = 255;
    g = 255;
    b = map(pct, 86, 100, 0, 255);
  }

  onboardRgb.setPixelColor(0, onboardRgb.Color(r, g, b));
  onboardRgb.show();
#endif
}

void pollSensorHardwareCore1() {
  if (!sensorRt.present) return;
  const uint32_t now = millis();
  if ((now - sensorHardwareLastPollMs) < SENSOR_POLL_MS) return;
  // Shares the I2C peripheral (i2c1/Wire1) with the OLED's DMA-driven push.
  // Touching it here while that transfer is still in flight would corrupt
  // both. Skip this pass without marking it polled, so the very next
  // loop1() pass tries again right away instead of waiting out a whole new
  // interval once the push finishes.
  if (displayDmaBusy()) return;
  sensorHardwareLastPollMs = now;

  // Peek data-ready first. The library's read blocks polling I2C until a
  // sample arrives, and this core also drains outgoing MIDI, so a slow or
  // wedged sensor must cost one register read and nothing more.
  static uint32_t sensorFreshSampleMs = 0;
  if (sensorFreshSampleMs == 0) sensorFreshSampleMs = now;
  uint16_t mm;
  bool timedOut;
  if ((tof.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
    if (now - sensorFreshSampleMs < 250UL) return;
    mm = 0xFFFF;
    timedOut = true;
  } else {
    mm = tof.readRangeContinuousMillimeters();
    timedOut = tof.timeoutOccurred();
  }
  sensorFreshSampleMs = now;
  ++sensorSampleSequence;
  __dmb();
  sensorSampleMm = mm;
  sensorSampleTimedOut = timedOut;
  __dmb();
  ++sensorSampleSequence;
}

void pollSensor() {
  if (!sensorRt.present) return;
  const uint32_t sequence = sensorSampleSequence;
  if ((sequence & 1U) || sequence == consumedSensorSampleSequence) return;
  __dmb();
  const uint16_t mm = sensorSampleMm;
  const bool timedOut = sensorSampleTimedOut;
  __dmb();
  if (sequence != sensorSampleSequence) return;
  consumedSensorSampleSequence = sequence;

  const uint32_t now = millis();
  const bool wasInRange = sensorRt.inRange;
  if (timedOut) {
    sensorRt.inRange = false;
    updateSensorOutput();
    updateSensorLed();
    if (wasInRange && sensorParamEligible(ui.selectedSetting)) ui.dirty = true;
    return;
  }

  sensorRt.mm = mm;
  sensorRt.inRange = (mm >= VL53_VALID_MIN_MM && mm <= SENSOR_ACTIVE_MAX_MM);
  if (sensorRt.inRange) {
    sensorRt.lastSeenMs = now;
    markActivity(false);
    if (sensorParamEligible(ui.selectedSetting)) ui.dirty = true;
  } else if ((now - sensorRt.lastSeenMs) > SENSOR_TIMEOUT_MS) {
    sensorRt.inRange = false;
  }

  if (wasInRange != sensorRt.inRange && sensorParamEligible(ui.selectedSetting)) {
    ui.dirty = true;
  }

  if (ui.selectedSetting == SET_BPM) {
    if (!wasInRange && sensorRt.inRange) registerTapTempo();
    updateControllerOutput(253, settings.sensorMode, false, 0, settings.sensorChannel,
                           sensorRt.activeNote, sensorRt.lastPitch, sensorRt.lastCcValue,
                           arpLatchSensorClose, arpFreezeSensorClose);
    updateSensorLed();
    return;
  }

  updateSensorOutput();
  updateSensorLed();
}

void pollPush() {
  const uint32_t now = millis();
  if ((now - pushRt.lastPollMs) < PUSH_POLL_MS) return;
  pushRt.lastPollMs = now;

  pushRt.raw = analogRead(PIN_PUSH);
  const bool wasInRange = pushRt.inRange;
  pushRt.inRange = (pushRt.raw <= PUSH_RAW_FAR) && (pushRt.raw < PUSH_RAW_OFF);

  if (pushRt.inRange) {
    const uint16_t clamped = constrain(pushRt.raw, PUSH_RAW_NEAR, PUSH_RAW_FAR);
    // Lower raw = closer -> higher percent.
    const uint8_t linearPct = map(clamped, PUSH_RAW_FAR, PUSH_RAW_NEAR, 0, 100);
    uint16_t shaped = linearPct;
    for (uint8_t i = 1; i < PUSH_CURVE_POWER; ++i) {
      shaped = (shaped * linearPct + 50) / 100;
    }
    pushRt.pct = static_cast<uint8_t>(constrain(shaped, 0, 100));
    markActivity(false);
    if (sensorParamEligible(ui.selectedSetting)) ui.dirty = true;
  } else {
    pushRt.pct = 0;
  }

  if (wasInRange != pushRt.inRange && sensorParamEligible(ui.selectedSetting)) {
    ui.dirty = true;
  }

  if (ui.selectedSetting == SET_BPM) {
    if (!wasInRange && pushRt.inRange) registerTapTempo();
    updateControllerOutput(252, settings.pushMode, false, 0, settings.sensorChannel,
                           pushRt.activeNote, pushRt.lastPitch, pushRt.lastCcValue,
                           arpLatchPushClose, arpFreezePushClose);
    return;
  }

  updatePushOutput();
}

String settingValueString(uint8_t id) {
  if (cancelSelectedFor(id)) return "CANCEL";
  const int16_t v = effectiveSettingValue(id);
  switch (id) {
    case SET_BPM: {
      return String(v);
    }
    case SET_ARP_MODE: return "ARP";
    case SET_DIVISION: return v == ARP_DIVISION_FOLLOW_DRUM ? "DRUM" : kDivisionNames[v];
    case SET_VELOCITY: return String(map(v, 0, 127, 0, 100)) + "%";
    case SET_LENGTH: return String(v) + "%";
    case SET_INPUT_CH: return midiChannelLabel(v);
    case SET_ARP_OUT_CH: return midiChannelLabel(v, true);
    case SET_BASS_CH: return bassLabel(v);
    case SET_THRU_OUT_CH: return midiChannelLabel(v, true);
    case SET_RND_RBN:
      if (v == RND_RBN_CH10_TO_1_SLOT) return roundRobinCh10To1Enabled() ? "[x]CH10-1+" : "[ ]CH10-1+";
      if (v == RND_RBN_CH10_TO_2_SLOT) return roundRobinCh10To2Enabled() ? "[x]CH10-2+" : "[ ]CH10-2+";
      if (v == RND_RBN_RANDOM_SLOT) return roundRobinRandomEnabled() ? "[x] RANDOM" : "[ ] RANDOM";
      if (v == RND_RBN_CLEAR_SLOT) return "CLEAR";
      if (v == RND_RBN_BACK_SLOT) return "BACK";
      return String((settings.roundRobinMask & channelBit(v + 1)) ? "[x] CH " : "[ ] CH ") + String(v + 1);
    case SET_ROUTER:
      if (routerEditStage == ROUTER_STAGE_LIST && v == ROUTER_CLEAR_SLOT) return "CLEAR";
      if (routerEditStage == ROUTER_STAGE_LIST && v == ROUTER_BACK_SLOT) return "BACK";
      return "";
    case SET_DIV_NOTES:
      if (v == DIV_NOTE_PLUS_SLOT) return "+HAT NOTE";
      if (v == DIV_NOTE_RESET_SLOT) return "RESET";
      if (v == DIV_NOTE_BACK_SLOT) return "BACK";
      return kDivisionNames[divNoteSlotToDivision(v)];
    case SET_MAP_CC:
      return "FEATURES";
    case SET_CC_MAP:
      return "CC MAP";
    case SET_NOTE_CC:
      return "NOTE>CC";
    case SET_FOUR_BUTTON:
      return "4BUTTON";
    case SET_MUTE_SOLO:
      return "LOOP MIX";
    case SET_LEGATO_CH: return midiChannelLabel(v, true);
    case SET_CC_OUT_CH: return ccChannelLabel(v);
    case SET_SENSOR_CH: return midiChannelLabel(v);
    case SET_SENSOR_MODE: return kSensorModeNames[v];
    case SET_PUSH_MODE: return kSensorModeNames[v];
    case SET_LOOP_BARS: return "LOOPER";
    case SET_FORCE_KEY:
      if (v == 0) return "OFF";
      if (v <= 12) return String(kNoteNames[v - 1]);
      return String("CKEY ") + kNoteNames[v - 13];
    case SET_FORCE_SCALE: return "SCALE";
    case SET_GUITAR_PIANO: return (v == 0) ? "GUITAR" : "PIANO";
    case SET_LOAD_PRESET: return String(v + 1);
    case SET_SAVE_PRESET: return String(v + 1);
    case SET_SCREEN_SAVER:
      return screenSaverLabel(v);
    case SET_PANIC: return "PANIC";
    default: return "";
  }
}

bool currentSubmenuLabel(String &label, uint8_t &index) {
  if (ui.menuMode != MENU_EDIT) return false;
  switch (ui.selectedSetting) {
    case SET_ARP_MODE: {
      static const char *const names[] = {
        "MODE", "DIVISION", "ARP VEL", "ARP LENGTH", "OCTAVES", "RETRIG",
        "ORDER", "LENGTH", "LEARN ARP", "CLEAR ARP", "BACK"
      };
      index = arpMenuUi.cursor; label = names[index]; return true;
    }
    case SET_LIVE_VELOCITY: {
      static const char *const names[] = {"TARGET", "ON/OFF", "VELOCITY", "BACK"};
      index = liveVelocityUi.cursor; label = names[index]; return true;
    }
    case SET_LIVE_NOTE_LENGTH: {
      static const char *const names[] = {"TARGET", "ON/OFF", "NOTELENGT", "BACK"};
      index = liveNoteLengthUi.cursor; label = names[index]; return true;
    }
    case SET_STUTTER: {
      static const char *const names[] = {"DIVISION", "ON/OFF", "TIMEOUT", "TARGET", "BACK"};
      index = stutterUi.cursor; label = names[index]; return true;
    }
    case SET_ECHO: {
      static const char *const names[] = {"LENGTH", "ON/OFF", "WET", "DELAY", "DRIFT", "TARGET", "BACK"};
      index = echoUi.cursor; label = names[index]; return true;
    }
    case SET_QUICK_JUMP: {
      static const char *const names[] = {"INPUT", "OUTPUT", "ON/OFF", "HOLD", "BACK"};
      index = quickJumpUi.cursor; label = names[index]; return true;
    }
    case SET_BASS_CH: {
      static const char *const names[] = {"CH", "OCTAVE", "HIGH NOTE", "BACK"};
      index = bassUi.cursor; label = names[index]; return true;
    }
    case SET_DRUM_MAGIC: {
      static const char *const names[] = {
        "ON/OFF", "INPUT", "OUTPUT", "SPLIT", "MAP START", "AT>VEL", "DIVISION", "BACK"
      };
      index = drumMagicUi.cursor; label = names[index]; return true;
    }
    case SET_LOOP_BARS: {
      static const char *const names[] = {
        "TRACK", "LENGTH", "TRK QUANT", "NEW TRACK", "AUTO ARM",
        "TIME TRAV", "REC CC", "TRNSPRT", "BACK"
      };
      index = looperSettingsUi.cursor; label = names[index]; return true;
    }
    case SET_NOTE_CC:
      if (noteCcUiStage == NOTE_CC_UI_SLOT_ACTION) {
        static const char *const names[] = {"EDIT", "CLEAR", "CANCEL"};
        index = noteCcSlotActionCursor;
        label = names[index];
        return true;
      }
      if (noteCcUiStage != NOTE_CC_UI_LIST) {
        static const char *const names[] = {
          "INPUT CH", "INPUT NOTE", "OUTPUT CH", "OUTPUT CC", "BEHAVIOR"
        };
        index = noteCcUiStage - NOTE_CC_UI_INPUT_CHANNEL;
        label = names[index];
        return true;
      }
      return false;
    case SET_PARAMETER_LOCK: {
      static const char *const names[] = {"CHANNEL", "CLEAR", "BACK"};
      index = parameterLockUi.cursor; label = names[index]; return true;
    }
    case SET_CHORD: {
      static const char *const names[] = {"ON/OFF", "POSITION 1", "POSITION 2", "POSITION 3", "POSITION 4", "BACK"};
      index = chordUi.cursor; label = names[index]; return true;
    }
    case SET_FORCE_SCALE:
      index = scaleUi.cursor;
      if (index == 0) label = "SCALE TYPE";
      else if (index == 13) label = "BACK";
      else if (index == 1) label = "USER ROOT";
      else label = String("INTVL +") + String(index - 1U);
      return true;
    case SET_LIVE_CC: {
      static const char *const names[] = {"CC NUMBER", "VALUE", "BACK"};
      index = liveCcCursor; label = names[index]; return true;
    }
    case SET_GLOBAL: {
      static const char *const names[] = {
        "AUTO SAVE", "CLOCK IN", "CLOCK OUT", "TIME SIG", "CH AFTERTCH",
        "POLY AFTER", "AFTER CC", "AT>ARP VEL", "RESET SLOT", "BACK"
      };
      index = globalUi.cursor; label = names[index]; return true;
    }
    default: return false;
  }
}

// Holding the knob fired the global panic. The screen says only that, in the
// same face as the yellow titles, centered in the blue section, until the
// confirmation window passes.
void drawPanicHoldScreen() {
  display.setFont(&FreeSans9pt7b);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(F("KNOB HOLD"), 0, 0, &x1, &y1, &w, &h);
  display.setCursor((SCREEN_W - static_cast<int16_t>(w)) / 2, 36);
  display.print(F("KNOB HOLD"));
  display.getTextBounds(F("PANIC"), 0, 0, &x1, &y1, &w, &h);
  display.setCursor((SCREEN_W - static_cast<int16_t>(w)) / 2, 58);
  display.print(F("PANIC"));
  display.setFont();
}

void drawModeLabel() {
  display.fillRect(0, MODE_INFO_Y, SCREEN_W, MODE_INFO_H, SSD1306_BLACK);
  display.setTextColor(SSD1306_WHITE);
  display.setFont(&FreeSans9pt7b);
  display.setTextSize(1);
  display.setCursor(0, MODE_INFO_Y + 14);
  if (ui.selectedSetting == SET_FORCE_KEY) {
    const int16_t key = effectiveSettingValue(SET_FORCE_KEY);
    if (key == 0) {
      display.print(F("KEY OFF"));
    } else if (key <= 12) {
      display.print(F("KEY "));
      display.print(kNoteNames[key - 1]);
    } else {
      display.print(F("CKEY "));
      display.print(kNoteNames[key - 13]);
    }
  } else if (ui.selectedSetting == SET_FORCE_SCALE && ui.menuMode != MENU_EDIT) {
    display.print(kForceScaleNames[settings.forceScale]);
  } else {
    String submenuLabel;
    uint8_t submenuIndex = 0;
    if (currentSubmenuLabel(submenuLabel, submenuIndex)) {
      display.print(static_cast<char>('A' + submenuIndex));
      display.print(F(" "));
      display.print(submenuLabel);
    } else {
      display.print(kSettingNames[ui.selectedSetting]);
    }
  }
  display.setFont();
}

void drawBarValue(uint8_t pct, const String &label) {
  const uint8_t barW = map(pct, 0, 100, 0, SCREEN_W);
  display.drawRect(0, 0, SCREEN_W, 24, SSD1306_WHITE);
  display.fillRect(1, 1, min<int>(barW, SCREEN_W - 2), 22, SSD1306_WHITE);
  display.fillRect(0, 26, SCREEN_W, 20, SSD1306_BLACK);
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);
  display.setCursor(0, 28);
  display.print(label);
}

void drawNamedBarValue(const String &name, uint16_t value, uint16_t maximum,
                       const String &label, int16_t neutral = -1) {
  (void)name;
  maximum = max<uint16_t>(1, maximum);
  value = min<uint16_t>(value, maximum);
  display.setTextColor(SSD1306_WHITE);
  display.drawRect(0, 0, SCREEN_W, 20, SSD1306_WHITE);
  const uint8_t fillWidth = static_cast<uint8_t>(
      (static_cast<uint32_t>(value) * (SCREEN_W - 2U)) / maximum);
  if (fillWidth > 0) display.fillRect(1, 1, fillWidth, 18, SSD1306_WHITE);
  if (neutral >= 0 && neutral <= static_cast<int16_t>(maximum)) {
    const int markerX = 1 + (static_cast<uint32_t>(neutral) * (SCREEN_W - 2U)) / maximum;
    display.drawLine(markerX, 0, markerX, 21, SSD1306_WHITE);
  }
  display.setTextSize(label.length() <= 7 ? 3 : (label.length() <= 11 ? 2 : 1));
  display.setCursor(0, label.length() <= 7 ? 22 : (label.length() <= 11 ? 26 : 32));
  display.print(label);
}

void drawDivisionPie(uint8_t divisionId) {
  const bool followsDrum = divisionId == ARP_DIVISION_FOLLOW_DRUM;
  if (followsDrum) divisionId = currentDivisionSetting();
  const int cx = 90;
  const int cy = 20;
  const int r = 18;
  display.drawCircle(cx, cy, r, SSD1306_WHITE);
  const float measureFraction = kDivisionQuarterSteps[divisionId] / 4.0f;
  if (measureFraction >= 0.99f) {
    display.fillCircle(cx, cy, r - 1, SSD1306_WHITE);
  } else {
    const float start = -HALF_PI;
    const float end = start + (TWO_PI * measureFraction);
    for (uint8_t i = 0; i <= 48; ++i) {
      const float t = static_cast<float>(i) / 48.0f;
      const float a = start + ((end - start) * t);
      display.drawLine(cx, cy, cx + static_cast<int>(cos(a) * (r - 1)), cy + static_cast<int>(sin(a) * (r - 1)), SSD1306_WHITE);
    }
    display.drawLine(cx, cy, cx, cy - r, SSD1306_WHITE);
    display.drawLine(cx, cy, cx + static_cast<int>(cos(end) * r), cy + static_cast<int>(sin(end) * r), SSD1306_WHITE);
  }
  display.setTextSize(2);
  display.setCursor(0, 28);
  display.print(followsDrum ? "DRUM" : kDivisionNames[divisionId]);
}

void buildScaleMask(bool *mask, int16_t keyValue = -1, int16_t scaleValue = -1) {
  memset(mask, 0, 12);
  const uint8_t key = (keyValue < 0) ? settings.forceKey : static_cast<uint8_t>(keyValue);
  const uint8_t rawScale = (scaleValue < 0) ? settings.forceScale : static_cast<uint8_t>(scaleValue);
  const uint8_t scale = effectiveScaleForKeyMode(key, rawScale);
  if (key == 0 || scale == SCALE_OFF) return;
  for (uint8_t n = 0; n < 12; ++n) mask[n] = noteInScale(n, key, scale);
}

bool noteHeldInInputPc(uint8_t pc) {
  for (uint8_t n = 0; n < 128; ++n) {
    if (heldInputNotes[n] && (n % 12) == pc) return true;
  }
  return false;
}

bool isRootPc(uint8_t pc, int16_t keyValue = -1) {
  const int16_t key = (keyValue < 0) ? settings.forceKey : keyValue;
  return key > 0 && pc == rootPcFromKeyValue(static_cast<uint8_t>(key));
}

void drawGuitarView() {
  bool mask[12];
  const int16_t key = (ui.selectedSetting == SET_FORCE_KEY) ? effectiveSettingValue(SET_FORCE_KEY) : settings.forceKey;
  const int16_t scale = (ui.selectedSetting == SET_FORCE_SCALE) ? effectiveSettingValue(SET_FORCE_SCALE) : settings.forceScale;
  buildScaleMask(mask, key, scale);
  for (uint8_t s = 0; s < 6; ++s) display.drawLine(0, 4 + s * 7, 127, 4 + s * 7, SSD1306_WHITE);
  display.drawLine(5, 0, 5, 42, SSD1306_WHITE);
  for (uint8_t f = 0; f < 13; ++f) display.drawLine(6 + f * 10, 0, 6 + f * 10, 42, SSD1306_WHITE);
  const uint8_t markerFrets[5] = {3, 5, 7, 9, 12};
  for (uint8_t i = 0; i < 5; ++i) {
    const uint8_t fret = markerFrets[i];
    const int mx = 6 + (fret - 1) * 10 + 5;
    if (fret == 12) {
      display.drawPixel(mx, 14, SSD1306_WHITE);
      display.drawPixel(mx, 15, SSD1306_WHITE);
      display.drawPixel(mx, 28, SSD1306_WHITE);
      display.drawPixel(mx, 29, SSD1306_WHITE);
    } else {
      display.drawPixel(mx, 21, SSD1306_WHITE);
      display.drawPixel(mx, 22, SSD1306_WHITE);
    }
  }
  const uint8_t openPcs[6] = {4, 9, 2, 7, 11, 4};
  for (uint8_t string = 0; string < 6; ++string) {
    for (uint8_t fret = 0; fret < 12; ++fret) {
      const uint8_t pc = (openPcs[string] + fret) % 12;
      const int x = 1 + fret * 10;
      const int y = 4 + string * 7;
      if (mask[pc]) {
        if (isRootPc(pc, key)) {
          display.fillCircle(x, y, 2, SSD1306_BLACK);
          display.drawCircle(x, y, 2, SSD1306_WHITE);
          display.drawLine(x, y - 2, x, y + 2, SSD1306_WHITE);
        } else {
          display.fillCircle(x, y, 2, SSD1306_WHITE);
        }
      }
      if (noteHeldInInputPc(pc)) display.drawCircle(x, y, 4, SSD1306_WHITE);
    }
  }
}

void drawPianoView() {
  bool mask[12];
  const int16_t key = (ui.selectedSetting == SET_FORCE_KEY) ? effectiveSettingValue(SET_FORCE_KEY) : settings.forceKey;
  const int16_t scale = (ui.selectedSetting == SET_FORCE_SCALE) ? effectiveSettingValue(SET_FORCE_SCALE) : settings.forceScale;
  buildScaleMask(mask, key, scale);
  const uint8_t whiteMap[7] = {0, 2, 4, 5, 7, 9, 11};
  for (uint8_t i = 0; i < 7; ++i) {
    const int x = i * 18;
    display.drawRect(x, 0, 18, 42, SSD1306_WHITE);
    const int cx = x + 9;
    const int cy = 31;
    if (mask[whiteMap[i]]) display.fillCircle(cx, cy, 3, SSD1306_WHITE);
    if (noteHeldInInputPc(whiteMap[i])) display.drawCircle(cx, cy, 5, SSD1306_WHITE);
  }
  const uint8_t blackPcs[5] = {1, 3, 6, 8, 10};
  const uint8_t blackPos[5] = {13, 31, 67, 85, 103};
  for (uint8_t i = 0; i < 5; ++i) {
    display.fillRect(blackPos[i], 0, 10, 22, SSD1306_WHITE);
    const int cx = blackPos[i] + 5;
    const int cy = 14;
    if (mask[blackPcs[i]]) display.fillCircle(cx, cy, 3, SSD1306_BLACK);
    if (noteHeldInInputPc(blackPcs[i])) {
      display.drawCircle(cx, cy, 5, SSD1306_BLACK);
      display.drawCircle(cx, cy, 4, SSD1306_BLACK);
    }
  }
}

void drawPresetGrid(uint8_t slot) {
  for (uint8_t row = 0; row < 4; ++row) {
    for (uint8_t col = 0; col < 4; ++col) {
      const uint8_t idx = row * 4 + col;
      const int x = 24 + col * 20;
      const int y = 4 + row * 10;
      display.drawRect(x, y, 14, 8, SSD1306_WHITE);
      if (idx == slot) display.fillRect(x + 2, y + 2, 10, 4, SSD1306_WHITE);
    }
  }
  display.setTextSize(2);
  display.setCursor(0, 16);
  display.print(slot + 1);
}

bool submenuBackSelected() {
  if (ui.menuMode != MENU_EDIT) return false;
  switch (ui.selectedSetting) {
    case SET_ARP_MODE: return arpMenuUi.cursor == 10;
    case SET_LIVE_VELOCITY: return liveVelocityUi.cursor == 3;
    case SET_LIVE_NOTE_LENGTH: return liveNoteLengthUi.cursor == 3;
    case SET_STUTTER: return stutterUi.cursor == 4;
    case SET_ECHO: return echoUi.cursor == 6;
    case SET_QUICK_JUMP: return quickJumpUi.cursor == 4;
    case SET_DRUM_MAGIC: return drumMagicUi.cursor == 7;
    case SET_BASS_CH: return bassUi.cursor == 3;
    case SET_RND_RBN: return roundRobinMenuCursor == RND_RBN_BACK_SLOT;
    case SET_ROUTER:
      return routerEditStage == ROUTER_STAGE_LIST && routerMenuCursor == ROUTER_BACK_SLOT;
    case SET_DIV_NOTES: return divNotesCursor == DIV_NOTE_BACK_SLOT;
    case SET_MAP_CC:
      if (featuresUiStage == FEATURES_UI_GROUPS) return featuresGroupCursor == 3;
      {
        const uint8_t count = featuresUiStage == FEATURES_UI_KNOBS
            ? static_cast<uint8_t>(FEATURE_KNOB_COUNT)
            : static_cast<uint8_t>(FEATURE_BUTTON_COUNT);
        return featuresItemCursor >= count;
      }
    case SET_CC_MAP:
      return ccRemapUiStage == CC_REMAP_UI_LIST && ccRemapCursor > CC_REMAP_SLOT_COUNT;
    case SET_NOTE_CC:
      return noteCcUiStage == NOTE_CC_UI_LIST && noteCcCursor > NOTE_CC_SLOT_COUNT;
    case SET_FOUR_BUTTON:
      if (fourButtonUiStage == FOUR_BUTTON_UI_MAIN) return fourButtonUiCursor == 4;
      if (fourButtonUiStage == FOUR_BUTTON_UI_CUSTOM_LIST) {
        return fourButtonUiCursor == FOUR_BUTTON_CUSTOM_BACK_SLOT;
      }
      if (fourButtonUiStage == FOUR_BUTTON_UI_LOOPER) {
        return fourButtonUiCursor == FOUR_BUTTON_LOOPER_BACK_SLOT;
      }
      return fourButtonUiStage == FOUR_BUTTON_UI_CHORD &&
             fourButtonUiCursor == FOUR_BUTTON_CHORD_BACK_SLOT;
    case SET_LOOP_BARS: return looperSettingsUi.cursor == 8;
    case SET_PARAMETER_LOCK: return parameterLockUi.cursor == 2;
    case SET_MUTE_SOLO: return muteSoloCursor == LOOP_MIX_BACK_SLOT;
    case SET_CHORD: return chordUi.cursor == 5;
    case SET_FORCE_SCALE: return scaleUi.cursor == 13;
    case SET_LIVE_CC: return liveCcCursor == 2;
    case SET_GLOBAL: return globalUi.cursor == 9;
    default: return false;
  }
}

bool parameterEditActive() {
  if (ui.menuMode != MENU_EDIT) return false;
  switch (ui.selectedSetting) {
    case SET_ARP_MODE: return arpMenuUi.editing;
    case SET_LIVE_VELOCITY: return liveVelocityUi.editing;
    case SET_LIVE_NOTE_LENGTH: return liveNoteLengthUi.editing;
    case SET_STUTTER: return stutterUi.editing;
    case SET_ECHO: return echoUi.editing;
    case SET_QUICK_JUMP: return quickJumpUi.editing;
    case SET_DRUM_MAGIC: return drumMagicUi.editing;
    case SET_BASS_CH: return bassUi.editing;
    case SET_LOOP_BARS:
      return looperSettingsUi.editing && looperSettingsUi.cursor < 8;
    case SET_ROUTER: return routerEditStage != ROUTER_STAGE_LIST;
    case SET_CC_MAP: return ccRemapUiStage != CC_REMAP_UI_LIST;
    case SET_NOTE_CC:
      return noteCcUiStage != NOTE_CC_UI_LIST &&
             noteCcUiStage != NOTE_CC_UI_SLOT_ACTION;
    case SET_FOUR_BUTTON:
      return fourButtonUiStage == FOUR_BUTTON_UI_MODE ||
             (fourButtonUiStage >= FOUR_BUTTON_UI_CUSTOM_CHANNEL &&
              fourButtonUiStage <= FOUR_BUTTON_UI_CUSTOM_BEHAVIOR);
    case SET_MAP_CC: return featuresLearnActive;
    case SET_PARAMETER_LOCK: return parameterLockUi.editing;
    case SET_CHORD: return chordUi.editing;
    case SET_FORCE_SCALE: return scaleUi.editing;
    case SET_LIVE_CC: return liveCcEditing;
    case SET_GLOBAL: return globalUi.editing;
    // These are direct option lists: the encoder is choosing a submenu item,
    // not entering a separate numeric parameter editor.
    case SET_RND_RBN:
    case SET_DIV_NOTES:
    case SET_MUTE_SOLO:
      return false;
    default:
      // Plain top-level settings enter MENU_EDIT directly to change their
      // value, so they use the blue parameter dot.
      return true;
  }
}

void drawBackNavigationArrow() {
  const int downX = 116;
  const int cornerY = 26;
  const int leftX = 98;
  display.drawLine(downX, 10, downX, cornerY - 4, SSD1306_WHITE);
  display.drawLine(downX, cornerY - 4, downX - 3, cornerY - 7, SSD1306_WHITE);
  display.drawLine(downX, cornerY - 4, downX + 3, cornerY - 7, SSD1306_WHITE);
  display.drawLine(downX, cornerY, leftX, cornerY, SSD1306_WHITE);
  display.drawLine(leftX, cornerY, leftX + 6, cornerY - 5, SSD1306_WHITE);
  display.drawLine(leftX, cornerY, leftX + 6, cornerY + 5, SSD1306_WHITE);
}

void drawModeIndicator() {
  if (ui.menuMode == MENU_SELECT) return;
  const int x = 122;
  const int editY = SETTING_AREA_Y + SETTING_AREA_H - 7;
  const int selectY = MODE_INFO_Y + (MODE_INFO_H / 2);
  if (parameterEditActive()) display.fillCircle(x, editY, 3, SSD1306_WHITE);
  else display.fillCircle(x, selectY, 3, SSD1306_WHITE);
}

void drawLoopStatusIcon() {
  const bool armed = multitrackLooper.recordingArmed();
  const bool recording = multitrackLooper.recording();
  const bool overdubbing = multitrackLooper.overdubbing();
  const bool playing = multitrackLooper.playing();
  const bool hasData = multitrackLooper.hasAnyData();
  if (!armed && !recording && !overdubbing && !playing && !hasData) return;
  const int x = 116;
  const int y = 1;
  display.fillRect(x, y, 12, 12, SSD1306_BLACK);

  if (armed || recording || overdubbing) {
    display.drawCircle(x + 5, y + 6, 4, SSD1306_WHITE);
  } else if (playing) {
    display.drawTriangle(x + 2, y + 2, x + 2, y + 10, x + 10, y + 6, SSD1306_WHITE);
  } else if (hasData) {
    display.drawLine(x + 3, y + 2, x + 3, y + 10, SSD1306_WHITE);
    display.drawLine(x + 4, y + 2, x + 4, y + 10, SSD1306_WHITE);
    display.drawLine(x + 8, y + 2, x + 8, y + 10, SSD1306_WHITE);
    display.drawLine(x + 9, y + 2, x + 9, y + 10, SSD1306_WHITE);
  }
}

void drawChannelScreen(const __FlashStringHelper *, int channel, bool allowOff = false) {
  if (channel == DIRECT_CANCEL_CHANNEL) {
    display.setTextSize(2);
    display.setCursor(0, 15);
    display.print(F("CANCEL"));
    return;
  }
  if (allowOff && channel == 0) {
    display.setTextSize(3);
    display.setCursor(0, 11);
    display.print(F("OFF"));
  } else {
    display.setTextSize(3);
    display.setCursor(0, 11);
    display.print(F("CH "));
    display.print(channel);
  }
}

void drawBassScreen(uint8_t mode) {
  if (mode == 0) {
    display.setTextSize(3);
    display.setCursor(0, 11);
    display.print(F("OFF"));
    return;
  }
  const uint8_t channel = bassModeChannel(mode);
  const int8_t octaves = bassModeOctaveOffset(mode);
  display.setTextSize(1);
  display.setCursor(84, 2);
  if (octaves > 0) display.print(F("+1oct"));
  else if (octaves == 0) display.print(F("0oct"));
  else {
    display.print(octaves);
    display.print(F("oct"));
  }
  display.setTextSize(3);
  display.setCursor(0, 11);
  display.print(F("CH "));
  display.print(channel);
}

void drawCcChannelScreen(uint8_t channel) {
  if (channel == DIRECT_CANCEL_CC_CHANNEL) {
    display.setTextSize(2);
    display.setCursor(0, 15);
    display.print(F("CANCEL"));
    return;
  }
  if (channel == 17) {
    display.setTextSize(3);
    display.setCursor(0, 11);
    display.print(F("ALL3"));
    return;
  }
  display.setTextSize(3);
  display.setCursor(0, 11);
  display.print(F("CH "));
  display.print(channel);
}

String roundRobinChannelList() {
  if (settings.roundRobinMask == 0 && !roundRobinCh10To1Enabled() && !roundRobinCh10To2Enabled()) return "OFF";
  String out;
  for (uint8_t ch = 1; ch <= 16; ++ch) {
    if (!(settings.roundRobinMask & channelBit(ch))) continue;
    if (out.length()) out += ",";
    out += String(ch);
  }
  if (roundRobinCh10To1Enabled()) {
    if (out.length()) out += ",";
    out += F("CH10-1+");
  }
  if (roundRobinCh10To2Enabled()) {
    if (out.length()) out += ",";
    out += F("CH10-2+");
  }
  if (roundRobinRandomEnabled()) out += F(" RANDOM");
  return out;
}

void drawRoundRobinScreen(uint8_t cursor) {
  display.setTextColor(SSD1306_WHITE);
  if (ui.menuMode == MENU_SELECT) {
    display.setTextSize(1);
    display.setCursor(0, 10);
    display.print(roundRobinChannelList());
    return;
  }

  display.setTextSize(2);
  display.setCursor(0, 11);
  if (cursor < 16) {
    display.print((settings.roundRobinMask & channelBit(cursor + 1)) ? F("[x] ") : F("[ ] "));
    display.print(F("CH "));
    display.print(cursor + 1);
  } else if (cursor == RND_RBN_CH10_TO_1_SLOT) {
    display.print(roundRobinCh10To1Enabled() ? F("[x]CH10-1+") : F("[ ]CH10-1+"));
  } else if (cursor == RND_RBN_CH10_TO_2_SLOT) {
    display.print(roundRobinCh10To2Enabled() ? F("[x]CH10-2+") : F("[ ]CH10-2+"));
  } else if (cursor == RND_RBN_RANDOM_SLOT) {
    display.print(roundRobinRandomEnabled() ? F("[x] RANDOM") : F("[ ] RANDOM"));
  } else if (cursor == RND_RBN_CLEAR_SLOT) {
    display.print(F("CLEAR"));
  } else {
    display.print(F("BACK"));
  }
}

String twoDigit(uint8_t value) {
  String out;
  if (value < 10) out += '0';
  out += String(value);
  return out;
}

String routerEntryString(uint8_t idx) {
  if (idx >= 16) return "";
  String out = twoDigit(idx + 1);
  out += '>';
  out += twoDigit(settings.routerOutChannels[idx]);
  const int8_t transpose = settings.routerTranspose[idx];
  if (transpose > 0) out += '+';
  else if (transpose < 0) out += '-';
  else out += '=';
  const uint8_t amount = abs(transpose);
  if (amount < 10) out += '0';
  out += String(amount);
  return out;
}

String routerActiveList() {
  if (settings.routerActiveMask == 0) return "OFF";
  String out;
  for (uint8_t i = 0; i < 16; ++i) {
    if (!(settings.routerActiveMask & static_cast<uint16_t>(1U << i))) continue;
    if (out.length()) out += " ";
    out += routerEntryString(i);
  }
  return out;
}

void drawRouterScreen(uint8_t cursor) {
  display.setTextColor(SSD1306_WHITE);
  if (ui.menuMode == MENU_SELECT) {
    display.setTextSize(1);
    display.setCursor(0, 10);
    display.print(routerActiveList());
    return;
  }

  display.setTextSize(2);
  display.setCursor(0, 11);
  if (routerEditStage == ROUTER_STAGE_LIST) {
    if (cursor < 16) display.print(routerEntryString(cursor));
    else if (cursor == ROUTER_CLEAR_SLOT) display.print(F("CLEAR"));
    else display.print(F("BACK"));
  } else {
    display.print(routerEntryString(routerEditChannel));
    display.setTextSize(1);
    display.setCursor(0, 39);
    if (routerEditStage == ROUTER_STAGE_DEST) display.print(F("OUT CH"));
    else if (routerEditStage == ROUTER_STAGE_LOW_NOTE) {
      display.print(F("LOW NOTE "));
      display.print(firmware3Settings.routerLowNotes[routerEditChannel]);
    } else if (routerEditStage == ROUTER_STAGE_HIGH_NOTE) {
      display.print(F("HIGH NOTE "));
      display.print(firmware3Settings.routerHighNotes[routerEditChannel]);
    } else display.print(F("TRANSPOSE"));
  }
}

void drawDivNotesScreen(uint8_t cursor) {
  if (cursor == DIV_NOTE_PLUS_SLOT) {
    display.setTextSize(2);
    display.setCursor(0, 8);
    display.print(F("+HAT NOTE"));
    display.setTextSize(2);
    display.setCursor(0, 34);
    if (settings.divNotePlusNote == 0xFF) {
      display.print(F("BLANK"));
    } else {
      display.print(kNoteNames[settings.divNotePlusNote % 12]);
      display.print(settings.divNotePlusNote / 12);
      display.print(F(" "));
      display.print(settings.divNotePlusNote);
    }
    return;
  }
  if (cursor == DIV_NOTE_RESET_SLOT) {
    display.setTextSize(3);
    display.setCursor(0, 11);
    display.print(F("RESET"));
    return;
  }
  if (cursor == DIV_NOTE_BACK_SLOT) {
    display.setTextSize(3);
    display.setCursor(0, 11);
    display.print(F("BACK"));
    return;
  }
  const uint8_t divId = divNoteSlotToDivision(cursor);
  display.setTextSize(2);
  display.setCursor(0, 3);
  display.print(kDivisionNames[divId]);
  display.setTextSize(2);
  display.setCursor(0, 28);
  const uint8_t mapCh = settings.divNoteChannels[cursor];
  const uint8_t mapNote = settings.divNoteNotes[cursor];
  const uint8_t bindingKind = featureControls.drumRollKinds[cursor];
  if (bindingKind == TRIGGER_BINDING_OFF || mapCh == 0 || mapNote == 0xFF) {
    display.setTextSize(1);
    display.setCursor(0, 32);
    display.print(F("LEARN NOTE/CC"));
  } else if (bindingKind == TRIGGER_BINDING_CC) {
    display.print(F("CH "));
    display.print(mapCh);
    display.print(F(" CC"));
    display.print(mapNote);
  } else {
    display.print(F("CH "));
    display.print(mapCh);
    display.print(F(" "));
    display.print(kNoteNames[mapNote % 12]);
    display.print(mapNote / 12);
  }
}

String liveTargetName(uint8_t target) {
  if (target == SELECTD_LIVE_TARGET) return String("SELECTD");
  return target == 0 ? String("MAIN") : String("LOOP ") + String(target);
}

String featureKnobName(uint8_t id) {
  struct BlockName { uint8_t base; const char *name; };
  static const BlockName blocks[] = {
    {FEATURE_KNOB_VELOCITY_BASE, "VELOCITY"},
    {FEATURE_KNOB_NOTE_LENGTH_BASE, "NOTELENGT"},
    {FEATURE_KNOB_STUTTER_BASE, "STUTTER"},
    {FEATURE_KNOB_ECHO_WET_BASE, "ECHO WET"},
    {FEATURE_KNOB_ECHO_LENGTH_BASE, "ECHO LENGTH"},
    {FEATURE_KNOB_ECHO_DELAY_BASE, "ECHO DELAY"},
    {FEATURE_KNOB_ECHO_DRIFT_BASE, "ECHO DRIFT"}
  };
  for (const BlockName &block : blocks) {
    if (id >= block.base && id < block.base + LIVE_TARGET_COUNT) {
      return liveTargetName(id - block.base) + " " + block.name;
    }
  }
  if (id == FEATURE_KNOB_ARP_DIVISION) return "ARP DIVISION";
  if (id == FEATURE_KNOB_DRUM_DIVISION) return "DRUM DIVISION";
  if (id == FEATURE_KNOB_QUICK_JUMP_INPUT) return "QJ INPUT CH";
  if (id == FEATURE_KNOB_QUICK_JUMP_OUTPUT) return "QJ OUTPUT CH";
  if (id == FEATURE_KNOB_BPM) return "BPM";
  if (id == FEATURE_KNOB_SWING) return "SWING";
  if (id == FEATURE_KNOB_ARP_MODE) return "ARP MODE";
  if (id == FEATURE_KNOB_ARP_VELOCITY) return "ARP VELOCITY";
  if (id == FEATURE_KNOB_ARP_LENGTH) return "ARP LENGTH";
  if (id == FEATURE_KNOB_ARP_OCTAVES) return "ARP OCTAVES";
  if (id == FEATURE_KNOB_LOOP_TRACK) return "LOOP TRACK";
  if (id == FEATURE_KNOB_LOOP_LENGTH) return "LOOP LENGTH";
  return "KNOB";
}

String featureButtonName(uint8_t id) {
  uint8_t target = featureTargetFromBlock(id, FEATURE_BUTTON_VELOCITY_BASE);
  if (target < LIVE_TARGET_COUNT) return liveTargetName(target) + " VELOCITY";
  target = featureTargetFromBlock(id, FEATURE_BUTTON_NOTE_LENGTH_BASE);
  if (target < LIVE_TARGET_COUNT) return liveTargetName(target) + " NOTELENGT";
  target = featureTargetFromBlock(id, FEATURE_BUTTON_STUTTER_BASE);
  if (target < LIVE_TARGET_COUNT) return liveTargetName(target) + " STUTTER";
  target = featureTargetFromBlock(id, FEATURE_BUTTON_ECHO_BASE);
  if (target < LIVE_TARGET_COUNT) return liveTargetName(target) + " ECHO";
  if (id == FEATURE_BUTTON_LOOP_RECORD) return "LOOP RECORD/ARM";
  if (id == FEATURE_BUTTON_LOOP_PLAY_STOP) return "LOOP PLAY/STOP";
  if (id == FEATURE_BUTTON_LOOP_CLEAR_UNDO) return "LOOP CLEAR/UNDO";
  if (id == FEATURE_BUTTON_LOOP_COMBO) return "LOOP COMBO";
  if (id >= FEATURE_BUTTON_TRACK_SELECT_BASE &&
      id < FEATURE_BUTTON_TRACK_SELECT_BASE + arpnmidi3::kLoopTrackCount) {
    return String("SELECT TRACK ") + String(id - FEATURE_BUTTON_TRACK_SELECT_BASE + 1U);
  }
  if (id >= FEATURE_BUTTON_TRACK_MUTE_BASE &&
      id < FEATURE_BUTTON_TRACK_MUTE_BASE + arpnmidi3::kLoopTrackCount) {
    return String("MUTE TRACK ") + String(id - FEATURE_BUTTON_TRACK_MUTE_BASE + 1U);
  }
  if (id >= FEATURE_BUTTON_TRACK_SOLO_BASE &&
      id < FEATURE_BUTTON_TRACK_SOLO_BASE + arpnmidi3::kLoopTrackCount) {
    return String("SOLO TRACK ") + String(id - FEATURE_BUTTON_TRACK_SOLO_BASE + 1U);
  }
  if (id == FEATURE_BUTTON_QUICK_JUMP) return "QUICK JUMP";
  if (id >= FEATURE_BUTTON_STUTTER_DIV_BASE && id < FEATURE_BUTTON_STUTTER_DIV_END) {
    static const char *const divisions[STUTTER_BUTTON_DIVISION_COUNT] = {
      "1/2", "1/4", "1/8", "1/16", "1/32", "1/64"
    };
    const uint8_t offset = id - FEATURE_BUTTON_STUTTER_DIV_BASE;
    return liveTargetName(offset / STUTTER_BUTTON_DIVISION_COUNT) + " STUT " +
        divisions[offset % STUTTER_BUTTON_DIVISION_COUNT];
  }
  if (id == FEATURE_BUTTON_ARP_RETRIGGER) return "ARP RETRIGGER SYNC";
  if (id == FEATURE_BUTTON_ARP_NOTE_ORDER) return "ARP AS-PLAYED";
  if (id == FEATURE_BUTTON_DRUM_MAGIC) return "DRUMROLL";
  if (id == FEATURE_BUTTON_DRUM_AFTERTOUCH_VELOCITY) return "DRUM AT>VELOCITY";
  if (id == FEATURE_BUTTON_CHORD) return "CHORD";
  if (id == FEATURE_BUTTON_LOOP_AUTO_REC) return "LOOP AUTO REC";
  if (id == FEATURE_BUTTON_LOOP_TIME_TRAVEL) return "LOOP TIME TRAV";
  if (id == FEATURE_BUTTON_LOOP_RECORD_CC) return "LOOP RECORD CC";
  if (id == FEATURE_BUTTON_LOOP_MIDI_TRANSPORT) return "LOOP MIDI TRANSPORT";
  if (id == FEATURE_BUTTON_CLOCK_INPUT) return "CLOCK INPUT";
  if (id == FEATURE_BUTTON_CLOCK_OUTPUT) return "CLOCK OUTPUT";
  if (id == FEATURE_BUTTON_PANIC) return "PANIC";
  return "BUTTON";
}

void drawFeaturesScreen() {
  if (featuresUiStage == FEATURES_UI_GROUPS) {
    static const char *const groups[] = {"CC KNOBS", "CC BUTTONS", "CLEAR", "BACK"};
    display.setTextSize(featuresGroupCursor < 2 ? 2 : 3);
    display.setCursor(0, 11);
    display.print(groups[featuresGroupCursor]);
    return;
  }
  const uint8_t count = featuresUiStage == FEATURES_UI_KNOBS
      ? static_cast<uint8_t>(FEATURE_KNOB_COUNT)
      : static_cast<uint8_t>(FEATURE_BUTTON_COUNT);
  if (featuresItemCursor >= count) {
    display.setTextSize(3);
    display.setCursor(0, 11);
    display.print(F("BACK"));
    return;
  }
  if (!featuresItemOpen) {
    // A compact scrolling list: every row is just the feature's name and a
    // dot if it already has a mapping, no CH/CC text, so as many rows as
    // possible fit at once instead of stepping through features one at a
    // time. Turning the encoder moves the highlighted row and scrolls the
    // window to keep it visible; selecting a row opens the same detail view
    // this screen has always shown below, and turning again closes it back
    // to the list, scrolled to wherever that turn lands.
    constexpr uint8_t kVisibleRows = 6;
    constexpr uint8_t kRowHeight = 8;
    uint8_t scrollTop = featuresItemCursor >= kVisibleRows
        ? featuresItemCursor - kVisibleRows + 1 : 0;
    if (count > kVisibleRows && scrollTop > count - kVisibleRows) {
      scrollTop = count - kVisibleRows;
    }
    display.setTextSize(1);
    for (uint8_t row = 0; row < kVisibleRows; ++row) {
      const uint8_t index = scrollTop + row;
      if (index >= count) break;
      const int y = row * kRowHeight;
      display.setCursor(0, y);
      display.print(index == featuresItemCursor ? F(">") : F(" "));
      display.print(featuresUiStage == FEATURES_UI_KNOBS
          ? featureKnobName(index) : featureButtonName(index));
      bool mapped;
      if (featuresUiStage == FEATURES_UI_KNOBS) {
        const FeatureKnobBinding &binding = featureControls.knobs[index];
        mapped = binding.channel != 0 && binding.cc <= 127;
      } else {
        const FeatureButtonBinding &binding = featureControls.buttons[index];
        mapped = binding.kind != TRIGGER_BINDING_OFF && binding.channel != 0;
      }
      if (mapped) display.fillCircle(124, y + 3, 2, SSD1306_WHITE);
    }
    return;
  }
  const String name = featuresUiStage == FEATURES_UI_KNOBS
      ? featureKnobName(featuresItemCursor) : featureButtonName(featuresItemCursor);
  display.setTextSize(1);
  display.setCursor(0, 6);
  display.println(name);
  display.setCursor(0, 23);
  if (featuresLearnActive) {
    display.print(featuresUiStage == FEATURES_UI_KNOBS ? F("MOVE A CC") : F("MOVE CC / PLAY NOTE"));
    return;
  }
  if (featuresUiStage == FEATURES_UI_KNOBS) {
    const FeatureKnobBinding &binding = featureControls.knobs[featuresItemCursor];
    if (binding.channel == 0 || binding.cc > 127) display.print(F("UNMAPPED"));
    else {
      display.print(F("CH ")); display.print(binding.channel);
      display.print(F(" CC ")); display.print(binding.cc);
    }
  } else {
    const FeatureButtonBinding &binding = featureControls.buttons[featuresItemCursor];
    if (binding.kind == TRIGGER_BINDING_OFF || binding.channel == 0) {
      display.print(F("UNMAPPED"));
    } else {
      display.print(F("CH ")); display.print(binding.channel);
      display.print(binding.kind == TRIGGER_BINDING_CC ? F(" CC ") : F(" NOTE "));
      display.print(binding.number);
    }
  }
}

void drawCcRemapScreen() {
  if (ccRemapUiStage == CC_REMAP_UI_LIST) {
    if (ccRemapCursor == CC_REMAP_SLOT_COUNT) {
      display.setTextSize(3); display.setCursor(0, 11); display.print(F("CLEAR")); return;
    }
    if (ccRemapCursor > CC_REMAP_SLOT_COUNT) {
      display.setTextSize(3); display.setCursor(0, 11); display.print(F("BACK")); return;
    }
    const CcRemapEntry &entry = featureControls.ccRemaps[ccRemapCursor];
    display.setTextSize(1); display.setCursor(0, 6);
    display.print(F("SLOT ")); display.print(ccRemapCursor + 1U);
    display.setCursor(0, 23);
    if (entry.inputCc > 127) display.print(F("UNMAPPED"));
    else {
      display.print(F("CC ")); display.print(entry.inputCc);
      display.print(F(" > CH ")); display.print(entry.outputChannel);
      display.print(F(" CC ")); display.print(entry.outputCc);
    }
    return;
  }
  const CcRemapEntry &entry = featureControls.ccRemaps[ccRemapCursor];
  display.setTextSize(1); display.setCursor(0, 8);
  if (ccRemapUiStage == CC_REMAP_UI_INPUT) {
    display.print(F("INPUT CC: "));
    if (entry.inputCc > 127) display.print(F("OFF")); else display.print(entry.inputCc);
    display.setCursor(0, 34);
    if (ccRemapLearnActive) display.print(F("MOVE MAIN CC"));
  } else if (ccRemapUiStage == CC_REMAP_UI_OUTPUT_CHANNEL) {
    display.print(F("OUTPUT CH: ")); display.print(entry.outputChannel);
  } else {
    display.print(F("OUTPUT CC: ")); display.print(entry.outputCc);
  }
}

void drawNoteCcScreen() {
  if (noteCcUiStage == NOTE_CC_UI_LIST) {
    if (noteCcCursor == NOTE_CC_SLOT_COUNT) {
      display.setTextSize(3); display.setCursor(0, 11); display.print(F("CLEAR")); return;
    }
    if (noteCcCursor > NOTE_CC_SLOT_COUNT) {
      display.setTextSize(3); display.setCursor(0, 11); display.print(F("BACK")); return;
    }
    const NoteCcMapEntry &entry = featureControls.noteCcMaps[noteCcCursor];
    display.setTextSize(1); display.setCursor(0, 6);
    display.print(F("SLOT ")); display.print(noteCcCursor + 1U);
    display.setCursor(0, 23);
    if (entry.inputChannel == 0 || entry.inputNote > 127) display.print(F("UNMAPPED"));
    else {
      display.print(F("CH ")); display.print(entry.inputChannel);
      display.print(F(" N ")); display.print(entry.inputNote);
      display.print(F(" > ")); display.print(entry.outputChannel);
      display.print(F(":")); display.print(entry.outputCc);
    }
    return;
  }
  if (noteCcUiStage == NOTE_CC_UI_SLOT_ACTION) {
    static const char *const actions[] = {"EDIT", "CLEAR", "CANCEL"};
    display.setTextSize(1); display.setCursor(0, 5);
    display.print(F("SLOT ")); display.print(noteCcCursor + 1U);
    display.setTextSize(actions[noteCcSlotActionCursor][0] == 'C' ? 2 : 3);
    display.setCursor(0, actions[noteCcSlotActionCursor][0] == 'C' ? 18 : 11);
    display.print(actions[noteCcSlotActionCursor]);
    return;
  }
  const NoteCcMapEntry &entry = featureControls.noteCcMaps[noteCcCursor];
  display.setTextSize(2); display.setCursor(0, 11);
  if (cancelSelectedFor(SET_NOTE_CC)) {
    display.print(F("CANCEL"));
    return;
  }
  if (noteCcUiStage == NOTE_CC_UI_INPUT_CHANNEL) {
    display.print(F("CH ")); display.print(max<uint8_t>(1, entry.inputChannel));
    if (noteCcLearnActive) {
      display.setTextSize(1); display.setCursor(0, 34); display.print(F("PLAY NOTE TO LEARN"));
    }
  } else if (noteCcUiStage == NOTE_CC_UI_INPUT_NOTE) {
    display.print(F("N ")); display.print(entry.inputNote <= 127 ? entry.inputNote : 0);
    if (noteCcLearnActive) {
      display.setTextSize(1); display.setCursor(0, 34); display.print(F("PLAY NOTE TO LEARN"));
    }
  } else if (noteCcUiStage == NOTE_CC_UI_OUTPUT_CHANNEL) {
    display.print(F("CH ")); display.print(entry.outputChannel);
    if (noteCcLearnActive) {
      display.setTextSize(1); display.setCursor(0, 34); display.print(F("MOVE CC TO LEARN"));
    }
  } else if (noteCcUiStage == NOTE_CC_UI_OUTPUT_CC) {
    display.print(F("CC ")); display.print(entry.outputCc);
    if (noteCcLearnActive) {
      display.setTextSize(1); display.setCursor(0, 34); display.print(F("MOVE CC TO LEARN"));
    }
  } else {
    display.setCursor(0, 18);
    display.print(entry.behavior == NOTE_CC_TOGGLE ? F("TOGGLE") : F("MOMENTARY"));
  }
}

void drawFourButtonScreen() {
  display.setTextSize(1);
  display.setCursor(0, 8);
  if (ui.menuMode == MENU_SELECT) {
    static const char *const modes[] = {"CUSTOM", "LOOPER", "CHORD MEM"};
    display.setTextSize(2);
    display.print(modes[featureControls.fourButtonMode]);
    return;
  }
  if (fourButtonUiStage == FOUR_BUTTON_UI_MAIN) {
    static const char *const items[] = {"MODE", "CUSTOM", "LOOPER", "CHORD MEM", "BACK"};
    display.setTextSize(2);
    display.print(items[fourButtonUiCursor]);
    return;
  }
  if (fourButtonUiStage == FOUR_BUTTON_UI_MODE) {
    static const char *const modes[] = {"CUSTOM", "LOOPER", "CHORD MEM"};
    display.print(F("MODE"));
    display.setTextSize(2); display.setCursor(0, 31);
    display.print(modes[featureControls.fourButtonMode]);
    return;
  }
  if (fourButtonUiStage == FOUR_BUTTON_UI_CUSTOM_LIST) {
    if (fourButtonUiCursor == FOUR_BUTTON_CUSTOM_DONE_SLOT) {
      display.setTextSize(3); display.print(F("DONE"));
    } else if (fourButtonUiCursor == FOUR_BUTTON_CUSTOM_BACK_SLOT) {
      display.setTextSize(3); display.print(F("BACK"));
    } else {
      display.setTextSize(2); display.print(F("BUTTON ")); display.print(fourButtonUiCursor + 1U);
    }
    return;
  }
  if (fourButtonUiStage >= FOUR_BUTTON_UI_CUSTOM_CHANNEL &&
      fourButtonUiStage <= FOUR_BUTTON_UI_CUSTOM_BEHAVIOR) {
    const CustomButtonConfig &button = featureControls.customButtons[fourButtonEditButton];
    display.print(F("BUTTON ")); display.print(fourButtonEditButton + 1U); display.print(F("  "));
    if (fourButtonUiStage == FOUR_BUTTON_UI_CUSTOM_CHANNEL) {
      display.print(F("CHANNEL")); display.setCursor(0, 34); display.setTextSize(2); display.print(button.channel);
    } else if (fourButtonUiStage == FOUR_BUTTON_UI_CUSTOM_KIND) {
      display.print(F("TYPE")); display.setCursor(0, 34); display.setTextSize(2);
      display.print(button.kind == TRIGGER_BINDING_CC ? F("CC") : F("NOTE"));
    } else if (fourButtonUiStage == FOUR_BUTTON_UI_CUSTOM_NUMBER) {
      display.print(button.kind == TRIGGER_BINDING_CC ? F("CC NUMBER") : F("NOTE NUMBER"));
      display.setCursor(0, 34);
      if (fourButtonLearnActive) display.print(F("MOVE/PLAY TO LEARN"));
      else { display.setTextSize(2); display.print(button.number); }
    } else {
      static const char *const behaviors[] = {"MOMENTARY", "LATCH", "FLAPPY BIRD"};
      display.print(F("BEHAVIOR")); display.setCursor(0, 34); display.setTextSize(2);
      display.print(behaviors[button.behavior]);
    }
    return;
  }
  if (fourButtonUiStage == FOUR_BUTTON_UI_LOOPER) {
    static const char *const names[] = {
      "SELECT TRK", "ARM", "MUTE", "SOLO", "DELETE", "UNDO", "DONE", "BACK"
    };
    static constexpr uint8_t masks[] = {
      LOOPER_BUTTON_SELECT, LOOPER_BUTTON_ARM, LOOPER_BUTTON_MUTE,
      LOOPER_BUTTON_SOLO, LOOPER_BUTTON_DELETE, LOOPER_BUTTON_UNDO
    };
    display.setTextSize(2);
    display.print(names[fourButtonUiCursor]);
    if (fourButtonUiCursor < 6) {
      display.setCursor(0, 31);
      display.print((featureControls.looperButtonActions & masks[fourButtonUiCursor])
          ? F("ON") : F("OFF"));
    }
    return;
  }
  static const char *const chordItems[] = {"LEARN", "CLEAR", "DONE", "BACK"};
  display.setTextSize(2); display.print(chordItems[fourButtonUiCursor]);
  if ((chordLearnArmed && fourButtonUiCursor == 0) ||
      (chordClearArmed && fourButtonUiCursor == 1)) {
    display.setTextSize(1); display.setCursor(0, 39); display.print(F("PRESS BUTTON 1-4"));
  }
}

void drawMuteSoloScreen() {
  display.setTextSize(1);
  const bool editingLoopMix = ui.menuMode == MENU_EDIT;
  const uint8_t highlightedTrack = editingLoopMix
      ? muteSoloCursor
      : multitrackLooper.selectedTrack();
  for (uint8_t track = 0; track < arpnmidi3::kLoopTrackCount; ++track) {
    const int x = track * 32;
    const int y = 1;
    const arpnmidi3::LoopTrackState &state = multitrackLooper.track(track);
    display.drawRect(x, y, 31, 17, SSD1306_WHITE);
    if (state.solo) {
      display.fillRect(x + 1, y + 1, 29, 15, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
    } else if (state.muted) {
      for (uint8_t px = x + 3; px < x + 28; px += 4) {
        display.drawPixel(px, y + 8, SSD1306_WHITE);
        display.drawPixel(px + 1, y + 9, SSD1306_WHITE);
      }
    }
    display.setCursor(x + 13, y + 5);
    display.print(track + 1U);
    display.setTextColor(SSD1306_WHITE);
    // A cleared track is struck through, and the track a record is aimed at
    // carries a dot, so the write target is never left to guesswork.
    if (state.hidden && state.count > 0) {
      display.drawLine(x + 5, y + 13, x + 26, y + 13, SSD1306_WHITE);
    }
    if ((multitrackLooper.recordingArmed() || multitrackLooper.recording()) &&
        multitrackLooper.recordingTrack() == track) {
      display.fillRect(x + 25, y + 3, 3, 3, SSD1306_WHITE);
    }
    if (highlightedTrack == track) display.drawRect(x + 1, y + 1, 29, 15, SSD1306_WHITE);
  }
  if (!editingLoopMix) return;
  static const char *const actions[LOOP_MIX_MODE_COUNT] = {"SOLO", "MUTE", "CLEAR", "ARM"};
  static const uint8_t actionInset[LOOP_MIX_MODE_COUNT] = {4, 4, 1, 7};
  for (uint8_t i = 0; i < LOOP_MIX_MODE_COUNT; ++i) {
    const int x = i * 32;
    const int y = 28;
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(x + actionInset[i], y);
    display.print(actions[i]);
    // The underline marks the mode in force, the box marks where the cursor is.
    if (loopMixMode == i) display.drawLine(x + 2, y + 9, x + 29, y + 9, SSD1306_WHITE);
    if (muteSoloCursor == LOOP_MIX_MODE_BASE + i) {
      display.drawRect(x, y - 3, 31, 13, SSD1306_WHITE);
    }
  }
}

void drawSubmenuField(const String &, const String &value, bool) {
  if (!value.length()) return;
  if (value.length() <= 7) {
    display.setTextSize(3);
    display.setCursor(0, 11);
  } else if (value.length() <= 11) {
    display.setTextSize(2);
    display.setCursor(0, 15);
  } else {
    display.setTextSize(1);
    display.setCursor(0, 19);
  }
  display.print(value);
}

String onOff(bool enabled) { return enabled ? String("ON") : String("OFF"); }

String customArpLengthName(uint8_t selection) {
  static const char *const names[] = {"1/4 BAR", "1/2 BAR", "1 BAR", "2 BARS", "4 BARS", "8 BARS"};
  return names[clampU8(selection, 0, 5)];
}

void drawArpMenuScreen() {
  static const char *const names[] = {
    "MODE", "DIVISION", "ARP VEL", "ARP LENGTH", "OCTAVES",
    "RETRIG", "ORDER", "LENGTH", "LEARN ARP", "CLEAR ARP", "BACK"
  };
  const uint8_t cursor = arpMenuUi.cursor;
  if (ui.menuMode == MENU_SELECT) {
    drawSubmenuField("", String(kArpSelectionNames[settings.arpMode]) + " " +
        (settings.division == ARP_DIVISION_FOLLOW_DRUM
            ? String("DRUM") : String(kDivisionNames[settings.division])), false);
    return;
  }
  if (arpMenuUi.editing && cancelSelectedFor(SET_ARP_MODE)) {
    drawSubmenuField(names[cursor], "CANCEL", true);
    return;
  }
  if (cursor == 2) {
    drawNamedBarValue("", map(settings.arpVelocity, 0, 127, 0, 100), 100,
                      String(settings.arpVelocity));
    return;
  }
  if (cursor == 3) {
    drawNamedBarValue("", settings.arpLengthPct, 100,
                      String(settings.arpLengthPct) + "%");
    return;
  }
  String value;
  if (cursor == 0) value = kArpSelectionNames[settings.arpMode];
  else if (cursor == 1) value = settings.division == ARP_DIVISION_FOLLOW_DRUM
      ? "DRUM" : kDivisionNames[settings.division];
  else if (cursor == 2) value = String(settings.arpVelocity);
  else if (cursor == 3) value = String(settings.arpLengthPct) + "%";
  else if (cursor == 4) value = String(firmware3Settings.arpOctaves);
  else if (cursor == 5) value = firmware3Settings.arpRetriggerSync ? "CLOCK SYNC" : "KEY PRESS";
  else if (cursor == 6) value = firmware3Settings.arpNoteOrder ? "AS-PLAYED" : "IN ORDER";
  else if (cursor == 7) value = customArpLengthName(firmware3Settings.customArpLength);
  else if (cursor == 8) {
    if (customArpLearning) value = customArpWaitingForFirstNote
        ? "WAIT NOTE" : String("REC ") + String(customArpPattern.count);
    else value = String(customArpPattern.count) + " EVENTS";
  } else if (cursor == 9) value = String(customArpPattern.count) + " EVENTS";
  drawSubmenuField(names[cursor], value, arpMenuUi.editing);
}

void drawLiveVelocityScreen() {
  static const char *const names[] = {"TARGET", "ON/OFF", "VELOCITY", "BACK"};
  const LiveTargetSettings &target = firmware3Settings.liveTargets[liveVelocityTarget];
  if (ui.menuMode == MENU_SELECT || liveVelocityUi.cursor == 2) {
    drawNamedBarValue("", target.velocityPercent, 200,
        liveTargetName(liveVelocityTarget) + " " + String(target.velocityPercent) + "%", 100);
    return;
  }
  String value;
  if (liveVelocityUi.cursor == 0) value = liveTargetName(liveVelocityTarget);
  else if (liveVelocityUi.cursor == 1) value = onOff(target.velocityEnabled);
  drawSubmenuField(names[liveVelocityUi.cursor], value, liveVelocityUi.editing);
}

void drawLiveNoteLengthScreen() {
  static const char *const names[] = {"TARGET", "ON/OFF", "NOTELENGT", "BACK"};
  const LiveTargetSettings &target = firmware3Settings.liveTargets[liveNoteLengthTarget];
  if (ui.menuMode == MENU_SELECT || liveNoteLengthUi.cursor == 2) {
    drawNamedBarValue("", target.noteLengthPercent, 200,
        liveTargetName(liveNoteLengthTarget) + " " + String(target.noteLengthPercent) + "%", 100);
    return;
  }
  String value;
  if (liveNoteLengthUi.cursor == 0) value = liveTargetName(liveNoteLengthTarget);
  else if (liveNoteLengthUi.cursor == 1) value = onOff(target.noteLengthEnabled);
  drawSubmenuField(names[liveNoteLengthUi.cursor], value, liveNoteLengthUi.editing);
}

void drawStutterScreen() {
  static const char *const names[] = {"DIVISION", "ON/OFF", "TIMEOUT", "TARGET", "BACK"};
  const LiveTargetSettings &target = firmware3Settings.liveTargets[stutterTarget];
  if (ui.menuMode == MENU_SELECT) {
    drawSubmenuField("", liveTargetName(stutterTarget) + " " +
        (target.stutterEnabled
            ? compactLengthSelectionName(target.stutterLengthSelection) : String("OFF")), false);
    return;
  }
  String value;
  if (stutterUi.editing && cancelSelectedFor(SET_STUTTER)) value = "CANCEL";
  else if (stutterUi.cursor == 0) value = lengthSelectionName(target.stutterLengthSelection);
  else if (stutterUi.cursor == 1) value = onOff(target.stutterEnabled);
  else if (stutterUi.cursor == 2) value = String(firmware3Settings.stutterTimeoutBars) + " BARS";
  else if (stutterUi.cursor == 3) value = liveTargetName(stutterTarget);
  drawSubmenuField(names[stutterUi.cursor], value, stutterUi.editing);
}

void drawEchoScreen() {
  static const char *const names[] = {"LENGTH", "ON/OFF", "WET", "DELAY", "DRIFT", "TARGET", "BACK"};
  const LiveTargetSettings &echo = firmware3Settings.liveTargets[echoTarget];
  if (ui.menuMode == MENU_SELECT) {
    drawSubmenuField("", liveTargetName(echoTarget) + " " +
        (echo.echoEnabled ? String("WET ") + String(echo.echoWet) + "%" : String("OFF")), false);
    return;
  }
  if (echoUi.editing && cancelSelectedFor(SET_ECHO)) {
    drawSubmenuField(names[echoUi.cursor], "CANCEL", true);
    return;
  }
  if (echoUi.cursor == 2) {
    drawNamedBarValue("", echo.echoWet, 100, String(echo.echoWet) + "%");
    return;
  }
  String value;
  if (echoUi.cursor == 0) value = lengthSelectionName(echo.echoLength);
  else if (echoUi.cursor == 1) value = onOff(echo.echoEnabled);
  else if (echoUi.cursor == 3) value = lengthSelectionName(echo.echoDelay);
  else if (echoUi.cursor == 4) value = String(echo.echoDrift);
  else if (echoUi.cursor == 5) value = liveTargetName(echoTarget);
  drawSubmenuField(names[echoUi.cursor], value, echoUi.editing);
}

void drawQuickJumpScreen() {
  static const char *const names[] = {"INPUT", "OUTPUT", "ON/OFF", "HOLD", "BACK"};
  if (ui.menuMode == MENU_SELECT) {
    drawSubmenuField("", firmware3Settings.quickJumpEnabled
        ? String(firmware3Settings.quickJumpInputChannel) + " > " + String(firmware3Settings.quickJumpOutputChannel)
        : String("OFF"), false);
    return;
  }
  String value;
  if (quickJumpUi.editing && cancelSelectedFor(SET_QUICK_JUMP)) value = "CANCEL";
  else if (quickJumpUi.cursor == 0) value = String("CH ") + String(firmware3Settings.quickJumpInputChannel);
  else if (quickJumpUi.cursor == 1) value = String("CH ") + String(firmware3Settings.quickJumpOutputChannel);
  else if (quickJumpUi.cursor == 2) value = onOff(firmware3Settings.quickJumpEnabled);
  else if (quickJumpUi.cursor == 3) value = onOff(firmware3Settings.quickJumpHold);
  drawSubmenuField(names[quickJumpUi.cursor], value, quickJumpUi.editing);
}

void drawBassMenuScreen() {
  if (ui.menuMode == MENU_SELECT) {
    drawBassScreen(settings.bassMode);
    display.setTextSize(1);
    display.setCursor(82, 39);
    display.print(F("MAX "));
    display.print(firmware3Settings.bassHighestNote);
    return;
  }
  static const char *const names[] = {"CH", "OCTAVE", "HIGH NOTE", "BACK"};
  String value;
  if (bassUi.editing && cancelSelectedFor(SET_BASS_CH)) value = "CANCEL";
  else if (bassUi.cursor == 0) {
    const uint8_t channel = bassModeChannel(settings.bassMode);
    value = channel == 0 ? String(F("OFF")) : String("CH ") + String(channel);
  } else if (bassUi.cursor == 1) {
    const int8_t octave = bassModeOctaveOffset(settings.bassMode);
    value = octave > 0 ? String("+") + String(octave) : String(octave);
  } else if (bassUi.cursor == 2) value = String(firmware3Settings.bassHighestNote);
  drawSubmenuField(names[bassUi.cursor], value, bassUi.editing);
}

void drawMonoRetrigScreen(uint8_t channel) {
  if (ui.menuMode == MENU_SELECT) {
    display.setTextSize(1);
    display.setCursor(0, 5);
    display.print(F("Retrig last key"));
    display.setTextSize(2);
    display.setCursor(0, 29);
    display.print(channel == 0 ? F("OFF") : F("CH "));
    if (channel != 0) display.print(channel);
    return;
  }
  drawChannelScreen(F("MONO"), channel, true);
}

void drawScaleMenuScreen() {
  if (ui.menuMode == MENU_SELECT) {
    if (effectiveSettingValue(SET_GUITAR_PIANO) == 0) drawGuitarView();
    else drawPianoView();
    return;
  }
  if (scaleUi.cursor == 0) {
    drawSubmenuField("SCALE TYPE",
                     cancelSelectedFor(SET_FORCE_SCALE)
                         ? String("CANCEL")
                         : String(kForceScaleNames[settings.forceScale]),
                     scaleUi.editing);
    return;
  }
  if (scaleUi.cursor == 13) {
    drawSubmenuField("BACK", "", false);
    return;
  }
  const uint8_t semitones = scaleUi.cursor - 1U;
  const bool included = (firmware3Settings.userScaleMask & (1U << semitones)) != 0;
  const String name = semitones == 0 ? String("USER ROOT")
                                     : String("INTVL +") + String(semitones);
  drawSubmenuField(name, included ? "INCLUDED" : "OMITTED", false);
}

void drawPanicScreen() {
  const bool confirmed = static_cast<int32_t>(panicConfirmedUntilMs - millis()) > 0;
  const bool overload = secondaryTxDropped || secondaryTxCriticalDropped ||
      multitrackLooper.overflowCount() || parameterLockOverflowCount ||
      loopCcPruneOverflowCount;
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  if (confirmed) display.print(F("PANIC SENT  "));
  display.print(F("D")); display.print(dinIncomingMessageCount);
  display.print(F(" U")); display.print(usbIncomingMessageCount);

  display.setCursor(0, 8);
  display.print(F("LAST "));
  if (lastIncomingSource == USB_DEVICE_SOURCE_PORT) display.print(F("U "));
  else if (lastIncomingSource == 0) display.print(F("D "));
  else display.print(F("- "));
  if (lastIncomingStatus < 0x10) display.print('0');
  display.print(lastIncomingStatus, HEX);
  display.print(' ');
  if (lastIncomingData1 < 0x10) display.print('0');
  display.print(lastIncomingData1, HEX);
  display.print(' ');
  if (lastIncomingData2 < 0x10) display.print('0');
  display.print(lastIncomingData2, HEX);

  display.setCursor(0, 16);
  display.print(F("TX ")); display.print(secondaryTxDepth());
  display.print(F(" H")); display.print(secondaryTxHighWater);
  display.print(F(" D")); display.print(secondaryTxDropped);
  if (secondaryTxCriticalDropped) {
    display.print(F(" !")); display.print(secondaryTxCriticalDropped);
  }

  display.setCursor(0, 24);
  display.print(F("LOOP ")); display.print(multitrackLooper.usedEvents());
  display.print('/'); display.print(arpnmidi3::kLoopEventPoolSize);
  display.print(F(" O")); display.print(multitrackLooper.overflowCount());

  display.setCursor(0, 32);
  display.print(F("HIST ")); display.print(rollingHistory.size());
  display.print('/'); display.print(arpnmidi3::kRollingHistoryCapacity);
  display.print(F(" O")); display.print(rollingHistory.overwrittenCount());

  display.setCursor(0, 40);
  display.print(overload ? F("RISK ") : F("OK "));
  if (firmware3Settings.clockInFollow) {
    const uint32_t age = musicalClock.lastExternalClockAgeMs(time_us_64());
    display.print(F("CLK "));
    if (age == UINT32_MAX) display.print(F("NONE"));
    else { display.print(age); display.print(F("ms")); }
  } else display.print(F("CLK INT"));
  display.print(F(" S"));
  if (presetStorageDirty) display.print('P');
  if (loopStorageDirty) display.print('L');
  if (extendedPresetDirty) display.print('E');

  // Storage is the one subsystem that can fail silently, so it reports plainly.
  // NO FS means the board was built without a filesystem partition and nothing
  // can ever be saved.
  display.setCursor(0, 56);
  // Spelled out rather than single letters: P and L above already mean the
  // preset and loop dirty flags, and reusing them here for pass/lateness
  // timing would read as the same thing on a screen this dense.
  display.print(F("PASS ")); display.print(perfLoopMaxUsShown);
  display.print(F("u LATE ")); display.print(perfLateMaxUsShown);
  display.print(F("u"));

  display.setCursor(0, 48);
  if (!littleFsReady) {
    display.print(F("FS NONE - set 512KB FS"));
  } else if (storageError || loopStorageError) {
    display.print(F("FS WRITE ERROR"));
  } else {
    FSInfo info;
    display.print(F("FS OK "));
    if (LittleFS.info(info)) {
      display.print((info.totalBytes - info.usedBytes) / 1024U);
      display.print(F("K free"));
    }
  }
}

void drawDrumMagicScreen() {
  static const char *const names[] = {
    "ON/OFF", "INPUT", "OUTPUT", "SPLIT", "MAP START",
    "AT>VEL", "DIVISION", "BACK"
  };
  String division;
  if (firmware3Settings.drumDivision == DRUM_DIVISION_FOLLOW_ARP) division = "ARP";
  else if (firmware3Settings.drumDivision == DRUM_DIVISION_FREE) division = "FREE";
  else division = kDivisionNames[firmware3Settings.drumDivision];
  if (ui.menuMode == MENU_SELECT) {
    drawSubmenuField("", firmware3Settings.drumEnabled
        ? String(firmware3Settings.drumOutputChannel) + " " + division : String("OFF"), false);
    return;
  }
  String value;
  if (drumMagicUi.cursor == 0) value = onOff(firmware3Settings.drumEnabled);
  else if (drumMagicUi.cursor == 1) value = firmware3Settings.drumInputMode
      ? "KEY SPLIT" : "CH 10";
  else if (drumMagicUi.cursor == 2) value = String("CH ") + String(firmware3Settings.drumOutputChannel);
  else if (drumMagicUi.cursor == 3) value = String(firmware3Settings.drumSplitNote);
  else if (drumMagicUi.cursor == 4) value = String(firmware3Settings.drumMappedStart);
  else if (drumMagicUi.cursor == 5) value = onOff(firmware3Settings.drumAftertouchVelocity);
  else if (drumMagicUi.cursor == 6) value = division;
  drawSubmenuField(names[drumMagicUi.cursor], value, drumMagicUi.editing);
}

String loopLengthSelectionName(uint8_t selection) {
  static const char *const names[] = {
    "1/4 BAR", "1/2 BAR", "1 BAR", "2 BARS", "4 BARS", "8 BARS", "FREE"
  };
  return names[clampU8(selection, 0, 6)];
}

// Short enough that each summary row can also carry its quantize value.
const char *loopLengthSummaryName(uint8_t selection) {
  static const char *const names[] = {
    "1/4B", "1/2B", "1Br", "2Br", "4Br", "8Br", "Free"
  };
  return names[clampU8(selection, 0, 6)];
}

const char *loopQuantizeSummaryName(uint8_t selection) {
  // Off, then one entry per division from 1/4 through 1/64T. A straight
  // division keeps the "q" prefix; a dotted or triplet name already carries
  // its own D or T letter, so dropping "q" there is what keeps it fitting.
  static const char *const names[] = {
    "q-", "q4", "8D", "4T", "q8", "16D", "8T", "q16", "32D", "16T",
    "q32", "64D", "32T", "q64", "64T"
  };
  return names[clampU8(selection, 0, LOOP_QUANTIZE_DIVISION_COUNT)];
}

char looperTrackModeSummaryLetter() {
  switch (static_cast<arpnmidi3::LoopTrackMode>(firmware3Settings.looperTrackMode)) {
    case arpnmidi3::LoopTrackMode::Layers: return 'L';
    case arpnmidi3::LoopTrackMode::PartsAutoSolo: return 'P';
    case arpnmidi3::LoopTrackMode::Manual: return 'M';
  }
  return 'M';
}

void drawLooperFlagBox(uint8_t x, uint8_t y, char label, bool enabled) {
  display.drawRect(x, y, 13, 11, SSD1306_WHITE);
  if (!enabled) return;
  display.setTextSize(1);
  display.setCursor(x + 4, y + 2);
  display.print(label);
}

void drawLooperSettingsScreen() {
  static const char *const names[] = {
    "TRACK", "LENGTH", "TRK QUANT", "NEW TRACK", "AUTO ARM",
    "TIME TRAV", "REC CC", "TRNSPRT", "BACK"
  };
  const uint8_t track = multitrackLooper.selectedTrack();
  if (ui.menuMode == MENU_SELECT) {
    display.setTextSize(1);
    for (uint8_t i = 0; i < arpnmidi3::kLoopTrackCount; ++i) {
      const int y = 2 + i * 10;
      display.setCursor(0, y);
      display.print(i == track ? F(">") : F(" "));
      display.print(i + 1U);
      display.setCursor(17, y);
      display.print(loopLengthSummaryName(loopTrackLengthSelection[i]));
      display.setCursor(39, y);
      display.print(loopQuantizeSummaryName(loopTrackQuantizeSelection(i)));
      const arpnmidi3::LoopTrackState &state = multitrackLooper.track(i);
      // A filled play triangle is audible content. A hollow one is cleared
      // content that Undo can still bring back. Nothing is an empty track.
      const int ty = 5 + i * 10;
      if (state.count > 0 && !state.hidden) {
        display.fillTriangle(59, ty - 3, 59, ty + 3, 64, ty, SSD1306_WHITE);
      } else if (state.count > 0) {
        display.drawTriangle(59, ty - 3, 59, ty + 3, 64, ty, SSD1306_WHITE);
      }
    }
    drawLooperFlagBox(70, 1, 'A', firmware3Settings.looperAutoRec);
    drawLooperFlagBox(85, 1, 'T', firmware3Settings.looperTimeTravel);
    // A master indicator: lit whenever any track has a quantize set, not just
    // the selected one, since quantize is per track now.
    bool anyTrackQuantized = false;
    for (uint8_t i = 0; i < arpnmidi3::kLoopTrackCount; ++i) {
      anyTrackQuantized |= loopTrackQuantizeSelection(i) != 0;
    }
    drawLooperFlagBox(70, 14, 'Q', anyTrackQuantized);
    drawLooperFlagBox(85, 14, 'C', firmware3Settings.looperRecordCc);
    display.drawRect(101, 1, 16, 24, SSD1306_WHITE);
    display.setTextSize(2);
    display.setCursor(104, 6);
    display.print(looperTrackModeSummaryLetter());
    display.setTextSize(1);
    display.setCursor(71, 31);
    display.print(F("T"));
    display.print(track + 1U);
    const uint8_t summaryQuant = loopTrackQuantizeSelection(track);
    if (summaryQuant) {
      // Quantize Off says nothing at all. The track label alone means free.
      display.print(' ');
      display.print(kDivisionNames[DIV_1_4 + summaryQuant - 1]);
    }
    return;
  }
  String value;
  if (looperSettingsUi.cursor == 0) value = String(track + 1U);
  else if (looperSettingsUi.cursor == 1) value = loopLengthSelectionName(loopTrackLengthSelection[track]);
  else if (looperSettingsUi.cursor == 2) {
    const uint8_t selection = loopTrackQuantizeSelection(track);
    value = selection == 0 ? String("OFF") : String(kDivisionNames[DIV_1_4 + selection - 1]);
  } else if (looperSettingsUi.cursor == 3) {
    static const char *const modes[] = {"LAYERS", "PARTS SOLO", "MANUAL"};
    value = modes[firmware3Settings.looperTrackMode];
  } else if (looperSettingsUi.cursor == 4) value = onOff(firmware3Settings.looperAutoRec);
  else if (looperSettingsUi.cursor == 5) value = onOff(firmware3Settings.looperTimeTravel);
  else if (looperSettingsUi.cursor == 6) value = onOff(firmware3Settings.looperRecordCc);
  else if (looperSettingsUi.cursor == 7) value = onOff(firmware3Settings.looperMidiTransport);
  drawSubmenuField(names[looperSettingsUi.cursor], value, looperSettingsUi.editing);
}

void drawParameterLockScreen() {
  static const char *const names[] = {"CHANNEL", "CLEAR", "BACK"};
  if (ui.menuMode == MENU_SELECT) {
    display.setTextSize(1);
    display.setCursor(0, 5);
    display.print(F("Per-Note"));
    display.setCursor(0, 15);
    display.print(F("Parameter Lock"));
    display.setTextSize(2);
    display.setCursor(0, 29);
    display.print(firmware3Settings.parameterLockChannel == 0
        ? F("OFF") : F("CH "));
    if (firmware3Settings.parameterLockChannel != 0) {
      display.print(firmware3Settings.parameterLockChannel);
    }
    return;
  }
  String value;
  if (parameterLockUi.cursor == 0) value = firmware3Settings.parameterLockChannel == 0
      ? "OFF" : String("CH ") + String(firmware3Settings.parameterLockChannel);
  else if (parameterLockUi.cursor == 1) value = String(parameterLockCount) + " STORED";
  if (parameterLockUi.cursor == 0 && cancelSelectedFor(SET_PARAMETER_LOCK)) value = "CANCEL";
  drawSubmenuField(names[parameterLockUi.cursor], value, parameterLockUi.editing);
}

void drawChordScreen() {
  static const char *const names[] = {"ON/OFF", "POSITION 1", "POSITION 2", "POSITION 3", "POSITION 4", "BACK"};
  if (ui.menuMode == MENU_SELECT) {
    drawSubmenuField("", onOff(firmware3Settings.chordEnabled), false);
    return;
  }
  String value;
  if (chordUi.cursor == 0) value = onOff(firmware3Settings.chordEnabled);
  else if (chordUi.cursor < 5) value = String(firmware3Settings.chordPositions[chordUi.cursor - 1]);
  drawSubmenuField(names[chordUi.cursor], value, chordUi.editing);
}

void drawLiveCcScreen() {
  static const char *const names[] = {"CC NUMBER", "VALUE", "BACK"};
  if (ui.menuMode == MENU_SELECT) {
    drawSubmenuField("", String("CC ") + String(liveCcNumber) + " = " + String(liveCcValue), false);
    return;
  }
  String value;
  if (liveCcCursor < 2) value = String(liveCcCursor == 0 ? liveCcNumber : liveCcValue);
  drawSubmenuField(names[liveCcCursor], value, liveCcEditing);
  if (liveCcEditing && liveCcCursor == 1) {
    const int cx = 105;
    const int cy = 28;
    display.drawCircle(cx, cy, 12, SSD1306_WHITE);
    const float angle = -2.35619f + (static_cast<float>(liveCcValue) / 127.0f) * 4.71239f;
    display.drawLine(cx, cy, cx + static_cast<int>(9 * cosf(angle)),
                     cy + static_cast<int>(9 * sinf(angle)), SSD1306_WHITE);
  }
}

void drawGlobalScreen() {
  static const char *const names[] = {
    "AUTO SAVE", "CLOCK IN", "CLOCK OUT", "TIME SIG", "CH AFTERTCH",
    "POLY AFTER", "AFTER CC", "AT>ARP VEL", "RESET SLOT", "BACK"
  };
  if (ui.menuMode == MENU_SELECT) {
    drawSubmenuField("", firmware3Settings.timeSignature ? String("3/4") : String("4/4"), false);
    return;
  }
  String value;
  if (globalUi.cursor == 0) value = onOff(storage.autoSave);
  else if (globalUi.cursor == 1) value = firmware3Settings.clockInFollow ? "CLIENT" : "IGNORE";
  else if (globalUi.cursor == 2) value = firmware3Settings.clockOutSend ? "SEND/HOST" : "OFF";
  else if (globalUi.cursor == 3) value = firmware3Settings.timeSignature ? "3/4" : "4/4";
  else if (globalUi.cursor == 4) value = onOff(firmware3Settings.forwardChannelAftertouch);
  else if (globalUi.cursor == 5) value = onOff(firmware3Settings.forwardPolyAftertouch);
  else if (globalUi.cursor == 6) value = firmware3Settings.channelAftertouchCc <= 127
      ? String(firmware3Settings.channelAftertouchCc) : "OFF";
  else if (globalUi.cursor == 7) value = onOff(firmware3Settings.mainAftertouchArpVelocity);
  drawSubmenuField(names[globalUi.cursor], value, globalUi.editing);
}

bool displayBufferPixel(const uint8_t *buffer, uint8_t x, uint8_t y) {
  return buffer[x + (static_cast<uint16_t>(y / 8) * SCREEN_W)] & (1U << (y & 7));
}

void setDisplayBufferPixel(uint8_t *buffer, uint8_t x, uint8_t y, bool on) {
  uint8_t &cell = buffer[x + (static_cast<uint16_t>(y / 8) * SCREEN_W)];
  const uint8_t mask = 1U << (y & 7);
  if (on) cell |= mask;
  else cell &= ~mask;
}

void moveRenderedSettingArea() {
  if (SETTING_AREA_Y == 0) return;

  uint8_t *buffer = display.getBuffer();
  for (int y = SETTING_AREA_H - 1; y >= 0; --y) {
    for (uint8_t x = 0; x < SCREEN_W; ++x) {
      const bool on = displayBufferPixel(buffer, x, static_cast<uint8_t>(y));
      setDisplayBufferPixel(buffer, x, static_cast<uint8_t>(y + SETTING_AREA_Y), on);
    }
  }

  for (uint8_t y = 0; y < SETTING_AREA_Y; ++y) {
    for (uint8_t x = 0; x < SCREEN_W; ++x) {
      setDisplayBufferPixel(buffer, x, y, false);
    }
  }
}

void renderMainTop() {
  const uint8_t id = ui.selectedSetting;
  const int16_t v = effectiveSettingValue(id);
  switch (id) {
    case SET_BPM:
      display.setTextSize(4);
      display.setCursor(10, 6);
      display.print(currentBpm());
      if (static_cast<int32_t>(tapTempoVisibleUntilMs - millis()) > 0) {
        display.setTextSize(1);
        display.setCursor(98, 37);
        display.print(F("Tap"));
      }
      break;
    case SET_SWING:
      drawBarValue(firmware3Settings.swing, String(firmware3Settings.swing) + "%");
      break;
    case SET_ARP_MODE:
      drawArpMenuScreen();
      break;
    case SET_LIVE_VELOCITY:
      drawLiveVelocityScreen();
      break;
    case SET_LIVE_NOTE_LENGTH:
      drawLiveNoteLengthScreen();
      break;
    case SET_STUTTER:
      drawStutterScreen();
      break;
    case SET_ECHO:
      drawEchoScreen();
      break;
    case SET_DIVISION:
      drawDivisionPie(v);
      break;
    case SET_VELOCITY:
      drawBarValue(map(v, 0, 127, 0, 100), String(v));
      break;
    case SET_LENGTH:
      drawBarValue(v, String(v) + "%");
      break;
    case SET_QUICK_JUMP:
      drawQuickJumpScreen();
      break;
    case SET_INPUT_CH:
      drawChannelScreen(F("MAIN INPUT"), v);
      break;
    case SET_ARP_OUT_CH:
      drawChannelScreen(F("ARP OUT"), v, true);
      break;
    case SET_DRUM_MAGIC:
      drawDrumMagicScreen();
      break;
    case SET_BASS_CH:
      drawBassMenuScreen();
      break;
    case SET_THRU_OUT_CH:
      drawChannelScreen(F("THRU"), v, true);
      break;
    case SET_RND_RBN:
      drawRoundRobinScreen(v);
      break;
    case SET_ROUTER:
      drawRouterScreen(v);
      break;
    case SET_DIV_NOTES:
      drawDivNotesScreen(v);
      break;
    case SET_MAP_CC:
      drawFeaturesScreen();
      break;
    case SET_CC_MAP:
      drawCcRemapScreen();
      break;
    case SET_NOTE_CC:
      drawNoteCcScreen();
      break;
    case SET_LEGATO_CH:
      drawMonoRetrigScreen(v);
      break;
    case SET_CC_OUT_CH:
      drawCcChannelScreen(v);
      break;
    case SET_FOUR_BUTTON:
      drawFourButtonScreen();
      break;
    case SET_MUTE_SOLO:
      drawMuteSoloScreen();
      break;
    case SET_SENSOR_CH:
      drawChannelScreen(F("SENSORS"), v);
      break;
    case SET_LOOP_BARS:
      drawLooperSettingsScreen();
      break;
    case SET_PARAMETER_LOCK:
      drawParameterLockScreen();
      break;
    case SET_CHORD:
      drawChordScreen();
      break;
    case SET_FORCE_KEY:
    case SET_GUITAR_PIANO:
      if (effectiveSettingValue(SET_GUITAR_PIANO) == 0) drawGuitarView();
      else drawPianoView();
      break;
    case SET_FORCE_SCALE:
      drawScaleMenuScreen();
      break;
    case SET_LIVE_CC:
      drawLiveCcScreen();
      break;
    case SET_GLOBAL:
      drawGlobalScreen();
      break;
    case SET_PANIC:
      drawPanicScreen();
      break;
    case SET_LOAD_PRESET:
      drawPresetGrid(v);
      break;
    case SET_SAVE_PRESET:
      drawPresetGrid(v);
      break;
    default:
      drawWrappedTopValue(settingValueString(id));
      break;
  }
  if (submenuBackSelected()) drawBackNavigationArrow();
}

void drawScreenSaver() {
  if (!screenSaverSeeded) {
    randomSeed(micros() + millis());
    screenSaverSeeded = true;
  }
  display.clearDisplay();
  int sy = random(34) + 30;
  int sx = 0;
  int moon = random(112) + 8;
  int size = random(4) + 6;
  while (sx < 127) {
    sx++;
    sy += random(5) - 2;
    sy = constrain(sy, size + 4, 63);
    if (sx == moon) display.fillCircle(sx, sy - random(8) - 8, size, SSD1306_WHITE);
    display.drawPixel(sx, sy, SSD1306_WHITE);
  }
  for (uint8_t i = 0; i < 18; ++i) display.drawPixel(random(128), random(16), SSD1306_WHITE);
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, random(56));
  display.print(F("woz.lol"));
  display.display();
}

// DMA-paced push of the OLED's full 1024-byte frame over I2C, so this core
// only has to start the transfer, not sit through the whole ~20ms of it
// while the outgoing MIDI queue waits. The RP2040's I2C peripheral (i2c1,
// the block Wire1 already configured pins and clock speed for) paces the
// DMA controller off its own TX FIFO room, its DREQ, so once this call
// returns, the transfer runs in the background at full bus speed while this
// core goes back to draining outgoing MIDI. That peripheral encodes a
// RESTART/STOP bit alongside each data byte in its command register rather
// than a separate control line, so the DMA source has to be 16-bit words,
// not raw bytes: one control byte (0x40, the data-stream marker) followed
// by the 1024 pixel bytes, with STOP set only on the very last word.
constexpr uint16_t DISPLAY_DMA_FRAME_BYTES = SCREEN_W * ((SCREEN_H + 7) / 8);  // 1024
int displayDmaChannel = -1;
bool displayDmaInFlight = false;
uint16_t displayDmaWords[DISPLAY_DMA_FRAME_BYTES + 1];

// DMA finishing only means every word was handed to the peripheral's FIFO;
// the peripheral can still be shifting the last few bytes onto the wire and
// sending the final STOP afterward, so both have to read idle before it is
// safe to touch the frame buffer again or start another transfer.
bool displayDmaBusy() {
  if (!displayDmaInFlight) return false;
  if (dma_channel_is_busy(displayDmaChannel)) return true;
  if (i2c1->hw->status & I2C_IC_STATUS_ACTIVITY_BITS) return true;
  displayDmaInFlight = false;
  return false;
}

// A tiny, rare-path safety wait for the few display writers that do not run
// through renderDisplayIfNeeded and so are not already covered by its own
// displayDmaBusy() check at the top: without this, one of them could start
// a conflicting transaction on the same I2C peripheral while a previous
// DMA-driven push is still finishing in the background.
void waitForDisplayDma() {
  while (displayDmaBusy()) tight_loop_contents();
}

void pushDisplayFrameDma() {
  if (displayDmaChannel < 0) {
    display.display();
    return;
  }
  uint8_t *buf = display.getBuffer();
  displayDmaWords[0] = 0x40;
  for (uint16_t i = 0; i < DISPLAY_DMA_FRAME_BYTES; ++i) {
    displayDmaWords[1 + i] = buf[i];
  }
  displayDmaWords[DISPLAY_DMA_FRAME_BYTES] |= (1U << 9);  // STOP on the last word

  i2c1->hw->enable = 0;
  i2c1->hw->tar = OLED_ADDR;
  i2c1->hw->enable = 1;

  dma_channel_config config = dma_channel_get_default_config(displayDmaChannel);
  channel_config_set_transfer_data_size(&config, DMA_SIZE_16);
  channel_config_set_read_increment(&config, true);
  channel_config_set_write_increment(&config, false);
  channel_config_set_dreq(&config, i2c_get_dreq(i2c1, true));
  dma_channel_configure(displayDmaChannel, &config, &i2c1->hw->data_cmd,
                        displayDmaWords, DISPLAY_DMA_FRAME_BYTES + 1, true);
  displayDmaInFlight = true;
}

void renderDisplayIfNeeded() {
  // Nothing here may touch the frame buffer, or start a command sequence on
  // the same I2C peripheral, while a previous push is still in flight.
  if (displayDmaBusy()) return;
  const uint32_t now = millis();
  const uint32_t saverTimeout = screenSaverTimeoutMs(settings.screenSaver);
  if (screenSaverForceNow || (saverTimeout && (now - ui.lastActivityMs) > saverTimeout)) {
    if (!ui.inSaver || (now - ui.lastRenderMs) > SCREEN_SAVER_REFRESH_MS) {
      ui.inSaver = true;
      drawScreenSaver();
      ui.lastRenderMs = now;
    }
    return;
  }

  if (!ui.dirty) return;
  // Dirty can be set faster than frames are worth drawing, one drum-roll hit
  // at a time. Capping the frame rate keeps this core mostly free for the
  // MIDI drain.
  if ((now - ui.lastRenderMs) < uiMinFrameIntervalMs()) return;
  // Clear before drawing so a real-time-side change during the I2C transfer
  // leaves dirty asserted for the next frame instead of being lost.
  ui.dirty = false;
  ui.inSaver = false;
  display.clearDisplay();
  if (static_cast<int32_t>(panicConfirmedUntilMs - now) > 0) {
    drawPanicHoldScreen();
    display.display();
    ui.lastRenderMs = now;
    return;
  }
  renderMainTop();
  drawLoopStatusIcon();
  moveRenderedSettingArea();
  drawModeLabel();
  drawModeIndicator();
  pushDisplayFrameDma();
  ui.lastRenderMs = now;
}

// A flash write disables interrupts and parks the rendering core for tens of
// milliseconds, so the instrument stops answering. The display says so first.
// Only the core that owns the panel may draw, so this is a request and a wait
// for that core to acknowledge rather than a direct draw.
void showBusyHourglass() {
  if (uiBusyRequest) return;
  uiBusyShown = false;
  uiBusyRequest = true;
  const uint32_t startMs = millis();
  while (!uiBusyShown && millis() - startMs < 40UL) tight_loop_contents();
}

void endBusyHourglass() {
  if (!uiBusyRequest) return;
  uiBusyRequest = false;
  ui.dirty = true;
}

void drawBusyHourglassNow() {
  // Runs outside renderDisplayIfNeeded, so its own displayDmaBusy() check at
  // the top does not cover this call: a previous DMA-driven push could
  // still be reading the frame buffer or still own the I2C peripheral.
  waitForDisplayDma();
  const int y = SETTING_AREA_Y + 23;
  display.fillRect(116, y, 12, 18, SSD1306_BLACK);
  display.drawTriangle(118, y + 1, 126, y + 1, 122, y + 8, SSD1306_WHITE);
  display.drawTriangle(118, y + 16, 126, y + 16, 122, y + 9, SSD1306_WHITE);
  display.display();
}

void processDeferredUiActions() {
  if (!ui.deferredExitWork) return;

  if (ui.deferredSaveOnly) {
    storage.currentPreset = settings.savePreset;
    saveStorage();
    settings.loadPreset = storage.currentPreset;
    settings.savePreset = storage.currentPreset;
    ui.dirty = true;
  } else if (ui.deferredLoadPreset) {
    // Anything still pending belongs to the preset being left, so it is written
    // before the switch rather than dropped by the load.
    if (presetStorageDirty) saveStorage();
    else if (extendedPresetDirty && savePresetLearnedContent(storage.currentPreset)) {
      extendedPresetDirty = false;
    }
    panicAll();
    storage.currentPreset = settings.loadPreset;
    loadCurrentPreset();
  } else {
    if (ui.hasPendingEdit && ui.pendingSetting == ui.selectedSetting) {
      panicMidiOnly();
      setSettingValueRaw(ui.pendingSetting, ui.pendingValue);
      ui.hasPendingEdit = false;
    }
    if (ui.selectedSetting == SET_DIV_NOTES && divNotesCursor == DIV_NOTE_RESET_SLOT) {
      clearDivNoteAssignments();
      divNotesCursor = 0;
      ui.dirty = true;
    }
    if (ui.selectedSetting == SET_ROUTER || ui.selectedSetting == SET_BASS_CH ||
        ui.selectedSetting == SET_FORCE_SCALE) {
      panicMidiOnly();
      ui.dirty = true;
    }

    // Back is a navigation choice, not the persistent status of the parent
    // menu. Clear it after deferred exit work so list-style submenus cannot
    // reopen or render with BACK still selected.
    if (ui.selectedSetting == SET_DIV_NOTES &&
        divNotesCursor == DIV_NOTE_BACK_SLOT) {
      divNotesCursor = 0;
    }
    if (ui.selectedSetting == SET_RND_RBN &&
        roundRobinMenuCursor == RND_RBN_BACK_SLOT) {
      roundRobinMenuCursor = 0;
    }
    if (ui.selectedSetting == SET_ROUTER &&
        routerEditStage == ROUTER_STAGE_LIST &&
        routerMenuCursor == ROUTER_BACK_SLOT) {
      routerMenuCursor = 0;
    }
    if (ui.selectedSetting == SET_NOTE_CC &&
        noteCcUiStage == NOTE_CC_UI_LIST &&
        noteCcCursor > NOTE_CC_SLOT_COUNT) {
      noteCcCursor = 0;
    }
    if (ui.selectedSetting == SET_CC_MAP &&
        ccRemapUiStage == CC_REMAP_UI_LIST &&
        ccRemapCursor > CC_REMAP_SLOT_COUNT) {
      ccRemapCursor = 0;
    }
    if (ui.selectedSetting == SET_MUTE_SOLO &&
        muteSoloCursor == LOOP_MIX_BACK_SLOT) {
      muteSoloCursor = 0;
    }
    if (ui.selectedSetting == SET_FOUR_BUTTON) {
      if (fourButtonUiStage == FOUR_BUTTON_UI_MAIN && fourButtonUiCursor == 4) {
        fourButtonUiCursor = 0;
      } else if (fourButtonUiStage == FOUR_BUTTON_UI_CUSTOM_LIST &&
                 fourButtonUiCursor >= FOUR_BUTTON_CUSTOM_DONE_SLOT) {
        fourButtonUiStage = FOUR_BUTTON_UI_MAIN;
        fourButtonUiCursor = 0;
      } else if (fourButtonUiStage == FOUR_BUTTON_UI_LOOPER &&
                 fourButtonUiCursor >= FOUR_BUTTON_LOOPER_DONE_SLOT) {
        fourButtonUiStage = FOUR_BUTTON_UI_MAIN;
        fourButtonUiCursor = 0;
      } else if (fourButtonUiStage == FOUR_BUTTON_UI_CHORD &&
                 fourButtonUiCursor >= FOUR_BUTTON_CHORD_DONE_SLOT) {
        fourButtonUiStage = FOUR_BUTTON_UI_MAIN;
        fourButtonUiCursor = 0;
      }
    }
    if (storage.autoSave) {
      // The click that leaves a screen is a deliberate, one-shot commitment,
      // not a repeating background trigger, so it always attempts the save
      // rather than waiting for a playing loop to go quiet: the compare-skip
      // makes an unchanged setting free, and a real change pays one visible
      // pause exactly when the performer asked for it, loop playing or not.
      // Recording is the one state still worth deferring, since a stall
      // landing inside a take is a genuinely bad moment for one.
      if (storageWriteAlwaysBlocked()) {
        presetStorageDirty = true;
        presetStorageDirtyMs = millis();
      } else {
        saveStorage();
      }
    }
  }

  ui.deferredExitWork = false;
  ui.deferredLoadPreset = false;
  ui.deferredSaveOnly = false;

  ui.dirty = true;
}


void showBootStage(const __FlashStringHelper *line1,
                   const __FlashStringHelper *line2) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.println(F("ARPnMIDI"));
  display.setTextSize(1);
  display.setCursor(0, 24);
  display.println(line1);
  if (line2) display.println(line2);
  display.display();
}

void setupDisplay() {
  Wire1.setSDA(PIN_I2C_SDA);
  Wire1.setSCL(PIN_I2C_SCL);
  Wire1.begin();
  Wire1.setClock(400000);
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  display.setRotation(DISPLAY_ROTATION);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  showBootStage(F("Display OK"), F("Starting..."));
  display.display();
  // Claimed once and reused for every push; required:true panics on boot if
  // every channel is somehow already spoken for, loudly, rather than
  // silently falling back to a blocking display() that would race a DMA
  // push that in fact never started.
  displayDmaChannel = dma_claim_unused_channel(true);
}

void setupSensor() {
  tof.setBus(&Wire1);
  sensorRt.present = tof.init();
  if (!sensorRt.present) return;
  tof.setTimeout(50);
  tof.setMeasurementTimingBudget(20000);
  tof.startContinuous();
  sensorRt.ready = true;
}

void setupDinMidi() {
  DinSerial.setRX(PIN_DIN_MIDI_RX);
  DinSerial.setTX(PIN_DIN_MIDI_TX);
  DinMIDI.begin(MIDI_CHANNEL_OMNI);
  DinMIDI.turnThruOff();
  DinMIDI.setHandleNoteOn(handleDinNoteOn);
  DinMIDI.setHandleNoteOff(handleDinNoteOff);
  DinMIDI.setHandleControlChange(handleDinCc);
  DinMIDI.setHandlePitchBend(handleDinPb);
  DinMIDI.setHandleProgramChange(handleDinProgramChange);
  DinMIDI.setHandleAfterTouchChannel(handleDinAfterTouchChannel);
  DinMIDI.setHandleClock(handleDinClock);
  DinMIDI.setHandleStart(handleDinStart);
  DinMIDI.setHandleContinue(handleDinContinue);
  DinMIDI.setHandleStop(handleDinStop);
  DinMIDI.setHandleSongSelect(handleSongSelect);
  DinMIDI.setHandleSystemExclusive(handleDinSystemExclusive);
}

void setupUsbDeviceMidi() {
#if ARPNMIDI_ENABLE_USB_DEVICE_MIDI
  if (!TinyUSBDevice.isInitialized()) {
    TinyUSBDevice.begin(0);
  }

  TinyUSBDevice.setManufacturerDescriptor("WozAction1");
  TinyUSBDevice.setProductDescriptor("WozAction1");
  usbDeviceMidi.setStringDescriptor("WozAction1");
  usbDeviceMidi.begin();

  if (TinyUSBDevice.mounted()) {
    TinyUSBDevice.detach();
    delay(10);
    TinyUSBDevice.attach();
  }
#endif
}

void pumpUsbDeviceMidiInput() {
#if ARPNMIDI_ENABLE_USB_DEVICE_MIDI
  uint8_t packet[4];
  while (usbDeviceMidi.readPacket(packet)) {
    const uint8_t len = usbDevicePacketDataLength(packet[0]);
    if (len == 0) continue;

    const uint8_t cin = packet[0] & 0x0F;
    if (cin >= 0x04 && cin <= 0x07) {
      for (uint8_t index = 1; index <= len; ++index) {
        const uint8_t value = packet[index];
        if (value == 0xF0) usbSysexLength = 0;
        if (usbSysexLength < sizeof(usbSysexBuffer)) {
          usbSysexBuffer[usbSysexLength++] = value;
        }
        if (value == 0xF7) {
          parseTransportSysex(usbSysexBuffer, usbSysexLength);
          usbSysexLength = 0;
        }
      }
      continue;
    }

    const uint8_t status = packet[1];
    const uint8_t data1 = (len > 1) ? packet[2] : 0;
    const uint8_t data2 = (len > 2) ? packet[3] : 0;
    if (status >= 0xF8) {
      if (status == 0xF8 || status == 0xFA || status == 0xFB || status == 0xFC) {
        handleRealtimeByte(USB_DEVICE_SOURCE_PORT, status);
      } else {
        sendFanout(USB_DEVICE_SOURCE_PORT, status, 0, 0);
      }
    } else if (status >= 0x80 && status <= 0xEF) {
      routeIncomingChannelMessage(USB_DEVICE_SOURCE_PORT, status, data1, data2);
    } else if (status == 0xF3 && len > 1) {
      handleSongSelect(data1);
    }
  }
#endif
}

void setupPins() {
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  pinMode(PIN_ENC_SW, INPUT_PULLUP);
  pinMode(PIN_BUTTON_1, INPUT_PULLDOWN);
  pinMode(PIN_BUTTON_2, INPUT_PULLDOWN);
  pinMode(PIN_BUTTON_3, INPUT_PULLDOWN);
  pinMode(PIN_BUTTON_4, INPUT_PULLDOWN);
  pinMode(PIN_PUSH, INPUT);
  encoder.lastAB = (digitalRead(PIN_ENC_A) << 1) | digitalRead(PIN_ENC_B);
  encoder.lastSwitch = digitalRead(PIN_ENC_SW);
  for (uint8_t button = 0; button < 4; ++button) {
    physicalButtonState[button] = digitalRead(kButtonPins[button]);
  }
#if ARPNMIDI_ENABLE_RGB_LED
  onboardRgb.begin();
  onboardRgb.setBrightness(80);
  onboardRgb.setPixelColor(0, onboardRgb.Color(0, 0, 0));
  onboardRgb.show();
#endif
}

// The filesystem is not mounted yet when the boot prompt runs, so the request is
// recorded and honoured by storage init.
void factoryResetStorage() {
  factoryResetRequested = true;
}

void maybeConfirmFactoryResetAtBoot() {
  if (watchdog_caused_reboot() || watchdog_hw->scratch[0] == UI_RESUME_MAGIC) return;
  if (digitalRead(PIN_ENC_SW) != LOW) return;

  showBootStage(F("Reset all?"), F("Release button"));
  while (digitalRead(PIN_ENC_SW) == LOW) delay(5);

  showBootStage(F("Factory reset"), F("Press again"));
  const uint32_t startMs = millis();
  while ((millis() - startMs) < 5000UL) {
    if (digitalRead(PIN_ENC_SW) == LOW) {
      delay(20);
      while (digitalRead(PIN_ENC_SW) == LOW) delay(5);
      showBootStage(F("Clearing"), F("defaults..."));
      factoryResetStorage();
      delay(150);
      break;
    }
    delay(5);
  }
}

void setup() {
  delay(300);
  setupPins();
  setupDisplay();
  maybeConfirmFactoryResetAtBoot();
  showBootStage(F("Storage..."));
  littleFsReady = LittleFS.begin();
  if (!littleFsReady) {
    // A first mount can fail on a blank filesystem, which formatting fixes. It
    // also fails when the board was built with no filesystem partition at all,
    // and then nothing can be saved, so that case is shown rather than hidden.
    LittleFS.format();
    littleFsReady = LittleFS.begin();
  }
  if (!littleFsReady) {
    showBootStage(F("NO FILESYSTEM"), F("Flash Size: 512KB FS"));
    delay(4000);
  }
  initStorageIfNeeded();
  syncMusicalClockConfig(true);
  uint8_t resumeSetting = SET_BPM;
  if (loadPersistedUiSetting(resumeSetting)) ui.selectedSetting = resumeSetting;
  if (takeUiResumeHint(resumeSetting) && resumeSetting < SETTING_COUNT && selectableSetting(resumeSetting)) {
    ui.selectedSetting = resumeSetting;
  }
  persistedUiSetting = ui.selectedSetting;
  observedUiSetting = ui.selectedSetting;
  showBootStage(F("Held state..."));
  resetHeldState();
  showBootStage(F("Sensor..."));
  setupSensor();
  showBootStage(F("DIN MIDI..."));
  setupDinMidi();
  showBootStage(F("USB device..."));
  setupUsbDeviceMidi();
  panicMidiOnly();
  showBootStage(F("Ready"));
  ui.lastActivityMs = millis();
  ui.dirty = true;
  __dmb();
  mainSetupComplete = true;
}

void setup1() {
  while (!mainSetupComplete) delay(1);
  secondaryTxQueueEnabled = true;
}

void loop1() {
  drainSecondaryMidiTx();
  pollSensorHardwareCore1();
  if (uiBusyRequest) {
    if (!uiBusyShown) {
      drawBusyHourglassNow();
      uiBusyShown = true;
    }
  } else if (secondaryTxDepth() == 0 ||
             (millis() - ui.lastRenderMs) > RENDER_STARVED_MS) {
    // A frame push occupies this core and the I2C bus for tens of
    // milliseconds while queued notes wait. Notes outrank pixels, so a render
    // only starts when the outgoing queue is empty, with a starvation bound
    // so the screen still moves under sustained traffic.
    renderDisplayIfNeeded();
  }
  yield();
}

void loop() {
  const uint64_t perfPassStartUs = time_us_64();
#ifdef TINYUSB_NEED_POLLING_TASK
  TinyUSBDevice.task();
#endif
  pollEncoder();
  pollButtons();
  // The inter-brain UART is intentionally faster than a physical MIDI cable.
  // Drain a bounded byte batch so USB/CC bursts cannot sit behind one-byte MIDI
  // Library parsing, while still returning promptly to musical scheduling.
  for (uint8_t parsed = 0; parsed < 32 && DinSerial.available() > 0; ++parsed) {
    DinMIDI.read();
  }
  pumpUsbDeviceMidiInput();
  for (uint8_t sent = 0; sent < 4 && musicalClock.takeInternalClock(time_us_64()); ++sent) {
    sendFanout(255, 0xF8, 0, 0);
  }
  pollSensor();
  pollPush();
  tickLooper();
  tickArp();
  tickNoteLength();
  tickStutter();
  tickEcho();
  if (tapTempoVisibleUntilMs &&
      static_cast<int32_t>(millis() - tapTempoVisibleUntilMs) >= 0) {
    tapTempoVisibleUntilMs = 0;
    if (ui.selectedSetting == SET_BPM) ui.dirty = true;
  }
  if (panicConfirmedUntilMs &&
      static_cast<int32_t>(millis() - panicConfirmedUntilMs) >= 0) {
    panicConfirmedUntilMs = 0;
    ui.dirty = true;  // the panic overlay covers every screen, so always redraw
  }
  processDeferredUiActions();
  pollPresetStoragePersistence();
  pollLoopStoragePersistence();
  pollExtendedPresetPersistence();
  pollUiScreenPersistence();

  const uint32_t perfPassUs =
      static_cast<uint32_t>(time_us_64() - perfPassStartUs);
  if (perfPassUs > perfLoopMaxUs) perfLoopMaxUs = perfPassUs;
  const uint32_t perfNowMs = millis();
  if (perfNowMs - perfWindowStartMs >= 1000UL) {
    perfWindowStartMs = perfNowMs;
    const bool changed = perfLoopMaxUsShown != perfLoopMaxUs ||
                         perfLateMaxUsShown != perfLateMaxUs;
    perfLoopMaxUsShown = perfLoopMaxUs;
    perfLateMaxUsShown = perfLateMaxUs;
    perfLoopMaxUs = 0;
    perfLateMaxUs = 0;
    if (changed && ui.selectedSetting == SET_PANIC) ui.dirty = true;
  }
}
