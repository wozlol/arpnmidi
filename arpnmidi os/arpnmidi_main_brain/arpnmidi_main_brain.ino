/*
  arpnmidi.ino
  RP2040 Zero MIDI router / arpeggiator / processor / multitool

  Pin plan using the board labels you can see on this RP2040 Zero:

  0   Serial TX to ESP32-C3
  1   Serial RX from ESP32-C3
  2   I2C SDA for SSD1306 + VL53L0X
  3   I2C SCL for SSD1306 + VL53L0X
  4   MIDI serial TX to secondary brain
  5   MIDI serial RX from secondary brain
  6   Encoder A
  7   Encoder B
  8   Encoder push
  9   Local button 1
  12  Local button 2
  10  Local button 3 (reserved; action not implemented yet)
  13  Local button 4 (reserved; action not implemented yet)
  26  Push/pressure sensor analog in

  11 intentionally avoided.

  Notes:
  - MAX3421E USB host lives on the secondary brain in this build.
  - Build this with the Arduino-Pico core, Tools->USB Stack = Adafruit TinyUSB,
    and CPU speed = 120 MHz or 240 MHz.
  - The display is only redrawn when something visible changes.
  - Settings persist in flash-backed EEPROM with 16 preset slots.
*/

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <hardware/watchdog.h>
#include <MIDI.h>
#if ARPNMIDI_ENABLE_RGB_LED
#include <Adafruit_NeoPixel.h>
#endif
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Fonts/FreeSans9pt7b.h>
#include <VL53L0X.h>

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
constexpr uint8_t PIN_PUSH = 26;
constexpr uint8_t PIN_RGB_LED = 16;
constexpr uint8_t USB_DEVICE_SOURCE_PORT = 253;

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

constexpr uint32_t SCREEN_SAVER_IDLE_MS = 60000UL;
constexpr uint32_t SCREEN_SAVER_REFRESH_MS = 4000UL;
constexpr uint32_t LONG_HOLD_PANIC_MS = 2000UL;
constexpr uint32_t BUTTON_DEBOUNCE_MS = 25UL;
constexpr uint32_t BUTTON_PULSE_MS = 35UL;

constexpr uint8_t MAX_HELD_NOTES = 32;
constexpr uint8_t MAX_ARP_OUTPUT_NOTES = 8;
constexpr uint16_t MAX_LOOP_EVENTS = 672;
constexpr uint8_t LOOP_BOUNDARY_OFF_RESERVE = 64;
constexpr uint8_t PRESET_COUNT = 16;
constexpr uint8_t DIV_NOTE_SLOT_COUNT = 9;
constexpr uint8_t DIV_NOTE_PLUS_SLOT = DIV_NOTE_SLOT_COUNT;
constexpr uint8_t DIV_NOTE_RESET_SLOT = DIV_NOTE_SLOT_COUNT + 1;
constexpr uint8_t DIV_NOTE_BACK_SLOT = DIV_NOTE_SLOT_COUNT + 2;
constexpr uint8_t RND_RBN_CH10_TO_1_SLOT = 16;
constexpr uint8_t RND_RBN_CH10_TO_2_SLOT = 17;
constexpr uint8_t RND_RBN_CLEAR_SLOT = 18;
constexpr uint8_t RND_RBN_BACK_SLOT = 19;
constexpr uint8_t ROUTER_CLEAR_SLOT = 16;
constexpr uint8_t ROUTER_BACK_SLOT = 17;
constexpr int8_t ROUTER_TRANSPOSE_MIN = -24;
constexpr int8_t ROUTER_TRANSPOSE_MAX = 24;
constexpr uint8_t MAPCC_PARAM_COUNT_V5 = 16;
constexpr uint8_t MAP_CC_CHANNEL_ALL_BIT = 0x01;
constexpr uint8_t MAP_CC_RR_CH10_TO_1_BIT = 0x02;
constexpr uint8_t MAP_CC_RR_CH10_TO_2_BIT = 0x04;
constexpr uint32_t MAP_CC_UI_SETTLE_MS = 260UL;
constexpr uint32_t MAP_CC_DEFER_COMMIT_MS = 500UL;
constexpr uint8_t LOOP_SOURCE_PORT = 251;
constexpr uint32_t LOOP_STOP_DELETE_DEBOUNCE_MS = 300;
constexpr uint32_t LOOP_REC_PLAY_DOUBLE_SWIPE_MS = 1000UL;
constexpr uint32_t ARP_KEY_SYNC_CAPTURE_MS = 6UL;
constexpr uint32_t UI_RESUME_MAGIC = 0x41524D44UL;  // "ARMD"
constexpr uint16_t EEPROM_MAGIC_V1 = 0x4D43;
constexpr uint16_t EEPROM_MAGIC_V2 = 0x4D44;
constexpr uint16_t EEPROM_MAGIC_V3 = 0x4D45;
constexpr uint16_t EEPROM_MAGIC_V4 = 0x4D46;
constexpr uint16_t EEPROM_MAGIC_V5 = 0x4D47;
constexpr uint16_t EEPROM_MAGIC_V6 = 0x4D48;
constexpr uint16_t EEPROM_MAGIC_V7 = 0x4D49;
constexpr uint16_t EEPROM_MAGIC_V8 = 0x4D4A;
constexpr uint16_t EEPROM_MAGIC = 0x4D4D;
constexpr uint16_t LOOP_EEPROM_MAGIC = 0x4C32;
constexpr size_t EEPROM_BYTES = 8192;
constexpr uint8_t ARP_CH_1_PLUS_10 = 17;
constexpr uint8_t ARP_CH_1_PLUS_10_AFTERTOUCH = 18;
constexpr uint8_t ARP_CH_1_TO_10_SPLIT_24 = 19;
constexpr uint8_t ARP_CH_1_TO_10_SPLIT_36 = 20;
constexpr uint8_t ARP_CH_1_TO_10_SPLIT_48 = 21;
constexpr uint8_t ARP_CH_MAX = ARP_CH_1_TO_10_SPLIT_48;
constexpr uint8_t DRUM_AFTERTOUCH_MIN_VELOCITY = 42;  // 33% floor.

decltype(Serial2) &DinSerial = Serial2;
MIDI_CREATE_INSTANCE(HardwareSerial, Serial2, DinMIDI);
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
void saveStorage();
void onInputNote(uint8_t sourcePort, uint8_t channel1, uint8_t note, uint8_t velocity, bool on,
                 bool recordForLoop = true);
void routeIncomingChannelMessage(uint8_t sourcePort, uint8_t status, uint8_t data1, uint8_t data2);
void loopAllOff();
void tickLooperAt(uint64_t now);
void tickLooper();
void clearSavedLoopStorage();
void saveLoopStorageIfAny();
void loadSavedLoopStorage();
void capturePhysicalHeldNotesForOverdub();
bool insertLoopEvent(uint32_t atMs, uint8_t channel1, uint8_t note, uint8_t velocity, bool on);
bool insertLoopEventLimited(uint32_t atMs, uint8_t channel1, uint8_t note, uint8_t velocity, bool on, uint32_t limitMs);
void noteThrough(uint8_t sourcePort, uint8_t inNote, uint8_t velocity, bool on);
void noteArpOffPassthrough(uint8_t sourcePort, uint8_t inNote, uint8_t velocity, bool on);
void thruOutputRefOn(uint8_t sourcePort, uint8_t outNote, uint8_t velocity);
void thruOutputRefOff(uint8_t sourcePort, uint8_t outNote);
void updateBassVoice();

enum MenuMode : uint8_t {
  MENU_SELECT = 0,
  MENU_EDIT = 1
};

enum RouterEditStage : uint8_t {
  ROUTER_STAGE_LIST = 0,
  ROUTER_STAGE_DEST,
  ROUTER_STAGE_TRANSPOSE
};

enum SettingId : uint8_t {
  SET_BPM = 0,
  SET_ARP_MODE,
  SET_DIVISION,
  SET_VELOCITY,
  SET_LENGTH,
  SET_PATTERN,
  SET_INPUT_CH,
  SET_ARP_OUT_CH,
  SET_BASS_CH,
  SET_THRU_OUT_CH,
  SET_RND_RBN,
  SET_ROUTER,
  SET_DIV_NOTES,
  SET_MAP_CC,
  SET_CC_OUT_CH,
  SET_LEGATO_CH,
  SET_REMOTE_CH,
  SET_REMOTE1,
  SET_REMOTE2,
  SET_SCREEN_SAVER,
  SET_SENSOR_CH,
  SET_SENSOR_MODE,
  SET_PUSH_MODE,
  SET_LOOP_BARS,
  SET_FORCE_KEY,
  SET_FORCE_SCALE,
  SET_GUITAR_PIANO,
  SET_LOAD_PRESET,
  SET_SAVE_PRESET,
  SET_PANIC,
  SETTING_COUNT
};

enum MapCcParamId : uint8_t {
  MAPCC_BPM = 0,
  MAPCC_ARP,
  MAPCC_DIV,
  MAPCC_VEL,
  MAPCC_LEN,
  MAPCC_IN,
  MAPCC_ACH,
  MAPCC_BCH,
  MAPCC_TCH,
  MAPCC_INCC,
  MAPCC_EYCH,
  MAPCC_EYMD,
  MAPCC_PSMD,
  MAPCC_KEY,
  MAPCC_SCL,
  MAPCC_LOOP,
  MAPCC_LOAD,
  MAPCC_PARAM_COUNT
};

constexpr uint8_t MAP_CC_CLEAR_SLOT = MAPCC_PARAM_COUNT;
constexpr uint8_t MAP_CC_CHMODE_SLOT = MAPCC_PARAM_COUNT + 1;
constexpr uint8_t MAP_CC_SLOT_COUNT = MAPCC_PARAM_COUNT + 2;

enum LoopBarsId : uint8_t {
  LOOP_BARS_1 = 0,
  LOOP_BARS_2,
  LOOP_BARS_4,
  LOOP_BARS_8,
  LOOP_BARS_FREE,
  LOOP_BARS_COUNT
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
  ARP_SELECTION_COUNT
};

enum DivisionId : uint8_t {
  DIV_1_1 = 0,
  DIV_1_2,
  DIV_1_2T,
  DIV_1_4,
  DIV_1_4T,
  DIV_1_8,
  DIV_1_8T,
  DIV_1_16,
  DIV_1_16T,
  DIV_1_32,
  DIV_1_32T,
  DIV_1_64,
  DIVISION_COUNT
};

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
  uint8_t pattern;
  uint8_t inputChannel;
  uint8_t arpOutChannel;
  uint8_t bassMode;
  uint8_t thruOutChannel;
  uint16_t roundRobinMask;
  uint16_t routerActiveMask;
  uint8_t routerOutChannels[16];
  int8_t routerTranspose[16];
  uint8_t ccOutChannel;
  uint8_t remoteChannel;
  uint8_t sensorChannel;
  uint8_t sensorMode;
  uint8_t forceKey;
  uint8_t forceScale;
  uint8_t instrumentView;
  uint8_t remote1Action;
  uint8_t remote2Action;
  uint8_t loadPreset;
  uint8_t savePreset;
  uint8_t reserved;
  uint8_t screenSaver;
  uint8_t divNoteChannels[DIV_NOTE_SLOT_COUNT];
  uint8_t divNoteNotes[DIV_NOTE_SLOT_COUNT];
  uint8_t divNotePlusNote;
  uint8_t pushMode;
  uint8_t loopBars;
  uint8_t loopAutoOverdub;
  uint8_t legatoChannel;
  uint8_t mapCcChannels[MAPCC_PARAM_COUNT];
  uint8_t mapCcNumbers[MAPCC_PARAM_COUNT];
  uint8_t mapCcChannelMode;
};

struct SettingsV8 {
  uint16_t manualBpm;
  uint8_t arpMode;
  uint8_t division;
  uint8_t arpVelocity;
  uint8_t arpLengthPct;
  uint8_t pattern;
  uint8_t inputChannel;
  uint8_t arpOutChannel;
  uint8_t bassMode;
  uint8_t thruOutChannel;
  uint16_t roundRobinMask;
  uint8_t roundRobinCh10To1;
  uint8_t ccOutChannel;
  uint8_t remoteChannel;
  uint8_t sensorChannel;
  uint8_t sensorMode;
  uint8_t forceKey;
  uint8_t forceScale;
  uint8_t instrumentView;
  uint8_t remote1Action;
  uint8_t remote2Action;
  uint8_t loadPreset;
  uint8_t savePreset;
  uint8_t reserved;
  uint8_t screenSaver;
  uint8_t divNoteChannels[DIV_NOTE_SLOT_COUNT];
  uint8_t divNoteNotes[DIV_NOTE_SLOT_COUNT];
  uint8_t divNotePlusNote;
  uint8_t pushMode;
  uint8_t loopBars;
  uint8_t loopAutoOverdub;
  uint8_t legatoChannel;
  uint8_t mapCcChannels[MAPCC_PARAM_COUNT];
  uint8_t mapCcNumbers[MAPCC_PARAM_COUNT];
  uint8_t mapCcChannelMode;
};

struct SettingsV7 {
  uint16_t manualBpm;
  uint8_t arpMode;
  uint8_t division;
  uint8_t arpVelocity;
  uint8_t arpLengthPct;
  uint8_t pattern;
  uint8_t inputChannel;
  uint8_t arpOutChannel;
  uint8_t bassMode;
  uint8_t thruOutChannel;
  uint16_t roundRobinMask;
  uint8_t ccOutChannel;
  uint8_t remoteChannel;
  uint8_t sensorChannel;
  uint8_t sensorMode;
  uint8_t forceKey;
  uint8_t forceScale;
  uint8_t instrumentView;
  uint8_t remote1Action;
  uint8_t remote2Action;
  uint8_t loadPreset;
  uint8_t savePreset;
  uint8_t reserved;
  uint8_t screenSaver;
  uint8_t divNoteChannels[DIV_NOTE_SLOT_COUNT];
  uint8_t divNoteNotes[DIV_NOTE_SLOT_COUNT];
  uint8_t divNotePlusNote;
  uint8_t pushMode;
  uint8_t loopBars;
  uint8_t loopAutoOverdub;
  uint8_t legatoChannel;
  uint8_t mapCcChannels[MAPCC_PARAM_COUNT];
  uint8_t mapCcNumbers[MAPCC_PARAM_COUNT];
  uint8_t mapCcChannelMode;
};

struct SettingsV5 {
  uint16_t manualBpm;
  uint8_t arpMode;
  uint8_t division;
  uint8_t arpVelocity;
  uint8_t arpLengthPct;
  uint8_t pattern;
  uint8_t inputChannel;
  uint8_t arpOutChannel;
  uint8_t bassMode;
  uint8_t thruOutChannel;
  uint8_t roundRobinRange;
  uint8_t ccOutChannel;
  uint8_t remoteChannel;
  uint8_t sensorChannel;
  uint8_t sensorMode;
  uint8_t forceKey;
  uint8_t forceScale;
  uint8_t instrumentView;
  uint8_t remote1Action;
  uint8_t remote2Action;
  uint8_t loadPreset;
  uint8_t savePreset;
  uint8_t reserved;
  uint8_t screenSaver;
  uint8_t divNoteChannels[DIV_NOTE_SLOT_COUNT];
  uint8_t divNoteNotes[DIV_NOTE_SLOT_COUNT];
  uint8_t divNotePlusNote;
  uint8_t pushMode;
  uint8_t loopBars;
  uint8_t legatoChannel;
  uint8_t mapCcChannels[MAPCC_PARAM_COUNT_V5];
  uint8_t mapCcNumbers[MAPCC_PARAM_COUNT_V5];
  uint8_t mapCcChannelMode;
};

struct SettingsV6 {
  uint16_t manualBpm;
  uint8_t arpMode;
  uint8_t division;
  uint8_t arpVelocity;
  uint8_t arpLengthPct;
  uint8_t pattern;
  uint8_t inputChannel;
  uint8_t arpOutChannel;
  uint8_t bassMode;
  uint8_t thruOutChannel;
  uint16_t roundRobinMask;
  uint8_t ccOutChannel;
  uint8_t remoteChannel;
  uint8_t sensorChannel;
  uint8_t sensorMode;
  uint8_t forceKey;
  uint8_t forceScale;
  uint8_t instrumentView;
  uint8_t remote1Action;
  uint8_t remote2Action;
  uint8_t loadPreset;
  uint8_t savePreset;
  uint8_t reserved;
  uint8_t screenSaver;
  uint8_t divNoteChannels[DIV_NOTE_SLOT_COUNT];
  uint8_t divNoteNotes[DIV_NOTE_SLOT_COUNT];
  uint8_t divNotePlusNote;
  uint8_t pushMode;
  uint8_t loopBars;
  uint8_t loopAutoOverdub;
  uint8_t legatoChannel;
  uint8_t mapCcChannels[MAPCC_PARAM_COUNT_V5];
  uint8_t mapCcNumbers[MAPCC_PARAM_COUNT_V5];
  uint8_t mapCcChannelMode;
};

struct SettingsV4 {
  uint16_t manualBpm;
  uint8_t arpMode;
  uint8_t division;
  uint8_t arpVelocity;
  uint8_t arpLengthPct;
  uint8_t pattern;
  uint8_t inputChannel;
  uint8_t arpOutChannel;
  uint8_t bassMode;
  uint8_t thruOutChannel;
  uint8_t ccOutChannel;
  uint8_t remoteChannel;
  uint8_t sensorChannel;
  uint8_t sensorMode;
  uint8_t forceKey;
  uint8_t forceScale;
  uint8_t instrumentView;
  uint8_t remote1Action;
  uint8_t remote2Action;
  uint8_t loadPreset;
  uint8_t savePreset;
  uint8_t reserved;
  uint8_t screenSaver;
  uint8_t divNoteChannels[DIV_NOTE_SLOT_COUNT];
  uint8_t divNoteNotes[DIV_NOTE_SLOT_COUNT];
  uint8_t divNotePlusNote;
  uint8_t pushMode;
  uint8_t legatoChannel;
  uint8_t mapCcChannels[MAPCC_PARAM_COUNT_V5];
  uint8_t mapCcNumbers[MAPCC_PARAM_COUNT_V5];
  uint8_t mapCcChannelMode;
};

struct SettingsV2 {
  uint16_t manualBpm;
  uint8_t arpMode;
  uint8_t division;
  uint8_t arpVelocity;
  uint8_t arpLengthPct;
  uint8_t pattern;
  uint8_t inputChannel;
  uint8_t arpOutChannel;
  uint8_t bassMode;
  uint8_t thruOutChannel;
  uint8_t ccOutChannel;
  uint8_t remoteChannel;
  uint8_t sensorChannel;
  uint8_t sensorMode;
  uint8_t forceKey;
  uint8_t forceScale;
  uint8_t instrumentView;
  uint8_t remote1Action;
  uint8_t remote2Action;
  uint8_t loadPreset;
  uint8_t savePreset;
  uint8_t reserved;
  uint8_t screenSaver;
  uint8_t divNoteChannels[DIV_NOTE_SLOT_COUNT];
  uint8_t divNoteNotes[DIV_NOTE_SLOT_COUNT];
  uint8_t divNotePlusNote;
  uint8_t pushMode;
  uint8_t legatoChannel;
};

struct SettingsV1 {
  uint16_t manualBpm;
  uint8_t arpMode;
  uint8_t division;
  uint8_t arpVelocity;
  uint8_t arpLengthPct;
  uint8_t pattern;
  uint8_t inputChannel;
  uint8_t arpOutChannel;
  uint8_t bassMode;
  uint8_t thruOutChannel;
  uint8_t ccOutChannel;
  uint8_t remoteChannel;
  uint8_t sensorChannel;
  uint8_t sensorMode;
  uint8_t forceKey;
  uint8_t forceScale;
  uint8_t instrumentView;
  uint8_t remote1Action;
  uint8_t remote2Action;
  uint8_t loadPreset;
  uint8_t savePreset;
  uint8_t reserved;
  uint8_t screenSaver;
  uint8_t divNoteChannels[DIV_NOTE_SLOT_COUNT];
  uint8_t divNoteNotes[DIV_NOTE_SLOT_COUNT];
  uint8_t divNotePlusNote;
  uint8_t pushMode;
};

struct StorageImage {
  uint16_t magic;
  uint8_t currentPreset;
  Settings presets[PRESET_COUNT];
};

struct StorageImageV8 {
  uint16_t magic;
  uint8_t currentPreset;
  SettingsV8 presets[PRESET_COUNT];
};

struct StorageImageV7 {
  uint16_t magic;
  uint8_t currentPreset;
  SettingsV7 presets[PRESET_COUNT];
};

struct StorageImageV1 {
  uint16_t magic;
  uint8_t currentPreset;
  SettingsV1 presets[PRESET_COUNT];
};

struct StorageImageV2 {
  uint16_t magic;
  uint8_t currentPreset;
  SettingsV2 presets[PRESET_COUNT];
};

struct StorageImageV4 {
  uint16_t magic;
  uint8_t currentPreset;
  SettingsV4 presets[PRESET_COUNT];
};

struct StorageImageV5 {
  uint16_t magic;
  uint8_t currentPreset;
  SettingsV5 presets[PRESET_COUNT];
};

struct StorageImageV6 {
  uint16_t magic;
  uint8_t currentPreset;
  SettingsV6 presets[PRESET_COUNT];
};

struct PackedLoopEvent {
  uint16_t at10ms;
  uint8_t channel;
  uint8_t note;
  uint8_t velocity;
  uint8_t flags;
};

struct LoopStorageImage {
  uint16_t magic;
  uint16_t count;
  uint32_t lengthMs;
  uint32_t activeLengthMs;
  uint8_t bars;
  uint8_t reserved[3];
  PackedLoopEvent events[MAX_LOOP_EVENTS];
};

constexpr size_t LOOP_STORAGE_OFFSET = 4096;
constexpr size_t UI_SCREEN_STORAGE_OFFSET = LOOP_STORAGE_OFFSET - 2;
constexpr uint8_t UI_SCREEN_STORAGE_MAGIC = 0xA7;
constexpr uint32_t UI_SCREEN_SAVE_IDLE_MS = 2000UL;
static_assert(sizeof(StorageImage) <= UI_SCREEN_STORAGE_OFFSET,
              "Preset storage overlaps saved UI state");
static_assert(LOOP_STORAGE_OFFSET + sizeof(LoopStorageImage) <= EEPROM_BYTES,
              "EEPROM_BYTES too small for presets plus saved loop");

struct ClockTracker {
  uint32_t lastClockMicros = 0;
  uint32_t lastDinClockMs = 0;
  uint8_t pulseCount = 0;
  float bpm = 120.0f;
};

struct ButtonPulse {
  bool active = false;
  bool isCc = false;
  uint8_t number = 0;
  uint32_t offAtMs = 0;
};

struct CcMapSlot {
  uint8_t cc = 0xFF;
  uint8_t channel = 0;
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
  bool dirty = true;
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

StorageImage storage;
Settings settings;
ClockTracker clockTracker;
EncoderState encoder;
SensorRuntime sensorRt;
PushRuntime pushRt;
UiState ui;
uint8_t persistedUiSetting = SET_BPM;
uint8_t observedUiSetting = SET_BPM;
bool uiScreenSavePending = false;
uint32_t uiScreenChangedMs = 0;

uint8_t drumAftertouchPressure = 127;

bool heldInputNotes[128];
uint8_t heldVelocities[128];
bool physicalHeldInputNotes[128];
uint8_t physicalHeldVelocities[128];
bool loopHeldInputNotes[128];
uint8_t loopHeldVelocities[128];
bool arpLatchedNotes[128];
uint8_t arpLatchedVelocities[128];
bool thruLatchedNotes[128];
bool heldDrumNotes[128];
uint8_t heldDrumVelocities[128];
bool physicalHeldDrumNotes[128];
uint8_t physicalHeldDrumVelocities[128];
bool loopHeldDrumNotes[128];
uint8_t loopHeldDrumVelocities[128];
uint8_t mappedThruNotes[128];
uint8_t mappedLoopThruNotes[128];
uint8_t mappedArpOffNotes[128];
uint8_t mappedArpOffChannels[128];
uint8_t mappedLoopArpOffNotes[128];
uint8_t mappedLoopArpOffChannels[128];
uint8_t thruOutputRefCount[128];
uint8_t arpOffOutputRefCount[128];
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
uint32_t arpGlobalStep = 0;
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
uint8_t mapCcCursor = 0;
CcMapSlot mapCcSlots[MAPCC_PARAM_COUNT];
bool mapCcChannelAll = false;
bool mapCcTouchedSetting[SETTING_COUNT];
int16_t mapCcPersistBaseline[SETTING_COUNT];
bool mapCcUiPending = false;
uint32_t mapCcUiLastMoveMs = 0;
uint8_t mapCcUiTargetSetting = SET_BPM;
bool mapCcPreviewActive = false;
uint8_t mapCcPreviewSetting = SET_BPM;
int16_t mapCcPreviewValue = 0;
bool mapCcLoadInProgress = false;
bool mapCcDeferredActive[MAPCC_PARAM_COUNT];
int16_t mapCcDeferredValue[MAPCC_PARAM_COUNT];
uint32_t mapCcDeferredLastMoveMs[MAPCC_PARAM_COUNT];
bool screenSaverSeeded = false;
bool screenSaverForceNow = false;

struct LoopEvent {
  uint32_t atMs = 0;
  uint8_t channel = 1;
  uint8_t note = 0;
  uint8_t velocity = 0;
  bool on = false;
};

LoopEvent loopEvents[MAX_LOOP_EVENTS];
uint16_t loopEventCount = 0;
uint16_t loopPlayIndex = 0;
uint64_t loopStartUs = 0;
uint64_t loopLengthUs = 0;
uint32_t loopLengthMs = 0;
uint32_t loopStoredLengthMs = 0;
bool loopHasData = false;
bool loopRecordingArmed = false;
bool loopRecording = false;
bool loopPlaying = false;
bool loopOverdubbing = false;
bool loopSdCloseState = false;
bool loopDeleteArmed = false;
uint32_t loopSdLastTriggerMs = 0;
uint32_t sensorLoopReleaseStartMs = 0;
uint32_t pushLoopReleaseStartMs = 0;
bool loopOverdubHeld[16][128];
uint8_t loopOverdubHeldVelocity[16][128];
uint32_t loopOverdubStartMs[16][128];
uint8_t loopOverdubWrapped[16][16];
uint8_t loopOverdubFullCycle[16][16];
uint8_t loopPlaybackHeld[16][16];
bool loopHiddenForReplace = false;
bool loopReplaceArmed = false;
uint32_t loopRecPlayLastSwipeMs = 0;
const int8_t kEncoderTransitionTable[16] = {
  0,  1, -1,  0,
 -1,  0,  0,  1,
  1,  0,  0, -1,
  0, -1,  1,  0
};

const char *const kSettingNames[SETTING_COUNT] = {
  "1 BPM", "2 ARP MODE", "3 DIVISION", "4 VELOCITY", "5 LENGTH", "",
  "6 INPUT CH", "7 ARP CH", "8 BASS CH", "9 THRU OUT", "10 RNDRBN", "11 ROUTER",
  "12 DIV NOTE", "13 MAP CC", "14 IN CC >", "15 MONO RETRIG", "16 REMOTE", "17 REMOTE 1",
  "18 REMOTE 2", "19 SCRNSVR", "20 EYE/PUSH", "21 EYE MODE", "22 PUSH",
  "23 LOOP", "24 KEY", "25 SCALE", "26 GIT/KEYS", "27 LOAD", "28 SAVE", "29 PANIC"
};

const char *const kArpModeNames[ARP_MODE_COUNT] = {
  "UP", "DOWN", "UP-DOWN 1", "UP-DOWN 2", "TRIGGER", "RANDOM", "OFF"
};

const char *const kArpSelectionNames[ARP_SELECTION_COUNT] = {
  "OFF", "UP", "DOWN", "UP-DOWN 1", "UP-DOWN 2", "TRIGGER", "RANDOM",
  "UP 1-OCT",
  "RHYTHM", "OSTINATO", "OCT WALK", "FIFTH", "BASS+CHORD", "CHORD+RUN"
};

const char *const kDivisionNames[DIVISION_COUNT] = {
  "1/1", "1/2", "1/2T", "1/4", "1/4T", "1/8", "1/8T",
  "1/16", "1/16T", "1/32", "1/32T", "1/64"
};

const float kDivisionQuarterSteps[DIVISION_COUNT] = {
  4.0f, 2.0f, 4.0f / 3.0f, 1.0f, 2.0f / 3.0f, 0.5f, 1.0f / 3.0f,
  0.25f, 1.0f / 6.0f, 0.125f, 1.0f / 12.0f, 0.0625f
};

constexpr uint16_t MUSICAL_PPQN = 48;
const uint16_t kDivisionPulseSteps[DIVISION_COUNT] = {
  192, 96, 64, 48, 32, 24, 16, 12, 8, 6, 4, 3
};

const char *const kPatternNames[PATTERN_COUNT] = {
  "MODE", "UP 1-OCT", "DOWN", "UP-DOWN 1", "UP-DOWN 2", "RANDOM", "TRIGGER",
  "RHYTHM", "OSTINATO", "OCT WALK", "FIFTH", "BASS+CHORD", "CHORD+RUN"
};

const char *const kForceScaleNames[FORCE_SCALE_COUNT] = {
  "OFF", "MAJOR", "MINOR", "MAJ+MIN", "BLUES", "MAJ BLUES",
  "BLUES+BOTH", "HARM MIN", "MEL MIN"
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

const char *const kLoopBarsNames[LOOP_BARS_COUNT] = {
  "1 BAR", "2 BAR", "4 BAR", "8 BAR", "FREE"
};

const char *const kMapCcParamNames[MAPCC_PARAM_COUNT] = {
  "1 BPM", "2 ARP", "3 DIV", "4 VEL", "5 LEN", "6 IN", "7 ACH", "8 BCH",
  "9 TCH", "10 INCC", "11 EYCH", "12 EYMD", "13 PSMD", "14 KEY", "15 SCL", "16 LOOP", "17 LOAD"
};

const uint8_t kMapCcTargets[MAPCC_PARAM_COUNT] = {
  SET_BPM, SET_ARP_MODE, SET_DIVISION, SET_VELOCITY, SET_LENGTH, SET_INPUT_CH, SET_ARP_OUT_CH, SET_BASS_CH,
  SET_THRU_OUT_CH, SET_CC_OUT_CH, SET_SENSOR_CH, SET_SENSOR_MODE, SET_PUSH_MODE, SET_FORCE_KEY, SET_FORCE_SCALE,
  SET_LOOP_BARS, SET_LOAD_PRESET
};

const uint8_t kMapCcSensorAllowedModes[] = {
  SENSOR_OFF,
  SENSOR_PARAM_PLUS2, SENSOR_PARAM_MINUS2, SENSOR_PARAM_PLUS3, SENSOR_PARAM_MINUS3, SENSOR_PARAM_FULL, SENSOR_DIV3,
  SENSOR_VEL_DOWN, SENSOR_LEN_DOWN,
  SENSOR_ARP_LATCH, SENSOR_ARP_LATCH_PLUS, SENSOR_ARP_FREEZE, SENSOR_ARP_FREEZ_PLUS,
  SENSOR_PITCH_UP, SENSOR_PITCH_DOWN,
  SENSOR_NOTES_C0, SENSOR_NOTES_C1, SENSOR_NOTES_C2, SENSOR_NOTES_C3, SENSOR_NOTES_C4, SENSOR_NOTES_C5, SENSOR_NOTES_C6, SENSOR_NOTES_C7
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
  return (s.mapCcChannelMode & MAP_CC_RR_CH10_TO_1_BIT) != 0;
}

bool roundRobinCh10To1Enabled() {
  return roundRobinCh10To1Enabled(settings);
}

bool roundRobinCh10To2Enabled(const Settings &s) {
  return (s.mapCcChannelMode & MAP_CC_RR_CH10_TO_2_BIT) != 0;
}

bool roundRobinCh10To2Enabled() {
  return roundRobinCh10To2Enabled(settings);
}

void setRoundRobinCh10To1(Settings &s, bool enabled) {
  if (enabled) {
    s.mapCcChannelMode |= MAP_CC_RR_CH10_TO_1_BIT;
    s.mapCcChannelMode &= static_cast<uint8_t>(~MAP_CC_RR_CH10_TO_2_BIT);
  } else {
    s.mapCcChannelMode &= static_cast<uint8_t>(~MAP_CC_RR_CH10_TO_1_BIT);
  }
}

void setRoundRobinCh10To2(Settings &s, bool enabled) {
  if (enabled) {
    s.mapCcChannelMode |= MAP_CC_RR_CH10_TO_2_BIT;
    s.mapCcChannelMode &= static_cast<uint8_t>(~MAP_CC_RR_CH10_TO_1_BIT);
  } else {
    s.mapCcChannelMode &= static_cast<uint8_t>(~MAP_CC_RR_CH10_TO_2_BIT);
  }
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
    const int outNote = static_cast<int>(data1) + settings.routerTranspose[idx];
    if (outNote < 0 || outNote > 127) return false;
    data1 = static_cast<uint8_t>(outNote);
  }
  status = type | ((outCh - 1) & 0x0F);
  return true;
}

bool arpChannelSpecialMode() {
  return settings.arpOutChannel >= ARP_CH_1_PLUS_10;
}

bool arpChannelAftertouchMode() {
  return settings.arpOutChannel == ARP_CH_1_PLUS_10_AFTERTOUCH;
}

bool arpChannelSplitMode() {
  return settings.arpOutChannel == ARP_CH_1_TO_10_SPLIT_24 ||
         settings.arpOutChannel == ARP_CH_1_TO_10_SPLIT_36 ||
         settings.arpOutChannel == ARP_CH_1_TO_10_SPLIT_48;
}

uint8_t splitDrumStartNote() {
  if (settings.arpOutChannel == ARP_CH_1_TO_10_SPLIT_24) return 24;
  if (settings.arpOutChannel == ARP_CH_1_TO_10_SPLIT_36) return 36;
  return 48;
}

uint8_t splitDrumOutputNote(uint8_t note) {
  if (settings.arpOutChannel == ARP_CH_1_TO_10_SPLIT_24) return static_cast<uint8_t>(note + 12);
  if (settings.arpOutChannel == ARP_CH_1_TO_10_SPLIT_48) return static_cast<uint8_t>(note - 12);
  return note;
}

uint8_t mainArpOutChannel() {
  return arpChannelSpecialMode() ? 1 : settings.arpOutChannel;
}

bool splitDrumInputNote(uint8_t note) {
  const uint8_t start = splitDrumStartNote();
  return note >= start && note < static_cast<uint8_t>(start + 8);
}

bool selectableSetting(uint8_t settingId) {
  return settingId != SET_PATTERN;
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
  return constrain(static_cast<int>(settings.manualBpm), 20, 300);
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

void sendDinRaw1(uint8_t status) {
  DinSerial.write(status);
}

void sendDinRaw2(uint8_t status, uint8_t data1) {
  DinSerial.write(status);
  DinSerial.write(data1);
}

uint64_t musicalDurationUs(uint64_t pulses) {
  return (pulses * 60000000ULL) /
         (static_cast<uint64_t>(currentBpm()) * MUSICAL_PPQN);
}

uint64_t divisionStepUs() {
  return musicalDurationUs(kDivisionPulseSteps[currentDivisionSetting()]);
}

uint32_t divisionStepMs() {
  return static_cast<uint32_t>((divisionStepUs() + 999ULL) / 1000ULL);
}

bool settingNeedsPanic(uint8_t settingId) {
  return settingId == SET_INPUT_CH || settingId == SET_ARP_OUT_CH ||
         settingId == SET_BASS_CH || settingId == SET_THRU_OUT_CH ||
         settingId == SET_LEGATO_CH ||
         settingId == SET_FORCE_KEY || settingId == SET_FORCE_SCALE ||
         settingId == SET_SENSOR_CH || settingId == SET_SENSOR_MODE ||
         settingId == SET_PUSH_MODE;
}

String midiChannelLabel(uint8_t value, bool allowOff = false) {
  if (allowOff && value == 0) return "OFF";
  if (value == ARP_CH_1_PLUS_10) return "1+10";
  if (value == ARP_CH_1_PLUS_10_AFTERTOUCH) return "1+10-A";
  if (value == ARP_CH_1_TO_10_SPLIT_24) return "1-10 24";
  if (value == ARP_CH_1_TO_10_SPLIT_36) return "1-10 36";
  if (value == ARP_CH_1_TO_10_SPLIT_48) return "1-10 48";
  return "CH " + String(value);
}

String ccChannelLabel(uint8_t value) {
  if (value == 17) return "ALL3";
  return midiChannelLabel(value);
}

uint8_t divNoteSlotToDivision(uint8_t slot) {
  return DIV_1_4 + slot;
}

bool isMapCcTargetSetting(uint8_t settingId) {
  for (uint8_t i = 0; i < MAPCC_PARAM_COUNT; ++i) {
    if (kMapCcTargets[i] == settingId) return true;
  }
  return false;
}

int16_t mapCcSettingFieldValue(const Settings &s, uint8_t settingId) {
  switch (settingId) {
    case SET_BPM: return s.manualBpm;
    case SET_ARP_MODE: return s.arpMode;
    case SET_DIVISION: return s.division;
    case SET_VELOCITY: return s.arpVelocity;
    case SET_LENGTH: return s.arpLengthPct;
    case SET_INPUT_CH: return s.inputChannel;
    case SET_ARP_OUT_CH: return s.arpOutChannel;
    case SET_BASS_CH: return s.bassMode;
    case SET_THRU_OUT_CH: return s.thruOutChannel;
    case SET_CC_OUT_CH: return s.ccOutChannel;
    case SET_SENSOR_CH: return s.sensorChannel;
    case SET_SENSOR_MODE: return s.sensorMode;
    case SET_PUSH_MODE: return s.pushMode;
    case SET_LOOP_BARS: return s.loopBars;
    case SET_FORCE_KEY: return s.forceKey;
    case SET_FORCE_SCALE: return s.forceScale;
    case SET_LOAD_PRESET: return s.loadPreset;
    default: return 0;
  }
}

void mapCcApplyPersistBaseline(Settings &s, uint8_t settingId, int16_t value) {
  switch (settingId) {
    case SET_BPM: s.manualBpm = constrain(value, 20, 300); break;
    case SET_ARP_MODE: s.arpMode = clampU8(value, 0, ARP_SELECTION_COUNT - 1); break;
    case SET_DIVISION: s.division = clampU8(value, 0, DIVISION_COUNT - 1); break;
    case SET_VELOCITY: s.arpVelocity = clampU8(value, 1, 127); break;
    case SET_LENGTH: s.arpLengthPct = clampU8(value, 1, 100); break;
    case SET_INPUT_CH: s.inputChannel = clampU8(value, 1, 16); break;
    case SET_ARP_OUT_CH: s.arpOutChannel = clampU8(value, 0, ARP_CH_MAX); break;
    case SET_BASS_CH: s.bassMode = clampU8(value, 0, 48); break;
    case SET_THRU_OUT_CH: s.thruOutChannel = clampU8(value, 0, 16); break;
    case SET_CC_OUT_CH: s.ccOutChannel = clampU8(value, 1, 17); break;
    case SET_SENSOR_CH: s.sensorChannel = clampU8(value, 1, 16); break;
    case SET_SENSOR_MODE: s.sensorMode = clampU8(value, 0, SENSOR_MODE_COUNT - 1); break;
    case SET_PUSH_MODE: s.pushMode = clampU8(value, 0, SENSOR_MODE_COUNT - 1); break;
    case SET_LOOP_BARS: s.loopBars = clampU8(value, 0, LOOP_BARS_COUNT - 1); break;
    case SET_FORCE_KEY: s.forceKey = clampU8(value, 0, 24); break;
    case SET_FORCE_SCALE: s.forceScale = clampU8(value, 0, FORCE_SCALE_COUNT - 1); break;
    case SET_LOAD_PRESET: s.loadPreset = clampU8(value, 0, PRESET_COUNT - 1); break;
    default: break;
  }
}

void captureMapCcPersistBaseline(const Settings &source = settings) {
  memset(mapCcTouchedSetting, 0, sizeof(mapCcTouchedSetting));
  for (uint8_t i = 0; i < SETTING_COUNT; ++i) mapCcPersistBaseline[i] = 0;
  for (uint8_t i = 0; i < MAPCC_PARAM_COUNT; ++i) {
    const uint8_t settingId = kMapCcTargets[i];
    mapCcPersistBaseline[settingId] = mapCcSettingFieldValue(source, settingId);
  }
}

void syncMapCcRuntimeFromSettings() {
  for (uint8_t i = 0; i < MAPCC_PARAM_COUNT; ++i) {
    mapCcSlots[i].channel = settings.mapCcChannels[i];
    mapCcSlots[i].cc = settings.mapCcNumbers[i];
  }
  mapCcChannelAll = (settings.mapCcChannelMode & MAP_CC_CHANNEL_ALL_BIT) != 0;
}

void syncMapCcRuntimeToSettings() {
  for (uint8_t i = 0; i < MAPCC_PARAM_COUNT; ++i) {
    settings.mapCcChannels[i] = mapCcSlots[i].channel;
    settings.mapCcNumbers[i] = mapCcSlots[i].cc;
  }
  settings.mapCcChannelMode = (settings.mapCcChannelMode & (MAP_CC_RR_CH10_TO_1_BIT | MAP_CC_RR_CH10_TO_2_BIT)) |
                              (mapCcChannelAll ? MAP_CC_CHANNEL_ALL_BIT : 0);
}

void clearMapCcMappings() {
  for (uint8_t i = 0; i < MAPCC_PARAM_COUNT; ++i) {
    mapCcSlots[i].cc = 0xFF;
    mapCcSlots[i].channel = 0;
    mapCcDeferredActive[i] = false;
    mapCcDeferredValue[i] = 0;
    mapCcDeferredLastMoveMs[i] = 0;
  }
  mapCcPreviewActive = false;
  syncMapCcRuntimeToSettings();
}

int16_t mapCcLinearValue(uint8_t ccValue, int16_t minValue, int16_t maxValue) {
  if (maxValue <= minValue) return minValue;
  const int32_t span = static_cast<int32_t>(maxValue) - static_cast<int32_t>(minValue);
  return static_cast<int16_t>(minValue + ((static_cast<int32_t>(ccValue) * span + 63) / 127));
}

int16_t mapCcSensorModeValue(uint8_t ccValue) {
  const uint8_t count = sizeof(kMapCcSensorAllowedModes) / sizeof(kMapCcSensorAllowedModes[0]);
  const uint8_t idx = static_cast<uint8_t>(mapCcLinearValue(ccValue, 0, count - 1));
  return kMapCcSensorAllowedModes[idx];
}

int16_t mapCcTargetValue(uint8_t paramId, uint8_t ccValue) {
  switch (paramId) {
    case MAPCC_BPM:
      return static_cast<int16_t>(20 + (mapCcLinearValue(ccValue, 0, 56) * 5));
    case MAPCC_ARP: return mapCcLinearValue(ccValue, 0, ARP_SELECTION_COUNT - 1);
    case MAPCC_DIV: return mapCcLinearValue(ccValue, 0, DIVISION_COUNT - 1);
    case MAPCC_VEL: return mapCcLinearValue(ccValue, 1, 127);
    case MAPCC_LEN: return mapCcLinearValue(ccValue, 1, 100);
    case MAPCC_IN: return mapCcLinearValue(ccValue, 1, 16);
    case MAPCC_ACH: return mapCcLinearValue(ccValue, 0, ARP_CH_MAX);
    case MAPCC_BCH: return mapCcLinearValue(ccValue, 0, 48);
    case MAPCC_TCH: return mapCcLinearValue(ccValue, 0, 16);
    case MAPCC_INCC: return mapCcLinearValue(ccValue, 1, 17);
    case MAPCC_EYCH: return mapCcLinearValue(ccValue, 1, 16);
    case MAPCC_EYMD: return mapCcSensorModeValue(ccValue);
    case MAPCC_PSMD: return mapCcSensorModeValue(ccValue);
    case MAPCC_KEY: return mapCcLinearValue(ccValue, 0, 24);
    case MAPCC_SCL: return mapCcLinearValue(ccValue, 0, FORCE_SCALE_COUNT - 1);
    case MAPCC_LOOP: return mapCcLinearValue(ccValue, LOOP_BARS_1, LOOP_BARS_8);
    case MAPCC_LOAD: return mapCcLinearValue(ccValue, 0, PRESET_COUNT - 1);
    default: return 0;
  }
}

bool mapCcDeferredParam(uint8_t paramId) {
  if (paramId >= MAPCC_IN && paramId <= MAPCC_PSMD) return true;
  return paramId == MAPCC_LOAD;
}

void queueMapCcUiSetting(uint8_t targetSetting, bool immediate) {
  if (immediate) {
    ui.selectedSetting = targetSetting;
    ui.menuMode = MENU_SELECT;
    ui.dirty = true;
    mapCcUiPending = false;
    return;
  }
  mapCcUiTargetSetting = targetSetting;
  mapCcUiLastMoveMs = millis();
  mapCcUiPending = true;
}

void setMapCcPreview(uint8_t settingId, int16_t value) {
  mapCcPreviewActive = true;
  mapCcPreviewSetting = settingId;
  mapCcPreviewValue = value;
}

void clearMapCcPreview(uint8_t settingId) {
  if (mapCcPreviewActive && mapCcPreviewSetting == settingId) mapCcPreviewActive = false;
}

void panicMappedRelevantOnly(uint8_t settingId, int16_t nextValue);

bool mapCcNeedsPanic(uint8_t settingId) {
  if (settingId == SET_FORCE_KEY || settingId == SET_FORCE_SCALE) return false;
  return settingNeedsPanic(settingId);
}

void applyMappedSettingChange(uint8_t settingId, int16_t nextValue, bool allowArpRestart,
                              bool immediateUiFocus) {
  const int16_t currentValue = getSettingValueRaw(settingId);
  if (nextValue == currentValue) {
    clearMapCcPreview(settingId);
    queueMapCcUiSetting(settingId, immediateUiFocus);
    return;
  }
  if (mapCcNeedsPanic(settingId)) panicMappedRelevantOnly(settingId, nextValue);
  setSettingValueRaw(settingId, nextValue);
  if (allowArpRestart &&
      (settingId == SET_DIVISION || settingId == SET_ARP_MODE || settingId == SET_LENGTH || settingId == SET_VELOCITY)) {
    restartArpTiming(true);
  }
  mapCcTouchedSetting[settingId] = true;
  clearMapCcPreview(settingId);
  queueMapCcUiSetting(settingId, immediateUiFocus);
}

void queueDeferredMappedChange(uint8_t paramId, int16_t mappedValue) {
  mapCcDeferredActive[paramId] = true;
  mapCcDeferredValue[paramId] = mappedValue;
  mapCcDeferredLastMoveMs[paramId] = millis();
  const uint8_t settingId = kMapCcTargets[paramId];
  setMapCcPreview(settingId, mappedValue);
  queueMapCcUiSetting(settingId, true);
}

void applyMappedLoadPreset(int16_t slot, bool immediateUiFocus) {
  const uint8_t nextPreset = clampU8(slot, 0, PRESET_COUNT - 1);
  if (nextPreset == storage.currentPreset) {
    queueMapCcUiSetting(SET_LOAD_PRESET, immediateUiFocus);
    return;
  }
  ui.selectedSetting = SET_LOAD_PRESET;
  ui.menuMode = MENU_SELECT;
  mapCcLoadInProgress = true;
  ui.dirty = true;
  renderDisplayIfNeeded();
  panicAll();
  storage.currentPreset = nextPreset;
  loadCurrentPreset();
  mapCcLoadInProgress = false;
  ui.selectedSetting = SET_LOAD_PRESET;
  ui.menuMode = MENU_SELECT;
  ui.dirty = true;
  clearMapCcPreview(SET_LOAD_PRESET);
  queueMapCcUiSetting(SET_LOAD_PRESET, immediateUiFocus);
}

void applyMappedCcAssignments(uint8_t channel, uint8_t control, uint8_t value) {
  for (uint8_t i = 0; i < MAPCC_PARAM_COUNT; ++i) {
    const CcMapSlot &slot = mapCcSlots[i];
    if (slot.cc > 127) continue;
    if (slot.cc != control) continue;
    if (!mapCcChannelAll && slot.channel != channel) continue;
    const uint8_t targetSetting = kMapCcTargets[i];
    const int16_t mappedValue = mapCcTargetValue(i, value);
    if (mapCcDeferredParam(i)) {
      queueDeferredMappedChange(i, mappedValue);
      continue;
    }
    const bool immediateUi = (i == MAPCC_BPM || i == MAPCC_KEY || i == MAPCC_SCL);
    const bool allowArpRestart = false;
    if (targetSetting == SET_LOAD_PRESET) applyMappedLoadPreset(mappedValue, immediateUi);
    else applyMappedSettingChange(targetSetting, mappedValue, allowArpRestart, immediateUi);
  }
}

bool captureMapCcAssignment(uint8_t channel, uint8_t control) {
  if (ui.selectedSetting != SET_MAP_CC || ui.menuMode != MENU_EDIT) return false;
  if (mapCcCursor < MAPCC_PARAM_COUNT) {
    mapCcSlots[mapCcCursor].channel = channel;
    mapCcSlots[mapCcCursor].cc = control;
  }
  ui.dirty = true;
  markActivity(false);
  return true;
}

void pollMapCcUiFocus() {
  const uint32_t now = millis();
  for (uint8_t paramId = MAPCC_IN; paramId <= MAPCC_LOAD; ++paramId) {
    if (!mapCcDeferredActive[paramId]) continue;
    if ((now - mapCcDeferredLastMoveMs[paramId]) < MAP_CC_DEFER_COMMIT_MS) continue;
    mapCcDeferredActive[paramId] = false;
    const uint8_t settingId = kMapCcTargets[paramId];
    const int16_t mappedValue = mapCcDeferredValue[paramId];
    if (settingId == SET_LOAD_PRESET) {
      applyMappedLoadPreset(mappedValue, true);
    } else {
      applyMappedSettingChange(settingId, mappedValue, false, true);
    }
  }

  if (!mapCcUiPending) return;
  if ((now - mapCcUiLastMoveMs) < MAP_CC_UI_SETTLE_MS) return;
  ui.selectedSetting = mapCcUiTargetSetting;
  ui.menuMode = MENU_SELECT;
  ui.dirty = true;
  mapCcUiPending = false;
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

String bassLabel(uint8_t value) {
  if (value == 0) return "OFF";
  const uint8_t channel = bassModeChannel(value);
  const int8_t octaves = bassModeOctaveOffset(value);
  if (octaves > 0) return "CH " + String(channel) + " +" + String(octaves) + " OCT";
  if (octaves == 0) return "CH " + String(channel) + " 0 OCT";
  return "CH " + String(channel) + " " + String(octaves) + " OCT";
}

String remoteActionLabel(uint8_t action) {
  if (action < 128) {
    uint8_t octave = action / 12;
    uint8_t note = action % 12;
    return String("NOTE ") + kNoteNames[note] + String(octave);
  }
  return String("CC ") + String(action - 128 + 1);
}

void drawRemoteActionScreen(uint8_t action) {
  display.setCursor(0, 0);
  if (action < 128) {
    const uint8_t octave = action / 12;
    const uint8_t note = action % 12;
    display.setTextSize(3);
    display.print(kNoteNames[note]);
    display.print(octave);
    display.setTextSize(2);
    display.setCursor(0, 26);
    display.print(action);
  } else {
    display.setTextSize(3);
    display.print(F("CC"));
    display.print(action - 128 + 1);
  }
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
    for (int i = 0; i < remaining.length(); ++i) {
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
  DinMIDI.sendNoteOn(note, velocity, channel1);
}

void sendDinNoteOff(uint8_t channel1, uint8_t note, uint8_t velocity = 0) {
  DinMIDI.sendNoteOff(note, velocity, channel1);
}

void sendDinCc(uint8_t channel1, uint8_t cc, uint8_t value) {
  DinMIDI.sendControlChange(cc, value, channel1);
}

void sendDinPitchBend(uint8_t channel1, int bend) {
  DinMIDI.sendPitchBend(bend, channel1);
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
  ch1 = (status & 0x0F) + 1;
  if (type == 0x90) {
    if (data2 == 0) sendDinNoteOff(ch1, data1, 0);
    else sendDinNoteOn(ch1, data1, data2);
  } else if (type == 0x80) {
    sendDinNoteOff(ch1, data1, data2);
  } else if (type == 0xB0) {
    sendDinCc(ch1, data1, data2);
  } else if (type == 0xC0) {
    DinMIDI.sendProgramChange(data1, ch1);
  } else if (type == 0xD0) {
    DinMIDI.sendAfterTouch(data1, ch1);
  } else if (type == 0xA0) {
    sendDinRaw2(status, data1);
    DinSerial.write(data2);
  } else if (type == 0xE0) {
    int bend = static_cast<int>(data1) | (static_cast<int>(data2) << 7);
    sendDinPitchBend(ch1, bend - 8192);
  } else if (status >= 0xF8) {
    sendDinRaw1(status);
  }
  sendMidiToUsbDevice(sourcePort, status, data1, data2);
}

void sendAllNoteOffChannel(uint8_t ch1) {
  if (!channelEnabled(ch1)) return;
  for (uint8_t note = 0; note < 128; ++note) {
    sendDinNoteOff(ch1, note, 0);
    sendMidiToUsbDevice(255, 0x80 | ((ch1 - 1) & 0x0F), note, 0);
  }
  sendFanout(255, 0xB0 | ((ch1 - 1) & 0x0F), 123, 0);
  sendFanout(255, 0xB0 | ((ch1 - 1) & 0x0F), 120, 0);
}

[[noreturn]] void rebootBoard(const __FlashStringHelper *reason = nullptr) {
  if (ui.selectedSetting < SETTING_COUNT && selectableSetting(ui.selectedSetting)) {
    storeUiResumeHint(ui.selectedSetting);
  }
  saveStorage();
  saveLoopStorageIfAny();
  showBootStage(F("Rebooting..."), reason);
  delay(80);
  watchdog_reboot(0, 0, 10);
  while (true) delay(1);
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

void addUniqueChannel(uint8_t *list, uint8_t &count, uint8_t ch1) {
  if (!channelEnabled(ch1)) return;
  for (uint8_t i = 0; i < count; ++i) {
    if (list[i] == ch1) return;
  }
  if (count >= 16) return;
  list[count++] = ch1;
}

uint8_t mainArpOutChannelForSettings(const Settings &s) {
  return (s.arpOutChannel >= ARP_CH_1_PLUS_10) ? 1 : s.arpOutChannel;
}

uint16_t roundRobinMaskFromLegacyRange(uint8_t arpOutChannel, uint8_t range) {
  if (range == 0) return 0;
  const uint8_t baseCh = (arpOutChannel >= ARP_CH_1_PLUS_10) ? 1 : arpOutChannel;
  if (!channelEnabled(baseCh)) return 0;
  uint16_t mask = 0;
  const uint8_t span = min<uint8_t>(16, static_cast<uint8_t>(range + 1));
  for (uint8_t i = 0; i < span; ++i) {
    const uint8_t ch = ((baseCh - 1 + i) % 16) + 1;
    mask |= channelBit(ch);
  }
  return mask;
}

void collectRelevantPanicChannels(const Settings &s, uint8_t *channels, uint8_t &count) {
  addUniqueChannel(channels, count, mainArpOutChannelForSettings(s));
  for (uint8_t ch = 1; ch <= 16; ++ch) {
    if (s.roundRobinMask & channelBit(ch)) addUniqueChannel(channels, count, ch);
  }
  if (roundRobinCh10To1Enabled(s) || roundRobinCh10To2Enabled(s)) {
    for (uint8_t ch = 1; ch <= 16; ++ch) addUniqueChannel(channels, count, ch);
  }
  addUniqueChannel(channels, count, s.thruOutChannel);
  addUniqueChannel(channels, count, s.legatoChannel);
  addUniqueChannel(channels, count, s.sensorChannel);
  if (s.bassMode > 0) addUniqueChannel(channels, count, bassModeChannel(s.bassMode));
}

void sendMappedAllOffChannel(uint8_t ch1) {
  if (!channelEnabled(ch1)) return;
  sendAllNoteOffChannel(ch1);
}

void panicMappedRelevantOnly(uint8_t settingId, int16_t nextValue) {
  if (!settingNeedsPanic(settingId)) return;

  Settings after = settings;
  mapCcApplyPersistBaseline(after, settingId, nextValue);

  uint8_t channels[16];
  uint8_t count = 0;
  collectRelevantPanicChannels(settings, channels, count);
  collectRelevantPanicChannels(after, channels, count);
  for (uint8_t i = 0; i < count; ++i) sendMappedAllOffChannel(channels[i]);

  activeArpCount = 0;
  activeDrumArpCount = 0;
  loopPlaying = false;
  loopRecording = false;
  loopRecordingArmed = false;
  loopOverdubbing = false;
  loopDeleteArmed = false;
  currentBassOutNote = -1;
  sensorRt.activeNote = -1;
  pushRt.activeNote = -1;
  sensorRt.lastPitch = 0;
  pushRt.lastPitch = 0;
  sensorRt.lastCcValue = -1;
  pushRt.lastCcValue = -1;
}

void panicMidiOnly() {
  if (loopPlaying) loopAllOff();
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
  if (channelEnabled(settings.remoteChannel)) sendAllNoteOffChannel(settings.remoteChannel);
  if (channelEnabled(settings.sensorChannel)) sendAllNoteOffChannel(settings.sensorChannel);
  activeArpCount = 0;
  activeDrumArpCount = 0;
  loopPlaying = false;
  loopRecording = false;
  loopRecordingArmed = false;
  loopOverdubbing = false;
  loopDeleteArmed = false;
  currentBassOutNote = -1;
  sensorRt.activeNote = -1;
  pushRt.activeNote = -1;
  sensorRt.lastPitch = 0;
  pushRt.lastPitch = 0;
  sensorRt.lastCcValue = -1;
  pushRt.lastCcValue = -1;
}

void sendDinAllNoteOffChannelOnly(uint8_t ch1) {
  if (!channelEnabled(ch1)) return;
  for (uint8_t note = 0; note < 128; ++note) sendDinNoteOff(ch1, note, 0);
  sendDinCc(ch1, 123, 0);
  sendDinCc(ch1, 120, 0);
}

void panicDinOnly() {
  // loopAllOff() routes through normal fanout, including USB host output.
  // This DIN-only path is used while changing USB host mode to avoid a stuck hub write during save/reboot.
  const uint8_t panicArpCh = mainArpOutChannel();
  if (channelEnabled(panicArpCh)) sendDinAllNoteOffChannelOnly(panicArpCh);
  for (uint8_t ch = 1; ch <= 16; ++ch) {
    if (settings.roundRobinMask & channelBit(ch)) sendDinAllNoteOffChannelOnly(ch);
  }
  if (roundRobinCh10To1Enabled() || roundRobinCh10To2Enabled()) {
    for (uint8_t ch = 1; ch <= 16; ++ch) sendDinAllNoteOffChannelOnly(ch);
  }
  if (arpChannelSpecialMode()) sendDinAllNoteOffChannelOnly(10);
  if (channelEnabled(settings.thruOutChannel)) sendDinAllNoteOffChannelOnly(settings.thruOutChannel);
  if (channelEnabled(settings.legatoChannel)) sendDinAllNoteOffChannelOnly(settings.legatoChannel);
  if (settings.bassMode > 0) sendDinAllNoteOffChannelOnly(bassModeChannel(settings.bassMode));
  if (channelEnabled(settings.remoteChannel)) sendDinAllNoteOffChannelOnly(settings.remoteChannel);
  if (channelEnabled(settings.sensorChannel)) sendDinAllNoteOffChannelOnly(settings.sensorChannel);
  activeArpCount = 0;
  activeDrumArpCount = 0;
  loopPlaying = false;
  loopRecording = false;
  loopRecordingArmed = false;
  loopOverdubbing = false;
  loopDeleteArmed = false;
  currentBassOutNote = -1;
  sensorRt.activeNote = -1;
  pushRt.activeNote = -1;
  sensorRt.lastPitch = 0;
  pushRt.lastPitch = 0;
  sensorRt.lastCcValue = -1;
  pushRt.lastCcValue = -1;
}

void panicAll() {
  panicMidiOnly();
  resetHeldState();
  ui.menuMode = MENU_SELECT;
  ui.dirty = true;
  markActivity();
}

void resetHeldState() {
  memset(heldInputNotes, 0, sizeof(heldInputNotes));
  memset(heldVelocities, 0, sizeof(heldVelocities));
  memset(physicalHeldInputNotes, 0, sizeof(physicalHeldInputNotes));
  memset(physicalHeldVelocities, 0, sizeof(physicalHeldVelocities));
  memset(loopHeldInputNotes, 0, sizeof(loopHeldInputNotes));
  memset(loopHeldVelocities, 0, sizeof(loopHeldVelocities));
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
  memset(mappedThruNotes, 0xFF, sizeof(mappedThruNotes));
  memset(mappedLoopThruNotes, 0xFF, sizeof(mappedLoopThruNotes));
  memset(mappedArpOffNotes, 0xFF, sizeof(mappedArpOffNotes));
  memset(mappedArpOffChannels, 0, sizeof(mappedArpOffChannels));
  memset(mappedLoopArpOffNotes, 0xFF, sizeof(mappedLoopArpOffNotes));
  memset(mappedLoopArpOffChannels, 0, sizeof(mappedLoopArpOffChannels));
  memset(thruOutputRefCount, 0, sizeof(thruOutputRefCount));
  memset(arpOffOutputRefCount, 0, sizeof(arpOffOutputRefCount));
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
  activeArpCount = 0;
  activeDrumArpCount = 0;
  memset(activeArpNotes, 0xFF, sizeof(activeArpNotes));
  memset(activeArpChannels, 0, sizeof(activeArpChannels));
  roundRobinCursor = 0;
  arpGlobalStep = 0;
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

bool loopOwnsInput(uint8_t sourcePort) {
  return sourcePort == LOOP_SOURCE_PORT;
}

bool inputOwnerHeld(uint8_t sourcePort, uint8_t note) {
  return loopOwnsInput(sourcePort) ? loopHeldInputNotes[note] : physicalHeldInputNotes[note];
}

void setInputOwnerState(uint8_t sourcePort, uint8_t note, uint8_t velocity, bool on) {
  bool *ownerNotes = loopOwnsInput(sourcePort) ? loopHeldInputNotes : physicalHeldInputNotes;
  uint8_t *ownerVelocities = loopOwnsInput(sourcePort) ? loopHeldVelocities : physicalHeldVelocities;
  ownerNotes[note] = on;
  ownerVelocities[note] = on ? velocity : 0;
  heldInputNotes[note] = physicalHeldInputNotes[note] || loopHeldInputNotes[note];
  heldVelocities[note] = physicalHeldInputNotes[note]
                           ? physicalHeldVelocities[note]
                           : loopHeldVelocities[note];
}

void setDrumOwnerState(uint8_t sourcePort, uint8_t note, uint8_t velocity, bool on) {
  bool *ownerNotes = loopOwnsInput(sourcePort) ? loopHeldDrumNotes : physicalHeldDrumNotes;
  uint8_t *ownerVelocities = loopOwnsInput(sourcePort) ? loopHeldDrumVelocities : physicalHeldDrumVelocities;
  ownerNotes[note] = on;
  ownerVelocities[note] = on ? velocity : 0;
  heldDrumNotes[note] = physicalHeldDrumNotes[note] || loopHeldDrumNotes[note];
  heldDrumVelocities[note] = physicalHeldDrumNotes[note]
                            ? physicalHeldDrumVelocities[note]
                            : loopHeldDrumVelocities[note];
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
    if (include) arpHeldSorted[arpHeldCount++] = n;
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
  const int8_t nextSource = (arpLatchPlusEnabled() || arpFreezePlusActive)
                              ? ((arpHeldCount > 0) ? arpHeldSorted[0] : -1)
                              : ((heldCount > 0) ? heldSorted[0] : -1);
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
  return constrain(effectiveSettingValue(SET_ARP_MODE), 0, ARP_SELECTION_COUNT - 1);
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
  const int8_t divSlot = activeDivNoteSlot();
  if (divSlot >= 0) return divNoteSlotToDivision(static_cast<uint8_t>(divSlot));
  return constrain(effectiveSettingValue(SET_DIVISION), 0, DIVISION_COUNT - 1);
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
  if (sendNoteOffs) arpNoteOffs();
  arpGateOffMs = 0;
  arpGridOriginUs = time_us_64();
  arpNextStepUs = arpGridOriginUs;
  arpGlobalStep = 0;
  arpPatternStep = 0;
}

void restartArpFromNewKeyPhraseAt(uint64_t phraseStartUs) {
  arpGateOffMs = 0;
  arpGlobalStep = 0;
  arpPatternStep = 0;
  arpGridOriginUs = phraseStartUs + (ARP_KEY_SYNC_CAPTURE_MS * 1000ULL);
  arpNextStepUs = arpGridOriginUs;
}

void restartArpFromNewKeyPhrase() {
  restartArpFromNewKeyPhraseAt(time_us_64());
}

void syncArpDivisionToGrid() {
  if (arpNextStepUs == 0) return;
  const uint64_t now = time_us_64();
  if (now < arpGridOriginUs) {
    arpNextStepUs = arpGridOriginUs;
    return;
  }
  const uint64_t stepNumerator =
      static_cast<uint64_t>(kDivisionPulseSteps[currentDivisionSetting()]) * 60000000ULL;
  const uint64_t pulseDenominator = static_cast<uint64_t>(currentBpm()) * MUSICAL_PPQN;
  const uint64_t elapsed = now - arpGridOriginUs;
  uint64_t boundary = (elapsed * pulseDenominator + stepNumerator - 1) / stepNumerator;
  arpNextStepUs = arpGridOriginUs + ((boundary * stepNumerator) / pulseDenominator);
  if (arpNextStepUs < now) {
    boundary++;
    arpNextStepUs = arpGridOriginUs + ((boundary * stepNumerator) / pulseDenominator);
  }
}

void noteArpOffPassthrough(uint8_t sourcePort, uint8_t inNote, uint8_t velocity, bool on) {
  if (currentArpSelection() != ARPSEL_OFF) return;
  const bool drumSplit = arpChannelSplitMode() && splitDrumInputNote(inNote);
  const uint8_t baseOutCh = drumSplit ? 10 : mainArpOutChannel();
  uint8_t outCh = baseOutCh;
  if (!channelEnabled(outCh)) return;
  if (outCh == effectiveThruChannel()) return;
  uint8_t *mappedNotes = loopOwnsInput(sourcePort) ? mappedLoopArpOffNotes : mappedArpOffNotes;
  uint8_t *mappedChannels = loopOwnsInput(sourcePort) ? mappedLoopArpOffChannels : mappedArpOffChannels;
  const uint8_t q = drumSplit ? inNote : quantizeUp(inNote);
  if (on) {
    mappedNotes[inNote] = q;
    if (arpOffOutputRefCount[q]++ == 0) {
      if (!drumSplit) outCh = nextRoundRobinChannel(baseOutCh);
      mappedChannels[inNote] = outCh;
      sendFanout(sourcePort, 0x90 | ((outCh - 1) & 0x0F), q, velocity);
    }
  } else {
    const uint8_t out = mappedNotes[inNote];
    outCh = mappedChannels[inNote] ? mappedChannels[inNote] : baseOutCh;
    if (out <= 127 && arpOffOutputRefCount[out] > 0 && --arpOffOutputRefCount[out] == 0) {
      sendFanout(sourcePort, 0x80 | ((outCh - 1) & 0x0F), out, 0);
    }
    mappedNotes[inNote] = 0xFF;
    mappedChannels[inNote] = 0;
  }
}

uint8_t loopBarCount() {
  switch (settings.loopBars) {
    case LOOP_BARS_2: return 2;
    case LOOP_BARS_4: return 4;
    case LOOP_BARS_8: return 8;
    default: return 1;
  }
}

uint8_t loopBarsToCount(uint8_t barsSetting) {
  switch (barsSetting) {
    case LOOP_BARS_2: return 2;
    case LOOP_BARS_4: return 4;
    case LOOP_BARS_8: return 8;
    default: return 1;
  }
}

uint32_t fixedLoopLengthMsForBars(uint8_t barsSetting) {
  const uint64_t lengthUs = musicalDurationUs(
      static_cast<uint64_t>(MUSICAL_PPQN) * 4ULL * loopBarsToCount(barsSetting));
  return static_cast<uint32_t>((lengthUs + 999ULL) / 1000ULL);
}

uint32_t fixedLoopLengthMs() {
  return fixedLoopLengthMsForBars(settings.loopBars);
}

uint64_t fixedLoopLengthUs() {
  return musicalDurationUs(
      static_cast<uint64_t>(MUSICAL_PPQN) * 4ULL * loopBarsToCount(settings.loopBars));
}

void clearLoopPseudoReplaceState() {
  loopHiddenForReplace = false;
  loopReplaceArmed = false;
  loopRecPlayLastSwipeMs = 0;
}

void releaseArpClockIfLooperIdle() {
  if (loopRecording || loopPlaying) return;
  if (anyPhysicalInputNotesHeld() || heldDrumCount > 0) return;
  arpHadKeys = false;
  arpGateOffMs = 0;
  arpNextStepUs = 0;
}

void releaseResidualLoopOwners() {
  const uint8_t previousDivision = currentDivisionSetting();
  for (uint8_t note = 0; note < 128; ++note) {
    if (mappedLoopThruNotes[note] <= 127) noteThrough(LOOP_SOURCE_PORT, note, 0, false);
    if (mappedLoopArpOffNotes[note] <= 127) {
      noteArpOffPassthrough(LOOP_SOURCE_PORT, note, 0, false);
    }
    setInputOwnerState(LOOP_SOURCE_PORT, note, 0, false);
    setDrumOwnerState(LOOP_SOURCE_PORT, note, 0, false);
  }
  for (uint8_t i = 0; i < DIV_NOTE_SLOT_COUNT; ++i) {
    loopDivNoteHeld[i] = false;
    divNoteHeld[i] = physicalDivNoteHeld[i];
  }
  if (currentDivisionSetting() != previousDivision) syncArpDivisionToGrid();
  rebuildHeldSorted();
  rebuildArpHeldSorted();
  rebuildHeldDrumCount();
}

void clearLoopOverdubTracking() {
  memset(loopOverdubHeld, 0, sizeof(loopOverdubHeld));
  memset(loopOverdubHeldVelocity, 0, sizeof(loopOverdubHeldVelocity));
  memset(loopOverdubStartMs, 0, sizeof(loopOverdubStartMs));
  memset(loopOverdubWrapped, 0, sizeof(loopOverdubWrapped));
  memset(loopOverdubFullCycle, 0, sizeof(loopOverdubFullCycle));
}

void loopAllOff() {
  for (uint8_t channel = 0; channel < 16; ++channel) {
    for (uint8_t noteByte = 0; noteByte < 16; ++noteByte) {
      const uint8_t heldBits = loopPlaybackHeld[channel][noteByte];
      loopPlaybackHeld[channel][noteByte] = 0;
      if (heldBits == 0) continue;
      for (uint8_t bit = 0; bit < 8; ++bit) {
        if ((heldBits & (1u << bit)) == 0) continue;
        const uint8_t note = noteByte * 8 + bit;
        routeIncomingChannelMessage(LOOP_SOURCE_PORT,
                                    0x80 | channel, note, 0);
      }
    }
  }
  releaseResidualLoopOwners();
}

void setLoopPlaybackHeld(uint8_t channel1, uint8_t note, bool held) {
  if (channel1 < 1 || channel1 > 16 || note > 127) return;
  uint8_t &bits = loopPlaybackHeld[channel1 - 1][note >> 3];
  const uint8_t mask = 1u << (note & 0x07);
  if (held) bits |= mask;
  else bits &= ~mask;
}

void clearLoopData() {
  loopAllOff();
  clearLoopPseudoReplaceState();
  loopEventCount = 0;
  loopPlayIndex = 0;
  loopLengthUs = 0;
  loopLengthMs = 0;
  loopStoredLengthMs = 0;
  loopHasData = false;
  loopRecordingArmed = false;
  loopRecording = false;
  loopPlaying = false;
  loopOverdubbing = false;
  loopDeleteArmed = false;
  clearLoopOverdubTracking();
  clearSavedLoopStorage();
  releaseArpClockIfLooperIdle();
  ui.dirty = true;
}

void startLoopPlaybackAt(uint64_t startUs) {
  if (!loopHasData || loopEventCount == 0 || loopLengthMs == 0) return;
  loopAllOff();
  loopHiddenForReplace = false;
  loopReplaceArmed = false;
  loopPlayIndex = 0;
  if (loopLengthUs == 0) loopLengthUs = static_cast<uint64_t>(loopLengthMs) * 1000ULL;
  loopStartUs = startUs;
  loopPlaying = true;
  loopRecording = false;
  loopRecordingArmed = false;
  loopOverdubbing = false;
  clearLoopOverdubTracking();
  ui.dirty = true;
}

void startLoopPlayback() {
  startLoopPlaybackAt(time_us_64());
}

uint16_t firstLoopEventAfter(uint32_t atMs) {
  uint16_t idx = 0;
  while (idx < loopEventCount && loopEvents[idx].atMs <= atMs) idx++;
  return idx;
}

void resyncLoopPlaybackWindow(uint64_t oldLengthUs) {
  if (!loopPlaying || loopLengthMs == 0) return;
  const uint64_t now = time_us_64();
  uint64_t posUs = 0;
  if (oldLengthUs > 0) posUs = (now - loopStartUs) % oldLengthUs;
  if (loopLengthUs > 0 && posUs >= loopLengthUs) posUs %= loopLengthUs;
  loopAllOff();
  loopStartUs = now - posUs;
  loopPlayIndex = firstLoopEventAfter(static_cast<uint32_t>(posUs / 1000ULL));
}

void duplicateLoopToLength(uint32_t targetLengthMs) {
  if (!loopHasData || loopEventCount == 0 || loopStoredLengthMs == 0) {
    loopStoredLengthMs = targetLengthMs;
    return;
  }
  while (loopStoredLengthMs < targetLengthMs && loopEventCount < MAX_LOOP_EVENTS) {
    const uint32_t baseLen = loopStoredLengthMs;
    const uint16_t baseCount = loopEventCount;
    bool copiedAny = false;
    for (uint16_t i = 0; i < baseCount && loopEventCount < MAX_LOOP_EVENTS; ++i) {
      const uint32_t at = loopEvents[i].atMs + baseLen;
      if (at >= targetLengthMs) continue;
      LoopEvent event = loopEvents[i];
      event.atMs = at;
      if (insertLoopEventLimited(event.atMs, event.channel, event.note, event.velocity, event.on, targetLengthMs)) {
        copiedAny = true;
      }
    }
    loopStoredLengthMs = min<uint32_t>(targetLengthMs, loopStoredLengthMs + baseLen);
    if (!copiedAny) break;
  }
  if (loopStoredLengthMs < targetLengthMs) loopStoredLengthMs = targetLengthMs;
}

void applyLoopBarsLengthChange(bool preservePlayback = true) {
  if (!loopHasData) return;
  const uint64_t oldLengthUs = loopLengthUs;
  uint32_t targetLength = loopStoredLengthMs;
  if (settings.loopBars != LOOP_BARS_FREE) {
    targetLength = fixedLoopLengthMs();
    if (targetLength > loopStoredLengthMs) duplicateLoopToLength(targetLength);
  }
  loopLengthMs = max<uint32_t>(1, targetLength);
  loopLengthUs = (settings.loopBars == LOOP_BARS_FREE)
      ? static_cast<uint64_t>(loopLengthMs) * 1000ULL
      : fixedLoopLengthUs();
  if (preservePlayback) resyncLoopPlaybackWindow(oldLengthUs);
  saveLoopStorageIfAny();
  ui.dirty = true;
}

void stopLoopPlaybackOnly() {
  if (loopPlaying) loopAllOff();
  loopReplaceArmed = false;
  loopPlaying = false;
  loopOverdubbing = false;
  loopPlayIndex = 0;
  clearLoopOverdubTracking();
  releaseArpClockIfLooperIdle();
}

void closeRecordedNotesAtLoopBoundary() {
  if (loopLengthMs == 0 || loopEventCount == 0) return;
  bool active[16][128] = {};
  for (uint16_t i = 0; i < loopEventCount; ++i) {
    const LoopEvent &event = loopEvents[i];
    active[event.channel - 1][event.note] = event.on && event.velocity > 0;
  }

  const uint32_t boundaryMs = loopLengthMs - 1;
  for (uint8_t channel = 0; channel < 16; ++channel) {
    for (uint8_t note = 0; note < 128; ++note) {
      if (!active[channel][note]) continue;
      if (!insertLoopEventLimited(boundaryMs, channel + 1, note, 0, false, loopLengthMs)) return;
    }
  }
}

void finishLoopRecording(bool playNow) {
  if (!loopRecording && !loopRecordingArmed) return;
  if (loopRecording && settings.loopBars == LOOP_BARS_FREE) {
    loopLengthUs = max<uint64_t>(1000ULL, time_us_64() - loopStartUs);
    loopLengthMs = static_cast<uint32_t>((loopLengthUs + 999ULL) / 1000ULL);
  }
  if (loopRecording) closeRecordedNotesAtLoopBoundary();
  loopRecording = false;
  loopRecordingArmed = false;
  loopStoredLengthMs = max(loopStoredLengthMs, loopLengthMs);
  loopHasData = (loopEventCount > 0 && loopLengthMs > 0);
  if (loopHasData && playNow) {
    startLoopPlaybackAt(loopStartUs + loopLengthUs);
    if (settings.loopAutoOverdub) {
      loopOverdubbing = true;
      clearLoopOverdubTracking();
      capturePhysicalHeldNotesForOverdub();
    }
  } else ui.dirty = true;
}

bool insertLoopEventLimited(uint32_t atMs, uint8_t channel1, uint8_t note, uint8_t velocity, bool on, uint32_t limitMs) {
  if (loopEventCount >= MAX_LOOP_EVENTS || channel1 < 1 || channel1 > 16 || note > 127) return false;
  if (limitMs > 0 && atMs >= limitMs) atMs = limitMs - 1;

  uint16_t pos = 0;
  while (pos < loopEventCount && loopEvents[pos].atMs <= atMs) pos++;
  for (uint16_t i = loopEventCount; i > pos; --i) loopEvents[i] = loopEvents[i - 1];
  loopEvents[pos].atMs = atMs;
  loopEvents[pos].channel = channel1;
  loopEvents[pos].note = note;
  loopEvents[pos].velocity = velocity;
  loopEvents[pos].on = on && velocity > 0;
  loopEventCount++;
  if (loopPlaying && pos <= loopPlayIndex) loopPlayIndex++;
  return true;
}

bool insertLoopEvent(uint32_t atMs, uint8_t channel1, uint8_t note, uint8_t velocity, bool on) {
  const uint32_t limitMs = loopStoredLengthMs ? loopStoredLengthMs : loopLengthMs;
  return insertLoopEventLimited(atMs, channel1, note, velocity, on, limitMs);
}

uint32_t currentLoopPlaybackMs() {
  if (!loopPlaying || loopLengthMs == 0) return 0;
  if (loopLengthUs == 0) return 0;
  return static_cast<uint32_t>(((time_us_64() - loopStartUs) % loopLengthUs) / 1000ULL);
}

void capturePhysicalHeldNotesForOverdub() {
  if (!loopOverdubbing || !loopPlaying || loopLengthMs == 0) return;
  const uint32_t atMs = currentLoopPlaybackMs();
  const uint8_t inputChannel = constrain(settings.inputChannel, 1, 16);
  for (uint8_t note = 0; note < 128; ++note) {
    if (physicalHeldInputNotes[note] &&
        insertLoopEvent(atMs, inputChannel, note, physicalHeldVelocities[note], true)) {
      loopOverdubHeld[inputChannel - 1][note] = true;
      loopOverdubHeldVelocity[inputChannel - 1][note] = physicalHeldVelocities[note];
      loopOverdubStartMs[inputChannel - 1][note] = atMs;
    }
    if (physicalHeldDrumNotes[note] &&
        insertLoopEvent(atMs, 10, note, physicalHeldDrumVelocities[note], true)) {
      loopOverdubHeld[10 - 1][note] = true;
      loopOverdubHeldVelocity[10 - 1][note] = physicalHeldDrumVelocities[note];
      loopOverdubStartMs[10 - 1][note] = atMs;
    }
  }
}

bool loopOverdubWasWrapped(uint8_t channel, uint8_t note) {
  return (loopOverdubWrapped[channel][note >> 3] & (1u << (note & 0x07))) != 0;
}

void setLoopOverdubWrapped(uint8_t channel, uint8_t note, bool wrapped) {
  uint8_t &bits = loopOverdubWrapped[channel][note >> 3];
  const uint8_t mask = 1u << (note & 0x07);
  if (wrapped) bits |= mask;
  else bits &= ~mask;
}

bool loopOverdubCoveredFullCycle(uint8_t channel, uint8_t note) {
  return (loopOverdubFullCycle[channel][note >> 3] & (1u << (note & 0x07))) != 0;
}

void setLoopOverdubFullCycle(uint8_t channel, uint8_t note, bool fullCycle) {
  uint8_t &bits = loopOverdubFullCycle[channel][note >> 3];
  const uint8_t mask = 1u << (note & 0x07);
  if (fullCycle) bits |= mask;
  else bits &= ~mask;
}

void eraseLoopNoteEventsInRange(uint8_t channel1, uint8_t note,
                                uint32_t startMs, uint32_t endMs) {
  const uint16_t oldPlayIndex = loopPlayIndex;
  uint16_t removedBeforePlayIndex = 0;
  uint16_t write = 0;
  for (uint16_t read = 0; read < loopEventCount; ++read) {
    const LoopEvent &event = loopEvents[read];
    const bool remove = event.channel == channel1 && event.note == note &&
                        event.atMs >= startMs && event.atMs <= endMs;
    if (remove) {
      if (read < oldPlayIndex) removedBeforePlayIndex++;
      continue;
    }
    if (write != read) loopEvents[write] = event;
    write++;
  }
  loopEventCount = write;
  loopPlayIndex = oldPlayIndex - min<uint16_t>(oldPlayIndex, removedBeforePlayIndex);
}

void commitLoopOverdubNoteOff(uint8_t channel, uint8_t note, uint32_t atMs) {
  const uint8_t channel1 = channel + 1;
  const uint8_t velocity = max<uint8_t>(1, loopOverdubHeldVelocity[channel][note]);
  const bool wrapped = loopOverdubWasWrapped(channel, note);
  const bool fullCycle = loopOverdubCoveredFullCycle(channel, note);

  if (!fullCycle) {
    const uint32_t startMs = wrapped ? 0 : loopOverdubStartMs[channel][note];
    eraseLoopNoteEventsInRange(channel1, note, startMs, atMs);
    insertLoopEventLimited(startMs, channel1, note, velocity, true, loopLengthMs);
    insertLoopEventLimited(atMs, channel1, note, 0, false, loopLengthMs);
  }

  if (wrapped || fullCycle) {
    setLoopPlaybackHeld(channel1, note, false);
    routeIncomingChannelMessage(LOOP_SOURCE_PORT,
                                0x80 | (channel & 0x0F), note, 0);
  }
  loopOverdubHeld[channel][note] = false;
  loopOverdubHeldVelocity[channel][note] = 0;
  loopOverdubStartMs[channel][note] = 0;
  setLoopOverdubWrapped(channel, note, false);
  setLoopOverdubFullCycle(channel, note, false);
}

void carryHeldOverdubNotesAcrossBoundary() {
  if (!loopOverdubbing || loopLengthMs == 0) return;
  const uint32_t boundaryMs = loopLengthMs - 1;
  for (uint8_t channel = 0; channel < 16; ++channel) {
    for (uint8_t note = 0; note < 128; ++note) {
      if (!loopOverdubHeld[channel][note] || loopOverdubCoveredFullCycle(channel, note)) continue;
      const uint8_t velocity = max<uint8_t>(1, loopOverdubHeldVelocity[channel][note]);
      if (loopOverdubWasWrapped(channel, note)) {
        eraseLoopNoteEventsInRange(channel + 1, note, 0, boundaryMs);
        if (!insertLoopEventLimited(0, channel + 1, note, velocity, true, loopLengthMs)) return;
        if (!insertLoopEventLimited(boundaryMs, channel + 1, note, 0, false, loopLengthMs)) return;
        setLoopOverdubFullCycle(channel, note, true);
        continue;
      }

      const uint32_t startMs = loopOverdubStartMs[channel][note];
      eraseLoopNoteEventsInRange(channel + 1, note, startMs, boundaryMs);
      eraseLoopNoteEventsInRange(channel + 1, note, 0, 0);
      if (!insertLoopEventLimited(startMs, channel + 1, note, velocity, true, loopLengthMs)) return;
      if (!insertLoopEventLimited(boundaryMs, channel + 1, note, 0, false, loopLengthMs)) return;
      if (!insertLoopEventLimited(0, channel + 1, note, velocity, true, loopLengthMs)) return;
      setLoopOverdubWrapped(channel, note, true);
    }
  }
}

void finishLoopOverdub() {
  if (!loopOverdubbing || !loopPlaying || loopLengthMs == 0) {
    loopOverdubbing = false;
    clearLoopOverdubTracking();
    return;
  }
  if ((time_us_64() - loopStartUs) >= loopLengthUs) {
    carryHeldOverdubNotesAcrossBoundary();
  }
  const uint32_t atMs = currentLoopPlaybackMs();
  for (uint8_t ch = 0; ch < 16; ++ch) {
    for (uint8_t note = 0; note < 128; ++note) {
      if (loopOverdubHeld[ch][note]) {
        commitLoopOverdubNoteOff(ch, note, atMs);
      }
    }
  }
  loopOverdubbing = false;
  ui.dirty = true;
}

void toggleLoopOverdub() {
  if (!loopPlaying || !loopHasData) return;
  if (loopOverdubbing) {
    finishLoopOverdub();
  } else {
    loopOverdubbing = true;
    clearLoopOverdubTracking();
    capturePhysicalHeldNotesForOverdub();
    loopDeleteArmed = false;
    ui.dirty = true;
  }
}

bool loopRecPlayDoubleSwipe() {
  const uint32_t now = millis();
  const bool isDouble = loopRecPlayLastSwipeMs != 0 &&
                        (now - loopRecPlayLastSwipeMs) <= LOOP_REC_PLAY_DOUBLE_SWIPE_MS;
  loopRecPlayLastSwipeMs = isDouble ? 0 : now;
  return isDouble;
}

void hideLoopForReplace() {
  if (!loopHasData) return;
  if (loopOverdubbing) finishLoopOverdub();
  if (loopPlaying) loopAllOff();
  loopPlaying = false;
  loopRecording = false;
  loopRecordingArmed = false;
  loopOverdubbing = false;
  loopReplaceArmed = false;
  loopHiddenForReplace = true;
  loopDeleteArmed = false;
  clearLoopOverdubTracking();
  releaseArpClockIfLooperIdle();
  ui.dirty = true;
}

void armLoopReplaceRecording() {
  if (!loopHasData) return;
  if (loopPlaying) loopAllOff();
  loopPlaying = false;
  loopRecording = false;
  loopRecordingArmed = true;
  loopOverdubbing = false;
  loopReplaceArmed = true;
  loopHiddenForReplace = true;
  loopPlayIndex = 0;
  loopLengthMs = loopLengthMs ? loopLengthMs : fixedLoopLengthMs();
  if (loopLengthUs == 0) {
    loopLengthUs = (settings.loopBars == LOOP_BARS_FREE)
        ? static_cast<uint64_t>(loopLengthMs) * 1000ULL
        : fixedLoopLengthUs();
  }
  loopDeleteArmed = false;
  clearLoopOverdubTracking();
  ui.dirty = true;
}

void handleLoopRecPlayTrigger() {
  const bool doubleSwipe = loopRecPlayDoubleSwipe();

  if (loopRecordingArmed) {
    if (loopReplaceArmed && loopHiddenForReplace && loopHasData) {
      loopRecordingArmed = false;
      loopReplaceArmed = false;
      if (doubleSwipe) startLoopPlayback();
      else {
        releaseArpClockIfLooperIdle();
        ui.dirty = true;
      }
    } else {
      clearLoopData();
    }
    return;
  }
  if (loopRecording) {
    if (settings.loopBars == LOOP_BARS_FREE) finishLoopRecording(true);
    else clearLoopData();
    return;
  }
  if (loopHiddenForReplace && loopHasData) {
    if (doubleSwipe) startLoopPlayback();
    else armLoopReplaceRecording();
    return;
  }
  if (doubleSwipe && loopHasData) {
    hideLoopForReplace();
    return;
  }
  if (loopPlaying) {
    toggleLoopOverdub();
    return;
  }
  if (loopHasData) {
    startLoopPlayback();
  } else {
    loopRecordingArmed = true;
    loopEventCount = 0;
    loopPlayIndex = 0;
    loopLengthMs = 0;
  }
  loopDeleteArmed = false;
  ui.dirty = true;
}

void handleLoopStopDeleteTrigger() {
  const uint32_t now = millis();
  const bool active = loopPlaying || loopRecording || loopRecordingArmed;
  if (active) {
    if (loopRecording || loopRecordingArmed) finishLoopRecording(false);
    if (loopOverdubbing) finishLoopOverdub();
    stopLoopPlaybackOnly();
    loopDeleteArmed = true;
    loopSdLastTriggerMs = now;
  } else if (loopHasData && loopDeleteArmed) {
    clearLoopData();
  } else if (!loopDeleteArmed || (now - loopSdLastTriggerMs) >= LOOP_STOP_DELETE_DEBOUNCE_MS) {
    loopDeleteArmed = loopHasData;
    loopSdLastTriggerMs = now;
  }
  ui.dirty = true;
}

void recordLoopNote(uint8_t sourcePort, uint8_t channel1, uint8_t note, uint8_t velocity, bool on) {
  if (sourcePort == LOOP_SOURCE_PORT) return;
  if (loopRecordingArmed) {
    if (!on || velocity == 0) return;
    loopRecordingArmed = false;
    loopRecording = true;
    loopHiddenForReplace = false;
    loopReplaceArmed = false;
    loopEventCount = 0;
    loopHasData = false;
    loopStartUs = time_us_64();
    restartArpFromNewKeyPhraseAt(loopStartUs);
    loopLengthMs = (settings.loopBars == LOOP_BARS_FREE) ? 0 : fixedLoopLengthMs();
    loopLengthUs = (settings.loopBars == LOOP_BARS_FREE) ? 0 : fixedLoopLengthUs();
    loopStoredLengthMs = loopLengthMs;
  }
  if (loopOverdubbing && loopPlaying && loopHasData && loopLengthMs > 0) {
    // Settle a due wrap before assigning this live event to a loop cycle. This
    // also advances the playback cursor so the overdub is not replayed now.
    const uint64_t eventUs = time_us_64();
    tickLooperAt(eventUs);
    const uint32_t atMs = static_cast<uint32_t>(
        ((eventUs - loopStartUs) % loopLengthUs) / 1000ULL);
    const uint8_t channel = channel1 - 1;
    const bool held = on && velocity > 0;
    if (held) {
      if (loopOverdubHeld[channel][note]) return;
      if (loopEventCount >= MAX_LOOP_EVENTS - 2) return;
      if (insertLoopEvent(atMs, channel1, note, velocity, true)) {
        loopOverdubHeld[channel][note] = true;
        loopOverdubHeldVelocity[channel][note] = velocity;
        loopOverdubStartMs[channel][note] = atMs;
        setLoopOverdubWrapped(channel, note, false);
        setLoopOverdubFullCycle(channel, note, false);
      }
    } else if (loopOverdubHeld[channel][note]) {
      commitLoopOverdubNoteOff(channel, note, atMs);
    }
    return;
  }
  if (!loopRecording || loopEventCount >= (MAX_LOOP_EVENTS - LOOP_BOUNDARY_OFF_RESERVE)) return;
  uint32_t atMs = static_cast<uint32_t>((time_us_64() - loopStartUs) / 1000ULL);
  if (settings.loopBars != LOOP_BARS_FREE && loopLengthMs > 0 && atMs >= loopLengthMs) {
    finishLoopRecording(true);
    return;
  }
  insertLoopEvent(atMs, channel1, note, velocity, on);
}

void tickLooperAt(uint64_t now) {
  if (loopRecording && settings.loopBars != LOOP_BARS_FREE && loopLengthUs > 0 &&
      (now - loopStartUs) >= loopLengthUs) {
    finishLoopRecording(true);
  }
  if (!loopPlaying || !loopHasData || loopLengthMs == 0 || loopLengthUs == 0) return;
  uint64_t elapsedUs = now - loopStartUs;
  if (elapsedUs >= loopLengthUs) {
    carryHeldOverdubNotesAcrossBoundary();
    loopAllOff();
    do {
      loopStartUs += loopLengthUs;
    } while ((now - loopStartUs) >= loopLengthUs);
    loopPlayIndex = 0;
    elapsedUs = now - loopStartUs;
  }
  const uint32_t elapsed = static_cast<uint32_t>(elapsedUs / 1000ULL);
  while (loopPlayIndex < loopEventCount && loopEvents[loopPlayIndex].atMs <= elapsed) {
    const LoopEvent &event = loopEvents[loopPlayIndex++];
    setLoopPlaybackHeld(event.channel, event.note, event.on && event.velocity > 0);
    routeIncomingChannelMessage(LOOP_SOURCE_PORT,
                                (event.on ? 0x90 : 0x80) | ((event.channel - 1) & 0x0F),
                                event.note, event.velocity);
  }
}

void tickLooper() {
  tickLooperAt(time_us_64());
}

void clearSavedLoopStorage() {
  uint16_t emptyMagic = 0xFFFF;
  EEPROM.put(LOOP_STORAGE_OFFSET, emptyMagic);
  EEPROM.commit();
}

void saveLoopStorageIfAny() {
  if (!loopHasData || loopEventCount == 0 || loopLengthMs == 0) {
    clearSavedLoopStorage();
    return;
  }

  LoopStorageImage image{};
  image.magic = LOOP_EEPROM_MAGIC;
  image.count = min<uint16_t>(loopEventCount, MAX_LOOP_EVENTS);
  image.lengthMs = loopStoredLengthMs ? loopStoredLengthMs : loopLengthMs;
  image.activeLengthMs = loopLengthMs;
  image.bars = settings.loopBars;
  for (uint16_t i = 0; i < image.count; ++i) {
    image.events[i].at10ms = min<uint32_t>(65535UL, (loopEvents[i].atMs + 5UL) / 10UL);
    image.events[i].channel = loopEvents[i].channel;
    image.events[i].note = loopEvents[i].note;
    image.events[i].velocity = loopEvents[i].velocity;
    image.events[i].flags = loopEvents[i].on ? 1 : 0;
  }
  EEPROM.put(LOOP_STORAGE_OFFSET, image);
  EEPROM.commit();
}

void loadSavedLoopStorage() {
  LoopStorageImage image{};
  EEPROM.get(LOOP_STORAGE_OFFSET, image);
  if (image.magic != LOOP_EEPROM_MAGIC || image.count == 0 || image.count > MAX_LOOP_EVENTS ||
      image.lengthMs == 0) {
    loopHasData = false;
    loopEventCount = 0;
    return;
  }

  loopEventCount = image.count;
  loopPlayIndex = 0;
  loopStoredLengthMs = image.lengthMs;
  loopLengthMs = image.activeLengthMs ? image.activeLengthMs : image.lengthMs;
  loopLengthUs = static_cast<uint64_t>(loopLengthMs) * 1000ULL;
  loopHasData = true;
  loopRecordingArmed = false;
  loopRecording = false;
  loopPlaying = false;
  loopOverdubbing = false;
  loopDeleteArmed = false;
  clearLoopOverdubTracking();
  for (uint16_t i = 0; i < loopEventCount; ++i) {
    loopEvents[i].atMs = static_cast<uint32_t>(image.events[i].at10ms) * 10UL;
    if (loopEvents[i].atMs >= loopStoredLengthMs) loopEvents[i].atMs = loopStoredLengthMs - 1;
    loopEvents[i].channel = clampU8(image.events[i].channel, 1, 16);
    loopEvents[i].note = clampU8(image.events[i].note, 0, 127);
    loopEvents[i].velocity = clampU8(image.events[i].velocity, 0, 127);
    loopEvents[i].on = (image.events[i].flags & 1) != 0;
  }
  applyLoopBarsLengthChange(false);
}

void thruOutputRefOn(uint8_t sourcePort, uint8_t outNote, uint8_t velocity) {
  const uint8_t outCh = effectiveThruChannel();
  if (!channelEnabled(outCh) || outNote > 127) return;
  if (thruOutputRefCount[outNote]++ == 0) {
    sendFanout(sourcePort, 0x90 | ((outCh - 1) & 0x0F), outNote, velocity);
  }
}

void thruOutputRefOff(uint8_t sourcePort, uint8_t outNote) {
  const uint8_t outCh = effectiveThruChannel();
  if (!channelEnabled(outCh) || outNote > 127) return;
  if (thruOutputRefCount[outNote] > 0 && --thruOutputRefCount[outNote] == 0) {
    sendFanout(sourcePort, 0x80 | ((outCh - 1) & 0x0F), outNote, 0);
  }
}

void noteThrough(uint8_t sourcePort, uint8_t inNote, uint8_t velocity, bool on) {
  uint8_t *mappedNotes = loopOwnsInput(sourcePort) ? mappedLoopThruNotes : mappedThruNotes;
  if (on) {
    const uint8_t q = quantizeUp(inNote);
    mappedNotes[inNote] = q;
    thruOutputRefOn(sourcePort, q, velocity);
  } else {
    const uint8_t q = mappedNotes[inNote];
    if (q <= 127) thruOutputRefOff(sourcePort, q);
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

  uint8_t *thruMap = loopOwnsInput(sourcePort) ? mappedLoopThruNotes : mappedThruNotes;
  const uint8_t thruOut = thruMap[note];
  const uint8_t thruCh = effectiveThruChannel();
  if (thruOut <= 127 && channelEnabled(thruCh) &&
      thruOutputRefCount[thruOut] > 0 && --thruOutputRefCount[thruOut] == 0) {
    sendFanout(sourcePort, 0x80 | ((thruCh - 1) & 0x0F), thruOut, 0);
  }
  thruMap[note] = 0xFF;

  uint8_t *arpMap = loopOwnsInput(sourcePort) ? mappedLoopArpOffNotes : mappedArpOffNotes;
  uint8_t *arpChannels = loopOwnsInput(sourcePort) ? mappedLoopArpOffChannels : mappedArpOffChannels;
  const uint8_t arpOut = arpMap[note];
  const uint8_t arpCh = arpChannels[note] ? arpChannels[note] : mainArpOutChannel();
  if (arpOut <= 127 && channelEnabled(arpCh) &&
      arpOffOutputRefCount[arpOut] > 0 && --arpOffOutputRefCount[arpOut] == 0) {
    sendFanout(sourcePort, 0x80 | ((arpCh - 1) & 0x0F), arpOut, 0);
  }
  arpMap[note] = 0xFF;
  arpChannels[note] = 0;
}

uint8_t nextRoundRobinChannel(uint8_t baseCh) {
  if (!channelEnabled(baseCh) || settings.roundRobinMask == 0) return baseCh;
  for (uint8_t attempts = 0; attempts < 16; ++attempts) {
    const uint8_t idx = roundRobinCursor++ & 0x0F;
    if (settings.roundRobinMask & static_cast<uint16_t>(1U << idx)) return idx + 1;
  }
  return baseCh;
}

void handleClockByte(bool fromDin) {
  (void)fromDin;
}

bool sensorParamEligible(uint8_t settingId) {
  return settingId == SET_DIVISION;
}

int16_t settingRangeMax(uint8_t settingId) {
  switch (settingId) {
    case SET_BPM: return 300;
    case SET_ARP_MODE: return ARP_SELECTION_COUNT - 1;
    case SET_DIVISION: return DIVISION_COUNT - 1;
    case SET_VELOCITY: return 127;
    case SET_LENGTH: return 100;
    case SET_PATTERN: return PATTERN_COUNT - 1;
    case SET_INPUT_CH: return 16;
    case SET_ARP_OUT_CH: return ARP_CH_MAX;
    case SET_BASS_CH: return 48;
    case SET_THRU_OUT_CH: return 16;
    case SET_RND_RBN: return RND_RBN_BACK_SLOT;
    case SET_ROUTER:
      if (routerEditStage == ROUTER_STAGE_DEST) return 16;
      if (routerEditStage == ROUTER_STAGE_TRANSPOSE) return ROUTER_TRANSPOSE_MAX - ROUTER_TRANSPOSE_MIN;
      return ROUTER_BACK_SLOT;
    case SET_DIV_NOTES: return DIV_NOTE_BACK_SLOT;
    case SET_MAP_CC: return MAP_CC_CHMODE_SLOT;
    case SET_LEGATO_CH: return 16;
    case SET_CC_OUT_CH: return 17;
    case SET_REMOTE_CH: return 16;
    case SET_SENSOR_CH: return 16;
    case SET_SENSOR_MODE: return SENSOR_MODE_COUNT - 1;
    case SET_PUSH_MODE: return SENSOR_MODE_COUNT - 1;
    case SET_LOOP_BARS: return LOOP_BARS_COUNT - 1;
    case SET_FORCE_KEY: return 24;
    case SET_FORCE_SCALE: return FORCE_SCALE_COUNT - 1;
    case SET_GUITAR_PIANO: return 1;
    case SET_REMOTE1: return 254;
    case SET_REMOTE2: return 254;
    case SET_LOAD_PRESET: return PRESET_COUNT - 1;
    case SET_SAVE_PRESET: return PRESET_COUNT - 1;
    case SET_SCREEN_SAVER: return 2;
    default: return 0;
  }
}

int16_t getSettingValueRaw(uint8_t settingId) {
  switch (settingId) {
    case SET_BPM: return settings.manualBpm;
    case SET_ARP_MODE: return settings.arpMode;
    case SET_DIVISION: return settings.division;
    case SET_VELOCITY: return settings.arpVelocity;
    case SET_LENGTH: return settings.arpLengthPct;
    case SET_PATTERN: return settings.pattern;
    case SET_INPUT_CH: return settings.inputChannel;
    case SET_ARP_OUT_CH: return settings.arpOutChannel;
    case SET_BASS_CH: return settings.bassMode;
    case SET_THRU_OUT_CH: return settings.thruOutChannel;
    case SET_RND_RBN: return roundRobinMenuCursor;
    case SET_ROUTER:
      if (routerEditStage == ROUTER_STAGE_DEST) return settings.routerOutChannels[routerEditChannel];
      if (routerEditStage == ROUTER_STAGE_TRANSPOSE) return settings.routerTranspose[routerEditChannel] - ROUTER_TRANSPOSE_MIN;
      return routerMenuCursor;
    case SET_DIV_NOTES: return divNotesCursor;
    case SET_MAP_CC: return mapCcCursor;
    case SET_LEGATO_CH: return settings.legatoChannel;
    case SET_CC_OUT_CH: return settings.ccOutChannel;
    case SET_REMOTE_CH: return settings.remoteChannel;
    case SET_SENSOR_CH: return settings.sensorChannel;
    case SET_SENSOR_MODE: return settings.sensorMode;
    case SET_PUSH_MODE: return settings.pushMode;
    case SET_LOOP_BARS: return settings.loopBars;
    case SET_FORCE_KEY: return settings.forceKey;
    case SET_FORCE_SCALE: return settings.forceScale;
    case SET_GUITAR_PIANO: return settings.instrumentView;
    case SET_REMOTE1: return settings.remote1Action;
    case SET_REMOTE2: return settings.remote2Action;
    case SET_LOAD_PRESET:
      if (ui.menuMode == MENU_SELECT && ui.selectedSetting == SET_LOAD_PRESET) return storage.currentPreset;
      return settings.loadPreset;
    case SET_SAVE_PRESET:
      if (ui.menuMode == MENU_SELECT && ui.selectedSetting == SET_SAVE_PRESET) return storage.currentPreset;
      return settings.savePreset;
    case SET_SCREEN_SAVER: return screenSaverForceNow ? 2 : settings.screenSaver;
    default: return 0;
  }
}

void setSettingValueRaw(uint8_t settingId, int16_t value) {
  switch (settingId) {
    case SET_BPM: settings.manualBpm = constrain(value, 20, 300); break;
    case SET_ARP_MODE: settings.arpMode = clampU8(value, 0, ARP_SELECTION_COUNT - 1); break;
    case SET_DIVISION: settings.division = clampU8(value, 0, DIVISION_COUNT - 1); break;
    case SET_VELOCITY: settings.arpVelocity = clampU8(value, 1, 127); break;
    case SET_LENGTH: settings.arpLengthPct = clampU8(value, 1, 100); break;
    case SET_PATTERN: settings.pattern = clampU8(value, 0, PATTERN_COUNT - 1); break;
    case SET_INPUT_CH: settings.inputChannel = clampU8(value, 1, 16); break;
    case SET_ARP_OUT_CH:
      settings.arpOutChannel = clampU8(value, 0, ARP_CH_MAX);
      break;
    case SET_BASS_CH: settings.bassMode = clampU8(value, 0, 48); break;
    case SET_THRU_OUT_CH: settings.thruOutChannel = clampU8(value, 0, 16); break;
    case SET_RND_RBN: roundRobinMenuCursor = clampU8(value, 0, RND_RBN_BACK_SLOT); break;
    case SET_ROUTER:
      if (routerEditStage == ROUTER_STAGE_DEST) {
        settings.routerOutChannels[routerEditChannel] = clampU8(value, 1, 16);
        updateRouterActiveBit(settings, routerEditChannel);
      } else if (routerEditStage == ROUTER_STAGE_TRANSPOSE) {
        settings.routerTranspose[routerEditChannel] =
          static_cast<int8_t>(constrain(value + ROUTER_TRANSPOSE_MIN, ROUTER_TRANSPOSE_MIN, ROUTER_TRANSPOSE_MAX));
        updateRouterActiveBit(settings, routerEditChannel);
      } else {
        routerMenuCursor = clampU8(value, 0, ROUTER_BACK_SLOT);
      }
      break;
    case SET_DIV_NOTES: divNotesCursor = clampU8(value, 0, DIV_NOTE_BACK_SLOT); break;
    case SET_MAP_CC: mapCcCursor = clampU8(value, 0, MAP_CC_CHMODE_SLOT); break;
    case SET_LEGATO_CH: {
      const uint8_t previous = settings.legatoChannel;
      settings.legatoChannel = clampU8(value, 0, 16);
      if (settings.legatoChannel != previous) clearLegatoState(true, previous);
      break;
    }
    case SET_CC_OUT_CH: settings.ccOutChannel = clampU8(value, 1, 17); break;
    case SET_REMOTE_CH: settings.remoteChannel = clampU8(value, 1, 16); break;
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
    case SET_LOOP_BARS:
      settings.loopBars = clampU8(value, 0, LOOP_BARS_COUNT - 1);
      applyLoopBarsLengthChange(true);
      break;
    case SET_FORCE_KEY:
      settings.forceKey = clampU8(value, 0, 24);
      if (ckeyEnabled() && scaleIsCombo(settings.forceScale)) settings.forceScale = SCALE_MAJOR;
      break;
    case SET_FORCE_SCALE: settings.forceScale = clampU8(value, 0, FORCE_SCALE_COUNT - 1); break;
    case SET_GUITAR_PIANO: settings.instrumentView = clampU8(value, 0, 1); break;
    case SET_REMOTE1: settings.remote1Action = clampU8(value, 0, 254); break;
    case SET_REMOTE2: settings.remote2Action = clampU8(value, 0, 254); break;
    case SET_LOAD_PRESET: settings.loadPreset = clampU8(value, 0, PRESET_COUNT - 1); break;
    case SET_SAVE_PRESET: settings.savePreset = clampU8(value, 0, PRESET_COUNT - 1); break;
    case SET_SCREEN_SAVER:
      if (value >= 2) {
        screenSaverForceNow = true;
      } else {
        screenSaverForceNow = false;
        settings.screenSaver = clampU8(value, 0, 1);
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
  if (ui.hasPendingEdit && ui.pendingSetting == settingId) return ui.pendingValue;
  if (mapCcPreviewActive && mapCcPreviewSetting == settingId) return mapCcPreviewValue;
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
  s.division = clampU8(s.division, 0, DIVISION_COUNT - 1);
  s.arpVelocity = clampU8(s.arpVelocity, 1, 127);
  s.arpLengthPct = clampU8(s.arpLengthPct, 1, 100);
  s.inputChannel = clampU8(s.inputChannel, 1, 16);
  s.arpOutChannel = clampU8(s.arpOutChannel, 0, ARP_CH_MAX);
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
  s.remoteChannel = clampU8(s.remoteChannel, 1, 16);
  s.sensorChannel = clampU8(s.sensorChannel, 1, 16);
  s.sensorMode = clampU8(s.sensorMode, 0, SENSOR_MODE_COUNT - 1);
  s.pushMode = clampU8(s.pushMode, 0, SENSOR_MODE_COUNT - 1);
  s.loopBars = clampU8(s.loopBars, 0, LOOP_BARS_COUNT - 1);
  s.loopAutoOverdub = (s.loopAutoOverdub > 0) ? 1 : 0;
  s.forceKey = clampU8(s.forceKey, 0, 24);
  s.forceScale = clampU8(s.forceScale, 0, FORCE_SCALE_COUNT - 1);
  if (ckeyEnabledForValue(s.forceKey) && scaleIsCombo(s.forceScale)) s.forceScale = SCALE_MAJOR;
  s.instrumentView = clampU8(s.instrumentView, 0, 1);
  s.remote1Action = clampU8(s.remote1Action, 0, 254);
  s.remote2Action = clampU8(s.remote2Action, 0, 254);
  s.loadPreset = clampU8(s.loadPreset, 0, PRESET_COUNT - 1);
  s.savePreset = clampU8(s.savePreset, 0, PRESET_COUNT - 1);
  s.reserved = 0;
  s.screenSaver = clampU8(s.screenSaver, 0, 1);
  for (uint8_t i = 0; i < DIV_NOTE_SLOT_COUNT; ++i) {
    if (s.divNoteChannels[i] > 16) s.divNoteChannels[i] = 0;
    if (s.divNoteNotes[i] > 127) s.divNoteNotes[i] = 0xFF;
  }
  if (s.divNotePlusNote > 127) s.divNotePlusNote = 0xFF;
  for (uint8_t i = 0; i < MAPCC_PARAM_COUNT; ++i) {
    if (s.mapCcChannels[i] > 16) s.mapCcChannels[i] = 0;
    if (s.mapCcNumbers[i] > 127) s.mapCcNumbers[i] = 0xFF;
  }
  s.mapCcChannelMode &= (MAP_CC_CHANNEL_ALL_BIT | MAP_CC_RR_CH10_TO_1_BIT | MAP_CC_RR_CH10_TO_2_BIT);
}

void loadCurrentPreset() {
  screenSaverForceNow = false;
  settings = storage.presets[storage.currentPreset];
  sanitizeSettings(settings);
  divNotesCursor = 0;
  mapCcCursor = 0;
  syncMapCcRuntimeFromSettings();
  mapCcPreviewActive = false;
  mapCcUiPending = false;
  for (uint8_t i = 0; i < MAPCC_PARAM_COUNT; ++i) mapCcDeferredActive[i] = false;
  captureMapCcPersistBaseline();
  settings.loadPreset = storage.currentPreset;
  settings.savePreset = storage.currentPreset;
  ui.dirty = true;
}

void stagePersistedUiSetting(uint8_t settingId) {
  if (settingId >= SETTING_COUNT || !selectableSetting(settingId)) return;
  EEPROM.write(UI_SCREEN_STORAGE_OFFSET, UI_SCREEN_STORAGE_MAGIC);
  EEPROM.write(UI_SCREEN_STORAGE_OFFSET + 1, settingId);
  persistedUiSetting = settingId;
  uiScreenSavePending = false;
}

bool loadPersistedUiSetting(uint8_t &settingId) {
  if (EEPROM.read(UI_SCREEN_STORAGE_OFFSET) != UI_SCREEN_STORAGE_MAGIC) return false;
  const uint8_t stored = EEPROM.read(UI_SCREEN_STORAGE_OFFSET + 1);
  if (stored >= SETTING_COUNT || !selectableSetting(stored)) return false;
  settingId = stored;
  persistedUiSetting = stored;
  return true;
}

void saveStorage() {
  storage.magic = EEPROM_MAGIC;
  syncMapCcRuntimeToSettings();
  storage.presets[storage.currentPreset] = settings;
  storage.presets[storage.currentPreset].loadPreset = storage.currentPreset;
  storage.presets[storage.currentPreset].savePreset = storage.currentPreset;
  EEPROM.put(0, storage);
  stagePersistedUiSetting(ui.selectedSetting);
  EEPROM.commit();
  captureMapCcPersistBaseline(settings);
}

Settings defaultSettings() {
  Settings s{};
  s.manualBpm = 120;
  s.arpMode = ARPSEL_UP;
  s.division = DIV_1_8;
  s.arpVelocity = 96;
  s.arpLengthPct = 55;
  s.pattern = PAT_MODE;
  s.inputChannel = 1;
  s.arpOutChannel = ARP_CH_1_PLUS_10;
  s.bassMode = 0;
  s.thruOutChannel = 2;
  s.roundRobinMask = 0;
  clearRouterMappings(s);
  s.legatoChannel = 0;
  s.ccOutChannel = 17;
  s.remoteChannel = 16;
  s.sensorChannel = 3;
  s.sensorMode = SENSOR_OFF;
  s.pushMode = SENSOR_OFF;
  s.forceKey = 0;
  s.forceScale = SCALE_OFF;
  s.instrumentView = 0;
  s.remote1Action = 103;
  s.remote2Action = 104;
  s.loadPreset = 0;
  s.savePreset = 0;
  s.reserved = 0;
  s.screenSaver = 1;
  for (uint8_t i = 0; i < DIV_NOTE_SLOT_COUNT; ++i) {
    s.divNoteChannels[i] = 0;
    s.divNoteNotes[i] = 0xFF;
  }
  s.divNotePlusNote = 0xFF;
  s.loopBars = LOOP_BARS_1;
  s.loopAutoOverdub = 0;
  for (uint8_t i = 0; i < MAPCC_PARAM_COUNT; ++i) {
    s.mapCcChannels[i] = 0;
    s.mapCcNumbers[i] = 0xFF;
  }
  s.mapCcChannelMode = 0;
  return s;
}

Settings migrateSettingsV2(const SettingsV2 &oldSettings) {
  Settings upgraded = defaultSettings();
  upgraded.manualBpm = oldSettings.manualBpm;
  upgraded.arpMode = oldSettings.arpMode;
  upgraded.division = oldSettings.division;
  upgraded.arpVelocity = oldSettings.arpVelocity;
  upgraded.arpLengthPct = oldSettings.arpLengthPct;
  upgraded.pattern = oldSettings.pattern;
  upgraded.inputChannel = oldSettings.inputChannel;
  upgraded.arpOutChannel = oldSettings.arpOutChannel;
  upgraded.bassMode = oldSettings.bassMode;
  upgraded.thruOutChannel = oldSettings.thruOutChannel;
  upgraded.ccOutChannel = oldSettings.ccOutChannel;
  upgraded.remoteChannel = oldSettings.remoteChannel;
  upgraded.sensorChannel = oldSettings.sensorChannel;
  upgraded.sensorMode = oldSettings.sensorMode;
  upgraded.forceKey = oldSettings.forceKey;
  upgraded.forceScale = oldSettings.forceScale;
  upgraded.instrumentView = oldSettings.instrumentView;
  upgraded.remote1Action = oldSettings.remote1Action;
  upgraded.remote2Action = oldSettings.remote2Action;
  upgraded.loadPreset = oldSettings.loadPreset;
  upgraded.savePreset = oldSettings.savePreset;
  upgraded.reserved = 0;
  upgraded.screenSaver = oldSettings.screenSaver;
  memcpy(upgraded.divNoteChannels, oldSettings.divNoteChannels, sizeof(upgraded.divNoteChannels));
  memcpy(upgraded.divNoteNotes, oldSettings.divNoteNotes, sizeof(upgraded.divNoteNotes));
  upgraded.divNotePlusNote = oldSettings.divNotePlusNote;
  upgraded.pushMode = oldSettings.pushMode;
  upgraded.legatoChannel = oldSettings.legatoChannel;
  sanitizeSettings(upgraded);
  return upgraded;
}

Settings migrateSettingsV4(const SettingsV4 &oldSettings) {
  Settings upgraded = defaultSettings();
  upgraded.manualBpm = oldSettings.manualBpm;
  upgraded.arpMode = oldSettings.arpMode;
  upgraded.division = oldSettings.division;
  upgraded.arpVelocity = oldSettings.arpVelocity;
  upgraded.arpLengthPct = oldSettings.arpLengthPct;
  upgraded.pattern = oldSettings.pattern;
  upgraded.inputChannel = oldSettings.inputChannel;
  upgraded.arpOutChannel = oldSettings.arpOutChannel;
  upgraded.bassMode = oldSettings.bassMode;
  upgraded.thruOutChannel = oldSettings.thruOutChannel;
  upgraded.ccOutChannel = oldSettings.ccOutChannel;
  upgraded.remoteChannel = oldSettings.remoteChannel;
  upgraded.sensorChannel = oldSettings.sensorChannel;
  upgraded.sensorMode = oldSettings.sensorMode;
  upgraded.forceKey = oldSettings.forceKey;
  upgraded.forceScale = oldSettings.forceScale;
  upgraded.instrumentView = oldSettings.instrumentView;
  upgraded.remote1Action = oldSettings.remote1Action;
  upgraded.remote2Action = oldSettings.remote2Action;
  upgraded.loadPreset = oldSettings.loadPreset;
  upgraded.savePreset = oldSettings.savePreset;
  upgraded.reserved = 0;
  upgraded.screenSaver = oldSettings.screenSaver;
  memcpy(upgraded.divNoteChannels, oldSettings.divNoteChannels, sizeof(upgraded.divNoteChannels));
  memcpy(upgraded.divNoteNotes, oldSettings.divNoteNotes, sizeof(upgraded.divNoteNotes));
  upgraded.divNotePlusNote = oldSettings.divNotePlusNote;
  upgraded.pushMode = oldSettings.pushMode;
  upgraded.legatoChannel = oldSettings.legatoChannel;
  memcpy(upgraded.mapCcChannels, oldSettings.mapCcChannels, sizeof(oldSettings.mapCcChannels));
  memcpy(upgraded.mapCcNumbers, oldSettings.mapCcNumbers, sizeof(oldSettings.mapCcNumbers));
  upgraded.mapCcChannelMode = oldSettings.mapCcChannelMode;
  sanitizeSettings(upgraded);
  return upgraded;
}

Settings migrateSettingsV5(const SettingsV5 &oldSettings) {
  Settings upgraded = defaultSettings();
  upgraded.manualBpm = oldSettings.manualBpm;
  upgraded.arpMode = oldSettings.arpMode;
  upgraded.division = oldSettings.division;
  upgraded.arpVelocity = oldSettings.arpVelocity;
  upgraded.arpLengthPct = oldSettings.arpLengthPct;
  upgraded.pattern = oldSettings.pattern;
  upgraded.inputChannel = oldSettings.inputChannel;
  upgraded.arpOutChannel = oldSettings.arpOutChannel;
  upgraded.bassMode = oldSettings.bassMode;
  upgraded.thruOutChannel = oldSettings.thruOutChannel;
  upgraded.roundRobinMask = roundRobinMaskFromLegacyRange(oldSettings.arpOutChannel, oldSettings.roundRobinRange);
  upgraded.ccOutChannel = oldSettings.ccOutChannel;
  upgraded.remoteChannel = oldSettings.remoteChannel;
  upgraded.sensorChannel = oldSettings.sensorChannel;
  upgraded.sensorMode = oldSettings.sensorMode;
  upgraded.forceKey = oldSettings.forceKey;
  upgraded.forceScale = oldSettings.forceScale;
  upgraded.instrumentView = oldSettings.instrumentView;
  upgraded.remote1Action = oldSettings.remote1Action;
  upgraded.remote2Action = oldSettings.remote2Action;
  upgraded.loadPreset = oldSettings.loadPreset;
  upgraded.savePreset = oldSettings.savePreset;
  upgraded.reserved = 0;
  upgraded.screenSaver = oldSettings.screenSaver;
  memcpy(upgraded.divNoteChannels, oldSettings.divNoteChannels, sizeof(upgraded.divNoteChannels));
  memcpy(upgraded.divNoteNotes, oldSettings.divNoteNotes, sizeof(upgraded.divNoteNotes));
  upgraded.divNotePlusNote = oldSettings.divNotePlusNote;
  upgraded.pushMode = oldSettings.pushMode;
  upgraded.loopBars = oldSettings.loopBars;
  upgraded.legatoChannel = oldSettings.legatoChannel;
  memcpy(upgraded.mapCcChannels, oldSettings.mapCcChannels, sizeof(oldSettings.mapCcChannels));
  memcpy(upgraded.mapCcNumbers, oldSettings.mapCcNumbers, sizeof(oldSettings.mapCcNumbers));
  upgraded.mapCcChannelMode = oldSettings.mapCcChannelMode;
  sanitizeSettings(upgraded);
  return upgraded;
}

Settings migrateSettingsV6(const SettingsV6 &oldSettings) {
  Settings upgraded = defaultSettings();
  upgraded.manualBpm = oldSettings.manualBpm;
  upgraded.arpMode = oldSettings.arpMode;
  upgraded.division = oldSettings.division;
  upgraded.arpVelocity = oldSettings.arpVelocity;
  upgraded.arpLengthPct = oldSettings.arpLengthPct;
  upgraded.pattern = oldSettings.pattern;
  upgraded.inputChannel = oldSettings.inputChannel;
  upgraded.arpOutChannel = oldSettings.arpOutChannel;
  upgraded.bassMode = oldSettings.bassMode;
  upgraded.thruOutChannel = oldSettings.thruOutChannel;
  upgraded.roundRobinMask = oldSettings.roundRobinMask;
  upgraded.ccOutChannel = oldSettings.ccOutChannel;
  upgraded.remoteChannel = oldSettings.remoteChannel;
  upgraded.sensorChannel = oldSettings.sensorChannel;
  upgraded.sensorMode = oldSettings.sensorMode;
  upgraded.forceKey = oldSettings.forceKey;
  upgraded.forceScale = oldSettings.forceScale;
  upgraded.instrumentView = oldSettings.instrumentView;
  upgraded.remote1Action = oldSettings.remote1Action;
  upgraded.remote2Action = oldSettings.remote2Action;
  upgraded.loadPreset = oldSettings.loadPreset;
  upgraded.savePreset = oldSettings.savePreset;
  upgraded.reserved = 0;
  upgraded.screenSaver = oldSettings.screenSaver;
  memcpy(upgraded.divNoteChannels, oldSettings.divNoteChannels, sizeof(upgraded.divNoteChannels));
  memcpy(upgraded.divNoteNotes, oldSettings.divNoteNotes, sizeof(upgraded.divNoteNotes));
  upgraded.divNotePlusNote = oldSettings.divNotePlusNote;
  upgraded.pushMode = oldSettings.pushMode;
  upgraded.loopBars = oldSettings.loopBars;
  upgraded.loopAutoOverdub = oldSettings.loopAutoOverdub;
  upgraded.legatoChannel = oldSettings.legatoChannel;
  memcpy(upgraded.mapCcChannels, oldSettings.mapCcChannels, sizeof(oldSettings.mapCcChannels));
  memcpy(upgraded.mapCcNumbers, oldSettings.mapCcNumbers, sizeof(oldSettings.mapCcNumbers));
  upgraded.mapCcChannelMode = oldSettings.mapCcChannelMode;
  sanitizeSettings(upgraded);
  return upgraded;
}

Settings migrateSettingsV7(const SettingsV7 &oldSettings) {
  Settings upgraded = defaultSettings();
  upgraded.manualBpm = oldSettings.manualBpm;
  upgraded.arpMode = oldSettings.arpMode;
  upgraded.division = oldSettings.division;
  upgraded.arpVelocity = oldSettings.arpVelocity;
  upgraded.arpLengthPct = oldSettings.arpLengthPct;
  upgraded.pattern = oldSettings.pattern;
  upgraded.inputChannel = oldSettings.inputChannel;
  upgraded.arpOutChannel = oldSettings.arpOutChannel;
  upgraded.bassMode = oldSettings.bassMode;
  upgraded.thruOutChannel = oldSettings.thruOutChannel;
  upgraded.roundRobinMask = oldSettings.roundRobinMask;
  upgraded.ccOutChannel = oldSettings.ccOutChannel;
  upgraded.remoteChannel = oldSettings.remoteChannel;
  upgraded.sensorChannel = oldSettings.sensorChannel;
  upgraded.sensorMode = oldSettings.sensorMode;
  upgraded.forceKey = oldSettings.forceKey;
  upgraded.forceScale = oldSettings.forceScale;
  upgraded.instrumentView = oldSettings.instrumentView;
  upgraded.remote1Action = oldSettings.remote1Action;
  upgraded.remote2Action = oldSettings.remote2Action;
  upgraded.loadPreset = oldSettings.loadPreset;
  upgraded.savePreset = oldSettings.savePreset;
  upgraded.reserved = 0;
  upgraded.screenSaver = oldSettings.screenSaver;
  memcpy(upgraded.divNoteChannels, oldSettings.divNoteChannels, sizeof(upgraded.divNoteChannels));
  memcpy(upgraded.divNoteNotes, oldSettings.divNoteNotes, sizeof(upgraded.divNoteNotes));
  upgraded.divNotePlusNote = oldSettings.divNotePlusNote;
  upgraded.pushMode = oldSettings.pushMode;
  upgraded.loopBars = oldSettings.loopBars;
  upgraded.loopAutoOverdub = oldSettings.loopAutoOverdub;
  upgraded.legatoChannel = oldSettings.legatoChannel;
  memcpy(upgraded.mapCcChannels, oldSettings.mapCcChannels, sizeof(upgraded.mapCcChannels));
  memcpy(upgraded.mapCcNumbers, oldSettings.mapCcNumbers, sizeof(upgraded.mapCcNumbers));
  upgraded.mapCcChannelMode = oldSettings.mapCcChannelMode;
  sanitizeSettings(upgraded);
  return upgraded;
}

Settings migrateSettingsV3(const SettingsV4 &oldSettings) {
  Settings upgraded = migrateSettingsV4(oldSettings);
  if (upgraded.arpOutChannel >= ARP_CH_1_PLUS_10_AFTERTOUCH &&
      upgraded.arpOutChannel <= ARP_CH_1_TO_10_SPLIT_36) {
    upgraded.arpOutChannel = static_cast<uint8_t>(upgraded.arpOutChannel + 1);
  }
  sanitizeSettings(upgraded);
  return upgraded;
}

Settings migrateSettingsV8(const SettingsV8 &oldSettings) {
  Settings upgraded = defaultSettings();
  upgraded.manualBpm = oldSettings.manualBpm;
  upgraded.arpMode = oldSettings.arpMode;
  upgraded.division = oldSettings.division;
  upgraded.arpVelocity = oldSettings.arpVelocity;
  upgraded.arpLengthPct = oldSettings.arpLengthPct;
  upgraded.pattern = oldSettings.pattern;
  upgraded.inputChannel = oldSettings.inputChannel;
  upgraded.arpOutChannel = oldSettings.arpOutChannel;
  upgraded.bassMode = oldSettings.bassMode;
  upgraded.thruOutChannel = oldSettings.thruOutChannel;
  upgraded.roundRobinMask = oldSettings.roundRobinMask;
  upgraded.ccOutChannel = oldSettings.ccOutChannel;
  upgraded.remoteChannel = oldSettings.remoteChannel;
  upgraded.sensorChannel = oldSettings.sensorChannel;
  upgraded.sensorMode = oldSettings.sensorMode;
  upgraded.forceKey = oldSettings.forceKey;
  upgraded.forceScale = oldSettings.forceScale;
  upgraded.instrumentView = oldSettings.instrumentView;
  upgraded.remote1Action = oldSettings.remote1Action;
  upgraded.remote2Action = oldSettings.remote2Action;
  upgraded.loadPreset = oldSettings.loadPreset;
  upgraded.savePreset = oldSettings.savePreset;
  upgraded.reserved = 0;
  upgraded.screenSaver = oldSettings.screenSaver;
  memcpy(upgraded.divNoteChannels, oldSettings.divNoteChannels, sizeof(upgraded.divNoteChannels));
  memcpy(upgraded.divNoteNotes, oldSettings.divNoteNotes, sizeof(upgraded.divNoteNotes));
  upgraded.divNotePlusNote = oldSettings.divNotePlusNote;
  upgraded.pushMode = oldSettings.pushMode;
  upgraded.loopBars = oldSettings.loopBars;
  upgraded.loopAutoOverdub = oldSettings.loopAutoOverdub;
  upgraded.legatoChannel = oldSettings.legatoChannel;
  memcpy(upgraded.mapCcChannels, oldSettings.mapCcChannels, sizeof(upgraded.mapCcChannels));
  memcpy(upgraded.mapCcNumbers, oldSettings.mapCcNumbers, sizeof(upgraded.mapCcNumbers));
  upgraded.mapCcChannelMode = oldSettings.mapCcChannelMode;
  setRoundRobinCh10To1(upgraded, oldSettings.roundRobinCh10To1 > 0);
  sanitizeSettings(upgraded);
  return upgraded;
}

Settings migrateSettingsV1(const SettingsV1 &oldSettings) {
  Settings upgraded = defaultSettings();
  upgraded.manualBpm = oldSettings.manualBpm;
  upgraded.arpMode = oldSettings.arpMode;
  upgraded.division = oldSettings.division;
  upgraded.arpVelocity = oldSettings.arpVelocity;
  upgraded.arpLengthPct = oldSettings.arpLengthPct;
  upgraded.pattern = oldSettings.pattern;
  upgraded.inputChannel = oldSettings.inputChannel;
  upgraded.arpOutChannel = oldSettings.arpOutChannel;
  upgraded.bassMode = oldSettings.bassMode;
  upgraded.thruOutChannel = oldSettings.thruOutChannel;
  upgraded.ccOutChannel = oldSettings.ccOutChannel;
  upgraded.remoteChannel = oldSettings.remoteChannel;
  upgraded.sensorChannel = oldSettings.sensorChannel;
  upgraded.sensorMode = oldSettings.sensorMode;
  upgraded.forceKey = oldSettings.forceKey;
  upgraded.forceScale = oldSettings.forceScale;
  upgraded.instrumentView = oldSettings.instrumentView;
  upgraded.remote1Action = oldSettings.remote1Action;
  upgraded.remote2Action = oldSettings.remote2Action;
  upgraded.loadPreset = oldSettings.loadPreset;
  upgraded.savePreset = oldSettings.savePreset;
  upgraded.reserved = 0;
  upgraded.screenSaver = oldSettings.screenSaver;
  memcpy(upgraded.divNoteChannels, oldSettings.divNoteChannels, sizeof(upgraded.divNoteChannels));
  memcpy(upgraded.divNoteNotes, oldSettings.divNoteNotes, sizeof(upgraded.divNoteNotes));
  upgraded.divNotePlusNote = oldSettings.divNotePlusNote;
  upgraded.pushMode = oldSettings.pushMode;
  sanitizeSettings(upgraded);
  return upgraded;
}

void initStorageIfNeeded() {
  EEPROM.begin(EEPROM_BYTES);
  uint16_t storedMagic = 0;
  EEPROM.get(0, storedMagic);
  if (storedMagic != EEPROM_MAGIC) storedMagic = 0;

  if (storedMagic == EEPROM_MAGIC) {
    EEPROM.get(0, storage);
  } else if (storedMagic == EEPROM_MAGIC_V8) {
    StorageImageV8 legacy{};
    EEPROM.get(0, legacy);
    storage.magic = EEPROM_MAGIC;
    storage.currentPreset = clampU8(legacy.currentPreset, 0, PRESET_COUNT - 1);
    for (uint8_t i = 0; i < PRESET_COUNT; ++i) {
      storage.presets[i] = migrateSettingsV8(legacy.presets[i]);
      storage.presets[i].loadPreset = i;
      storage.presets[i].savePreset = i;
    }
    EEPROM.put(0, storage);
    EEPROM.commit();
  } else if (storedMagic == EEPROM_MAGIC_V7) {
    StorageImageV7 legacy{};
    EEPROM.get(0, legacy);
    storage.magic = EEPROM_MAGIC;
    storage.currentPreset = clampU8(legacy.currentPreset, 0, PRESET_COUNT - 1);
    for (uint8_t i = 0; i < PRESET_COUNT; ++i) {
      storage.presets[i] = migrateSettingsV7(legacy.presets[i]);
      storage.presets[i].loadPreset = i;
      storage.presets[i].savePreset = i;
    }
    EEPROM.put(0, storage);
    EEPROM.commit();
  } else if (storedMagic == EEPROM_MAGIC_V6) {
    StorageImageV6 legacy{};
    EEPROM.get(0, legacy);
    storage.magic = EEPROM_MAGIC;
    storage.currentPreset = clampU8(legacy.currentPreset, 0, PRESET_COUNT - 1);
    for (uint8_t i = 0; i < PRESET_COUNT; ++i) {
      storage.presets[i] = migrateSettingsV6(legacy.presets[i]);
      storage.presets[i].loadPreset = i;
      storage.presets[i].savePreset = i;
    }
    EEPROM.put(0, storage);
    EEPROM.commit();
  } else if (storedMagic == EEPROM_MAGIC_V5) {
    StorageImageV5 legacy{};
    EEPROM.get(0, legacy);
    storage.magic = EEPROM_MAGIC;
    storage.currentPreset = clampU8(legacy.currentPreset, 0, PRESET_COUNT - 1);
    for (uint8_t i = 0; i < PRESET_COUNT; ++i) {
      storage.presets[i] = migrateSettingsV5(legacy.presets[i]);
      storage.presets[i].loadPreset = i;
      storage.presets[i].savePreset = i;
    }
    EEPROM.put(0, storage);
    EEPROM.commit();
  } else if (storedMagic == EEPROM_MAGIC_V4) {
    StorageImageV4 legacy{};
    EEPROM.get(0, legacy);
    storage.magic = EEPROM_MAGIC;
    storage.currentPreset = clampU8(legacy.currentPreset, 0, PRESET_COUNT - 1);
    for (uint8_t i = 0; i < PRESET_COUNT; ++i) {
      storage.presets[i] = migrateSettingsV4(legacy.presets[i]);
      storage.presets[i].loadPreset = i;
      storage.presets[i].savePreset = i;
    }
    EEPROM.put(0, storage);
    EEPROM.commit();
  } else if (storedMagic == EEPROM_MAGIC_V3) {
    StorageImageV4 legacy{};
    EEPROM.get(0, legacy);
    storage.magic = EEPROM_MAGIC;
    storage.currentPreset = clampU8(legacy.currentPreset, 0, PRESET_COUNT - 1);
    for (uint8_t i = 0; i < PRESET_COUNT; ++i) {
      storage.presets[i] = migrateSettingsV3(legacy.presets[i]);
      storage.presets[i].loadPreset = i;
      storage.presets[i].savePreset = i;
    }
    EEPROM.put(0, storage);
    EEPROM.commit();
  } else if (storedMagic == EEPROM_MAGIC_V2) {
    StorageImageV2 legacy{};
    EEPROM.get(0, legacy);
    storage.magic = EEPROM_MAGIC;
    storage.currentPreset = clampU8(legacy.currentPreset, 0, PRESET_COUNT - 1);
    for (uint8_t i = 0; i < PRESET_COUNT; ++i) {
      storage.presets[i] = migrateSettingsV2(legacy.presets[i]);
      storage.presets[i].loadPreset = i;
      storage.presets[i].savePreset = i;
    }
    EEPROM.put(0, storage);
    EEPROM.commit();
  } else if (storedMagic == EEPROM_MAGIC_V1) {
    StorageImageV1 legacy{};
    EEPROM.get(0, legacy);
    storage.magic = EEPROM_MAGIC;
    storage.currentPreset = clampU8(legacy.currentPreset, 0, PRESET_COUNT - 1);
    for (uint8_t i = 0; i < PRESET_COUNT; ++i) {
      storage.presets[i] = migrateSettingsV1(legacy.presets[i]);
      storage.presets[i].loadPreset = i;
      storage.presets[i].savePreset = i;
    }
    EEPROM.put(0, storage);
    EEPROM.commit();
  } else {
    storage.magic = EEPROM_MAGIC;
    storage.currentPreset = 0;
    for (uint8_t i = 0; i < PRESET_COUNT; ++i) {
      storage.presets[i] = defaultSettings();
      storage.presets[i].loadPreset = i;
      storage.presets[i].savePreset = i;
    }
    EEPROM.put(0, storage);
    EEPROM.commit();
  }
  loadCurrentPreset();
  loadSavedLoopStorage();
}

void applySettingDelta(int delta, bool fastStep) {
  const uint8_t id = ui.selectedSetting;
  if (id == SET_PANIC) return;
  const int fast = fastStep ? 10 : 1;
  const int step = delta * fast;
  const int oldValue = (ui.hasPendingEdit && ui.pendingSetting == id) ? ui.pendingValue : getSettingValueRaw(id);
  int next = oldValue + step;
  const int maxValue = settingRangeMax(id);

  if (id == SET_BPM) next = constrain(next, 20, 300);
  else if (id == SET_INPUT_CH || id == SET_REMOTE_CH || id == SET_SENSOR_CH) next = constrain(next, 1, 16);
  else if (id == SET_CC_OUT_CH) next = wrapIndex(next - 1, 17) + 1;
  else if (id == SET_ROUTER && routerEditStage == ROUTER_STAGE_DEST) next = wrapIndex(next - 1, 16) + 1;
  else if (id == SET_ROUTER && routerEditStage == ROUTER_STAGE_TRANSPOSE) {
    next = constrain(next, 0, ROUTER_TRANSPOSE_MAX - ROUTER_TRANSPOSE_MIN);
  }
  else if (id == SET_VELOCITY) next = constrain(next, 1, 127);
  else if (id == SET_LENGTH) next = constrain(next, 1, 100);
  else if (id == SET_FORCE_SCALE) {
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
  if (settingNeedsPanic(id) && ui.menuMode == MENU_EDIT) {
    ui.hasPendingEdit = true;
    ui.pendingSetting = id;
    ui.pendingValue = next;
  } else {
    if (settingNeedsPanic(id)) panicMidiOnly();
    setSettingValueRaw(id, next);
  }

  if (id == SET_DIVISION || id == SET_PATTERN || id == SET_ARP_MODE || id == SET_LENGTH || id == SET_VELOCITY) {
    restartArpTiming(true);
  }

  ui.dirty = true;
  markActivity();
}

void activateClickAction() {
  if (ui.selectedSetting == SET_PANIC) {
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
    if (ui.selectedSetting == SET_MAP_CC && mapCcCursor == MAP_CC_CHMODE_SLOT) {
      mapCcChannelAll = !mapCcChannelAll;
    } else if (ui.selectedSetting == SET_RND_RBN) {
      if (roundRobinMenuCursor < 16) {
        settings.roundRobinMask ^= channelBit(roundRobinMenuCursor + 1);
        saveStorage();
        ui.dirty = true;
        markActivity();
        encoder.switchIgnoreUntilMs = millis() + 120;
        return;
      }
      if (roundRobinMenuCursor == RND_RBN_CH10_TO_1_SLOT) {
        setRoundRobinCh10To1(settings, !roundRobinCh10To1Enabled());
        saveStorage();
        ui.dirty = true;
        markActivity();
        encoder.switchIgnoreUntilMs = millis() + 120;
        return;
      }
      if (roundRobinMenuCursor == RND_RBN_CH10_TO_2_SLOT) {
        setRoundRobinCh10To2(settings, !roundRobinCh10To2Enabled());
        saveStorage();
        ui.dirty = true;
        markActivity();
        encoder.switchIgnoreUntilMs = millis() + 120;
        return;
      }
      if (roundRobinMenuCursor == RND_RBN_CLEAR_SLOT) {
        settings.roundRobinMask = 0;
        setRoundRobinCh10To1(settings, false);
        setRoundRobinCh10To2(settings, false);
        saveStorage();
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
          saveStorage();
          ui.dirty = true;
          markActivity();
          encoder.switchIgnoreUntilMs = millis() + 120;
          return;
        }
      } else if (routerEditStage == ROUTER_STAGE_DEST) {
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
    if (ui.selectedSetting == SET_ROUTER) routerEditStage = ROUTER_STAGE_LIST;
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
    panicAll();
    encoder.turnWhilePressed = true;
  }
}

bool loopLocksArpClock() {
  return loopRecording || loopPlaying;
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

  channel1 = 10;
  note = splitDrumOutputNote(note);
  return true;
}

bool captureDivNoteAssignment(uint8_t channel1, uint8_t note, bool on) {
  if (ui.selectedSetting != SET_DIV_NOTES || ui.menuMode != MENU_EDIT) return false;
  if (!on) return true;
  if (divNotesCursor < DIV_NOTE_SLOT_COUNT) {
    settings.divNoteChannels[divNotesCursor] = channel1;
    settings.divNoteNotes[divNotesCursor] = note;
  } else if (divNotesCursor == DIV_NOTE_PLUS_SLOT) {
    settings.divNotePlusNote = note;
  }
  ui.dirty = true;
  markActivity(false);
  return true;
}

uint8_t handleDivNoteOverride(uint8_t sourcePort, uint8_t channel1, uint8_t &note,
                              uint8_t velocity, bool on) {
  for (uint8_t i = 0; i < DIV_NOTE_SLOT_COUNT; ++i) {
    if (settings.divNoteChannels[i] == channel1 && settings.divNoteNotes[i] == note) {
      const uint8_t previousDivision = currentDivisionSetting();
      recordLoopNote(sourcePort, channel1, note, velocity, on);
      if (loopOwnsInput(sourcePort)) loopDivNoteHeld[i] = on;
      else physicalDivNoteHeld[i] = on;
      divNoteHeld[i] = physicalDivNoteHeld[i] || loopDivNoteHeld[i];
      if (on) divNoteHeldStamp[i] = ++divNotePressCounter;
      if (currentDivisionSetting() != previousDivision) syncArpDivisionToGrid();
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
  if (recordForLoop) recordLoopNote(sourcePort, channel1, note, velocity, on);

  if (arpChannelSpecialMode() && channel1 == 10) {
    handleDrumInputNote(sourcePort, note, velocity, on);
    return;
  }

  if (channel1 != settings.inputChannel) {
    if (channelEnabled(settings.legatoChannel) && channel1 == settings.legatoChannel) {
      handleLegatoInputNote(sourcePort, channel1, note, velocity, on);
    } else {
      sendFanout(sourcePort, (on ? 0x90 : 0x80) | ((channel1 - 1) & 0x0F), note, velocity);
    }
    return;
  }

  markActivity(false);
  if (liveNoteViewActive()) ui.dirty = true;

  if (on && velocity > 0) {
    const bool hadNoPhysicalInputNotes = !anyPhysicalInputNotesHeld();
    if (inputOwnerHeld(sourcePort, note)) releaseDuplicateInputNote(sourcePort, note);

    if (arpLatchEnabled()) {
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
    if (!arpLatchPlusEnabled()) {
      noteThrough(sourcePort, note, velocity, true);
      if (arpLatchEnabled()) thruLatchedNotes[note] = true;
    }
    noteArpOffPassthrough(sourcePort, note, velocity, true);
  } else {
    setInputOwnerState(sourcePort, note, 0, false);
    if (arpLatchEnabled() && !anyPhysicalInputNotesHeld()) arpLatchAwaitingNewPhrase = true;
    const bool sustainHeld = (arpLatchEnabled() && arpLatchedNotes[note]) ||
                             (arpFreezeActive && arpFrozenNotes[note]);
    if (!arpLatchPlusEnabled() && !sustainHeld) noteThrough(sourcePort, note, 0, false);
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
}

void handleDinNoteOn(byte channel, byte pitch, byte velocity) {
  if (captureDivNoteAssignment(channel, pitch, velocity > 0)) return;
  uint8_t ch = channel;
  uint8_t note = pitch;
  const uint8_t divNoteAction = handleDivNoteOverride(0, ch, note, velocity, velocity > 0);
  if (divNoteAction == 1) return;
  translateSplitInputToDrum(ch, note);
  onInputNote(0, ch, note, velocity, velocity > 0, divNoteAction != 2);
}

void handleDinNoteOff(byte channel, byte pitch, byte velocity) {
  (void)velocity;
  if (captureDivNoteAssignment(channel, pitch, false)) return;
  uint8_t ch = channel;
  uint8_t note = pitch;
  const uint8_t divNoteAction = handleDivNoteOverride(0, ch, note, 0, false);
  if (divNoteAction == 1) return;
  translateSplitInputToDrum(ch, note);
  onInputNote(0, ch, note, 0, false, divNoteAction != 2);
}

void routeControlChange(uint8_t sourcePort, byte channel, byte control, byte value) {
  if (captureMapCcAssignment(channel, control)) return;
  applyMappedCcAssignments(channel, control, value);
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
  routeControlChange(0, channel, control, value);
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
  routeProgramLikeMessage(sourcePort, 0xD0 | ((channel - 1) & 0x0F), pressure, 0);
}

void handleDinPb(byte channel, int bend) {
  routePitchBend(0, channel, bend);
}

void handleDinProgramChange(byte channel, byte number) {
  routeProgramLikeMessage(0, 0xC0 | ((channel - 1) & 0x0F), number, 0);
}

void handleDinAfterTouchChannel(byte channel, byte pressure) {
  routeChannelAftertouch(0, channel, pressure);
}

void handleDinClock() {
  handleClockByte(true);
}

void routeIncomingChannelMessage(uint8_t sourcePort, uint8_t status, uint8_t data1, uint8_t data2) {
  const uint8_t type = status & 0xF0;
  uint8_t channel = (status & 0x0F) + 1;
  if (type == 0x90) {
    if (captureDivNoteAssignment(channel, data1, data2 > 0)) return;
    const uint8_t divNoteAction = handleDivNoteOverride(sourcePort, channel, data1, data2, data2 > 0);
    if (divNoteAction == 1) return;
    translateSplitInputToDrum(channel, data1);
    onInputNote(sourcePort, channel, data1, data2, data2 > 0, divNoteAction != 2);
  } else if (type == 0x80) {
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
  } else if (type == 0xC0 || type == 0xA0) {
    routeProgramLikeMessage(sourcePort, status, data1, data2);
  }
}


int8_t arpModeNextIndex() {
  if (arpHeldCount == 0) return -1;
  const uint8_t mode = classicArpModeFromSelection(currentArpSelection());
  const uint32_t phase = arpGlobalStep;
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

void arpAddOutput(uint8_t note) {
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

void drumArpNoteOffs() {
  if (!arpChannelSpecialMode()) return;
  for (uint8_t i = 0; i < activeDrumArpCount; ++i) {
    if (activeDrumArpNotes[i] >= 0) {
      sendFanout(255, 0x80 | (10 - 1), activeDrumArpNotes[i], 0);
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
  sendFanout(255, 0x90 | (10 - 1), note, drumArpPulseVelocity());
}

void runArpStep() {
  const uint8_t arpSelection = currentArpSelection();
  const uint8_t arpPattern = patternFromArpSelection(arpSelection);
  const uint8_t arpMode = classicArpModeFromSelection(arpSelection);
  const bool mainArpEnabled = channelEnabled(mainArpOutChannel()) && arpMode != ARP_OFF && arpHeldCount > 0;
  const bool drumArpEnabled = arpChannelSpecialMode() && heldDrumCount > 0;

  if (!mainArpEnabled && !drumArpEnabled) {
    arpNoteOffs();
    drumArpNoteOffs();
    return;
  }

  drumArpNoteOffs();
  arpNoteOffs();
  const uint8_t step = arpPatternStep % 16;
  const PatternToken token = kPatterns[arpPattern][step];

  if (mainArpEnabled && token.noteIndex == TOK_REST) {
    arpGateOffMs = 0;
    arpGlobalStep++;
    arpPatternStep = (arpPatternStep + 1) % 16;
    if (drumArpEnabled) {
      for (uint8_t note = 0; note < 128; ++note) {
        if (heldDrumNotes[note]) drumArpAddOutput(note);
      }
      arpGateOffMs = millis() + max<uint32_t>(15, (divisionStepMs() * currentArpLengthPctSetting()) / 100);
    }
    return;
  }

  if (drumArpEnabled) {
    for (uint8_t note = 0; note < 128; ++note) {
      if (heldDrumNotes[note]) {
        drumArpAddOutput(note);
      }
    }
  }

  if (mainArpEnabled && (token.noteIndex == TOK_ALL || arpMode == ARP_TRIGGER)) {
    for (uint8_t i = 0; i < arpHeldCount && i < MAX_ARP_OUTPUT_NOTES; ++i) {
      arpAddOutput(quantizeUp(arpHeldSorted[i]));
    }
  } else if (mainArpEnabled) {
    int8_t idx = token.noteIndex;
    if (idx == TOK_MODE || arpPattern == PAT_MODE || arpPattern == PAT_RANDOM) {
      idx = arpModeNextIndex();
    }
    if (idx >= 0 && arpHeldCount > 0) {
      const uint8_t base = arpHeldSorted[idx % arpHeldCount];
      int note = base + token.semitoneOffset + (token.octaveOffset * 12);
      note = constrain(note, 0, 127);
      arpAddOutput(quantizeUp(note));
    }
  }

  const uint32_t gateMs = max<uint32_t>(15, (divisionStepMs() * currentArpLengthPctSetting()) / 100);
  arpGateOffMs = millis() + gateMs;
  arpGlobalStep++;
  arpPatternStep = (arpPatternStep + 1) % 16;
}

void tickArp() {
  const uint32_t nowMs = millis();
  const uint64_t nowUs = time_us_64();
  if (arpHeldCount == 0 && heldDrumCount == 0) {
    drumArpNoteOffs();
    arpNoteOffs();
    return;
  }
  if (arpGateOffMs && nowMs >= arpGateOffMs) {
    drumArpNoteOffs();
    arpNoteOffs();
    arpGateOffMs = 0;
  }
  if (arpNextStepUs == 0 || nowUs >= arpNextStepUs) {
    const uint64_t stepNumerator =
        static_cast<uint64_t>(kDivisionPulseSteps[currentDivisionSetting()]) * 60000000ULL;
    const uint64_t pulseDenominator = static_cast<uint64_t>(currentBpm()) * MUSICAL_PPQN;
    if (arpNextStepUs == 0) {
      arpGridOriginUs = nowUs;
      arpGlobalStep = 0;
      arpPatternStep = 0;
      runArpStep();
      arpNextStepUs = arpGridOriginUs + (stepNumerator / pulseDenominator);
      return;
    }
    const uint64_t elapsed = nowUs - arpGridOriginUs;
    const uint32_t gridStep = static_cast<uint32_t>(
        (elapsed * pulseDenominator) / stepNumerator);
    arpGlobalStep = gridStep;
    arpPatternStep = gridStep % 16;
    runArpStep();
    arpNextStepUs = arpGridOriginUs +
                    ((static_cast<uint64_t>(gridStep + 1) * stepNumerator) / pulseDenominator);
  }
}

void triggerRemoteAction(uint8_t action, ButtonPulse &pulse) {
  const uint8_t ch = settings.remoteChannel;
  if (!channelEnabled(ch)) return;
  if (action < 128) {
    sendFanout(254, 0x90 | ((ch - 1) & 0x0F), action, 127);
    pulse.active = true;
    pulse.isCc = false;
    pulse.number = action;
    pulse.offAtMs = millis() + BUTTON_PULSE_MS;
  } else {
    const uint8_t cc = action - 128 + 1;
    sendFanout(254, 0xB0 | ((ch - 1) & 0x0F), cc, 127);
    pulse.active = true;
    pulse.isCc = true;
    pulse.number = cc;
    pulse.offAtMs = millis() + BUTTON_PULSE_MS;
  }
}

ButtonPulse button1Pulse;
ButtonPulse button2Pulse;
bool button1State = false;
bool button2State = false;
uint32_t button1ChangeMs = 0;
uint32_t button2ChangeMs = 0;

void tickButtonPulse(ButtonPulse &pulse) {
  if (!pulse.active || millis() < pulse.offAtMs) return;
  const uint8_t ch = settings.remoteChannel;
  if (pulse.isCc) sendFanout(254, 0xB0 | ((ch - 1) & 0x0F), pulse.number, 0);
  else sendFanout(254, 0x80 | ((ch - 1) & 0x0F), pulse.number, 0);
  pulse.active = false;
}

void pollButtons() {
  const uint32_t now = millis();
  const bool b1 = digitalRead(PIN_BUTTON_1);
  const bool b2 = digitalRead(PIN_BUTTON_2);

  if (b1 != button1State && (now - button1ChangeMs) > BUTTON_DEBOUNCE_MS) {
    button1ChangeMs = now;
    button1State = b1;
    if (b1) {
      markActivity(false);
      triggerRemoteAction(settings.remote1Action, button1Pulse);
    }
  }
  if (b2 != button2State && (now - button2ChangeMs) > BUTTON_DEBOUNCE_MS) {
    button2ChangeMs = now;
    button2State = b2;
    if (b2) {
      markActivity(false);
      triggerRemoteAction(settings.remote2Action, button2Pulse);
    }
  }

  tickButtonPulse(button1Pulse);
  tickButtonPulse(button2Pulse);
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

void pollSensor() {
  if (!sensorRt.present) return;
  const uint32_t now = millis();
  if ((now - sensorRt.lastPollMs) < SENSOR_POLL_MS) return;
  sensorRt.lastPollMs = now;
  const bool wasInRange = sensorRt.inRange;

  uint16_t mm = tof.readRangeContinuousMillimeters();
  if (tof.timeoutOccurred()) {
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
    if (!wasInRange && ui.selectedSetting == SET_LOOP_BARS && ui.menuMode == MENU_EDIT) {
      settings.loopAutoOverdub = settings.loopAutoOverdub ? 0 : 1;
      ui.dirty = true;
      return;
    }
  } else {
    pushRt.pct = 0;
  }

  if (wasInRange != pushRt.inRange && sensorParamEligible(ui.selectedSetting)) {
    ui.dirty = true;
  }

  updatePushOutput();
}

String settingValueString(uint8_t id) {
  const int16_t v = effectiveSettingValue(id);
  switch (id) {
    case SET_BPM: {
      return String(v);
    }
    case SET_ARP_MODE: return kArpSelectionNames[v];
    case SET_DIVISION: return kDivisionNames[v];
    case SET_VELOCITY: return String(map(v, 0, 127, 0, 100)) + "%";
    case SET_LENGTH: return String(v) + "%";
    case SET_PATTERN: return "";
    case SET_INPUT_CH: return midiChannelLabel(v);
    case SET_ARP_OUT_CH: return midiChannelLabel(v, true);
    case SET_BASS_CH: return bassLabel(v);
    case SET_THRU_OUT_CH: return midiChannelLabel(v, true);
    case SET_RND_RBN:
      if (v == RND_RBN_CH10_TO_1_SLOT) return roundRobinCh10To1Enabled() ? "[x]CH10-1+" : "[ ]CH10-1+";
      if (v == RND_RBN_CH10_TO_2_SLOT) return roundRobinCh10To2Enabled() ? "[x]CH10-2+" : "[ ]CH10-2+";
      if (v == RND_RBN_CLEAR_SLOT) return "CLEAR";
      if (v == RND_RBN_BACK_SLOT) return "BACK";
      return String((settings.roundRobinMask & channelBit(v + 1)) ? "[x] CH " : "[ ] CH ") + String(v + 1);
    case SET_ROUTER:
      if (routerEditStage == ROUTER_STAGE_LIST && v == ROUTER_CLEAR_SLOT) return "CLEAR";
      if (routerEditStage == ROUTER_STAGE_LIST && v == ROUTER_BACK_SLOT) return "BACK";
      return "";
    case SET_DIV_NOTES:
      if (v == DIV_NOTE_PLUS_SLOT) return "+NOTE";
      if (v == DIV_NOTE_RESET_SLOT) return "RESET";
      if (v == DIV_NOTE_BACK_SLOT) return "BACK";
      return kDivisionNames[divNoteSlotToDivision(v)];
    case SET_MAP_CC:
      if (v == MAP_CC_CLEAR_SLOT) return "CLEAR";
      if (v == MAP_CC_CHMODE_SLOT) return mapCcChannelAll ? "CH:ALL" : "CH:SET";
      return kMapCcParamNames[v];
    case SET_LEGATO_CH: return midiChannelLabel(v, true);
    case SET_CC_OUT_CH: return ccChannelLabel(v);
    case SET_REMOTE_CH: return midiChannelLabel(v);
    case SET_SENSOR_CH: return midiChannelLabel(v);
    case SET_SENSOR_MODE: return kSensorModeNames[v];
    case SET_PUSH_MODE: return kSensorModeNames[v];
    case SET_LOOP_BARS: return kLoopBarsNames[v];
    case SET_FORCE_KEY:
      if (v == 0) return "OFF";
      if (v <= 12) return String(kNoteNames[v - 1]);
      return String("CKEY ") + kNoteNames[v - 13];
    case SET_FORCE_SCALE: return kForceScaleNames[v];
    case SET_GUITAR_PIANO: return (v == 0) ? "GUITAR" : "PIANO";
    case SET_REMOTE1: return remoteActionLabel(v);
    case SET_REMOTE2: return remoteActionLabel(v);
    case SET_LOAD_PRESET: return String(v + 1);
    case SET_SAVE_PRESET: return String(v + 1);
    case SET_SCREEN_SAVER:
      if (v == 0) return "OFF";
      if (v == 1) return "AUTO";
      return "NOW";
    case SET_PANIC: return "CLICK";
    default: return "";
  }
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
  } else if (ui.selectedSetting == SET_FORCE_SCALE) {
    display.print(kForceScaleNames[effectiveSettingValue(SET_FORCE_SCALE)]);
  } else if (ui.selectedSetting == SET_LOAD_PRESET && mapCcLoadInProgress) {
    display.print(F("LOADING"));
  } else {
    display.print(kSettingNames[ui.selectedSetting]);
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

void drawDivisionPie(uint8_t divisionId) {
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
  display.print(kDivisionNames[divisionId]);
}

void drawArpModeSymbol(uint8_t mode) {
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.print(kArpSelectionNames[mode]);
  const int y = 40;
  if (mode == ARPSEL_UP) {
    display.drawLine(10, y, 30, y - 20, SSD1306_WHITE);
    display.drawLine(30, y - 20, 50, y, SSD1306_WHITE);
    display.drawLine(50, y, 70, y - 20, SSD1306_WHITE);
  } else if (mode == ARPSEL_DOWN) {
    display.drawLine(10, y - 20, 30, y, SSD1306_WHITE);
    display.drawLine(30, y, 50, y - 20, SSD1306_WHITE);
    display.drawLine(50, y - 20, 70, y, SSD1306_WHITE);
  } else if (mode == ARPSEL_TRIGGER) {
    for (uint8_t i = 0; i < 4; ++i) display.drawLine(18 + i * 22, 34, 18 + i * 22, 44, SSD1306_WHITE);
  } else if (mode == ARPSEL_RANDOM) {
    display.drawLine(8, 42, 22, 30, SSD1306_WHITE);
    display.drawLine(22, 30, 42, 44, SSD1306_WHITE);
    display.drawLine(42, 44, 68, 26, SSD1306_WHITE);
    display.drawLine(68, 26, 94, 38, SSD1306_WHITE);
  } else if (mode == ARPSEL_OFF) {
    display.drawLine(8, 26, 94, 42, SSD1306_WHITE);
    display.drawLine(8, 42, 94, 26, SSD1306_WHITE);
  } else {
    display.drawLine(8, 42, 24, 26, SSD1306_WHITE);
    display.drawLine(24, 26, 40, 42, SSD1306_WHITE);
    display.drawLine(40, 42, 56, 26, SSD1306_WHITE);
    display.drawLine(56, 26, 72, 42, SSD1306_WHITE);
  }
}

void drawPatternPreview(uint8_t pat) {
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(kPatternNames[pat]);
  const uint8_t y = 28;
  for (uint8_t i = 0; i < 12; ++i) {
    const PatternToken &t = kPatterns[pat][i];
    const uint8_t x = 8 + i * 10;
    if (t.noteIndex == TOK_REST) {
      display.drawLine(x - 3, y, x + 3, y, SSD1306_WHITE);
    } else if (t.noteIndex == TOK_ALL) {
      display.drawCircle(x, y, 3, SSD1306_WHITE);
      display.drawCircle(x, y, 1, SSD1306_WHITE);
    } else {
      display.fillCircle(x, y - (t.octaveOffset * 4), 2, SSD1306_WHITE);
    }
  }
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

void drawModeIndicator() {
  const int x = 122;
  const int editY = SETTING_AREA_Y + SETTING_AREA_H - 4;
  const int selectY = MODE_INFO_Y + (MODE_INFO_H / 2);
  if (ui.menuMode == MENU_EDIT) display.fillCircle(x, editY, 3, SSD1306_WHITE);
  else display.fillCircle(x, selectY, 3, SSD1306_WHITE);
}

void drawLoopStatusIcon() {
  if (!loopRecordingArmed && !loopRecording && !loopOverdubbing && !loopPlaying && !loopHasData) return;
  const int x = 116;
  const int y = 1;
  display.fillRect(x - 2, y, 14, 12, SSD1306_BLACK);

  if (loopRecordingArmed || loopRecording || loopOverdubbing) {
    display.drawCircle(x + 5, y + 6, 4, SSD1306_WHITE);
  } else if (loopPlaying) {
    display.drawTriangle(x + 2, y + 2, x + 2, y + 10, x + 10, y + 6, SSD1306_WHITE);
  } else if (loopHasData) {
    display.drawLine(x + 3, y + 2, x + 3, y + 10, SSD1306_WHITE);
    display.drawLine(x + 4, y + 2, x + 4, y + 10, SSD1306_WHITE);
    display.drawLine(x + 8, y + 2, x + 8, y + 10, SSD1306_WHITE);
    display.drawLine(x + 9, y + 2, x + 9, y + 10, SSD1306_WHITE);
  }
}

void drawChannelScreen(const __FlashStringHelper *title, int channel, bool allowOff = false) {
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.print(title);
  if (allowOff && channel == 0) {
    display.setTextSize(3);
    display.setCursor(0, 18);
    display.print(F("OFF"));
  } else if (channel == ARP_CH_1_PLUS_10) {
    display.setTextSize(3);
    display.setCursor(0, 18);
    display.print(F("1+10"));
  } else if (channel == ARP_CH_1_PLUS_10_AFTERTOUCH) {
    display.setTextSize(3);
    display.setCursor(0, 18);
    display.print(F("1+10-A"));
  } else if (channel == ARP_CH_1_TO_10_SPLIT_24) {
    display.setTextSize(3);
    display.setCursor(0, 18);
    display.print(F("1-10 24"));
  } else if (channel == ARP_CH_1_TO_10_SPLIT_36) {
    display.setTextSize(3);
    display.setCursor(0, 18);
    display.print(F("1-10 36"));
  } else if (channel == ARP_CH_1_TO_10_SPLIT_48) {
    display.setTextSize(3);
    display.setCursor(0, 18);
    display.print(F("1-10 48"));
  } else {
    display.setTextSize(3);
    display.setCursor(0, 18);
    display.print(F("CH "));
    display.print(channel);
  }
}

void drawBassScreen(uint8_t mode) {
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.print(F("BASS "));
  if (mode == 0) {
    display.setTextSize(3);
    display.setCursor(0, 18);
    display.print(F("OFF"));
    return;
  }
  const uint8_t channel = bassModeChannel(mode);
  const int8_t octaves = bassModeOctaveOffset(mode);
  display.setTextSize(2);
  display.setCursor(60, 0);
  if (octaves > 0) display.print(F("+1oct"));
  else if (octaves == 0) display.print(F("0oct"));
  else {
    display.print(octaves);
    display.print(F("oct"));
  }
  display.setTextSize(3);
  display.setCursor(0, 18);
  display.print(F("CH "));
  display.print(channel);
}

void drawCcChannelScreen(uint8_t channel) {
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.print(F("CC"));
  if (channel == 17) {
    display.setTextSize(3);
    display.setCursor(0, 18);
    display.print(F("ALL3"));
    return;
  }
  display.setTextSize(3);
  display.setCursor(0, 18);
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
  return out;
}

void drawRoundRobinScreen(uint8_t cursor) {
  display.setTextColor(SSD1306_WHITE);
  if (ui.menuMode == MENU_SELECT) {
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print(F("ROUND ROBIN"));
    display.setCursor(0, 14);
    display.print(roundRobinChannelList());
    return;
  }

  display.setTextSize(2);
  display.setCursor(0, 0);
  display.print(F("RNDRBN"));
  display.setCursor(0, 22);
  if (cursor < 16) {
    display.print((settings.roundRobinMask & channelBit(cursor + 1)) ? F("[x] ") : F("[ ] "));
    display.print(F("CH "));
    display.print(cursor + 1);
  } else if (cursor == RND_RBN_CH10_TO_1_SLOT) {
    display.print(roundRobinCh10To1Enabled() ? F("[x]CH10-1+") : F("[ ]CH10-1+"));
  } else if (cursor == RND_RBN_CH10_TO_2_SLOT) {
    display.print(roundRobinCh10To2Enabled() ? F("[x]CH10-2+") : F("[ ]CH10-2+"));
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
    display.setCursor(0, 0);
    display.print(F("ROUTER"));
    display.setCursor(0, 14);
    display.print(routerActiveList());
    return;
  }

  display.setTextSize(2);
  display.setCursor(0, 0);
  display.print(F("ROUTER"));
  display.setCursor(0, 22);
  if (routerEditStage == ROUTER_STAGE_LIST) {
    if (cursor < 16) display.print(routerEntryString(cursor));
    else if (cursor == ROUTER_CLEAR_SLOT) display.print(F("CLEAR"));
    else display.print(F("BACK"));
  } else {
    display.print(routerEntryString(routerEditChannel));
    display.setTextSize(1);
    display.setCursor(0, 39);
    display.print(routerEditStage == ROUTER_STAGE_DEST ? F("OUT CH") : F("TRANSPOSE"));
  }
}

void drawLoopBarsScreen(uint8_t value) {
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.print(kLoopBarsNames[value]);
  display.setTextSize(1);
  display.setCursor(0, 18);
  display.print(F("Auto Overdub"));
  display.setCursor(0, 30);
  display.print(settings.loopAutoOverdub ? F("On") : F("Off"));
  display.setCursor(0, 40);
  display.print(F("(Push)"));
}

void drawDivNotesScreen(uint8_t cursor) {
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.print(F("DIV NOTES"));
  if (cursor == DIV_NOTE_PLUS_SLOT) {
    display.setTextSize(2);
    display.setCursor(0, 18);
    display.print(F("+NOTE"));
    display.setTextSize(1);
    display.setCursor(0, 38);
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
    display.setCursor(0, 18);
    display.print(F("RESET"));
    return;
  }
  if (cursor == DIV_NOTE_BACK_SLOT) {
    display.setTextSize(3);
    display.setCursor(0, 18);
    display.print(F("BACK"));
    return;
  }
  const uint8_t divId = divNoteSlotToDivision(cursor);
  display.setTextSize(2);
  display.setCursor(0, 18);
  display.print(kDivisionNames[divId]);
  display.setTextSize(1);
  display.setCursor(0, 38);
  const uint8_t mapCh = settings.divNoteChannels[cursor];
  const uint8_t mapNote = settings.divNoteNotes[cursor];
  if (mapCh == 0 || mapNote == 0xFF) {
    display.print(F("LEARN NOTE"));
  } else {
    display.print(F("CH "));
    display.print(mapCh);
    display.print(F(" "));
    display.print(kNoteNames[mapNote % 12]);
    display.print(mapNote / 12);
    display.print(F(" "));
    display.print(mapNote);
  }
}

void drawMapCcScreen(uint8_t cursor) {
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.print(F("MAP CC"));

  if (cursor == MAP_CC_CLEAR_SLOT) {
    display.setTextSize(3);
    display.setCursor(0, 18);
    display.print(F("CLEAR"));
    return;
  }

  if (cursor == MAP_CC_CHMODE_SLOT) {
    display.setTextSize(2);
    display.setCursor(0, 18);
    display.print(F("CH:"));
    display.print(mapCcChannelAll ? F("ALL") : F("SET"));
    display.setTextSize(1);
    display.setCursor(0, 38);
    display.print(F("CC MATCH MODE"));
    return;
  }

  display.setTextSize(2);
  display.setCursor(0, 18);
  display.print(kMapCcParamNames[cursor]);

  display.setTextSize(1);
  display.setCursor(0, 38);
  const CcMapSlot &slot = mapCcSlots[cursor];
  if (slot.cc > 127 || slot.channel == 0) {
    display.print(F("LEARN CC"));
    return;
  }
  display.print(F("CC "));
  display.print(slot.cc);
  display.print(F(" CH "));
  display.print(slot.channel);
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
      break;
    case SET_ARP_MODE:
      if (arpSelectionIsClassicMode(v)) drawArpModeSymbol(v);
      else drawPatternPreview(patternFromArpSelection(v));
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
    case SET_INPUT_CH:
      drawChannelScreen(F("INPUT"), v);
      break;
    case SET_ARP_OUT_CH:
      drawChannelScreen(F("ARP"), v, true);
      break;
    case SET_BASS_CH:
      drawBassScreen(v);
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
      drawMapCcScreen(v);
      break;
    case SET_LEGATO_CH:
      drawChannelScreen(F("MONO RETRIG"), v, true);
      break;
    case SET_CC_OUT_CH:
      drawCcChannelScreen(v);
      break;
    case SET_REMOTE_CH:
      drawChannelScreen(F("REMOTE"), v);
      break;
    case SET_SENSOR_CH:
      drawChannelScreen(F("SENSORS"), v);
      break;
    case SET_LOOP_BARS:
      drawLoopBarsScreen(v);
      break;
    case SET_FORCE_KEY:
    case SET_FORCE_SCALE:
    case SET_GUITAR_PIANO:
      if (effectiveSettingValue(SET_GUITAR_PIANO) == 0) drawGuitarView();
      else drawPianoView();
      break;
    case SET_REMOTE1:
    case SET_REMOTE2:
      drawRemoteActionScreen(v);
      break;
    case SET_LOAD_PRESET:
      if (mapCcLoadInProgress) {
        drawWrappedTopValue("LOADING");
        break;
      }
      drawPresetGrid(v);
      break;
    case SET_SAVE_PRESET:
      drawPresetGrid(v);
      break;
    default:
      drawWrappedTopValue(settingValueString(id));
      break;
  }
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

void renderDisplayIfNeeded() {
  const uint32_t now = millis();
  if (screenSaverForceNow || (settings.screenSaver && (now - ui.lastActivityMs) > SCREEN_SAVER_IDLE_MS)) {
    if (!ui.inSaver || (now - ui.lastRenderMs) > SCREEN_SAVER_REFRESH_MS) {
      ui.inSaver = true;
      drawScreenSaver();
      ui.lastRenderMs = now;
    }
    return;
  }

  if (!ui.dirty) return;
  ui.inSaver = false;
  display.clearDisplay();
  renderMainTop();
  drawLoopStatusIcon();
  moveRenderedSettingArea();
  drawModeLabel();
  drawModeIndicator();
  display.display();
  ui.dirty = false;
  ui.lastRenderMs = now;
}

void showBusyHourglass() {
  const int y = SETTING_AREA_Y + 23;
  display.fillRect(116, y, 12, 18, SSD1306_BLACK);
  display.drawTriangle(118, y + 1, 126, y + 1, 122, y + 8, SSD1306_WHITE);
  display.drawTriangle(118, y + 16, 126, y + 16, 122, y + 9, SSD1306_WHITE);
  display.display();
}

void processDeferredUiActions() {
  if (!ui.deferredExitWork) return;
  showBusyHourglass();

  if (ui.deferredSaveOnly) {
    storage.currentPreset = settings.savePreset;
    saveStorage();
    settings.loadPreset = storage.currentPreset;
    settings.savePreset = storage.currentPreset;
    ui.dirty = true;
  } else if (ui.deferredLoadPreset) {
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
      for (uint8_t i = 0; i < DIV_NOTE_SLOT_COUNT; ++i) {
        settings.divNoteChannels[i] = 0;
        settings.divNoteNotes[i] = 0xFF;
        divNoteHeld[i] = false;
        physicalDivNoteHeld[i] = false;
        loopDivNoteHeld[i] = false;
        divNoteHeldStamp[i] = 0;
      }
      settings.divNotePlusNote = 0xFF;
      divNotePressCounter = 0;
      ui.dirty = true;
    }
    if (ui.selectedSetting == SET_MAP_CC && mapCcCursor == MAP_CC_CLEAR_SLOT) {
      clearMapCcMappings();
      ui.dirty = true;
    }
    if (ui.selectedSetting == SET_ROUTER) {
      panicMidiOnly();
      ui.dirty = true;
    }
    saveStorage();
  }

  ui.deferredExitWork = false;
  ui.deferredLoadPreset = false;
  ui.deferredSaveOnly = false;

  ui.dirty = true;
  renderDisplayIfNeeded();
}

void pollUiScreenPersistence() {
  const uint32_t now = millis();
  if (ui.selectedSetting != observedUiSetting) {
    observedUiSetting = ui.selectedSetting;
    uiScreenSavePending = (ui.selectedSetting != persistedUiSetting);
    uiScreenChangedMs = now;
  }
  if (!uiScreenSavePending || (now - uiScreenChangedMs) < UI_SCREEN_SAVE_IDLE_MS) return;
  if (ui.deferredExitWork || loopRecordingArmed || loopRecording || loopPlaying || loopOverdubbing) return;
  if (anyPhysicalInputNotesHeld() || heldDrumCount > 0 || arpAnyPlaybackActive()) return;
  stagePersistedUiSetting(ui.selectedSetting);
  EEPROM.commit();
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
  DinSerial.begin(31250);
  DinMIDI.begin(MIDI_CHANNEL_OMNI);
  DinMIDI.turnThruOff();
  DinMIDI.setHandleNoteOn(handleDinNoteOn);
  DinMIDI.setHandleNoteOff(handleDinNoteOff);
  DinMIDI.setHandleControlChange(handleDinCc);
  DinMIDI.setHandlePitchBend(handleDinPb);
  DinMIDI.setHandleProgramChange(handleDinProgramChange);
  DinMIDI.setHandleAfterTouchChannel(handleDinAfterTouchChannel);
  DinMIDI.setHandleClock(handleDinClock);
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

    const uint8_t status = packet[1];
    const uint8_t data1 = (len > 1) ? packet[2] : 0;
    const uint8_t data2 = (len > 2) ? packet[3] : 0;
    if (status >= 0xF8) {
      if (status == 0xF8) handleClockByte(false);
      sendFanout(USB_DEVICE_SOURCE_PORT, status, 0, 0);
    } else if (status >= 0x80 && status <= 0xEF) {
      routeIncomingChannelMessage(USB_DEVICE_SOURCE_PORT, status, data1, data2);
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
  pinMode(PIN_PUSH, INPUT);
  encoder.lastAB = (digitalRead(PIN_ENC_A) << 1) | digitalRead(PIN_ENC_B);
  encoder.lastSwitch = digitalRead(PIN_ENC_SW);
  button1State = digitalRead(PIN_BUTTON_1);
  button2State = digitalRead(PIN_BUTTON_2);
#if ARPNMIDI_ENABLE_RGB_LED
  onboardRgb.begin();
  onboardRgb.setBrightness(80);
  onboardRgb.setPixelColor(0, onboardRgb.Color(0, 0, 0));
  onboardRgb.show();
#endif
}

void factoryResetStorage() {
  EEPROM.begin(EEPROM_BYTES);
  for (size_t i = 0; i < EEPROM_BYTES; ++i) EEPROM.write(i, 0xFF);
  EEPROM.commit();
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
  initStorageIfNeeded();
  uint8_t resumeSetting = SET_BPM;
  if (loadPersistedUiSetting(resumeSetting)) ui.selectedSetting = resumeSetting;
  if (takeUiResumeHint(resumeSetting) && resumeSetting < SETTING_COUNT && selectableSetting(resumeSetting)) {
    ui.selectedSetting = resumeSetting;
  }
  observedUiSetting = ui.selectedSetting;
  persistedUiSetting = ui.selectedSetting;
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
}

void loop() {
#ifdef TINYUSB_NEED_POLLING_TASK
  TinyUSBDevice.task();
#endif
  pollEncoder();
  pollButtons();
  DinMIDI.read();
  pumpUsbDeviceMidiInput();
  pollSensor();
  pollPush();
  tickLooper();
  tickArp();
  pollMapCcUiFocus();
  renderDisplayIfNeeded();
  processDeferredUiActions();
  pollUiScreenPersistence();
}
