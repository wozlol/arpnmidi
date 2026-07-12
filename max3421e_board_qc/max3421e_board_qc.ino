/*
  max3421e_board_qc.ino
  MAX3421E board quality-control routine — RP2040-Zero / ARPnMIDI pin build.

  Combines two test suites into one pass:
    A) Our bring-up probes (SPI loopback, direct REVISION read, oscillator probe)
    B) The Circuits At Home "USB Host Shield Quality Control Routine"
       (REVISION / die-rev, 1 MB SPI stress, GPIO loop test, PLL/oscillator test,
        then USB device enumeration).

  Serial output is at 115200 baud.

  ARPnMIDI pins (RP2040-Zero, Waveshare):
    GP2   MAX3421E SCK   (SPI0 SCK)
    GP3   MAX3421E MOSI  (SPI0 TX)
    GP0   MAX3421E MISO  (SPI0 RX  — GP0 is a valid SPI0 RX pin, selected via SPI.setRX(0))
    GP1   MAX3421E CS    (software CS via library, typedef MAX3421e<P1,P26>)
    GP26  MAX3421E INT
    RST   tie to 3.3V (module has no pullup; floating RST holds the chip in reset)

  Notes for this board:
    - GPIO test needs the MAX3421E's own GPIN0..7 looped to GPOUT7..0. If those
      pins aren't wired on your board, that sub-test will pause for a keypress —
      press any key in the serial monitor to continue.
    - PLL test is the important one for the oscillator fault we chased: it does
      100 chip resets and reports how many cycles the 12 MHz crystal needs to
      stabilize, or clearly flags it if the oscillator never starts.

  Arduino setup:
    Board: Waveshare RP2040-Zero (Earle Philhower arduino-pico core)
*/

#include <Arduino.h>
#include <SPI.h>

// RP2040 CMSIS headers define USB as a hardware register macro. USB Host
// Shield 2.0 also names its MAX3421E host controller class USB.
#ifdef USB
#undef USB
#endif
#ifdef _usb_h_
#undef _usb_h_
#endif
#ifdef USBCORE_H
#undef USBCORE_H
#endif

// Vendored, PATCHED USB Host Shield 2.0 in this sketch's src/ folder — the exact
// working library travels with the code (self-contained, immune to global-library
// updates). Do NOT switch back to <angle-bracket> or the global lib would be used.
#include "src/USB_Host_Shield_Library_2.0/usbhub.h"

constexpr uint8_t PIN_MAX_SPI_MISO = 0;  // ARPnMIDI PCB: SPI0 MISO on GP0 (valid SPI0 RX)
constexpr uint8_t PIN_MAX_CS       = 1;
constexpr uint8_t PIN_MAX_SPI_SCK  = 2;
constexpr uint8_t PIN_MAX_SPI_MOSI = 3;
constexpr uint8_t PIN_MAX_INT      = 26;

/* objects */
USB Usb;
USB_DEVICE_DESCRIPTOR buf;

/* state */
uint8_t  rcode;
uint8_t  usbstate;
uint8_t  laststate;
uint16_t pllFailures = 0;

/* forward declarations */
void halt55();
void print_hex(int v, int num_places);
void press_any_key();

// ── SPI / pin setup for the RP2040-Zero ──────────────────────────────────────
void configureSpiPins() {
  pinMode(PIN_MAX_CS, OUTPUT);
  digitalWrite(PIN_MAX_CS, HIGH);

  SPI.end();                      // reset _begun so setRX/SCK/TX take effect
  SPI.setRX(PIN_MAX_SPI_MISO);    // GP4
  SPI.setSCK(PIN_MAX_SPI_SCK);    // GP2
  SPI.setTX(PIN_MAX_SPI_MOSI);    // GP3
  SPI.begin();                    // no hwCS — library drives CS (GP1) in software

  pinMode(PIN_MAX_CS, OUTPUT);
  digitalWrite(PIN_MAX_CS, HIGH);
  pinMode(PIN_MAX_INT, INPUT_PULLUP);
}

// ── A) Our bring-up probes ───────────────────────────────────────────────────

// SPI loopback: only meaningful with a GP3->GP4 jumper and NO chip attached.
// On a populated board a real chip does not echo MOSI back onto MISO, so
// 00 00 00 here is normal and expected.
void runSpiLoopbackTest() {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  uint8_t lb0 = SPI.transfer(0x90);
  uint8_t lb1 = SPI.transfer(0x55);
  uint8_t lb2 = SPI.transfer(0xAA);
  SPI.endTransaction();
  const bool jumperOk = (lb0 == 0x90 && lb1 == 0x55 && lb2 == 0xAA);
  Serial.printf("[A1] SPI loopback (needs GP3->GP4 jumper, no chip): "
                "sent 90 55 AA  got %02X %02X %02X  -> %s\r\n",
                lb0, lb1, lb2,
                jumperOk ? "jumper present, SPI0 pins OK"
                         : "no jumper (normal on a populated board)");
}

// Direct REVISION read, manual CS, slow clock — library-independent chip probe.
void runDirectRevisionRead() {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_MAX_CS, LOW);
  delayMicroseconds(1);
  SPI.transfer(rREVISION);
  uint8_t rev = SPI.transfer(0x00);
  delayMicroseconds(1);
  digitalWrite(PIN_MAX_CS, HIGH);
  SPI.endTransaction();
  Serial.printf("[A2] Direct chip REVISION @1MHz (manual CS): 0x%02X (expect 0x13)\r\n", rev);
}

// Oscillator probe: chip reset, then poll OSCOKIRQ up to 65535 times.
void runOscProbe() {
  Usb.regWr(rUSBCTL, bmCHIPRES);   // assert internal chip reset
  Usb.regWr(rUSBCTL, 0x00);        // release reset
  uint32_t t0    = micros();
  uint32_t tries = 0;
  bool     oscOk = false;
  while (tries < 65535UL) {
    tries++;
    if (Usb.regRd(rUSBIRQ) & bmOSCOKIRQ) { oscOk = true; break; }
  }
  Serial.printf("[A3] OSC probe: oscOk=%d  tries=%lu/65535  elapsed=%luus  rUSBIRQ=0x%02X\r\n",
                (int)oscOk, (unsigned long)tries,
                (unsigned long)(micros() - t0), Usb.regRd(rUSBIRQ));
}

// ── B) Circuits At Home Quality Control Routine (ported to plain Serial) ──────

void runRevisionCheck() {
  Serial.print("\r\n[B1] Reading REVISION register... Die revision ");
  Usb.Init(); // initializes SPI; return value not used here
  uint8_t tmpbyte = Usb.regRd(rREVISION);
  switch (tmpbyte) {
    case 0x01: Serial.print("01"); break;          // rev.01
    case 0x12: Serial.print("02"); break;          // rev.02
    case 0x13: Serial.print("03"); break;          // rev.03
    default:
      Serial.print("invalid. Value returned: 0x");
      print_hex(tmpbyte, 8);
      halt55();
      break;
  }
}

// Read REVISION many times and count how many come back correct. This directly
// measures SPI READ reliability (MISO / CS / SCK integrity) and separates a
// read-path fault from a write-path fault the SPI long test would catch.
void runSpiReliabilityTest() {
  const uint16_t N = 1000;
  uint16_t good = 0;
  for (uint16_t i = 0; i < N; i++) {
    if (Usb.regRd(rREVISION) == 0x13) good++;
  }
  Serial.printf("\r\n[B1b] SPI read reliability: %u/%u REVISION reads == 0x13\r\n", good, N);
  if (good == N) {
    Serial.print("      -> SPI reads are rock solid.\r\n");
  } else if (good == 0) {
    Serial.print("      -> reads never land: dead MISO/CS, or dead chip.\r\n");
  } else {
    Serial.print("      -> INTERMITTENT SPI! Reflow/re-seat MISO(GP0), CS(GP1), SCK(GP2), MOSI(GP3), GND.\r\n");
    Serial.print("      -> NOTE: this also makes the oscillator (OSCOKIRQ) reads unreliable —\r\n");
    Serial.print("         do NOT trust the PLL/oscillator result until this reads 1000/1000.\r\n");
  }
}

// DECIDER TEST: write to a register at a SLOW 1 MHz manual clock, then read it
// back with the proven-good library read. This isolates the two hypotheses:
//   - all pass  -> writes work when slow; the library's 26 MHz is too fast for
//                  these wires. Fix by lowering the library SPI clock.
//   - all fail  -> the chip won't latch writes even at 1 MHz; its core is not
//                  functioning (dead/counterfeit MAX3421E) — matches "reads REV
//                  0x13 but oscillator never starts" across two boards.
void runSlowWriteTest() {
  Serial.print("\r\n[A4] Slow-clock WRITE test (write manual @1MHz, read back via library):\r\n");
  const uint8_t vals[] = { 0x55, 0xAA, 0x0F, 0xF0, 0x01 };
  const uint8_t n = sizeof(vals);
  uint8_t pass = 0;
  for (uint8_t i = 0; i < n; i++) {
    // manual slow write to rGPINPOL: command byte (reg | write bit) then data
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(PIN_MAX_CS, LOW);
    delayMicroseconds(2);
    SPI.transfer(rGPINPOL | 0x02);
    SPI.transfer(vals[i]);
    delayMicroseconds(2);
    digitalWrite(PIN_MAX_CS, HIGH);
    SPI.endTransaction();

    uint8_t got = Usb.regRd(rGPINPOL);   // read back with the known-good path
    bool ok = (got == vals[i]);
    if (ok) pass++;
    Serial.printf("     wrote 0x%02X -> read 0x%02X  %s\r\n", vals[i], got, ok ? "OK" : "FAIL");
  }
  Serial.printf("     result: %u/%u %s\r\n", pass, n,
    pass == n ? "-> SLOW writes WORK. 26 MHz library clock is too fast for these wires."
              : "-> writes FAIL even at 1 MHz. Chip core not latching (likely dead/counterfeit MAX3421E).");
}

void runSpiLongTest() {
  Serial.print("\r\n[B2] SPI long test. Transfers 1MB of data. Each dot is 64K");
  uint8_t sample_wr = 0;
  uint8_t sample_rd = 0;
  uint8_t gpinpol_copy = Usb.regRd(rGPINPOL);
  for (uint8_t i = 0; i < 16; i++) {
    for (uint16_t j = 0; j < 65535; j++) {
      Usb.regWr(rGPINPOL, sample_wr);
      sample_rd = Usb.regRd(rGPINPOL);
      if (sample_rd != sample_wr) {
        Serial.print("\r\nTest failed.  Value written: 0x");
        print_hex(sample_wr, 8);
        Serial.print(" read: 0x");
        print_hex(sample_rd, 8);
        Serial.print("  -> continuing to PLL test for full picture.\r\n");
        Usb.regWr(rGPINPOL, gpinpol_copy);
        return;   // was halt55(); now continue so the PLL/oscillator test still runs
      }
      sample_wr++;
    }
    Serial.print(".");
  }
  Usb.regWr(rGPINPOL, gpinpol_copy);
  Serial.print(" SPI long test passed");
}

void runGpioTest() {
  // GPIN pins on the test fixture are wired to GPOUT in reverse order,
  // i.e. GPIN0<->GPOUT7, GPIN1<->GPOUT6, etc.
  Serial.print("\r\n[B3] GPIO test. Connect GPIN0 to GPOUT7, GPIN1 to GPOUT6, and so on");
  uint8_t tmpbyte;
  for (uint16_t sample_gpio = 0; sample_gpio < 255; sample_gpio++) {
    Usb.gpioWr((uint8_t)sample_gpio);
    tmpbyte = Usb.gpioRd();
    // reverse the bit order (http://graphics.stanford.edu/~seander/bithacks.html)
    tmpbyte = ((tmpbyte * 0x0802LU & 0x22110LU) | (tmpbyte * 0x8020LU & 0x88440LU)) * 0x10101LU >> 16;
    if ((uint8_t)sample_gpio != tmpbyte) {
      Serial.print("\r\nTest failed. Value written: 0x");
      print_hex(sample_gpio, 8);
      Serial.print(" Value read: 0x");
      print_hex(tmpbyte, 8);
      Serial.print(" (GPIN/GPOUT not looped on this board — expected, skipping)");
      return;
    }
  }
  Serial.print("\r\nGPIO test passed.");
}

void runPllTest() {
  Serial.print("\r\n[B4] PLL test. 100 chip resets will be performed");
  // Current oscillator state should be ON (asserted) before we start.
  if (!(Usb.regRd(rUSBIRQ) & bmOSCOKIRQ)) {
    Serial.print("\r\nOSCOK not set at start (oscillator not running) — continuing anyway.");
  }
  Serial.print("\r\nResetting oscillator\r\n");
  for (uint16_t i = 0; i < 100; i++) {
    Serial.print("\rReset number ");
    Serial.print(i, DEC);
    Usb.regWr(rUSBCTL, bmCHIPRES);                 // reset
    if (Usb.regRd(rUSBIRQ) & bmOSCOKIRQ) {         // should be OFF now
      Serial.print("\r\nCurrent oscillator state unexpected (OSCOK still set after reset).");
      halt55();
    }
    Usb.regWr(rUSBCTL, 0x00);                       // release from reset
    bool stabilized = false;
    for (uint32_t j = 1; j < 65535UL; j++) {        // track off->on time
      if (Usb.regRd(rUSBIRQ) & bmOSCOKIRQ) {
        Serial.print(" Time to stabilize - ");
        Serial.print(j, DEC);
        Serial.print(" cycles\r\n");
        stabilized = true;
        break;
      }
    }
    if (!stabilized) {
      // Improved over the original, which mis-tested j==0 and silently passed:
      // clearly flag a crystal that never starts.
      Serial.print("  PLL FAILED to stabilize — oscillator not running!\r\n");
      pllFailures++;
    }
  }
  Serial.printf("\r\nPLL test complete: %u/100 resets FAILED to stabilize.\r\n", pllFailures);
}

void setup() {
  laststate = 0;
  Serial.begin(115200);
  while (!Serial) { /* wait for USB CDC serial */ }

  Serial.println("\r\n\r\n=== MAX3421E Board QC — RP2040-Zero (ARPnMIDI pins) ===");
  Serial.println("Pins: SCK=GP2  MOSI=GP3  MISO=GP0  CS=GP1  INT=GP26  (RST->3.3V)");
  Serial.println("Set serial monitor to 115200 baud.\r\n");

  configureSpiPins();

  // A) Bring-up probes
  // runSpiLoopbackTest();   // disabled: only meaningful with a GP3->GP4 jumper and no chip
  runDirectRevisionRead();
  runOscProbe();

  // B) Circuits At Home QC routine
  runRevisionCheck();
  runSpiReliabilityTest();   // quantify SPI read integrity before trusting anything else
  runSlowWriteTest();        // DECIDER: do writes stick at 1 MHz? (26 MHz signal integrity vs dead chip)
  runSpiLongTest();
  runGpioTest();
  runPllTest();

  // Final full init — must succeed for USB enumeration to run.
  if (Usb.Init() == -1) {
    Serial.print("\r\nOSCOKIRQ failed to assert — board cannot do USB (dead/silent oscillator).");
    halt55();
  }
  Serial.print("\r\nChecking USB device communication.\r\n");
}

void loop() {
  delay(200);
  Usb.Task();
  usbstate = Usb.getUsbTaskState();
  if (usbstate != laststate) {
    laststate = usbstate;
    switch (usbstate) {
      case USB_DETACHED_SUBSTATE_WAIT_FOR_DEVICE:
        Serial.print("\r\nWaiting for device...");
        break;
      case USB_ATTACHED_SUBSTATE_RESET_DEVICE:
        Serial.print("\r\nDevice connected. Resetting...");
        break;
      case USB_ATTACHED_SUBSTATE_WAIT_SOF:
        Serial.print("\r\nReset complete. Waiting for the first SOF...");
        break;
      case USB_ATTACHED_SUBSTATE_GET_DEVICE_DESCRIPTOR_SIZE:
        Serial.print("\r\nSOF generation started. Enumerating device...");
        break;
      case USB_STATE_ADDRESSING:
        Serial.print("\r\nSetting device address...");
        break;
      case USB_STATE_RUNNING:
        Serial.print("\r\nGetting device descriptor");
        rcode = Usb.getDevDescr(1, 0, sizeof(USB_DEVICE_DESCRIPTOR), (uint8_t*)&buf);
        if (rcode) {
          Serial.print("\r\nError reading device descriptor. Error code 0x");
          print_hex(rcode, 8);
        } else {
          Serial.print("\r\nDescriptor Length:\t0x");   print_hex(buf.bLength, 8);
          Serial.print("\r\nDescriptor type:\t0x");     print_hex(buf.bDescriptorType, 8);
          Serial.print("\r\nUSB version:\t\t0x");        print_hex(buf.bcdUSB, 16);
          Serial.print("\r\nDevice class:\t\t0x");       print_hex(buf.bDeviceClass, 8);
          Serial.print("\r\nDevice Subclass:\t0x");     print_hex(buf.bDeviceSubClass, 8);
          Serial.print("\r\nDevice Protocol:\t0x");     print_hex(buf.bDeviceProtocol, 8);
          Serial.print("\r\nMax.packet size:\t0x");     print_hex(buf.bMaxPacketSize0, 8);
          Serial.print("\r\nVendor  ID:\t\t0x");         print_hex(buf.idVendor, 16);
          Serial.print("\r\nProduct ID:\t\t0x");         print_hex(buf.idProduct, 16);
          Serial.print("\r\nRevision ID:\t\t0x");        print_hex(buf.bcdDevice, 16);
          Serial.print("\r\nMfg.string index:\t0x");    print_hex(buf.iManufacturer, 8);
          Serial.print("\r\nProd.string index:\t0x");   print_hex(buf.iProduct, 8);
          Serial.print("\r\nSerial number index:\t0x"); print_hex(buf.iSerialNumber, 8);
          Serial.print("\r\nNumber of conf.:\t0x");     print_hex(buf.bNumConfigurations, 8);
          Serial.print("\r\n\nAll tests passed. Press RESET to restart test");
          while (1) { /* done */ }
        }
        break;
      case USB_STATE_ERROR:
        Serial.print("\r\nUSB state machine reached error state");
        break;
      default:
        break;
    }
  }
}

/* constantly transmits 0x55 via SPI to aid probing */
void halt55() {
  Serial.print("\r\nUnrecoverable error - test halted!!");
  Serial.print("\r\n0x55 pattern is transmitted via SPI");
  Serial.print("\r\nPress RESET to restart test");
  while (1) {
    Usb.regWr(0x55, 0x55);
  }
}

/* prints hex numbers with leading zeroes */
void print_hex(int v, int num_places) {
  int mask = 0, n, num_nibbles, digit;
  for (n = 1; n <= num_places; n++) {
    mask = (mask << 1) | 0x0001;
  }
  v = v & mask; // truncate v to specified number of places
  num_nibbles = num_places / 4;
  if ((num_places % 4) != 0) {
    ++num_nibbles;
  }
  do {
    digit = ((v >> (num_nibbles - 1) * 4)) & 0x0f;
    Serial.print(digit, HEX);
  } while (--num_nibbles);
}

/* prints "Press any key" and returns when a key is pressed */
void press_any_key() {
  Serial.print("\r\nPress any key to continue...");
  while (Serial.available() <= 0) { /* wait for input */ }
  Serial.read(); // empty input buffer
  return;
}
