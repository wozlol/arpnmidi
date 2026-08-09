/*
  Minimal USB2514B MIDI host test for RP2040 PIO-USB

  Purpose: test whether the 0-5 second MIDI RX stall is in the
  stack (PIO-USB/TinyUSB) or in the ARPnMIDI app code.

  Hardware: RP2040 Zero, USB2514B hub on GPIO14/15
  Bridge:   Secondary RP2040 holds hub RESET_N (GP24) — must have
            USB_HUB_RELEASE_COMMAND_TIMEOUT_MS = 0 in bridge firmware.
            This sketch sends AHR1 on GP4/GP5 and waits for AHD1 before
            starting PIO-USB.

  Pin assignments:
  GPIO4  = command UART TX to bridge (sends AHR1)
  GPIO5  = command UART RX from bridge (waits for AHD1)
  GPIO14 = USB host D+
  GPIO15 = USB host D-
*/

#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include "pio_usb_configuration.h"
#include "pio_usb.h"
#include <hardware/gpio.h>
#include <hardware/clocks.h>
#include <hardware/irq.h>
#include <hardware/timer.h>

Adafruit_USBH_Host USBHost;

volatile uint32_t midi_note_count = 0;
volatile uint32_t last_note_time_ms = 0;
volatile uint32_t usb_device_mount_count = 0;
volatile uint32_t usb_midi_mount_count = 0;

constexpr uint32_t HUB_READY_TO_HOST_START_MS = 100;

// ── callbacks (fire on core1) ────────────────────────────────────────────────

void tuh_mount_cb(uint8_t dev_addr) {
  uint16_t vid = 0, pid = 0;
  tuh_vid_pid_get(dev_addr, &vid, &pid);
  usb_device_mount_count++;
  Serial.printf("USB device mounted:   dev_addr=%d  VID=%04X PID=%04X\n", dev_addr, vid, pid);
}

void tuh_umount_cb(uint8_t dev_addr) {
  Serial.printf("USB device unmounted: dev_addr=%d\n", dev_addr);
}

void tuh_midi_mount_cb(uint8_t idx, const tuh_midi_mount_cb_t *mount_cb_data) {
  Serial.printf("MIDI mounted: idx=%d daddr=%d rx_cables=%d tx_cables=%d\n",
                idx,
                mount_cb_data->daddr,
                mount_cb_data->rx_cable_count,
                mount_cb_data->tx_cable_count);
  usb_midi_mount_count++;
  midi_note_count = 0;
  last_note_time_ms = millis();
}

void tuh_midi_umount_cb(uint8_t idx) {
  Serial.printf("MIDI unmounted: idx=%d\n", idx);
}

void tuh_midi_rx_cb(uint8_t idx, uint32_t xferred_bytes) {
  (void)xferred_bytes;
  uint8_t cable_num;
  uint8_t packet[48];
  uint32_t bytes_read;

  while ((bytes_read = tuh_midi_stream_read(idx, &cable_num, packet, sizeof(packet))) > 0) {
    uint8_t cin   = packet[0] & 0x0F;
    uint8_t midi0 = packet[1];
    uint8_t midi1 = packet[2];
    uint8_t midi2 = packet[3];

    if (cin == 0x9 || cin == 0x8) {
      midi_note_count++;
      last_note_time_ms = millis();
      if (midi_note_count % 10 == 0) {
        Serial.printf("Note #%lu  CIN=%d  %02x %02x %02x\n",
                      midi_note_count, cin, midi0, midi1, midi2);
      }
    }
  }
}

// ── hub release handshake ────────────────────────────────────────────────────

static void releaseHub() {
  const uint8_t CMD[]  = {'A', 'H', 'R', '1'};
  const uint8_t RESP[] = {'A', 'H', 'D', '1'};

  Serial2.setTX(4);
  Serial2.setRX(5);
  Serial2.begin(115200);
  while (Serial2.available()) Serial2.read();

  Serial.println("Sending AHR1 to bridge...");
  const uint32_t start = millis();
  uint32_t nextCmd = start;
  uint8_t matched = 0;

  while (millis() - start < 1000) {
    if ((int32_t)(millis() - nextCmd) >= 0) {
      Serial2.write(CMD, sizeof(CMD));
      Serial2.flush();
      nextCmd = millis() + 10;
    }
    while (Serial2.available()) {
      uint8_t b = Serial2.read();
      matched = (b == RESP[matched]) ? matched + 1 : (b == RESP[0] ? 1 : 0);
      if (matched == sizeof(RESP)) {
        Serial.println("AHD1 received — hub released");
        delay(HUB_READY_TO_HOST_START_MS);
        return;
      }
    }
    delay(1);
  }
  Serial.println("Hub handshake timed out (hub may already be released by timeout)");
}

// ── core0 ────────────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== ARPnMIDI USB2514B Minimal Test ===");

  releaseHub();
}

void loop() {
  static uint32_t last_print = 0;
  uint32_t now = millis();
  if (now - last_print >= 2000) {
    last_print = now;
    uint32_t silence = now - last_note_time_ms;
    Serial.printf("[%6lu ms] USB mounts: %lu  MIDI mounts: %lu  Notes: %lu  Silence: %lu ms\n",
                  now, usb_device_mount_count, usb_midi_mount_count, midi_note_count, silence);
    if (midi_note_count > 0 && silence > 10000)
      Serial.println("WARNING: MIDI silent >10 sec!");
  }
}

// ── core1: USB host ──────────────────────────────────────────────────────────

void setup1() {
  // Wait for core0 to finish hub handshake
  delay(1200);

  pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
  pio_cfg.pin_dp = 14;
  USBHost.configure_pio_usb(1, &pio_cfg);

  USBHost.begin(1);
  irq_set_priority(TIMER_IRQ_2, PICO_HIGHEST_IRQ_PRIORITY);
  Serial.println("PIO-USB host started on GPIO14/15");
}

void loop1() {
  USBHost.task(0);
}
