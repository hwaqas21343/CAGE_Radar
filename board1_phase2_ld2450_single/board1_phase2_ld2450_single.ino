// CAGE Phase 2 - Single LD2450 bring-up on Board #1  (WORKING)
// Wire: LD2450 TX -> GPIO 18, LD2450 RX -> GPIO 17, 5V -> 5V, GND -> GND
// Library: rbegamer/HLK-LD2450 (Library Manager)
//
// IMPORTANT: this project relies on a local fix to the library's decoder.
// LD2450.cpp was patched to decode X/Y/speed as SIGN-MAGNITUDE (bit 15 = sign),
// not two's complement. Without that patch, targets read as bogus values like
// x=-32000mm. If you reinstall/update the library, re-apply that patch.
//
// RAW_BYTES 1 = hex dump (wiring/baud check).  RAW_BYTES 0 = parsed targets.

#define RAW_BYTES 0

#include <HardwareSerial.h>

#if RAW_BYTES == 0
#include <LD2450.h>
#endif

// UART1 pins for LD2450 #1 (left side)
#define LD2450_RX 18
#define LD2450_TX 17

#define LD2450_MAX_RANGE_MM 6000   // sensor spec: 6m. Beyond this = bogus.

HardwareSerial ld2450Serial(1);   // UART1

#if RAW_BYTES == 0
LD2450 radar;
#endif

// The LD2450 defaults to SINGLE-target mode. Send the command sequence to switch
// it into multi-target tracking (up to 3). Protocol: header FD FC FB FA, 2-byte
// length, command word, footer 04 03 02 01. Setting persists in the sensor's flash.
void ld2450EnableMultiTarget(HardwareSerial &s) {
  uint8_t enterConfig[] = {0xFD,0xFC,0xFB,0xFA, 0x04,0x00, 0xFF,0x00, 0x01,0x00, 0x04,0x03,0x02,0x01};
  uint8_t multiTarget[] = {0xFD,0xFC,0xFB,0xFA, 0x02,0x00, 0x90,0x00, 0x04,0x03,0x02,0x01};
  uint8_t exitConfig[]  = {0xFD,0xFC,0xFB,0xFA, 0x02,0x00, 0xFE,0x00, 0x04,0x03,0x02,0x01};

  s.write(enterConfig, sizeof(enterConfig)); s.flush(); delay(100);
  s.write(multiTarget, sizeof(multiTarget)); s.flush(); delay(100);
  s.write(exitConfig,  sizeof(exitConfig));  s.flush(); delay(100);

  // Drain the sensor's ACK replies so they don't confuse the first parse.
  while (s.available()) s.read();
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println("CAGE Phase 2 - LD2450 bring-up");

  ld2450Serial.begin(256000, SERIAL_8N1, LD2450_RX, LD2450_TX);
  delay(100);
  ld2450EnableMultiTarget(ld2450Serial);   // switch sensor to multi-target mode

#if RAW_BYTES == 0
  radar.begin(ld2450Serial);
  Serial.println("Parser ready (multi-target mode)");
#else
  Serial.println("RAW BYTES mode");
#endif
}

void loop() {
#if RAW_BYTES

  while (ld2450Serial.available()) {
    uint8_t b = ld2450Serial.read();
    if (b < 0x10) Serial.print('0');
    Serial.print(b, HEX);
    Serial.print(' ');
  }

#else

  static unsigned long lastPrint = 0;

  // Read every loop to keep the UART buffer drained and data fresh...
  radar.read();

  // ...but only print twice a second so the output is readable.
  if (millis() - lastPrint >= 500) {
    lastPrint = millis();
    Serial.println("--- FRAME ---");

    bool anyTarget = false;
    for (int i = 0; i < 3; i++) {
      LD2450::RadarTarget t = radar.getTarget(i);

      // Empty slots read 0; anything past the sensor's range is bogus.
      if (t.distance == 0 || t.distance > LD2450_MAX_RANGE_MM) continue;

      anyTarget = true;
      Serial.printf(" Target %d: X=%.2fm Y=%.2fm dist=%.2fm speed=%.2fm/s\n",
                    i + 1,
                    t.x / 1000.0, t.y / 1000.0,
                    t.distance / 1000.0, t.speed / 100.0);
    }

    if (!anyTarget) Serial.println(" (no targets in frame)");
  }

#endif
}
