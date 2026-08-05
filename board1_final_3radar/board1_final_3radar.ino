// CAGE - Board #1 FINAL: 3x HLK-LD2450 in triangular layout -> WiFi/UDP -> laptop
//
// Layout: sensors mounted 120 degrees apart (equilateral triangle).
// Each LD2450 has +/-60 deg FOV, so 3 x 120 deg = full 360 coverage.
//
// Wiring (LD2450 TX -> ESP32 RX pin, LD2450 RX -> ESP32 TX pin):
//   Sensor A (  0 deg): TX -> GPIO 18, RX -> GPIO 17   (UART1)
//   Sensor B (+120 deg): TX -> GPIO 16, RX -> GPIO 15   (UART2)
//   Sensor C (-120 deg): TX -> GPIO 5,  RX -> GPIO 4    (UART0)
//   All: 5V -> 5V (shared), GND -> any GND pin.
//
// IDE setting REQUIRED: Tools -> USB CDC On Boot -> Enabled
//   (frees UART0 for sensor C; Serial Monitor runs over native USB)
//
// Library: rbegamer/HLK-LD2450 WITH the local sign-magnitude patch in LD2450.cpp.
//   If the library is ever reinstalled, re-apply the patch (see project notes).
//
// UDP packet, one per sensor per cycle (plain text):
//   "<sensorId>,<mountAngleDeg>:<x>,<y>,<dist>,<speed>;..."   mm and cm/s
//   e.g.  "A,0:252,335,419,0;-1101,1116,1568,-12"
//   or    "B,120:-"        when that sensor sees nothing

#include <WiFi.h>
#include <WiFiUdp.h>
#include <HardwareSerial.h>
#include <LD2450.h>

// ===== Network =====
// WiFi credentials live in secrets.h, which is gitignored.
// Copy secrets.example.h -> secrets.h and fill in your own values.
#include "secrets.h"

// ===== Sensors =====
#define NUM_SENSORS 3
#define LD2450_MAX_RANGE_MM 6000

struct RadarUnit {
  const char* id;        // tag sent in every packet
  int mountAngleDeg;     // mounting angle on the airframe (0 = forward)
  int rxPin, txPin;      // ESP32 pins (rx = sensor's TX arrives here)
  HardwareSerial serial;
  LD2450 radar;
};

RadarUnit units[NUM_SENSORS] = {
  { "A",    0, 18, 17, HardwareSerial(1), LD2450() },
  { "B",  120, 16, 15, HardwareSerial(2), LD2450() },
  { "C", -120,  5,  4, HardwareSerial(0), LD2450() },
};

WiFiUDP udp;

// LD2450 ships in single-target mode; switch to multi-target (persists in sensor flash).
void ld2450EnableMultiTarget(HardwareSerial &s) {
  uint8_t enterConfig[] = {0xFD,0xFC,0xFB,0xFA, 0x04,0x00, 0xFF,0x00, 0x01,0x00, 0x04,0x03,0x02,0x01};
  uint8_t multiTarget[] = {0xFD,0xFC,0xFB,0xFA, 0x02,0x00, 0x90,0x00, 0x04,0x03,0x02,0x01};
  uint8_t exitConfig[]  = {0xFD,0xFC,0xFB,0xFA, 0x02,0x00, 0xFE,0x00, 0x04,0x03,0x02,0x01};
  s.write(enterConfig, sizeof(enterConfig)); s.flush(); delay(100);
  s.write(multiTarget, sizeof(multiTarget)); s.flush(); delay(100);
  s.write(exitConfig,  sizeof(exitConfig));  s.flush(); delay(100);
  while (s.available()) s.read();
}

void setup() {
  Serial.begin(115200);
  // With USB CDC, give the host a moment to open the port or the first
  // prints are lost. Timeout so the board still runs headless.
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);
  delay(200);
  Serial.println("CAGE Board #1 - 3x LD2450 triangular array");

  for (int i = 0; i < NUM_SENSORS; i++) {
    units[i].serial.begin(256000, SERIAL_8N1, units[i].rxPin, units[i].txPin);
    delay(100);
    ld2450EnableMultiTarget(units[i].serial);
    units[i].radar.begin(units[i].serial);
    Serial.printf("  sensor %s @ %+d deg on RX=%d TX=%d\n",
                  units[i].id, units[i].mountAngleDeg, units[i].rxPin, units[i].txPin);
  }

  Serial.printf("Connecting to %s", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(400);
    Serial.print(".");
  }
  Serial.printf("\nConnected. ESP32 IP: %s -> %s:%u\n",
                WiFi.localIP().toString().c_str(), LAPTOP_IP, UDP_PORT);
}

void loop() {
  static unsigned long lastSend = 0;
  static unsigned long lastDbg = 0;

  // Keep every sensor's buffer drained each pass
  for (int i = 0; i < NUM_SENSORS; i++) units[i].radar.read();

  if (millis() - lastSend < 50) return;   // ~20 packets/s per sensor
  lastSend = millis();

  for (int i = 0; i < NUM_SENSORS; i++) {
    String payload = String(units[i].id) + "," + String(units[i].mountAngleDeg) + ":";
    int count = 0;
    for (int t = 0; t < 3; t++) {
      LD2450::RadarTarget tg = units[i].radar.getTarget(t);
      if (tg.distance == 0 || tg.distance > LD2450_MAX_RANGE_MM) continue;
      if (count > 0) payload += ";";
      payload += String(tg.x) + "," + String(tg.y) + "," + String(tg.distance) + "," + String(tg.speed);
      count++;
    }
    if (count == 0) payload += "-";

    udp.beginPacket(LAPTOP_IP, UDP_PORT);
    udp.print(payload);
    udp.endPacket();
  }

  // Debug heartbeat twice a second
  if (millis() - lastDbg >= 500) {
    lastDbg = millis();
    Serial.printf("WiFi=%d  sending 3 sensor packets to %s:%u\n",
                  WiFi.status(), LAPTOP_IP, UDP_PORT);
  }
}
