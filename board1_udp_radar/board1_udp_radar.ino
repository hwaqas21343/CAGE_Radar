// CAGE Phase 2->3 - Board #1, single LD2450 over WiFi/UDP to laptop
// Wire: LD2450 TX -> GPIO 18, LD2450 RX -> GPIO 17, 5V -> 5V, GND -> GND
// Requires the sign-magnitude patch in LD2450.cpp (see project notes).
//
// Sends one UDP packet per radar frame. Packet is plain text:
//   "x,y,dist,speed;x,y,dist,speed;..."   (mm and cm/s), or "-" if no targets.

#include <WiFi.h>
#include <WiFiUdp.h>
#include <HardwareSerial.h>
#include <LD2450.h>

// ===== Network =====
// WiFi credentials live in secrets.h, which is gitignored.
// Copy secrets.example.h -> secrets.h and fill in your own values.
#include "secrets.h"

#define LD2450_RX 18
#define LD2450_TX 17
#define LD2450_MAX_RANGE_MM 6000

HardwareSerial ld2450Serial(1);   // UART1
LD2450 radar;
WiFiUDP udp;

// Switch LD2450 from default single-target into multi-target tracking (up to 3).
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
  delay(200);

  ld2450Serial.begin(256000, SERIAL_8N1, LD2450_RX, LD2450_TX);
  delay(100);
  ld2450EnableMultiTarget(ld2450Serial);
  radar.begin(ld2450Serial);

  // Connect WiFi
  Serial.printf("Connecting to %s", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(400);
    Serial.print(".");
  }
  Serial.printf("\nConnected. ESP32 IP: %s  ->  sending to %s:%u\n",
                WiFi.localIP().toString().c_str(), LAPTOP_IP, UDP_PORT);
}

void loop() {
  static unsigned long lastSend = 0;

  radar.read();   // keep buffer drained / data fresh every loop

  // Send ~20 packets/sec
  if (millis() - lastSend < 50) return;
  lastSend = millis();

  String payload = "";
  int count = 0;
  for (int i = 0; i < 3; i++) {
    LD2450::RadarTarget t = radar.getTarget(i);
    if (t.distance == 0 || t.distance > LD2450_MAX_RANGE_MM) continue;
    if (count > 0) payload += ";";
    payload += String(t.x) + "," + String(t.y) + "," + String(t.distance) + "," + String(t.speed);
    count++;
  }
  if (count == 0) payload = "-";   // explicit "no targets" so the plot can clear

  udp.beginPacket(LAPTOP_IP, UDP_PORT);
  udp.print(payload);
  udp.endPacket();

  // DEBUG: show WiFi state + what we're sending, twice a second
  static unsigned long lastDbg = 0;
  if (millis() - lastDbg >= 500) {
    lastDbg = millis();
    Serial.printf("WiFi=%d  -> %s:%u  payload=%s\n",
                  WiFi.status(), LAPTOP_IP, UDP_PORT, payload.c_str());
  }
}
