// CAGE - Board #1: 3x HLK-LD2450 triangular array
//   -> serves a phone-friendly web radar scope over WiFi
//   -> ALSO keeps sending UDP, so the Python scope still works
//
// Open on your phone:  http://<board-ip>/     (IP is printed to Serial)
//                or:   http://cage.local/     (iOS/macOS; Android often needs the IP)
//
// Wiring (LD2450 TX -> ESP32 RX pin, LD2450 RX -> ESP32 TX pin):
//   Sensor A (   0 deg): TX -> GPIO 18, RX -> GPIO 17   (UART1)
//   Sensor B (+120 deg): TX -> GPIO 16, RX -> GPIO 15   (UART2)
//   Sensor C (-120 deg): TX -> GPIO 5,  RX -> GPIO 4    (UART0)
//   All: 5V shared, GND shared.
//
// IDE setting REQUIRED: Tools -> USB CDC On Boot -> Enabled
// Library: rbegamer/HLK-LD2450 WITH the local sign-magnitude patch.

#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <HardwareSerial.h>
#include <LD2450.h>

// ===== Network =====
// WiFi credentials live in secrets.h, which is gitignored.
// Copy secrets.example.h -> secrets.h and fill in your own values.
#include "secrets.h"

#define NUM_SENSORS 3
#define LD2450_MAX_RANGE_MM 6000

struct RadarUnit {
  const char* id;
  int mountAngleDeg;
  int rxPin, txPin;
  HardwareSerial serial;
  LD2450 radar;
};

RadarUnit units[NUM_SENSORS] = {
  { "A",    0, 18, 17, HardwareSerial(1), LD2450() },
  { "B",  120, 16, 15, HardwareSerial(2), LD2450() },
  { "C", -120,  5,  4, HardwareSerial(0), LD2450() },
};

WiFiUDP udp;
WebServer server(80);

// ---------- the web app (served from flash) ----------
static const char PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>CAGE Radar</title>
<style>
  :root { --g:#00ff41; --dim:#0a5c1e; --bg:#001a00; }
  * { box-sizing:border-box; -webkit-tap-highlight-color:transparent; }
  html,body { margin:0; height:100%; background:#000; color:var(--g);
    font-family:Helvetica,Arial,sans-serif; font-weight:bold; overflow:hidden; }
  #wrap { display:flex; flex-direction:column; height:100%; }
  header { padding:8px 12px 2px; font-size:14px; letter-spacing:.5px; }
  #scope { flex:1; display:block; width:100%; }
  footer { padding:6px 12px 10px; font-size:12px; opacity:.85;
    display:flex; gap:14px; flex-wrap:wrap; }
  .bad { color:#ff5555; }
</style></head><body>
<div id="wrap">
  <header>CAGE &middot; 360 SCOPE</header>
  <canvas id="scope"></canvas>
  <footer>
    <span id="stat">connecting...</span>
    <span id="cnt"></span>
    <span id="raw"></span>
  </footer>
</div>
<script>
const MAXR = 6.0;
// same filter constants as the Python scope
const CLUSTER = 0.60, MATCH = 0.90, SMOOTH = 0.35;
const MINHITS = 3, MAXMISS = 10, TRAIL = 8;

const cv = document.getElementById('scope');
const ctx = cv.getContext('2d');
let W = 0, H = 0, cx = 0, cy = 0, R = 0;

function resize() {
  const dpr = window.devicePixelRatio || 1;
  const r = cv.getBoundingClientRect();
  cv.width = r.width * dpr; cv.height = r.height * dpr;
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  W = r.width; H = r.height;
  cx = W / 2; cy = H / 2; R = Math.min(W, H) * 0.45;
}
window.addEventListener('resize', resize);

// polar (theta rad, 0=up, clockwise) + range m  ->  screen px
function px(th, rm) {
  const k = (rm / MAXR) * R;
  return [cx + k * Math.sin(th), cy - k * Math.cos(th)];
}

let tracks = [], rawCount = 0, ok = false;

function cluster(pts) {
  const g = [];
  for (const p of pts) {
    let hit = null;
    for (const c of g) {
      if (Math.hypot(p[0] - c.x, p[1] - c.y) <= CLUSTER) { hit = c; break; }
    }
    if (hit) {
      hit.pts.push(p);
      hit.x = hit.pts.reduce((s, q) => s + q[0], 0) / hit.pts.length;
      hit.y = hit.pts.reduce((s, q) => s + q[1], 0) / hit.pts.length;
    } else g.push({ x: p[0], y: p[1], pts: [p] });
  }
  return g.map(c => [c.x, c.y]);
}

function step(cs) {
  const left = cs.slice();
  for (const t of tracks) {
    let bi = -1, bd = MATCH;
    for (let i = 0; i < left.length; i++) {
      const d = Math.hypot(left[i][0] - t.x, left[i][1] - t.y);
      if (d < bd) { bd = d; bi = i; }
    }
    if (bi >= 0) {
      const [x, y] = left.splice(bi, 1)[0];
      t.x += SMOOTH * (x - t.x); t.y += SMOOTH * (y - t.y);
      t.hits++; t.miss = 0;
      t.tail.push([t.x, t.y]); if (t.tail.length > TRAIL) t.tail.shift();
    } else t.miss++;
  }
  for (const c of left) tracks.push({ x: c[0], y: c[1], hits: 1, miss: 0, tail: [[c[0], c[1]]] });
  tracks = tracks.filter(t => t.miss <= MAXMISS);
}

function draw() {
  ctx.clearRect(0, 0, W, H);

  // scope face
  ctx.beginPath(); ctx.arc(cx, cy, R, 0, 6.284);
  ctx.fillStyle = 'rgba(0,40,0,.55)'; ctx.fill();

  // range rings
  ctx.strokeStyle = '#0a5c1e'; ctx.lineWidth = 1;
  ctx.font = '10px Helvetica,Arial'; ctx.fillStyle = 'rgba(0,255,65,.65)';
  for (let m = 1; m <= MAXR; m++) {
    const k = (m / MAXR) * R;
    ctx.beginPath(); ctx.arc(cx, cy, k, 0, 6.284); ctx.stroke();
    ctx.fillText(m + 'm', cx + 3, cy - k + 11);
  }
  // spokes every 30 deg
  for (let d = 0; d < 360; d += 30) {
    const th = d * Math.PI / 180;
    const [x, y] = px(th, MAXR);
    ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(x, y); ctx.stroke();
  }
  // bearing labels
  ctx.fillStyle = 'rgba(0,255,65,.8)'; ctx.textAlign = 'center';
  for (let d = 0; d < 360; d += 90) {
    const th = d * Math.PI / 180;
    const [x, y] = px(th, MAXR * 1.1);
    ctx.fillText(d === 0 ? 'FWD' : d + '°',
                 Math.max(14, Math.min(W - 14, x)), Math.max(12, Math.min(H - 4, y)));
  }
  ctx.textAlign = 'left';

  // sensor at origin
  ctx.beginPath(); ctx.moveTo(cx, cy - 8); ctx.lineTo(cx - 7, cy + 6); ctx.lineTo(cx + 7, cy + 6);
  ctx.closePath(); ctx.fillStyle = '#00ff41'; ctx.fill();

  // blips + fading tails
  let shown = 0;
  for (const t of tracks) {
    if (t.hits < MINHITS) continue;
    shown++;
    for (let i = 0; i < t.tail.length; i++) {
      const [x, y] = t.tail[i];
      const a = 0.15 + 0.85 * ((i + 1) / t.tail.length);
      const rm = Math.hypot(x, y), th = Math.atan2(x, y);
      if (rm > MAXR) continue;
      const [sx, sy] = px(th, rm);
      ctx.beginPath();
      ctx.arc(sx, sy, i === t.tail.length - 1 ? 7 : 4, 0, 6.284);
      ctx.fillStyle = 'rgba(0,255,65,' + a.toFixed(2) + ')';
      ctx.fill();
    }
  }

  document.getElementById('cnt').textContent = 'targets: ' + shown;
  document.getElementById('raw').textContent = 'raw: ' + rawCount;
  const st = document.getElementById('stat');
  st.textContent = ok ? 'live' : 'no data';
  st.className = ok ? '' : 'bad';
}

async function tick() {
  try {
    const r = await fetch('/data', { cache: 'no-store' });
    const j = await r.json();
    rawCount = j.t.length;
    step(cluster(j.t));
    ok = true;
  } catch (e) { ok = false; }
  draw();
}

resize();
draw();
setInterval(tick, 100);
</script></body></html>
)rawliteral";

// ---------- LD2450 config ----------
void ld2450EnableMultiTarget(HardwareSerial &s) {
  uint8_t enterConfig[] = {0xFD,0xFC,0xFB,0xFA, 0x04,0x00, 0xFF,0x00, 0x01,0x00, 0x04,0x03,0x02,0x01};
  uint8_t multiTarget[] = {0xFD,0xFC,0xFB,0xFA, 0x02,0x00, 0x90,0x00, 0x04,0x03,0x02,0x01};
  uint8_t exitConfig[]  = {0xFD,0xFC,0xFB,0xFA, 0x02,0x00, 0xFE,0x00, 0x04,0x03,0x02,0x01};
  s.write(enterConfig, sizeof(enterConfig)); s.flush(); delay(100);
  s.write(multiTarget, sizeof(multiTarget)); s.flush(); delay(100);
  s.write(exitConfig,  sizeof(exitConfig));  s.flush(); delay(100);
  while (s.available()) s.read();
}

void handleRoot() {
  server.sendHeader("Cache-Control", "no-store");
  server.send_P(200, "text/html", PAGE);
}

// JSON of every current target, already rotated into the airframe frame.
// X = right (m), Y = forward (m).  {"t":[[x,y],...]}
void handleData() {
  String out = "{\"t\":[";
  bool first = true;
  for (int i = 0; i < NUM_SENSORS; i++) {
    float mount = units[i].mountAngleDeg * PI / 180.0f;
    for (int t = 0; t < 3; t++) {
      LD2450::RadarTarget tg = units[i].radar.getTarget(t);
      if (tg.distance == 0 || tg.distance > LD2450_MAX_RANGE_MM) continue;
      float th = mount + atan2f((float)tg.x, (float)tg.y);
      float r  = tg.distance / 1000.0f;
      if (!first) out += ",";
      out += "[" + String(r * sinf(th), 3) + "," + String(r * cosf(th), 3) + "]";
      first = false;
    }
  }
  out += "]}";
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", out);
}

void setup() {
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);
  delay(200);
  Serial.println("CAGE Board #1 - 3x LD2450 + web scope");

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
  while (WiFi.status() != WL_CONNECTED) { delay(400); Serial.print("."); }

  if (MDNS.begin("cage")) MDNS.addService("http", "tcp", 80);

  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.begin();

  Serial.printf("\n\n  OPEN ON YOUR PHONE:  http://%s/\n",
                WiFi.localIP().toString().c_str());
  Serial.printf("  or (iOS/macOS):      http://cage.local/\n\n");
}

void loop() {
  static unsigned long lastSend = 0;

  for (int i = 0; i < NUM_SENSORS; i++) units[i].radar.read();
  server.handleClient();

  if (millis() - lastSend < 50) return;
  lastSend = millis();

  // keep the UDP stream alive so the Python scope still works
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
}
