/*
 * RD-03D Radar — XIAO ESP32-S3 (Single-Sensor, horizontal vor dem Display)
 * Liest Radar-Frames, verschmilzt Targets < 1m (Clustering) und sendet
 * per UDP an den Giga R1 (der den Hotspot hostet).
 *
 * PINBELEGUNG:
 *   Radar TX → D0 (GPIO1)  = UART1 RX
 *   Radar RX → D1 (GPIO2)  = UART1 TX
 *   Radar VCC → 5V
 *   Radar GND → GND
 *
 * HINWEIS: D6/D7 (GPIO43/44) NICHT verwenden — das ist USB-Serial (UART0)
 *
 * NETZWERK:
 *   XIAO verbindet sich als Client mit dem Hotspot des Giga R1
 *   SSID:    "RadarNet"
 *   Passwort: "radar12345"
 *   UDP Port: 4210
 *   Giga-IP wird automatisch aus DHCP-Gateway geholt
 *
 * Einschaltreihenfolge beliebig: Ohne Hotspot läuft das Radar weiter;
 * der XIAO versucht die WLAN-Verbindung alle 10 Sekunden erneut.
 */

#include <WiFi.h>
#include <WiFiUdp.h>

// ── Pins ──
#define RADAR_RX_PIN 1   // D0 am XIAO S3
#define RADAR_TX_PIN 2   // D1 am XIAO S3

// ── Netzwerk ──
const char*    SSID     = "RadarNet";
const char*    PASSWORD = "radar12345";
const uint16_t UDP_PORT = 4210;
IPAddress      gigaIP;

// ── Radar Protokoll ──
static const uint8_t CMD_ENABLE[14] = {0xFD,0xFC,0xFB,0xFA,0x04,0x00,0xFF,0x00,0x01,0x00,0x04,0x03,0x02,0x01};
static const uint8_t CMD_MULTI[12]  = {0xFD,0xFC,0xFB,0xFA,0x02,0x00,0x90,0x00,0x04,0x03,0x02,0x01};
static const uint8_t CMD_END[12]    = {0xFD,0xFC,0xFB,0xFA,0x02,0x00,0xFE,0x00,0x04,0x03,0x02,0x01};
static const uint8_t HDR[4]         = {0xAA,0xFF,0x03,0x00};

#define MAX_TARGETS 3

struct Target {
  int16_t x;
  int16_t y;
  int16_t spd;
  uint8_t valid;
};

struct __attribute__((packed)) UdpPacket {
  uint32_t magic;
  uint32_t frameCount;
  Target   targets[MAX_TARGETS];
};

HardwareSerial RadarSerial(1);
WiFiUDP        udp;
bool           wifiReady = false;
UdpPacket      pkt;
uint32_t       frameCount = 0;
uint32_t       badCount   = 0;
uint32_t       byteCount  = 0;  // Debug: Rohdaten-Zähler

// ── Dekodierung (offizielles Datenblatt) ──
int16_t decodeVal(uint8_t lo, uint8_t hi) {
  uint16_t raw = (uint16_t)lo | ((uint16_t)hi << 8);
  if (raw >= 0x8000) return  (int16_t)(raw - 0x8000);
  else               return -(int16_t)(raw);
}

// ── Clustering: 2 Targets desselben Sensors, die näher als 1m ──
// beieinander liegen (z.B. Oberkörper + Armbewegung derselben Person),
// zu einem verschmelzen statt als 2 separate Punkte anzuzeigen.
const float CLUSTER_DIST_MM = 1000.0f;

void clusterTargets() {
  for (int i = 0; i < MAX_TARGETS; i++) {
    if (!pkt.targets[i].valid) continue;
    for (int j = i + 1; j < MAX_TARGETS; j++) {
      if (!pkt.targets[j].valid) continue;
      float dx = (float)pkt.targets[i].x - (float)pkt.targets[j].x;
      float dy = (float)pkt.targets[i].y - (float)pkt.targets[j].y;
      float dist = sqrtf(dx*dx + dy*dy);
      if (dist < CLUSTER_DIST_MM) {
        pkt.targets[i].x   = (int16_t)(((float)pkt.targets[i].x + (float)pkt.targets[j].x) / 2.0f);
        pkt.targets[i].y   = (int16_t)(((float)pkt.targets[i].y + (float)pkt.targets[j].y) / 2.0f);
        // Geschwindigkeit mit dem größeren Betrag behalten (Vorzeichen = Richtung).
        // max() auf den vorzeichenbehafteten Werten wählte bei zwei sich nähernden
        // Echos (z.B. -50 und -10 cm/s) die langsamere Komponente.
        if (abs(pkt.targets[j].spd) > abs(pkt.targets[i].spd)) pkt.targets[i].spd = pkt.targets[j].spd;
        pkt.targets[j] = {0, 0, 0, 0};
      }
    }
  }
}

// ── Frame parsen + per UDP senden ──
void parseAndSend(const uint8_t *pl) {
  for (int i = 0; i < MAX_TARGETS; i++) {
    const uint8_t *b = pl + i * 8;
    bool empty = true;
    for (int j = 0; j < 8; j++) if (b[j]) { empty = false; break; }
    if (empty) { pkt.targets[i] = {0,0,0,0}; continue; }
    int16_t xi  = decodeVal(b[0], b[1]);
    int16_t yi  = decodeVal(b[2], b[3]);
    int16_t spi = decodeVal(b[4], b[5]);
    float dist  = sqrtf((float)xi*xi + (float)yi*yi);
    if (dist > 100.0f && dist <= 8000.0f) {
      pkt.targets[i] = {xi, yi, spi, 1};
    } else {
      pkt.targets[i] = {0,0,0,0};
    }
  }

  clusterTargets();

  pkt.frameCount = ++frameCount;
  if (!wifiReady) return;   // noch kein Netz: Radar trotzdem weiterlesen, nur nicht senden
  int r1 = udp.beginPacket(gigaIP, UDP_PORT);
  udp.write((uint8_t*)&pkt, sizeof(pkt));
  int r2 = udp.endPacket();

  static uint32_t lastPrint = 0;
  if (millis() - lastPrint > 2000) {
    lastPrint = millis();
    Serial.print("UDP sent fr="); Serial.print(frameCount);
    Serial.print(" begin="); Serial.print(r1);
    Serial.print(" end="); Serial.println(r2);
  }
}

// ── Frame-Reader ──
uint8_t pl[26];
uint8_t plIdx = 0, hdrIdx = 0;
bool    inFrame = false;
uint8_t cmdStep = 0;   // Multi-Target-Kommandokette aktiv? (siehe unten)

void readRadar() {
  if (cmdStep) {   // während der Konfiguration kommen Antwortframes, keine Messwerte
    while (RadarSerial.available()) RadarSerial.read();
    return;
  }
  while (RadarSerial.available()) {
    uint8_t c = RadarSerial.read();
    byteCount++;
    if (!inFrame) {
      if (c == HDR[hdrIdx]) {
        hdrIdx++;
        if (hdrIdx == 4) { hdrIdx = 0; inFrame = true; plIdx = 0; }
      } else {
        hdrIdx = (c == HDR[0]) ? 1 : 0;
      }
    } else {
      pl[plIdx++] = c;
      if (plIdx == 26) {
        inFrame = false; plIdx = 0;
        if (pl[24] == 0x55 && pl[25] == 0xCC) {
          parseAndSend(pl);
        } else {
          badCount++;
        }
      }
    }
  }
}

// ── Multi-Target CMD (nicht blockierend) ──
// Schrittkette mit 200 ms Abstand: loop() läuft während der Konfiguration weiter.
// readRadar() verwirft dabei UART-Daten; Messwerte pausieren weiterhin ca. 600 ms.
// Nach CMD_END wird der Parser neu synchronisiert.
uint32_t cmdStepMs = 0;

void startMultiTargetCmd() {
  if (cmdStep) return;
  cmdStep   = 1;
  cmdStepMs = millis() - 200;   // erster Schritt sofort fällig
}

void runMultiTargetCmd() {
  if (cmdStep == 0 || millis() - cmdStepMs < 200) return;
  while (RadarSerial.available()) RadarSerial.read();   // Antworten des Moduls verwerfen
  switch (cmdStep) {
    case 1: RadarSerial.write(CMD_ENABLE, sizeof(CMD_ENABLE)); break;
    case 2: RadarSerial.write(CMD_MULTI,  sizeof(CMD_MULTI));  break;
    case 3: RadarSerial.write(CMD_END,    sizeof(CMD_END));    break;
    default:
      inFrame = false; plIdx = 0; hdrIdx = 0;   // Parser neu synchronisieren
      Serial.println("Multi-Target CMD gesendet.");
      cmdStep = 0;
      return;
  }
  cmdStepMs = millis();
  cmdStep++;
}

// ── WiFi verbinden (nicht blockierend) ──
// Vorher blockierte connectWiFi() bis zu 20 s und rief danach ESP.restart()
// auf: ohne erreichbaren Giga lief der XIAO in einer Neustart-Schleife und las
// währenddessen kein Radar. Jetzt verbindet er im Hintergrund, versucht es alle
// 10 s erneut. Außerhalb der Radar-Konfiguration liest readRadar() weiter.
bool     udpStarted       = false;
uint32_t lastConnectTryMs = 0;

void connectWiFi() {
  Serial.print("Verbinde mit "); Serial.println(SSID);
  WiFi.disconnect();
  WiFi.begin(SSID, PASSWORD);
  lastConnectTryMs = millis();
}

void checkWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!wifiReady) {
      wifiReady = true;
      gigaIP = WiFi.gatewayIP();
      if (!udpStarted) { udp.begin(UDP_PORT); udpStarted = true; }
      Serial.print("Verbunden. IP: "); Serial.print(WiFi.localIP());
      Serial.print("  Giga IP: "); Serial.println(gigaIP);
    }
    return;
  }
  if (wifiReady) { wifiReady = false; Serial.println("WiFi verloren — reconnect..."); }
  if (millis() - lastConnectTryMs > 10000) connectWiFi();
}

// ── Debug-Timer ──
uint32_t lastDebugMs = 0;
uint32_t lastCmdMs   = 0;
uint32_t lastCheckMs = 0;

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("XIAO ESP32-S3 — RD-03D Sender");
  Serial.print("UART1 RX=GPIO"); Serial.print(RADAR_RX_PIN);
  Serial.print(" TX=GPIO"); Serial.println(RADAR_TX_PIN);

  pkt.magic = 0xD03DA7A;

  RadarSerial.begin(256000, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);
  delay(300);

  WiFi.mode(WIFI_STA);
  connectWiFi();            // kehrt sofort zurück; Verbindung wird in loop() geprüft
  startMultiTargetCmd();    // läuft in loop() schrittweise ab
  lastCmdMs   = millis();
  lastDebugMs = millis();

  Serial.println("Bereit — Radar wird gelesen, WiFi verbindet im Hintergrund.");
}

void loop() {
  // WiFi-Watchdog (nicht blockierend)
  if (millis() - lastCheckMs > 1000) {
    lastCheckMs = millis();
    checkWiFi();
  }

  // Multi-Target CMD alle 60s wiederholen
  if (millis() - lastCmdMs > 60000) {
    lastCmdMs = millis();
    startMultiTargetCmd();
  }
  runMultiTargetCmd();

  // Debug: Rohdaten-Zähler alle 2s ausgeben
  if (millis() - lastDebugMs > 2000) {
    lastDebugMs = millis();
    Serial.print("UART bytes="); Serial.print(byteCount);
    Serial.print(" frames="); Serial.print(frameCount);
    Serial.print(" bad="); Serial.println(badCount);
  }

  readRadar();
}
