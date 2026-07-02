/*
 * RD-03D Radar — XIAO ESP32-S3
 * Liest Radar-Frames und sendet Target-Daten per UDP an Giga R1
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
 *   Verbindet sich mit Hotspot des Giga R1
 *   SSID:    "RadarNet"
 *   Passwort: "radar12345"
 *   UDP Port: 4210
 *   Giga-IP wird automatisch aus DHCP-Gateway geholt
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
  pkt.frameCount = ++frameCount;
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

void readRadar() {
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

// ── Multi-Target CMD ──
void sendMultiTargetCmd() {
  while (RadarSerial.available()) RadarSerial.read();
  delay(50);
  RadarSerial.write(CMD_ENABLE, sizeof(CMD_ENABLE));
  RadarSerial.flush(); delay(200);
  while (RadarSerial.available()) RadarSerial.read();
  RadarSerial.write(CMD_MULTI, sizeof(CMD_MULTI));
  RadarSerial.flush(); delay(200);
  while (RadarSerial.available()) RadarSerial.read();
  RadarSerial.write(CMD_END, sizeof(CMD_END));
  RadarSerial.flush(); delay(200);
  while (RadarSerial.available()) RadarSerial.read();
  Serial.println("Multi-Target CMD gesendet.");
}

// ── WiFi verbinden ──
void connectWiFi() {
  WiFi.begin(SSID, PASSWORD);
  Serial.print("Verbinde mit "); Serial.print(SSID);
  uint8_t tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 40) {
    delay(500); Serial.print('.'); tries++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    gigaIP = WiFi.gatewayIP();
    Serial.print("\nVerbunden. IP: "); Serial.print(WiFi.localIP());
    Serial.print("  Giga IP: "); Serial.println(gigaIP);
  } else {
    Serial.println("\nFehlgeschlagen — Neustart.");
    delay(1000);
    ESP.restart();
  }
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

  connectWiFi();
  udp.begin(UDP_PORT);

  sendMultiTargetCmd();
  lastCmdMs   = millis();
  lastDebugMs = millis();

  // RX-Buffer nach CMD leeren + Parser-State reset
  delay(500);
  while (RadarSerial.available()) RadarSerial.read();
  inFrame = false; plIdx = 0; hdrIdx = 0;

  Serial.print("Bereit — UDP an "); Serial.println(gigaIP);
}

void loop() {
  // WiFi-Watchdog
  if (millis() - lastCheckMs > 5000) {
    lastCheckMs = millis();
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi verloren — reconnect...");
      connectWiFi();
    }
  }

  // Multi-Target CMD alle 60s wiederholen
  if (millis() - lastCmdMs > 60000) {
    lastCmdMs = millis();
    sendMultiTargetCmd();
  }

  // Debug: Rohdaten-Zähler alle 2s ausgeben
  if (millis() - lastDebugMs > 2000) {
    lastDebugMs = millis();
    Serial.print("UART bytes="); Serial.print(byteCount);
    Serial.print(" frames="); Serial.print(frameCount);
    Serial.print(" bad="); Serial.println(badCount);
  }

  readRadar();
}
