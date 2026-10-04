/*
 * RD-03D Radar Display — Arduino Giga R1
 * Empfängt UDP-Pakete vom Single-Sensor-Sender und zeigt bis zu 3 Targets
 * auf dem Display (ein RD-03D, 120° Sichtfeld).
 *
 * NETZWERK:
 *   Giga R1 ist Hotspot: SSID "RadarNet", PW "radar12345"
 *   Eigene IP: 192.168.3.1 (Standard des Arduino-Mbed-Cores; der XIAO nutzt die Gateway-IP)
 *   UDP Port: 4210
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include "Arduino_GigaDisplay_GFX.h"
#include "Arduino_GigaDisplayTouch.h"

// ── Netzwerk ──
const char*    AP_SSID = "RadarNet";
const char*    AP_PASS = "radar12345";
const uint16_t UDP_PORT = 4210;

// ── Display ──
#define SCREEN_W    800
#define SCREEN_H    480
#define RADAR_CX    400
#define RADAR_CY    480
#define RADAR_R     440
#define MAX_DIST_MM 8000.0f
#define SECTOR_HALF 60.0f
#define MAX_TARGETS 3
#define MOVE_THRESH_MM 15.0f

// ── Ein Sensor, 120° FOV, zentriert auf die Gerätefront (0°) ──
#define NUM_SECTORS 1
const float SECTOR_CENTERS[NUM_SECTORS] = {0.0f};

// ── Buzzer (Tracker-Sound) ──
#define BUZZER_PIN     9      // passiver Piezo-Buzzer, KEIN aktiver Buzzer!

// Jeder Ping besteht aus 2 Stufen: kurzer heller "Klick" (Attack) direkt
// gefolgt vom eigentlichen Ton — kommt dem hellen, sonarartigen Charakter
// des Vorbilds näher als ein einzelner flacher Ton.
// Jeder Ping ist ein kurzer 3-stufiger AUFWÄRTS-Sweep (tief -> mittel ->
// Zielfrequenz), statt eines einzelnen flachen Tons — klingt nach einem
// ansteigenden Sonar-Ping statt einem Klick.
const uint16_t PING_STAGE1_MS = 26;
const uint16_t PING_STAGE2_MS = 24;
const uint16_t PING_STAGE3_MS = 30;
#define BEEP_DUR_MS  (PING_STAGE1_MS + PING_STAGE2_MS + PING_STAGE3_MS)   // Gesamtlänge eines Pings

const float    BEEP_DIST_MIN      = 300.0f;   // mm — ab hier schnellster/höchster Ping
const float    BEEP_DIST_MAX      = 8000.0f;  // mm — ab hier langsamster/tiefster Ping
const uint16_t BEEP_INTERVAL_MIN  = 120;      // ms Pause zwischen Pings bei minimaler Distanz
const uint16_t BEEP_INTERVAL_MAX  = 900;      // ms Pause zwischen Pings bei maximaler Distanz
const uint16_t BEEP_FREQ_MIN      = 700;      // Hz — Tonhöhe weit weg (Kompromiss: lauter, aber unterhalb des schrillen Bereichs)
const uint16_t BEEP_FREQ_MAX      = 1400;     // Hz — Tonhöhe nah dran

// ── Mute-Button (Touch) ──
#define BTN_X 660
#define BTN_Y 400
#define BTN_W 130
#define BTN_H 40
bool buzzerMuted = false;

// ── Range-HUD-Position (Basis der Radar-Kuppel, überdeckt Ursprung) ──
#define RANGE_HUD_W 140
#define RANGE_HUD_H 40
#define RANGE_HUD_X (RADAR_CX - RANGE_HUD_W/2)
#define RANGE_HUD_Y (RADAR_CY - RANGE_HUD_H)

// ── Farben ──
#define C_BG    0x0000
#define C_GREEN 0x07E0
#define C_GDIM  0x02E0
#define C_RED   0xF800
#define C_AMBER 0xFD20
#define C_CYAN  0x07FF
#define C_WHITE 0xFFFF

GigaDisplay_GFX          display;
Arduino_GigaDisplayTouch touch;
WiFiUDP                  udp;

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

struct DisplayTarget {
  float x, y, spd;
  bool  valid;
};

struct DotState {
  int16_t cx, cy, bx, by, bw, bh;
  bool active;
};

DisplayTarget targets[MAX_TARGETS];
DisplayTarget lastDrawn[MAX_TARGETS];
DotState      dots[MAX_TARGETS];

// ── Ghost-Filter: Ziel, das sich 2s lang nicht (spürbar) bewegt, gilt als ──
// Störung/Geisterziel (der RD-03D erkennt stehende Personen ohnehin nicht
// zuverlässig — ein "Ziel", das sich gar nicht bewegt, ist verdächtig) und
// wird ausgeblendet, bis es sich wieder über MOVE_THRESH_MM hinaus bewegt.
DisplayTarget prevRaw[MAX_TARGETS];       // Ground-Truth letzte Rohposition
uint32_t      stillSinceMs[MAX_TARGETS];  // Zeitpunkt der letzten echten Bewegung
const uint32_t STILL_TIMEOUT_MS = 2000;

uint32_t frameCount = 0;
uint32_t lastPacketMs = 0;
bool     txConnected = false;

// Buzzer-Status
uint32_t lastBeepMs   = 0;
bool     beepActive   = false;
uint8_t  chirpStage      = 0;   // 0=idle, 1/2/3=Sweep-Stufen (tief->mittel->Ziel)
uint32_t chirpStageStart = 0;
uint16_t chirpMainFreq   = 0;

// Hintergrund-Puls: läuft konstant, solange das Gerät an ist — unabhängig
// davon, ob ein Ziel erkannt wird. Signalisiert "Radar läuft/scannt".
const uint16_t PULSE_FREQ        = 300;   // Hz, Original-Wert
const uint16_t PULSE_DUR_MS      = 40;    // kurz, damit es nicht aufdringlich wird
const uint32_t PULSE_INTERVAL_MS = 1800;  // Pause zwischen zwei Pulsen
uint32_t lastPulseMs  = 0;
bool     pulseActive  = false;
uint32_t pulseStartMs = 0;

// ── UDP empfangen ──
// Wartende Datagramme abholen und das letzte gültige Paket anzeigen.
void receiveUdp() {
  static uint32_t lastDebug = 0;
  static uint32_t rxCount   = 0;
  UdpPacket pkt = {};
  bool gotPacket = false;
  int sz;
  while ((sz = udp.parsePacket()) > 0) {
    if (sz < (int)sizeof(UdpPacket)) continue;
    UdpPacket tmp = {};
    if (udp.read((uint8_t*)&tmp, sizeof(tmp)) != (int)sizeof(tmp)) continue;
    if (tmp.magic != 0xD03DA7A) continue;
    pkt = tmp;
    gotPacket = true;
    rxCount++;
  }
  if (millis() - lastDebug > 2000) {
    lastDebug = millis();
    Serial.print("UDP Pakete gesamt="); Serial.println(rxCount);
  }
  if (!gotPacket) return;

  frameCount = pkt.frameCount;
  lastPacketMs = millis();
  txConnected = true;

  for (int i = 0; i < MAX_TARGETS; i++) {
    if (pkt.targets[i].valid) {
      targets[i].x     = (float)pkt.targets[i].x;
      targets[i].y     = (float)pkt.targets[i].y;
      targets[i].spd   = (float)abs(pkt.targets[i].spd);
      targets[i].valid = true;
    } else {
      targets[i].valid = false;
    }
  }
}

// ── Maßstab ──
float distToR(float d) {
  if (d <= 0)       return 0;
  if (d <= 2000.0f) return (d/2000.0f)                          * 0.50f * RADAR_R;
  if (d <= 4000.0f) return (0.50f+(d-2000.0f)/2000.0f*0.25f)   * RADAR_R;
  if (d <= 6000.0f) return (0.75f+(d-4000.0f)/2000.0f*0.15f)   * RADAR_R;
  return              (0.90f+(d-6000.0f)/2000.0f*0.10f)         * RADAR_R;
}

void targetToPixel(float x, float y, int16_t &px, int16_t &py) {
  float dist  = sqrtf(x*x + y*y);
  float angle = atan2f(x, y) * 180.0f / PI;
  float r     = distToR(dist);
  float rad   = (angle - 90.0f) * PI / 180.0f;
  px = RADAR_CX + (int16_t)(r * cosf(rad));
  py = RADAR_CY + (int16_t)(r * sinf(rad));
}

// ── Hintergrund-Region neu zeichnen ──
void redrawBg(int16_t rx1, int16_t ry1, int16_t rw, int16_t rh) {
  int16_t rx2 = rx1+rw-1, ry2 = ry1+rh-1;
  if (rx1<0) rx1=0; if (ry1<0) ry1=0;
  if (rx2>=SCREEN_W) rx2=SCREEN_W-1;
  if (ry2>=SCREEN_H) ry2=SCREEN_H-1;
  if (rx1>rx2||ry1>ry2) return;
  display.fillRect(rx1,ry1,rx2-rx1+1,ry2-ry1+1,C_BG);
  for (int s=0; s<NUM_SECTORS; s++) {
    float c = SECTOR_CENTERS[s];
    float rd[]={1000,2000,3000,4000,5000,6000,7000,8000};
    uint16_t rc[]={C_GREEN,0x00C0,0x00C0,C_GDIM,0x00C0,0x00C0,0x00C0,C_GREEN};
    for (int ri=0; ri<8; ri++) {
      float r=distToR(rd[ri]);
      for (float a=c-SECTOR_HALF; a<=c+SECTOR_HALF; a+=0.5f) {
        float rad=(a-90.0f)*PI/180.0f;
        int16_t px=RADAR_CX+(int16_t)(r*cosf(rad)), py=RADAR_CY+(int16_t)(r*sinf(rad));
        if (px>=rx1&&px<=rx2&&py>=ry1&&py<=ry2) display.drawPixel(px,py,rc[ri]);
      }
    }
    int degs[]={-60,-30,0,30,60};
    for (int i=0; i<5; i++) {
      float rad=(c+degs[i]-90.0f)*PI/180.0f;
      for (float r=0; r<=RADAR_R; r+=1.5f) {
        int16_t px=RADAR_CX+(int16_t)(r*cosf(rad)), py=RADAR_CY+(int16_t)(r*sinf(rad));
        if (px>=rx1&&px<=rx2&&py>=ry1&&py<=ry2) display.drawPixel(px,py,degs[i]==0?C_GDIM:0x0200);
      }
    }
  }
  if (RADAR_CX>=rx1&&RADAR_CX<=rx2&&RADAR_CY>=ry1&&RADAR_CY<=ry2) {
    display.fillCircle(RADAR_CX,RADAR_CY,5,C_GREEN);
    display.drawCircle(RADAR_CX,RADAR_CY,9,C_GDIM);
  }
}

// ── Aktive Dots neu zeichnen, die von einer Region betroffen sein könnten ──
// (z.B. nachdem redrawBg() über sie drübergemalt hat)
void redrawDotsInRegion(int16_t rx1, int16_t ry1, int16_t rw, int16_t rh) {
  int16_t rx2 = rx1 + rw - 1, ry2 = ry1 + rh - 1;
  for (int i = 0; i < MAX_TARGETS; i++) {
    if (!dots[i].active) continue;
    if (dots[i].bx <= rx2 && dots[i].bx + dots[i].bw - 1 >= rx1 &&
        dots[i].by <= ry2 && dots[i].by + dots[i].bh - 1 >= ry1) {
      drawDot(i);
    }
  }
}

// ── Sichtbarer Sonar-Ping: ein Ring, der vom Ursprung nach außen läuft ──
void drawPingRing(float r, uint16_t color) {
  for (int s = 0; s < NUM_SECTORS; s++) {
    float c = SECTOR_CENTERS[s];
    for (float a = c - SECTOR_HALF; a <= c + SECTOR_HALF; a += 0.5f) {
      float rad = (a - 90.0f) * PI / 180.0f;
      int16_t px = RADAR_CX + (int16_t)(r * cosf(rad));
      int16_t py = RADAR_CY + (int16_t)(r * sinf(rad));
      if (px < 0 || px >= SCREEN_W || py < 0 || py >= SCREEN_H) continue;
      // Range-HUD-Bar an der Basis aussparen, sonst überschreibt der
      // durchlaufende Ping kurzzeitig die Zahl
      if (px >= RANGE_HUD_X && px <= RANGE_HUD_X + RANGE_HUD_W &&
          py >= RANGE_HUD_Y && py <= RANGE_HUD_Y + RANGE_HUD_H) continue;
      display.drawPixel(px, py, color);
    }
  }
}

// Hintergrundfarbe, die an Radius r "eigentlich" stehen würde (feste
// Range-Ring-Farbe falls Treffer, sonst normale BG-Farbe) — für das
// günstige Zurückschreiben beim Löschen des Ping-Rings.
uint16_t bgColorAtRadius(float r) {
  float rangeMm[]  = {1000,2000,3000,4000,5000,6000,7000,8000};
  uint16_t rangeCol[] = {C_GREEN,0x00C0,0x00C0,C_GDIM,0x00C0,0x00C0,0x00C0,C_GREEN};
  for (int k = 0; k < 8; k++) {
    if (fabsf(distToR(rangeMm[k]) - r) < 1.0f) return rangeCol[k];
  }
  return C_BG;
}

// Bounding-Box eines Rings bei Radius r über alle Sektoren hinweg — NUR
// zur groben Prüfung, ob ein Dot vom Ring gekreuzt wird, NICHT für einen
// vollflächigen Redraw (der wäre bei einem 120°-Bogen riesig und würde
// weit entfernte UI-Elemente wie Info-Box/Button mit überschreiben).
void ringBoundingBox(float r, int16_t &bx1, int16_t &by1, int16_t &bx2, int16_t &by2) {
  float minA = SECTOR_CENTERS[0] - SECTOR_HALF;
  float maxA = SECTOR_CENTERS[NUM_SECTORS-1] + SECTOR_HALF;
  bx1 = 32000; by1 = 32000; bx2 = -32000; by2 = -32000;
  for (float a = minA; a <= maxA; a += 2.0f) {
    float rad = (a - 90.0f) * PI / 180.0f;
    int16_t px = RADAR_CX + (int16_t)(r * cosf(rad));
    int16_t py = RADAR_CY + (int16_t)(r * sinf(rad));
    if (px < bx1) bx1 = px; if (px > bx2) bx2 = px;
    if (py < by1) by1 = py; if (py > by2) by2 = py;
  }
  bx1 -= 2; by1 -= 2; bx2 += 2; by2 += 2;
}

// Sweep-Dauer bewusst etwas kürzer als PULSE_INTERVAL_MS (Puls löst den
// Ring aus, siehe triggerPing() in updateBackgroundPulse) — Ring läuft
// exakt im Takt des Sounds, kein eigener Pause-Timer mehr nötig.
const float    PING_SPEED_PX_MS = 440.0f / (PULSE_INTERVAL_MS - 200.0f);
const uint32_t PING_FRAME_MS    = 40;                // ~25 FPS für die Animation

float    pingRadius        = 0;
float    pingPrevRadius    = -1;
bool     pingRunning        = false;
uint32_t lastPingFrameMs   = 0;

// Wird vom Sound-Puls aufgerufen — startet den Ring exakt im gleichen Moment
void triggerPing() {
  pingRadius   = 0;
  pingRunning  = true;
}

void updatePing() {
  uint32_t now = millis();
  if (now - lastPingFrameMs < PING_FRAME_MS) return;
  float dt = (float)(now - lastPingFrameMs);
  lastPingFrameMs = now;

  // Alten Ring löschen: NUR die exakt gleichen Pixel zurückschreiben, die
  // gezeichnet wurden (billig, ~240 Punkte) statt eine teure
  // Bounding-Box-Fläche über redrawBg() neu zu zeichnen (das kostet pro
  // Aufruf zehntausende Iterationen und malt dabei über Info-Box/Button).
  if (pingPrevRadius >= 0) {
    drawPingRing(pingPrevRadius, bgColorAtRadius(pingPrevRadius));
    int16_t bx1, by1, bx2, by2;
    ringBoundingBox(pingPrevRadius, bx1, by1, bx2, by2);
    redrawDotsInRegion(bx1, by1, bx2 - bx1 + 1, by2 - by1 + 1);
    pingPrevRadius = -1;
  }

  if (!pingRunning) return;

  drawPingRing(pingRadius, C_GREEN);
  pingPrevRadius = pingRadius;
  pingRadius += PING_SPEED_PX_MS * dt;

  if (pingRadius > RADAR_R) {
    pingRunning = false;
  }
}

// ── Hintergrund initial ──
void drawBackground() {
  display.fillScreen(C_BG);
  for (int s=0; s<NUM_SECTORS; s++) {
    float c = SECTOR_CENTERS[s];

    float rd[]={1000,2000,3000,4000,5000,6000,7000,8000};
    uint16_t rc[]={C_GREEN,0x00C0,0x00C0,C_GDIM,0x00C0,0x00C0,0x00C0,C_GREEN};
    for (int ri=0; ri<8; ri++) {
      float r=distToR(rd[ri]);
      for (float a=c-SECTOR_HALF; a<=c+SECTOR_HALF; a+=0.5f) {
        float rad=(a-90.0f)*PI/180.0f;
        int16_t px=RADAR_CX+(int16_t)(r*cosf(rad)), py=RADAR_CY+(int16_t)(r*sinf(rad));
        if (px>=0&&px<SCREEN_W&&py>=0&&py<SCREEN_H) display.drawPixel(px,py,rc[ri]);
      }
      // Distanz-Label nur an der äußeren rechten Kante von Sektor 0 setzen,
      // sonst würden sich die Labels beider Sektoren an der gemeinsamen
      // Nahtstelle (60°) überlappen.
      if (s==0) {
        display.setTextColor(ri==0||ri==7?C_GREEN:0x0380); display.setTextSize(1);
        char buf[8]; sprintf(buf,"%.0fm",rd[ri]/1000.0f);
        float rR=(c+SECTOR_HALF+6.0f-90.0f)*PI/180.0f;  // etwas außerhalb der Randlinie, nicht mehr drauf
        int16_t lx=RADAR_CX+(int16_t)((r+4)*cosf(rR))+4, ly=RADAR_CY+(int16_t)((r+4)*sinf(rR))-4;
        if (lx>=0&&lx<SCREEN_W&&ly>=0&&ly<SCREEN_H) { display.setCursor(lx,ly); display.print(buf); }
      }
    }

    float radL=(c-SECTOR_HALF-90.0f)*PI/180.0f, radR=(c+SECTOR_HALF-90.0f)*PI/180.0f;
    for (int off=-1; off<=1; off++) {
      display.drawLine(RADAR_CX+off,RADAR_CY,RADAR_CX+off+(int16_t)(RADAR_R*cosf(radL)),RADAR_CY+(int16_t)(RADAR_R*sinf(radL)),C_GREEN);
      display.drawLine(RADAR_CX+off,RADAR_CY,RADAR_CX+off+(int16_t)(RADAR_R*cosf(radR)),RADAR_CY+(int16_t)(RADAR_R*sinf(radR)),C_GREEN);
    }

    int degs[]={-60,-30,0,30,60};
    for (int i=0; i<5; i++) {
      float rad=(c+degs[i]-90.0f)*PI/180.0f;
      display.drawLine(RADAR_CX,RADAR_CY,RADAR_CX+(int16_t)(RADAR_R*cosf(rad)),RADAR_CY+(int16_t)(RADAR_R*sinf(rad)),degs[i]==0?C_GDIM:0x0260);
      char buf[5]; sprintf(buf,"%d",(int)(c+degs[i]));
      float lr=RADAR_R+14.0f;
      int16_t lx=RADAR_CX+(int16_t)(lr*cosf(rad))-6, ly=RADAR_CY+(int16_t)(lr*sinf(rad))-4;
      if (lx>=0&&lx<SCREEN_W&&ly>=0) { display.setTextColor(degs[i]==0?C_GREEN:0x0380); display.setTextSize(1); display.setCursor(lx,ly); display.print(buf); }
    }
  }
  display.fillCircle(RADAR_CX,RADAR_CY,5,C_GREEN);
  display.drawCircle(RADAR_CX,RADAR_CY,9,C_GDIM);
  // Info-Box
  display.fillRect(670,0,130,70,0x0820);
  display.setTextColor(C_GDIM); display.setTextSize(1);
  display.setCursor(675, 4); display.print("RD-03D 24GHz");
  display.setCursor(675,14); display.print("3T 8m 120deg");
  display.setCursor(675,24); display.print("AP: RadarNet");
}

// 3 Targets vom einzigen Sensor
static const uint16_t DOT_COLS[MAX_TARGETS] = {C_RED, C_AMBER, C_CYAN};

void eraseDot(int i) {
  if (!dots[i].active) return;
  redrawBg(dots[i].bx, dots[i].by, dots[i].bw, dots[i].bh);
  dots[i].active = false;
}

void drawDot(int i) {
  int16_t tx, ty;
  targetToPixel(targets[i].x, targets[i].y, tx, ty);
  uint16_t col = DOT_COLS[i];
  int16_t bx=tx-22, by=ty-22, bw=44, bh=44;
  display.drawCircle(tx,ty,18,col);
  display.drawCircle(tx,ty,11,col);
  display.fillCircle(tx,ty, 5,col);
  dots[i]      = {tx,ty,bx,by,bw,bh,true};
  lastDrawn[i] = targets[i];
}

bool staleNow[MAX_TARGETS];

// ── Distanz-Labels fest am Ursprung statt am beweglichen Punkt ──
// Fester, umrandeter Kasten unter dem Mute-Button — nicht mehr am
// beweglichen Ursprung, sondern in der gleichen UI-Spalte wie Info-Box
// und Button. Malt sich komplett selbst neu (eigener Hintergrund-Fill),
// braucht kein redrawBg() und kann daher auch nichts anderes überschreiben.
#define TGT_BOX_X   10
#define TGT_BOX_Y   400
#define TGT_BOX_W   130
#define TGT_BOX_LINE_H 14
#define TGT_BOX_H   (MAX_TARGETS * TGT_BOX_LINE_H + 16)
const uint32_t TGT_BOX_INTERVAL_MS = 150;
uint32_t lastTgtBoxMs = 0;

void drawTargetBox() {
  display.fillRect(TGT_BOX_X, TGT_BOX_Y, TGT_BOX_W, TGT_BOX_H, 0x0820);
  display.drawRect(TGT_BOX_X, TGT_BOX_Y, TGT_BOX_W, TGT_BOX_H, C_GDIM);
  display.setTextColor(C_GDIM); display.setTextSize(1);
  display.setCursor(TGT_BOX_X + 6, TGT_BOX_Y + 4);
  display.print("TARGETS");

  display.setTextSize(1);
  int line = 0;
  for (int i = 0; i < MAX_TARGETS; i++) {
    if (!targets[i].valid || staleNow[i]) continue;
    float dist = sqrtf(targets[i].x*targets[i].x + targets[i].y*targets[i].y);
    char buf[20];
    if (targets[i].spd > 2.0f) sprintf(buf, "T%d %.2fm %dcm/s", i+1, dist/1000.0f, (int)targets[i].spd);
    else                       sprintf(buf, "T%d %.2fm", i+1, dist/1000.0f);
    display.setTextColor(DOT_COLS[i]);
    display.setCursor(TGT_BOX_X + 6, TGT_BOX_Y + 16 + line * TGT_BOX_LINE_H);
    display.print(buf);
    line++;
  }
}

void updateTargetBox() {
  if (millis() - lastTgtBoxMs < TGT_BOX_INTERVAL_MS) return;
  lastTgtBoxMs = millis();
  drawTargetBox();
}

// ── Großer Range-Readout an der Basis der Radar-Kuppel (Ursprungspunkt) ──
// Referenz: Tracker-UI mit großer Zahl direkt unter der Scope-Grafik.
// Bei unserem Layout sitzt der Ursprung am unteren Bildschirmrand, daher
// wird die Bar dort platziert statt "unter" dem Bogen — überdeckt den
// Ursprungsmarker bewusst, genau wie im Referenzbild der Fokuspunkt der
// Kuppel durch die Zahl ersetzt wird.
const uint32_t RANGE_HUD_INTERVAL_MS = 150;
uint32_t lastRangeHudMs = 0;

void drawRangeHud() {
  display.fillRect(RANGE_HUD_X, RANGE_HUD_Y, RANGE_HUD_W, RANGE_HUD_H, 0x0000);
  display.drawRect(RANGE_HUD_X, RANGE_HUD_Y, RANGE_HUD_W, RANGE_HUD_H, C_GDIM);

  float dist = closestTargetDistMm();
  display.setTextSize(3);
  char buf[10];
  if (dist >= 0) {
    display.setTextColor(C_RED);
    sprintf(buf, "%.2fm", dist/1000.0f);
  } else {
    display.setTextColor(C_GDIM);
    sprintf(buf, "---");
  }
  int16_t textW = strlen(buf) * 18;  // grobe Breitenschätzung bei textSize(3)
  display.setCursor(RADAR_CX - textW/2, RANGE_HUD_Y + (RANGE_HUD_H - 24) / 2);
  display.print(buf);

  // Ziel-Dots, die von der Bar überdeckt wurden, wiederherstellen
  redrawDotsInRegion(RANGE_HUD_X, RANGE_HUD_Y, RANGE_HUD_W, RANGE_HUD_H);
}

void updateRangeHud() {
  if (millis() - lastRangeHudMs < RANGE_HUD_INTERVAL_MS) return;
  lastRangeHudMs = millis();
  drawRangeHud();
}

void updateDots() {
  uint32_t now = millis();
  for (int i = 0; i < MAX_TARGETS; i++) {
    staleNow[i] = false;
    if (!targets[i].valid) {
      stillSinceMs[i] = 0;
      prevRaw[i].valid = false;
      if (dots[i].active) eraseDot(i);
      continue;
    }

    // Echte Bewegung seit dem letzten Frame? (Ground-Truth, unabhängig vom
    // Rendering — läuft auch weiter, wenn der Punkt gerade ausgeblendet ist)
    bool movedRaw = !prevRaw[i].valid ||
                    fabsf(targets[i].x - prevRaw[i].x) > MOVE_THRESH_MM ||
                    fabsf(targets[i].y - prevRaw[i].y) > MOVE_THRESH_MM;
    if (movedRaw) stillSinceMs[i] = now;
    prevRaw[i] = targets[i];

    bool stale = (now - stillSinceMs[i]) > STILL_TIMEOUT_MS;
    staleNow[i] = stale;
    if (stale) {
      if (dots[i].active) eraseDot(i);
      continue;  // ausgeblendet, Tracking läuft im Hintergrund weiter
    }

    bool moved = fabsf(targets[i].x - lastDrawn[i].x) > MOVE_THRESH_MM ||
                 fabsf(targets[i].y - lastDrawn[i].y) > MOVE_THRESH_MM;
    if (dots[i].active && !moved) continue;
    if (dots[i].active) eraseDot(i);
    drawDot(i);
  }

  // Info-Box aktualisieren (nur noch Verbindungsstatus — Zielanzahl/Distanz
  // stehen jetzt in der TARGETS-Box und im Range-HUD, keine Duplikate mehr)
  display.fillRect(671,34,128,36,0x0820);

  // Verbindungsstatus zum XIAO (Transmitter)
  bool alive = (millis() - lastPacketMs) < 3000;
  display.setTextColor(alive ? C_GREEN : C_RED); display.setTextSize(1);
  display.setCursor(675,36);
  display.print(alive ? "TX: OK  " : "TX: ---  ");
  char fb[16]; snprintf(fb, sizeof(fb), "fr:%lu", (unsigned long)frameCount);
  display.print(fb);
}

uint32_t lastUptimeMs = 0;
void updateUptime() {
  if (millis()-lastUptimeMs < 1000) return;
  lastUptimeMs = millis();
  // Verbindungsverlust: alle Dots löschen
  if ((millis() - lastPacketMs) > 3000 && txConnected) {
    for (int i=0; i<MAX_TARGETS; i++) {
      targets[i].valid = false;
      if (dots[i].active) eraseDot(i);
    }
  }
}

// ── Periodischer Voll-Redraw gegen "Schatten-Streifen" ──
// Die regionalen Teil-Redraws (redrawBg beim Dot-Löschen) summieren über
// Zeit kleine Rundungsdifferenzen zu sichtbaren Artefakten. Ein kompletter
// Neuzeichnen-Durchlauf alle 10s beseitigt das zuverlässig.
uint32_t lastBgRefreshMs = 0;
const uint32_t BG_REFRESH_INTERVAL_MS = 10000;

void refreshBackground() {
  drawBackground();
  drawMuteButton();
  for (int i = 0; i < MAX_TARGETS; i++) {
    dots[i].active = false;
    if (targets[i].valid) drawDot(i);
  }
  drawTargetBox();
  drawRangeHud();
  pingPrevRadius = -1;  // Ping-Ring-Status zurücksetzen, Bildschirm ist frisch
}

// ── Tracker-Sound ──

// Entfernung des nächstgelegenen validen Targets in mm, -1 wenn keins
float closestTargetDistMm() {
  float best = -1.0f;
  for (int i = 0; i < MAX_TARGETS; i++) {
    if (!targets[i].valid || staleNow[i]) continue;
    float d = sqrtf(targets[i].x * targets[i].x + targets[i].y * targets[i].y);
    if (best < 0 || d < best) best = d;
  }
  return best;
}

// Nicht-blockierend: Tempo & Tonhöhe der Pings skalieren mit der Distanz
// zum nächsten Ziel. Nah = schnell + hoch, weit = langsam + tief.
void updateTrackerSound() {
  uint32_t now = millis();

  if (buzzerMuted) {
    if (beepActive) { noTone(BUZZER_PIN); beepActive = false; chirpStage = 0; }
    return;
  }

  // Chirp-Stufen weiterschalten: tief -> mittel -> Zielfrequenz -> fertig
  if (chirpStage == 1 && (now - chirpStageStart >= PING_STAGE1_MS)) {
    tone(BUZZER_PIN, (uint16_t)(chirpMainFreq * 0.80f), PING_STAGE2_MS);
    chirpStage = 2;
    chirpStageStart = now;
  } else if (chirpStage == 2 && (now - chirpStageStart >= PING_STAGE2_MS)) {
    tone(BUZZER_PIN, chirpMainFreq, PING_STAGE3_MS);
    chirpStage = 3;
    chirpStageStart = now;
  } else if (chirpStage == 3 && (now - chirpStageStart >= PING_STAGE3_MS)) {
    chirpStage = 0;
  }
  beepActive = (chirpStage != 0);

  float dist = closestTargetDistMm();
  if (dist < 0) return;  // kein Ziel -> still

  float d = constrain(dist, BEEP_DIST_MIN, BEEP_DIST_MAX);
  float t = (d - BEEP_DIST_MIN) / (BEEP_DIST_MAX - BEEP_DIST_MIN);  // 0=nah, 1=weit

  uint16_t interval = BEEP_INTERVAL_MIN + (uint16_t)(t * (BEEP_INTERVAL_MAX - BEEP_INTERVAL_MIN));
  uint16_t freq     = BEEP_FREQ_MAX     - (uint16_t)(t * (BEEP_FREQ_MAX - BEEP_FREQ_MIN));

  if (chirpStage == 0 && (now - lastBeepMs >= interval)) {
    tone(BUZZER_PIN, (uint16_t)(freq * 0.55f), PING_STAGE1_MS);  // tiefer Einstieg, steigt danach an
    chirpMainFreq   = freq;
    chirpStage      = 1;
    chirpStageStart = now;
    beepActive      = true;
    lastBeepMs      = now;
  }
}

// Konstanter Hintergrund-Puls, unabhängig von Zielerkennung — läuft immer,
// solange nicht gemutet. Der Annäherungs-Ping hat Vorrang: ist gerade ein
// Ping aktiv, wird der Puls für diesen Zyklus übersprungen statt beide
// gleichzeitig auf denselben Piezo zu legen.
void updateBackgroundPulse() {
  uint32_t now = millis();

  if (buzzerMuted) {
    if (pulseActive) { noTone(BUZZER_PIN); pulseActive = false; }
    return;
  }

  if (pulseActive && (now - pulseStartMs >= PULSE_DUR_MS)) {
    pulseActive = false;
  }

  if (beepActive) {
    lastPulseMs = now;  // Intervall verschieben, damit er nicht direkt danach nachfeuert
    return;
  }

  if (!pulseActive && (now - lastPulseMs >= PULSE_INTERVAL_MS)) {
    tone(BUZZER_PIN, PULSE_FREQ, PULSE_DUR_MS);
    pulseActive  = true;
    pulseStartMs = now;
    lastPulseMs  = now;
    triggerPing();
  }
}

// ── Mute-Button zeichnen ──
void drawMuteButton() {
  uint16_t border = buzzerMuted ? C_RED : C_GREEN;
  uint16_t fill    = buzzerMuted ? 0x4000 : 0x0320;
  display.fillRect(BTN_X, BTN_Y, BTN_W, BTN_H, fill);
  display.drawRect(BTN_X, BTN_Y, BTN_W, BTN_H, border);
  display.setTextColor(border); display.setTextSize(1);
  display.setCursor(BTN_X + 14, BTN_Y + 16);
  display.print(buzzerMuted ? "STUMM" : "BUZZER AN");
}

// ── Touch-Abfrage mit Entprellung ──
// Ein einzelner Tap löst laut Arduino-Doku mehrere (5-20) Touch-Events
// hintereinander aus, solange der Finger auf dem Screen liegt. Wir
// toggeln daher nur beim ÜBERGANG "nicht berührt" -> "berührt", nicht
// bei jedem einzelnen Event, plus eine kurze Mindestpause zwischen zwei
// Toggles als zusätzliche Sicherheit.
bool     touchDownLast = false;
uint32_t lastToggleMs  = 0;

void checkTouch() {
  uint8_t contacts;
  GDTpoint_t points[5];
  contacts = touch.getTouchPoints(points);

  bool touchingButton = false;
  for (uint8_t i = 0; i < contacts; i++) {
    // Touch-Controller liefert Rohkoordinaten im nativen Hochformat
    // (480x800), unabhängig von display.setRotation(1). Umrechnung auf
    // das gedrehte Landscape-Koordinatensystem (800x480), das die
    // Grafik tatsächlich nutzt.
    int16_t tx = points[i].y;
    int16_t ty = 480 - points[i].x;
    if (tx >= BTN_X && tx <= BTN_X + BTN_W &&
        ty >= BTN_Y && ty <= BTN_Y + BTN_H) {
      touchingButton = true;
      break;
    }
  }

  if (touchingButton && !touchDownLast && (millis() - lastToggleMs > 300)) {
    buzzerMuted = !buzzerMuted;
    lastToggleMs = millis();
    drawMuteButton();
  }
  touchDownLast = touchingButton;
}

// ── Hotspot starten ──
void startAP() {
  Serial.print("Starte Hotspot '");
  Serial.print(AP_SSID);
  Serial.println("'...");
  // Rückgabewert prüfen: fehlt die WLAN-Firmware im QSPI-Flash (oder wurde sie
  // beim Core-Wechsel gelöscht), schlägt beginAP() still fehl, die Anzeige zeigt
  // IP 0.0.0.0 und der XIAO findet nie ein Netz — das sah bisher wie ein Problem
  // der Boot-Reihenfolge aus. Jetzt: Fehler anzeigen und alle 5 s neu versuchen.
  int status;
  while ((status = WiFi.beginAP(AP_SSID, AP_PASS, 6)) != WL_AP_LISTENING) {
    Serial.print("Hotspot-Start fehlgeschlagen, Status "); Serial.println(status);
    display.setTextColor(C_RED); display.setTextSize(1);
    display.setCursor(120,265);
    display.println("AP FAILED - WLAN-Firmware fehlt? (Beispiel WiFiFirmwareUpdater flashen)");
    delay(5000);
  }
  delay(5000);
  // Giga vergibt eigene IP — einfach nehmen was der Stack setzt
  Serial.print("AP IP: ");
  Serial.println(WiFi.localIP());
  delay(1000);
  udp.begin(UDP_PORT);
  Serial.print("UDP lauscht auf Port ");
  Serial.println(UDP_PORT);
}

void setup() {
  Serial.begin(115200);

  display.begin();
  display.setRotation(1);
  display.fillScreen(C_BG);
  display.setTextColor(C_GREEN); display.setTextSize(2);
  display.setCursor(120,200); display.println("mmWAVE RADAR RD-03D");
  display.setTextSize(1); display.setTextColor(C_GDIM);
  display.setCursor(120,235); display.println("Starte Hotspot...");

  touch.begin();
  Wire1.begin();

  pinMode(BUZZER_PIN, OUTPUT);
  noTone(BUZZER_PIN);

  for (int i=0; i<MAX_TARGETS; i++) {
    targets[i] = lastDrawn[i] = {0,0,0,false};
    dots[i]    = {0,0,0,0,0,0,false};
  }

  startAP();

  display.setCursor(120,250);
  display.print("AP: "); display.print(AP_SSID);
  display.print("  IP: "); display.println(WiFi.localIP());
  delay(1000);

  drawBackground();
  drawMuteButton();
  Serial.println("Giga R1 bereit — warte auf XIAO...");
}

void loop() {
  receiveUdp();
  updateDots();
  updateTargetBox();
  updateRangeHud();
  updatePing();
  updateUptime();
  updateTrackerSound();
  updateBackgroundPulse();
  checkTouch();

  if (millis() - lastBgRefreshMs >= BG_REFRESH_INTERVAL_MS) {
    lastBgRefreshMs = millis();
    refreshBackground();
  }
}
