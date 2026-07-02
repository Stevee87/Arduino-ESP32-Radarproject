/*
 * RD-03D Radar Display — Arduino Giga R1
 * Empfängt UDP-Pakete vom R4 WiFi und zeigt Targets auf dem Display
 *
 * NETZWERK:
 *   Giga R1 ist Hotspot: SSID "RadarNet", PW "radar12345"
 *   Eigene IP: 192.168.4.1
 *   UDP Port: 4210
 */

#include <WiFi.h>
#include <WiFiUDP.h>
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

uint32_t frameCount = 0;
uint32_t lastPacketMs = 0;
bool     r4Connected = false;

// ── UDP empfangen ──
void receiveUdp() {
  int sz = udp.parsePacket();
  static uint32_t lastDebug = 0;
  if (millis() - lastDebug > 2000) {
    lastDebug = millis();
    Serial.print("UDP check sz="); Serial.println(sz);
  }
  if (sz < (int)sizeof(UdpPacket)) return;

  UdpPacket pkt;
  udp.read((uint8_t*)&pkt, sizeof(pkt));

  if (pkt.magic != 0xD03DA7A) return;

  frameCount = pkt.frameCount;
  lastPacketMs = millis();
  r4Connected = true;

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
  for (float a=-SECTOR_HALF; a<=SECTOR_HALF; a+=0.6f) {
    float rad=(a-90.0f)*PI/180.0f;
    for (float r=0; r<=RADAR_R; r+=1.5f) {
      int16_t px=RADAR_CX+(int16_t)(r*cosf(rad)), py=RADAR_CY+(int16_t)(r*sinf(rad));
      if (px>=rx1&&px<=rx2&&py>=ry1&&py<=ry2) display.drawPixel(px,py,0x0180);
    }
  }
  float rd[]={500,1000,2000,3000,4000,5000,6000,7000,8000};
  uint16_t rc[]={0x00C0,C_GREEN,0x00C0,0x00C0,C_GDIM,0x00C0,0x00C0,0x00C0,C_GREEN};
  for (int ri=0; ri<9; ri++) {
    float r=distToR(rd[ri]);
    for (float a=-SECTOR_HALF; a<=SECTOR_HALF; a+=0.5f) {
      float rad=(a-90.0f)*PI/180.0f;
      int16_t px=RADAR_CX+(int16_t)(r*cosf(rad)), py=RADAR_CY+(int16_t)(r*sinf(rad));
      if (px>=rx1&&px<=rx2&&py>=ry1&&py<=ry2) display.drawPixel(px,py,rc[ri]);
    }
  }
  int degs[]={-60,-30,0,30,60};
  for (int i=0; i<5; i++) {
    float rad=(degs[i]-90.0f)*PI/180.0f;
    for (float r=0; r<=RADAR_R; r+=1.5f) {
      int16_t px=RADAR_CX+(int16_t)(r*cosf(rad)), py=RADAR_CY+(int16_t)(r*sinf(rad));
      if (px>=rx1&&px<=rx2&&py>=ry1&&py<=ry2) display.drawPixel(px,py,degs[i]==0?C_GDIM:0x0200);
    }
  }
  if (RADAR_CX>=rx1&&RADAR_CX<=rx2&&RADAR_CY>=ry1&&RADAR_CY<=ry2) {
    display.fillCircle(RADAR_CX,RADAR_CY,5,C_GREEN);
    display.drawCircle(RADAR_CX,RADAR_CY,9,C_GDIM);
  }
}

// ── Hintergrund initial ──
void drawBackground() {
  display.fillScreen(C_BG);
  for (float a=-SECTOR_HALF; a<=SECTOR_HALF; a+=0.6f) {
    float rad=(a-90.0f)*PI/180.0f;
    for (float r=0; r<=RADAR_R; r+=1.5f) {
      int16_t px=RADAR_CX+(int16_t)(r*cosf(rad)), py=RADAR_CY+(int16_t)(r*sinf(rad));
      if (px>=0&&px<SCREEN_W&&py>=0&&py<SCREEN_H) display.drawPixel(px,py,0x0180);
    }
  }
  float rd[]={500,1000,2000,3000,4000,5000,6000,7000,8000};
  uint16_t rc[]={0x00C0,C_GREEN,0x00C0,0x00C0,C_GDIM,0x00C0,0x00C0,0x00C0,C_GREEN};
  for (int ri=0; ri<9; ri++) {
    float r=distToR(rd[ri]);
    for (float a=-SECTOR_HALF; a<=SECTOR_HALF; a+=0.5f) {
      float rad=(a-90.0f)*PI/180.0f;
      int16_t px=RADAR_CX+(int16_t)(r*cosf(rad)), py=RADAR_CY+(int16_t)(r*sinf(rad));
      if (px>=0&&px<SCREEN_W&&py>=0&&py<SCREEN_H) display.drawPixel(px,py,rc[ri]);
    }
    display.setTextColor(ri==1||ri==8?C_GREEN:0x0380); display.setTextSize(1);
    char buf[8];
    if (rd[ri]<1000) sprintf(buf,"0.5m"); else sprintf(buf,"%.0fm",rd[ri]/1000.0f);
    float rR=(SECTOR_HALF-90.0f)*PI/180.0f;
    int16_t lx=RADAR_CX+(int16_t)((r+4)*cosf(rR))+2, ly=RADAR_CY+(int16_t)((r+4)*sinf(rR))-4;
    if (lx>=0&&lx<SCREEN_W&&ly>=0&&ly<SCREEN_H) { display.setCursor(lx,ly); display.print(buf); }
  }
  float radL=(-SECTOR_HALF-90.0f)*PI/180.0f, radR=(SECTOR_HALF-90.0f)*PI/180.0f;
  display.drawLine(RADAR_CX,RADAR_CY,RADAR_CX+(int16_t)(RADAR_R*cosf(radL)),RADAR_CY+(int16_t)(RADAR_R*sinf(radL)),C_GREEN);
  display.drawLine(RADAR_CX,RADAR_CY,RADAR_CX+(int16_t)(RADAR_R*cosf(radR)),RADAR_CY+(int16_t)(RADAR_R*sinf(radR)),C_GREEN);
  int degs[]={-60,-30,0,30,60};
  for (int i=0; i<5; i++) {
    float rad=(degs[i]-90.0f)*PI/180.0f;
    display.drawLine(RADAR_CX,RADAR_CY,RADAR_CX+(int16_t)(RADAR_R*cosf(rad)),RADAR_CY+(int16_t)(RADAR_R*sinf(rad)),degs[i]==0?C_GDIM:0x0260);
    char buf[5]; sprintf(buf,"%d",degs[i]);
    float lr=RADAR_R+14.0f;
    int16_t lx=RADAR_CX+(int16_t)(lr*cosf(rad))-6, ly=RADAR_CY+(int16_t)(lr*sinf(rad))-4;
    if (lx>=0&&lx<SCREEN_W&&ly>=0) { display.setTextColor(degs[i]==0?C_GREEN:0x0380); display.setTextSize(1); display.setCursor(lx,ly); display.print(buf); }
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
  int16_t bx=tx-20, by=ty-20, bw=112, bh=44;
  display.drawCircle(tx,ty,18,col);
  display.drawCircle(tx,ty,11,col);
  display.fillCircle(tx,ty, 5,col);
  display.setTextColor(C_WHITE); display.setTextSize(1);
  float dist = sqrtf(targets[i].x*targets[i].x + targets[i].y*targets[i].y);
  char buf[14]; sprintf(buf,"%.2fm", dist/1000.0f);
  display.setCursor(tx+21,ty-8); display.print(buf);
  if (targets[i].spd > 2.0f) {
    char sb[14]; sprintf(sb,"%dcm/s",(int)targets[i].spd);
    display.setCursor(tx+21,ty+4); display.print(sb);
  }
  dots[i]      = {tx,ty,bx,by,bw,bh,true};
  lastDrawn[i] = targets[i];
}

void updateDots() {
  for (int i = 0; i < MAX_TARGETS; i++) {
    if (!targets[i].valid) {
      if (dots[i].active) eraseDot(i);
      continue;
    }
    bool moved = fabsf(targets[i].x - lastDrawn[i].x) > MOVE_THRESH_MM ||
                 fabsf(targets[i].y - lastDrawn[i].y) > MOVE_THRESH_MM;
    if (dots[i].active && !moved) continue;
    if (dots[i].active) eraseDot(i);
    drawDot(i);
  }

  // Info-Box aktualisieren
  display.fillRect(671,34,128,36,0x0820);
  int cnt = 0;
  for (int i=0; i<MAX_TARGETS; i++) if (targets[i].valid) cnt++;

  // R4 Verbindungsstatus
  bool alive = (millis() - lastPacketMs) < 3000;
  display.setTextColor(alive ? C_GREEN : C_RED); display.setTextSize(1);
  display.setCursor(675,36);
  display.print(alive ? "R4: OK  " : "R4: ---  ");
  char fb[12]; sprintf(fb,"fr:%lu", frameCount);
  display.print(fb);

  display.setTextColor(cnt>0?C_GREEN:C_GDIM);
  char cb[16]; sprintf(cb,"Targets: %d", cnt);
  display.setCursor(675,48); display.print(cb);

  for (int i=0; i<MAX_TARGETS; i++) {
    if (targets[i].valid) {
      float d = sqrtf(targets[i].x*targets[i].x + targets[i].y*targets[i].y);
      char db[20]; sprintf(db,"T%d:%.2fm %dcm/s", i+1, d/1000.0f, (int)targets[i].spd);
      display.setTextColor(DOT_COLS[i]);
      display.setCursor(675,60); display.print(db);
      break;
    }
  }
}

uint32_t lastUptimeMs = 0;
void updateUptime() {
  if (millis()-lastUptimeMs < 1000) return;
  lastUptimeMs = millis();
  // Verbindungsverlust: alle Dots löschen
  if ((millis() - lastPacketMs) > 3000 && r4Connected) {
    for (int i=0; i<MAX_TARGETS; i++) {
      targets[i].valid = false;
      if (dots[i].active) eraseDot(i);
    }
  }
}

// ── Hotspot starten ──
void startAP() {
  Serial.print("Starte Hotspot '");
  Serial.print(AP_SSID);
  Serial.println("'...");
  WiFi.beginAP(AP_SSID, AP_PASS, 6);
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
  Serial.println("Giga R1 bereit — warte auf R4...");
}

void loop() {
  receiveUdp();
  updateDots();
  updateUptime();
}
