# Arduino ESP32 mmWave Radar Project

A wireless 24GHz mmWave motion tracking system. A standalone radar node reports detected
targets over Wi-Fi to a separate display unit — so the radar and the screen don't have to
be in the same place.

## How it works

The radar module talks to an Arduino GIGA R1 via a Wi-Fi hotspot using a Seeed XIAO
ESP32-S3 as the sender. Because the link is wireless, the radar node can be placed
anywhere as a standalone motion detector, independent of where the display sits.

## Features

- Multi-target tracking (up to 3 people simultaneously, RD-03D built-in algorithm)
- ~7m detection range
- Live radar-style visualization on the GIGA Display Shield
- Battery powered on both ends, USB-C rechargeable

## Known limitations

- The RD-03D is a **Doppler-based** radar — it only detects *moving* targets. People
  standing still are not detected.
- Works best stationary. It can detect people while the sensor itself is moving, but
  accuracy drops significantly.
- Can detect motion through thin doors/walls, but range is severely reduced — a
  consequence of 24GHz wave physics, not a firmware limitation.
- **Boot order matters**: always power on the GIGA R1 first and let it fully initialize
  before powering the radar node. Otherwise the Wi-Fi connection will not be established.

## Bill of materials

| Qty | Part |
|---|---|
| 1 | Arduino GIGA R1 |
| 1 | Arduino GIGA Display Shield |
| 1 | Seeed Studio XIAO ESP32-S3 |
| 1 | RD-03D radar module (Ai-Thinker) |
| 2 | Toggle switch |
| 2 | MT3608 step-up converter |
| 1 | 3.7V LiPo, 1160100 (≥5000mAh) — GIGA side |
| 1 | 3.7V LiPo, 602560 (~1300mAh) — radar side |
| 1 | BW4056 USB-C charging module |
| 2 | Magnets |
| — | Wires, M3 heat-set inserts, M3x10mm screws, 3D printer, glue, tape |

> Note: the wiring diagram below labels the step-up converters as **MT3601**, while the
> BOM says **MT3608** — double-check which one you actually used before reordering parts.

## Wiring

<img width="1264" height="1082" alt="circuit diagram final" src="https://github.com/user-attachments/assets/20d39d77-e845-42f0-bf09-e311e40ed9f7" />

**Power (GIGA side):** LiPo (5000mAh) → TP4056 charging module → toggle switch →
step-up converter set to 6V → GIGA `VIN`

**Power (radar side):** LiPo (1300mAh) → toggle switch → step-up converter set to 5V →
XIAO `BAT+`/`BAT-` (on the back of the board). The XIAO also charges its LiPo directly
over USB-C.

### XIAO ESP32-S3 ↔ RD-03D (sender)

| RD-03D pin | XIAO pin | Notes |
|---|---|---|
| TX | D0 (GPIO1) | UART1 RX |
| RX | D1 (GPIO2) | UART1 TX |
| VCC | 5V | from step-up converter |
| GND | GND | |

Baud rate: 256000. Do **not** use D6/D7 (GPIO43/44) — reserved for USB-Serial (UART0).

### Wi-Fi link

| | Value |
|---|---|
| Mode | GIGA R1 hosts an access point, XIAO connects as client |
| SSID | `RadarNet` |
| Password | `radar12345` |
| UDP port | 4210 |
| GIGA IP | 192.168.4.1 |

## Firmware

Open `firmware/rd03d_xiao_s3_sender/rd03d_xiao_s3_sender.ino` and flash it to the XIAO
ESP32-S3, then open `firmware/rd03d_giga_receiver/rd03d_giga_receiver.ino` and flash it
to the GIGA R1 — both via Arduino IDE.

**Required libraries:**
- `Arduino_GigaDisplay_GFX` (GIGA — display rendering)
- `Arduino_GigaDisplayTouch` (GIGA — touch input)
- `WiFi` / `WiFiUdp` (both boards, built into their respective cores)

## Enclosures

Three 3D-printable parts are included:

| File | Dimensions (W×D×H) |
|---|---|
| `case/display_case.stl` | 31 × 83 × 109 mm |
| `case/radar_top_case.stl` | 29 × 64 × 27 mm |
| `case/radar_bottom_case.stl` | 29 × 67 × 30 mm |

These are self-designed and still **beta** — expect rough edges. For a display
protection case instead, this community-designed enclosure is recommended:
[Enclosure for Arduino GIGA R1 WiFi and GIGA Display](https://www.printables.com/model/605051-enclosure-for-arduino-giga-r1-wifi-and-giga-displa).

## Build photos

<img width="1954" height="1086" alt="building pictures" src="https://github.com/user-attachments/assets/7b6fa393-973d-4ab6-bb09-1d1765c21c6f" />

## License

MIT — see [LICENSE](LICENSE).
