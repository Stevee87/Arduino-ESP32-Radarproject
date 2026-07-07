# Arduino ESP32 mmWave Radar Project

No guarantee is provided for errors in the building instructions, the construction process, or anything similar. If you decide to build this project, you do so at your own risk.
This is a hobby project shared as-is. Building this involves handling LiPo batteries and RF hardware — verify all wiring yourself before powering on. No warranty, use at your own risk!

A wireless 24GHz mmWave motion tracking system. A standalone radar node reports detected
targets over Wi-Fi to a separate display unit — so the radar and the screen don't have to
be in the same place.

<img width="662" height="600" alt="v22" src="https://github.com/user-attachments/assets/5aac037d-5a40-4d57-81f7-d398d023f9f8" /> <img width="661" height="677" alt="v21" src="https://github.com/user-attachments/assets/622ae2f7-1498-4317-b406-c157c214ad4a" />



## How it works

The radar module talks to an Arduino GIGA R1 via a Wi-Fi hotspot using a Seeed XIAO
ESP32-S3 as the sender. Because the link is wireless, the radar node can be placed
anywhere as a standalone motion detector, independent of where the display sits.

## Features

- Multi-target tracking (up to 3 people simultaneously, RD-03D built-in algorithm)
- ~7m detection range
- Live radar-style visualization on the GIGA Display Shield
- Distance-reactive tracker sound (passive piezo buzzer, tempo and pitch scale with
  the closest target's distance) — mute toggle built into the touchscreen
- LED battery level indicators on both battery packs
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
| 2 | BW4056 USB-C charging module |
| 1 | Passive piezo buzzer module (VCC/GND/Signal, tracker sound, GIGA side) |
| 2 | Mini Battery Level Indicator, 1S Li-ion — [ElectroPeak BAT-03-086](https://electropeak.com/mini-battery-level-indicator-1s-li-ion), one per LiPo |
| 4 | Magnets |
| — | Wires, M3 heat-set inserts, M3x10mm screws, 3D printer, glue, tape |


## Wiring

<img width="1283" height="1008" alt="circuit diagram v2" src="https://github.com/user-attachments/assets/89379cd5-6137-40ea-9238-3816095cbb12" />

**Power (GIGA side):** LiPo (5000mAh) → TP4056 charging module → toggle switch →
step-up converter set to 6V → GIGA `VIN`

**Power (radar side):** LiPo (1300mAh) → toggle switch → step-up converter set to 5V →
XIAO `BAT+`/`BAT-` (on the back of the board). The XIAO also charges its LiPo directly
over USB-C.

### XIAO ESP32-S3 ↔ RD-03D (transmitter)

| RD-03D pin | XIAO pin | Notes |
|---|---|---|
| TX | D0 (GPIO1) | UART1 RX |
| RX | D1 (GPIO2) | UART1 TX |
| VCC | 5V | from step-up converter |
| GND | GND | |

CAUTION: When charging the battery, set the POWER button to OFF. The same applies when the microcontroller is connected via USB-C. 

Baud rate: 256000. Do **not** use D6/D7 (GPIO43/44) — reserved for USB-Serial (UART0).

### Wi-Fi link
### Arduino Giga R1 ↔ (receiver)
| | Value |
|---|---|
| Mode | GIGA R1 hosts an access point, XIAO connects as client |
| SSID | `RadarNet` |
| Password | `radar12345` |
| UDP port | 4210 |
| GIGA IP | 192.168.4.1 |

### Tracker sound (buzzer)

| Buzzer | GIGA R1 |
|---|---|
| VCC | 5V |
| Signal (I/O) | D9 |
| GND | GND |

CAUTION: When charging the battery, set the POWER button to OFF. The same applies when the microcontroller is connected via USB-C. 

Must be a **passive** piezo buzzer, not active — an active buzzer has its own
oscillator and only turns on/off, ignoring the frequency argument the code sends it.

Behavior: silent when no target is detected. Once a target appears, it emits short
(70ms) pings whose repeat rate and pitch both scale with the distance to the closest
detected target — pings speed up and rise in pitch as someone gets closer (900ms /
700Hz at ~8m down to 120ms / 1800Hz at ≤30cm). A mute toggle button is drawn directly
on the touchscreen; tapping it silences the buzzer without affecting the radar
visualization.

### Battery level indicators

Two [ElectroPeak Mini Battery Level Indicator (1S Li-ion)](https://electropeak.com/mini-battery-level-indicator-1s-li-ion)
boards, one per LiPo pack. These are standalone analog modules (built-in comparator,
±1% accuracy) — no microcontroller pin or firmware involved. Wire each board directly
across its own battery's `+`/`-` terminals (in parallel with the existing TP4056 /
step-up wiring, not in series):

| Indicator LEDs lit | Charge level |
|---|---|
| 4 | 100% |
| 3 | 75% |
| 2 | 50% |
| 1 | 25% |

Board size is tiny (5 × 9.5mm) — tuck it wherever it fits in the enclosure near the
battery leads.

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


## Build photos (old once, without the battery indicator and buzzer)

<img width="1954" height="1086" alt="building pictures" src="https://github.com/user-attachments/assets/91471574-c4d0-4898-a278-bfc3f9234ae1" />


No guarantee is provided for errors in the building instructions, the construction process, or anything similar. If you decide to build this project, you do so at your own risk.
This is a hobby project shared as-is. Building this involves handling LiPo batteries and RF hardware — verify all wiring yourself before powering on. No warranty, use at your own risk!



## License

MIT — see [LICENSE](LICENSE).
