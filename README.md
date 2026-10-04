# Arduino ESP32 mmWave Radar Project
-------------------------------------------------------------------------------------------------------
Safety & disclaimer

This is a hobby project, shared as-is for anyone who wants to build their own version.
A few things to know before you do:


LiPo batteries: incorrect wiring, reverse polarity, or physical damage to a
LiPo cell can cause fire. Double-check polarity yourself before powering anything
on, use a charging module rated for the cell you're using, and never leave a
charging LiPo unattended.
RF hardware: the RD-03D and any radio modules in this project operate
according to their respective regulatory approvals (e.g. CE) for their intended
region — verify this applies to your location before use.
No warranty: this repository (firmware, wiring diagrams, enclosures, and this
README) is provided without any warranty, express or implied. Verify every
connection yourself before applying power — don't assume the diagrams here are
free of mistakes.
Use at your own risk: building and operating this project is entirely your
own responsibility. The author assumes no liability for damage, injury, or loss
resulting from building or using this project.

If you spot an error in the docs, wiring, or code, please open an issue — corrections
are welcome.
-------------------------------------------------------------------------------------------------------


A wireless 24GHz mmWave motion tracking system. A standalone radar node reports detected
targets over Wi-Fi to a separate display unit — so the radar and the screen don't have to
be in the same place.

<img width="662" height="600" alt="v22" src="https://github.com/user-attachments/assets/5aac037d-5a40-4d57-81f7-d398d023f9f8" /> <img width="661" height="677" alt="v21" src="https://github.com/user-attachments/assets/622ae2f7-1498-4317-b406-c157c214ad4a" />



## How it works

The radar module talks to an Arduino GIGA R1 via a Wi-Fi hotspot using a Seeed XIAO
ESP32-S3 as the transmitter. Because the link is wireless, the radar node can be placed
anywhere as a standalone motion detector, independent of where the display sits.

## Features

- Multi-target tracking (up to 3 people simultaneously, RD-03D built-in algorithm)
- 8 m rated detection range (expect 5–7 m indoors)
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

<img width="1225" height="1011" alt="circuit diagram v2" src="https://github.com/user-attachments/assets/cb173ee7-187a-4ba3-9613-0f9de48bbb0f" />


**Power (GIGA side):** LiPo (5000mAh) → BW4056 charging module → toggle switch →
step-up converter set to 7–7.5V → GIGA `VIN`. Do not set it to 6V: that is the
bottom of the GIGA's 6–24V input range and the board browns out as the cell
discharges. Alternatively feed 5V into the GIGA's USB-C port from a 5V boost or
power-bank module and skip `VIN` altogether.

**Power (radar side):** LiPo (1300mAh) → BW4056 charging module → toggle switch →
XIAO `BAT+`/`BAT-` (on the back of the board), **directly**. A second branch from
the switched battery rail feeds the step-up converter set to 5V, which powers
**only the RD-03D**.

Never connect the 5V step-up output to `BAT+`: the BAT pads are the cell
terminals of the XIAO's 1S charger (4.2V maximum), and a cell behind a boost
converter cannot be charged by the XIAO. With the cell on the BAT pads the XIAO's
USB-C also charges it (100mA, slow); never charge through the BW4056 and the XIAO
at the same time.

### XIAO ESP32-S3 ↔ RD-03D (transmitter)

| RD-03D pin | XIAO pin | Notes |
|---|---|---|
| TX | D0 (GPIO1) | UART1 RX |
| RX | D1 (GPIO2) | UART1 TX |
| BAT+ | — | 3,7 V from the TP4056 |
| GND | GND | |

CAUTION: When charging the battery, set the POWER button to OFF. The same applies when the microcontroller is connected via USB-C. 

Baud rate: 256000. Do **not** use D6/D7 (GPIO43/44) — reserved for USB-Serial (UART0).

Antennas: neither the XIAO ESP32-S3 nor the GIGA R1 WiFi has an on-board antenna.
Plug in the u.FL antenna that ships with each board, and route the XIAO's away from
the LiPo and the magnets, or the Wi-Fi link will only work over a few metres.

### Wi-Fi link
### Arduino Giga R1 ↔ (receiver)
| | Value |
|---|---|
| Mode | GIGA R1 hosts an access point, XIAO connects as client |
| SSID | `RadarNet` |
| Password | `radar12345` |
| UDP port | 4210 |
| GIGA IP | 192.168.3.1 (default of the Arduino Mbed core; the sketch never sets an address and the XIAO uses the DHCP gateway, so nothing depends on this value) |

### Tracker sound (buzzer)

| Buzzer | GIGA R1 |
|---|---|
| VCC | 3V3 (GIGA pins are not 5V tolerant and many 3-pin buzzer modules pull the signal line up to VCC) |
| Signal (I/O) | D9 |
| GND | GND |

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
across its own battery's `+`/`-` terminals (in parallel with the existing BW4056 /
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

Open `Firmware/rd03d_xiao_s3_transmitter/rd03d_xiao_s3_transmitter.ino` and flash it to
the XIAO ESP32-S3, then open `Firmware/rd03d_giga_receiver/rd03d_giga_receiver.ino` and
flash it to the GIGA R1 — both via Arduino IDE.

**Board packages:**
- `esp32` by Espressif (3.x) — board "XIAO_ESP32S3", *USB CDC On Boot: Enabled*
- `Arduino Mbed OS GIGA Boards` (4.x) — board "Arduino GIGA R1 WiFi". If the GIGA has never
  run Wi-Fi, flash *File → Examples → STM32H747_System → WiFiFirmwareUpdater* once first.

**Required libraries:**
- `Arduino_GigaDisplay_GFX` (GIGA — display rendering; depends on `Adafruit GFX Library`)
- `Arduino_GigaDisplayTouch` (GIGA — touch input)
- `WiFi` / `WiFiUdp` (both boards, built into their respective cores)

## Enclosures

Four 3D-printable parts are included in `CAD files/` (binary STL; bounding boxes
measured from the files):

| File | Bounding box (mm) | Part |
|---|---|---|
| `CAD files/body display.stl` | 34 × 105 × 113 | display body (GIGA + shield + 1160100 LiPo) |
| `CAD files/top display.stl` | 8 × 85 × 111 | display lid |
| `CAD files/top radarmodule.stl` | 47 × 31 × 87 | radar node body |
| `CAD files/bottom radarmodule.stl` | 47 × 6 × 67 | radar node base plate |


## Build photos (older ones, without the battery indicator and buzzer)

<img width="1954" height="1086" alt="building pictures" src="https://github.com/user-attachments/assets/91471574-c4d0-4898-a278-bfc3f9234ae1" />







## License

MIT — see [LICENSE](LICENSE).
