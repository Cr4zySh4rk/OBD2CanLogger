# OBD2 CAN Logger

> **Capture, decode, and reconstruct your car's ECU maps from the OBD2 port — using an Adafruit Feather M4 CAN, an SD card, and a browser.**

---

## Overview

OBD2 CAN Logger is an open-source hardware + firmware + web-app project that:

1. **Logs every CAN frame** from your car's OBD2 port to an SD card in real-time (CSV + binary)
2. **Streams live data** to a Betaflight-style web dashboard over USB Serial (Web Serial API)
3. **Decodes signals** using standard OBD2 PIDs or custom `.dbc` files from [opendbc](https://github.com/commaai/opendbc)
4. **Reconstructs engine maps** — fuel trims, ignition advance, MAF, MAP, throttle, RPM — as a baseline for tuning

The ultimate goal is to reverse-engineer and document the factory calibration maps of any OBD2-compliant car, so they can serve as a starting point for custom ECU tuning.

---

## Hardware

| Component | Part |
|-----------|------|
| Microcontroller | [Adafruit Feather M4 CAN Express (ATSAMD51J19A)](https://www.adafruit.com/product/4759) |
| SD card + RTC | [Adafruit Adalogger FeatherWing](https://www.adafruit.com/product/2922) |
| CAN Transceiver | **Built-in** on Feather M4 CAN — TCAN1051 on CAN1 (CANH/CANL pins exposed) |
| OBD2 Interface | OBD2 DB9/breakout cable wired to CANH/CANL |

> **Note:** The Feather M4 CAN (product 4759) has an **integrated CANFD transceiver** on CAN1. It is different from the plain Feather M4 Express (product 3857). If you have product 3857 you will need to add an MCP2515 FeatherWing.

### Wiring

```
Feather M4 CAN          OBD2 Connector (J1962)
──────────────          ──────────────────────
CANH           ────────  Pin 6  (CAN High)
CANL           ────────  Pin 14 (CAN Low)
GND            ────────  Pin 4 or 5 (Ground)
```

The Adalogger FeatherWing stacks directly on top of the Feather M4 CAN — no additional wiring needed for SD or RTC.

---

## Project Structure

```
OBD2CanLogger/
├── firmware/
│   └── OBD2CanLogger/
│       └── OBD2CanLogger.ino      # Arduino sketch for Feather M4 CAN
├── webapp/
│   └── index.html                 # Self-contained web dashboard (no server needed)
├── dbc/
│   └── (place .dbc files here)    # Download from opendbc or add your own
└── README.md
```

---

## Firmware

### Requirements

Install the following libraries via the Arduino Library Manager:

| Library | Author | Notes |
|---------|--------|-------|
| `ACANFD_FeatherM4CAN` | Pierre Molinaro | CAN/CANFD driver for the ATSAMD51 built-in controller |
| `SD` | Arduino | SD card read/write |
| `RTClib` | Adafruit | Real-time clock (PCF8523 on Adalogger) |
| `ArduinoJson` | Benoit Blanchon | v6+ — JSON serial protocol |

### Board Setup

1. In Arduino IDE, add Adafruit's board package URL:
   ```
   https://adafruit.github.io/arduino-board-index/package_adafruit_index.json
   ```
2. Install **Adafruit SAMD Boards** via Boards Manager
3. Select: **Adafruit Feather M4 CAN (SAMD51)**
4. Upload `firmware/OBD2CanLogger/OBD2CanLogger.ino`

### Default Behavior

- On power-up, the firmware reads `/config.json` from the SD card (if present)
- Creates a timestamped session log: `session_20240101_120000.csv` and `.can`
- Logs **every CAN frame** to SD
- Streams decoded frames as JSON over USB Serial at 115200 baud
- Red LED blinks every 5 seconds as heartbeat

### SD Card Files

| File | Description |
|------|-------------|
| `/config.json` | Device configuration (auto-created on first config command) |
| `/session_YYYYMMDD_HHMMSS.csv` | Human-readable log — one row per frame |
| `/session_YYYYMMDD_HHMMSS.can` | Binary log — 24 bytes per frame, magic header `OBD2CAN\0` |

### CSV Format

```
timestamp_ms,id_hex,id_dec,extended,fd,dlc,data_hex,data_bytes
0,000007E8,2024,0,0,8,04 41 0C 1A F8 00 00 00,8
4,000007E8,2024,0,0,8,04 41 0D 62 00 00 00 00,8
```

### Serial JSON Protocol

Send commands as JSON terminated by `\n`:

```jsonc
// Get device status
{"cmd":"status"}

// Start a new logging session
{"cmd":"start"}

// Stop logging
{"cmd":"stop"}

// Update configuration (restart to apply bitrate changes)
{"cmd":"config","bitrate":500000,"logcsv":true,"logbin":true,"stream":true}

// List files on SD card
{"cmd":"ls"}
```

The device sends frames as compact JSON:
```json
{"t":1234,"id":"0x7E8","ext":0,"fd":0,"dlc":8,"d":"04 41 0C 1A F8 00 00 00"}
```

### config.json Example

```json
{
  "bitrate": 500000,
  "logcsv": true,
  "logbin": true,
  "stream": true,
  "session": "mycar",
  "filter": []
}
```

Set `"filter": [0x7E8, 0x7DF]` to only log those IDs. Leave empty `[]` to capture everything.

---

## Web App

Open `webapp/index.html` in **Google Chrome** or **Microsoft Edge** (Web Serial API required).

No server, no Node.js, no installation — it's a single self-contained HTML file.

### Tabs

| Tab | Description |
|-----|-------------|
| 📡 **Live Data** | Real-time gauge dashboard + scrolling frame table. Filter by CAN ID. OBD2 PIDs auto-decoded. |
| 🗺️ **Map Reconstruction** | Live scatter plots: TPS/RPM, Fuel Trim/RPM, Ignition Advance/RPM, MAF/RPM, MAP/RPM, Speed/RPM. Export as JSON. |
| 📋 **DBC Decoder** | Load any `.dbc` file (drag & drop) or use built-in OBD2 PID table. Shows live decoded signal values. |
| 💾 **Log Viewer** | Open CSV logs from the SD card. Searchable/filterable. |
| ⚙️ **Configuration** | Configure bitrate, session name, log format, ID filters. Apply directly to device. |
| 🖥️ **Serial Console** | Raw JSON command terminal for direct device communication. |

### Connecting

1. Plug Feather M4 CAN into your computer via USB
2. Open `webapp/index.html` in Chrome
3. Click **Connect USB**
4. Select the correct COM port
5. Live data begins streaming immediately

---

## DBC Files & Signal Decoding

This project uses the [opendbc](https://github.com/commaai/opendbc) database to decode manufacturer-specific CAN messages.

### Quick Start with opendbc

```bash
# Clone opendbc
git clone https://github.com/commaai/opendbc.git

# Find your car's DBC file
ls opendbc/opendbc/dbc/ | grep -i toyota
ls opendbc/opendbc/dbc/ | grep -i honda

# Copy to dbc/ folder in this project
cp opendbc/opendbc/dbc/toyota_nodsu_pt_generated.dbc ./dbc/
```

Then drag the `.dbc` file into the **DBC Decoder** tab of the web app.

### Supported Signal Types

- **Standard OBD2 PIDs** (Mode 01): Built-in — RPM, speed, TPS, ECT, MAF, MAP, fuel trims, O2 sensors, ignition advance
- **Manufacturer-specific CAN** (via DBC): Engine maps, transmission data, ADAS data, EV battery data
- **CANFD** frames: Logged and streamed; DBC decoding uses the first 8 data bytes

---

## Map Reconstruction Methodology

The web app reconstructs factory maps by correlating live sensor readings:

### Maps Being Built

| Map | X-Axis | Y-Axis | What It Tells You |
|-----|--------|--------|-------------------|
| Throttle Map | RPM | TPS % | Throttle body characterization |
| Fuel Trim Map | RPM | STFT/LTFT % | How much the ECU is correcting fueling |
| Ignition Map | RPM | Ignition Advance ° | Spark timing at different RPM |
| MAF Map | RPM | MAF g/s | Airflow at different RPM (volumetric efficiency proxy) |
| MAP Pressure | RPM | Intake MAP kPa | Manifold vacuum/boost curve |
| Speed/RPM | RPM | Speed km/h | Gear ratio estimation |

### Recommended Drive Cycle

To capture representative maps:

1. Cold start and idle warmup (captures idle fuel trims)
2. Gentle acceleration runs 1000–6000 RPM in each gear
3. Cruise at steady speeds (30, 60, 100 km/h)
4. Deceleration / overrun (captures decel fuel cut)
5. Wide Open Throttle run (optional — captures WOT fueling)

Export the map data JSON after the drive for offline analysis.

---

## Binary Log Format (.can)

The binary log is optimized for post-processing. Each file starts with an 8-byte magic header followed by 24-byte records:

```
Magic: "OBD2CAN\0" (8 bytes)

Per-frame record (24 bytes):
  Offset  Size  Field
  0       4     timestamp_ms (uint32, ms since session start)
  4       4     id (uint32, CAN arbitration ID)
  8       1     dlc (data length code)
  9       1     flags (bit0=FD, bit1=BRS, bit2=Extended ID)
  10      2     padding
  12      8     data (first 8 bytes; FD frames truncated to 8)
```

Parse with Python:
```python
import struct

with open('session.can', 'rb') as f:
    magic = f.read(8)
    assert magic == b'OBD2CAN\x00'
    while True:
        rec = f.read(24)
        if len(rec) < 24: break
        ts, id_, dlc, flags, _, _, *data = struct.unpack('<IIBB2B8B', rec)
        print(f"t={ts}ms id=0x{id_:X} dlc={dlc} data={bytes(data[:dlc]).hex()}")
```

---

## Roadmap

- [ ] OBD2 PID poller (auto-query Mode 01 PIDs at configurable rate)
- [ ] 3D surface map visualization (RPM × Load → value)
- [ ] Compare two log sessions side-by-side
- [ ] Export maps to CSV / MegaTune format
- [ ] Automatic gear detection from speed/RPM ratio
- [ ] CANFD full 64-byte data logging
- [ ] Wi-Fi streaming (with ESP32 co-processor or Feather variant)
- [ ] opendbc fingerprinting (auto-identify car model from CAN traffic)

---

## Credits & References

- [ACANFD_FeatherM4CAN](https://github.com/pierremolinaro/acanfd-feather-m4-can) — Pierre Molinaro — CAN driver
- [opendbc](https://github.com/commaai/opendbc) — comma.ai — DBC database & car CAN decoding
- [Adafruit Feather M4 CAN](https://learn.adafruit.com/adafruit-feather-m4-can-express) — Hardware documentation
- [Betaflight Configurator](https://github.com/betaflight/betaflight-configurator) — UI inspiration

---

## License

MIT License — see [LICENSE](LICENSE)
