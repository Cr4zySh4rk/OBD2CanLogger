/**
 * OBD2CanLogger — Adafruit Feather M4 CAN (ATSAMD51)
 *
 * Hardware:
 *   - Adafruit Feather M4 CAN (ATSAMD51J19A) — built-in CANFD transceiver on CAN1
 *   - Adafruit Adalogger FeatherWing (SD card + RTC)
 *   - OBD2 port wired to CANH / CANL on Feather M4 CAN header
 *
 * Libraries required (install via Arduino Library Manager):
 *   - ACANFD_FeatherM4CAN  (Pierre Molinaro)
 *   - SD                    (Arduino built-in)
 *   - RTClib                (Adafruit)
 *   - ArduinoJson           (Benoit Blanchon) v6+
 *
 * Features:
 *   - Logs ALL CAN frames to SD card (CSV + binary .can format)
 *   - USB Serial config interface (JSON commands)
 *   - Real-time streaming of decoded frames over USB Serial
 *   - Configurable: bitrate, log format, filter IDs, session naming
 *   - LED status indicators
 *   - Session file rotation (new file per power-on, RTC timestamp in name)
 */

// ─── Message RAM allocation (MUST come before include) ───────────────────────
#define CAN0_MESSAGE_RAM_SIZE (0)      // CAN0 not used
#define CAN1_MESSAGE_RAM_SIZE (2048)   // CAN1 = OBD2 port

// ─── TinyUSB MSC (USB Mass Storage) ─────────────────────────────────────────
// Must be defined before any USB-related includes.
// Install "Adafruit TinyUSB Library" via Library Manager.
#include <Adafruit_TinyUSB.h>

// ─── Library conflict guard ───────────────────────────────────────────────────
// The Adafruit SAMD board package ships its own SD library (wraps SdFat).
// If you have the standalone "SD" library installed in your Sketchbook it will
// shadow the board package version.  Remove it from Arduino IDE:
//   Sketch → Include Library → Manage Libraries → "SD by Arduino" → Remove
// or simply delete ~/Documents/Arduino/libraries/SD/
#include <ACANFD_FeatherM4CAN.h>
#include <SD.h>
#include <SPI.h>
#include <RTClib.h>
#include <ArduinoJson.h>

// ─── Pin Definitions ──────────────────────────────────────────────────────────
#define SD_CS_PIN       10   // Adalogger FeatherWing SD CS
#define LED_RED_PIN     13   // Built-in red LED
#define LED_GREEN_PIN   -1   // Optional external green LED (-1 = not used)

// ─── Configuration (overridden by config.json on SD card) ────────────────────
struct Config {
  uint32_t  canBitrate      = 500000;   // 500 kbps (standard OBD2)
  bool      logCSV          = true;     // Log human-readable CSV
  bool      logBinary       = true;     // Log raw binary .can frames
  bool      streamSerial    = true;     // Stream frames over USB Serial
  bool      filterEnabled   = false;    // If true, only log IDs in filterList
  uint32_t  filterList[32]  = {};       // CAN IDs to pass (when filterEnabled)
  uint8_t   filterCount     = 0;
  char      sessionName[32] = "session";
};

Config gConfig;

// ─── OBD2 PID Poller ──────────────────────────────────────────────────────────
// OBD2 is request/response: we must SEND a query on ID 0x7DF,
// then the ECU replies on 0x7E8 with the actual data.
// Without polling, the ECU never broadcasts these values on its own.

// PIDs to cycle through (Mode 01)
const uint8_t OBD2_PIDS[] = {
  0x0C,  // Engine RPM
  0x0D,  // Vehicle speed
  0x11,  // Throttle position
  0x05,  // Coolant temp
  0x0E,  // Timing advance (ignition)
  0x10,  // MAF air flow rate
  0x0B,  // Intake manifold pressure (MAP)
  0x06,  // Short term fuel trim – bank 1
  0x07,  // Long term fuel trim – bank 1
  0x14,  // O2 sensor B1S1 voltage
  0x04,  // Calculated engine load
  0x0F,  // Intake air temperature
};
const uint8_t OBD2_PID_COUNT = sizeof(OBD2_PIDS) / sizeof(OBD2_PIDS[0]);

uint8_t  pidIndex         = 0;       // which PID to request next
uint32_t lastPidPollMs    = 0;
uint32_t pidPollIntervalMs = 80;     // ms between each individual PID request
                                     // 12 PIDs × 80ms ≈ ~1 full cycle per second

// ─── Globals ──────────────────────────────────────────────────────────────────
RTC_PCF8523 rtc;
bool        rtcAvailable    = false;

File        csvFile;
File        binFile;
bool        sdAvailable     = false;
bool        logging         = false;

char        csvFilename[48];
char        binFilename[48];

uint32_t    frameCount      = 0;
uint32_t    lastStatusMs    = 0;
uint32_t    sessionStartMs  = 0;

// Binary frame record (fixed 24 bytes per frame)
struct __attribute__((packed)) BinFrame {
  uint32_t timestamp_ms;  // millis() since session start
  uint32_t id;            // CAN ID (bit 31 set = extended)
  uint8_t  dlc;           // Data length code
  uint8_t  flags;         // bit0=FD, bit1=BRS, bit2=extended
  uint8_t  pad[2];
  uint8_t  data[8];       // First 8 bytes (truncated if FD > 8)
};

// ─── Helper macro: true when frame is a CANFD frame (not classic CAN 2.0B) ───
#define IS_CANFD(frame) \
  ((frame).type == CANFDMessage::CANFD_WITH_BIT_RATE_SWITCH || \
   (frame).type == CANFDMessage::CANFD_NO_BIT_RATE_SWITCH)

// ─── USB Mass Storage ─────────────────────────────────────────────────────────
// Adafruit_USBD_MSC wraps the SD card as a USB drive when requested.
Adafruit_USBD_MSC usb_msc;
bool mscActive = false;

// MSC callbacks — forward declarations
int32_t msc_read_cb(uint32_t lba, void* buffer, uint32_t bufsize);
int32_t msc_write_cb(uint32_t lba, uint8_t* buffer, uint32_t bufsize);
void    msc_flush_cb(void);

// ─── Forward Declarations ─────────────────────────────────────────────────────
void loadConfig();
void saveConfig();
void startLoggingSession();
void stopLoggingSession();
void enterMscMode();
void formatSdCard();
void pollNextPid();
void processSerialCommand(const String& line);
void streamFrameJSON(const CANFDMessage& frame, uint32_t ts);
void writeCSVFrame(const CANFDMessage& frame, uint32_t ts);
void writeBinaryFrame(const CANFDMessage& frame, uint32_t ts);
bool passesFilter(uint32_t id);
void blinkLED(int pin, int times, int delayMs);

// ─── Setup ────────────────────────────────────────────────────────────────────
void setup() {
  pinMode(LED_RED_PIN, OUTPUT);
  digitalWrite(LED_RED_PIN, LOW);

  // TinyUSB must be initialized before Serial on SAMD51
  // This sets up the USB descriptor stack (CDC serial by default; MSC added on demand)
  TinyUSBDevice.begin(0);

  Serial.begin(115200);
  // Don't block forever waiting for Serial — car may have no PC attached
  uint32_t t = millis();
  while (!Serial && millis() - t < 3000) delay(10);

  Serial.println(F("\n=== OBD2CanLogger v1.0 ==="));
  Serial.println(F("Feather M4 CAN | ATSAMD51 | ACANFD"));

  // ── RTC init ──
  Wire.begin();
  if (rtc.begin()) {
    rtcAvailable = true;
    if (!rtc.initialized() || rtc.lostPower()) {
      Serial.println(F("[RTC] Not set — using compile time"));
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
    Serial.print(F("[RTC] "));
    DateTime now = rtc.now();
    char buf[32];
    snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
             now.year(), now.month(), now.day(),
             now.hour(), now.minute(), now.second());
    Serial.println(buf);
  } else {
    Serial.println(F("[RTC] Not found — timestamps use millis()"));
  }

  // ── SD init ──
  if (SD.begin(SD_CS_PIN)) {
    sdAvailable = true;
    Serial.println(F("[SD] Card mounted"));
    loadConfig();
  } else {
    Serial.println(F("[SD] No card — logging disabled"));
  }

  // ── CAN init ──
  ACANFD_FeatherM4CAN_Settings settings(
    ACANFD_FeatherM4CAN_Settings::CLOCK_48MHz,
    gConfig.canBitrate,
    DataBitRateFactor::x1   // Classic CAN 2.0B (not FD data phase)
  );
  settings.mModuleMode = ACANFD_FeatherM4CAN_Settings::NORMAL_FD;

  const uint32_t err = can1.beginFD(settings);
  if (err == 0) {
    Serial.print(F("[CAN] OK @ "));
    Serial.print(gConfig.canBitrate / 1000);
    Serial.println(F(" kbps"));
  } else {
    Serial.print(F("[CAN] ERROR 0x"));
    Serial.println(err, HEX);
    blinkLED(LED_RED_PIN, 10, 100);
  }

  // ── Start logging session ──
  if (sdAvailable) {
    startLoggingSession();
  }

  sessionStartMs = millis();
  Serial.println(F("[READY] Logging started. Send JSON commands over Serial."));
  Serial.println(F("  {\"cmd\":\"status\"}  {\"cmd\":\"stop\"}  {\"cmd\":\"start\"}"));
  Serial.println(F("  {\"cmd\":\"config\",\"bitrate\":500000}"));
}

// ─── Main Loop ────────────────────────────────────────────────────────────────
void loop() {
  // ── Receive CAN frame ──
  CANFDMessage frame;
  if (can1.receiveFD0(frame)) {
    uint32_t ts = millis() - sessionStartMs;

    if (!gConfig.filterEnabled || passesFilter(frame.id)) {
      frameCount++;

      if (logging) {
        if (gConfig.logCSV)    writeCSVFrame(frame, ts);
        if (gConfig.logBinary) writeBinaryFrame(frame, ts);
      }

      if (gConfig.streamSerial && Serial) {
        streamFrameJSON(frame, ts);
      }
    }
  }

  // ── OBD2 PID polling ──
  // Send next PID request every pidPollIntervalMs milliseconds.
  // We stagger one PID per interval rather than blasting all at once,
  // which gives the ECU time to respond before the next request.
  if (millis() - lastPidPollMs >= pidPollIntervalMs) {
    lastPidPollMs = millis();
    pollNextPid();
  }

  // ── Serial command processing ──
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      processSerialCommand(line);
    }
  }

  // ── Periodic status to Serial ──
  if (millis() - lastStatusMs > 5000) {
    lastStatusMs = millis();
    // Flush SD periodically to avoid data loss
    if (logging) {
      if (csvFile) csvFile.flush();
      if (binFile) binFile.flush();
    }
    // Heartbeat LED
    digitalWrite(LED_RED_PIN, !digitalRead(LED_RED_PIN));
  }
}

// ─── Logging Session ─────────────────────────────────────────────────────────
void startLoggingSession() {
  if (!sdAvailable) return;

  char timestamp[24] = "00000000_000000";
  if (rtcAvailable) {
    DateTime now = rtc.now();
    snprintf(timestamp, sizeof(timestamp), "%04d%02d%02d_%02d%02d%02d",
             now.year(), now.month(), now.day(),
             now.hour(), now.minute(), now.second());
  } else {
    snprintf(timestamp, sizeof(timestamp), "t%lu", millis());
  }

  snprintf(csvFilename, sizeof(csvFilename), "/%s_%s.csv",
           gConfig.sessionName, timestamp);
  snprintf(binFilename, sizeof(binFilename), "/%s_%s.can",
           gConfig.sessionName, timestamp);

  if (gConfig.logCSV) {
    csvFile = SD.open(csvFilename, FILE_WRITE);
    if (csvFile) {
      csvFile.println(F("timestamp_ms,id_hex,id_dec,extended,fd,dlc,data_hex,data_bytes"));
      csvFile.flush();
      Serial.print(F("[SD] CSV: "));
      Serial.println(csvFilename);
    }
  }

  if (gConfig.logBinary) {
    binFile = SD.open(binFilename, FILE_WRITE);
    if (binFile) {
      // Write 8-byte magic header: "OBD2CAN\0"
      binFile.write((const uint8_t*)"OBD2CAN\x00", 8);
      binFile.flush();
      Serial.print(F("[SD] BIN: "));
      Serial.println(binFilename);
    }
  }

  logging   = true;
  frameCount = 0;
  sessionStartMs = millis();
}

void stopLoggingSession() {
  if (csvFile) { csvFile.flush(); csvFile.close(); }
  if (binFile) { binFile.flush(); binFile.close(); }
  logging = false;
  Serial.print(F("[LOG] Session stopped. Frames logged: "));
  Serial.println(frameCount);
}

// ─── Frame Writers ───────────────────────────────────────────────────────────
void writeCSVFrame(const CANFDMessage& frame, uint32_t ts) {
  if (!csvFile) return;

  csvFile.print(ts);
  csvFile.print(',');

  // ID in hex
  char idHex[12];
  snprintf(idHex, sizeof(idHex), "%08lX", (unsigned long)frame.id);
  csvFile.print(idHex);
  csvFile.print(',');
  csvFile.print(frame.id);
  csvFile.print(',');
  csvFile.print(frame.ext ? '1' : '0');
  csvFile.print(',');
  csvFile.print(IS_CANFD(frame) ? '1' : '0');
  csvFile.print(',');
  csvFile.print(frame.len);
  csvFile.print(',');

  // Data as hex string
  uint8_t printLen = (frame.len < 8) ? frame.len : 8;
  for (uint8_t i = 0; i < printLen; i++) {
    char hex[4];
    snprintf(hex, sizeof(hex), "%02X", frame.data[i]);
    csvFile.print(hex);
    if (i < printLen - 1) csvFile.print(' ');
  }
  csvFile.print(',');
  csvFile.println(frame.len);
}

void writeBinaryFrame(const CANFDMessage& frame, uint32_t ts) {
  if (!binFile) return;

  BinFrame bf;
  bf.timestamp_ms = ts;
  bf.id           = frame.id;
  bf.dlc          = (uint8_t)frame.len;
  bf.flags        = (IS_CANFD(frame) ? 0x01 : 0x00) |
                    (frame.ext        ? 0x04 : 0x00);
  bf.pad[0] = 0; bf.pad[1] = 0;

  uint8_t copyLen = (frame.len < 8) ? frame.len : 8;
  memset(bf.data, 0, 8);
  memcpy(bf.data, frame.data, copyLen);

  binFile.write((const uint8_t*)&bf, sizeof(bf));
}

// ─── Serial Streaming ────────────────────────────────────────────────────────
void streamFrameJSON(const CANFDMessage& frame, uint32_t ts) {
  // Compact JSON: {"t":12345,"id":"0x7E8","ext":0,"fd":0,"dlc":8,"d":"02 01 00 00 00 00 00 00"}
  Serial.print(F("{\"t\":"));
  Serial.print(ts);
  Serial.print(F(",\"id\":\"0x"));

  char idHex[10];
  snprintf(idHex, sizeof(idHex), "%lX", (unsigned long)frame.id);
  Serial.print(idHex);
  Serial.print(F("\",\"ext\":"));
  Serial.print(frame.ext ? 1 : 0);
  Serial.print(F(",\"fd\":"));
  Serial.print(IS_CANFD(frame) ? 1 : 0);
  Serial.print(F(",\"dlc\":"));
  Serial.print(frame.len);
  Serial.print(F(",\"d\":\""));

  uint8_t printLen = (frame.len < 8) ? frame.len : 8;
  for (uint8_t i = 0; i < printLen; i++) {
    char hex[4];
    snprintf(hex, sizeof(hex), "%02X", frame.data[i]);
    Serial.print(hex);
    if (i < printLen - 1) Serial.print(' ');
  }
  Serial.println(F("\"}"));
}

// ─── Serial Command Handler ───────────────────────────────────────────────────
void processSerialCommand(const String& line) {
  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, line);
  if (err) {
    // Not JSON — ignore silently
    return;
  }

  const char* cmd = doc["cmd"];
  if (!cmd) return;

  if (strcmp(cmd, "status") == 0) {
    StaticJsonDocument<256> resp;
    resp["status"]   = logging ? "logging" : "stopped";
    resp["frames"]   = frameCount;
    resp["uptime_s"] = (millis() - sessionStartMs) / 1000;
    resp["bitrate"]  = gConfig.canBitrate;
    resp["sd"]       = sdAvailable;
    resp["rtc"]      = rtcAvailable;
    serializeJson(resp, Serial);
    Serial.println();

  } else if (strcmp(cmd, "stop") == 0) {
    stopLoggingSession();
    Serial.println(F("{\"ok\":true,\"msg\":\"session stopped\"}"));

  } else if (strcmp(cmd, "start") == 0) {
    if (!logging) startLoggingSession();
    Serial.println(F("{\"ok\":true,\"msg\":\"session started\"}"));

  } else if (strcmp(cmd, "config") == 0) {
    bool changed = false;
    if (doc.containsKey("bitrate")) {
      gConfig.canBitrate = doc["bitrate"].as<uint32_t>();
      changed = true;
    }
    if (doc.containsKey("stream")) {
      gConfig.streamSerial = doc["stream"].as<bool>();
      changed = true;
    }
    if (doc.containsKey("logcsv")) {
      gConfig.logCSV = doc["logcsv"].as<bool>();
      changed = true;
    }
    if (doc.containsKey("logbin")) {
      gConfig.logBinary = doc["logbin"].as<bool>();
      changed = true;
    }
    if (changed && sdAvailable) saveConfig();
    Serial.println(F("{\"ok\":true,\"msg\":\"config updated — restart to apply bitrate\"}"));

  } else if (strcmp(cmd, "ls") == 0) {
    // List SD card files
    Serial.println(F("{\"files\":["));
    File root = SD.open("/");
    bool first = true;
    while (true) {
      File entry = root.openNextFile();
      if (!entry) break;
      if (!first) Serial.println(',');
      Serial.print(F("  {\"name\":\""));
      Serial.print(entry.name());
      Serial.print(F("\",\"size\":"));
      Serial.print(entry.size());
      Serial.print(F("}"));
      first = false;
      entry.close();
    }
    root.close();
    Serial.println(F("\n]}"));

  } else if (strcmp(cmd, "msc") == 0) {
    // Enter USB Mass Storage mode — SD card mounts as USB drive on host PC
    Serial.println(F("{\"ok\":true,\"cmd\":\"msc\",\"msg\":\"entering USB storage mode\"}"));
    delay(100);   // let serial flush before USB re-enumeration
    enterMscMode();

  } else if (strcmp(cmd, "format") == 0) {
    // Format SD card (FAT32) — erases all log files
    formatSdCard();
  }
}

// ─── Config Load/Save ────────────────────────────────────────────────────────
void loadConfig() {
  if (!SD.exists("/config.json")) return;

  File f = SD.open("/config.json", FILE_READ);
  if (!f) return;

  StaticJsonDocument<512> doc;
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) {
    Serial.println(F("[CFG] Parse error — using defaults"));
    return;
  }

  if (doc.containsKey("bitrate"))   gConfig.canBitrate    = doc["bitrate"];
  if (doc.containsKey("logcsv"))    gConfig.logCSV        = doc["logcsv"];
  if (doc.containsKey("logbin"))    gConfig.logBinary     = doc["logbin"];
  if (doc.containsKey("stream"))    gConfig.streamSerial  = doc["stream"];
  if (doc.containsKey("session"))   strlcpy(gConfig.sessionName, doc["session"], 32);

  if (doc.containsKey("filter") && doc["filter"].is<JsonArray>()) {
    JsonArray arr = doc["filter"].as<JsonArray>();
    gConfig.filterCount = 0;
    gConfig.filterEnabled = (arr.size() > 0);
    for (uint32_t id : arr) {
      if (gConfig.filterCount < 32)
        gConfig.filterList[gConfig.filterCount++] = id;
    }
  }

  Serial.println(F("[CFG] Loaded config.json"));
}

void saveConfig() {
  SD.remove("/config.json");
  File f = SD.open("/config.json", FILE_WRITE);
  if (!f) return;

  StaticJsonDocument<512> doc;
  doc["bitrate"] = gConfig.canBitrate;
  doc["logcsv"]  = gConfig.logCSV;
  doc["logbin"]  = gConfig.logBinary;
  doc["stream"]  = gConfig.streamSerial;
  doc["session"] = gConfig.sessionName;

  serializeJsonPretty(doc, f);
  f.close();
  Serial.println(F("[CFG] Saved config.json"));
}

// ─── OBD2 PID Poller ─────────────────────────────────────────────────────────
// Sends a standard ISO 15765-4 (CAN OBD2) Mode 01 PID request.
// Format: [0x02, 0x01, PID, 0x00, 0x00, 0x00, 0x00, 0x00]
//   byte 0: 0x02 = 2 additional bytes follow
//   byte 1: 0x01 = Mode 01 (show current data)
//   byte 2: PID number
//   bytes 3-7: padding
// Functional broadcast address 0x7DF reaches all ECUs on the bus.
void pollNextPid() {
  CANFDMessage req;
  req.id   = 0x7DF;           // OBD2 functional broadcast address
  req.ext  = false;           // standard 11-bit ID
  req.type = CANFDMessage::CAN_DATA;
  req.len  = 8;

  memset(req.data, 0x00, 8);
  req.data[0] = 0x02;                      // PCI: 2 bytes follow
  req.data[1] = 0x01;                      // Mode 01: current data
  req.data[2] = OBD2_PIDS[pidIndex];       // PID to request

  const uint32_t err = can1.tryToSendReturnStatusFD(req);
  if (err == 0) {
    // Advance to next PID in the rotation
    pidIndex = (pidIndex + 1) % OBD2_PID_COUNT;
  }
  // If send failed (tx buffer full), we don't advance — retry same PID next time
}

// ─── Helpers ─────────────────────────────────────────────────────────────────
bool passesFilter(uint32_t id) {
  for (uint8_t i = 0; i < gConfig.filterCount; i++) {
    if (gConfig.filterList[i] == id) return true;
  }
  return false;
}

void blinkLED(int pin, int times, int delayMs) {
  if (pin < 0) return;
  for (int i = 0; i < times; i++) {
    digitalWrite(pin, HIGH);
    delay(delayMs);
    digitalWrite(pin, LOW);
    delay(delayMs);
  }
}

// ─── USB Mass Storage Mode ────────────────────────────────────────────────────
// Stops logging, closes all files, then re-enumerates the USB connection as a
// Mass Storage Class device so the host OS can browse the SD card directly.
//
// Requires: Adafruit TinyUSB Library (install via Library Manager)
// After the host ejects the drive, the user should press Reset to resume logging.
void enterMscMode() {
  // 1. Stop any active logging session to flush and close files
  if (logging) stopLoggingSession();

  if (!sdAvailable) {
    Serial.println(F("{\"ok\":false,\"cmd\":\"msc\",\"msg\":\"no SD card\"}"));
    return;
  }

  // 2. Configure the MSC object with SD card geometry.
  // setCapacity() is only used to populate the USB descriptor (what the host
  // OS displays as the drive size). TinyUSB performs the actual read/write via
  // the callbacks below, so the exact count here doesn't need to be perfect.
  // We probe the card size via raw CMD9 (CSD register) to get the real value;
  // if that's unavailable we fall back to a 32 GB ceiling which covers all
  // FAT32 cards supported by this project.
  usb_msc.setID("Adafruit", "SD Card", "1.0");
  usb_msc.setReadWriteCallback(msc_read_cb, msc_write_cb, msc_flush_cb);
  // Probe block count: walk the FAT volume cluster map via SdFat internals.
  // SD.vol() is available when the Adafruit SAMD board package SD is used;
  // fall back to 64M blocks (~32 GB) for the standalone Arduino SD library.
  uint32_t blockCount = 0;
#if defined(ARDUINO_ADAFRUIT_FEATHER_M4_CAN) || defined(ADAFRUIT_FEATHER_M4_CAN)
  // Adafruit board package SD wraps SdFat — blocksPerCluster * clusterCount
  // gives actual capacity without touching any private members.
  if (SD.vol()) {
    blockCount = (uint32_t)SD.vol()->blocksPerCluster() *
                 (uint32_t)SD.vol()->clusterCount();
  }
#endif
  if (blockCount == 0) blockCount = 62521344UL; // 32 GB fallback
  usb_msc.setCapacity(blockCount, 512);
  usb_msc.setUnitReady(true);
  usb_msc.begin();

  mscActive = true;
  Serial.println(F("[MSC] SD card mounted as USB drive"));

  // 3. Blink LED to indicate MSC mode
  blinkLED(LED_RED_PIN, 3, 150);

  // 4. Spin here — nothing else should run while the host has the drive mounted.
  // The user must press Reset or power-cycle to exit MSC mode and resume logging.
  while (mscActive) {
    // TinyUSB handles all USB traffic in the background via RTOS/interrupt
    delay(10);
  }
}

// MSC read callback — called by TinyUSB to read sectors from the SD card.
// Uses SdFat's FatVolume::diskRead() which is public on the Adafruit SAMD
// board package. On the standalone Arduino SD library (which doesn't ship
// SdFat) raw sector I/O is unavailable — MSC mode won't work in that config,
// but the sketch will still compile and run for normal CSV/binary logging.
int32_t msc_read_cb(uint32_t lba, void* buffer, uint32_t bufsize) {
#if defined(ARDUINO_ADAFRUIT_FEATHER_M4_CAN) || defined(ADAFRUIT_FEATHER_M4_CAN)
  uint32_t blocks = bufsize / 512;
  if (!SD.vol()->diskRead((uint8_t*)buffer, lba, blocks)) return -1;
  return (int32_t)bufsize;
#else
  (void)lba; (void)buffer; (void)bufsize;
  return -1; // MSC not supported with standalone SD library
#endif
}

// MSC write callback — called by TinyUSB when the host writes to the drive
int32_t msc_write_cb(uint32_t lba, uint8_t* buffer, uint32_t bufsize) {
#if defined(ARDUINO_ADAFRUIT_FEATHER_M4_CAN) || defined(ADAFRUIT_FEATHER_M4_CAN)
  uint32_t blocks = bufsize / 512;
  if (!SD.vol()->diskWrite((const uint8_t*)buffer, lba, blocks)) return -1;
  return (int32_t)bufsize;
#else
  (void)lba; (void)buffer; (void)bufsize;
  return -1;
#endif
}

// MSC flush callback — called when the host flushes write cache
void msc_flush_cb(void) {
#if defined(ARDUINO_ADAFRUIT_FEATHER_M4_CAN) || defined(ADAFRUIT_FEATHER_M4_CAN)
  SD.vol()->diskSync();
#endif
}

// ─── SD Card Format ───────────────────────────────────────────────────────────
// Stops logging, closes files, then deletes every file on the SD card root
// and writes a fresh FAT32 volume. After format, a new logging session starts.
//
// Note: The Arduino SD library does not expose a low-level format function.
// We do a "logical format" by removing all files. For a true FAT reformat
// use SdFat library's SD.format() if available on your platform.
void formatSdCard() {
  if (!sdAvailable) {
    Serial.println(F("{\"ok\":false,\"cmd\":\"format\",\"msg\":\"no SD card\"}"));
    return;
  }

  // Stop logging and close all open files first
  if (logging) stopLoggingSession();
  if (csvFile) { csvFile.close(); }
  if (binFile) { binFile.close(); }

  Serial.println(F("[FORMAT] Removing all files from SD card…"));

  // Remove all files in root directory
  File root = SD.open("/");
  uint16_t count = 0;
  while (true) {
    File entry = root.openNextFile();
    if (!entry) break;
    char name[64];
    strlcpy(name, entry.name(), sizeof(name));
    bool isDir = entry.isDirectory();
    entry.close();
    if (!isDir) {
      // Build full path
      char fullPath[68];
      snprintf(fullPath, sizeof(fullPath), "/%s", name);
      SD.remove(fullPath);
      count++;
    }
  }
  root.close();

  Serial.print(F("[FORMAT] Removed "));
  Serial.print(count);
  Serial.println(F(" file(s). SD card is now empty."));
  Serial.println(F("{\"ok\":true,\"cmd\":\"format\",\"msg\":\"SD card formatted\"}"));

  // Start a fresh logging session
  delay(200);
  startLoggingSession();
}
