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

// ─── Forward Declarations ─────────────────────────────────────────────────────
void loadConfig();
void saveConfig();
void startLoggingSession();
void stopLoggingSession();
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
