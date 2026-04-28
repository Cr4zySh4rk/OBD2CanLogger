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
 *   - SdFat                (Bill Greiman)  ← use this instead of "SD by Arduino"
 *   - RTClib               (Adafruit)
 *   - ArduinoJson          (Benoit Blanchon) v6+
 *   - Adafruit TinyUSB Library
 *
 * NOTE: Do NOT install "SD by Arduino" from the Library Manager.
 * This sketch uses SdFat directly to avoid library conflicts with TinyUSB.
 *
 * Features:
 *   - Logs ALL CAN frames to SD card (CSV + binary .can format)
 *   - USB Serial config interface (JSON commands)
 *   - Real-time streaming of decoded frames over USB Serial
 *   - USB Mass Storage mode — mount SD card as a USB drive on host PC
 *   - Configurable: bitrate, log format, filter IDs, session naming
 *   - LED status indicators
 *   - Session file rotation (new file per power-on, RTC timestamp in name)
 */

// ─── Message RAM allocation (MUST come before include) ───────────────────────
#define CAN0_MESSAGE_RAM_SIZE (0)      // CAN0 not used
#define CAN1_MESSAGE_RAM_SIZE (2048)   // CAN1 = OBD2 port

// ─── TinyUSB MSC (USB Mass Storage) ──────────────────────────────────────────
// Must come before any other USB or Serial includes on SAMD51.
#include <Adafruit_TinyUSB.h>

// ─── SdFat ────────────────────────────────────────────────────────────────────
// Use SdFat directly — avoids conflict with "SD by Arduino" library.
// SdFat ships inside the Adafruit SAMD board package and also as a standalone
// Library Manager entry "SdFat by Bill Greiman".  Either works.
#include <SdFat.h>

#include <ACANFD_FeatherM4CAN.h>
#include <SPI.h>
#include <RTClib.h>
#include <ArduinoJson.h>

// ─── Pin Definitions ──────────────────────────────────────────────────────────
#define SD_CS_PIN       10   // Adalogger FeatherWing SD CS
#define LED_RED_PIN     13   // Built-in red LED

// ─── SdFat instance ──────────────────────────────────────────────────────────
// SdFat uses its own SPI speed constant.  SD_SCK_MHZ(50) is the max the
// Adalogger reliably supports over the shared SPI bus.
SdFat  sd;
SdFile csvFile;
SdFile binFile;

// ─── Configuration (overridden by config.json on SD card) ────────────────────
struct Config {
  uint32_t  canBitrate      = 500000;
  bool      logCSV          = true;
  bool      logBinary       = true;
  bool      streamSerial    = true;
  bool      filterEnabled   = false;
  uint32_t  filterList[32]  = {};
  uint8_t   filterCount     = 0;
  char      sessionName[32] = "session";
};

Config gConfig;

// ─── OBD2 PID Poller ──────────────────────────────────────────────────────────
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

uint8_t  pidIndex          = 0;
uint32_t lastPidPollMs     = 0;
uint32_t pidPollIntervalMs = 80;

// ─── Globals ──────────────────────────────────────────────────────────────────
RTC_PCF8523 rtc;
bool        rtcAvailable   = false;
bool        sdAvailable    = false;
bool        logging        = false;

char        csvFilename[48];
char        binFilename[48];

uint32_t    frameCount     = 0;
uint32_t    lastStatusMs   = 0;
uint32_t    sessionStartMs = 0;

// Binary frame record (fixed 24 bytes per frame)
struct __attribute__((packed)) BinFrame {
  uint32_t timestamp_ms;
  uint32_t id;
  uint8_t  dlc;
  uint8_t  flags;   // bit0=FD, bit1=BRS, bit2=extended
  uint8_t  pad[2];
  uint8_t  data[8];
};

#define IS_CANFD(frame) \
  ((frame).type == CANFDMessage::CANFD_WITH_BIT_RATE_SWITCH || \
   (frame).type == CANFDMessage::CANFD_NO_BIT_RATE_SWITCH)

// ─── USB Mass Storage ─────────────────────────────────────────────────────────
Adafruit_USBD_MSC usb_msc;
bool mscActive = false;

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

  // TinyUSB must initialise before Serial on SAMD51
  TinyUSBDevice.begin(0);

  Serial.begin(115200);
  uint32_t t = millis();
  while (!Serial && millis() - t < 3000) delay(10);

  Serial.println(F("\n=== OBD2CanLogger v1.1 ==="));
  Serial.println(F("Feather M4 CAN | ATSAMD51 | ACANFD | SdFat"));

  // ── RTC init ──
  Wire.begin();
  if (rtc.begin()) {
    rtcAvailable = true;
    if (!rtc.initialized() || rtc.lostPower()) {
      Serial.println(F("[RTC] Not set — using compile time"));
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
    DateTime now = rtc.now();
    char buf[32];
    snprintf(buf, sizeof(buf), "[RTC] %04d-%02d-%02d %02d:%02d:%02d",
             now.year(), now.month(), now.day(),
             now.hour(), now.minute(), now.second());
    Serial.println(buf);
  } else {
    Serial.println(F("[RTC] Not found — timestamps use millis()"));
  }

  // ── SD init (SdFat) ──
  if (sd.begin(SD_CS_PIN, SD_SCK_MHZ(50))) {
    sdAvailable = true;
    Serial.println(F("[SD] Card mounted (SdFat)"));
    loadConfig();
  } else {
    Serial.println(F("[SD] No card — logging disabled"));
  }

  // ── CAN init ──
  ACANFD_FeatherM4CAN_Settings settings(
    ACANFD_FeatherM4CAN_Settings::CLOCK_48MHz,
    gConfig.canBitrate,
    DataBitRateFactor::x1
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

  if (sdAvailable) startLoggingSession();

  sessionStartMs = millis();
  Serial.println(F("[READY] Send JSON commands over Serial."));
  Serial.println(F("  {\"cmd\":\"status\"}  {\"cmd\":\"stop\"}  {\"cmd\":\"start\"}"));
  Serial.println(F("  {\"cmd\":\"msc\"}  {\"cmd\":\"format\"}"));
}

// ─── Main Loop ────────────────────────────────────────────────────────────────
void loop() {
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

  if (millis() - lastPidPollMs >= pidPollIntervalMs) {
    lastPidPollMs = millis();
    pollNextPid();
  }

  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) processSerialCommand(line);
  }

  if (millis() - lastStatusMs > 5000) {
    lastStatusMs = millis();
    if (logging) {
      csvFile.sync();
      binFile.sync();
    }
    digitalWrite(LED_RED_PIN, !digitalRead(LED_RED_PIN));
  }
}

// ─── Logging Session ──────────────────────────────────────────────────────────
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

  snprintf(csvFilename, sizeof(csvFilename), "%s_%s.csv",
           gConfig.sessionName, timestamp);
  snprintf(binFilename, sizeof(binFilename), "%s_%s.can",
           gConfig.sessionName, timestamp);

  if (gConfig.logCSV) {
    if (csvFile.open(csvFilename, O_WRONLY | O_CREAT | O_TRUNC)) {
      csvFile.println(F("timestamp_ms,id_hex,id_dec,extended,fd,dlc,data_hex,data_bytes"));
      csvFile.sync();
      Serial.print(F("[SD] CSV: ")); Serial.println(csvFilename);
    }
  }

  if (gConfig.logBinary) {
    if (binFile.open(binFilename, O_WRONLY | O_CREAT | O_TRUNC)) {
      binFile.write((const uint8_t*)"OBD2CAN\x00", 8);
      binFile.sync();
      Serial.print(F("[SD] BIN: ")); Serial.println(binFilename);
    }
  }

  logging        = true;
  frameCount     = 0;
  sessionStartMs = millis();
}

void stopLoggingSession() {
  csvFile.sync(); csvFile.close();
  binFile.sync(); binFile.close();
  logging = false;
  Serial.print(F("[LOG] Session stopped. Frames: "));
  Serial.println(frameCount);
}

// ─── Frame Writers ────────────────────────────────────────────────────────────
void writeCSVFrame(const CANFDMessage& frame, uint32_t ts) {
  if (!csvFile.isOpen()) return;

  char line[80];
  char idHex[12];
  snprintf(idHex, sizeof(idHex), "%08lX", (unsigned long)frame.id);

  csvFile.print(ts);        csvFile.print(',');
  csvFile.print(idHex);     csvFile.print(',');
  csvFile.print(frame.id);  csvFile.print(',');
  csvFile.print(frame.ext ? '1' : '0'); csvFile.print(',');
  csvFile.print(IS_CANFD(frame) ? '1' : '0'); csvFile.print(',');
  csvFile.print(frame.len); csvFile.print(',');

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
  if (!binFile.isOpen()) return;

  BinFrame bf;
  bf.timestamp_ms = ts;
  bf.id           = frame.id;
  bf.dlc          = (uint8_t)frame.len;
  bf.flags        = (IS_CANFD(frame) ? 0x01 : 0x00) | (frame.ext ? 0x04 : 0x00);
  bf.pad[0] = 0; bf.pad[1] = 0;
  uint8_t copyLen = (frame.len < 8) ? frame.len : 8;
  memset(bf.data, 0, 8);
  memcpy(bf.data, frame.data, copyLen);

  binFile.write((const uint8_t*)&bf, sizeof(bf));
}

// ─── Serial Streaming ─────────────────────────────────────────────────────────
void streamFrameJSON(const CANFDMessage& frame, uint32_t ts) {
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

// ─── Serial Command Handler ────────────────────────────────────────────────────
void processSerialCommand(const String& line) {
  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, line)) return;

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
    if (doc.containsKey("bitrate")) { gConfig.canBitrate   = doc["bitrate"].as<uint32_t>(); changed = true; }
    if (doc.containsKey("stream"))  { gConfig.streamSerial = doc["stream"].as<bool>();  changed = true; }
    if (doc.containsKey("logcsv"))  { gConfig.logCSV       = doc["logcsv"].as<bool>();  changed = true; }
    if (doc.containsKey("logbin"))  { gConfig.logBinary    = doc["logbin"].as<bool>();  changed = true; }
    if (changed && sdAvailable) saveConfig();
    Serial.println(F("{\"ok\":true,\"msg\":\"config updated\"}"));

  } else if (strcmp(cmd, "ls") == 0) {
    Serial.println(F("{\"files\":["));
    SdFile root, entry;
    root.open("/");
    bool first = true;
    while (entry.openNext(&root, O_RDONLY)) {
      char name[64];
      entry.getName(name, sizeof(name));
      uint32_t sz = entry.fileSize();
      bool isDir  = entry.isDir();
      entry.close();
      if (isDir) continue;
      if (!first) Serial.println(',');
      Serial.print(F("  {\"name\":\""));
      Serial.print(name);
      Serial.print(F("\",\"size\":"));
      Serial.print(sz);
      Serial.print(F("}"));
      first = false;
    }
    root.close();
    Serial.println(F("\n]}"));

  } else if (strcmp(cmd, "msc") == 0) {
    Serial.println(F("{\"ok\":true,\"cmd\":\"msc\",\"msg\":\"entering USB storage mode\"}"));
    delay(100);
    enterMscMode();

  } else if (strcmp(cmd, "format") == 0) {
    formatSdCard();
  }
}

// ─── Config Load/Save ─────────────────────────────────────────────────────────
void loadConfig() {
  SdFile f;
  if (!f.open("config.json", O_RDONLY)) return;

  StaticJsonDocument<512> doc;
  // Read entire file into a buffer
  uint32_t sz = f.fileSize();
  if (sz > 480) { f.close(); return; }
  char buf[481];
  f.read(buf, sz);
  buf[sz] = '\0';
  f.close();

  if (deserializeJson(doc, buf)) {
    Serial.println(F("[CFG] Parse error — using defaults"));
    return;
  }

  if (doc.containsKey("bitrate")) gConfig.canBitrate   = doc["bitrate"];
  if (doc.containsKey("logcsv"))  gConfig.logCSV        = doc["logcsv"];
  if (doc.containsKey("logbin"))  gConfig.logBinary     = doc["logbin"];
  if (doc.containsKey("stream"))  gConfig.streamSerial  = doc["stream"];
  if (doc.containsKey("session")) strlcpy(gConfig.sessionName, doc["session"], 32);

  if (doc.containsKey("filter") && doc["filter"].is<JsonArray>()) {
    JsonArray arr = doc["filter"].as<JsonArray>();
    gConfig.filterCount   = 0;
    gConfig.filterEnabled = (arr.size() > 0);
    for (uint32_t id : arr) {
      if (gConfig.filterCount < 32)
        gConfig.filterList[gConfig.filterCount++] = id;
    }
  }
  Serial.println(F("[CFG] Loaded config.json"));
}

void saveConfig() {
  sd.remove("config.json");
  SdFile f;
  if (!f.open("config.json", O_WRONLY | O_CREAT | O_TRUNC)) return;

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

// ─── OBD2 PID Poller ──────────────────────────────────────────────────────────
void pollNextPid() {
  CANFDMessage req;
  req.id   = 0x7DF;
  req.ext  = false;
  req.type = CANFDMessage::CAN_DATA;
  req.len  = 8;
  memset(req.data, 0x00, 8);
  req.data[0] = 0x02;
  req.data[1] = 0x01;
  req.data[2] = OBD2_PIDS[pidIndex];

  if (can1.tryToSendReturnStatusFD(req) == 0) {
    pidIndex = (pidIndex + 1) % OBD2_PID_COUNT;
  }
}

// ─── Helpers ──────────────────────────────────────────────────────────────────
bool passesFilter(uint32_t id) {
  for (uint8_t i = 0; i < gConfig.filterCount; i++)
    if (gConfig.filterList[i] == id) return true;
  return false;
}

void blinkLED(int pin, int times, int delayMs) {
  if (pin < 0) return;
  for (int i = 0; i < times; i++) {
    digitalWrite(pin, HIGH); delay(delayMs);
    digitalWrite(pin, LOW);  delay(delayMs);
  }
}

// ─── USB Mass Storage Mode ────────────────────────────────────────────────────
// Uses SdFat's SdCard object for raw sector access — fully public API,
// no dependency on the Arduino SD wrapper at all.
void enterMscMode() {
  if (logging) stopLoggingSession();

  if (!sdAvailable) {
    Serial.println(F("{\"ok\":false,\"cmd\":\"msc\",\"msg\":\"no SD card\"}"));
    return;
  }

  usb_msc.setID("Adafruit", "SD Card", "1.0");
  usb_msc.setReadWriteCallback(msc_read_cb, msc_write_cb, msc_flush_cb);

  // sd.card() on SdFat returns SdCard* which is fully public.
  uint32_t blockCount = sd.card()->sectorCount();
  if (blockCount == 0) blockCount = 62521344UL; // 32 GB fallback
  usb_msc.setCapacity(blockCount, 512);
  usb_msc.setUnitReady(true);
  usb_msc.begin();

  mscActive = true;
  Serial.println(F("[MSC] SD card mounted as USB drive. Press Reset to exit."));
  blinkLED(LED_RED_PIN, 3, 150);

  while (mscActive) delay(10);
}

// SdFat SdCard::readSectors / writeSectors / syncDevice are public in all
// versions of SdFat (both the standalone library and the board package copy).
int32_t msc_read_cb(uint32_t lba, void* buffer, uint32_t bufsize) {
  if (!sd.card()->readSectors(lba, (uint8_t*)buffer, bufsize / 512)) return -1;
  return (int32_t)bufsize;
}

int32_t msc_write_cb(uint32_t lba, uint8_t* buffer, uint32_t bufsize) {
  if (!sd.card()->writeSectors(lba, buffer, bufsize / 512)) return -1;
  return (int32_t)bufsize;
}

void msc_flush_cb(void) {
  sd.card()->syncDevice();
}

// ─── SD Card Format ───────────────────────────────────────────────────────────
void formatSdCard() {
  if (!sdAvailable) {
    Serial.println(F("{\"ok\":false,\"cmd\":\"format\",\"msg\":\"no SD card\"}"));
    return;
  }

  if (logging) stopLoggingSession();
  csvFile.close();
  binFile.close();

  Serial.println(F("[FORMAT] Removing all files…"));

  SdFile root, entry;
  root.open("/");
  uint16_t count = 0;
  while (entry.openNext(&root, O_RDONLY)) {
    char name[64];
    entry.getName(name, sizeof(name));
    bool isDir = entry.isDir();
    entry.close();
    if (!isDir) {
      sd.remove(name);
      count++;
    }
  }
  root.close();

  Serial.print(F("[FORMAT] Removed "));
  Serial.print(count);
  Serial.println(F(" file(s)."));
  Serial.println(F("{\"ok\":true,\"cmd\":\"format\",\"msg\":\"SD card formatted\"}"));

  delay(200);
  startLoggingSession();
}
