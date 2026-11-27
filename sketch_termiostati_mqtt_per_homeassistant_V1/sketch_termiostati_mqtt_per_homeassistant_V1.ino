/*
 * PiZaff Termostato MQTT per Home Assistant (firmware ESP8266)
 * Copyright (C) 2026 Pietro Zaffarano
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

/*
 * v2.0.0
 * - Un solo firmware per tutte le stanze: zona, correzione sensore, MQTT e
 *   password OTA si impostano dal portale web (niente più sketch duplicati).
 * - OTA: da Arduino IDE (porta di rete) o caricando il .bin dal portale web.
 * - WiFi: rimosso WiFi.disconnect() dal watchdog (cancellava le credenziali
 *   salvate in flash: era la causa dei termostati che "dimenticavano" il WiFi).
 * - Config su LittleFS con scrittura atomica; credenziali e stato in due file
 *   separati; nessuna scrittura in flash a ogni avvio.
 * - Portale non bloccante: il termostato continua a funzionare mentre è aperto.
 * - Encoder su interrupt, display ridisegnato a intervalli fissi, OLED
 *   attenuato a riposo e con micro-spostamento anti burn-in.
 */

#include <ESP8266WiFi.h>
#include <ArduinoOTA.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <FS.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <math.h>

#define FW_VERSION "2.0.0"

#if ARDUINOJSON_VERSION_MAJOR >= 7
  #define JSON_DOC(name, size) JsonDocument name
#else
  #define JSON_DOC(name, size) DynamicJsonDocument name(size)
#endif

// ===================== PIN =====================
#define PIN_FLASH    0    // GPIO0 (tasto FLASH): tenuto 5 s = reset di fabbrica
#define PIN_ONEWIRE  16   // D0  DS18B20 (DQ)
#define PIN_ENC_CLK  5    // D1
#define PIN_ENC_DT   4    // D2
#define PIN_ENC_SW   13   // D7
#define PIN_SDA      12   // D6
#define PIN_SCL      14   // D5

// ===================== TEMPI / LIMITI =====================
const uint32_t SENSOR_PERIOD_MS   = 2000;
const uint32_t CONV_MS            = 750;      // conversione DS18B20 a 12 bit
const uint8_t  SENSOR_BAD_LIMIT   = 3;        // letture errate consecutive prima di dichiarare guasto
const uint32_t PUBLISH_PERIOD_MS  = 10000;
const uint32_t INFO_PERIOD_MS     = 300000;
const uint32_t MQTT_RETRY_MS      = 5000;
const uint32_t WIFI_KICK_MS       = 60000;    // WiFi giù: ritenta la connessione ogni 60 s
const uint32_t WIFI_REBOOT_MS     = 600000;   // WiFi giù da 10 min: riavvio
const uint32_t STATE_SAVE_DELAY_MS = 10000;   // salva setpoint/modo 10 s dopo l'ultima modifica
const uint32_t RESTART_DELAY_MS   = 3000;
const uint32_t UI_TIMEOUT_MS      = 3000;
const uint32_t SEASON_SHOW_MS     = 2000;
const uint32_t ALERT_MS           = 2500;
const uint32_t DIM_AFTER_MS       = 60000;    // OLED attenuato dopo 1 min senza interazione
const uint32_t SHIFT_PERIOD_MS    = 120000;   // micro-spostamento anti burn-in ogni 2 min
const uint32_t RENDER_PERIOD_MS   = 150;
const uint32_t BTN_DEBOUNCE_MS    = 30;
const uint32_t SEASON_PRESS_MS    = 2000;     // pressione lunga encoder = cambio stagione
const uint32_t PORTAL_PRESS_MS    = 8000;     // pressione 8 s encoder = apre il portale
const uint32_t FACTORY_PRESS_MS   = 5000;     // FLASH tenuto 5 s = reset di fabbrica
const uint16_t PORTAL_TIMEOUT_S   = 300;      // portale aperto a mano: si chiude dopo 5 min

const float HYST   = 0.3f;
const float SP_MIN = 5.0f;
const float SP_MAX = 30.0f;
const float OFFSET_LIMIT = 10.0f;

const char* AP_PASS = "pizaff123";            // password dell'AP del portale

const uint8_t CONTRAST_ON  = 0x8F;
const uint8_t CONTRAST_DIM = 0x05;

// ===================== OLED 128x32 =====================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ===================== CONFIGURAZIONE =====================
struct NetConfig {
  char server[64];
  char port[6];
  char user[64];
  char pass[64];
  char zona[24];
  char offset[8];
  char otaPass[33];
};
NetConfig cfg = { "", "1883", "", "", "", "-2.5", "" };
float tempOffset = -2.5f;

struct ThermoState {
  float setpoint;
  bool  winter;    // true = riscaldamento, false = raffrescamento
  bool  enabled;   // false = modo "off"
};
ThermoState st      = { 21.0f, true, true };
ThermoState stSaved = st;
bool     stateDirty = false;
uint32_t stateChangedAt = 0;

bool fsOk = false;
const char* NET_FILE   = "/net.json";
const char* STATE_FILE = "/state.json";

// ===================== RETE =====================
WiFiClient   net;
PubSubClient mqtt(net);
WiFiManager  wm;

WiFiManagerParameter *pServer, *pPort, *pUser, *pPass, *pZona, *pOffset, *pOta;

bool configured     = false;   // WiFi + server MQTT + zona impostati
bool portalActive   = false;
bool otaStarted     = false;
bool otaRunning     = false;
bool portalOta      = false;   // aggiornamento caricato dal portale web
uint32_t otaSince   = 0;
bool restartPending = false;
uint32_t restartAt  = 0;

char hostName[40];
char apName[24];
char resetReason[32];

char tTemp[64], tSetpoint[64], tSetpointCmd[64], tHeating[64], tAction[64],
     tMode[64], tModeCmd[64], tSeason[64], tSeasonCmd[64], tAvail[64],
     tSensor[64], tInfo[64];

// ===================== SENSORE =====================
OneWire oneWire(PIN_ONEWIRE);
DallasTemperature sensors(&oneWire);
float   tempC = NAN;
uint8_t sensorBad = 0;
bool    sensorError = false;
bool    callForHeat = false;

// ===================== ENCODER (interrupt) =====================
static const int8_t ENC_TAB[16] = {
   0, -1, +1,  0,
  +1,  0,  0, -1,
  -1,  0,  0, +1,
   0, +1, -1,  0
};
volatile int16_t encSteps = 0;
volatile uint8_t encState = 0;
volatile int8_t  encAccum = 0;

void IRAM_ATTR encISR() {
  uint8_t s = (GPIP(PIN_ENC_CLK) << 1) | GPIP(PIN_ENC_DT);
  encState = ((encState << 2) | s) & 0x0F;
  int8_t step = ENC_TAB[encState];
  if (step) {
    encAccum += step;
    if (encAccum >= 4)       { encAccum = 0; encSteps++; }
    else if (encAccum <= -4) { encAccum = 0; encSteps--; }
  }
}

// ===================== UI =====================
enum Screen { SCR_AMBIENT, SCR_SETPOINT, SCR_SEASON, SCR_ALERT };
Screen   screen = SCR_AMBIENT;
uint32_t screenSince = 0;
uint32_t lastActivity = 0;
uint32_t lastRender = 0;
uint8_t  contrastCur = 0;
char alert1[22] = "";
char alert2[22] = "";

const uint8_t ICON_FLAME[8] = {
  0b00011000, 0b00011100, 0b00111110, 0b00111110,
  0b00111110, 0b00011100, 0b00001000, 0b00000000
};
const uint8_t ICON_SNOW[8] = {
  0b00101010, 0b00011100, 0b11111111, 0b00111110,
  0b11111111, 0b00011100, 0b00101010, 0b00000000
};

// ============================================================
//                         UTILITÀ
// ============================================================
bool timeReached(uint32_t t) { return (int32_t)(millis() - t) >= 0; }

bool parseFloatStrict(const char* s, float& out) {
  char buf[16];
  strlcpy(buf, s, sizeof(buf));
  for (char* p = buf; *p; ++p) if (*p == ',') *p = '.';
  char* end = nullptr;
  float v = strtof(buf, &end);
  if (end == buf || !isfinite(v)) return false;
  while (*end == ' ') end++;
  if (*end) return false;
  out = v;
  return true;
}

void sanitizeZone(char* z) {
  for (char* p = z; *p; ++p) {
    char c = *p;
    if (!(isalnum((unsigned char)c) || c == '_' || c == '-')) *p = '_';
  }
}

void applyOffset() {
  float v;
  if (parseFloatStrict(cfg.offset, v) && fabsf(v) <= OFFSET_LIMIT) tempOffset = v;
  else { tempOffset = 0.0f; strlcpy(cfg.offset, "0.0", sizeof(cfg.offset)); }
}

void buildNames() {
  snprintf(apName, sizeof(apName), "ESP_Config-%04X", (unsigned)(ESP.getChipId() & 0xFFFF));
  char z[24];
  size_t j = 0;
  for (size_t i = 0; cfg.zona[i] && j < sizeof(z) - 1; i++) {
    char c = cfg.zona[i];
    z[j++] = isalnum((unsigned char)c) ? (char)tolower((unsigned char)c) : '-';
  }
  z[j] = '\0';
  if (j) snprintf(hostName, sizeof(hostName), "termostato-%s", z);
  else   snprintf(hostName, sizeof(hostName), "termostato-%06x", ESP.getChipId());
}

static void mkTopic(char* buf, size_t n, const char* suffix) {
  snprintf(buf, n, "termostato/%s/%s", cfg.zona, suffix);
}

void buildTopics() {
  mkTopic(tTemp,        sizeof(tTemp),        "temperature");
  mkTopic(tSetpoint,    sizeof(tSetpoint),    "setpoint");
  mkTopic(tSetpointCmd, sizeof(tSetpointCmd), "setpoint_cmd");
  mkTopic(tHeating,     sizeof(tHeating),     "heating");
  mkTopic(tAction,      sizeof(tAction),      "action");
  mkTopic(tMode,        sizeof(tMode),        "mode");
  mkTopic(tModeCmd,     sizeof(tModeCmd),     "mode_cmd");
  mkTopic(tSeason,      sizeof(tSeason),      "season");
  mkTopic(tSeasonCmd,   sizeof(tSeasonCmd),   "season_cmd");
  mkTopic(tAvail,       sizeof(tAvail),       "availability");
  mkTopic(tSensor,      sizeof(tSensor),      "sensor");
  mkTopic(tInfo,        sizeof(tInfo),        "info");
}

// ============================================================
//                 FILESYSTEM (LittleFS, scrittura atomica)
// ============================================================
bool writeJsonAtomic(const char* path, JsonDocument& doc) {
  if (!fsOk) return false;
  char tmp[24];
  snprintf(tmp, sizeof(tmp), "%s.tmp", path);
  File f = LittleFS.open(tmp, "w");
  if (!f) return false;
  size_t n = serializeJson(doc, f);
  f.close();
  if (n == 0) { LittleFS.remove(tmp); return false; }
  // rename su LittleFS sostituisce il file in modo atomico: un blackout a metà
  // lascia il vecchio file intatto, mai uno vuoto o troncato.
  if (LittleFS.rename(tmp, path)) return true;
  LittleFS.remove(path);
  return LittleFS.rename(tmp, path);
}

bool readJson(const char* path, JsonDocument& doc) {
  if (!fsOk) return false;
  File f = LittleFS.open(path, "r");
  if (!f) return false;
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  return err == DeserializationError::Ok;
}

bool saveNetConfig() {
  JSON_DOC(doc, 512);
  doc["server"]   = cfg.server;
  doc["port"]     = cfg.port;
  doc["user"]     = cfg.user;
  doc["pass"]     = cfg.pass;
  doc["zona"]     = cfg.zona;
  doc["offset"]   = cfg.offset;
  doc["ota_pass"] = cfg.otaPass;
  return writeJsonAtomic(NET_FILE, doc);
}

void loadNetConfig() {
  JSON_DOC(doc, 512);
  if (!readJson(NET_FILE, doc)) return;
  strlcpy(cfg.server,  doc["server"]   | "",     sizeof(cfg.server));
  strlcpy(cfg.port,    doc["port"]     | "1883", sizeof(cfg.port));
  strlcpy(cfg.user,    doc["user"]     | "",     sizeof(cfg.user));
  strlcpy(cfg.pass,    doc["pass"]     | "",     sizeof(cfg.pass));
  strlcpy(cfg.zona,    doc["zona"]     | "",     sizeof(cfg.zona));
  strlcpy(cfg.offset,  doc["offset"]   | "0.0",  sizeof(cfg.offset));
  strlcpy(cfg.otaPass, doc["ota_pass"] | "",     sizeof(cfg.otaPass));
}

bool saveState() {
  JSON_DOC(doc, 128);
  doc["setpoint"] = st.setpoint;
  doc["season"]   = st.winter ? "winter" : "summer";
  doc["enabled"]  = st.enabled;
  bool ok = writeJsonAtomic(STATE_FILE, doc);
  if (ok) stSaved = st;
  return ok;
}

void loadState() {
  JSON_DOC(doc, 128);
  if (!readJson(STATE_FILE, doc)) return;
  float sp = doc["setpoint"] | 21.0f;
  if (isfinite(sp)) st.setpoint = constrain(sp, SP_MIN, SP_MAX);
  st.winter  = strcmp(doc["season"] | "winter", "summer") != 0;
  st.enabled = doc["enabled"] | true;
}

// Primo avvio dopo il firmware v1 (SPIFFS): recupera credenziali MQTT e stato,
// poi formatta la partizione in LittleFS. Succede una sola volta.
void fsBegin() {
  LittleFSConfig lcfg;
  lcfg.setAutoFormat(false);
  LittleFS.setConfig(lcfg);
  if (LittleFS.begin() || LittleFS.begin()) { fsOk = true; return; }

  JSON_DOC(old, 512);
  bool haveOld = false;
  SPIFFSConfig scfg;
  scfg.setAutoFormat(false);
  SPIFFS.setConfig(scfg);
  if (SPIFFS.begin()) {
    File f = SPIFFS.open("/config.json", "r");
    if (f) {
      haveOld = (deserializeJson(old, f) == DeserializationError::Ok);
      f.close();
    }
    SPIFFS.end();
  }

  if (!LittleFS.format()) return;
  fsOk = LittleFS.begin();
  if (!fsOk || !haveOld) return;

  strlcpy(cfg.server, old["mqtt_server"] | "",     sizeof(cfg.server));
  strlcpy(cfg.port,   old["mqtt_port"]   | "1883", sizeof(cfg.port));
  strlcpy(cfg.user,   old["mqtt_user"]   | "",     sizeof(cfg.user));
  strlcpy(cfg.pass,   old["mqtt_pass"]   | "",     sizeof(cfg.pass));
  saveNetConfig();   // la zona manca: il portale si aprirà per chiederla

  if (!old["setpoint"].isNull()) {
    float sp = old["setpoint"] | 21.0f;
    if (isfinite(sp)) st.setpoint = constrain(sp, SP_MIN, SP_MAX);
    st.winter  = strcmp(old["season"] | "winter", "summer") != 0;
    st.enabled = old["enabled"] | true;
    saveState();
  }
}

// ============================================================
//                         DISPLAY
// ============================================================
void setContrast(uint8_t c) {
  if (c == contrastCur) return;
  display.ssd1306_command(SSD1306_SETCONTRAST);
  display.ssd1306_command(c);
  contrastCur = c;
}

void showMsgNow(const char* l1, const char* l2) {
  setContrast(CONTRAST_ON);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 8);
  display.print(l1);
  display.setCursor(0, 20);
  display.print(l2);
  display.display();
}

void drawOta(uint8_t pct) {
  setContrast(CONTRAST_ON);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(F("Aggiornamento "));
  display.print(pct);
  display.print('%');
  display.drawRect(0, 14, 128, 10, SSD1306_WHITE);
  display.fillRect(2, 16, (uint16_t)124 * pct / 100, 6, SSD1306_WHITE);
  display.setCursor(0, 25);
  display.print(F("Non spegnere"));
  display.display();
}

void showScreen(Screen s) { screen = s; screenSince = millis(); }
void userActivity()       { lastActivity = millis(); }

void showAlert(const char* a, const char* b) {
  strlcpy(alert1, a, sizeof(alert1));
  strlcpy(alert2, b, sizeof(alert2));
  showScreen(SCR_ALERT);
}

void drawBig(const char* txt, int dx, int dy) {
  display.setTextSize(3);
  int16_t x1, y1; uint16_t w, h;
  display.getTextBounds(txt, 0, 0, &x1, &y1, &w, &h);
  int x = (SCREEN_WIDTH - (int)w) / 2 + dx;
  int y = (SCREEN_HEIGHT - (int)h) / 2 - 4 + dy;
  if (x < 0) x = 0;
  if (y < 0) y = 0;
  display.setCursor(x, y);
  display.print(txt);
  display.setTextSize(1);
}

void drawStatus() {
  // stato richiesta (alto a destra)
  if (!st.enabled) {
    display.setCursor(110, 0);
    display.print(F("OFF"));
  } else if (callForHeat) {
    display.drawBitmap(118, 2, st.winter ? ICON_FLAME : ICON_SNOW, 8, 8, SSD1306_WHITE);
  }
  // WiFi (cerchio) e MQTT (quadrato), pieni = connesso
  int bx = 116, by = 29;
  if (WiFi.status() == WL_CONNECTED) display.fillCircle(bx, by, 2, SSD1306_WHITE);
  else                               display.drawCircle(bx, by, 2, SSD1306_WHITE);
  if (mqtt.connected()) display.fillRect(bx + 8, by - 2, 5, 5, SSD1306_WHITE);
  else                  display.drawRect(bx + 8, by - 2, 5, 5, SSD1306_WHITE);
}

void render() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  char buf[12];

  if (portalActive) {
    display.setCursor(0, 0);
    display.print(F("CONFIGURA DA WEB"));
    display.setCursor(0, 8);
    display.print(F("WiFi: "));
    display.print(apName);
    display.setCursor(0, 16);
    display.print(F("Pass: "));
    display.print(AP_PASS);
    display.setCursor(0, 24);
    if (!cfg.zona[0]) display.print(F("Manca la zona!"));
    else              display.print(F("Apri 192.168.4.1"));
  } else {
    switch (screen) {
      case SCR_ALERT:
        display.setCursor(0, 8);
        display.print(alert1);
        display.setCursor(0, 20);
        display.print(alert2);
        break;

      case SCR_SEASON:
        display.setTextSize(2);
        display.setCursor(10, 10);
        display.print(st.winter ? F("INVERNO") : F("ESTATE"));
        display.drawBitmap(100, 0, st.winter ? ICON_FLAME : ICON_SNOW, 8, 8, SSD1306_WHITE);
        display.setTextSize(1);
        break;

      case SCR_SETPOINT:
        dtostrf(st.setpoint, 0, 1, buf);
        drawBig(buf, 0, 0);
        display.drawLine(0, 0, SCREEN_WIDTH - 1, 0, SSD1306_WHITE);
        display.setCursor(0, 12);
        display.print(F("SET"));
        drawStatus();
        break;

      case SCR_AMBIENT:
      default:
        if (sensorError) {
          display.setCursor(0, 8);
          display.print(F("Sensore"));
          display.setCursor(0, 20);
          display.print(F("non rilevato"));
          drawStatus();
        } else {
          static const int8_t SX[5] = { 0, 2, -2, 1, -1 };
          static const int8_t SY[5] = { 0, 1,  2, 1,  0 };
          uint8_t k = (millis() / SHIFT_PERIOD_MS) % 5;
          if (isnan(tempC)) strcpy(buf, "--.-");
          else              dtostrf(tempC, 0, 1, buf);
          drawBig(buf, SX[k], SY[k]);
          drawStatus();
        }
        break;
    }
  }
  display.display();
}

void updateContrast() {
  bool bright = portalActive || screen != SCR_AMBIENT || sensorError ||
                (millis() - lastActivity < DIM_AFTER_MS);
  setContrast(bright ? CONTRAST_ON : CONTRAST_DIM);
}

// ============================================================
//                     LOGICA TERMOSTATO
// ============================================================
void updateHeating() {
  if (!st.enabled || isnan(tempC)) { callForHeat = false; return; }
  if (st.winter) {
    if (tempC < st.setpoint - HYST)      callForHeat = true;
    else if (tempC > st.setpoint + HYST) callForHeat = false;
  } else {
    if (tempC > st.setpoint + HYST)      callForHeat = true;
    else if (tempC < st.setpoint - HYST) callForHeat = false;
  }
}

const char* modeStr()   { return st.enabled ? (st.winter ? "heat" : "cool") : "off"; }
const char* actionStr() {
  if (!st.enabled) return "off";
  if (callForHeat) return st.winter ? "heating" : "cooling";
  return "idle";
}

void publishHeating() {
  updateHeating();
  if (!mqtt.connected()) return;
  mqtt.publish(tHeating, (st.enabled && callForHeat) ? "ON" : "OFF", true);
  mqtt.publish(tAction, actionStr(), true);
}

void publishAll() {
  if (!mqtt.connected()) return;
  char b[16];
  if (!isnan(tempC)) { dtostrf(tempC, 0, 2, b); mqtt.publish(tTemp, b, true); }
  dtostrf(st.setpoint, 0, 1, b);
  mqtt.publish(tSetpoint, b, true);
  mqtt.publish(tMode, modeStr(), true);
  mqtt.publish(tSeason, st.winter ? "winter" : "summer", true);
  mqtt.publish(tSensor, sensorError ? "error" : "ok", true);
  publishHeating();
}

void publishInfo() {
  if (!mqtt.connected()) return;
  char b[256];
  snprintf(b, sizeof(b),
    "{\"fw\":\"%s\",\"host\":\"%s\",\"ip\":\"%s\",\"rssi\":%d,\"reset\":\"%s\","
    "\"heap\":%u,\"uptime_s\":%lu,\"offset\":%s}",
    FW_VERSION, hostName, WiFi.localIP().toString().c_str(), WiFi.RSSI(),
    resetReason, ESP.getFreeHeap(), (unsigned long)(millis() / 1000UL), cfg.offset);
  mqtt.publish(tInfo, b, true);
}

// Ogni modifica di setpoint/modo/stagione passa da qui: pubblica subito e
// programma il salvataggio in flash (differito, solo se cambiato davvero).
void stateChanged() {
  stateDirty = true;
  stateChangedAt = millis();
  publishAll();
}

void setSetpoint(float v) {
  if (!isfinite(v)) return;
  v = constrain(v, SP_MIN, SP_MAX);
  st.setpoint = roundf(v * 10.0f) / 10.0f;
  stateChanged();
}

void saveStateIfNeeded() {
  if (!stateDirty || millis() - stateChangedAt < STATE_SAVE_DELAY_MS) return;
  stateDirty = false;
  if (fabsf(st.setpoint - stSaved.setpoint) > 0.01f ||
      st.winter != stSaved.winter || st.enabled != stSaved.enabled) {
    saveState();
  }
}

void handleSensor() {
  static bool waiting = false;
  static uint32_t tStart = 0, tLast = 0;
  uint32_t now = millis();

  if (!waiting && now - tLast >= SENSOR_PERIOD_MS) {
    tLast = now;
    sensors.requestTemperatures();   // non bloccante (setWaitForConversion(false))
    tStart = now;
    waiting = true;
  } else if (waiting && now - tStart >= CONV_MS) {
    waiting = false;
    float raw = sensors.getTempCByIndex(0);
    // -127 = sensore assente, 85.0 = valore di reset all'accensione del DS18B20
    bool valid = raw != DEVICE_DISCONNECTED_C && raw != 85.0f && raw > -30.0f && raw < 70.0f;
    if (valid) {
      tempC = raw + tempOffset;
      sensorBad = 0;
      if (sensorError) { sensorError = false; publishAll(); }
    } else if (sensorBad < 255 && ++sensorBad >= SENSOR_BAD_LIMIT && !sensorError) {
      // una lettura sbagliata isolata non fa nulla; 3 di fila = guasto
      sensorError = true;
      tempC = NAN;
      publishAll();   // pubblica sensor=error e heating=OFF
    }
    bool prev = callForHeat;
    updateHeating();
    if (prev != callForHeat) publishHeating();
  }
}

// ============================================================
//                        PULSANTI / ENCODER
// ============================================================
void startPortal();
void factoryReset();

void handleEncoder() {
  noInterrupts();
  int16_t d = encSteps;
  encSteps = 0;
  interrupts();
  if (!d) return;

  userActivity();
  if (screen != SCR_SETPOINT) {
    // primo scatto: mostra il setpoint senza cambiarlo
    showScreen(SCR_SETPOINT);
    return;
  }
  setSetpoint(st.setpoint + d * 0.5f);
}

void handleButton() {
  static bool stable = HIGH, lastRaw = HIGH, portalFired = false;
  static uint32_t tRaw = 0, tPress = 0;
  uint32_t now = millis();
  bool raw = digitalRead(PIN_ENC_SW);

  if (raw != lastRaw) { lastRaw = raw; tRaw = now; }
  if (now - tRaw >= BTN_DEBOUNCE_MS && raw != stable) {
    stable = raw;
    userActivity();
    if (stable == LOW) {
      tPress = now;
      portalFired = false;
    } else if (!portalFired && now - tPress >= SEASON_PRESS_MS) {
      st.winter  = !st.winter;
      st.enabled = true;
      stateChanged();
      showScreen(SCR_SEASON);
    }
  }
  if (stable == LOW && !portalFired && now - tPress >= PORTAL_PRESS_MS) {
    portalFired = true;
    startPortal();
  }
}

void handleFlashButton() {
  static bool down = false;
  static uint32_t t0 = 0;
  if (digitalRead(PIN_FLASH) == LOW) {
    if (!down) { down = true; t0 = millis(); }
    else if (millis() - t0 >= FACTORY_PRESS_MS) factoryReset();
  } else {
    down = false;
  }
}

// ============================================================
//                       MQTT
// ============================================================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  char msg[32];
  if (length >= sizeof(msg)) length = sizeof(msg) - 1;
  memcpy(msg, payload, length);
  msg[length] = '\0';
  // trim + minuscolo
  char* m = msg;
  while (*m == ' ') m++;
  for (char* p = m + strlen(m); p > m && (p[-1] == ' ' || p[-1] == '\r' || p[-1] == '\n'); ) *--p = '\0';
  for (char* p = m; *p; ++p) *p = (char)tolower((unsigned char)*p);

  if (strcmp(topic, tSetpointCmd) == 0) {
    float v;
    if (parseFloatStrict(m, v)) {
      setSetpoint(v);
      showScreen(SCR_SETPOINT);
      userActivity();
    }
  } else if (strcmp(topic, tModeCmd) == 0) {
    if      (strcmp(m, "off")  == 0) { st.enabled = false; }
    else if (strcmp(m, "heat") == 0) { st.enabled = true; st.winter = true;  }
    else if (strcmp(m, "cool") == 0) { st.enabled = true; st.winter = false; }
    else return;
    stateChanged();
  } else if (strcmp(topic, tSeasonCmd) == 0) {
    if      (strcmp(m, "winter") == 0) st.winter = true;
    else if (strcmp(m, "summer") == 0) st.winter = false;
    else return;
    st.enabled = true;
    stateChanged();
    showScreen(SCR_SEASON);
    userActivity();
  }
}

void ensureMqtt() {
  static bool tried = false;
  static uint32_t lastAttempt = 0;
  if (mqtt.connected() || WiFi.status() != WL_CONNECTED || otaRunning) return;
  if (tried && millis() - lastAttempt < MQTT_RETRY_MS) return;
  tried = true;
  lastAttempt = millis();

  char cid[24];
  snprintf(cid, sizeof(cid), "thermo-%06x", ESP.getChipId());
  bool auth = cfg.user[0] != '\0';
  bool ok = mqtt.connect(cid, auth ? cfg.user : nullptr, auth ? cfg.pass : nullptr,
                         tAvail, 1, true, "offline");
  if (!ok) return;

  mqtt.subscribe(tSetpointCmd);
  mqtt.subscribe(tModeCmd);
  mqtt.subscribe(tSeasonCmd);
  mqtt.publish(tAvail, "online", true);
  publishInfo();
  publishAll();
}

// ============================================================
//                    WIFI / PORTALE / OTA
// ============================================================
void onPortalSave() {
  strlcpy(cfg.server,  pServer->getValue(), sizeof(cfg.server));
  strlcpy(cfg.port,    pPort->getValue(),   sizeof(cfg.port));
  strlcpy(cfg.user,    pUser->getValue(),   sizeof(cfg.user));
  strlcpy(cfg.pass,    pPass->getValue(),   sizeof(cfg.pass));
  strlcpy(cfg.zona,    pZona->getValue(),   sizeof(cfg.zona));
  strlcpy(cfg.offset,  pOffset->getValue(), sizeof(cfg.offset));
  strlcpy(cfg.otaPass, pOta->getValue(),    sizeof(cfg.otaPass));
  if (!cfg.port[0]) strlcpy(cfg.port, "1883", sizeof(cfg.port));
  sanitizeZone(cfg.zona);
  applyOffset();
  saveNetConfig();
  restartPending = true;
  restartAt = millis() + RESTART_DELAY_MS;
}

void onWifiSaved() {
  // chiamata solo se la connessione alla nuova rete è riuscita
  restartPending = true;
  restartAt = millis() + RESTART_DELAY_MS;
}

void onPortalOta() {
  otaRunning = true;
  portalOta  = true;
  otaSince   = millis();
  drawOta(0);
}

void setupPortal() {
  static WiFiManagerParameter hMqtt("<h3>MQTT</h3>");
  static WiFiManagerParameter hDev("<h3>Termostato</h3>");
  static WiFiManagerParameter hOta("<h3>Aggiornamento OTA</h3>");
  pServer = new WiFiManagerParameter("server", "Server (IP o nome)", cfg.server, sizeof(cfg.server) - 1);
  pPort   = new WiFiManagerParameter("port",   "Porta",             cfg.port,   sizeof(cfg.port) - 1);
  pUser   = new WiFiManagerParameter("user",   "Utente",            cfg.user,   sizeof(cfg.user) - 1);
  pPass   = new WiFiManagerParameter("pass",   "Password",          cfg.pass,   sizeof(cfg.pass) - 1, "type='password'");
  pZona   = new WiFiManagerParameter("zona",   "Zona (es. Sala, Bagno, CameraGrande)", cfg.zona, sizeof(cfg.zona) - 1);
  pOffset = new WiFiManagerParameter("offset", "Correzione sensore &deg;C (es. -2.5)", cfg.offset, sizeof(cfg.offset) - 1);
  pOta    = new WiFiManagerParameter("ota",    "Password OTA (vuota = OTA da IDE disattivato)", cfg.otaPass, sizeof(cfg.otaPass) - 1, "type='password'");

  wm.addParameter(&hMqtt);
  wm.addParameter(pServer);
  wm.addParameter(pPort);
  wm.addParameter(pUser);
  wm.addParameter(pPass);
  wm.addParameter(&hDev);
  wm.addParameter(pZona);
  wm.addParameter(pOffset);
  wm.addParameter(&hOta);
  wm.addParameter(pOta);

  // Menu ridotto all'essenziale: WiFi, Impostazioni (MQTT/zona/OTA),
  // aggiornamento firmware da browser, riavvio, chiudi portale.
  static const char* menu[] = { "wifi", "param", "update", "restart", "exit" };
  wm.setMenu(menu, sizeof(menu) / sizeof(menu[0]));
  wm.setTitle("PiZaff Termostato");
  wm.setClass("invert");
  wm.setDebugOutput(false);
  wm.setConfigPortalBlocking(false);
  wm.setBreakAfterConfig(false);     // WiFi sbagliato: resta nel portale e ripeti
  wm.setConnectTimeout(20);
  wm.setSaveParamsCallback(onPortalSave);
  wm.setSaveConfigCallback(onWifiSaved);
  wm.setPreOtaUpdateCallback(onPortalOta);
}

void startPortal() {
  if (portalActive) return;
  WiFi.persistent(true);   // la nuova rete inserita nel portale va salvata in flash
  wm.setConfigPortalTimeout(configured ? PORTAL_TIMEOUT_S : 0);
  wm.startConfigPortal(apName, AP_PASS);
  portalActive = true;
}

void startWifi() {
  // persistent(false): da qui in poi nessuna chiamata WiFi scrive in flash.
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.hostname(hostName);
  WiFi.setSleepMode(WIFI_NONE_SLEEP);   // il modem-sleep causa cadute MQTT su molti AP
  WiFi.setAutoReconnect(true);
  WiFi.begin();                         // usa le credenziali salvate dal portale
}

// MAI WiFi.disconnect() qui: su ESP8266 cancella le credenziali salvate.
// WiFi.reconnect() invece riprova la connessione senza toccare la flash.
void wifiWatchdog() {
  static uint32_t downSince = 0, lastKick = 0;
  uint32_t now = millis();
  if (WiFi.status() == WL_CONNECTED || portalActive || otaRunning) { downSince = 0; return; }
  if (!downSince) { downSince = now; lastKick = now; return; }
  if (now - lastKick >= WIFI_KICK_MS) { lastKick = now; WiFi.reconnect(); }
  if (now - downSince >= WIFI_REBOOT_MS) ESP.restart();
}

void startOta() {
  if (otaStarted || !cfg.otaPass[0] || WiFi.status() != WL_CONNECTED) return;
  ArduinoOTA.setHostname(hostName);
  ArduinoOTA.setPassword(cfg.otaPass);
  ArduinoOTA.onStart([]() {
    otaRunning = true;
    if (mqtt.connected()) { mqtt.publish(tAvail, "offline", true); mqtt.disconnect(); }
    drawOta(0);
  });
  ArduinoOTA.onProgress([](unsigned int p, unsigned int t) {
    static uint8_t last = 255;
    uint8_t pct = t ? (uint8_t)((uint32_t)p * 100UL / t) : 0;
    if (pct != last && pct % 5 == 0) { last = pct; drawOta(pct); }
  });
  ArduinoOTA.onEnd([]() { showMsgNow("Aggiornato", "Riavvio..."); });
  ArduinoOTA.onError([](ota_error_t) {
    otaRunning = false;
    showAlert("OTA fallito", "");
  });
  ArduinoOTA.begin();
  otaStarted = true;
}

void connectionAlerts() {
  static bool ever = false, wasW = false, wasM = false;
  bool w = WiFi.status() == WL_CONNECTED;
  bool m = mqtt.connected();
  if (!ever) {
    if (w && m) { ever = true; wasW = w; wasM = m; }
    return;   // nessun avviso finché non ci si è connessi almeno una volta
  }
  if (w == wasW && m == wasM) return;
  wasW = w; wasM = m;
  if (w && m) showAlert("RICONNESSO", "");
  else        showAlert("DISCONNESSO", (!w && !m) ? "WiFi+MQTT" : (!w ? "WiFi" : "MQTT"));
}

void factoryReset() {
  showMsgNow("Reset fabbrica", "Riavvio...");
  if (mqtt.connected()) { mqtt.publish(tAvail, "offline", true); mqtt.disconnect(); }
  wm.resetSettings();   // cancella le credenziali WiFi (voluto, solo qui)
  if (fsOk) { LittleFS.remove(NET_FILE); LittleFS.remove(STATE_FILE); }
  delay(1000);
  ESP.restart();
}

// ============================================================
//                        SETUP / LOOP
// ============================================================
void setup() {
  Serial.begin(115200);
  Serial.println();
  strlcpy(resetReason, ESP.getResetReason().c_str(), sizeof(resetReason));

  pinMode(PIN_FLASH,   INPUT_PULLUP);
  pinMode(PIN_ENC_CLK, INPUT_PULLUP);
  pinMode(PIN_ENC_DT,  INPUT_PULLUP);
  pinMode(PIN_ENC_SW,  INPUT_PULLUP);

  Wire.begin(PIN_SDA, PIN_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C, true, false)) {
    Serial.println(F("SSD1306 non trovato"));
  }
  showMsgNow("Avvio...", "FW " FW_VERSION);

  fsBegin();
  if (fsOk) { loadNetConfig(); loadState(); }
  else      showMsgNow("Errore memoria", "flash");
  stSaved = st;
  sanitizeZone(cfg.zona);
  applyOffset();
  buildNames();
  buildTopics();

  sensors.begin();
  sensors.setWaitForConversion(false);

  encState = (GPIP(PIN_ENC_CLK) << 1) | GPIP(PIN_ENC_DT);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_CLK), encISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_DT),  encISR, CHANGE);

  net.setTimeout(3000);
  mqtt.setServer(cfg.server, atoi(cfg.port));
  mqtt.setCallback(mqttCallback);
  mqtt.setKeepAlive(30);
  mqtt.setSocketTimeout(3);
  mqtt.setBufferSize(512);

  setupPortal();
  configured = wm.getWiFiIsSaved() && cfg.server[0] && cfg.zona[0];
  if (configured) startWifi();
  else            startPortal();   // primo avvio: portale aperto finché non si configura

  lastActivity = millis();
  Serial.printf("PiZaff termostato %s - zona '%s' - host %s\n", FW_VERSION, cfg.zona, hostName);
}

void loop() {
  if (portalActive) {
    wm.process();
    if (!wm.getConfigPortalActive()) {
      portalActive = false;
      WiFi.persistent(false);
    }
  }
  // Upload dal portale fallito o interrotto: il portale non riavvia, quindi
  // dopo 2 minuti si torna al funzionamento normale.
  if (portalOta && millis() - otaSince > 120000UL) { portalOta = false; otaRunning = false; }

  if (restartPending && timeReached(restartAt)) {
    showMsgNow("Salvato", "Riavvio...");
    delay(300);
    ESP.restart();
  }

  handleFlashButton();
  handleButton();
  handleEncoder();

  if (configured) {
    wifiWatchdog();
    startOta();
    if (otaStarted) ArduinoOTA.handle();
    ensureMqtt();
    mqtt.loop();
    connectionAlerts();
  }

  handleSensor();
  saveStateIfNeeded();

  static uint32_t tPub = 0, tInfo = 0;
  if (millis() - tPub >= PUBLISH_PERIOD_MS) { tPub = millis(); publishAll(); }
  if (millis() - tInfo >= INFO_PERIOD_MS)   { tInfo = millis(); publishInfo(); }

  // timeout schermate
  uint32_t now = millis();
  if (screen == SCR_SETPOINT && now - lastActivity >= UI_TIMEOUT_MS)  showScreen(SCR_AMBIENT);
  if (screen == SCR_SEASON   && now - screenSince  >= SEASON_SHOW_MS) showScreen(SCR_AMBIENT);
  if (screen == SCR_ALERT    && now - screenSince  >= ALERT_MS)       showScreen(SCR_AMBIENT);

  if (!otaRunning) {
    updateContrast();
    if (now - lastRender >= RENDER_PERIOD_MS) { lastRender = now; render(); }
  }

  delay(2);
}
