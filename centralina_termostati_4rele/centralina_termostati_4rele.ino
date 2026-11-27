/*
 * PiZaff Centralina 4 Relè per termostati MQTT
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
  ESP8266 NodeMCU - Centralina 4 relè, v2.0.0

  - Legge termostato/<Zona>/(season|heating|availability)
      * season=summer  -> relè sempre OFF (spegnimento immediato)
      * season=winter  -> segue heating (ON/OFF) con tempi minimi ON/OFF
      * availability=offline -> la zona si spegne (dopo il tempo minimo ON)
  - Pubblica lo stato reale dei relè (retained) a ogni cambio e ogni 5 s:
      riscaldamento/<Zona>/state
  - Override manuale da HA: riscaldamento/<Zona>/set = ON/OFF (solo in winter)
  - Disponibilità (LWT): riscaldamento/centralina/availability
  - Diagnostica: riscaldamento/centralina/info (JSON retained)
  - Pulsanti: breve = toggle zona; lungo >=2 s = TUTTO OFF;
              Bagno+CameraGrande 3 s = apre il portale; 15 s = reset di fabbrica
  - Nomi delle 4 zone, MQTT e password OTA si impostano dal portale web.
  - OTA da Arduino IDE (porta di rete) o caricando il .bin dal portale web.
    All'inizio di ogni aggiornamento tutti i relè vengono spenti.

  Relè: D0/D1/D2/D5  | Pulsanti: D6/D7/D3/D4
*/

#include <ESP8266WiFi.h>
#include <ArduinoOTA.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <FS.h>
#include <LittleFS.h>
#include <ArduinoJson.h>

#define FW_VERSION "2.0.0"

#if ARDUINOJSON_VERSION_MAJOR >= 7
  #define JSON_DOC(name, size) JsonDocument name
#else
  #define JSON_DOC(name, size) DynamicJsonDocument name(size)
#endif

// ---------------- PINOUT (etichette NodeMCU) ----------------
// Relè (active-LOW) -> IN1..IN4
const uint8_t RELAY_PINS[4]  = { D0, D1, D2, D5 };
// Pulsanti verso GND, pull-up interno. D3 = GPIO0: non tenerlo premuto
// all'accensione (l'ESP entrerebbe in modalità programmazione).
const uint8_t BUTTON_PINS[4] = { D6, D7, D3, D4 };
const char* const DEFAULT_ZONES[4] = { "Bagno", "CameraPiccola", "Sala", "CameraGrande" };

// ---------------- CONTROLLO ----------------
const uint32_t MIN_ON_MS        = 180000;   // 3 min
const uint32_t MIN_OFF_MS       = 120000;   // 2 min
const uint32_t SAFETY_MS        = 900000;   // 15 min senza comandi -> zona OFF
const uint32_t STAGGER_MS       = 800;      // sfasamento accensioni
const uint32_t STATE_PUB_MS     = 5000;
const uint32_t INFO_PERIOD_MS   = 300000;
const uint32_t MQTT_RETRY_MS    = 5000;
const uint32_t WIFI_KICK_MS     = 60000;
const uint32_t WIFI_REBOOT_MS   = 900000;   // WiFi giù da 15 min: riavvio
const uint32_t RESTART_DELAY_MS = 3000;
const uint32_t DEBOUNCE_MS      = 35;
const uint32_t LONG_MS          = 2000;
const uint32_t COMBO_PORTAL_MS  = 3000;
const uint32_t COMBO_RESET_MS   = 15000;
const uint16_t PORTAL_TIMEOUT_S = 300;

const char* AP_PASS = "pizaff123";
const char* TOPIC_BASE  = "riscaldamento";
const char* THERMO_BASE = "termostato";
const char* T_AVAIL = "riscaldamento/centralina/availability";
const char* T_INFO  = "riscaldamento/centralina/info";

// ---------------- CONFIGURAZIONE ----------------
struct NetConfig {
  char server[64];
  char port[6];
  char user[64];
  char pass[64];
  char otaPass[33];
  char zone[4][24];
};
NetConfig cfg = { "", "1883", "", "", "",
                  { "Bagno", "CameraPiccola", "Sala", "CameraGrande" } };

bool fsOk = false;
const char* NET_FILE = "/net.json";

WiFiClient   net;
PubSubClient mqtt(net);
WiFiManager  wm;
WiFiManagerParameter *pServer, *pPort, *pUser, *pPass, *pOta, *pZone[4];

bool configured     = false;
bool portalActive   = false;
bool otaStarted     = false;
bool otaRunning     = false;
bool portalOta      = false;
uint32_t otaSince   = 0;
bool restartPending = false;
uint32_t restartAt  = 0;

char hostName[32];
char apName[24];
char resetReason[32];

// ---------------- STATO ZONE ----------------
struct ZoneState {
  bool commanded = false;
  bool relayOn   = false;
  bool allowFromThermo = true;    // true = winter, segue heating
  uint32_t lastChange = 0;
  uint32_t lastCmd    = 0;
  bool btnLast = true;            // pull-up: true = riposo
  uint32_t btnLastChange = 0;
};
ZoneState Z[4];

char topicSeason[4][64], topicHeating[4][64], topicThAvail[4][64],
     topicSet[4][64], topicState[4][64];

uint32_t lastOnAt = 0;            // ultimo relè acceso (sfasamento)
bool suppressRelease[4] = { false, false, false, false };

// ---------------- UTILITÀ ----------------
bool timeReached(uint32_t t) { return (int32_t)(millis() - t) >= 0; }

void sanitizeZone(char* z, uint8_t idx) {
  for (char* p = z; *p; ++p) {
    char c = *p;
    if (!(isalnum((unsigned char)c) || c == '_' || c == '-')) *p = '_';
  }
  if (!z[0]) strlcpy(z, DEFAULT_ZONES[idx], sizeof(cfg.zone[0]));
}

void buildNames() {
  snprintf(apName, sizeof(apName), "ESP_Config-%04X", (unsigned)(ESP.getChipId() & 0xFFFF));
  snprintf(hostName, sizeof(hostName), "centralina-risc");
}

void buildTopics() {
  for (uint8_t i = 0; i < 4; i++) {
    snprintf(topicSeason[i],  64, "%s/%s/season",       THERMO_BASE, cfg.zone[i]);
    snprintf(topicHeating[i], 64, "%s/%s/heating",      THERMO_BASE, cfg.zone[i]);
    snprintf(topicThAvail[i], 64, "%s/%s/availability", THERMO_BASE, cfg.zone[i]);
    snprintf(topicSet[i],     64, "%s/%s/set",          TOPIC_BASE,  cfg.zone[i]);
    snprintf(topicState[i],   64, "%s/%s/state",        TOPIC_BASE,  cfg.zone[i]);
  }
}

// ---------------- FILESYSTEM ----------------
bool writeJsonAtomic(const char* path, JsonDocument& doc) {
  if (!fsOk) return false;
  char tmp[24];
  snprintf(tmp, sizeof(tmp), "%s.tmp", path);
  File f = LittleFS.open(tmp, "w");
  if (!f) return false;
  size_t n = serializeJson(doc, f);
  f.close();
  if (n == 0) { LittleFS.remove(tmp); return false; }
  if (LittleFS.rename(tmp, path)) return true;
  LittleFS.remove(path);
  return LittleFS.rename(tmp, path);
}

bool saveNetConfig() {
  JSON_DOC(doc, 768);
  doc["server"]   = cfg.server;
  doc["port"]     = cfg.port;
  doc["user"]     = cfg.user;
  doc["pass"]     = cfg.pass;
  doc["ota_pass"] = cfg.otaPass;
  for (uint8_t i = 0; i < 4; i++) {
    char key[4] = { 'z', (char)('1' + i), '\0', '\0' };
    doc[key] = cfg.zone[i];
  }
  return writeJsonAtomic(NET_FILE, doc);
}

void loadNetConfig() {
  if (!fsOk) return;
  File f = LittleFS.open(NET_FILE, "r");
  if (!f) return;
  JSON_DOC(doc, 768);
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) return;
  strlcpy(cfg.server,  doc["server"]   | "",     sizeof(cfg.server));
  strlcpy(cfg.port,    doc["port"]     | "1883", sizeof(cfg.port));
  strlcpy(cfg.user,    doc["user"]     | "",     sizeof(cfg.user));
  strlcpy(cfg.pass,    doc["pass"]     | "",     sizeof(cfg.pass));
  strlcpy(cfg.otaPass, doc["ota_pass"] | "",     sizeof(cfg.otaPass));
  for (uint8_t i = 0; i < 4; i++) {
    char key[4] = { 'z', (char)('1' + i), '\0', '\0' };
    strlcpy(cfg.zone[i], doc[key] | DEFAULT_ZONES[i], sizeof(cfg.zone[i]));
  }
}

// Primo avvio dopo il firmware v1 (SPIFFS): recupera le credenziali MQTT,
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
  saveNetConfig();
}

// ---------------- RELÈ ----------------
inline void relayWrite(uint8_t pin, bool on) { digitalWrite(pin, on ? LOW : HIGH); }

void publishState(uint8_t i) {
  if (mqtt.connected()) mqtt.publish(topicState[i], Z[i].relayOn ? "ON" : "OFF", true);
}

void publishAllStates() {
  for (uint8_t i = 0; i < 4; i++) publishState(i);
}

void setRelay(uint8_t i, bool on) {
  relayWrite(RELAY_PINS[i], on);
  Z[i].relayOn = on;
  Z[i].lastChange = millis();
  if (on) lastOnAt = millis();
  publishState(i);
}

void allOff() {
  for (uint8_t i = 0; i < 4; i++) {
    Z[i].commanded = false;
    Z[i].lastCmd = millis();
    if (Z[i].relayOn) setRelay(i, false);
    else relayWrite(RELAY_PINS[i], false);
  }
}

bool canTurnOn (uint8_t i) { return !Z[i].relayOn && millis() - Z[i].lastChange > MIN_OFF_MS; }
bool canTurnOff(uint8_t i) { return  Z[i].relayOn && millis() - Z[i].lastChange > MIN_ON_MS;  }

// Nessun delay(): applyLogic() viene chiamata anche dal callback MQTT.
void applyLogic() {
  if (otaRunning) return;
  for (uint8_t i = 0; i < 4; i++) {
    if (millis() - Z[i].lastCmd > SAFETY_MS) Z[i].commanded = false;
  }
  for (uint8_t i = 0; i < 4; i++) {
    if (Z[i].commanded && canTurnOn(i) && millis() - lastOnAt >= STAGGER_MS) {
      setRelay(i, true);
    }
    if (!Z[i].commanded && canTurnOff(i)) {
      setRelay(i, false);
    }
  }
}

// ---------------- PULSANTI ----------------
void startPortal();
void factoryReset();

void handleButtons() {
  static uint32_t pressedAt[4] = { 0, 0, 0, 0 };
  uint32_t now = millis();

  for (uint8_t i = 0; i < 4; i++) {
    bool raw = digitalRead(BUTTON_PINS[i]);   // true = riposo, false = premuto
    if (raw != Z[i].btnLast && now - Z[i].btnLastChange > DEBOUNCE_MS) {
      Z[i].btnLast = raw;
      Z[i].btnLastChange = now;

      if (!raw) {
        pressedAt[i] = now;
      } else if (suppressRelease[i]) {
        suppressRelease[i] = false;         // rilascio della combo: ignorato
      } else if (now - pressedAt[i] >= LONG_MS) {
        allOff();                           // lunga: TUTTO OFF immediato
      } else {
        Z[i].commanded = !Z[i].relayOn;     // breve: toggle manuale
        Z[i].lastCmd   = now;
      }
    }
  }

  // combo Bagno + CameraGrande: 3 s = portale (relè continuano a funzionare),
  // 15 s = reset di fabbrica
  static bool comboDown = false, comboPortal = false;
  static uint32_t comboT0 = 0;
  bool both = digitalRead(BUTTON_PINS[0]) == LOW && digitalRead(BUTTON_PINS[3]) == LOW;
  if (both) {
    if (!comboDown) { comboDown = true; comboPortal = false; comboT0 = now; }
    if (!comboPortal && now - comboT0 >= COMBO_PORTAL_MS) {
      comboPortal = true;
      suppressRelease[0] = true;
      suppressRelease[3] = true;
      startPortal();
    }
    if (now - comboT0 >= COMBO_RESET_MS) factoryReset();
  } else {
    comboDown = false;
  }
}

// ---------------- MQTT ----------------
void mqttCallback(char* topic, byte* payload, unsigned int len) {
  char msg[16];
  if (len >= sizeof(msg)) len = sizeof(msg) - 1;
  memcpy(msg, payload, len);
  msg[len] = '\0';
  char* m = msg;
  while (*m == ' ') m++;
  for (char* p = m + strlen(m); p > m && (p[-1] == ' ' || p[-1] == '\r' || p[-1] == '\n'); ) *--p = '\0';
  for (char* p = m; *p; ++p) *p = (char)toupper((unsigned char)*p);
  bool on = strcmp(m, "ON") == 0 || strcmp(m, "1") == 0 || strcmp(m, "TRUE") == 0;

  for (uint8_t i = 0; i < 4; i++) {
    if (strcmp(topic, topicSeason[i]) == 0) {
      if (strcmp(m, "WINTER") == 0) {
        Z[i].allowFromThermo = true;
      } else if (strcmp(m, "SUMMER") == 0) {
        Z[i].allowFromThermo = false;
        Z[i].commanded = false;
        if (Z[i].relayOn) setRelay(i, false);   // estate: spegni subito
      }
      return;
    }
    if (strcmp(topic, topicHeating[i]) == 0 || strcmp(topic, topicSet[i]) == 0) {
      if (Z[i].allowFromThermo) {
        Z[i].commanded = on;
        Z[i].lastCmd   = millis();
        applyLogic();
      } else {
        publishState(i);
      }
      return;
    }
    if (strcmp(topic, topicThAvail[i]) == 0) {
      // termostato offline (LWT): non fidarsi dell'ultimo "heating"
      if (strcmp(m, "OFFLINE") == 0) {
        Z[i].commanded = false;
        Z[i].lastCmd   = millis();
        applyLogic();
      }
      return;
    }
  }
}

void publishInfo() {
  if (!mqtt.connected()) return;
  char b[320];
  snprintf(b, sizeof(b),
    "{\"fw\":\"%s\",\"host\":\"%s\",\"ip\":\"%s\",\"rssi\":%d,\"reset\":\"%s\","
    "\"heap\":%u,\"uptime_s\":%lu,\"zones\":[\"%s\",\"%s\",\"%s\",\"%s\"]}",
    FW_VERSION, hostName, WiFi.localIP().toString().c_str(), WiFi.RSSI(),
    resetReason, ESP.getFreeHeap(), (unsigned long)(millis() / 1000UL),
    cfg.zone[0], cfg.zone[1], cfg.zone[2], cfg.zone[3]);
  mqtt.publish(T_INFO, b, true);
}

void ensureMqtt() {
  static bool tried = false;
  static uint32_t lastAttempt = 0;
  if (mqtt.connected() || WiFi.status() != WL_CONNECTED || otaRunning) return;
  if (tried && millis() - lastAttempt < MQTT_RETRY_MS) return;
  tried = true;
  lastAttempt = millis();

  char cid[32];
  snprintf(cid, sizeof(cid), "ESP8266_VALVOLE_%06x", ESP.getChipId());
  bool auth = cfg.user[0] != '\0';
  bool ok = mqtt.connect(cid, auth ? cfg.user : nullptr, auth ? cfg.pass : nullptr,
                         T_AVAIL, 1, true, "offline");
  if (!ok) return;

  for (uint8_t i = 0; i < 4; i++) {
    mqtt.subscribe(topicSeason[i], 1);
    mqtt.subscribe(topicHeating[i], 1);
    mqtt.subscribe(topicThAvail[i], 1);
    mqtt.subscribe(topicSet[i], 1);
  }
  mqtt.publish(T_AVAIL, "online", true);
  publishInfo();
  publishAllStates();
}

// ---------------- WIFI / PORTALE / OTA ----------------
void onPortalSave() {
  strlcpy(cfg.server,  pServer->getValue(), sizeof(cfg.server));
  strlcpy(cfg.port,    pPort->getValue(),   sizeof(cfg.port));
  strlcpy(cfg.user,    pUser->getValue(),   sizeof(cfg.user));
  strlcpy(cfg.pass,    pPass->getValue(),   sizeof(cfg.pass));
  strlcpy(cfg.otaPass, pOta->getValue(),    sizeof(cfg.otaPass));
  if (!cfg.port[0]) strlcpy(cfg.port, "1883", sizeof(cfg.port));
  for (uint8_t i = 0; i < 4; i++) {
    strlcpy(cfg.zone[i], pZone[i]->getValue(), sizeof(cfg.zone[i]));
    sanitizeZone(cfg.zone[i], i);
  }
  saveNetConfig();
  restartPending = true;
  restartAt = millis() + RESTART_DELAY_MS;
}

void onWifiSaved() {
  restartPending = true;
  restartAt = millis() + RESTART_DELAY_MS;
}

void onPortalOta() {
  allOff();
  otaRunning = true;
  portalOta  = true;
  otaSince   = millis();
}

void setupPortal() {
  static WiFiManagerParameter hMqtt("<h3>MQTT</h3>");
  static WiFiManagerParameter hZone("<h3>Nomi zone (uguali ai termostati)</h3>");
  static WiFiManagerParameter hOta("<h3>Aggiornamento OTA</h3>");
  static char zid[4][4];
  static char zlabel[4][32];
  pServer = new WiFiManagerParameter("server", "Server (IP o nome)", cfg.server, sizeof(cfg.server) - 1);
  pPort   = new WiFiManagerParameter("port",   "Porta",             cfg.port,   sizeof(cfg.port) - 1);
  pUser   = new WiFiManagerParameter("user",   "Utente",            cfg.user,   sizeof(cfg.user) - 1);
  pPass   = new WiFiManagerParameter("pass",   "Password",          cfg.pass,   sizeof(cfg.pass) - 1, "type='password'");
  pOta    = new WiFiManagerParameter("ota",    "Password OTA (vuota = OTA da IDE disattivato)", cfg.otaPass, sizeof(cfg.otaPass) - 1, "type='password'");
  for (uint8_t i = 0; i < 4; i++) {
    snprintf(zid[i], sizeof(zid[i]), "z%u", (unsigned)(i + 1));
    snprintf(zlabel[i], sizeof(zlabel[i]), "Zona %u (rel&egrave; %u)", (unsigned)(i + 1), (unsigned)(i + 1));
    pZone[i] = new WiFiManagerParameter(zid[i], zlabel[i], cfg.zone[i], sizeof(cfg.zone[i]) - 1);
  }

  wm.addParameter(&hMqtt);
  wm.addParameter(pServer);
  wm.addParameter(pPort);
  wm.addParameter(pUser);
  wm.addParameter(pPass);
  wm.addParameter(&hZone);
  for (uint8_t i = 0; i < 4; i++) wm.addParameter(pZone[i]);
  wm.addParameter(&hOta);
  wm.addParameter(pOta);

  static const char* menu[] = { "wifi", "param", "update", "restart", "exit" };
  wm.setMenu(menu, sizeof(menu) / sizeof(menu[0]));
  wm.setTitle("PiZaff Centralina");
  wm.setClass("invert");
  wm.setDebugOutput(false);
  wm.setConfigPortalBlocking(false);
  wm.setBreakAfterConfig(false);
  wm.setConnectTimeout(20);
  wm.setSaveParamsCallback(onPortalSave);
  wm.setSaveConfigCallback(onWifiSaved);
  wm.setPreOtaUpdateCallback(onPortalOta);
}

void startPortal() {
  if (portalActive) return;
  WiFi.persistent(true);
  wm.setConfigPortalTimeout(configured ? PORTAL_TIMEOUT_S : 0);
  wm.startConfigPortal(apName, AP_PASS);
  portalActive = true;
  Serial.printf("Portale aperto: %s\n", apName);
}

void startWifi() {
  WiFi.persistent(false);              // da qui in poi nessuna scrittura WiFi in flash
  WiFi.mode(WIFI_STA);
  WiFi.hostname(hostName);
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  WiFi.setAutoReconnect(true);
  WiFi.begin();
}

// MAI WiFi.disconnect() qui: su ESP8266 cancella le credenziali salvate.
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
    allOff();                         // sicurezza: niente relè accesi durante l'update
    otaRunning = true;
    if (mqtt.connected()) { mqtt.publish(T_AVAIL, "offline", true); mqtt.disconnect(); }
  });
  ArduinoOTA.onError([](ota_error_t) { otaRunning = false; });
  ArduinoOTA.begin();
  otaStarted = true;
}

void factoryReset() {
  allOff();
  if (mqtt.connected()) { mqtt.publish(T_AVAIL, "offline", true); mqtt.disconnect(); }
  wm.resetSettings();
  if (fsOk) LittleFS.remove(NET_FILE);
  Serial.println(F("Reset di fabbrica - riavvio"));
  delay(500);
  ESP.restart();
}

// ---------------- SETUP / LOOP ----------------
void setup() {
  // Relè fail-safe: livello impostato PRIMA di pinMode(OUTPUT).
  for (uint8_t i = 0; i < 4; i++) {
    digitalWrite(RELAY_PINS[i], HIGH);
    pinMode(RELAY_PINS[i], OUTPUT);
  }

  Serial.begin(115200);
  Serial.println();
  strlcpy(resetReason, ESP.getResetReason().c_str(), sizeof(resetReason));

  uint32_t now = millis();
  for (uint8_t i = 0; i < 4; i++) {
    Z[i].lastChange = now;
    Z[i].lastCmd    = now;
    pinMode(BUTTON_PINS[i], INPUT_PULLUP);
    Z[i].btnLast = digitalRead(BUTTON_PINS[i]);
    Z[i].btnLastChange = now;
  }

  fsBegin();
  loadNetConfig();
  for (uint8_t i = 0; i < 4; i++) sanitizeZone(cfg.zone[i], i);
  buildNames();
  buildTopics();

  net.setTimeout(3000);
  mqtt.setServer(cfg.server, atoi(cfg.port));
  mqtt.setCallback(mqttCallback);
  mqtt.setKeepAlive(30);
  mqtt.setSocketTimeout(3);
  mqtt.setBufferSize(512);

  setupPortal();
  configured = wm.getWiFiIsSaved() && cfg.server[0];
  if (configured) startWifi();
  else            startPortal();   // primo avvio; i pulsanti funzionano comunque

  Serial.printf("PiZaff centralina %s - zone %s/%s/%s/%s\n", FW_VERSION,
                cfg.zone[0], cfg.zone[1], cfg.zone[2], cfg.zone[3]);
}

void loop() {
  if (portalActive) {
    wm.process();
    if (!wm.getConfigPortalActive()) {
      portalActive = false;
      WiFi.persistent(false);
    }
  }
  if (portalOta && millis() - otaSince > 120000UL) { portalOta = false; otaRunning = false; }
  if (restartPending && timeReached(restartAt)) {
    allOff();
    delay(200);
    ESP.restart();
  }

  handleButtons();
  applyLogic();

  if (configured) {
    wifiWatchdog();
    startOta();
    if (otaStarted) ArduinoOTA.handle();
    ensureMqtt();
    mqtt.loop();
  }

  static uint32_t tState = 0, tInfo = 0;
  if (millis() - tState >= STATE_PUB_MS)   { tState = millis(); publishAllStates(); }
  if (millis() - tInfo  >= INFO_PERIOD_MS) { tInfo  = millis(); publishInfo(); }

  delay(5);
}
