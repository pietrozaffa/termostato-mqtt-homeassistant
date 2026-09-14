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
  ESP8266 NodeMCU (LOLIN) - Centralina 4 Relè
  - Captive portal identico ai termostati: WiFiManager.autoConnect("ESP_Config")
  - Reset Wi-Fi+config all'avvio tenendo premuto FLASH (GPIO0 / D3)
  - MQTT: server/porta/utente/password configurabili dal portale (nessuna
    credenziale hardcoded: vedi ATTENZIONE più sotto)
  - Legge termostato/<Zona>/(season|heating)
      * season=summer -> relè sempre OFF (spegnimento immediato)
      * season=winter -> segue heating (ON/OFF) con min ON/OFF
  - Pubblica stato relè (retained) ad ogni cambio e ogni 5s:
      riscaldamento/<Zona>/state
  - Pubblica disponibilità (LWT) su riscaldamento/centralina/availability
  - (Opzionale) override manuale: riscaldamento/<Zona>/set = ON/OFF (valido solo in winter)
  - Tastini locali: breve=toggle zona; lunga>=2s=ALL OFF; combo Z1+Z4 3s=portal soft

  Zone: Bagno, CameraPiccola, Sala, CameraGrande
  Relè: D0/D1/D2/D5  | Tasti: D6/D7/D3/D4

  ATTENZIONE: al primo avvio (o dopo un reset di fabbrica) il dispositivo apre
  il portale "ESP_Config" (http://192.168.4.1) dove vanno inseriti server,
  porta, utente e password del tuo broker MQTT: vengono poi salvati in
  /config.json sulla flash del dispositivo, MAI nel codice sorgente.
*/

#include <ESP8266WiFi.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <FS.h>
#include <ArduinoJson.h>

// ---------------- PINOUT (NodeMCU labels) ----------------
#define PIN_RESET_BOOT   D3      // GPIO0 (FLASH) per reset + portal all'avvio (⚠️ non tenerlo premuto a caso)

// Relè (active-LOW) -> IN1..IN4
const uint8_t RELAY_PINS[4]   = { D0, D1, D2, D5 }; // Bagno, CameraPiccola, Sala, CameraGrande

// Pulsanti (verso GND, pullup interno)
const uint8_t BUTTON_PINS[4]  = { D6, D7, D3, D4 }; // Bagno, CameraPiccola, Sala, CameraGrande

// Nomi zona (come richiesto)
const char* ZONE_NAMES[4] = { "Bagno", "CameraPiccola", "Sala", "CameraGrande" };

// ---------------- CONTROLLO ----------------
const unsigned long MIN_ON_MS   = 180000;   // 3 min
const unsigned long MIN_OFF_MS  = 120000;   // 2 min
const unsigned long SAFETY_MS   = 900000;   // 15 min
const unsigned long STAGGER_MS  = 800;      // sfasamento accensioni
const bool IMMEDIATE_OFF_ON_SUMMER = true;  // spegni subito quando entra in summer

// ---------------- MQTT CONFIG (vuoti di default: si impostano dal portale) ----------------
char mqtt_server[40] = "192.168.1.10";
char mqtt_port[6]    = "1883";
char mqtt_user[32]   = "";
char mqtt_pass[32]   = "";

// base override manuale (HA): riscaldamento/<Zona>/(set|state)
String topic_base = "riscaldamento";

// base dei termostati: termostato/<Zona>/(season|heating)
char thermo_base[40] = "termostato";

// Disponibilità (LWT) di tutta la centralina
const char* tAvail = "riscaldamento/centralina/availability";

WiFiClient espClient;
PubSubClient mqtt(espClient);

// ---------------- STATO ----------------
struct ZoneState {
  bool commanded = false;
  bool relayOn   = false;
  bool allowFromThermo = true;    // true = accetta heating (winter)
  unsigned long lastChange = 0;
  unsigned long lastCmd    = 0;

  // pulsanti
  bool btnLast = true;            // pull-up: true=riposo
  unsigned long btnLastChange = 0;
};
ZoneState Z[4];

// Topic precalcolati una sola volta in setup() (buildTopics()): evitare di
// ricostruirli con concatenazioni di String a ogni messaggio MQTT, che su
// ESP8266 frammenta lentamente l'heap e nel tempo porta a riavvii casuali.
String topicSeason[4];
String topicHeating[4];
String topicSet[4];
String topicState[4];

void buildTopics() {
  for (uint8_t i = 0; i < 4; i++) {
    topicSeason[i]  = String(thermo_base) + "/" + ZONE_NAMES[i] + "/season";
    topicHeating[i] = String(thermo_base) + "/" + ZONE_NAMES[i] + "/heating";
    topicSet[i]     = topic_base + "/" + ZONE_NAMES[i] + "/set";
    topicState[i]   = topic_base + "/" + ZONE_NAMES[i] + "/state";
  }
}

// Timer publish periodico stato (ogni 5s)
const unsigned long STATE_PUB_INTERVAL = 5000;
unsigned long lastStatePub = 0;

// ---------------- HELPERS ----------------
inline void relayWrite(uint8_t pin, bool on) { digitalWrite(pin, on ? LOW : HIGH); }

void publishState(uint8_t i) {
  mqtt.publish(topicState[i].c_str(), Z[i].relayOn ? "ON" : "OFF", true);
}

void publishAllStates() {
  for (uint8_t i=0; i<4; i++) publishState(i);
}

bool canTurnOn (uint8_t i){ return !Z[i].relayOn && (millis()-Z[i].lastChange) > MIN_OFF_MS; }
bool canTurnOff(uint8_t i){ return  Z[i].relayOn && (millis()-Z[i].lastChange) > MIN_ON_MS;  }

// Prossimo istante in cui è permesso accendere un relè (sfasamento accensioni).
// Sostituisce i vecchi delay(STAGGER_MS)/delay(10): applyLogic() viene chiamata
// anche da dentro mqttCallback (quindi da mqtt.loop()), e un delay() lì dentro
// blocca l'intero loop() — inclusi i pulsanti fisici e lo spegnimento di
// sicurezza — fino a 4×800ms se più zone diventano eleggibili insieme (es. al
// riconnect MQTT con più stati "ON" retained).
unsigned long nextStaggerSlot = 0;

void applyLogic() {
  // safety timeout
  for (uint8_t i=0; i<4; i++) {
    if (millis() - Z[i].lastCmd > SAFETY_MS) Z[i].commanded = false;
  }
  // applica uscite con vincoli
  for (uint8_t i=0; i<4; i++) {
    if (Z[i].commanded && canTurnOn(i) && millis() >= nextStaggerSlot) {
      relayWrite(RELAY_PINS[i], true);
      Z[i].relayOn = true;
      Z[i].lastChange = millis();
      nextStaggerSlot = millis() + STAGGER_MS;
      publishState(i);
    }
    if (!Z[i].commanded && canTurnOff(i)) {
      relayWrite(RELAY_PINS[i], false);
      Z[i].relayOn = false;
      Z[i].lastChange = millis();
      publishState(i);
    }
  }
}

// ---------------- BUTTONS ----------------
void startWiFiManager(bool allowFactoryReset); // forward (portal identico ai termostati)

// Quando la combo Z1+Z4 scatta, i pulsanti 0 e 3 sono anche loro tenuti premuti
// da >=2000ms: al rilascio la logica "pressione lunga" qui sotto li avrebbe
// letti come un secondo comando ALL OFF non richiesto. Questo flag fa
// "consumare" in silenzio quel singolo rilascio per i due pulsanti coinvolti.
bool suppressRelease[4] = {false, false, false, false};

void handleButtons() {
  const unsigned long DEBOUNCE = 35, LONG_MS = 2000;
  static unsigned long pressedAt[4] = {0,0,0,0};

  for (uint8_t i=0; i<4; i++) {
    bool raw = digitalRead(BUTTON_PINS[i]); // true=riposo, false=premuto
    if (raw != Z[i].btnLast && (millis() - Z[i].btnLastChange) > DEBOUNCE) {
      Z[i].btnLast = raw;
      Z[i].btnLastChange = millis();

      if (!raw) { // pressed
        pressedAt[i] = millis();
      } else if (suppressRelease[i]) { // released, ma da ignorare (vedi combo)
        suppressRelease[i] = false;
      } else {    // released
        unsigned long d = millis() - pressedAt[i];
        if (d >= LONG_MS) {
          // lunga: ALL OFF immediato
          for (uint8_t k=0; k<4; k++) {
            Z[k].commanded = false;
            relayWrite(RELAY_PINS[k], false);
            Z[k].relayOn = false;
            Z[k].lastChange = millis();
            Z[k].lastCmd = millis();
            publishState(k);
          }
        } else {
          // breve: toggle locale (bypass gating, manutenzione)
          Z[i].commanded = !Z[i].relayOn;
          Z[i].lastCmd   = millis();
        }
      }
    }
  }

  // combo: Z1+Z4 premuti 3s => riapre portal soft (senza wipe, senza ALL OFF)
  static unsigned long cT0 = 0;
  bool leftDown  = (digitalRead(BUTTON_PINS[0]) == LOW);
  bool rightDown = (digitalRead(BUTTON_PINS[3]) == LOW);
  if (leftDown && rightDown) {
    if (cT0 == 0) cT0 = millis();
    if (millis() - cT0 >= 3000) {
      suppressRelease[0] = true;
      suppressRelease[3] = true;
      startWiFiManager(false); // soft: MAI il reset di fabbrica in questo path
      cT0 = 0;
    }
  } else cT0 = 0;
}

// ---------------- MQTT ----------------
void onSeasonOrHeating(uint8_t i, bool isSeasonTopic, const String& msgUp) {
  if (isSeasonTopic) {
    if      (msgUp == "WINTER") Z[i].allowFromThermo = true;
    else if (msgUp == "SUMMER") {
      Z[i].allowFromThermo = false;
      Z[i].commanded = false;
      if (IMMEDIATE_OFF_ON_SUMMER) {
        relayWrite(RELAY_PINS[i], false);
        Z[i].relayOn = false;
        Z[i].lastChange = millis();
        publishState(i);
      }
    }
  } else { // topic /heating
    if (Z[i].allowFromThermo) {
      bool on = (msgUp=="ON" || msgUp=="1" || msgUp=="TRUE");
      Z[i].commanded = on;
      Z[i].lastCmd   = millis();
      applyLogic();
    } else {
      publishState(i); // in summer: ignora
    }
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int len) {
  // Confronto diretto su topic (char*) con strcmp: evita di avvolgerlo in una
  // String ad ogni messaggio in arrivo, il punto più "caldo" del programma
  // (fino a 12 topic sottoscritti, con repubbliche periodiche dai 4 termostati).
  String M; M.reserve(len);
  for (unsigned int i=0; i<len; i++) M += (char)payload[i];
  M.trim();
  String MU = M; MU.toUpperCase();

  for (uint8_t i=0; i<4; i++) {
    // Termostati
    if (strcmp(topic, topicSeason[i].c_str())  == 0) { onSeasonOrHeating(i, true,  MU); return; }
    if (strcmp(topic, topicHeating[i].c_str()) == 0) { onSeasonOrHeating(i, false, MU); return; }

    // (Opzionale) override manuale da HA
    if (strcmp(topic, topicSet[i].c_str()) == 0) {
      if (Z[i].allowFromThermo) {
        bool on = (MU=="ON" || MU=="1" || MU=="TRUE");
        Z[i].commanded = on;
        Z[i].lastCmd   = millis();
        applyLogic();
      } else {
        publishState(i);
      }
      return;
    }
  }
}

void ensureMQTT() {
  if (mqtt.connected()) return;

  // Un tentativo ogni 3s, MAI bloccante: prima (while + delay interno) l'intero
  // loop() restava fermo finché il broker non rispondeva, quindi con MQTT giù
  // anche i pulsanti fisici (incluso l'ALL OFF di sicurezza) smettevano di
  // funzionare.
  static unsigned long lastAttempt = 0;
  if (millis() - lastAttempt < 3000) return;
  lastAttempt = millis();

  String cid = String("ESP8266_VALVOLE_") + String(ESP.getChipId(), HEX);
  bool ok = (strlen(mqtt_user) > 0)
    ? mqtt.connect(cid.c_str(), mqtt_user, mqtt_pass, tAvail, 1, true, "offline")
    : mqtt.connect(cid.c_str(), tAvail, 1, true, "offline");

  if (ok) {
    for (uint8_t i=0; i<4; i++) {
      mqtt.subscribe(topicSeason[i].c_str(), 1);
      mqtt.subscribe(topicHeating[i].c_str(), 1);
      mqtt.subscribe(topicSet[i].c_str(), 1); // override
    }
    mqtt.publish(tAvail, "online", true);
    publishAllStates(); // iniziale
  }
}

// ---------------- CONFIG (SPIFFS /config.json) ----------------
void saveConfig() {
  if (!SPIFFS.begin()) SPIFFS.begin();
  DynamicJsonDocument doc(384);
  doc["mqtt_server"] = mqtt_server;
  doc["mqtt_port"]   = mqtt_port;
  doc["mqtt_user"]   = mqtt_user;
  doc["mqtt_pass"]   = mqtt_pass;
  // manteniamo anche questi nel file, in caso servano in futuro:
  doc["topic_base"]  = topic_base;
  doc["thermo_base"] = thermo_base;

  File f = SPIFFS.open("/config.json", "w");
  if (f) { serializeJson(doc, f); f.close(); }
}

void loadConfig() {
  if (!SPIFFS.begin()) { SPIFFS.begin(); }
  if (!SPIFFS.exists("/config.json")) return;
  File f = SPIFFS.open("/config.json", "r");
  if (!f) return;
  DynamicJsonDocument doc(512);
  if (deserializeJson(doc, f) == DeserializationError::Ok) {
    strlcpy(mqtt_server, doc["mqtt_server"] | mqtt_server, sizeof(mqtt_server));
    strlcpy(mqtt_port,   doc["mqtt_port"]   | mqtt_port,   sizeof(mqtt_port));
    strlcpy(mqtt_user,   doc["mqtt_user"]   | mqtt_user,   sizeof(mqtt_user));
    strlcpy(mqtt_pass,   doc["mqtt_pass"]   | mqtt_pass,   sizeof(mqtt_pass));
    // opzionale
    topic_base = String(doc["topic_base"] | topic_base);
    strlcpy(thermo_base, doc["thermo_base"] | thermo_base, sizeof(thermo_base));
  }
  f.close();
}

// ---------------- WIFI MANAGER (identico ai termostati) ----------------
// allowFactoryReset=false quando richiamata dalla combo runtime Z1+Z4: PIN_RESET_BOOT
// (D3) è lo stesso pin del pulsante Sala, quindi su un pannello a 4 tasti
// adiacenti è plausibile che sia premuto insieme alla combo — senza questo
// parametro, quel "riapri portale soft" poteva trasformarsi in un factory
// reset silenzioso di WiFi+MQTT.
void startWiFiManager(bool allowFactoryReset) {
  WiFiManager wm;

  // se premi FLASH (GPIO0) all'avvio -> reset settings e config.json (come i termostati)
  pinMode(PIN_RESET_BOOT, INPUT_PULLUP);
  if (allowFactoryReset && digitalRead(PIN_RESET_BOOT) == LOW) {
    wm.resetSettings();
    if (SPIFFS.begin()) { SPIFFS.remove("/config.json"); }
    Serial.println("Config WiFi/MQTT resettata - riavvio");
    delay(800);
    ESP.restart();
  }

  // Parametri MQTT nel captive portal (precompilati), lunghezza allineata
  // alla dimensione reale dei buffer (mqtt_user/mqtt_pass sono char[32]).
  // type='password' sul campo pass solo per non mostrarla in chiaro sullo
  // schermo del portale (il traffico verso il broker resta comunque in chiaro,
  // vedi nota sicurezza nel README).
  WiFiManagerParameter p_server("server", "MQTT Server", mqtt_server, 40);
  WiFiManagerParameter p_port  ("port",   "MQTT Port",   mqtt_port,   6);
  WiFiManagerParameter p_user  ("user",   "MQTT User",   mqtt_user,  32);
  WiFiManagerParameter p_pass  ("pass",   "MQTT Pass",   mqtt_pass,  32, "type='password'");

  wm.addParameter(&p_server);
  wm.addParameter(&p_port);
  wm.addParameter(&p_user);
  wm.addParameter(&p_pass);

  wm.setConfigPortalTimeout(180);         // 3 minuti
  // Password sull'AP di configurazione: senza, chiunque nel raggio WiFi può
  // collegarsi al portale e ripuntare la centralina verso un broker arbitrario.
  if (!wm.autoConnect("ESP_Config", "pizaff123")) {
    Serial.println("Connessione fallita, riavvio...");
    ESP.restart();
  }

  // strlcpy, non strcpy: il "maxlength" del campo nel portale WiFiManager è
  // solo un attributo HTML lato client, non impedisce a un input costruito
  // ad arte (POST diretto) di superare la dimensione del buffer.
  strlcpy(mqtt_server, p_server.getValue(), sizeof(mqtt_server));
  strlcpy(mqtt_port,   p_port.getValue(),   sizeof(mqtt_port));
  strlcpy(mqtt_user,   p_user.getValue(),   sizeof(mqtt_user));
  strlcpy(mqtt_pass,   p_pass.getValue(),   sizeof(mqtt_pass));
  saveConfig();

  Serial.println("WiFi connesso!");
}

// ---------------- SETUP / LOOP ----------------
void setup() {
  Serial.begin(115200);
  delay(100);

  // Relè fail-safe: livello impostato PRIMA di pinMode(OUTPUT), altrimenti il
  // pin può passare per lo stato di default (LOW = relè ON, essendo
  // active-LOW) nella breve finestra tra pinMode() e la prima digitalWrite().
  for (uint8_t i=0; i<4; i++) {
    digitalWrite(RELAY_PINS[i], HIGH); // relè spento (active-LOW)
    pinMode(RELAY_PINS[i], OUTPUT);
    Z[i].relayOn=false; Z[i].commanded=false;
    Z[i].allowFromThermo = true;
    Z[i].lastChange=millis(); Z[i].lastCmd=millis();
  }

  // Pulsanti (pull-up)
  for (uint8_t i=0; i<4; i++) {
    pinMode(BUTTON_PINS[i], INPUT_PULLUP);
    Z[i].btnLast = digitalRead(BUTTON_PINS[i]);
    Z[i].btnLastChange = millis();
  }

  loadConfig();
  startWiFiManager(true); // <- CAPTIVE PORTAL IDENTICO AI TERMOSTATI, reset di fabbrica attivo
  buildTopics();          // topic MQTT calcolati una sola volta, dopo il portale

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);

  mqtt.setServer(mqtt_server, atoi(mqtt_port));
  mqtt.setCallback(mqttCallback);
  // Limita il tempo massimo di uno stallo di connect() (default libreria 15s):
  // resta comunque un'attesa sincrona, ma un tentativo fallito bloccato blocca
  // anche i pulsanti fisici molto meno a lungo.
  mqtt.setSocketTimeout(2);
}

void loop() {
  handleButtons();
  applyLogic();

  if (WiFi.status() == WL_CONNECTED) {
    ensureMQTT();
    mqtt.loop();
  }

  // Publish periodico stato ogni 5s
  if (millis() - lastStatePub >= STATE_PUB_INTERVAL) {
    lastStatePub = millis();
    if (mqtt.connected()) publishAllStates();
  }

  // watchdog Wi-Fi semplice
  static unsigned long tWifi = 0;
  if (millis() - tWifi > 30000) {
    tWifi = millis();
    if (WiFi.status() != WL_CONNECTED) {
      WiFi.disconnect();
      WiFi.begin();
    }
  }

  delay(20);
}
