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

#include <ESP8266WiFi.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <FS.h>
#include <ArduinoJson.h>
#include <math.h>

// ===== OLED 128x32 =====
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ===== Pulsante reset =====
#define PIN_RESET 0   // GPIO0 (FLASH)

// ===== MQTT Config (persistente in /config.json) =====
char mqtt_server[40] = "192.168.1.10";
char mqtt_port[6]    = "1883";
char mqtt_user[20]   = "";
char mqtt_pass[20]   = "";

WiFiClient espClient;
PubSubClient mqtt(espClient);

// ===== Topics =====
const char* tTemp        = "termostato/CameraPiccola/temperature";
const char* tSetpoint    = "termostato/CameraPiccola/setpoint";
const char* tSetpointCmd = "termostato/CameraPiccola/setpoint_cmd";
const char* tHeating     = "termostato/CameraPiccola/heating";    // ON/OFF richiesta
const char* tMode        = "termostato/CameraPiccola/mode";       // off | heat | cool
const char* tModeCmd     = "termostato/CameraPiccola/mode_cmd";   // off | heat | cool
const char* tSeason      = "termostato/CameraPiccola/season";     // winter | summer
const char* tSeasonCmd   = "termostato/CameraPiccola/season_cmd"; // winter | summer
const char* tAvail       = "termostato/CameraPiccola/availability"; // online | offline (LWT)

// ===== DS18B20 =====
#define PIN_ONEWIRE 16 // D0
OneWire oneWire(PIN_ONEWIRE);
DallasTemperature sensors(&oneWire);

// Lettura non bloccante (vedi loop()): fase corrente e istante di avvio conversione
enum SensorPhase { SENSOR_IDLE, SENSOR_WAITING };
SensorPhase sensorPhase = SENSOR_IDLE;
unsigned long tConvStart = 0;
const unsigned long CONV_MS = 750; // tempo conversione DS18B20 a risoluzione 12 bit

// ===== Encoder =====
#define PIN_ENC_CLK 5   // D1
#define PIN_ENC_DT  4   // D2
#define PIN_ENC_SW  13  // D7

// Decoding robusto (Gray, 4 micro-step = 1 scatto)
volatile int8_t encAccum = 0;
volatile int8_t encDelta = 0;
static uint8_t  encState = 0;

// ===== Stato =====
float tempC = NAN;
float setpointC = 21.0;
const float HYST = 0.3;
bool callForHeat = false;   // richiesta (in inverno = caldaia; in estate = clima)
bool enabled = true;
String season = "winter";   // "winter" = riscaldamento, "summer" = raffrescamento

enum UIMode { SHOW_AMBIENT, SHOW_SETPOINT, SHOW_SEASON, SHOW_ALERT };
UIMode ui = SHOW_AMBIENT;
unsigned long lastInputMs = 0;
const unsigned long UI_TIMEOUT = 3000;

// Monitor connessioni / avvisi
bool wasWiFi = false, wasMqtt = false;
unsigned long alertUntilMs = 0;
const unsigned long ALERT_MS = 2500;

// Persistenza setpoint/season/enabled: scrittura differita e "debounced" (non
// a ogni scatto encoder) per non consumare inutilmente i cicli di scrittura
// della flash SPIFFS.
bool configDirty = false;
unsigned long lastConfigSave = 0;
const unsigned long CONFIG_SAVE_DEBOUNCE_MS = 5000;

// Icona 🔥
const uint8_t ICON_FLAME[8] = {
  0b00011000,
  0b00011100,
  0b00111110,
  0b00111110,
  0b00111110,
  0b00011100,
  0b00001000,
  0b00000000
};

// Icona ❄️ (corretta)
const uint8_t ICON_SNOW[8] = {
  0b00101010,
  0b00011100,
  0b11111111,
  0b00111110,
  0b11111111,
  0b00011100,
  0b00101010,
  0b00000000
};

// ===== Helper UI =====
void showMsg(const char* l1, const char* l2 = "") {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 8);
  display.println(l1);
  if (l2 && l2[0]) {
    display.setCursor(0, 20);
    display.println(l2);
  }
  display.display();
}

void showPortalScreen(const char* ssid){
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(F("CONFIGURA DA WEB"));
  display.println();
  display.print(F("SSID: "));
  display.println(ssid);
  display.println(F("Apri: 192.168.4.1"));
  display.display();
}

// ---- UI principale ----
void drawNumberCenter(float value, bool isSetpoint){
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(3);
  char buf[12];
  dtostrf(value, 0, 1, buf);
  int16_t x1,y1; uint16_t w,h;
  display.getTextBounds(buf, 0, 0, &x1,&y1,&w,&h);
  int x = (SCREEN_WIDTH - w)/2;
  int y = (SCREEN_HEIGHT - h)/2 - 4;
  display.setCursor(x, y);
  display.print(buf);

  // Icona stagione quando c'è richiesta
  if (enabled && callForHeat) {
    if (season == "winter") display.drawBitmap(118, 2, ICON_FLAME, 8, 8, SSD1306_WHITE);
    else                    display.drawBitmap(118, 2, ICON_SNOW,  8, 8, SSD1306_WHITE);
  }

  // Indicatori in basso a destra: Wi-Fi (cerchio) e MQTT (quadrato)
  int bx = 116, by = 29;
  if (WiFi.status() == WL_CONNECTED) display.fillCircle(bx, by, 2, SSD1306_WHITE);
  else                               display.drawCircle(bx, by, 2, SSD1306_WHITE);
  if (mqtt.connected()) display.fillRect(bx+8, by-2, 5, 5, SSD1306_WHITE);
  else                  display.drawRect(bx+8, by-2, 5, 5, SSD1306_WHITE);

  if (isSetpoint) display.drawLine(0, 0, SCREEN_WIDTH, 0, SSD1306_WHITE);

  display.display();
}

void drawSeason(){
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(10, 10);
  if (season == "winter") {
    display.print("INVERNO");
    display.drawBitmap(100, 0, ICON_FLAME, 8, 8, SSD1306_WHITE);
  } else {
    display.print("ESTATE");
    display.drawBitmap(100, 0, ICON_SNOW, 8, 8, SSD1306_WHITE);
  }
  display.display();
}

// ===== Salvataggio config MQTT =====
void saveConfig() {
  DynamicJsonDocument doc(384);
  doc["mqtt_server"] = mqtt_server;
  doc["mqtt_port"]   = mqtt_port;
  doc["mqtt_user"]   = mqtt_user;
  doc["mqtt_pass"]   = mqtt_pass;
  // setpoint/season/enabled: senza questi, un riavvio (blackout, crash) fa
  // ripartire il termostato sempre a 21.0°C finché Home Assistant non rinvia
  // un comando, con il rischio di richiedere riscaldamento non voluto nel
  // frattempo.
  doc["setpoint"] = setpointC;
  doc["season"]   = season;
  doc["enabled"]  = enabled;
  File f = SPIFFS.open("/config.json", "w");
  if (!f) return;
  serializeJson(doc, f);
  f.close();
}

void loadConfig() {
  if (!SPIFFS.exists("/config.json")) return;
  File f = SPIFFS.open("/config.json", "r");
  if (!f) return;
  DynamicJsonDocument doc(384);
  if (deserializeJson(doc, f)) { f.close(); return; }
  strlcpy(mqtt_server, doc["mqtt_server"] | "192.168.1.10", sizeof(mqtt_server));
  strlcpy(mqtt_port,   doc["mqtt_port"]   | "1883", sizeof(mqtt_port));
  strlcpy(mqtt_user,   doc["mqtt_user"]   | "", sizeof(mqtt_user));
  strlcpy(mqtt_pass,   doc["mqtt_pass"]   | "", sizeof(mqtt_pass));
  setpointC = doc["setpoint"] | 21.0;
  season    = String((const char*)(doc["season"] | "winter"));
  enabled   = doc["enabled"]  | true;
  f.close();
}

// ---- Logica (stagione-aware) ----
void updateHeating(){
  if (!enabled || isnan(tempC)) { callForHeat = false; return; }

  if (season == "winter") {
    if (tempC < (setpointC - HYST)) callForHeat = true;
    else if (tempC > (setpointC + HYST)) callForHeat = false;
  } else {
    if (tempC > (setpointC + HYST)) callForHeat = true;
    else if (tempC < (setpointC - HYST)) callForHeat = false;
  }
}

// Ricalcola e ripubblica subito lo stato dopo un comando (setpoint/mode/season):
// prima veniva ripubblicato solo nel giro periodico ogni 10s, quindi un "off"
// da Home Assistant poteva impiegare fino a ~12s per riflettersi su tHeating.
void publishHeatingState(){
  updateHeating();
  mqtt.publish(tHeating, (enabled && callForHeat) ? "ON" : "OFF", true);
}

// ---- Encoder con tabella Gray ----
static const int8_t ENC_TAB[16] = {
  0, -1, +1,  0,
 +1,  0,  0, -1,
 -1,  0,  0, +1,
  0, +1, -1,  0
};

void pollEncoder(){
  // Canali: A=CLK (bit1), B=DT (bit0)
  uint8_t a = digitalRead(PIN_ENC_CLK);
  uint8_t b = digitalRead(PIN_ENC_DT);
  uint8_t s = (a << 1) | b;

  encState = ((encState << 2) | s) & 0x0F;      // conserva ultimi 4 bit
  int8_t step = ENC_TAB[encState];              // -1 / 0 / +1
  if (step != 0) {
    encAccum += step;
    if (encAccum >= 4) { encAccum = 0; encDelta++; }
    else if (encAccum <= -4) { encAccum = 0; encDelta--; }
    lastInputMs = millis();
    ui = SHOW_SETPOINT;
  }

  // Pulsante: breve/lunga
  static bool lastSw = HIGH;
  bool sw = digitalRead(PIN_ENC_SW);
  static unsigned long tPress = 0;

  if (lastSw == HIGH && sw == LOW) {
    tPress = millis();
  }
  if (lastSw == LOW && sw == HIGH) {
    unsigned long d = millis() - tPress;
    if (d >= 2000) {
      // Toggle stagione (long press)
      season = (season == "winter") ? "summer" : "winter";
      enabled = true;
      mqtt.publish(tMode, enabled ? (season=="winter" ? "heat" : "cool") : "off", true);
      mqtt.publish(tSeason, season.c_str(), true);
      publishHeatingState();
      configDirty = true;
      ui = SHOW_SEASON;
    }
    // pressione breve: nessuna azione (solo "focus" sul setpoint, già gestito
    // dalla rotazione dell'encoder)
    lastInputMs = millis();
  }
  lastSw = sw;
}

// ---- MQTT ----
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // Copiare in un buffer locale invece di scrivere "payload[length] = '\0'"
  // in-place: quel pattern (comune negli esempi PubSubClient) scrive un byte
  // oltre l'ultimo ricevuto e, se un messaggio riempisse esattamente
  // MQTT_MAX_PACKET_SIZE, andrebbe fuori dal buffer allocato dalla libreria.
  char buf[64];
  if (length >= sizeof(buf)) length = sizeof(buf) - 1;
  memcpy(buf, payload, length);
  buf[length] = '\0';

  String t = String(topic);
  String msg = String(buf);

  if (t == tSetpointCmd) {
    // strtod invece di toFloat(): su una stringa non numerica toFloat()
    // ritorna silenziosamente 0.0, che finiva "clampato" a 5.0°C cambiando
    // il setpoint senza alcun segnale d'errore.
    char* endPtr = nullptr;
    float v = strtod(msg.c_str(), &endPtr);
    if (endPtr != msg.c_str()) {
      v = constrain(v, 5.0, 30.0);
      setpointC = v;
      configDirty = true;
      ui = SHOW_SETPOINT; lastInputMs = millis();
      mqtt.publish(tSetpoint, String(setpointC,1).c_str(), true);
      publishHeatingState();
    }

  } else if (t == tModeCmd) {
    msg.toLowerCase();
    if (msg == "off") {
      enabled = false;
    } else if (msg == "heat") {
      enabled = true;  season = "winter";
    } else if (msg == "cool") {
      enabled = true;  season = "summer";
    }
    mqtt.publish(tMode, enabled ? (season=="winter" ? "heat" : "cool") : "off", true);
    mqtt.publish(tSeason, season.c_str(), true);
    publishHeatingState();
    configDirty = true;

  } else if (t == tSeasonCmd) {
    msg.toLowerCase();
    if (msg == "winter" || msg == "summer") {
      season = msg;
      enabled = true;
      mqtt.publish(tSeason, season.c_str(), true);
      mqtt.publish(tMode, (season=="winter") ? "heat" : "cool", true);
      publishHeatingState();
      configDirty = true;
      ui = SHOW_SEASON;
      lastInputMs = millis();
    }
  }
}

void ensureMqtt() {
  if (mqtt.connected()) return;

  // Un tentativo ogni 3s, MAI bloccante: prima (while + delay interno) l'intero
  // loop() restava fermo finché il broker non rispondeva, quindi con MQTT giù
  // si fermavano anche lettura sensore, encoder, display e watchdog WiFi.
  static unsigned long lastAttempt = 0;
  if (millis() - lastAttempt < 3000) return;
  lastAttempt = millis();

  String cid = String("thermo-") + String(ESP.getChipId(), HEX);

  bool ok;
  // Usa overload con LWT (compatibile con versioni “classiche” di PubSubClient)
  if (strlen(mqtt_user) > 0) {
    // connect(id, user, pass, willTopic, willQos, willRetain, willMessage)
    ok = mqtt.connect(
      cid.c_str(),
      mqtt_user, mqtt_pass,
      tAvail,     1,         true,       "offline"
    );
  } else {
    // connect(id, willTopic, willQos, willRetain, willMessage)
    ok = mqtt.connect(
      cid.c_str(),
      tAvail,     1,         true,       "offline"
    );
  }

  if (ok) {
    mqtt.subscribe(tSetpointCmd);
    mqtt.subscribe(tModeCmd);
    mqtt.subscribe(tSeasonCmd);

    // Annuncia online + stato iniziale
    mqtt.publish(tAvail, "online", true);
    mqtt.publish(tSetpoint, String(setpointC,1).c_str(), true);
    mqtt.publish(tMode, enabled ? (season=="winter" ? "heat" : "cool") : "off", true);
    mqtt.publish(tSeason, season.c_str(), true);
    publishHeatingState(); // prima mancava: la centralina restava sull'ultimo
                            // "heating" retenuto fino al giro periodico (10s)
  }
}

// ---- Setup/Loop ----
void setup(){
  Serial.begin(115200);
  SPIFFS.begin();
  pinMode(PIN_RESET, INPUT_PULLUP);

  if (digitalRead(PIN_RESET) == LOW) {
    WiFiManager wm;
    wm.resetSettings();
    SPIFFS.remove("/config.json");
    showMsg("Config resettata", "Riavvio...");
    delay(800);
    ESP.restart();
  }

  Wire.begin(12,14); // SDA=D6(GPIO12), SCL=D5(GPIO14)
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("SSD1306 non trovato");
  }
  showMsg("Avvio...", "Display OK");

  loadConfig();

  WiFiManager wm;
  wm.setAPCallback([&](WiFiManager *wmp){
    showPortalScreen(wmp->getConfigPortalSSID().c_str());
  });
  wm.setSaveConfigCallback([&](){
    showMsg("Salvataggio...", "Connessione WiFi");
  });

  WiFiManagerParameter p_server("server", "MQTT Server", mqtt_server, 40);
  WiFiManagerParameter p_port  ("port",   "MQTT Port",   mqtt_port,   6);
  WiFiManagerParameter p_user  ("user",   "MQTT User",   mqtt_user,   20);
  // type='password' solo per non mostrare la password in chiaro sullo schermo
  // del portale: il traffico verso il broker resta comunque non cifrato (vedi
  // nota sicurezza nel README).
  WiFiManagerParameter p_pass  ("pass",   "MQTT Pass",   mqtt_pass,   20, "type='password'");
  wm.addParameter(&p_server);
  wm.addParameter(&p_port);
  wm.addParameter(&p_user);
  wm.addParameter(&p_pass);

  wm.setConfigPortalTimeout(180);
  // Password sull'AP di configurazione: senza, chiunque nel raggio WiFi può
  // collegarsi al portale e ripuntare il dispositivo verso un broker arbitrario.
  if (!wm.autoConnect("ESP_Config", "pizaff123")) ESP.restart();

  // strlcpy, non strcpy: il limite di lunghezza del campo nel portale WiFiManager
  // è solo un attributo HTML "maxlength" lato client, non impedisce a un input
  // costruito ad arte di superare la dimensione del buffer.
  strlcpy(mqtt_server, p_server.getValue(), sizeof(mqtt_server));
  strlcpy(mqtt_port,   p_port.getValue(),   sizeof(mqtt_port));
  strlcpy(mqtt_user,   p_user.getValue(),   sizeof(mqtt_user));
  strlcpy(mqtt_pass,   p_pass.getValue(),   sizeof(mqtt_pass));
  saveConfig();

  showMsg("WiFi connesso", "MQTT in connessione");

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);

  mqtt.setServer(mqtt_server, atoi(mqtt_port));
  mqtt.setCallback(mqttCallback);
  mqtt.setKeepAlive(30);
  mqtt.setSocketTimeout(15);

  sensors.begin();
  sensors.setWaitForConversion(false); // lettura a due fasi non bloccante, vedi loop()
  pinMode(PIN_ENC_CLK, INPUT_PULLUP);
  pinMode(PIN_ENC_DT,  INPUT_PULLUP);
  pinMode(PIN_ENC_SW,  INPUT_PULLUP);

  // Stato iniziale indicatori
  wasWiFi = (WiFi.status() == WL_CONNECTED);
  wasMqtt = mqtt.connected();

  drawNumberCenter(0.0, false);
}

unsigned long tRead=0, tPub=0;
void loop(){
  // --- Stato connessioni attuale
  bool nowWiFi = (WiFi.status() == WL_CONNECTED);
  bool nowMqtt = mqtt.connected();

  // Cambi di stato -> mostra alert
  if ((nowWiFi != wasWiFi) || (nowMqtt != wasMqtt)) {
    if (nowWiFi && nowMqtt) {
      showMsg("RICONNESSO", "");
    } else {
      const char* who = (!nowWiFi && !nowMqtt) ? "WiFi+MQTT" : (!nowWiFi ? "WiFi" : "MQTT");
      showMsg("DISCONNESSO", who);
    }
    alertUntilMs = millis() + ALERT_MS;
    ui = SHOW_ALERT;
    wasWiFi = nowWiFi; wasMqtt = nowMqtt;
  }

  // Tenta di assicurare il MQTT (può riconnettere)
  ensureMqtt();
  mqtt.loop();

  // --- Encoder / input
  pollEncoder();
  if (encDelta != 0) {
    int d = encDelta; encDelta = 0;
    setpointC += d * 0.5;                      // 0.5°C per scatto completo
    setpointC = constrain(setpointC, 5.0, 30.0);
    mqtt.publish(tSetpoint, String(setpointC,1).c_str(), true);
    publishHeatingState();
    configDirty = true;
  }

  // --- UI timeout
  if (ui == SHOW_SETPOINT && (millis() - lastInputMs > UI_TIMEOUT)) ui = SHOW_AMBIENT;
  if (ui == SHOW_SEASON   && (millis() - lastInputMs > 2000))      ui = SHOW_AMBIENT;
  if (ui == SHOW_ALERT    && (millis() > alertUntilMs))            ui = SHOW_AMBIENT;

  unsigned long now = millis();

  // Lettura sensore ogni 2s, in due fasi non bloccanti: sensors.requestTemperatures()
  // di default ASPETTA la conversione (~750ms, il 37% del tempo ogni 2s) fermando
  // encoder/display/mqtt.loop() per tutto quel tempo. Con setWaitForConversion(false)
  // (impostato in setup) si avvia la conversione e si legge il risultato dopo
  // CONV_MS senza mai bloccare loop().
  if (sensorPhase == SENSOR_IDLE && now - tRead > 2000) {
    tRead = now;
    sensors.requestTemperatures();
    tConvStart = now;
    sensorPhase = SENSOR_WAITING;
  } else if (sensorPhase == SENSOR_WAITING && now - tConvStart >= CONV_MS) {
    sensorPhase = SENSOR_IDLE;
    float raw = sensors.getTempCByIndex(0);

    // DallasTemperature segnala sensore assente/guasto con -127.0, non con NaN:
    // senza questo controllo il termostato leggerebbe "-127°C" come temperatura
    // vera e richiederebbe riscaldamento continuo in modalità inverno.
    if (raw == DEVICE_DISCONNECTED_C) {
      tempC = NAN;
    } else {
      const float TEMP_OFFSET = -2.5;  // calibrazione
      tempC = raw + TEMP_OFFSET;
    }

    updateHeating();
  }

  // Salvataggio differito di setpoint/season/enabled: solo se qualcosa è
  // cambiato e non più spesso di una volta ogni 5s, per non consumare i
  // cicli di scrittura della flash a ogni scatto di encoder.
  if (configDirty && (now - lastConfigSave > CONFIG_SAVE_DEBOUNCE_MS)) {
    configDirty = false;
    lastConfigSave = now;
    saveConfig();
  }

  // --- Rendering
  if (ui == SHOW_ALERT) {
    // già disegnato da showMsg()
  } else if (ui == SHOW_SETPOINT) {
    drawNumberCenter(setpointC, true);
  } else if (ui == SHOW_SEASON) {
    drawSeason();
  } else if (isnan(tempC)) {
    showMsg("Sensore", "non rilevato");
  } else {
    drawNumberCenter(tempC, false);
  }

  // Pubblicazioni periodiche ogni 10s
  if (now - tPub > 10000) {
    tPub = now;
    if (!isnan(tempC)) mqtt.publish(tTemp, String(tempC,2).c_str(), true);
    mqtt.publish(tHeating, (enabled && callForHeat) ? "ON" : "OFF", true);
    mqtt.publish(tMode, enabled ? (season=="winter" ? "heat" : "cool") : "off", true);
    mqtt.publish(tSeason, season.c_str(), true);
  }

  // Mantieni availability coerente (ping ogni 60s – non indispensabile)
  static unsigned long tAvailPing = 0;
  if (mqtt.connected() && millis() - tAvailPing > 60000) {
    tAvailPing = millis();
    mqtt.publish(tAvail, "online", true);
  }

  // --- Watchdog Wi-Fi ---
  static unsigned long lastWiFiCheck = 0;
  if (millis() - lastWiFiCheck > 30000) {
    lastWiFiCheck = millis();
    if (WiFi.status() != WL_CONNECTED) {
      WiFi.disconnect();
      WiFi.begin();
    }
  }
  // Retry più rapido ogni 3s se disconnesso
  static unsigned long lastFastCheck = 0;
  if (WiFi.status() != WL_CONNECTED && millis() - lastFastCheck > 3000) {
    lastFastCheck = millis();
    WiFi.disconnect();
    WiFi.begin();
  }

  delay(1);
}

