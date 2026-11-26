# PiZaff Termostati MQTT — termostato da stanza + centralina 4 relè

Sistema di termostati smart **fai-da-te** per Home Assistant, su ESP8266,
senza cloud e senza abbonamenti: ogni stanza ha un termostato con sensore di
temperatura ed encoder, che parla via **MQTT** con una centralina che accende
i relè del riscaldamento.

- **Termostato da stanza** — legge la temperatura (DS18B20), mostra tutto su
  un piccolo OLED, si regola con un encoder rotativo, pubblica/riceve stato e
  comandi via MQTT. **Un solo firmware per tutte le stanze**: la zona si
  sceglie dal portale web.
- **Centralina 4 relè** — un unico ESP8266 che pilota fisicamente il
  riscaldamento di 4 zone (relè), leggendo via MQTT cosa chiede ciascun
  termostato. Gestisce anche 4 pulsanti locali per comando manuale.

Nessun server proprio da mantenere: basta un broker MQTT qualunque (es.
**Mosquitto** dentro Home Assistant) e la rete WiFi di casa.

Entrambi i firmware si aggiornano **via OTA** (da Arduino IDE o caricando il
`.bin` dal portale web).

---

## Contenuto del repository

| Percorso | Cosa contiene |
|---|---|
| `sketch_termiostati_mqtt_per_homeassistant_V1/` | Firmware del termostato da stanza (ESP8266 + DS18B20 + OLED + encoder) |
| `centralina_termostati_4rele/` | Firmware della centralina (ESP8266 + 4 relè + 4 pulsanti) |
| `homeassistant/` | Esempi di configurazione Home Assistant |
| `images/` | Foto della realizzazione |
| `termostato v1 v13.3mf` | Case del termostato pronto per la stampa 3D (progetto slicer) |

---

## Architettura

```
┌─────────────────────┐        MQTT        ┌──────────────────────┐
│  Termostato Bagno    │ ───publish/sub───▶ │                      │
│  Termostato Sala     │ ───publish/sub───▶ │   Broker MQTT         │
│  Termostato Camera.. │ ───publish/sub───▶ │  (es. Mosquitto/HA)   │
│  Termostato Camera.. │ ───publish/sub───▶ │                      │
└─────────────────────┘                    └──────────┬───────────┘
                                                        │ publish/sub
                                                        ▼
                                            ┌──────────────────────┐
                                            │  Centralina 4 relè    │
                                            │  (accende il vero     │
                                            │   riscaldamento)      │
                                            └──────────────────────┘
```

Ogni termostato **non pilota nessun relè**: decide solo se "vorrebbe"
riscaldamento acceso e lo pubblica su MQTT. È la centralina a decidere se e
quando accendere fisicamente ciascuna zona, con le sue regole di sicurezza
(tempi minimi, spegnimento di emergenza, timeout).

Home Assistant si collega allo stesso broker MQTT e vede/comanda tutto:
è il punto in cui imposti orari, automazioni, presenza, ecc. — i due firmware
si limitano a eseguire.

Qualunque cosa pubblichi `termostato/<Zona>/heating` e `.../season` nel
formato giusto può sostituire un termostato ESP: per esempio uno **Shelly
Wall Display** tramite un'automazione di Home Assistant
(vedi `homeassistant/sala_x2i_bridge.yaml`).

---

## Hardware

### Termostato da stanza
- **NodeMCU ESP8266**
- Sensore di temperatura **DS18B20** (OneWire) + resistenza pull-up 4.7 kΩ tra DQ e VCC
- Display **OLED 128×32 SSD1306** (I²C)
- **Encoder rotativo** con pulsante (setpoint, stagione, portale)

```
Wire.begin(12,14):  SDA = D6 (GPIO12)   SCL = D5 (GPIO14)   [OLED]
PIN_ONEWIRE = D0 (GPIO16)                                   [DS18B20, DQ]
PIN_ENC_CLK = D1 (GPIO5)     (interrupt)
PIN_ENC_DT  = D2 (GPIO4)     (interrupt)
PIN_ENC_SW  = D7 (GPIO13)
PIN_FLASH   = GPIO0 (FLASH)  tenuto premuto 5 s = reset di fabbrica
```

> Il DS18B20 vicino all'ESP8266 legge qualche grado in più per il calore
> della scheda: per questo esiste la **correzione sensore** nel portale
> (default −2.5 °C). Meglio ancora: sensore su un cavetto, lontano dall'ESP.

### Centralina 4 relè
- **NodeMCU ESP8266**
- Modulo relè **4 canali, active-LOW** (IN1..IN4 a livello logico invertito)
- 4 pulsanti (uno per zona), verso GND, pull-up interno

```
Zona (default)  Relè (uscita)     Pulsante (ingresso)
Bagno           D0 (GPIO16)       D6 (GPIO12)
CameraPiccola   D1 (GPIO5)        D7 (GPIO13)
Sala            D2 (GPIO4)        D3 (GPIO0)   ⚠️ vedi nota sotto
CameraGrande    D5 (GPIO14)       D4 (GPIO2)   ⚠️ vedi nota sotto
```

> **Nota GPIO0 (pulsante Sala):** se è premuto mentre la centralina si
> accende, l'ESP8266 entra in modalità programmazione e non parte.
> **Nota GPIO2 (pulsante CameraGrande):** deve restare alto al boot; un
> pulsante con pull-up interno (come qui) va benissimo.

### Consigli di alimentazione (stabilità)
Riavvii casuali "ogni tot giorni" spesso sono elettrici, non software:
- alimentatore 5 V decente (≥1 A), non una porta USB qualsiasi;
- modulo relè con **JD-VCC separato** (togli il jumper, 5 V alle bobine,
  3.3 V alla logica degli optoisolatori) così l'uscita a 3.3 V dell'ESP
  spegne davvero il relè;
- **snubber RC / varistore** sui carichi induttivi a 230 V (valvole di zona,
  contattori): i disturbi allo stacco resettano l'ESP.

Il campo `reset` nel topic `.../info` dice il motivo dell'ultimo riavvio
(`Power On`, `Software/System restart`, `Hardware Watchdog`, `Exception`…).

---

## Compilazione e primo flash (via USB, una sola volta)

Progetti **Arduino IDE** (o arduino-cli) per **ESP8266**, core **3.1.x**.

Impostazioni scheda (Strumenti):
- **Scheda:** *NodeMCU 1.0 (ESP-12E Module)*
- **Flash Size:** **4MB (FS:2MB OTA:~1019KB)** — usa sempre questa: se cambi
  la dimensione del filesystem la configurazione salvata viene persa.

Librerie (Library Manager):
- `WiFiManager` (tzapu) **2.0.x**
- `PubSubClient` (Nick O'Leary)
- `ArduinoJson` (Benoit Blanchon, v6 o v7)
- Solo per il termostato: `Adafruit SSD1306`, `Adafruit GFX Library`, `OneWire`, `DallasTemperature`

`LittleFS` e `ArduinoOTA` sono già inclusi nel core ESP8266.

Il **primo** caricamento del firmware v2 va fatto via USB. Da lì in poi si
aggiorna via rete (vedi *Aggiornamento OTA*).

> **Aggiornando da v1:** al primo avvio il firmware converte la memoria da
> SPIFFS a LittleFS e **recupera da solo** WiFi e credenziali MQTT. Sul
> termostato si apre il portale per chiedere la **zona** (in v1 era scritta
> nel codice): è l'unica cosa da reinserire.

---

## Portale di configurazione

Si apre da solo al primo avvio (o dopo un reset di fabbrica). Si può
riaprire in qualsiasi momento **senza fermare il dispositivo**:
- **Termostato:** tieni premuto il pulsante dell'encoder **8 secondi**;
- **Centralina:** tieni premuti insieme **Bagno + CameraGrande** per **3 secondi**.

Aperto a mano, si chiude da solo dopo 5 minuti.

1. Connettiti alla rete **`ESP_Config-XXXX`** (XXXX = ultime cifre del chip,
   le vedi sul display del termostato), password **`pizaff123`**.
2. Vai su **http://192.168.4.1**. Il menu ha solo le voci essenziali:
   - **Configure WiFi** — scegli la rete di casa e la password;
   - **Setup** — impostazioni del dispositivo:
     - *MQTT:* server, porta, utente, password;
     - *Termostato:* zona e correzione sensore (°C);
       *Centralina:* nomi delle 4 zone (uguali a quelli dei termostati);
     - *Aggiornamento OTA:* password OTA;
   - **Update** — carica un nuovo firmware `.bin` dal browser;
   - **Restart** / **Exit**.
3. Salva: il dispositivo si riavvia con la nuova configurazione.

Se la password WiFi è sbagliata il portale **resta aperto** e puoi
correggerla. Le credenziali stanno solo nella flash del dispositivo
(`/net.json`), mai nel codice sorgente.

**Zona:** lettere, numeri, `_` e `-` (es. `Sala`, `CameraGrande`). Diventa
parte dei topic MQTT (`termostato/Sala/...`) e del nome di rete
(`termostato-sala`).

**Reset di fabbrica** (cancella WiFi, MQTT e impostazioni):
- **Termostato:** tieni premuto il tasto **FLASH** per 5 secondi;
- **Centralina:** tieni premuti **Bagno + CameraGrande** per 15 secondi.

---

## Aggiornamento OTA

Due modi, entrambi senza cavo:

1. **Da Arduino IDE** — imposta una *Password OTA* nel portale. Il
   dispositivo compare tra le porte come `termostato-<zona>` o
   `centralina-risc`: selezionalo e premi *Carica* (chiede la password OTA).
   Se il PC è in un'altra VLAN, l'mDNS non passa: carica indicando l'IP con
   `espota.py -i <IP> -a <password> -f firmware.bin` (lo trovi nel core
   ESP8266, cartella `tools/`). Serve la porta UDP/TCP 8266 aperta verso il
   dispositivo.
2. **Dal portale web** — *Sketch → Esporta binario compilato* in Arduino
   IDE, poi apri il portale e usa **Update**.

Durante l'aggiornamento il termostato mostra la barra di avanzamento; la
centralina **spegne tutti i relè** prima di iniziare. Al riavvio tutto
riparte dallo stato salvato.

---

## Uso quotidiano

### Termostato da stanza
- **Ruota l'encoder** → il primo scatto mostra il setpoint, i successivi lo
  cambiano (0.5 °C a scatto).
- **Pressione lunga (2–8 s)** sul pulsante encoder → alterna inverno/estate.
- **Pressione 8 s** → apre il portale di configurazione.
- Il display mostra la temperatura ambiente, con 🔥/❄️ quando c'è richiesta
  attiva, `OFF` se il termostato è spento, e due indicatori in basso a destra
  (WiFi ● / MQTT ■, pieni = connessi).
- Dopo 1 minuto senza interazioni il display si **attenua**; il numero si
  sposta di qualche pixel ogni 2 minuti (anti burn-in dell'OLED).

### Centralina
- **Pressione breve** su un pulsante zona → accende/spegne manualmente quella
  zona (bypassa i tempi minimi, utile in manutenzione).
- **Pressione lunga (≥2s)** su un pulsante qualsiasi → **spegnimento
  immediato di tutte le zone** (sicurezza).
- Le zone seguono automaticamente i comandi dei termostati via MQTT.

---

## Protocollo MQTT

Ogni termostato pubblica/riceve sotto `termostato/<Zona>/...`:

| Topic | Direzione | Valori | Note |
|---|---|---|---|
| `.../temperature` | termostato → HA | numero (°C) | retained |
| `.../setpoint` | termostato → HA | numero (°C) | retained |
| `.../setpoint_cmd` | HA → termostato | numero (°C), 5–30 | |
| `.../heating` | termostato → **centralina** | `ON` / `OFF` | richiesta, retained |
| `.../action` | termostato → HA | `heating` / `cooling` / `idle` / `off` | retained |
| `.../mode` | termostato → HA | `off` / `heat` / `cool` | retained |
| `.../mode_cmd` | HA → termostato | `off` / `heat` / `cool` | |
| `.../season` | termostato → **centralina** | `winter` / `summer` | retained |
| `.../season_cmd` | HA → termostato | `winter` / `summer` | |
| `.../sensor` | termostato → HA | `ok` / `error` | retained |
| `.../availability` | termostato → HA + centralina | `online` / `offline` | LWT |
| `.../info` | termostato → HA | JSON (fw, ip, rssi, reset, heap, uptime) | retained, ogni 5 min |

La centralina, per ogni zona, pubblica sotto `riscaldamento/<Zona>/...`:

| Topic | Direzione | Valori | Note |
|---|---|---|---|
| `.../state` | centralina → HA | `ON` / `OFF` | stato reale del relè, retained |
| `.../set` | HA → centralina | `ON` / `OFF` | override manuale, valido solo in `winter` |

Più `riscaldamento/centralina/availability` (`online`/`offline`, LWT) e
`riscaldamento/centralina/info` (JSON diagnostico).

### Esempio configurazione Home Assistant (per zona)

```yaml
mqtt:
  climate:
    - name: "Termostato Camera Piccola"
      unique_id: termostato_camerapiccola
      modes: ["off", "heat", "cool"]
      current_temperature_topic: "termostato/CameraPiccola/temperature"
      temperature_command_topic: "termostato/CameraPiccola/setpoint_cmd"
      temperature_state_topic: "termostato/CameraPiccola/setpoint"
      mode_command_topic: "termostato/CameraPiccola/mode_cmd"
      mode_state_topic: "termostato/CameraPiccola/mode"
      action_topic: "termostato/CameraPiccola/action"
      availability_topic: "termostato/CameraPiccola/availability"
      min_temp: 5
      max_temp: 30
      temp_step: 0.5
  binary_sensor:
    - name: "Sensore Camera Piccola guasto"
      unique_id: termostato_camerapiccola_sensor
      state_topic: "termostato/CameraPiccola/sensor"
      payload_on: "error"
      payload_off: "ok"
      device_class: problem
```

Ripeti per ciascuna zona cambiando `CameraPiccola` con il nome impostato nel
portale di quel termostato.

---

## Come funziona

### Isteresi e stagione (termostato)

La richiesta diventa vera/falsa con **0.3 °C di isteresi** attorno al
setpoint, invertita tra le due stagioni: in `winter` scalda sotto il
setpoint, in `summer` raffresca sopra. Il calcolo è sempre locale sul
termostato: funziona anche se la centralina o il broker sono momentaneamente
irraggiungibili.

Il sensore è considerato guasto dopo **3 letture errate consecutive**
(assente = −127, valore di accensione 85 °C, fuori range): la richiesta va a
`OFF`, il display mostra "Sensore non rilevato" e su MQTT esce
`sensor = error`. Una singola lettura sbagliata viene ignorata.

### Tempi minimi e sfasamento (centralina)

- **ON minimo 3 minuti**, **OFF minimo 2 minuti** per zona (anti-ciclo).
- Accensioni **sfasate di 800 ms** l'una dall'altra.
- **Timeout di sicurezza 15 minuti**: se una zona non riceve un comando
  `heating` aggiornato entro questa finestra, si spegne da sola. Ogni
  termostato ripubblica il proprio stato ogni 10 s.
- **Termostato offline** (LWT `availability = offline`): la zona si spegne
  subito (rispettando il tempo minimo ON), senza aspettare i 15 minuti.
- **In estate (`summer`) il relè della zona è sempre spento**, subito.
- Pulsanti fisici, MQTT e portale non sono mai bloccanti tra loro.

### Rete e memoria (affidabilità)

- Il WiFi si riconnette da solo; se resta giù a lungo il dispositivo
  ritenta ogni minuto e si riavvia dopo 10 min (termostato) / 15 min
  (centralina). **Le credenziali non vengono mai cancellate** da questi
  tentativi.
- Se all'accensione il WiFi non c'è (es. router ancora spento dopo un
  blackout) il dispositivo **non** apre il portale: aspetta la rete.
- Configurazione e stato (setpoint, modo) sono in due file separati su
  LittleFS, scritti in modo atomico (file temporaneo + rinomina): un blackout
  durante il salvataggio lascia il file precedente intatto. Lo stato si salva
  solo se cambia davvero, 10 s dopo l'ultima modifica.

---

## Limiti noti

- **Nessuna cifratura verso il broker MQTT** (niente TLS): adeguato per una
  LAN domestica fidata (meglio se in una VLAN dedicata alla domotica).
- **Portale protetto da una password fissa** (`pizaff123`, costante
  `AP_PASS` nel codice): il portale però è attivo solo quando lo apri tu o al
  primo avvio, e la pagina *Update* è raggiungibile solo mentre è aperto.
- **Durante il salvataggio di una nuova rete WiFi** dal portale la
  centralina resta occupata fino a 20 s (tentativo di connessione): i relè
  restano nello stato in cui sono.
- Se nel portale inserisci una password WiFi sbagliata e poi chiudi il
  portale, il dispositivo continuerà a provare quella: riapri il portale col
  pulsante e correggila.

---

## Licenza

Copyright (C) 2026 Pietro Zaffarano.

Questo progetto è distribuito con licenza **GNU GPL v3.0**: puoi usarlo,
studiarlo, modificarlo e ridistribuirlo, ma ogni derivato che distribuisci
deve restare open source con la stessa licenza — non può essere chiuso in un
prodotto proprietario né essere fatto passare per opera di qualcun altro.
Testo completo in [LICENSE](LICENSE).
