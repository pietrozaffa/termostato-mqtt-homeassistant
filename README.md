# PiZaff Termostati MQTT — termostato da stanza + centralina 4 relè

Sistema di termostati smart **fai-da-te** per Home Assistant, su ESP8266,
senza cloud e senza abbonamenti: ogni stanza ha un termostato con sensore di
temperatura ed encoder, che parla via **MQTT** con una centralina che accende
i relè del riscaldamento.

- **Termostato da stanza** — legge la temperatura (DS18B20), mostra tutto su
  un piccolo OLED, si regola con un encoder rotativo, pubblica/riceve stato e
  comandi via MQTT. Un termostato per stanza (si duplica lo sketch cambiando
  il nome della zona).
- **Centralina 4 relè** — un unico ESP8266 che pilota fisicamente il
  riscaldamento di 4 zone (relè), leggendo via MQTT cosa chiede ciascun
  termostato. Gestisce anche 4 pulsanti locali per comando manuale.

Nessun server proprio da mantenere: basta un broker MQTT qualunque (es.
**Mosquitto** dentro Home Assistant) e la rete WiFi di casa.

---

## Contenuto del repository

| Percorso | Cosa contiene |
|---|---|
| `sketch_termiostati_mqtt_per_homeassistant_V1/` | Firmware del termostato da stanza (ESP8266 + DS18B20 + OLED + encoder) |
| `centralina_termostati_4rele/` | Firmware della centralina (ESP8266 + 4 relè + 4 pulsanti) |
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

---

## Hardware

### Termostato da stanza
- **NodeMCU ESP8266**
- Sensore di temperatura **DS18B20** (OneWire) + resistenza pull-up 4.7 kΩ tra DQ e VCC
- Display **OLED 128×32 SSD1306** (I²C)
- **Encoder rotativo** con pulsante (setpoint + cambio stagione)

```
Wire.begin(12,14):  SDA = D6 (GPIO12)   SCL = D5 (GPIO14)   [OLED]
PIN_ONEWIRE = D0 (GPIO16)                                   [DS18B20, DQ]
PIN_ENC_CLK = D1 (GPIO5)
PIN_ENC_DT  = D2 (GPIO4)
PIN_ENC_SW  = D7 (GPIO13)
PIN_RESET   = GPIO0 (FLASH)   tenuto premuto all'avvio = reset di fabbrica
```

### Centralina 4 relè
- **NodeMCU ESP8266**
- Modulo relè **4 canali, active-LOW** (IN1..IN4 a livello logico invertito)
- 4 pulsanti (uno per zona), verso GND, pull-up interno

```
Zona            Relè (uscita)     Pulsante (ingresso)
Bagno           D0 (GPIO16)       D6 (GPIO12)
CameraPiccola   D1 (GPIO5)        D7 (GPIO13)
Sala            D2 (GPIO4)        D3 (GPIO0)   ⚠️ vedi nota sotto
CameraGrande    D5 (GPIO14)       D4 (GPIO2)   ⚠️ vedi nota sotto
```

> **Nota GPIO0 (pulsante Sala):** è lo stesso pin usato per il reset di
> fabbrica tenuto premuto all'avvio. Non è un errore di cablaggio, è
> condiviso di proposito — basta non tenerlo premuto mentre il dispositivo
> si accende.
> **Nota GPIO2 (pulsante CameraGrande):** deve restare alto al boot; un
> pulsante con pull-up interno (come qui) va benissimo.

---

## Compilazione e flash

Sono progetti **Arduino IDE** (o arduino-cli) per **ESP8266** (board:
*NodeMCU 1.0 (ESP-12E Module)*).

Librerie richieste (installabili dal Library Manager):
- `WiFiManager` (tzapu)
- `PubSubClient` (Nick O'Leary)
- `ArduinoJson` (Benoit Blanchon, v6 o v7)
- Solo per il termostato: `Adafruit SSD1306`, `Adafruit GFX Library`, `OneWire`, `DallasTemperature`

Poi: apri lo sketch giusto, seleziona la porta, **Upload**. Monitor seriale a
115200 baud per il log di avvio.

Per 4 termostati: duplica la cartella `sketch_termiostati_mqtt_per_homeassistant_V1/`
e cambia la costante di zona in testa al file (di default `"CameraPiccola"`,
usata in tutti i topic `termostato/<Zona>/...`) con il nome della stanza.

---

## Primo avvio

Al primo avvio (o dopo un reset di fabbrica) entrambi i dispositivi aprono un
**portale di configurazione** WiFi:

1. Connettiti alla rete **`ESP_Config`**, password **`pizaff123`**.
2. Si apre (o vai su) **http://192.168.4.1**.
3. Scegli la tua rete WiFi di casa e inserisci **server, porta, utente e
   password del broker MQTT**.
4. Salva: il dispositivo si riavvia e si connette alla tua rete + al broker.

Le credenziali vengono salvate **solo** in `/config.json` sulla flash del
dispositivo — non sono mai scritte nel codice sorgente.

**Reset di fabbrica:** tieni premuto il pulsante FLASH (GPIO0) all'accensione
finché non riparte. Sulla centralina è lo stesso pulsante della zona Sala
(vedi nota pinout sopra).

**Riaprire il portale senza perdere la configurazione (solo centralina):**
tieni premuti insieme i pulsanti **Bagno** e **CameraGrande** per 3 secondi.

---

## Uso quotidiano

### Termostato da stanza
- **Ruota l'encoder** → cambia il setpoint (0.5°C a scatto), mostrato a
  schermo per qualche secondo.
- **Pressione lunga (≥2s)** sul pulsante encoder → alterna stagione
  inverno/estate.
- Il display mostra la temperatura ambiente, con icona 🔥/❄️ quando c'è
  richiesta attiva, e due indicatori in basso a destra (WiFi / MQTT connessi).

### Centralina
- **Pressione breve** su un pulsante zona → accende/spegne manualmente quella
  zona (bypassa i tempi minimi, utile in manutenzione).
- **Pressione lunga (≥2s)** su un pulsante qualsiasi → **spegnimento
  immediato di tutte le zone** (sicurezza).
- Le zone seguono automaticamente i comandi dei termostati via MQTT: non
  serve fare nulla in condizioni normali.

---

## Protocollo MQTT

Ogni termostato pubblica/riceve sotto `termostato/<Zona>/...`
(`<Zona>` = `Bagno`, `CameraPiccola`, `Sala`, `CameraGrande`, o come li hai
chiamati):

| Topic | Direzione | Valori | Note |
|---|---|---|---|
| `.../temperature` | termostato → HA | numero (°C) | retained |
| `.../setpoint` | termostato → HA | numero (°C) | retained |
| `.../setpoint_cmd` | HA → termostato | numero (°C), 5–30 | |
| `.../heating` | termostato → **centralina** | `ON` / `OFF` | richiesta di riscaldamento |
| `.../mode` | termostato → HA | `off` / `heat` / `cool` | retained |
| `.../mode_cmd` | HA → termostato | `off` / `heat` / `cool` | |
| `.../season` | termostato → **centralina** | `winter` / `summer` | retained |
| `.../season_cmd` | HA → termostato | `winter` / `summer` | |
| `.../availability` | termostato → HA | `online` / `offline` | LWT |

La centralina, per ogni zona, pubblica sotto `riscaldamento/<Zona>/...`:

| Topic | Direzione | Valori | Note |
|---|---|---|---|
| `.../state` | centralina → HA | `ON` / `OFF` | stato reale del relè, retained |
| `.../set` | HA → centralina | `ON` / `OFF` | override manuale, valido solo in `winter` |

Più `riscaldamento/centralina/availability` (`online`/`offline`, LWT) per
sapere se la centralina stessa è raggiungibile.

### Esempio configurazione Home Assistant (per zona)

```yaml
mqtt:
  climate:
    - name: "Termostato Camera Piccola"
      modes: ["off", "heat", "cool"]
      current_temperature_topic: "termostato/CameraPiccola/temperature"
      temperature_command_topic: "termostato/CameraPiccola/setpoint_cmd"
      temperature_state_topic: "termostato/CameraPiccola/setpoint"
      mode_command_topic: "termostato/CameraPiccola/mode_cmd"
      mode_state_topic: "termostato/CameraPiccola/mode"
      availability_topic: "termostato/CameraPiccola/availability"
      min_temp: 5
      max_temp: 30
      temp_step: 0.5
```

Ripeti per ciascuna zona cambiando `CameraPiccola` con il nome usato nello
sketch di quel termostato.

---

## Come funziona

### Isteresi e stagione (termostato)

`callForHeat` diventa vero/falso con **0.3°C di isteresi** attorno al
setpoint, invertita tra le due stagioni: in `winter` scalda sotto il
setpoint, in `summer` raffresca sopra. Il calcolo è sempre locale sul
termostato: funziona anche se la centralina o il broker sono momentaneamente
irraggiungibili.

### Tempi minimi e sfasamento (centralina)

Per non stressare caldaia/pompa e non far scattare tutti i relè insieme:

- **ON minimo 3 minuti**, **OFF minimo 2 minuti** per zona (anti-ciclo).
- Accensioni **sfasate di 800ms** l'una dall'altra quando più zone
  diventano eleggibili insieme (es. dopo una riconnessione MQTT).
- **Timeout di sicurezza 15 minuti**: se una zona non riceve un comando
  `heating` aggiornato entro questa finestra, si considera scaduto e si
  spegne da sola. In funzionamento normale non scatta mai: ogni termostato
  ripubblica il proprio stato ogni 10s (più subito ad ogni cambiamento), con
  un margine enorme rispetto ai 15 minuti.
- **In estate (`summer`) il relè della zona è sempre spento**, subito,
  bypassando i tempi minimi (non c'è motivo di aspettare per spegnere).
- Pulsanti fisici e MQTT non sono mai bloccanti tra loro: il ciclo principale
  gira sempre, anche a broker MQTT irraggiungibile.

---

## Limiti noti

Alcuni compromessi consapevoli, utile saperli prima di affidarci un impianto:

- **Il portale di configurazione blocca il dispositivo.** Mentre è aperto
  (fino a 3 minuti, sia all'avvio sia riaperto con la combo pulsanti) il
  ciclo principale è fermo: sulla centralina questo significa che una zona
  già accesa non può essere spenta né dai pulsanti né da MQTT finché il
  portale non si chiude. È un limite della libreria WiFiManager usata in
  modalità bloccante.
- **Nessuna cifratura verso il broker MQTT** (niente TLS): password e dati
  viaggiano in chiaro sulla rete locale. Adeguato per una LAN domestica
  fidata, da rivedere se il broker è esposto oltre la rete di casa.
- **Guasto del sensore di temperatura visibile solo sull'OLED**, non
  ripubblicato come stato d'errore via MQTT (Home Assistant continua a
  mostrare l'ultima temperatura valida finché il sensore non torna a
  funzionare).
- **Se un termostato smette di essere raggiungibile** (crash, WiFi giù),
  la centralina continua a fidarsi dell'ultimo comando `heating` ricevuto
  fino al timeout di sicurezza di 15 minuti.

---

## Licenza

Copyright (C) 2026 Pietro Zaffarano.

Questo progetto è distribuito con licenza **GNU GPL v3.0**: puoi usarlo,
studiarlo, modificarlo e ridistribuirlo, ma ogni derivato che distribuisci
deve restare open source con la stessa licenza — non può essere chiuso in un
prodotto proprietario né essere fatto passare per opera di qualcun altro.
Testo completo in [LICENSE](LICENSE).
