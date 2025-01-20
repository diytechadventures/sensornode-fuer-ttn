// VS Code Programm zur Übermittlung von Sensormesswerten an das TheTingsNetwork
// Programm wurde mit Unterstützung ChatGPT optimiert
// dieser Code steht jedem zur freien Verfügung

//Librarys einbinden
#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_SHT31.h>

//globales Ein-/Ausschalten der Ausgaben über die serielle Schnittstelle
#define DEBUG 1
#if DEBUG == 1
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#else
#define debug(x)
#define debugln(x)
#endif

//Variablen für die Zeitmesessung (Heater)
unsigned long heaterStartMillis = millis();
const unsigned long heaterDuration = 10000;      // Heizerdauer: 10 Sekunden
bool heater_status = false; // Status ob Heater im SHT31 ein- oder ausgeschaltet ist

//Sendeintervall der Sensorwerte an das TTN
uint16_t sendInterval = 900; // wie oft werden Sensorwerte gemessen und gesendet in Sekunden

// Variablen für Temperatur und Luftfeuchtigkeit
float t;
int t1;
float h;
int h1;

//Sensor- und Sendejob anlegen
Adafruit_SHT31 sht31 = Adafruit_SHT31(); // Sensorobjekt anlegen
osjob_t sendjob; // Lora Sendobjekt anlegen

// OTAA-Schlüssel aus dem angelegten End device in LSB/MSB-Format einfügen
static const u1_t PROGMEM DEVEUI[8] = { 0xE1, 0xCD, 0x06, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 }; //LSB
static const u1_t PROGMEM APPEUI[8] = { 0x05, 0xFB, 0xBF, 0x34, 0x12, 0x03, 0x02, 0x01 }; //LSB
static const u1_t PROGMEM APPKEY[16] = { 0xA2, 0x96, 0xFC, 0x20, 0x62, 0xE4, 0xF3, 0xC2, 0x58, 0x63, 0xEA, 0x00, 0x78, 0xBC, 0xB1, 0xBE }; //MSB

// Pinbelegung für TTGO LoRa32 Chip V2.0
const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {26, 33, 32},
    //.spi = {SCK, MISO, MOSI},
};

// Deklaration für Sensor-Abfrageroutine
void get_sht3x_data();

// die OTAA-Schlüssel in die Variablem kopieren
void os_getArtEui(u1_t *buf) { memcpy_P(buf, APPEUI, 8); }
void os_getDevEui(u1_t *buf) { memcpy_P(buf, DEVEUI, 8); }
void os_getDevKey(u1_t *buf) { memcpy_P(buf, APPKEY, 16); }

//Funktion zum Senden der Daten an das TTN
void sendeDaten(osjob_t* j) {
  if (LMIC.opmode & OP_TXRXPEND) {
    debugln(F("Übertragung läuft, warte..."));
  } 
  else {
    // Messung der Sensordaten
    debugln("Frage Sensor ab");
    get_sht3x_data();

    // Daten für die Übertragung vorbereiten
    debugln("Daten für Payload packen");
    uint8_t daten[4];
    daten[0] = t1 >> 8;
    daten[1] = t1 & 0xFF;
    daten[2] = h1 >> 8;
    daten[3] = h1 & 0xFF;

    // Daten senden
    LMIC_setTxData2(1, daten, sizeof(daten), 0);
    debugln(F("Paket in Warteschlange"));
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

// Initialisiere SHT31
  if (!sht31.begin(0x44)) {
    debugln(F("SHT31 nicht gefunden!"));
    while (true);
  }

  debugln(F("Starte OTAA..."));

  // LMIC-Initialisierung
  os_init();
  LMIC_reset();
  LMIC_setAdrMode(0); // Deaktiviert ADR
  //LMIC_setDrTxpow(DR_SF12, 14); // Bsp: SF12 für maximale Reichweite, 14 dBm Sendeleistung
  LMIC_startJoining(); // OTAA-Join starten
}

void loop() {
  os_runloop_once();

  // Prüfen, ob der Heizer aktiv ist und ob die Heizzeit abgelaufen ist
  if (heater_status && (millis() - heaterStartMillis >= heaterDuration)) {
    sht31.heater(false);      // Heizer ausschalten
    heater_status = false;    // Status des Heizers zurücksetzen
    debugln("Heater deaktiviert!");
  }      
}

// Routine zur Abfrage der Sensorwerte des SHT31 und Einschalten des Heaters
void get_sht3x_data() {
  t = sht31.readTemperature();
  h = sht31.readHumidity();
    if (! isnan(t)) {  // check if 'is not a number'
    debug("Temp *C = "); debug(t); debug("\t\t"); // Formatierte Ausgabe
  } else { 
    debugln("Failed to read temperature");
  }
  
  if (! isnan(h)) {  // check if 'is not a number'
    debug("Hum. % = "); debugln(h);
  } else { 
    debugln("Failed to read humidity");
  }
t1 = (t + 30) * 100; // negative Werte vermeiden und ganzzahlig machen
h1 = h * 10; // Luftfeuchtigkeit ganzzahlig machen

  // Heizer aktivieren für genauere Messwerte Luftfeuchtigkeit
  if (!heater_status) { // Nur aktivieren, wenn er nicht schon läuft
    sht31.heater(true); // Heater einschalten
    heater_status = true;
    heaterStartMillis = millis(); // Startzeit des Heizers speichern
    debugln("Heater eingeschaltet!");
  }
}

//Events prüfen und Aktionen starten
void onEvent(ev_t ev) {
  debug(os_getTime());
  debugln(": ");
  
  switch (ev) {
    case EV_JOINING:
      debugln(F("EV_JOINING: Versuche, dem Netzwerk beizutreten"));
      break;

    case EV_JOINED:
      debugln(F("EV_JOINED: Erfolgreich dem Netzwerk beigetreten"));
      LMIC_setLinkCheckMode(0);
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(1), sendeDaten); //erstes senden
      break;

    case EV_TXSTART:
      debugln(F("EV_TXSTART: Übertragung gestartet"));
      break;

    case EV_TXCOMPLETE:
      debugln(F("EV_TXCOMPLETE: Übertragung abgeschlossen"));
      if (LMIC.txrxFlags & TXRX_ACK) {
        debugln(F("ACK erhalten"));
      }
      if (LMIC.dataLen) {
        debug(F("Empfangene Daten: "));
        debug(LMIC.dataLen);
        debugln(F(" Bytes"));
      }

      // hier wird die eigentliche Sendung abhängig von sendinterval gestartet
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(sendInterval), sendeDaten);
      break;

    case EV_JOIN_FAILED:
      debugln(F("EV_JOIN_FAILED: Join-Versuch fehlgeschlagen"));
      break;

    case EV_REJOIN_FAILED:
      debugln(F("EV_REJOIN_FAILED: Rejoin-Versuch fehlgeschlagen"));
      break;

default:
    debug(F("Unbekanntes Ereignis: "));
    debug(ev);
    debug(F(" (Hex: 0x"));
   Serial.print(ev, HEX);
    debugln(F(")"));
    break;
  }
}