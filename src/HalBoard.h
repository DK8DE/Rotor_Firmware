#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// Externe Library (wird NICHT mit in unser Projekt-zip gepackt)
// Erwartet: RS485_Anemometer (Ordnername: RS485_Anemometer)
#include <RS485_Anemometer.h>

// -----------------------------
// Pinout
// -----------------------------
static const int PIN_RS485_DIR   = 1;   // RX/TX Umschaltung (DE/RE) | HIGH = TX, LOW = RX
static const int PIN_END_LEFT    = 12;  // Endschalter links, aktiv LOW (gedrueckt = LOW)
static const int PIN_END_RIGHT   = 13;  // Endschalter rechts, aktiv LOW (gedrueckt = LOW)
static const int PIN_LED_LB      = 14;  // LED, aktiv LOW

static const int PIN_OE_INVERTER = 4;    // Schaltet den Level-Shifter an nach dem Start

static const int PIN_RS485_RX    = 18;  // Serial1 RX
static const int PIN_RS485_TX    = 17;  // Serial1 TX

static const int PIN_IS1_ADC     = 15;  // BTN8982 IS1, analog 0..3.3V
static const int PIN_IS2_ADC     = 16;  // BTN8982 IS2, analog 0..3.3V

// ------------------------------------------------------------
// Wind- & Richtungsmesser (RS485 / Modbus)
// ------------------------------------------------------------
// Neuer Sensor laeuft separat ueber RS485 (nicht ueber unser Master-RS485).
// Verkabelung (Referenz):
// - RX = Pin 9
// - TX = Pin 8
// - DE/RE (Senden/Empfangen Umschaltung) = Pin 38
// Hinweis:
// - Wir nutzen hier Serial2 (HardwareSerial) mit 9600 8N1 (Lib-Default).
static const int PIN_WIND_RS485_RX  = 9;
static const int PIN_WIND_RS485_TX  = 8;
static const int PIN_WIND_RS485_DIR = 38;

static const int PIN_SRV_LEFT    = 47;  // Service-Taster links, aktiv LOW
static const int PIN_SRV_RIGHT   = 48;  // Service-Taster rechts, aktiv LOW

// -----------------------------
// HalBoard
// -----------------------------
class HalBoard {
public:
  void begin();

  // Startet den Hintergrund-Task fuer den Windsensor erst dann,
  // wenn die Preferences bereits geladen und die initialen Offsets/Enable-Zustaende
  // gesetzt wurden. Dadurch sprechen waehrend setup() noch keine parallelen
  // Sensorzugriffe mit Serial2 dazwischen.
  void startWindTask();

  // ------------------------------------------------------------
  // Wind- & Richtungsmesser (RS485)
  // ------------------------------------------------------------
  // Historische API: bleibt aus Kompatibilitaetsgruenden erhalten.
  // Die eigentliche Sensor-Abfrage laeuft jetzt in einem eigenen FreeRTOS-Task
  // und blockiert damit nicht mehr die Haupt-Loop.
  void updateWind(uint32_t nowMs);

  // Wind-Sensor global aktivieren/deaktivieren.
  // Wenn deaktiviert, werden keine RS485/Modbus-Abfragen an den Sensor geschickt
  // und die Rueckgabe der Windwerte ist immer 0.
  //
  // Zusaetzlich kann der Sensor intern temporaer deaktiviert werden, wenn er
  // laenger nicht antwortet. In diesem Fall liefert getWindEnable() ebenfalls 0.
  void setWindEnable(bool enable);
  bool getWindEnable() const;

  // Setzt die Offsets, die in Preferences persistiert sind.
  // - Speed-Offset wird in km/h uebergeben (Kompatibilitaet zu altem Anemometer)
  // - Richtungs-Offset in Grad
  void setWindSpeedOffsetKmh(float offsetKmh);
  void setWindDirOffsetDeg(float offsetDeg);

  // Letzte gueltige Messwerte abholen.
  // Rueckgabe: true wenn ein gueltiger Messwert vorhanden ist (nicht "stale").
  bool getWindSpeedKmh(float& outKmh) const;
  bool getWindDirDeg(float& outDeg) const;
  bool getWindBeaufort(uint8_t& outBft) const;

  // LED aktiv LOW
  void setLed(bool on);

  // Endschalter aktiv LOW (gedrueckt = LOW)
  bool readEndLeft() const;
  bool readEndRight() const;

  // Service-Taster: true = gedrueckt (aktiv LOW)
  bool readServiceLeft() const;
  bool readServiceRight() const;

  // Strommessung (mV)
  uint32_t readIs1mV() const;
  uint32_t readIs2mV() const;

private:
  // ------------------------------------------------------------
  // Wind-Task intern
  // ------------------------------------------------------------
  static void windTaskTrampoline(void* arg);
  void windTaskLoop();
  void windTaskStep(uint32_t nowMs);

  // Schneller Cache fuer Leser aus loop()/RS485.
  void setWindCacheZero(uint32_t nowMs, bool valid, bool clearAutoDisabled);
  void setWindCacheFromSensor(uint32_t nowMs);
  void applyPendingWindConfig();

  // Zustand atomar lesen/schreiben.
  void copyWindState(bool& configuredEnable, bool& autoDisabled, bool& valid,
                     bool& configDirty,
                     uint32_t& lastPollMs, uint32_t& lastOkMs, uint32_t& lastRetryMs,
                     float& speedMps, float& dirDeg, uint8_t& beaufort) const;

  // Zugriff auf die Sensor-Library exklusiv serialisieren.
  SemaphoreHandle_t _windSensorMutex = nullptr;
  TaskHandle_t _windTaskHandle = nullptr;
  portMUX_TYPE _windStateMux = portMUX_INITIALIZER_UNLOCKED;

  // ------------------------------------------------------------
  // Wind-Sensor intern (Cache)
  // ------------------------------------------------------------
  RS485_Anemometer _wind;
  bool     _windInit = false;
  bool     _windConfiguredEnable = true;
  bool     _windAutoDisabled = false;
  bool     _windValid = false;
  bool     _windConfigDirty = true;
  uint32_t _windLastPollMs = 0;
  uint32_t _windLastOkMs = 0;
  uint32_t _windLastRetryMs = 0;
  float    _windSpeedMps = 0.0f;
  float    _windDirDeg = 0.0f;
  uint8_t  _windBeaufort = 0;
  float    _windSpeedOffsetKmh = 0.0f;
  float    _windDirOffsetDeg = 0.0f;
};
