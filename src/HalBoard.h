#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// Externe Library (wird NICHT mit in unser Projekt-zip gepackt)
// Erwartet: RS485_Anemometer (Ordnername: RS485_Anemometer)
#include <RS485_Anemometer.h>
#include "EncoderAxis.h"

// -----------------------------
// Pinout
// -----------------------------
static const int PIN_END_LEFT    = 12;  // Endschalter links, aktiv LOW (gedrueckt = LOW)
static const int PIN_END_RIGHT   = 13;  // Endschalter rechts, aktiv LOW (gedrueckt = LOW)
static const int PIN_LED_LB      = 14;  // LED, aktiv LOW

static const int PIN_OE_INVERTER = 4;    // Schaltet den Level-Shifter an nach dem Start

static const int PIN_RS485_RX    = 18;  // Serial1 RX
static const int PIN_RS485_TX    = 17;  // Serial1 TX
// Master-RS485: THVD1406DR schaltet TX/RX automatisch (kein DE/RE-GPIO).

static const int PIN_IS1_ADC     = 15;  // BTN8982 IS1, analog 0..3.3V
static const int PIN_IS2_ADC     = 16;  // BTN8982 IS2, analog 0..3.3V

// ------------------------------------------------------------
// I2C-Bus (spaeter fuer Erweiterungsbausteine)
// ------------------------------------------------------------
// Frueher: GPIO1 = Master-RS485 DE/RE, GPIO38 = Wind-RS485 DE/RE.
// Beide Transceiver (THVD1406DR) schalten jetzt automatisch um.
static const int PIN_I2C_SDA = 1;
static const int PIN_I2C_SCL = 38;

// ------------------------------------------------------------
// Wind- & Richtungsmesser (RS485 / Modbus)
// ------------------------------------------------------------
// Separater RS485-Bus (nicht Master-RS485), Serial2 @ 9600 8N1.
// THVD1406DR: automatische Richtungsumschaltung, kein DE/RE-GPIO.
static const int PIN_WIND_RS485_RX  = 9;
static const int PIN_WIND_RS485_TX  = 8;

static const int PIN_SRV_LEFT    = 47;  // Service-Taster links, aktiv LOW
static const int PIN_SRV_RIGHT   = 48;  // Service-Taster rechts, aktiv LOW

// -----------------------------
// HalBoard
// -----------------------------
class HalBoard {
public:
  void begin();

  // Pin-Profil nach NVS-Laden (EncoderType):
  // Typ 1/2 = OE/Endschalter/Wind; Typ 3 = SSI (Pin 4 HIGH), kein Bus auf 8/9;
  // Typ 4 = Inklinometer auf Wind-Pins (Serial2 @115200), kein Wind.
  void applyHardwareProfile(EncoderType encType);

  bool isWindHardwareAvailable() const { return _windHardwareAvailable; }
  bool isInclinoHardwareAvailable() const { return _inclinoHardwareAvailable; }

  // Startet den Hintergrund-Task fuer den Windsensor erst dann,
  // wenn die Preferences bereits geladen und die initialen Offsets/Enable-Zustaende
  // gesetzt wurden. Dadurch sprechen waehrend setup() noch keine parallelen
  // Sensorzugriffe mit Serial2 dazwischen.
  void startWindTask();

  // Inklinometer-Poll-Task (nur Typ 4; gegenseitig ausschliessend zum Wind-Task).
  void startInclinoTask();

  // Letzter frischer Winkel (Deg01 = Grad*100). false wenn stale/ungueltig.
  bool getInclinoDeg01(int32_t& outDeg01) const;
  bool isInclinoValid() const;

  // Proxy-Transaktion auf dem Inklinometer-Bus (Serial2).
  // reqTail z.B. "GETSWAPXY" oder "SETCALIB,-2.50" (ohne #/$).
  // outValue = Payload nach dem Komma aus #ACK_<cmd>,<payload>$
  enum InclinoTxStatus : uint8_t {
    INCLINO_TX_OK = 0,
    INCLINO_TX_NOHW = 1,
    INCLINO_TX_TIMEOUT = 2,
    INCLINO_TX_NACK = 3,
    INCLINO_TX_BAD = 4,
  };
  InclinoTxStatus inclinoTransact(const char* reqTail, String& outValue,
                                  uint32_t timeoutMs = 100);

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

  // Endschalter aktiv LOW (gedrueckt = LOW) — Roh-Pins (Diagnose)
  bool readEndLeft() const;
  bool readEndRight() const;

  // Anzahl Endschalter: 0=keine, 1=ein Kontakt (L ODER R), 2=zwei Kontakte
  void setEndstopCount(uint8_t count);
  uint8_t getEndstopCount() const { return _endstopCount; }
  // Bei Modus 1: Kontakt an beliebigem Eingang (L || R)
  bool readEndContact() const;

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

  // ------------------------------------------------------------
  // Inklinometer-Task intern (Serial2, #GETDG$)
  // ------------------------------------------------------------
  static void inclinoTaskTrampoline(void* arg);
  void inclinoTaskLoop();
  void inclinoTaskStep(uint32_t nowMs);
  bool inclinoPollOnce(uint32_t nowMs);

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
  bool     _windHardwareAvailable = true;
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

  // ------------------------------------------------------------
  // Inklinometer (Cache) — gleicher UART/Pins wie Wind
  // ------------------------------------------------------------
  bool     _inclinoHardwareAvailable = false;
  bool     _inclinoInit = false;
  TaskHandle_t _inclinoTaskHandle = nullptr;
  SemaphoreHandle_t _inclinoBusMutex = nullptr;
  portMUX_TYPE _inclinoStateMux = portMUX_INITIALIZER_UNLOCKED;
  bool     _inclinoValid = false;
  int32_t  _inclinoDeg01 = 0;
  uint32_t _inclinoLastOkMs = 0;

  // 0=keine, 1=ein Kontakt (L||R), 2=zwei Kontakte (Default)
  uint8_t  _endstopCount = 2;
};
