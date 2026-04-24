#include "HalBoard.h"

// ------------------------------------------------------------
// Wind-Sensor Poll-Interval
// ------------------------------------------------------------
// Modbus-Read dauert (je nach Timing/Timeout) bis ~160ms.
// Wir pollen daher nicht zu schnell.
static const uint32_t WIND_POLL_INTERVAL_MS = 200;

// Wenn laenger als diese Zeit kein gueltiger Messwert kam, gilt er als "stale".
static const uint32_t WIND_STALE_MS = 2000;

// Wenn der Sensor laenger nicht antwortet, wird die Windmessung temporaer
// deaktiviert. Alle 10s wird ein neuer Verbindungsversuch gemacht.
static const uint32_t WIND_AUTO_DISABLE_AFTER_MS = 2000;
static const uint32_t WIND_RETRY_INTERVAL_MS = 10000;

// Der Wind-Task bleibt bewusst sehr klein priorisiert.
// Core 0 ist hier sinnvoll, weil die Arduino-loop() auf dem ESP32 typischerweise
// auf dem anderen Core laeuft und damit der Modbus-Zugriff des Windsensors
// die Haupt-Loop noch weniger stoert.
static const BaseType_t WIND_TASK_PRIORITY = 1;
static const uint32_t WIND_TASK_STACK_WORDS = 4096;
static const BaseType_t WIND_TASK_CORE = 0;

void HalBoard::begin() {
  // RS485 DIR Pin: wird von Rs485Proto geschaltet, hier nur als OUTPUT vorbereiten
  pinMode(PIN_RS485_DIR, OUTPUT);
  digitalWrite(PIN_RS485_DIR, LOW); // RX default

  // ------------------------------------------------------------
  // Endschalter aktiv LOW (gedrueckt = LOW)
  // Hinweis:
  // - In dieser Projekt-Hardware sind Pullups extern vorhanden.
  // - Daher wird hier INPUT verwendet (kein interner Pullup).
  //   Ohne externe Pullups: INPUT_PULLUP verwenden.
  // ------------------------------------------------------------
  pinMode(PIN_END_LEFT, INPUT);
  pinMode(PIN_END_RIGHT, INPUT);

    pinMode(PIN_WIND_RS485_RX, INPUT_PULLUP);

  // LED aktiv LOW
  pinMode(PIN_LED_LB, OUTPUT);
  digitalWrite(PIN_LED_LB, HIGH); // aus

  // ------------------------------------------------------------
  // Service-Taster aktiv LOW: interner Pullup
  // ------------------------------------------------------------
  pinMode(PIN_SRV_LEFT, INPUT_PULLUP);
  pinMode(PIN_SRV_RIGHT, INPUT_PULLUP);

  // ------------------------------------------------------------
  // Schaltet den Level-Shifter an nach dem Start
  // ------------------------------------------------------------
   pinMode(PIN_OE_INVERTER, OUTPUT);
   digitalWrite(PIN_OE_INVERTER, LOW);

  // ADC
  analogReadResolution(12);

  // ------------------------------------------------------------
  // Strommessung (IS1/IS2) am ADC
  // ------------------------------------------------------------
  // Hinweis:
  // - ESP32-S3: GPIO15/16 liegen auf ADC2 (GPIO11..20). Das ist grundsaetzlich
  //   ok. Falls spaeter WiFi aktiviert wird, kann ADC2 zwischen
  //   esp_wifi_start() und esp_wifi_stop() ungueltige Werte liefern.
  // - Attenuation:
  //   Wir setzen 11dB, damit der Messbereich bis ca. 3.1V sauber abgedeckt ist.
  //   (BTN8982 IS-Ausgang ist typischerweise deutlich darunter, aber 11dB ist
  //   robust und verhindert Sattigung bei Peaks.)
  // - INPUT-Modus setzen, damit keine Pullups/Pulldowns dazwischenfunken.
  pinMode(PIN_IS1_ADC, INPUT);
  pinMode(PIN_IS2_ADC, INPUT);
  analogSetPinAttenuation(PIN_IS1_ADC, ADC_11db);
  analogSetPinAttenuation(PIN_IS2_ADC, ADC_11db);


  // ------------------------------------------------------------
  // Wind- & Richtungsmesser (RS485 / Modbus)
  // ------------------------------------------------------------
  // Der Sensor laeuft an einer eigenen UART (Serial2), damit unser Master-RS485
  // (Serial1) unbeeinflusst bleibt.
  // Wichtig:
  // - DE/RE LOW = Empfangen
  // - RX/TX Pins sind fix laut Verkabelung.
  _wind.begin(Serial2, PIN_WIND_RS485_RX, PIN_WIND_RS485_TX, PIN_WIND_RS485_DIR, 0x01, 9600);
  _windInit = true;

  // Persistente Offsets werden spaeter in setup() gesetzt.
  _wind.setSpeedOffsetMps(0.0f);
  _wind.setAngleOffsetDeg(0.0f);

  // Eigener Mutex fuer exklusiven Zugriff auf die Sensor-Library.
  if (_windSensorMutex == nullptr) {
    _windSensorMutex = xSemaphoreCreateMutex();
  }

}

// ------------------------------------------------------------
// Wind- & Richtungsmesser
// ------------------------------------------------------------

void HalBoard::startWindTask() {
  if (!_windInit) return;
  if (_windTaskHandle != nullptr) return;

  // Beim Start sollen die zuletzt gesetzten Offsets aus den Globals/Preferences
  // garantiert zuerst in die Sensor-Library uebernommen werden.
  portENTER_CRITICAL(&_windStateMux);
  _windConfigDirty = true;
  _windLastPollMs = 0;
  _windLastRetryMs = 0;
  portEXIT_CRITICAL(&_windStateMux);

  xTaskCreatePinnedToCore(
    HalBoard::windTaskTrampoline,
    "wind_task",
    WIND_TASK_STACK_WORDS,
    this,
    WIND_TASK_PRIORITY,
    &_windTaskHandle,
    WIND_TASK_CORE
  );
}

void HalBoard::windTaskTrampoline(void* arg) {
  HalBoard* self = static_cast<HalBoard*>(arg);
  if (self) {
    self->windTaskLoop();
  }
  vTaskDelete(nullptr);
}

void HalBoard::windTaskLoop() {
  // Eigener, langsamer Hintergrund-Task.
  // Der Modbus-Zugriff darf hier blockieren, ohne die Haupt-Loop auszubremsen.
  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
    const uint32_t nowMs = millis();
    windTaskStep(nowMs);
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(WIND_POLL_INTERVAL_MS));
  }
}

void HalBoard::copyWindState(bool& configuredEnable, bool& autoDisabled, bool& valid,
                             bool& configDirty,
                             uint32_t& lastPollMs, uint32_t& lastOkMs, uint32_t& lastRetryMs,
                             float& speedMps, float& dirDeg, uint8_t& beaufort) const {
  portENTER_CRITICAL(const_cast<portMUX_TYPE*>(&_windStateMux));
  configuredEnable = _windConfiguredEnable;
  autoDisabled = _windAutoDisabled;
  valid = _windValid;
  configDirty = _windConfigDirty;
  lastPollMs = _windLastPollMs;
  lastOkMs = _windLastOkMs;
  lastRetryMs = _windLastRetryMs;
  speedMps = _windSpeedMps;
  dirDeg = _windDirDeg;
  beaufort = _windBeaufort;
  portEXIT_CRITICAL(const_cast<portMUX_TYPE*>(&_windStateMux));
}

void HalBoard::applyPendingWindConfig() {
  if (!_windInit) return;

  bool dirty = false;
  float speedOffsetKmh = 0.0f;
  float dirOffsetDeg = 0.0f;

  portENTER_CRITICAL(&_windStateMux);
  dirty = _windConfigDirty;
  speedOffsetKmh = _windSpeedOffsetKmh;
  dirOffsetDeg = _windDirOffsetDeg;
  portEXIT_CRITICAL(&_windStateMux);

  if (!dirty) {
    return;
  }

  const float speedOffsetMps = speedOffsetKmh / 3.6f;

  if (_windSensorMutex) {
    xSemaphoreTake(_windSensorMutex, portMAX_DELAY);
  }

  // Nur der Hintergrund-Task spricht mit der Sensor-Library.
  // Dadurch blockieren RS485-Handler nie mehr an Serial2/Modbus.
  _wind.setSpeedOffsetMps(speedOffsetMps);
  _wind.setAngleOffsetDeg(dirOffsetDeg);

  if (_windSensorMutex) {
    xSemaphoreGive(_windSensorMutex);
  }

  portENTER_CRITICAL(&_windStateMux);
  _windConfigDirty = false;
  portEXIT_CRITICAL(&_windStateMux);
}

void HalBoard::setWindCacheZero(uint32_t nowMs, bool valid, bool clearAutoDisabled) {
  portENTER_CRITICAL(&_windStateMux);
  _windSpeedMps = 0.0f;
  _windDirDeg = 0.0f;
  _windBeaufort = 0;
  _windValid = valid;
  _windLastOkMs = nowMs;
  if (clearAutoDisabled) {
    _windAutoDisabled = false;
  }
  portEXIT_CRITICAL(&_windStateMux);
}

void HalBoard::setWindCacheFromSensor(uint32_t nowMs) {
  float speedMps = 0.0f;
  float dirDeg = 0.0f;
  uint8_t beaufort = 0;

  if (_windSensorMutex) {
    xSemaphoreTake(_windSensorMutex, portMAX_DELAY);
  }

  // Werte aus der Library holen. Offsets wurden dort bereits angewendet.
  speedMps = _wind.getWindSpeedMps();
  dirDeg = _wind.getWindAngleDeg();
  int lvl = _wind.getWindLevel();
  if (lvl < 0) lvl = 0;
  if (lvl > 12) lvl = 12;
  beaufort = (uint8_t)lvl;

  if (_windSensorMutex) {
    xSemaphoreGive(_windSensorMutex);
  }

  portENTER_CRITICAL(&_windStateMux);
  _windSpeedMps = speedMps;
  _windDirDeg = dirDeg;
  _windBeaufort = beaufort;
  _windValid = true;
  _windLastOkMs = nowMs;
  _windAutoDisabled = false;
  portEXIT_CRITICAL(&_windStateMux);
}

void HalBoard::windTaskStep(uint32_t nowMs) {
  if (!_windInit) return;

  bool configuredEnable = true;
  bool autoDisabled = false;
  bool valid = false;
  bool configDirty = false;
  uint32_t lastPollMs = 0;
  uint32_t lastOkMs = 0;
  uint32_t lastRetryMs = 0;
  float speedMps = 0.0f;
  float dirDeg = 0.0f;
  uint8_t beaufort = 0;

  copyWindState(configuredEnable, autoDisabled, valid, configDirty, lastPollMs, lastOkMs, lastRetryMs,
                speedMps, dirDeg, beaufort);

  // Zuerst ausstehende Konfigurationsaenderungen (Offsets) in die Library uebernehmen.
  // Dadurch spricht ausser diesem Task niemand mehr mit dem Windsensor.
  if (configDirty) {
    applyPendingWindConfig();
  }

  // Vom Nutzer komplett deaktiviert: keine RS485-Abfragen senden.
  if (!configuredEnable) {
    setWindCacheZero(nowMs, true, true);
    return;
  }

  // Sensor antwortet aktuell nicht: effective GETWINDENABLE = 0.
  // Alle 10s einen neuen Versuch machen.
  if (autoDisabled) {
    setWindCacheZero(nowMs, true, false);

    if ((uint32_t)(nowMs - lastRetryMs) < WIND_RETRY_INTERVAL_MS) {
      return;
    }

    portENTER_CRITICAL(&_windStateMux);
    _windLastRetryMs = nowMs;
    portEXIT_CRITICAL(&_windStateMux);

    bool ok = false;
    if (_windSensorMutex) {
      xSemaphoreTake(_windSensorMutex, portMAX_DELAY);
    }
    ok = _wind.update();
    if (_windSensorMutex) {
      xSemaphoreGive(_windSensorMutex);
    }

    if (ok) {
      setWindCacheFromSensor(nowMs);
    }
    return;
  }

  // Zeit robust gegen millis()-Overflow.
  if ((uint32_t)(nowMs - lastPollMs) < WIND_POLL_INTERVAL_MS) {
    const uint32_t refMs = (lastOkMs != 0U) ? lastOkMs : 0U;
    if (((lastOkMs != 0U) && ((uint32_t)(nowMs - refMs) > WIND_AUTO_DISABLE_AFTER_MS)) ||
        ((lastOkMs == 0U) && (nowMs > WIND_AUTO_DISABLE_AFTER_MS))) {
      portENTER_CRITICAL(&_windStateMux);
      _windAutoDisabled = true;
      _windLastRetryMs = nowMs;
      portEXIT_CRITICAL(&_windStateMux);
      setWindCacheZero(nowMs, true, false);
    }
    return;
  }

  portENTER_CRITICAL(&_windStateMux);
  _windLastPollMs = nowMs;
  portEXIT_CRITICAL(&_windStateMux);

  bool ok = false;
  if (_windSensorMutex) {
    xSemaphoreTake(_windSensorMutex, portMAX_DELAY);
  }
  ok = _wind.update();
  if (_windSensorMutex) {
    xSemaphoreGive(_windSensorMutex);
  }

  if (ok) {
    setWindCacheFromSensor(nowMs);
    return;
  }

  if (((lastOkMs != 0U) && ((uint32_t)(nowMs - lastOkMs) > WIND_AUTO_DISABLE_AFTER_MS)) ||
      ((lastOkMs == 0U) && (nowMs > WIND_AUTO_DISABLE_AFTER_MS))) {
    portENTER_CRITICAL(&_windStateMux);
    _windAutoDisabled = true;
    _windLastRetryMs = nowMs;
    portEXIT_CRITICAL(&_windStateMux);
    setWindCacheZero(nowMs, true, false);
  }
}

void HalBoard::updateWind(uint32_t nowMs) {
  // Historische API: absichtlich leer.
  // Die Windabfrage laeuft jetzt in einem eigenen FreeRTOS-Task.
  (void)nowMs;
}

bool HalBoard::getWindEnable() const {
  bool configuredEnable = true;
  bool autoDisabled = false;

  portENTER_CRITICAL(const_cast<portMUX_TYPE*>(&_windStateMux));
  configuredEnable = _windConfiguredEnable;
  autoDisabled = _windAutoDisabled;
  portEXIT_CRITICAL(const_cast<portMUX_TYPE*>(&_windStateMux));

  return (configuredEnable && !autoDisabled);
}

void HalBoard::setWindEnable(bool enable) {
  portENTER_CRITICAL(&_windStateMux);
  _windConfiguredEnable = enable;
  _windAutoDisabled = false;

  // Cache sofort konsistent machen.
  if (!_windConfiguredEnable) {
    _windSpeedMps = 0.0f;
    _windDirDeg = 0.0f;
    _windBeaufort = 0;
    _windValid = true;
    _windLastOkMs = millis();
  } else {
    // Beim Einschalten: Messwert gilt erst wieder als gueltig,
    // wenn der Hintergrund-Task mindestens einmal erfolgreich war.
    _windValid = false;
    _windLastPollMs = 0;
    _windLastRetryMs = 0;
  }
  portEXIT_CRITICAL(&_windStateMux);
}

void HalBoard::setWindSpeedOffsetKmh(float offsetKmh) {
  portENTER_CRITICAL(&_windStateMux);
  _windSpeedOffsetKmh = offsetKmh;
  _windConfigDirty = true;
  portEXIT_CRITICAL(&_windStateMux);
}

void HalBoard::setWindDirOffsetDeg(float offsetDeg) {
  portENTER_CRITICAL(&_windStateMux);
  _windDirOffsetDeg = offsetDeg;
  _windConfigDirty = true;
  portEXIT_CRITICAL(&_windStateMux);
}

bool HalBoard::getWindSpeedKmh(float& outKmh) const {
  bool valid = false;
  uint32_t lastOkMs = 0;
  float speedMps = 0.0f;

  portENTER_CRITICAL(const_cast<portMUX_TYPE*>(&_windStateMux));
  valid = _windValid;
  lastOkMs = _windLastOkMs;
  speedMps = _windSpeedMps;
  portEXIT_CRITICAL(const_cast<portMUX_TYPE*>(&_windStateMux));

  if (!valid) return false;

  const uint32_t nowMs = millis();
  if ((uint32_t)(nowMs - lastOkMs) > WIND_STALE_MS) return false;

  outKmh = speedMps * 3.6f;
  return true;
}

bool HalBoard::getWindDirDeg(float& outDeg) const {
  bool valid = false;
  uint32_t lastOkMs = 0;
  float dirDeg = 0.0f;

  portENTER_CRITICAL(const_cast<portMUX_TYPE*>(&_windStateMux));
  valid = _windValid;
  lastOkMs = _windLastOkMs;
  dirDeg = _windDirDeg;
  portEXIT_CRITICAL(const_cast<portMUX_TYPE*>(&_windStateMux));

  if (!valid) return false;

  const uint32_t nowMs = millis();
  if ((uint32_t)(nowMs - lastOkMs) > WIND_STALE_MS) return false;

  outDeg = dirDeg;
  return true;
}

bool HalBoard::getWindBeaufort(uint8_t& outBft) const {
  bool valid = false;
  uint32_t lastOkMs = 0;
  uint8_t beaufort = 0;

  portENTER_CRITICAL(const_cast<portMUX_TYPE*>(&_windStateMux));
  valid = _windValid;
  lastOkMs = _windLastOkMs;
  beaufort = _windBeaufort;
  portEXIT_CRITICAL(const_cast<portMUX_TYPE*>(&_windStateMux));

  if (!valid) return false;

  const uint32_t nowMs = millis();
  if ((uint32_t)(nowMs - lastOkMs) > WIND_STALE_MS) return false;

  outBft = beaufort;
  return true;
}

void HalBoard::setLed(bool on) {
  // LED aktiv LOW
  digitalWrite(PIN_LED_LB, on ? LOW : HIGH);
}

bool HalBoard::readEndLeft() const {
  // aktiv LOW (gedrueckt = LOW)
  return (digitalRead(PIN_END_LEFT) == LOW);
}

bool HalBoard::readEndRight() const {
  // aktiv LOW (gedrueckt = LOW)
  return (digitalRead(PIN_END_RIGHT) == LOW);
}

bool HalBoard::readServiceLeft() const {
  // aktiv LOW
  return (digitalRead(PIN_SRV_LEFT) == LOW);
}

bool HalBoard::readServiceRight() const {
  // aktiv LOW
  return (digitalRead(PIN_SRV_RIGHT) == LOW);
}

uint32_t HalBoard::readIs1mV() const {
  // analogReadMilliVolts liefert einen kalibrierten Wert.
  // Auf manchen Setups (ADC2 / Kalibrierung) kann es vorkommen, dass der
  // Rueckgabewert unplausibel ist. Dann fallen wir auf eine einfache
  // Rohwert-Umrechnung (0..4095 -> 0..3300mV) zurueck.
  uint32_t mv = (uint32_t)analogReadMilliVolts(PIN_IS1_ADC);
  if (mv == 0) {
    // Falls kalibriert 0 zurueckkommt, aber ein Rohwert !=0 vorhanden ist,
    // rechnen wir grob um.
    uint16_t raw0 = analogRead(PIN_IS1_ADC);
    if (raw0 > 0) {
      mv = (uint32_t)raw0 * 3300UL / 4095UL;
    }
  }
  if (mv > 3300) {
    uint16_t raw = analogRead(PIN_IS1_ADC);
    mv = (uint32_t)raw * 3300UL / 4095UL;
  }
  return mv;
}

uint32_t HalBoard::readIs2mV() const {
  uint32_t mv = (uint32_t)analogReadMilliVolts(PIN_IS2_ADC);
  if (mv == 0) {
    uint16_t raw0 = analogRead(PIN_IS2_ADC);
    if (raw0 > 0) {
      mv = (uint32_t)raw0 * 3300UL / 4095UL;
    }
  }
  if (mv > 3300) {
    uint16_t raw = analogRead(PIN_IS2_ADC);
    mv = (uint32_t)raw * 3300UL / 4095UL;
  }
  return mv;
}
