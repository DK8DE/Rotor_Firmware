#include "HalBoard.h"

#include <Wire.h>
#include <math.h>
#include <cstring>

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

// Inklinometer (IIS2ICLX) am Wind-RS485: #GETDG$ @ 115200 8N1
static const uint32_t INCLINO_POLL_INTERVAL_MS = 80;
static const uint32_t INCLINO_STALE_MS = 500;
static const uint32_t INCLINO_REPLY_TIMEOUT_MS = 80;
static const uint32_t INCLINO_TRANSACT_TIMEOUT_MS = 100;
static const BaseType_t INCLINO_TASK_PRIORITY = 1;
static const uint32_t INCLINO_TASK_STACK_WORDS = 4096;
static const BaseType_t INCLINO_TASK_CORE = 0;

void HalBoard::begin() {
  // I2C-Bus vorbereiten (Bausteine folgen spaeter)
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

  // ------------------------------------------------------------
  // Endschalter aktiv LOW (gedrueckt = LOW)
  // Hinweis:
  // - In dieser Projekt-Hardware sind Pullups extern vorhanden.
  // - Daher wird hier INPUT verwendet (kein interner Pullup).
  //   Ohne externe Pullups: INPUT_PULLUP verwenden.
  // ------------------------------------------------------------
  // LED aktiv LOW
  pinMode(PIN_LED_LB, OUTPUT);
  digitalWrite(PIN_LED_LB, HIGH); // aus

  // ------------------------------------------------------------
  // Service-Taster aktiv LOW: interner Pullup
  // ------------------------------------------------------------
  pinMode(PIN_SRV_LEFT, INPUT_PULLUP);
  pinMode(PIN_SRV_RIGHT, INPUT_PULLUP);

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


  if (_windSensorMutex == nullptr) {
    _windSensorMutex = xSemaphoreCreateMutex();
  }
  if (_inclinoBusMutex == nullptr) {
    _inclinoBusMutex = xSemaphoreCreateMutex();
  }
}

void HalBoard::applyHardwareProfile(EncoderType encType) {
  if (encType == ENCTYPE_ABSOLUTE_SSI) {
    _windHardwareAvailable = false;
    _windInit = false;
    _inclinoHardwareAvailable = false;
    _inclinoInit = false;

    pinMode(PIN_OE_INVERTER, OUTPUT);
    digitalWrite(PIN_OE_INVERTER, HIGH);
    return;
  }

  if (encType == ENCTYPE_INCLINOMETER) {
    // Wind und Inklinometer teilen Serial2 / Pins 8+9 — gegenseitig ausschliessend.
    _windHardwareAvailable = false;
    _windInit = false;
    _inclinoHardwareAvailable = true;

    pinMode(PIN_END_LEFT, INPUT);
    pinMode(PIN_END_RIGHT, INPUT);

    pinMode(PIN_WIND_RS485_RX, INPUT_PULLUP);

    pinMode(PIN_OE_INVERTER, OUTPUT);
    digitalWrite(PIN_OE_INVERTER, LOW);

    Serial2.begin(115200, SERIAL_8N1, PIN_WIND_RS485_RX, PIN_WIND_RS485_TX);
    Serial2.setTimeout(INCLINO_REPLY_TIMEOUT_MS);
    _inclinoInit = true;
    return;
  }

  _inclinoHardwareAvailable = false;
  _inclinoInit = false;
  _windHardwareAvailable = true;

  pinMode(PIN_END_LEFT, INPUT);
  pinMode(PIN_END_RIGHT, INPUT);

  pinMode(PIN_WIND_RS485_RX, INPUT_PULLUP);

  pinMode(PIN_OE_INVERTER, OUTPUT);
  digitalWrite(PIN_OE_INVERTER, LOW);

  // deRePin=-1: THVD1406DR schaltet TX/RX automatisch
  _wind.begin(Serial2, PIN_WIND_RS485_RX, PIN_WIND_RS485_TX, -1, 0x01, 9600);
  _windInit = true;

  _wind.setSpeedOffsetMps(0.0f);
  _wind.setAngleOffsetDeg(0.0f);
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

// ------------------------------------------------------------
// Inklinometer (Serial2 @115200, #GETDG$)
// ------------------------------------------------------------

void HalBoard::startInclinoTask() {
  if (!_inclinoInit) return;
  if (_inclinoTaskHandle != nullptr) return;

  portENTER_CRITICAL(&_inclinoStateMux);
  _inclinoValid = false;
  _inclinoDeg01 = 0;
  _inclinoLastOkMs = 0;
  portEXIT_CRITICAL(&_inclinoStateMux);

  xTaskCreatePinnedToCore(
    HalBoard::inclinoTaskTrampoline,
    "inclino_task",
    INCLINO_TASK_STACK_WORDS,
    this,
    INCLINO_TASK_PRIORITY,
    &_inclinoTaskHandle,
    INCLINO_TASK_CORE
  );
}

void HalBoard::inclinoTaskTrampoline(void* arg) {
  HalBoard* self = static_cast<HalBoard*>(arg);
  if (self) {
    self->inclinoTaskLoop();
  }
  vTaskDelete(nullptr);
}

void HalBoard::inclinoTaskLoop() {
  TickType_t lastWake = xTaskGetTickCount();
  for (;;) {
    const uint32_t nowMs = millis();
    inclinoTaskStep(nowMs);
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(INCLINO_POLL_INTERVAL_MS));
  }
}

HalBoard::InclinoTxStatus HalBoard::inclinoTransact(const char* reqTail, String& outValue,
                                                    uint32_t timeoutMs) {
  outValue = "";
  if (!_inclinoInit || !reqTail || !reqTail[0]) {
    return INCLINO_TX_NOHW;
  }
  if (timeoutMs == 0) timeoutMs = INCLINO_TRANSACT_TIMEOUT_MS;

  // Erwarteter ACK-Name = Token vor dem ersten Komma in reqTail
  String ackName;
  {
    const char* comma = strchr(reqTail, ',');
    if (comma) {
      ackName = String(reqTail).substring(0, (unsigned)(comma - reqTail));
    } else {
      ackName = String(reqTail);
    }
  }
  const String ackPrefix = String("#ACK_") + ackName + ",";
  const String nackPrefix = String("#NACK_") + ackName;

  if (_inclinoBusMutex) {
    xSemaphoreTake(_inclinoBusMutex, portMAX_DELAY);
  }

  while (Serial2.available() > 0) {
    (void)Serial2.read();
  }
  Serial2.print('#');
  Serial2.print(reqTail);
  Serial2.print('$');
  Serial2.flush();

  const uint32_t t0 = millis();
  String line;
  line.reserve(64);
  InclinoTxStatus st = INCLINO_TX_TIMEOUT;

  while ((uint32_t)(millis() - t0) < timeoutMs) {
    while (Serial2.available() > 0) {
      const char c = (char)Serial2.read();
      if (c == '$') {
        if (line.indexOf(nackPrefix) >= 0) {
          st = INCLINO_TX_NACK;
        } else {
          const int ackPos = line.indexOf(ackPrefix);
          if (ackPos >= 0) {
            outValue = line.substring(ackPos + ackPrefix.length());
            outValue.trim();
            st = INCLINO_TX_OK;
          } else {
            st = INCLINO_TX_BAD;
          }
        }
        goto inclino_tx_done;
      }
      if (c == '\r' || c == '\n') continue;
      if (line.length() < 96) line += c;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }

inclino_tx_done:
  if (_inclinoBusMutex) {
    xSemaphoreGive(_inclinoBusMutex);
  }
  return st;
}

bool HalBoard::inclinoPollOnce(uint32_t nowMs) {
  if (!_inclinoInit) return false;

  String payload;
  const InclinoTxStatus st =
      inclinoTransact("GETDG", payload, INCLINO_REPLY_TIMEOUT_MS);
  if (st != INCLINO_TX_OK) return false;

  payload.replace(",", ".");
  const float deg = payload.toFloat();
  if (!isfinite(deg)) return false;

  const int32_t deg01 = (int32_t)lroundf(deg * 100.0f);
  portENTER_CRITICAL(&_inclinoStateMux);
  _inclinoDeg01 = deg01;
  _inclinoValid = true;
  _inclinoLastOkMs = nowMs;
  portEXIT_CRITICAL(&_inclinoStateMux);
  return true;
}

void HalBoard::inclinoTaskStep(uint32_t nowMs) {
  if (!_inclinoInit) return;
  (void)inclinoPollOnce(nowMs);

  // Stale markieren, falls zu lange kein OK
  portENTER_CRITICAL(&_inclinoStateMux);
  if (_inclinoValid && _inclinoLastOkMs != 0U &&
      (uint32_t)(nowMs - _inclinoLastOkMs) > INCLINO_STALE_MS) {
    _inclinoValid = false;
  }
  portEXIT_CRITICAL(&_inclinoStateMux);
}

bool HalBoard::getInclinoDeg01(int32_t& outDeg01) const {
  bool valid = false;
  int32_t deg01 = 0;
  uint32_t lastOkMs = 0;

  portENTER_CRITICAL(const_cast<portMUX_TYPE*>(&_inclinoStateMux));
  valid = _inclinoValid;
  deg01 = _inclinoDeg01;
  lastOkMs = _inclinoLastOkMs;
  portEXIT_CRITICAL(const_cast<portMUX_TYPE*>(&_inclinoStateMux));

  if (!valid || lastOkMs == 0U) return false;
  const uint32_t nowMs = millis();
  if ((uint32_t)(nowMs - lastOkMs) > INCLINO_STALE_MS) return false;

  outDeg01 = deg01;
  return true;
}

bool HalBoard::isInclinoValid() const {
  int32_t dummy = 0;
  return getInclinoDeg01(dummy);
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

void HalBoard::setEndstopCount(uint8_t count) {
  if (count > 2) count = 2;
  _endstopCount = count;
}

bool HalBoard::readEndContact() const {
  return readEndLeft() || readEndRight();
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
