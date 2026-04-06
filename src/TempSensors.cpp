#include "TempSensors.h"

// ----------------------------------------------------------------------------
// Helper: Float -> 0,01C
// ----------------------------------------------------------------------------
static int32_t floatToScaled100(float tC) {
  // Schutz: bei ungueltigen Werten (NaN) liefert Arduino typischerweise NaN.
  // Wir behandeln das hier als 0.
  if (isnan(tC)) return 0;
  // Grobe Plausibilitaet: DS18B20 kann -55..125C.
  if (tC < -100.0f || tC > 200.0f) return 0;
  // Rundung auf 0,01C
  float s = tC * 100.0f;
  if (s >= 0.0f) return (int32_t)(s + 0.5f);
  return (int32_t)(s - 0.5f);
}

// ----------------------------------------------------------------------------
// Mapping-Helfer
// ----------------------------------------------------------------------------
uint8_t TempSensors::mapAmbientIndex() const {
  // Logisch Umgebung
  return _swapTemp ? 1 : 0;
}

uint8_t TempSensors::mapMotorIndex() const {
  // Logisch Motor
  return _swapTemp ? 0 : 1;
}

bool TempSensors::begin(uint8_t pin, bool enableMotorSensor, uint32_t intervalMs) {
  _pin = pin;
  _intervalMs = (intervalMs < 250) ? 250 : intervalMs;
  _motorEnabled = enableMotorSensor;

  // Sensor-Objekt neu erzeugen (Pin sitzt im Konstruktor)
  if (_sensor) {
    delete _sensor;
    _sensor = nullptr;
  }
  _sensor = new DS18B20_7semi(_pin);

  _devCount = 0;
  _dev0Present = false;
  _dev1Present = false;

  _dev0Scaled100 = 0;
  _dev1Scaled100 = 0;

  if (!_sensor) return false;

  if (!_sensor->begin()) {
    // Kein DS18B20 gefunden
    _devCount = 0;
  } else {
    _devCount = _sensor->searchDevices();

    // Adresse 0 (Device 0)
    if (_devCount >= 1) {
      if (_sensor->getAddress(0, _addr0)) {
        _dev0Present = true;
      }
    }

    // Adresse 1 (Device 1)
    if (_devCount >= 2) {
      if (_sensor->getAddress(1, _addr1)) {
        _dev1Present = true;
      }
    }
  }

  // Task starten, falls noch nicht vorhanden
  if (_task == nullptr) {
    // Task auf Core 0 reicht; Prio niedrig.
    xTaskCreatePinnedToCore(taskThunk,
                            "TempSensors",
                            4096,
                            this,
                            1,
                            &_task,
                            0);
  }

  return (_devCount > 0);
}

void TempSensors::setMotorSensorEnabled(bool on) {
  _motorEnabled = on;
  // Wenn deaktiviert, soll GETTEMPM sofort 0 liefern.
  // (Die physische Messung laeuft weiter, damit "Swap" weiterhin sinnvoll bleibt.)
}

void TempSensors::setSwapTemp(bool on) {
  _swapTemp = on;
  // Keine weitere Aktion noetig: Getter mappen immer aus den physischen Caches.
}

bool TempSensors::hasAmbient() const {
  const uint8_t idx = mapAmbientIndex();
  return (idx == 0) ? _dev0Present : _dev1Present;
}

bool TempSensors::hasMotor() const {
  if (!_motorEnabled) return false;
  const uint8_t idx = mapMotorIndex();
  return (idx == 0) ? _dev0Present : _dev1Present;
}

float TempSensors::getAmbientC() const {
  return ((float)getAmbientScaled100()) / 100.0f;
}

float TempSensors::getMotorC() const {
  return ((float)getMotorScaled100()) / 100.0f;
}

int32_t TempSensors::getAmbientScaled100() const {
  // Atomar genug fuer 32bit auf ESP32
  const uint8_t idx = mapAmbientIndex();
  if (idx == 0) {
    return _dev0Present ? _dev0Scaled100 : 0;
  }
  return _dev1Present ? _dev1Scaled100 : 0;
}

int32_t TempSensors::getMotorScaled100() const {
  // Wenn Motor-Sensor deaktiviert ist: immer 0
  if (!_motorEnabled) return 0;

  const uint8_t idx = mapMotorIndex();
  if (idx == 0) {
    return _dev0Present ? _dev0Scaled100 : 0;
  }
  return _dev1Present ? _dev1Scaled100 : 0;
}

void TempSensors::taskThunk(void* arg) {
  TempSensors* self = (TempSensors*)arg;
  if (!self) {
    vTaskDelete(nullptr);
    return;
  }
  self->taskLoop();
}

void TempSensors::taskLoop() {
  // Kurze Startverzoegerung (damit Serial/Board init fertig wird)
  vTaskDelay(200 / portTICK_PERIOD_MS);

  while (true) {
    doOneRead();
    vTaskDelay(_intervalMs / portTICK_PERIOD_MS);
  }
}

void TempSensors::doOneRead() {
  if (!_sensor) return;

  // Wir lesen *immer* alle gefundenen Devices, unabhaengig von _motorEnabled.
  // Grund:
  // - Wenn Swap aktiv ist, kann die Umgebung ggf. auf Device 1 liegen.
  // - Auch wenn der Motor-Sensor (logisch) deaktiviert ist, wollen wir die
  //   Umgebung trotzdem korrekt liefern.

  // Device 0
  if (_dev0Present) {
    float t = _sensor->readTemperature(_addr0);
    _dev0Scaled100 = floatToScaled100(t);
  } else {
    _dev0Scaled100 = 0;
  }

  // Device 1
  if (_dev1Present) {
    float t = _sensor->readTemperature(_addr1);
    _dev1Scaled100 = floatToScaled100(t);
  } else {
    _dev1Scaled100 = 0;
  }
}
