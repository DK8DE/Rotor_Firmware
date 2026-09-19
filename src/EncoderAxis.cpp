#include "EncoderAxis.h"

#include <TWK_KBE58_SSI.h>

// Sicherheitskritisch: Turn muss möglichst sofort in NVS, sonst zeigt ein
// Neustart im Überdrehbereich (z.B. 382°) nur den Rohwinkel (22°).
// Kurze Entprellung nur gegen Wrap-Flattern an der 0/360-Grenze.
static const uint32_t kSsiTurnPersistDebounceMs = 50;

static int32_t clampI32(int32_t v, int32_t lo, int32_t hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static uint32_t mapSsiPositionLogical(uint32_t rawPos, int32_t cpr, bool invert) {
  if (cpr <= 0) return rawPos;
  const uint32_t ucpr = (uint32_t)cpr;
  const uint32_t p = rawPos % ucpr;
  if (!invert) return p;
  return (ucpr - p) % ucpr;
}

EncoderAxis::EncoderAxis() {
}

EncoderAxis::~EncoderAxis() {
  stop();
}

bool EncoderAxis::begin(const EncoderAxisConfig& cfg) {
  stop();
  _cfg = cfg;
  if (_cfg.axisMaxDeg01 < 0) _cfg.axisMaxDeg01 = 0;
  _ssiValid = false;
  _ssiPosition = 0;
  _ssiLastReadCounter = 0;
  _ssiHavePrevPos = false;
  // _ssiTurn bleibt unverändert, wenn setSsiTurn() nach begin() aufgerufen wird;
  // hier auf 0 zurücksetzen und Caller lädt ggf. aus NVS.
  _ssiTurn = 0;
  _ssiTurnDirty = false;
  _ssiTurnChangedMs = 0;

  if (_cfg.encType == ENCTYPE_ABSOLUTE_SSI) {
    return beginSsi();
  }
  return beginPcnt();
}

bool EncoderAxis::beginPcnt() {
  if (_cfg.pinA < 0 || _cfg.pinB < 0) {
    return false;
  }

  _enc = new UltraEncoderPCNT(_cfg.pinA, _cfg.pinB, _cfg.mode, _cfg.cpuCore, _cfg.serviceIntervalUs);
  if (!_enc) {
    return false;
  }

  if (!_enc->begin(0, 0.0f, 0, _cfg.glitchNs)) {
    delete _enc;
    _enc = nullptr;
    return false;
  }

  if (_cfg.zEnabled && _cfg.zPin != 255) {
    _enc->attachZ(_cfg.zPin, _cfg.zActiveHigh);
    _enc->configureZFilter(_cfg.zMinIntervalUs, _cfg.zMinAbsStepsBetween);

    if (_cfg.zCorrEnabled) {
      _enc->enableZDistanceCorrection(true, _cfg.zExpectedStepsBetweenZ, _cfg.zMaxAbsErrorSteps, _cfg.zCorrGain);
    } else {
      _enc->enableZDistanceCorrection(false, 0, 0, 0.0f);
    }
  }

  _enc->setPositionSteps(0);
  return true;
}

bool EncoderAxis::beginSsi() {
#if !defined(ESP32)
  (void)_cfg;
  return false;
#else
  if (_cfg.ssiClockPin < 0 || _cfg.ssiDataPin < 0) {
    return false;
  }

  _ssi = new TWK_KBE58_SSI((uint8_t)_cfg.ssiClockPin, (uint8_t)_cfg.ssiDataPin);
  if (!_ssi) {
    return false;
  }

  _ssi->setSpiMode(SPI_MODE3);

  if (!_ssi->beginESP32PreciseSPI((uint8_t)_cfg.ssiClockPin, (uint8_t)_cfg.ssiDataPin, _cfg.ssiSpiFreqHz)) {
    delete _ssi;
    _ssi = nullptr;
    return false;
  }

  _ssi->setRawBitShift(0);
  _ssi->setFramePauseUs(80);

  if (_cfg.ssiZeroPin >= 0) {
    pinMode((uint8_t)_cfg.ssiZeroPin, OUTPUT);
    digitalWrite((uint8_t)_cfg.ssiZeroPin, HIGH);
    _ssi->configureZeroPin((int8_t)_cfg.ssiZeroPin, _cfg.ssiZeroPulseMs);
  }

  if (!_ssi->startBackgroundRead(_cfg.ssiBgIntervalMs, 1)) {
    delete _ssi;
    _ssi = nullptr;
    return false;
  }

  const uint32_t spr = _ssi->stepsPerRevolution();
  _cfg.countsPerRevActual = (spr > 0) ? (int32_t)spr : 4096;
  return true;
#endif
}

void EncoderAxis::stop() {
  if (_ssi) {
#if defined(ESP32)
    _ssi->stopBackgroundRead();
#endif
    delete _ssi;
    _ssi = nullptr;
  }
  if (_enc) {
    _enc->stop();
    delete _enc;
    _enc = nullptr;
  }
  _ssiValid = false;
  _ssiPosition = 0;
  _ssiLastReadCounter = 0;
  _ssiHavePrevPos = false;
}

int32_t EncoderAxis::ssiOverlapDeg01_() const {
  const int32_t amax = (_cfg.axisMaxDeg01 > 0) ? _cfg.axisMaxDeg01 : 36000;
  int32_t overlap = amax - 36000;
  if (overlap < 0) overlap = 0;
  return overlap;
}

int32_t EncoderAxis::ssiRawDeg01FromCounts_(uint32_t counts) const {
  const int32_t cpr = _cfg.countsPerRevActual;
  if (cpr <= 0) return 0;
  const int64_t num = (int64_t)counts * 36000LL;
  int32_t shaftDeg01 = (int32_t)(num / (int64_t)cpr);
  const uint16_t scNum = (_cfg.ssiAngleScaleNum > 0) ? _cfg.ssiAngleScaleNum : 1;
  const uint16_t scDen = (_cfg.ssiAngleScaleDen > 0) ? _cfg.ssiAngleScaleDen : 1;
  return (int32_t)(((int64_t)shaftDeg01 * (int64_t)scNum) / (int64_t)scDen);
}

void EncoderAxis::markSsiTurnDirty_() {
  _ssiTurnDirty = true;
  _ssiTurnChangedMs = millis();
}

void EncoderAxis::applySsiPosition_(uint32_t newPos) {
  const int32_t cpr = _cfg.countsPerRevActual;
  if (cpr <= 0) {
    _ssiPosition = newPos;
    _ssiHavePrevPos = true;
    return;
  }

  if (_ssiHavePrevPos) {
    const long d = (long)newPos - (long)_ssiPosition;
    const long half = (long)cpr / 2L;
    uint8_t newTurn = _ssiTurn;
    if (d < -half) {
      // Vorwärts über Wrap (z.B. 4090 -> 10)
      newTurn = 1;
    } else if (d > half) {
      // Rückwärts über Wrap (z.B. 10 -> 4090)
      newTurn = 0;
    }
    if (newTurn != _ssiTurn) {
      _ssiTurn = newTurn;
      markSsiTurnDirty_();
    }
  }

  _ssiPosition = newPos;
  _ssiHavePrevPos = true;

  // Selbstheilung: turn=1 und Rohwert außerhalb Überlappungszone
  // (logisch wäre > axisMax) → unmöglich, Turn zurücksetzen.
  if (_ssiTurn != 0) {
    const int32_t overlap = ssiOverlapDeg01_();
    const int32_t rohDeg01 = ssiRawDeg01FromCounts_(_ssiPosition);
    if (rohDeg01 > overlap) {
      _ssiTurn = 0;
      markSsiTurnDirty_();
    }
  }
}

void EncoderAxis::update() {
  if (!_ssi) return;

#if defined(ESP32)
  _ssi->update();

  const uint32_t readCounter = _ssi->getReadCounter();
  if (readCounter != _ssiLastReadCounter) {
    _ssiLastReadCounter = readCounter;
    const TWK_KBE58_SSI::Reading r = _ssi->getLastReading();
    if (r.valid) {
      _ssiValid = true;
      const uint32_t mapped = mapSsiPositionLogical(r.position, _cfg.countsPerRevActual, _cfg.ssiInvertDirection);
      applySsiPosition_(mapped);
    }
  } else if (_ssi->hasNewReading()) {
    const TWK_KBE58_SSI::Reading r = _ssi->getLastReading();
    if (r.valid) {
      _ssiValid = true;
      const uint32_t mapped = mapSsiPositionLogical(r.position, _cfg.countsPerRevActual, _cfg.ssiInvertDirection);
      applySsiPosition_(mapped);
    }
  }
#endif
}

void EncoderAxis::setCountsPerRevActual(int32_t cpr) {
  if (cpr < 0) cpr = -cpr;
  _cfg.countsPerRevActual = cpr;
}

int32_t EncoderAxis::getCountsPerRevEffective() const {
  if (_cfg.countsPerRevActual <= 0) return 0;

  if (_cfg.encType == ENCTYPE_ABSOLUTE_SSI) {
    return _cfg.countsPerRevActual;
  }

  int32_t off = _cfg.rangeDegOffsetDeg01;
  if (off < 0) off = -off;
  if (off == 0) return _cfg.countsPerRevActual;

  const int64_t num = (int64_t)_cfg.countsPerRevActual * 36000LL;
  const int64_t den = 36000LL + (int64_t)off;
  int32_t eff = (int32_t)(num / den);
  if (eff <= 0) eff = 1;
  if (eff > _cfg.countsPerRevActual) eff = _cfg.countsPerRevActual;
  return eff;
}

void EncoderAxis::setRangeDegOffsetDeg01(int32_t offDeg01) {
  if (offDeg01 < 0) offDeg01 = -offDeg01;
  if (offDeg01 > 9000) offDeg01 = 9000;
  _cfg.rangeDegOffsetDeg01 = offDeg01;
}

void EncoderAxis::setAxisMaxDeg01(int32_t maxDeg01) {
  if (maxDeg01 < 0) maxDeg01 = 0;
  _cfg.axisMaxDeg01 = maxDeg01;
}

void EncoderAxis::setEncoderType(EncoderType t) {
  _cfg.encType = t;
}

long EncoderAxis::getCountsRaw() const {
  if (_ssi) return (long)_ssiPosition;
  if (!_enc) return 0;
  return _enc->getPositionStepsRaw();
}

long EncoderAxis::getCountsCorrected() const {
  if (_ssi) return (long)_ssiPosition;
  if (!_enc) return 0;
  return _enc->getPositionStepsCorrected();
}

long EncoderAxis::getCountsDefault() const {
  if (_ssi) return (long)_ssiPosition;
  if (!_enc) return 0;
  return _enc->getPositionSteps();
}

long EncoderAxis::getCountsExtended() const {
  if (!_ssi) {
    return getCountsDefault();
  }
  const int32_t cpr = _cfg.countsPerRevActual;
  if (cpr <= 0) return (long)_ssiPosition;
  return (long)_ssiTurn * (long)cpr + (long)_ssiPosition;
}

void EncoderAxis::setCountsZero() {
  if (_ssi) return;
  if (!_enc) return;
  _enc->setPositionSteps(0);
  _enc->resetZHistory();
}

bool EncoderAxis::setEncZero() {
  if (_cfg.encType != ENCTYPE_ABSOLUTE_SSI || !_ssi || _cfg.ssiZeroPin < 0) {
    return false;
  }
#if defined(ESP32)
  _ssi->setZero();
  if (_ssiTurn != 0) {
    _ssiTurn = 0;
    markSsiTurnDirty_();
  }
  return true;
#else
  return false;
#endif
}

void EncoderAxis::setSsiTurn(uint8_t turn) {
  const uint8_t t = (turn != 0) ? 1 : 0;
  if (_ssiTurn == t) return;
  _ssiTurn = t;
  markSsiTurnDirty_();
}

void EncoderAxis::loadSsiTurn(uint8_t turn) {
  _ssiTurn = (turn != 0) ? 1 : 0;
  _ssiTurnDirty = false;
  _ssiTurnChangedMs = 0;
}

bool EncoderAxis::consumeSsiTurnDirty(uint32_t nowMs, bool /*standstill*/) {
  // standstill wird absichtlich ignoriert: Abschalten während der Fahrt
  // darf den Turn nicht verlieren (Kabelbruch-Risiko).
  if (!_ssiTurnDirty) return false;
  if ((uint32_t)(nowMs - _ssiTurnChangedMs) < kSsiTurnPersistDebounceMs) return false;
  return true;
}

void EncoderAxis::clearSsiTurnDirty() {
  _ssiTurnDirty = false;
}

void EncoderAxis::setCounts(long newCounts) {
  if (_ssi) {
    if (newCounts < 0) newCounts = 0;
    const int32_t cpr = _cfg.countsPerRevActual;
    if (cpr > 0 && newCounts >= cpr) newCounts = cpr - 1;
    _ssiPosition = (uint32_t)newCounts;
    _ssiHavePrevPos = true;
    return;
  }
  if (!_enc) return;
  _enc->setPositionSteps(newCounts);
  _enc->resetZHistory();
}

bool EncoderAxis::getPositionDeg01(int32_t& outDeg01) const {
  const int32_t cprActual = _cfg.countsPerRevActual;
  if (cprActual <= 0) return false;

  if (_cfg.encType == ENCTYPE_ABSOLUTE_SSI) {
    if (!_ssiValid) return false;
    int32_t deg01 = ssiRawDeg01FromCounts_(_ssiPosition);
    if (_ssiTurn != 0) {
      deg01 += 36000;
    }
    const int32_t amax = (_cfg.axisMaxDeg01 > 0) ? _cfg.axisMaxDeg01 : 36000;
    deg01 = clampI32(deg01, 0, amax);
    outDeg01 = deg01;
    return true;
  }

  // Typ 1/2 (PCNT, nicht-absolut): die beim Homing gelernten Counts (End-
  // schalter zu Endschalter) werden auf axisMaxDeg01 (SETMAXDG) verteilt,
  // NICHT fest auf 360°. Ein per SETMAXDG auf z.B. 180° begrenzter Rotor
  // (z.B. Elevation) hat dann 1 logisches Grad = 1 reales Grad; vorher wurden
  // die Counts immer auf 360° verteilt, wodurch die Achse effektiv nur die
  // Haelfte ihres realen Hubs nutzen konnte.
  const int32_t amax = (_cfg.axisMaxDeg01 > 0) ? _cfg.axisMaxDeg01 : 36000;

  int32_t off = _cfg.rangeDegOffsetDeg01;
  if (off < 0) off = -off;

  const int32_t halfOff = off / 2;
  const int32_t totalDeg01 = amax + off;

  const long c = getCountsDefault();

  int64_t num = (int64_t)c * (int64_t)totalDeg01;
  int32_t physDeg01 = (int32_t)(num / (int64_t)cprActual);
  int32_t logDeg01 = physDeg01 - halfOff;

  logDeg01 = clampI32(logDeg01, 0, amax);

  outDeg01 = logDeg01;
  return true;
}

bool EncoderAxis::deg01ToCounts(int32_t deg01, int32_t& outCounts) const {
  const int32_t cprActual = _cfg.countsPerRevActual;
  if (cprActual <= 0) return false;

  if (_cfg.encType == ENCTYPE_ABSOLUTE_SSI) {
    const int32_t amax = (_cfg.axisMaxDeg01 > 0) ? _cfg.axisMaxDeg01 : 36000;
    deg01 = clampI32(deg01, 0, amax);
    // Eine Encoder-Umdrehung = 360°; Ziel >360° mappt auf denselben Roh-Count.
    int32_t shaftLogDeg01 = deg01;
    if (shaftLogDeg01 >= 36000) {
      shaftLogDeg01 -= 36000;
    }
    const uint16_t scNum = (_cfg.ssiAngleScaleNum > 0) ? _cfg.ssiAngleScaleNum : 1;
    const uint16_t scDen = (_cfg.ssiAngleScaleDen > 0) ? _cfg.ssiAngleScaleDen : 1;
    int64_t shaftNum = (int64_t)shaftLogDeg01 * (int64_t)scDen;
    int32_t shaftDeg01 = (int32_t)(shaftNum / (int64_t)scNum);
    shaftDeg01 = clampI32(shaftDeg01, 0, 36000);
    int64_t num = (int64_t)shaftDeg01 * (int64_t)cprActual;
    int32_t counts = (int32_t)(num / 36000LL);
    if (counts >= cprActual) counts = cprActual - 1;
    if (counts < 0) counts = 0;
    outCounts = counts;
    return true;
  }

  // Typ 1/2 (PCNT, nicht-absolut): siehe getPositionDeg01() — Spanne ist
  // axisMaxDeg01 (SETMAXDG), nicht fest 360°.
  const int32_t amax = (_cfg.axisMaxDeg01 > 0) ? _cfg.axisMaxDeg01 : 36000;
  deg01 = clampI32(deg01, 0, amax);

  int32_t off = _cfg.rangeDegOffsetDeg01;
  if (off < 0) off = -off;

  const int32_t halfOff = off / 2;
  const int32_t totalDeg01 = amax + off;

  const int32_t physDeg01 = deg01 + halfOff;

  int64_t num = (int64_t)physDeg01 * (int64_t)cprActual;
  int32_t counts = (int32_t)(num / (int64_t)totalDeg01);
  outCounts = counts;
  return true;
}

bool EncoderAxis::deg01ToCountsExtended(int32_t deg01, int32_t& outCounts) const {
  const int32_t cprActual = _cfg.countsPerRevActual;
  if (cprActual <= 0) return false;

  if (_cfg.encType != ENCTYPE_ABSOLUTE_SSI) {
    return deg01ToCounts(deg01, outCounts);
  }

  const int32_t amax = (_cfg.axisMaxDeg01 > 0) ? _cfg.axisMaxDeg01 : 36000;
  deg01 = clampI32(deg01, 0, amax);

  const int32_t turn = (deg01 >= 36000) ? 1 : 0;
  int32_t inRevDeg01 = deg01;
  if (turn != 0) {
    inRevDeg01 -= 36000;
  }

  const uint16_t scNum = (_cfg.ssiAngleScaleNum > 0) ? _cfg.ssiAngleScaleNum : 1;
  const uint16_t scDen = (_cfg.ssiAngleScaleDen > 0) ? _cfg.ssiAngleScaleDen : 1;
  int64_t shaftNum = (int64_t)inRevDeg01 * (int64_t)scDen;
  int32_t shaftDeg01 = (int32_t)(shaftNum / (int64_t)scNum);
  shaftDeg01 = clampI32(shaftDeg01, 0, 36000);
  int64_t num = (int64_t)shaftDeg01 * (int64_t)cprActual;
  int32_t countsInRev = (int32_t)(num / 36000LL);
  if (countsInRev >= cprActual) countsInRev = cprActual - 1;
  if (countsInRev < 0) countsInRev = 0;

  outCounts = turn * cprActual + countsInRev;
  return true;
}

EncoderZStats EncoderAxis::getZStats() const {
  EncoderZStats s;
  if (!_enc) return s;

  s.enabled = _cfg.zEnabled;
  if (!_cfg.zEnabled) return s;

  s.zCount = _enc->getZPulseCount();
  s.dzSteps = _enc->getLastZDistanceSteps();
  s.dzUs = _enc->getLastZDistanceUs();
  s.zErrSteps = _enc->getLastZErrorSteps();
  s.corrOffsetSteps = _enc->getCorrectionOffsetSteps();
  return s;
}
