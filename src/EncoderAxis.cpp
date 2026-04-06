#include "EncoderAxis.h"

static int32_t clampI32(int32_t v, int32_t lo, int32_t hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

EncoderAxis::EncoderAxis() {
}

EncoderAxis::~EncoderAxis() {
  stop();
}

bool EncoderAxis::begin(const EncoderAxisConfig& cfg) {
  stop();

  _cfg = cfg;

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

  // Start-Referenz
  _enc->setPositionSteps(0);
  return true;
}

void EncoderAxis::stop() {
  if (_enc) {
    _enc->stop();
    delete _enc;
    _enc = nullptr;
  }
}

void EncoderAxis::setCountsPerRevActual(int32_t cpr) {
  if (cpr < 0) cpr = -cpr;
  _cfg.countsPerRevActual = cpr;
}

int32_t EncoderAxis::getCountsPerRevEffective() const {
  if (_cfg.countsPerRevActual <= 0) return 0;

  int32_t off = _cfg.rangeDegOffsetDeg01;
  if (off < 0) off = -off;
  if (off == 0) return _cfg.countsPerRevActual;

  // cprEff = cprActual * 36000 / (36000 + off)
  const int64_t num = (int64_t)_cfg.countsPerRevActual * 36000LL;
  const int64_t den = 36000LL + (int64_t)off;
  int32_t eff = (int32_t)(num / den);
  if (eff <= 0) eff = 1;
  if (eff > _cfg.countsPerRevActual) eff = _cfg.countsPerRevActual;
  return eff;
}

void EncoderAxis::setRangeDegOffsetDeg01(int32_t offDeg01) {
  if (offDeg01 < 0) offDeg01 = -offDeg01;
  // Grobe Begrenzung: mehr als 90 Grad Offset macht praktisch keinen Sinn.
  if (offDeg01 > 9000) offDeg01 = 9000;
  _cfg.rangeDegOffsetDeg01 = offDeg01;
}

void EncoderAxis::setEncoderType(EncoderType t) {
  _cfg.encType = t;
}

long EncoderAxis::getCountsRaw() const {
  if (!_enc) return 0;
  return _enc->getPositionStepsRaw();
}

long EncoderAxis::getCountsCorrected() const {
  if (!_enc) return 0;
  return _enc->getPositionStepsCorrected();
}

long EncoderAxis::getCountsDefault() const {
  if (!_enc) return 0;
  return _enc->getPositionSteps();
}

void EncoderAxis::setCountsZero() {
  if (!_enc) return;
  _enc->setPositionSteps(0);
  _enc->resetZHistory();
}

void EncoderAxis::setCounts(long newCounts) {
  if (!_enc) return;
  _enc->setPositionSteps(newCounts);
  _enc->resetZHistory();
}

bool EncoderAxis::getPositionDeg01(int32_t& outDeg01) const {
  // Wir rechnen hier bewusst mit der *gelernten* Endschalterstrecke
  // (countsPerRevActual) und dem Range-Offset, um den logischen 0..360deg
  // Bereich symmetrisch in die reale Endschalterstrecke zu legen.
  const int32_t cprActual = _cfg.countsPerRevActual;
  if (cprActual <= 0) return false;

  // Off immer positiv behandeln.
  int32_t off = _cfg.rangeDegOffsetDeg01;
  if (off < 0) off = -off;

  // halfOff = off/2 (Deg01)
  const int32_t halfOff = off / 2;
  const int32_t totalDeg01 = 36000 + off;  // reale Endschalterstrecke in Deg01

  const long c = getCountsDefault();

  // physDeg01 = counts * totalDeg01 / cprActual
  // logDeg01  = physDeg01 - halfOff
  int64_t num = (int64_t)c * (int64_t)totalDeg01;
  int32_t physDeg01 = (int32_t)(num / (int64_t)cprActual);
  int32_t logDeg01 = physDeg01 - halfOff;

  // Clamp auf 0..36000 (harte logische Grenzen)
  logDeg01 = clampI32(logDeg01, 0, 36000);

  outDeg01 = logDeg01;
  return true;
}

bool EncoderAxis::deg01ToCounts(int32_t deg01, int32_t& outCounts) const {
  // Siehe Kommentar in getPositionDeg01():
  // logDeg01 0..36000 wird symmetrisch in die reale Endschalterstrecke gelegt.
  const int32_t cprActual = _cfg.countsPerRevActual;
  if (cprActual <= 0) return false;

  int32_t off = _cfg.rangeDegOffsetDeg01;
  if (off < 0) off = -off;

  const int32_t halfOff = off / 2;
  const int32_t totalDeg01 = 36000 + off;

  deg01 = clampI32(deg01, 0, 36000);

  // physDeg01 = logDeg01 + halfOff
  const int32_t physDeg01 = deg01 + halfOff;

  // counts = physDeg01 * cprActual / totalDeg01
  int64_t num = (int64_t)physDeg01 * (int64_t)cprActual;
  int32_t counts = (int32_t)(num / (int64_t)totalDeg01);
  outCounts = counts;
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
