#include "RotorStore.h"

#include <string.h>

// ============================================================================
// CRC32 (IEEE, poly 0xEDB88320)
// ============================================================================
uint32_t RotorStore::crc32(const uint8_t* data, size_t len) {
  uint32_t c = 0xFFFFFFFFu;
  for (size_t i = 0; i < len; i++) {
    c ^= data[i];
    for (int k = 0; k < 8; k++) {
      c = (c & 1u) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
    }
  }
  return c ^ 0xFFFFFFFFu;
}

uint32_t RotorStore::computePayloadCrc() const {
  uint32_t c = 0xFFFFFFFFu;
  auto feed = [&](const uint8_t* p, size_t n) {
    for (size_t i = 0; i < n; i++) {
      c ^= p[i];
      for (int k = 0; k < 8; k++) {
        c = (c & 1u) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
      }
    }
  };
  feed(reinterpret_cast<const uint8_t*>(&_cfgCache), sizeof(_cfgCache));
  feed(reinterpret_cast<const uint8_t*>(&_calCache), sizeof(_calCache));
  // Runtime: nur genutzte Felder (Turn + lastDeg), Rest ist Reserve/Pad
  feed(reinterpret_cast<const uint8_t*>(&_rtCache), 8);
  return c ^ 0xFFFFFFFFu;
}

void RotorStore::copyStr(char dst[10], const String& s) {
  memset(dst, 0, 10);
  const size_t n = (s.length() > 9) ? 9 : (size_t)s.length();
  if (n > 0) memcpy(dst, s.c_str(), n);
}

bool RotorStore::nvsHasData(Preferences& prefs) {
  // Ein bekannter Key reicht als Indikator fuer bestehende Konfiguration.
  return prefs.isKey("id") || prefs.isKey("ect") || prefs.isKey("amax") ||
         prefs.isKey("calv") || prefs.isKey("sst") || prefs.isKey("ramp");
}

bool RotorStore::headerValid(const RotorStoreHeader& h, uint32_t expectCrc) const {
  if (h.magic != FRAM_MAGIC) return false;
  if (h.layoutVersion < FRAM_LAYOUT_VER_MIN || h.layoutVersion > FRAM_LAYOUT_VER) return false;
  if (h.crc32 != expectCrc) return false;
  return true;
}

uint32_t RotorStore::computePosCrc(const RotorPersistPos& p) const {
  return crc32(reinterpret_cast<const uint8_t*>(&p) + FRAM_POS_CRC_OFF, FRAM_POS_CRC_LEN);
}

bool RotorStore::posValid(const RotorPersistPos& p) const {
  if (p.magic != FRAM_POS_MAGIC) return false;
  if (p.version != FRAM_POS_VER) return false;
  return p.crc32 == computePosCrc(p);
}

bool RotorStore::readPosSection() {
  if (!_fram.read(FRAM_OFF_POS, reinterpret_cast<uint8_t*>(&_posCache), sizeof(_posCache))) {
    _posCacheValid = false;
    return false;
  }
  _posCacheValid = posValid(_posCache);
  return _posCacheValid;
}

bool RotorStore::writePosSection() {
  _posCache.magic = FRAM_POS_MAGIC;
  _posCache.version = FRAM_POS_VER;
  _posCache.crc32 = computePosCrc(_posCache);
  if (!_fram.write(FRAM_OFF_POS, reinterpret_cast<const uint8_t*>(&_posCache), sizeof(_posCache))) {
    return false;
  }
  _posCacheValid = true;
  return true;
}

bool RotorStore::readAllSections() {
  if (!_fram.read(FRAM_OFF_CONFIG, reinterpret_cast<uint8_t*>(&_cfgCache), sizeof(_cfgCache))) return false;
  if (!_fram.read(FRAM_OFF_CAL, reinterpret_cast<uint8_t*>(&_calCache), sizeof(_calCache))) return false;
  if (!_fram.read(FRAM_OFF_RUNTIME, reinterpret_cast<uint8_t*>(&_rtCache), sizeof(_rtCache))) return false;
  return true;
}

bool RotorStore::writeAllSections() {
  RotorStoreHeader h{};
  h.magic = FRAM_MAGIC;
  h.layoutVersion = FRAM_LAYOUT_VER;
  h.flags = 1;
  memset(h.reserved, 0, sizeof(h.reserved));
  h.crc32 = computePayloadCrc();

  if (!_fram.write(FRAM_OFF_HEADER, reinterpret_cast<const uint8_t*>(&h), sizeof(h))) return false;
  if (!_fram.write(FRAM_OFF_CONFIG, reinterpret_cast<const uint8_t*>(&_cfgCache), sizeof(_cfgCache))) return false;
  if (!_fram.write(FRAM_OFF_CAL, reinterpret_cast<const uint8_t*>(&_calCache), sizeof(_calCache))) return false;
  if (!_fram.write(FRAM_OFF_RUNTIME, reinterpret_cast<const uint8_t*>(&_rtCache), sizeof(_rtCache))) return false;
  // RESERVED_POS (0x0700) bewusst unangetastet
  return true;
}

void RotorStore::packConfigFromRefs(RotorPersistConfig& out) const {
  memset(&out, 0, sizeof(out));
  const RotorStoreRefs& r = _refs;

  if (r.slaveId) out.slaveId = *r.slaveId;
  if (r.rotorType) out.rotorType = *r.rotorType;
  if (r.antSelect) out.antSelect = *r.antSelect;
  if (r.encType) {
    if (*r.encType == ENCTYPE_MOTOR_AXIS) out.encTypeU8 = 1;
    else if (*r.encType == ENCTYPE_ABSOLUTE_SSI) out.encTypeU8 = 3;
    else out.encTypeU8 = 2;
  }

  if (r.axisMinDeg01) out.axisMinDeg01 = *r.axisMinDeg01;
  if (r.axisMaxDeg01) out.axisMaxDeg01 = *r.axisMaxDeg01;
  if (r.dgOffsetDeg01) out.dgOffsetDeg01 = *r.dgOffsetDeg01;
  if (r.homePosDeg01) out.homePosDeg01 = *r.homePosDeg01;
  if (r.dgCalDeg01) out.dgCalDeg01 = *r.dgCalDeg01;
  if (r.arriveTolDeg01) out.arriveTolDeg01 = *r.arriveTolDeg01;
  if (r.homeExpectedCountsRing) out.homeExpectedCountsRing = *r.homeExpectedCountsRing;
  if (r.homeExpectedCountsMotor) out.homeExpectedCountsMotor = *r.homeExpectedCountsMotor;

  if (r.homeTimeoutMs) out.homeTimeoutMs = *r.homeTimeoutMs;
  if (r.posTimeoutMs) out.posTimeoutMs = *r.posTimeoutMs;
  if (r.cmdTimeoutMs) out.cmdTimeoutMs = *r.cmdTimeoutMs;
  if (r.jogTimeoutMs) out.jogTimeoutMs = *r.jogTimeoutMs;
  if (r.isSoftWarnMv) out.isSoftWarnMv = *r.isSoftWarnMv;
  if (r.isHardStopMv) out.isHardStopMv = *r.isHardStopMv;
  if (r.stallTimeoutMs) out.stallTimeoutMs = *r.stallTimeoutMs;
  if (r.stallMinCounts) out.stallMinCounts = *r.stallMinCounts;
  if (r.isGraceMs) out.isGraceMs = *r.isGraceMs;
  if (r.isHardHoldMs) out.isHardHoldMs = *r.isHardHoldMs;
  if (r.antDis1) out.antDis1 = *r.antDis1;
  if (r.antDis2) out.antDis2 = *r.antDis2;
  if (r.antDis3) out.antDis3 = *r.antDis3;

  if (r.homeFastPwmPercent) out.homeFastPwmPercent = *r.homeFastPwmPercent;
  if (r.homeBackoff) out.homeBackoff = *r.homeBackoff;
  if (r.homeBacklashMeasScale) out.homeBacklashMeasScale = *r.homeBacklashMeasScale;
  if (r.handSpeedPercent) out.handSpeedPercent = *r.handSpeedPercent;
  if (r.rampDistDeg) out.rampDistDeg = *r.rampDistDeg;
  if (r.minPwm) out.minPwm = *r.minPwm;
  if (r.minStallPwm) out.minStallPwm = *r.minStallPwm;
  if (r.homeSeekMinPwmPercent) out.homeSeekMinPwmPercent = *r.homeSeekMinPwmPercent;
  if (r.tempWarnAmbientC) out.tempWarnAmbientC = *r.tempWarnAmbientC;
  if (r.tempWarnMotorC) out.tempWarnMotorC = *r.tempWarnMotorC;
  if (r.coldTempDegC) out.coldTempDegC = *r.coldTempDegC;
  if (r.coldExtraDragPct) out.coldExtraDragPct = *r.coldExtraDragPct;
  if (r.calIgnoreRampDeg) out.calIgnoreRampDeg = *r.calIgnoreRampDeg;
  if (r.statMinMoveDeg) out.statMinMoveDeg = *r.statMinMoveDeg;
  if (r.accIgnoreRampDeg) out.accIgnoreRampDeg = *r.accIgnoreRampDeg;
  if (r.dragWarnPct) out.dragWarnPct = *r.dragWarnPct;
  if (r.dragWarnBinsPct) out.dragWarnBinsPct = *r.dragWarnBinsPct;
  if (r.windPeakPct) out.windPeakPct = *r.windPeakPct;
  if (r.windCoherenceMin) out.windCoherenceMin = *r.windCoherenceMin;
  if (r.calStagePwm1) out.calStagePwm1 = *r.calStagePwm1;
  if (r.calStagePwm2) out.calStagePwm2 = *r.calStagePwm2;
  if (r.calStagePwm3) out.calStagePwm3 = *r.calStagePwm3;
  if (r.anemoOffsetKmh) out.anemoOffsetKmh = *r.anemoOffsetKmh;
  if (r.windDirOffsetDeg) out.windDirOffsetDeg = *r.windDirOffsetDeg;
  if (r.antOffset1Deg) out.antOffset1Deg = *r.antOffset1Deg;
  if (r.antOffset2Deg) out.antOffset2Deg = *r.antOffset2Deg;
  if (r.antOffset3Deg) out.antOffset3Deg = *r.antOffset3Deg;
  if (r.angle1Deg) out.angle1Deg = *r.angle1Deg;
  if (r.angle2Deg) out.angle2Deg = *r.angle2Deg;
  if (r.angle3Deg) out.angle3Deg = *r.angle3Deg;
  if (r.pwmMaxAbsNv) out.pwmMaxAbsNv = *r.pwmMaxAbsNv;

  if (r.homeReturnToZero) out.homeReturnToZero = *r.homeReturnToZero ? 1 : 0;
  if (r.stallMonitorEnabled) out.stallMonitorEnabled = *r.stallMonitorEnabled ? 1 : 0;
  if (r.isFilterLen) out.isFilterLen = *r.isFilterLen;
  if (r.dragPersistMoves) out.dragPersistMoves = *r.dragPersistMoves;
  if (r.tempSwapSensors) out.tempSwapSensors = *r.tempSwapSensors ? 1 : 0;
  if (r.windEnable) out.windEnable = *r.windEnable ? 1 : 0;
  if (r.antDp1) out.antDp1 = *r.antDp1 ? 1 : 0;
  if (r.antDp2) out.antDp2 = *r.antDp2 ? 1 : 0;
  if (r.antDp3) out.antDp3 = *r.antDp3 ? 1 : 0;

  out.flags = _amaxExplicit ? 0x01 : 0x00;

  if (r.antName1) copyStr(out.antName1, *r.antName1);
  if (r.antName2) copyStr(out.antName2, *r.antName2);
  if (r.antName3) copyStr(out.antName3, *r.antName3);
}

void RotorStore::unpackConfigToRefs(const RotorPersistConfig& in) {
  RotorStoreRefs& r = _refs;

  if (r.slaveId) *r.slaveId = in.slaveId;
  if (r.rotorType) *r.rotorType = in.rotorType;
  if (r.antSelect) *r.antSelect = in.antSelect;
  if (r.encType) {
    if (in.encTypeU8 == 1) *r.encType = ENCTYPE_MOTOR_AXIS;
    else if (in.encTypeU8 == 3) *r.encType = ENCTYPE_ABSOLUTE_SSI;
    else *r.encType = ENCTYPE_RING_OUTPUT;
  }

  if (r.axisMinDeg01) *r.axisMinDeg01 = in.axisMinDeg01;
  if (r.axisMaxDeg01) *r.axisMaxDeg01 = in.axisMaxDeg01;
  if (r.dgOffsetDeg01) *r.dgOffsetDeg01 = in.dgOffsetDeg01;
  if (r.homePosDeg01) *r.homePosDeg01 = in.homePosDeg01;
  if (r.dgCalDeg01) *r.dgCalDeg01 = in.dgCalDeg01;
  if (r.arriveTolDeg01) *r.arriveTolDeg01 = in.arriveTolDeg01;
  if (r.homeExpectedCountsRing) *r.homeExpectedCountsRing = in.homeExpectedCountsRing;
  if (r.homeExpectedCountsMotor) *r.homeExpectedCountsMotor = in.homeExpectedCountsMotor;

  if (r.homeTimeoutMs) *r.homeTimeoutMs = in.homeTimeoutMs;
  if (r.posTimeoutMs) *r.posTimeoutMs = in.posTimeoutMs;
  if (r.cmdTimeoutMs) *r.cmdTimeoutMs = in.cmdTimeoutMs;
  if (r.jogTimeoutMs) *r.jogTimeoutMs = in.jogTimeoutMs;
  if (r.isSoftWarnMv) *r.isSoftWarnMv = in.isSoftWarnMv;
  if (r.isHardStopMv) *r.isHardStopMv = in.isHardStopMv;
  if (r.stallTimeoutMs) *r.stallTimeoutMs = in.stallTimeoutMs;
  if (r.stallMinCounts) *r.stallMinCounts = in.stallMinCounts;
  if (r.isGraceMs) *r.isGraceMs = in.isGraceMs;
  if (r.isHardHoldMs) *r.isHardHoldMs = in.isHardHoldMs;
  if (r.antDis1) *r.antDis1 = in.antDis1;
  if (r.antDis2) *r.antDis2 = in.antDis2;
  if (r.antDis3) *r.antDis3 = in.antDis3;

  if (r.homeFastPwmPercent) *r.homeFastPwmPercent = in.homeFastPwmPercent;
  if (r.homeBackoff) *r.homeBackoff = in.homeBackoff;
  if (r.homeBacklashMeasScale) *r.homeBacklashMeasScale = in.homeBacklashMeasScale;
  if (r.handSpeedPercent) *r.handSpeedPercent = in.handSpeedPercent;
  if (r.rampDistDeg) *r.rampDistDeg = in.rampDistDeg;
  if (r.minPwm) *r.minPwm = in.minPwm;
  if (r.minStallPwm) *r.minStallPwm = in.minStallPwm;
  if (r.homeSeekMinPwmPercent) *r.homeSeekMinPwmPercent = in.homeSeekMinPwmPercent;
  if (r.tempWarnAmbientC) *r.tempWarnAmbientC = in.tempWarnAmbientC;
  if (r.tempWarnMotorC) *r.tempWarnMotorC = in.tempWarnMotorC;
  if (r.coldTempDegC) *r.coldTempDegC = in.coldTempDegC;
  if (r.coldExtraDragPct) *r.coldExtraDragPct = in.coldExtraDragPct;
  if (r.calIgnoreRampDeg) *r.calIgnoreRampDeg = in.calIgnoreRampDeg;
  if (r.statMinMoveDeg) *r.statMinMoveDeg = in.statMinMoveDeg;
  if (r.accIgnoreRampDeg) *r.accIgnoreRampDeg = in.accIgnoreRampDeg;
  if (r.dragWarnPct) *r.dragWarnPct = in.dragWarnPct;
  if (r.dragWarnBinsPct) *r.dragWarnBinsPct = in.dragWarnBinsPct;
  if (r.windPeakPct) *r.windPeakPct = in.windPeakPct;
  if (r.windCoherenceMin) *r.windCoherenceMin = in.windCoherenceMin;
  if (r.calStagePwm1) *r.calStagePwm1 = in.calStagePwm1;
  if (r.calStagePwm2) *r.calStagePwm2 = in.calStagePwm2;
  if (r.calStagePwm3) *r.calStagePwm3 = in.calStagePwm3;
  if (r.anemoOffsetKmh) *r.anemoOffsetKmh = in.anemoOffsetKmh;
  if (r.windDirOffsetDeg) *r.windDirOffsetDeg = in.windDirOffsetDeg;
  if (r.antOffset1Deg) *r.antOffset1Deg = in.antOffset1Deg;
  if (r.antOffset2Deg) *r.antOffset2Deg = in.antOffset2Deg;
  if (r.antOffset3Deg) *r.antOffset3Deg = in.antOffset3Deg;
  if (r.angle1Deg) *r.angle1Deg = in.angle1Deg;
  if (r.angle2Deg) *r.angle2Deg = in.angle2Deg;
  if (r.angle3Deg) *r.angle3Deg = in.angle3Deg;
  if (r.pwmMaxAbsNv) *r.pwmMaxAbsNv = in.pwmMaxAbsNv;

  if (r.homeReturnToZero) *r.homeReturnToZero = (in.homeReturnToZero != 0);
  if (r.stallMonitorEnabled) *r.stallMonitorEnabled = (in.stallMonitorEnabled != 0);
  if (r.isFilterLen) *r.isFilterLen = in.isFilterLen;
  if (r.dragPersistMoves) *r.dragPersistMoves = in.dragPersistMoves;
  if (r.tempSwapSensors) *r.tempSwapSensors = (in.tempSwapSensors != 0);
  if (r.windEnable) *r.windEnable = (in.windEnable != 0);
  if (r.antDp1) *r.antDp1 = (in.antDp1 != 0);
  if (r.antDp2) *r.antDp2 = (in.antDp2 != 0);
  if (r.antDp3) *r.antDp3 = (in.antDp3 != 0);

  _amaxExplicit = (in.flags & 0x01) != 0;

  if (r.antName1) *r.antName1 = String(in.antName1);
  if (r.antName2) *r.antName2 = String(in.antName2);
  if (r.antName3) *r.antName3 = String(in.antName3);
}

void RotorStore::applyLoadClamps() {
  RotorStoreRefs& r = _refs;
  if (r.rotorType) {
    if (*r.rotorType < 1 || *r.rotorType > 3) *r.rotorType = 1;
  }
  if (r.antSelect) {
    if (*r.antSelect < 1 || *r.antSelect > 3) *r.antSelect = 1;
  }
  if (r.dgCalDeg01) {
    if (*r.dgCalDeg01 < -36000) *r.dgCalDeg01 = -36000;
    if (*r.dgCalDeg01 > 36000) *r.dgCalDeg01 = 36000;
  }
  if (r.homeBacklashMeasScale) {
    if (*r.homeBacklashMeasScale < 0.1f) *r.homeBacklashMeasScale = 0.1f;
    if (*r.homeBacklashMeasScale > 1.0f) *r.homeBacklashMeasScale = 1.0f;
  }
  if (r.jogTimeoutMs && *r.jogTimeoutMs != 0) {
    if (*r.jogTimeoutMs < 100) *r.jogTimeoutMs = 100;
    if (*r.jogTimeoutMs > 5000) *r.jogTimeoutMs = 5000;
  }
  if (r.isFilterLen) {
    if (*r.isFilterLen < 1) *r.isFilterLen = 1;
    if (*r.isFilterLen > 32) *r.isFilterLen = 32;
  }
  if (r.minStallPwm) {
    if (*r.minStallPwm < 0.0f) *r.minStallPwm = 0.0f;
    if (*r.minStallPwm > 100.0f) *r.minStallPwm = 100.0f;
  }
  if (r.minPwm && r.minStallPwm) {
    const float minStallSuggested = *r.minPwm + 2.0f;
    if (*r.minStallPwm < minStallSuggested) *r.minStallPwm = minStallSuggested;
    if (*r.minStallPwm > 100.0f) *r.minStallPwm = 100.0f;
  }
  if (r.homeSeekMinPwmPercent) {
    if (*r.homeSeekMinPwmPercent < 0.0f) *r.homeSeekMinPwmPercent = 0.0f;
    if (*r.homeSeekMinPwmPercent > 100.0f) *r.homeSeekMinPwmPercent = 100.0f;
  }
  if (r.homeExpectedCountsRing && *r.homeExpectedCountsRing <= 0) *r.homeExpectedCountsRing = 158000;
  if (r.homeExpectedCountsMotor && *r.homeExpectedCountsMotor <= 0) *r.homeExpectedCountsMotor = 28000;

  if (r.encType && r.axisMaxDeg01 && r.axisMinDeg01) {
    if (*r.encType == ENCTYPE_ABSOLUTE_SSI && !_amaxExplicit) {
      *r.axisMaxDeg01 = 36000;
    }
    if (*r.axisMaxDeg01 > 72000) *r.axisMaxDeg01 = 72000;
    if (*r.axisMaxDeg01 < *r.axisMinDeg01) *r.axisMaxDeg01 = *r.axisMinDeg01;
  }

  if (r.antDis1 && *r.antDis1 > 99999u) *r.antDis1 = 99999u;
  if (r.antDis2 && *r.antDis2 > 99999u) *r.antDis2 = 99999u;
  if (r.antDis3 && *r.antDis3 > 99999u) *r.antDis3 = 99999u;

  if (r.antName1 && r.antName1->length() > 9) *r.antName1 = r.antName1->substring(0, 9);
  if (r.antName2 && r.antName2->length() > 9) *r.antName2 = r.antName2->substring(0, 9);
  if (r.antName3 && r.antName3->length() > 9) *r.antName3 = r.antName3->substring(0, 9);
}

bool RotorStore::writeDefaultsFromRefs() {
  packConfigFromRefs(_cfgCache);
  memset(&_calCache, 0, sizeof(_calCache));
  memset(&_rtCache, 0, sizeof(_rtCache));
  _rtCache.ssiLastDeg01 = -1;
  return writeAllSections();
}

bool RotorStore::migrateFromPreferences(Preferences& prefs) {
  // Config aus NVS in Refs (Defaults bleiben wenn Key fehlt)
  auto& r = _refs;
  if (r.slaveId) *r.slaveId = prefs.getUChar("id", *r.slaveId);
  if (r.rotorType) *r.rotorType = prefs.getUChar("rty", *r.rotorType);
  if (r.antSelect) *r.antSelect = prefs.getUChar("asel", *r.antSelect);
  if (r.encType) {
    uint8_t ect = prefs.getUChar("ect", 1);
    if (ect == 1) *r.encType = ENCTYPE_MOTOR_AXIS;
    else if (ect == 3) *r.encType = ENCTYPE_ABSOLUTE_SSI;
    else *r.encType = ENCTYPE_RING_OUTPUT;
  }
  if (r.axisMinDeg01) *r.axisMinDeg01 = prefs.getInt("amin", *r.axisMinDeg01);
  if (r.axisMaxDeg01) *r.axisMaxDeg01 = prefs.getInt("amax", *r.axisMaxDeg01);
  _amaxExplicit = prefs.isKey("amax");
  if (r.dgOffsetDeg01) *r.dgOffsetDeg01 = prefs.getInt("dgo", *r.dgOffsetDeg01);
  if (r.homePosDeg01) *r.homePosDeg01 = prefs.getInt("hpos", *r.homePosDeg01);
  if (r.dgCalDeg01) *r.dgCalDeg01 = prefs.getInt("dgcal", *r.dgCalDeg01);
  if (r.homeFastPwmPercent) *r.homeFastPwmPercent = prefs.getFloat("hfp", *r.homeFastPwmPercent);
  if (r.homeBackoff) *r.homeBackoff = prefs.getFloat("hbo", *r.homeBackoff);
  if (r.homeReturnToZero) *r.homeReturnToZero = prefs.getBool("hrz", *r.homeReturnToZero);
  if (r.homeTimeoutMs) *r.homeTimeoutMs = prefs.getUInt("hto", *r.homeTimeoutMs);
  if (r.homeBacklashMeasScale) *r.homeBacklashMeasScale = prefs.getFloat("bms", *r.homeBacklashMeasScale);
  if (r.posTimeoutMs) *r.posTimeoutMs = prefs.getUInt("pto", *r.posTimeoutMs);
  if (r.handSpeedPercent) *r.handSpeedPercent = prefs.getFloat("hsp", *r.handSpeedPercent);
  if (r.cmdTimeoutMs) *r.cmdTimeoutMs = prefs.getUInt("dman", *r.cmdTimeoutMs);
  if (r.jogTimeoutMs) *r.jogTimeoutMs = prefs.getUInt("jogt", *r.jogTimeoutMs);
  if (r.isSoftWarnMv) *r.isSoftWarnMv = prefs.getUInt("iw", *r.isSoftWarnMv);
  if (r.isHardStopMv) *r.isHardStopMv = prefs.getUInt("imax", *r.isHardStopMv);
  if (r.arriveTolDeg01) *r.arriveTolDeg01 = prefs.getInt("atol", *r.arriveTolDeg01);
  if (r.rampDistDeg) *r.rampDistDeg = prefs.getFloat("ramp", *r.rampDistDeg);
  if (r.minPwm) *r.minPwm = prefs.getFloat("minp", *r.minPwm);
  if (r.stallTimeoutMs) *r.stallTimeoutMs = prefs.getUInt("stto", *r.stallTimeoutMs);
  if (r.stallMonitorEnabled) *r.stallMonitorEnabled = prefs.getBool("sten", *r.stallMonitorEnabled);
  if (r.minStallPwm) *r.minStallPwm = prefs.getFloat("msp", *r.minStallPwm);
  if (r.stallMinCounts) *r.stallMinCounts = prefs.getUInt("smc", *r.stallMinCounts);
  if (r.isGraceMs) *r.isGraceMs = prefs.getUInt("igm", *r.isGraceMs);
  if (r.isHardHoldMs) *r.isHardHoldMs = prefs.getUInt("ihm", *r.isHardHoldMs);
  if (r.isFilterLen) *r.isFilterLen = prefs.getUChar("ifl", *r.isFilterLen);
  if (r.homeSeekMinPwmPercent) *r.homeSeekMinPwmPercent = prefs.getFloat("hsm", *r.homeSeekMinPwmPercent);
  if (r.homeExpectedCountsRing) *r.homeExpectedCountsRing = prefs.getInt("ecr", *r.homeExpectedCountsRing);
  if (r.homeExpectedCountsMotor) *r.homeExpectedCountsMotor = prefs.getInt("ecm", *r.homeExpectedCountsMotor);
  if (r.tempWarnAmbientC) *r.tempWarnAmbientC = prefs.getFloat("twa", *r.tempWarnAmbientC);
  if (r.tempWarnMotorC) *r.tempWarnMotorC = prefs.getFloat("twm", *r.tempWarnMotorC);
  if (r.tempSwapSensors) *r.tempSwapSensors = prefs.getBool("tsw", *r.tempSwapSensors);
  if (r.coldTempDegC) *r.coldTempDegC = prefs.getFloat("cth", *r.coldTempDegC);
  if (r.coldExtraDragPct) *r.coldExtraDragPct = prefs.getFloat("cpx", *r.coldExtraDragPct);
  if (r.calIgnoreRampDeg) *r.calIgnoreRampDeg = prefs.getFloat("cig", *r.calIgnoreRampDeg);
  if (r.statMinMoveDeg) *r.statMinMoveDeg = prefs.getFloat("smm", *r.statMinMoveDeg);
  if (r.accIgnoreRampDeg) *r.accIgnoreRampDeg = prefs.getFloat("rap", *r.accIgnoreRampDeg);
  if (r.dragWarnPct) *r.dragWarnPct = prefs.getFloat("drw", *r.dragWarnPct);
  if (r.dragWarnBinsPct) *r.dragWarnBinsPct = prefs.getFloat("drb", *r.dragWarnBinsPct);
  if (r.dragPersistMoves) *r.dragPersistMoves = prefs.getUChar("drn", *r.dragPersistMoves);
  if (r.windPeakPct) *r.windPeakPct = prefs.getFloat("wpk", *r.windPeakPct);
  if (r.windCoherenceMin) *r.windCoherenceMin = prefs.getFloat("wco", *r.windCoherenceMin);
  if (r.calStagePwm1) *r.calStagePwm1 = prefs.getFloat("cpw1", *r.calStagePwm1);
  if (r.calStagePwm2) *r.calStagePwm2 = prefs.getFloat("cpw2", *r.calStagePwm2);
  if (r.calStagePwm3) *r.calStagePwm3 = prefs.getFloat("cpw3", *r.calStagePwm3);
  if (r.anemoOffsetKmh) *r.anemoOffsetKmh = prefs.getFloat("ano", *r.anemoOffsetKmh);
  if (r.windDirOffsetDeg) *r.windDirOffsetDeg = prefs.getFloat("wdo", *r.windDirOffsetDeg);
  if (r.antOffset1Deg) *r.antOffset1Deg = prefs.getFloat("ao1", *r.antOffset1Deg);
  if (r.antOffset2Deg) *r.antOffset2Deg = prefs.getFloat("ao2", *r.antOffset2Deg);
  if (r.antOffset3Deg) *r.antOffset3Deg = prefs.getFloat("ao3", *r.antOffset3Deg);
  if (r.angle1Deg) *r.angle1Deg = prefs.getFloat("ag1", *r.angle1Deg);
  if (r.angle2Deg) *r.angle2Deg = prefs.getFloat("ag2", *r.angle2Deg);
  if (r.angle3Deg) *r.angle3Deg = prefs.getFloat("ag3", *r.angle3Deg);
  if (r.antDp1) *r.antDp1 = prefs.getBool("ad1", *r.antDp1);
  if (r.antDp2) *r.antDp2 = prefs.getBool("ad2", *r.antDp2);
  if (r.antDp3) *r.antDp3 = prefs.getBool("ad3", *r.antDp3);
  if (r.antDis1) *r.antDis1 = prefs.getUInt("di1", *r.antDis1);
  if (r.antDis2) *r.antDis2 = prefs.getUInt("di2", *r.antDis2);
  if (r.antDis3) *r.antDis3 = prefs.getUInt("di3", *r.antDis3);
  if (r.antName1) *r.antName1 = prefs.getString("an1", *r.antName1);
  if (r.antName2) *r.antName2 = prefs.getString("an2", *r.antName2);
  if (r.antName3) *r.antName3 = prefs.getString("an3", *r.antName3);
  if (r.windEnable) *r.windEnable = prefs.getBool("wen", *r.windEnable);
  if (r.pwmMaxAbsNv) *r.pwmMaxAbsNv = prefs.getFloat("maxp", *r.pwmMaxAbsNv);

  applyLoadClamps();
  packConfigFromRefs(_cfgCache);

  // Cal
  memset(&_calCache, 0, sizeof(_calCache));
  if (prefs.getBool("calv", false)) {
    const size_t expectBins = sizeof(_calCache.calCw);
    const size_t expectPwm = sizeof(_calCache.calStagePwm);
    if (prefs.getBytesLength("calcw") == expectBins &&
        prefs.getBytesLength("calcc") == expectBins) {
      prefs.getBytes("calcw", _calCache.calCw, expectBins);
      prefs.getBytes("calcc", _calCache.calCcw, expectBins);
      _calCache.valid = 1;
      if (prefs.getBytesLength("calpw") == expectPwm) {
        prefs.getBytes("calpw", _calCache.calStagePwm, expectPwm);
      } else {
        _calCache.calStagePwm[0] = 45.0f;
        _calCache.calStagePwm[1] = 75.0f;
        _calCache.calStagePwm[2] = 100.0f;
      }
      _calCache.tAmb = prefs.getFloat("ctA", 0.0f);
      _calCache.tMot = prefs.getFloat("ctM", 0.0f);
      _calCache.amaxDeg01 = prefs.getInt("cama", 36000);
    }
  }

  // SSI runtime
  memset(&_rtCache, 0, sizeof(_rtCache));
  struct SsiTurnPersist { uint8_t turn; int32_t sldeg; };
  SsiTurnPersist sst{};
  if (prefs.getBytes("sst", &sst, sizeof(sst)) == sizeof(sst)) {
    _rtCache.ssiTurn = sst.turn ? 1 : 0;
    _rtCache.ssiLastDeg01 = sst.sldeg;
  } else {
    _rtCache.ssiTurn = prefs.getUChar("sturn", 0) ? 1 : 0;
    _rtCache.ssiLastDeg01 = prefs.getInt("sldeg", -1);
  }

  if (!writeAllSections()) return false;

  // NVS leeren — eine Persistenzquelle
  prefs.clear();
  return true;
}

bool RotorStore::begin(TwoWire& wire, const RotorStoreRefs& refs) {
  _refs = refs;
  _fram.begin(wire, 400000);
  if (!_fram.ok()) {
    _ok = false;
    Serial.println("FRAM: probe failed — using RAM defaults");
    return false;
  }

  RotorStoreHeader h{};
  bool framOk = _fram.read(FRAM_OFF_HEADER, reinterpret_cast<uint8_t*>(&h), sizeof(h));
  bool sectionsOk = framOk && readAllSections();
  const uint32_t crc = sectionsOk ? computePayloadCrc() : 0;
  const bool valid = sectionsOk && headerValid(h, crc);

  if (valid) {
    _amaxExplicit = (_cfgCache.flags & 0x01) != 0;
    unpackConfigToRefs(_cfgCache);
    applyLoadClamps();
    (void)readPosSection(); // optional; ungueltig = noch nie gehomed
    _ok = true;
    Serial.println("FRAM: loaded");
    return true;
  }

  // FRAM leer/ungueltig -> Migration oder Defaults
  Preferences prefs;
  const bool prefsOk = prefs.begin("rotor", false);
  if (prefsOk && nvsHasData(prefs)) {
    Serial.println("FRAM: migrating from NVS");
    if (!migrateFromPreferences(prefs)) {
      prefs.end();
      _ok = false;
      Serial.println("FRAM: migration failed");
      return false;
    }
    prefs.end();
    unpackConfigToRefs(_cfgCache);
    applyLoadClamps();
    _ok = true;
    Serial.println("FRAM: migration done");
    return true;
  }
  if (prefsOk) prefs.end();

  Serial.println("FRAM: writing defaults");
  _amaxExplicit = true;
  if (!writeDefaultsFromRefs()) {
    _ok = false;
    Serial.println("FRAM: default write failed");
    return false;
  }
  _ok = true;
  return true;
}

void RotorStore::loadIntoGlobals() {
  if (!_ok) return;
  unpackConfigToRefs(_cfgCache);
  applyLoadClamps();
}

bool RotorStore::saveConfig() {
  if (!_ok) return false;
  _amaxExplicit = true; // nach explizitem SET immer gesetzt
  packConfigFromRefs(_cfgCache);
  return writeAllSections();
}

bool RotorStore::loadCalibration(bool& valid,
                                 uint16_t calCw[CAL_STAGES][LOAD_BINS],
                                 uint16_t calCcw[CAL_STAGES][LOAD_BINS],
                                 float calStagePwm[CAL_STAGES],
                                 float& tAmb, float& tMot, int32_t& amaxDeg01) {
  valid = false;
  if (!_ok) return false;
  if (!_calCache.valid) return true;
  memcpy(calCw, _calCache.calCw, sizeof(_calCache.calCw));
  memcpy(calCcw, _calCache.calCcw, sizeof(_calCache.calCcw));
  memcpy(calStagePwm, _calCache.calStagePwm, sizeof(_calCache.calStagePwm));
  tAmb = _calCache.tAmb;
  tMot = _calCache.tMot;
  amaxDeg01 = _calCache.amaxDeg01;
  valid = true;
  return true;
}

bool RotorStore::saveCalibration(bool valid,
                                 const uint16_t calCw[CAL_STAGES][LOAD_BINS],
                                 const uint16_t calCcw[CAL_STAGES][LOAD_BINS],
                                 const float calStagePwm[CAL_STAGES],
                                 float tAmb, float tMot, int32_t amaxDeg01) {
  if (!_ok) return false;
  memset(&_calCache, 0, sizeof(_calCache));
  _calCache.valid = valid ? 1 : 0;
  if (valid && calCw && calCcw && calStagePwm) {
    memcpy(_calCache.calCw, calCw, sizeof(_calCache.calCw));
    memcpy(_calCache.calCcw, calCcw, sizeof(_calCache.calCcw));
    memcpy(_calCache.calStagePwm, calStagePwm, sizeof(_calCache.calStagePwm));
    _calCache.tAmb = tAmb;
    _calCache.tMot = tMot;
    _calCache.amaxDeg01 = amaxDeg01;
  }
  return writeAllSections();
}

bool RotorStore::clearCalibration() {
  if (!_ok) return false;
  memset(&_calCache, 0, sizeof(_calCache));
  return writeAllSections();
}

bool RotorStore::loadSsiTurn(uint8_t& turn, int32_t& lastDeg01) {
  if (!_ok) return false;
  turn = _rtCache.ssiTurn ? 1 : 0;
  lastDeg01 = _rtCache.ssiLastDeg01;
  return true;
}

bool RotorStore::saveSsiTurn(uint8_t turn, int32_t lastDeg01) {
  if (!_ok) return false;
  _rtCache.ssiTurn = turn ? 1 : 0;
  _rtCache.ssiLastDeg01 = lastDeg01;
  return writeAllSections();
}

bool RotorStore::loadPos(RotorPersistPos& out) const {
  if (!_ok || !_posCacheValid) return false;
  out = _posCache;
  return true;
}

bool RotorStore::savePos(const RotorPersistPos& in) {
  if (!_ok) return false;
  _posCache = in;
  return writePosSection();
}

bool RotorStore::savePosHome(uint8_t encTypeU8, int32_t cpr, int32_t backlashDeg01,
                             int32_t backlashCounts, int32_t encoderCounts, int32_t deg01) {
  if (!_ok) return false;
  memset(&_posCache, 0, sizeof(_posCache));
  _posCache.flags = FRAM_POS_F_HOMVALID; // moveActive klar
  _posCache.encTypeU8 = encTypeU8;
  _posCache.countsPerRev = cpr;
  _posCache.backlashDeg01 = backlashDeg01;
  _posCache.backlashCounts = backlashCounts;
  _posCache.encoderCounts = encoderCounts;
  _posCache.deg01 = deg01;
  return writePosSection();
}

bool RotorStore::savePosSnapshot(int32_t encoderCounts, int32_t deg01) {
  if (!_ok || !_posCacheValid) return false;
  if ((_posCache.flags & FRAM_POS_F_HOMVALID) == 0) return false;
  _posCache.encoderCounts = encoderCounts;
  _posCache.deg01 = deg01;
  // moveActive-Bit bleibt wie gesetzt
  return writePosSection();
}

bool RotorStore::setMoveActive(bool active) {
  if (!_ok || !_posCacheValid) return false;
  if ((_posCache.flags & FRAM_POS_F_HOMVALID) == 0) return false;
  if (active) _posCache.flags = (uint16_t)(_posCache.flags | FRAM_POS_F_MOVEACTIVE);
  else _posCache.flags = (uint16_t)(_posCache.flags & ~FRAM_POS_F_MOVEACTIVE);
  return writePosSection();
}

bool RotorStore::clearHomeValid() {
  if (!_ok) return false;
  memset(&_posCache, 0, sizeof(_posCache));
  return writePosSection(); // schreibt magic/crc mit homeValid=0
}

bool RotorStore::isPosHomeValid() const {
  return _ok && _posCacheValid && ((_posCache.flags & FRAM_POS_F_HOMVALID) != 0);
}

bool RotorStore::isPosMoveActive() const {
  return _ok && _posCacheValid && ((_posCache.flags & FRAM_POS_F_MOVEACTIVE) != 0);
}

bool RotorStore::factoryReset() {
  if (!_fram.ok() && !_fram.probe()) return false;
  // Header invalidieren — Boot schreibt Defaults
  RotorStoreHeader h{};
  memset(&h, 0, sizeof(h));
  if (!_fram.write(FRAM_OFF_HEADER, reinterpret_cast<const uint8_t*>(&h), sizeof(h))) {
    return false;
  }
  // Cal/Config/POS nullen
  memset(&_cfgCache, 0, sizeof(_cfgCache));
  memset(&_calCache, 0, sizeof(_calCache));
  memset(&_rtCache, 0, sizeof(_rtCache));
  memset(&_posCache, 0, sizeof(_posCache));
  (void)_fram.write(FRAM_OFF_POS, reinterpret_cast<const uint8_t*>(&_posCache), sizeof(_posCache));
  _posCacheValid = false;
  _ok = false;
  return true;
}
