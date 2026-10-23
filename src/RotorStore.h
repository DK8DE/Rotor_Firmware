#pragma once

#include <Arduino.h>
#include <stddef.h>
#include <Preferences.h>
#include "Fm24cl16b.h"
#include "EncoderAxis.h"
#include "LoadMonitor.h" // CAL_STAGES, LOAD_BINS

// ------------------------------------------------------------
// RotorStore — persistente Konfiguration auf FM24CL16B (16 kbit)
// ------------------------------------------------------------
// Layout (2048 Byte):
//   0x0000 Header (32)
//   0x0020 Config
//   0x0200 Cal
//   0x0600 Runtime (SSI-Turn)
//   0x0700 POS (256) — Typ1/2 Homing-Restore + 50ms-Position (eigener CRC)
//   0x07FF Ende
// CRC32 Header ueber Config+Cal+Runtime (nicht POS).
// ------------------------------------------------------------

static constexpr uint16_t FRAM_OFF_HEADER  = 0x0000;
static constexpr uint16_t FRAM_OFF_CONFIG  = 0x0020;
static constexpr uint16_t FRAM_OFF_CAL     = 0x0200;
static constexpr uint16_t FRAM_OFF_RUNTIME = 0x0600;
static constexpr uint16_t FRAM_OFF_POS     = 0x0700;
static constexpr uint16_t FRAM_SIZE_POS    = 256;
static constexpr uint16_t FRAM_LAYOUT_VER  = 2;
static constexpr uint16_t FRAM_LAYOUT_VER_MIN = 1; // v1 noch lesbar
static constexpr uint32_t FRAM_MAGIC       = 0x52464652u; // 'RTFR'
static constexpr uint32_t FRAM_POS_MAGIC   = 0x31534F50u; // 'POS1'
static constexpr uint16_t FRAM_POS_VER     = 1;

// RotorPersistPos.flags
static constexpr uint8_t FRAM_POS_F_HOMVALID  = 0x01;
static constexpr uint8_t FRAM_POS_F_MOVEACTIVE = 0x02;

// RotorPersistConfig.flags
static constexpr uint8_t FRAM_CFG_F_AMAX_EXPLICIT   = 0x01;
static constexpr uint8_t FRAM_CFG_F_ENDSTOP_COUNT   = 0x02; // endstopCount explizit gespeichert

#pragma pack(push, 1)
struct RotorStoreHeader {
  uint32_t magic;
  uint16_t layoutVersion;
  uint16_t flags;      // bit0: configValid (immer 1 wenn geschrieben)
  uint32_t crc32;      // CRC Config+Cal+Runtime
  uint8_t  reserved[20];
};
static_assert(sizeof(RotorStoreHeader) == 32, "header size");

struct RotorPersistConfig {
  uint8_t  slaveId;
  uint8_t  rotorType;
  uint8_t  antSelect;
  uint8_t  encTypeU8;          // 1/2/3/4

  int32_t  axisMinDeg01;
  int32_t  axisMaxDeg01;
  int32_t  dgOffsetDeg01;
  int32_t  homePosDeg01;
  int32_t  dgCalDeg01;
  int32_t  arriveTolDeg01;
  int32_t  homeExpectedCountsRing;
  int32_t  homeExpectedCountsMotor;

  uint32_t homeTimeoutMs;
  uint32_t posTimeoutMs;
  uint32_t cmdTimeoutMs;
  uint32_t jogTimeoutMs;
  uint32_t isSoftWarnMv;
  uint32_t isHardStopMv;
  uint32_t stallTimeoutMs;
  uint32_t stallMinCounts;
  uint32_t isGraceMs;
  uint32_t isHardHoldMs;
  uint32_t antDis1;
  uint32_t antDis2;
  uint32_t antDis3;

  float    homeFastPwmPercent;
  float    homeBackoff;
  float    homeBacklashMeasScale;
  float    handSpeedPercent;
  float    rampDistDeg;
  float    minPwm;
  float    minStallPwm;
  float    homeSeekMinPwmPercent;
  float    tempWarnAmbientC;
  float    tempWarnMotorC;
  float    coldTempDegC;
  float    coldExtraDragPct;
  float    calIgnoreRampDeg;
  float    statMinMoveDeg;
  float    accIgnoreRampDeg;
  float    dragWarnPct;
  float    dragWarnBinsPct;
  float    windPeakPct;
  float    windCoherenceMin;
  float    calStagePwm1;
  float    calStagePwm2;
  float    calStagePwm3;
  float    anemoOffsetKmh;
  float    windDirOffsetDeg;
  float    antOffset1Deg;
  float    antOffset2Deg;
  float    antOffset3Deg;
  float    angle1Deg;
  float    angle2Deg;
  float    angle3Deg;
  float    pwmMaxAbsNv;

  uint8_t  homeReturnToZero;
  uint8_t  stallMonitorEnabled;
  uint8_t  isFilterLen;
  uint8_t  dragPersistMoves;
  uint8_t  tempSwapSensors;
  uint8_t  windEnable;
  uint8_t  antDp1;
  uint8_t  antDp2;
  uint8_t  antDp3;
  uint8_t  flags;              // bit0: amaxExplicit, bit1: endstopCount explizit
  uint8_t  endstopCount;       // 0=keine, 1=ein Kontakt, 2=zwei Kontakte
  uint8_t  pad[1];

  char     antName1[10];
  char     antName2[10];
  char     antName3[10];
};
static_assert(sizeof(RotorPersistConfig) <= (FRAM_OFF_CAL - FRAM_OFF_CONFIG), "config too big");

struct RotorPersistCal {
  uint8_t  valid;
  uint8_t  reserved[3];
  uint16_t calCw[CAL_STAGES][LOAD_BINS];
  uint16_t calCcw[CAL_STAGES][LOAD_BINS];
  float    calStagePwm[CAL_STAGES];
  float    tAmb;
  float    tMot;
  int32_t  amaxDeg01;
};
static_assert(sizeof(RotorPersistCal) <= (FRAM_OFF_RUNTIME - FRAM_OFF_CAL), "cal too big");

struct RotorPersistRuntime {
  uint8_t  ssiTurn;
  uint8_t  reserved[3];
  int32_t  ssiLastDeg01;
  // CRC deckt nur die ersten 8 Byte ab — Mastheight darf hier wachsen ohne Layout-Bruch.
  float    mastHeightM;    // Masthoehe in m, 0..9999
  uint8_t  pad[244];       // Rest der Runtime-Page bis POS
};
static_assert(sizeof(RotorPersistRuntime) == (FRAM_OFF_POS - FRAM_OFF_RUNTIME), "runtime size");

struct RotorPersistPos {
  uint32_t magic;           // FRAM_POS_MAGIC
  uint16_t version;         // FRAM_POS_VER
  uint16_t flags;           // HOMVALID | MOVEACTIVE
  uint32_t crc32;           // CRC ab encTypeU8 bis Ende payload (ohne magic..crc)
  uint8_t  encTypeU8;       // 1/2
  uint8_t  pad0[3];
  int32_t  countsPerRev;
  int32_t  backlashDeg01;
  int32_t  backlashCounts;
  int32_t  encoderCounts;
  int32_t  deg01;
  uint8_t  reserved[220]; // 256 - 36 Nutzdaten
};
static_assert(sizeof(RotorPersistPos) == FRAM_SIZE_POS, "pos size");
#pragma pack(pop)

// Offsets innerhalb RotorPersistPos fuer CRC (ab encTypeU8)
static constexpr size_t FRAM_POS_CRC_OFF = offsetof(RotorPersistPos, encTypeU8);
static constexpr size_t FRAM_POS_CRC_LEN = sizeof(RotorPersistPos) - FRAM_POS_CRC_OFF;

// Zeiger auf die g_*-Variablen in main.cpp (und Dispatcher-Config)
struct RotorStoreRefs {
  uint8_t*  slaveId = nullptr;
  uint8_t*  rotorType = nullptr;
  uint8_t*  antSelect = nullptr;
  EncoderType* encType = nullptr;

  int32_t*  axisMinDeg01 = nullptr;
  int32_t*  axisMaxDeg01 = nullptr;
  int32_t*  dgOffsetDeg01 = nullptr;
  int32_t*  homePosDeg01 = nullptr;
  int32_t*  dgCalDeg01 = nullptr;
  int32_t*  arriveTolDeg01 = nullptr;
  int32_t*  homeExpectedCountsRing = nullptr;
  int32_t*  homeExpectedCountsMotor = nullptr;

  uint32_t* homeTimeoutMs = nullptr;
  uint32_t* posTimeoutMs = nullptr;
  uint32_t* cmdTimeoutMs = nullptr;
  uint32_t* jogTimeoutMs = nullptr;
  uint32_t* isSoftWarnMv = nullptr;
  uint32_t* isHardStopMv = nullptr;
  uint32_t* stallTimeoutMs = nullptr;
  uint32_t* stallMinCounts = nullptr;
  uint32_t* isGraceMs = nullptr;
  uint32_t* isHardHoldMs = nullptr;
  uint32_t* antDis1 = nullptr;
  uint32_t* antDis2 = nullptr;
  uint32_t* antDis3 = nullptr;

  float*    homeFastPwmPercent = nullptr;
  float*    homeBackoff = nullptr;
  float*    homeBacklashMeasScale = nullptr;
  float*    handSpeedPercent = nullptr;
  float*    rampDistDeg = nullptr;
  float*    minPwm = nullptr;
  float*    minStallPwm = nullptr;
  float*    homeSeekMinPwmPercent = nullptr;
  float*    tempWarnAmbientC = nullptr;
  float*    tempWarnMotorC = nullptr;
  float*    coldTempDegC = nullptr;
  float*    coldExtraDragPct = nullptr;
  float*    calIgnoreRampDeg = nullptr;
  float*    statMinMoveDeg = nullptr;
  float*    accIgnoreRampDeg = nullptr;
  float*    dragWarnPct = nullptr;
  float*    dragWarnBinsPct = nullptr;
  float*    windPeakPct = nullptr;
  float*    windCoherenceMin = nullptr;
  float*    calStagePwm1 = nullptr;
  float*    calStagePwm2 = nullptr;
  float*    calStagePwm3 = nullptr;
  float*    anemoOffsetKmh = nullptr;
  float*    windDirOffsetDeg = nullptr;
  float*    antOffset1Deg = nullptr;
  float*    antOffset2Deg = nullptr;
  float*    antOffset3Deg = nullptr;
  float*    angle1Deg = nullptr;
  float*    angle2Deg = nullptr;
  float*    angle3Deg = nullptr;
  float*    pwmMaxAbsNv = nullptr;
  float*    mastHeightM = nullptr; // Masthoehe m, 0..9999 (FRAM Runtime)

  bool*     homeReturnToZero = nullptr;
  bool*     stallMonitorEnabled = nullptr;
  uint8_t*  isFilterLen = nullptr;
  uint8_t*  dragPersistMoves = nullptr;
  bool*     tempSwapSensors = nullptr;
  bool*     windEnable = nullptr;
  bool*     antDp1 = nullptr;
  bool*     antDp2 = nullptr;
  bool*     antDp3 = nullptr;
  uint8_t*  endstopCount = nullptr; // 0/1/2

  String*   antName1 = nullptr;
  String*   antName2 = nullptr;
  String*   antName3 = nullptr;
};

class RotorStore {
public:
  // refs: Zeiger auf Laufzeit-Globals (muessen gueltig bleiben)
  // Migration: Preferences Namespace "rotor" einmalig lesen, dann clearen.
  bool begin(TwoWire& wire, const RotorStoreRefs& refs);

  bool ok() const { return _ok; }

  // Config aus FRAM in die Refs laden (+ Clamps wie frueher in loadPreferences)
  void loadIntoGlobals();

  // Aktuelle Refs -> FRAM Config (+ CRC neu)
  bool saveConfig();

  // Kalibrierung
  bool loadCalibration(bool& valid,
                       uint16_t calCw[CAL_STAGES][LOAD_BINS],
                       uint16_t calCcw[CAL_STAGES][LOAD_BINS],
                       float calStagePwm[CAL_STAGES],
                       float& tAmb, float& tMot, int32_t& amaxDeg01);
  bool saveCalibration(bool valid,
                       const uint16_t calCw[CAL_STAGES][LOAD_BINS],
                       const uint16_t calCcw[CAL_STAGES][LOAD_BINS],
                       const float calStagePwm[CAL_STAGES],
                       float tAmb, float tMot, int32_t amaxDeg01);
  bool clearCalibration();

  // SSI-Turn Runtime
  bool loadSsiTurn(uint8_t& turn, int32_t& lastDeg01);
  bool saveSsiTurn(uint8_t turn, int32_t lastDeg01);

  // Typ 1/2 Positions-/Homing-Restore (eigener CRC, unabhaengig vom Header-CRC)
  bool loadPos(RotorPersistPos& out) const;
  bool savePos(const RotorPersistPos& in);
  bool savePosHome(uint8_t encTypeU8, int32_t cpr, int32_t backlashDeg01,
                   int32_t backlashCounts, int32_t encoderCounts, int32_t deg01);
  bool savePosSnapshot(int32_t encoderCounts, int32_t deg01);
  bool setMoveActive(bool active);
  bool clearHomeValid();
  bool isPosHomeValid() const;
  bool isPosMoveActive() const;

  // FRAM invalidieren (nach Reboot: Defaults). NVS wird nicht angefasst.
  bool factoryReset();

private:
  Fm24cl16b _fram;
  RotorStoreRefs _refs{};
  bool _ok = false;
  bool _amaxExplicit = true;

  RotorPersistConfig  _cfgCache{};
  RotorPersistCal     _calCache{};
  RotorPersistRuntime _rtCache{};
  RotorPersistPos     _posCache{};
  bool                _posCacheValid = false;

  static uint32_t crc32(const uint8_t* data, size_t len);
  uint32_t computePayloadCrc() const;
  uint32_t computePosCrc(const RotorPersistPos& p) const;
  bool readAllSections();
  bool writeAllSections();
  bool writePosSection();
  bool readPosSection();
  bool headerValid(const RotorStoreHeader& h, uint32_t expectCrc) const;
  bool posValid(const RotorPersistPos& p) const;

  void packConfigFromRefs(RotorPersistConfig& out) const;
  void unpackConfigToRefs(const RotorPersistConfig& in);
  void packRuntimeFromRefs();
  void unpackRuntimeToRefs();
  void applyLoadClamps();

  bool migrateFromPreferences(Preferences& prefs);
  bool writeDefaultsFromRefs();
  static bool nvsHasData(Preferences& prefs);
  static void copyStr(char dst[10], const String& s);
};
