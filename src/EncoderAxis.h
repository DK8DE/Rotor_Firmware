#pragma once

#include <Arduino.h>
#include "UltraEncoderPCNT.h"

/*
  EncoderAxis
  ----------
  Kapselt UltraEncoderPCNT (Typ 1/2) oder TWK_KBE58_SSI (Typ 3) und stellt bereit:
  - Raw Counts (Steps) und korrigierte Counts (Steps) (bei Z-Korrektur)
  - Grad-Umrechnung (0..360) auf Basis countsPerRevActual
  - Z-Statistik (Pulse count, dz_steps, dz_us, z_error, offset)
  - Konfiguration fuer Ringencoder (OUTPUT), Motorencoder (MOTOR), SSI-Absolut (SSI)
*/

// WICHTIG (RS485-Protokoll):
// - ENCTYPE_MOTOR_AXIS  = 1
// - ENCTYPE_RING_OUTPUT = 2
// - ENCTYPE_ABSOLUTE_SSI = 3
// Hintergrund:
// - Der EncoderType ist per RS485/EEPROM konfigurierbar.
// - Die Mapping-Werte sind absichtlich NICHT 0/1, damit ein alter/ungesetzter
//   Wert (0) im EEPROM/NVS eindeutig erkannt werden kann.
enum EncoderType : uint8_t {
  ENCTYPE_MOTOR_AXIS   = 1,  // Encoder auf Motorachse
  ENCTYPE_RING_OUTPUT  = 2,  // Encoder auf Abtrieb/Ring
  ENCTYPE_ABSOLUTE_SSI = 3   // TWK KBE58 SSI-Absolutencoder (kein A/B/Z)
};

struct EncoderAxisConfig {
  // Pins A/B (nur PCNT Typ 1/2)
  int pinA = -1;
  int pinB = -1;

  // UltraEncoder Mode
  UltraEncoderMode mode = ULTRA_MODE_SINGLE;

  // PCNT / Task
  uint8_t cpuCore = 0;
  uint32_t serviceIntervalUs = 1000;
  uint32_t glitchNs = 200;

  // Z (optional, nur Typ 2)
  bool zEnabled = false;
  uint8_t zPin = 255;
  bool zActiveHigh = true;

  // Z Filter
  uint32_t zMinIntervalUs = 2000;
  long zMinAbsStepsBetween = 50;

  // Z-Korrektur (optional)
  bool zCorrEnabled = false;
  long zExpectedStepsBetweenZ = 500;
  long zMaxAbsErrorSteps = 20;
  float zCorrGain = 1.0f;

  // SSI (nur Typ 3) — Defaults RD130-Verdrahtung
  int ssiClockPin = 8;
  int ssiDataPin = 9;
  int ssiZeroPin = 4;
  uint32_t ssiSpiFreqHz = 100000;
  uint32_t ssiBgIntervalMs = 10;
  uint32_t ssiZeroPulseMs = 200;
  bool ssiInvertDirection = false; // Raw-SSI nicht spiegeln: positive Bewegung muss Position erhoehen

  // SSI: logischer Abtriebswinkel = Encoder-Winkel * num / den.
  // RD130: Encoder-Welle und Abtriebsachse laufen 1:1.
  uint16_t ssiAngleScaleNum = 1;
  uint16_t ssiAngleScaleDen = 1;

  // Umrechnung (wird durch Homing gelernt bei Typ 1/2; bei Typ 3 fest 4096)
  int32_t countsPerRevActual = 0;

  // Bereichs-Offset (Deg01) — nur Typ 1/2 (Endschalter-Versatz)
  int32_t rangeDegOffsetDeg01 = 0;

  EncoderType encType = ENCTYPE_MOTOR_AXIS;
};

struct EncoderZStats {
  bool enabled = false;
  uint32_t zCount = 0;
  long dzSteps = 0;
  uint32_t dzUs = 0;
  long zErrSteps = 0;
  long corrOffsetSteps = 0;
};

class EncoderAxis {
public:
  EncoderAxis();
  ~EncoderAxis();

  bool begin(const EncoderAxisConfig& cfg);
  void stop();

  void update();

  bool isAbsoluteSsi() const { return _cfg.encType == ENCTYPE_ABSOLUTE_SSI; }
  bool isSsiValid() const { return _ssiValid; }

  EncoderAxisConfig getConfig() const { return _cfg; }
  void setCountsPerRevActual(int32_t cpr);
  int32_t getCountsPerRevActual() const { return _cfg.countsPerRevActual; }

  int32_t getCountsPerRevEffective() const;

  void setRangeDegOffsetDeg01(int32_t offDeg01);
  int32_t getRangeDegOffsetDeg01() const { return _cfg.rangeDegOffsetDeg01; }

  void setEncoderType(EncoderType t);
  EncoderType getEncoderType() const { return _cfg.encType; }

  long getCountsRaw() const;
  long getCountsCorrected() const;
  long getCountsDefault() const;

  void setCountsZero();
  void setCounts(long newCounts);

  // SSI-Hardware-Zero (SET0-Pin); nur ENCTYPE_ABSOLUTE_SSI
  bool setEncZero();

  bool getPositionDeg01(int32_t& outDeg01) const;
  bool deg01ToCounts(int32_t deg01, int32_t& outCounts) const;

  EncoderZStats getZStats() const;

private:
  bool beginPcnt();
  bool beginSsi();

  EncoderAxisConfig _cfg;
  UltraEncoderPCNT* _enc = nullptr;
  class TWK_KBE58_SSI* _ssi = nullptr;
  bool _ssiValid = false;
  uint32_t _ssiPosition = 0;
  uint32_t _ssiLastReadCounter = 0;
};
