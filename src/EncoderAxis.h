#pragma once

#include <Arduino.h>
#include "UltraEncoderPCNT.h"

/*
  EncoderAxis
  ----------
  Kapselt UltraEncoderPCNT (Typ 1/2) oder TWK_KBE58_SSI (Typ 3) und stellt bereit:
  - Raw Counts (Steps) und korrigierte Counts (Steps) (bei Z-Korrektur)
  - Grad-Umrechnung auf Basis countsPerRevActual
    - Typ 1/2: logisch 0..360°
    - Typ 3 (SSI): logisch 0..axisMaxDeg01 (Default 360°, bis 720° mit Turn-Zähler 0/1)
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

  // Logisches Achsmaximum (Deg01). Typ 1/2: typisch 36000.
  // Typ 3: bis 72000 (zwei Umdrehungen via Turn-Zähler 0/1).
  int32_t axisMaxDeg01 = 36000;

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

  void setAxisMaxDeg01(int32_t maxDeg01);
  int32_t getAxisMaxDeg01() const { return _cfg.axisMaxDeg01; }

  void setEncoderType(EncoderType t);
  EncoderType getEncoderType() const { return _cfg.encType; }

  long getCountsRaw() const;
  long getCountsCorrected() const;
  long getCountsDefault() const;

  // SSI: fortlaufende Counts über die Wrap-Grenze (turn * cpr + position)
  long getCountsExtended() const;

  void setCountsZero();
  void setCounts(long newCounts);

  // SSI-Hardware-Zero (SET0-Pin); nur ENCTYPE_ABSOLUTE_SSI
  bool setEncZero();

  // SSI-Turn-Zähler (0/1) für logischen Bereich > 360°
  // Sicherheitskritisch: nach Wrap sofort NVS-persistieren (sonst Reboot → Rohwinkel).
  uint8_t getSsiTurn() const { return _ssiTurn; }
  void setSsiTurn(uint8_t turn);           // Laufzeit / RS485 — markiert Dirty
  void loadSsiTurn(uint8_t turn);          // Boot aus NVS — ohne Dirty
  // Dirty nach kurzer Entprellung (~50 ms) melden — auch während Bewegung
  bool consumeSsiTurnDirty(uint32_t nowMs, bool standstill = true);
  void clearSsiTurnDirty();                // nach erfolgreichem NVS-Write

  bool getPositionDeg01(int32_t& outDeg01) const;
  bool deg01ToCounts(int32_t deg01, int32_t& outCounts) const;
  // SSI: Ziel in fortlaufende Counts (inkl. Turn)
  bool deg01ToCountsExtended(int32_t deg01, int32_t& outCounts) const;

  EncoderZStats getZStats() const;

private:
  bool beginPcnt();
  bool beginSsi();

  void applySsiPosition_(uint32_t newPos);
  void markSsiTurnDirty_();
  int32_t ssiOverlapDeg01_() const;
  int32_t ssiRawDeg01FromCounts_(uint32_t counts) const;

  EncoderAxisConfig _cfg;
  UltraEncoderPCNT* _enc = nullptr;
  class TWK_KBE58_SSI* _ssi = nullptr;
  bool _ssiValid = false;
  uint32_t _ssiPosition = 0;
  uint32_t _ssiLastReadCounter = 0;

  // SSI Mehrumdrehung (0 = 0..360°, 1 = 360..axisMax)
  uint8_t _ssiTurn = 0;
  bool _ssiHavePrevPos = false;
  bool _ssiTurnDirty = false;
  uint32_t _ssiTurnChangedMs = 0;
};
