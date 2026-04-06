#pragma once

#include <Arduino.h>
#include "UltraEncoderPCNT.h"

/*
  EncoderAxis
  ----------
  Kapselt UltraEncoderPCNT und stellt folgende Dinge bereit:
  - Raw Counts (Steps) und korrigierte Counts (Steps) (bei Z-Korrektur)
  - Grad-Umrechnung (0..360) auf Basis countsPerRevActual
  - Z-Statistik (Pulse count, dz_steps, dz_us, z_error, offset)
  - Konfiguration fuer Ringencoder (OUTPUT) und Motorencoder (MOTOR)
*/

// WICHTIG (RS485-Protokoll):
// - ENCTYPE_MOTOR_AXIS  = 1
// - ENCTYPE_RING_OUTPUT = 2
// Hintergrund:
// - Der EncoderType ist per RS485/EEPROM konfigurierbar.
// - Die Mapping-Werte sind absichtlich NICHT 0/1, damit ein alter/ungesetzter
//   Wert (0) im EEPROM/NVS eindeutig erkannt werden kann.
enum EncoderType : uint8_t {
  ENCTYPE_MOTOR_AXIS  = 1,  // Encoder auf Motorachse
  ENCTYPE_RING_OUTPUT = 2   // Encoder auf Abtrieb/Ring
};

struct EncoderAxisConfig {
  // Pins A/B
  int pinA = -1;
  int pinB = -1;

  // UltraEncoder Mode
  UltraEncoderMode mode = ULTRA_MODE_SINGLE;

  // PCNT / Task
  uint8_t cpuCore = 0;
  uint32_t serviceIntervalUs = 1000;
  uint32_t glitchNs = 200;

  // Z (optional)
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

  // Umrechnung (wird durch Homing gelernt)
  // countsPerRevActual = Counts fuer 360deg
  // Default = 0 -> noch unbekannt
  int32_t countsPerRevActual = 0;

  // Bereichs-Offset (Deg01) fuer mechanische Abweichungen am rechten Endschalter.
  // Beispiel: rechter Endschalter kommt 2,50deg zu spaet -> rangeDegOffsetDeg01 = 250.
  // Wirkung:
  // - countsPerRevActual wird weiterhin aus der Endschalterstrecke gelernt.
  // - Die gemessene Endschalterstrecke kann groesser als 360deg sein (z.B. +2,5deg).
  // - Wir bilden daraus einen logischen Bereich 0..360,00deg, der BEIDSEITIG Abstand
  //   zu den Endschaltern hat (damit "0" nicht in den linken Endschalter faehrt).
  //   Dazu verteilen wir den Offset symmetrisch:
  //     halfOff = rangeDegOffsetDeg01 / 2
  //     total   = 36000 + rangeDegOffsetDeg01
  //     physDeg01 = counts * total / cprActual
  //     logDeg01  = physDeg01 - halfOff
  //     counts    = (logDeg01 + halfOff) * cprActual / total
  // - Die Grad-Skalierung (deg pro Count) bleibt dabei konsistent zur bisherigen
  //   CPR_effektiv-Definition:
  //     cprEff = cprActual * 36000 / (36000 + rangeDegOffsetDeg01)
  // Hinweis:
  // - Der Offset wird NICHT fuer Homing selbst benoetigt (Homing arbeitet in Counts).
  // - Er beeinflusst aber die Grad-Skalierung und damit alle Positionsfahrten.
  int32_t rangeDegOffsetDeg01 = 0;

  // Encoder-Typ
  // Default: Ring/Abtrieb (2)
  EncoderType encType = ENCTYPE_RING_OUTPUT;
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

  // Konfig abfragen/aendern (RAM, spaeter EEPROM/NVS)
  EncoderAxisConfig getConfig() const { return _cfg; }
  void setCountsPerRevActual(int32_t cpr);
  int32_t getCountsPerRevActual() const { return _cfg.countsPerRevActual; }

  // Effektive Counts pro 360deg (nach Range-Offset).
  // 0 wenn unbekannt.
  int32_t getCountsPerRevEffective() const;

  void setRangeDegOffsetDeg01(int32_t offDeg01);
  int32_t getRangeDegOffsetDeg01() const { return _cfg.rangeDegOffsetDeg01; }

  void setEncoderType(EncoderType t);
  EncoderType getEncoderType() const { return _cfg.encType; }

  // Position/Counts
  long getCountsRaw() const;
  long getCountsCorrected() const;
  long getCountsDefault() const;   // je nach "use corrected"

  void setCountsZero();            // setzt Steps=0 (RAW) und reset Z-History
  void setCounts(long newCounts);

  // Grad (0..360) aus Counts
  // liefert false, wenn countsPerRevActual noch unbekannt
  bool getPositionDeg01(int32_t& outDeg01) const;

  // Sollwert in Grad -> Ziel-Counts
  // liefert false, wenn countsPerRevActual unbekannt
  bool deg01ToCounts(int32_t deg01, int32_t& outCounts) const;

  // Z-Stats
  EncoderZStats getZStats() const;

private:
  EncoderAxisConfig _cfg;
  UltraEncoderPCNT* _enc = nullptr;
};
