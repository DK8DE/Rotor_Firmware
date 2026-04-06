#pragma once

#include <Arduino.h>

#include "Rs485Proto.h"

// Vorwaertsdeklarationen, damit wir die Header nicht unnoetig ueberall ziehen.
class HalBoard;
class SafetyMonitor;
class HomingController;
class MotionController;
class Preferences;
class LoadMonitor;
class TempSensors;

// ============================================================================
// Rs485DispatcherConfig
// ============================================================================
// Diese Struktur kapselt alle "externen" Parameter, die der Dispatcher braucht.
// Ziel: Der Dispatcher bleibt in .h/.cpp und die .ino bleibt schlank.
//
// WICHTIG:
// - Viele Werte sind absichtlich als Zeiger implementiert.
//   Damit kann man spaeter (z.B. per EEPROM/RS485) Parameter zur Laufzeit aendern,
//   ohne dass der Dispatcher neue Werte kopieren muss.
// ============================================================================
struct Rs485DispatcherConfig {
  // Eigene Slave-ID (Zeiger auf z.B. g_slaveId)
  uint8_t* ownSlaveId = nullptr;

  // Debug-Schalter (Zeiger auf z.B. g_debug)
  bool* debug = nullptr;

  // Frame-Logging-Schalter (Zeiger auf z.B. g_logRs485Frames)
  bool* logFrames = nullptr;

  // Achsgrenzen (deg01) fuer SETPOSDG-Clamping
  int32_t* axisMinDeg01 = nullptr;
  int32_t* axisMaxDeg01 = nullptr;

  // Bereichs-Offset (Deg01) fuer rechten Endschalter-Versatz (DGOFFSET)
  int32_t* dgOffsetDeg01 = nullptr;

  // Homing-Kick-Retry (optional)
  // - tries/nextMs werden vom Dispatcher gesetzt, die Loop-Logik bleibt in der .ino.
  uint8_t*  homingKickTries = nullptr;
  uint32_t* homingKickNextMs = nullptr;
  uint32_t* homingKickSpacingMs = nullptr;
  uint8_t*  homingKickMaxTries = nullptr;

  // ----------------------------------------------------------
  // Persistente Parameter (werden per RS485 SET/GET gesetzt/gelesen)
  // ----------------------------------------------------------
  Preferences* prefs = nullptr;   // Zeiger auf Preferences-Instanz (muss in setup() begin() haben)

  float*    homeFastPwmPercent = nullptr;
  float*    homeBackoff        = nullptr;
  bool*     homeReturnToZero   = nullptr;
  uint32_t* homeTimeoutMs      = nullptr;

  uint32_t* posTimeoutMs      = nullptr;

  float*    handSpeedPercent   = nullptr;

  uint32_t* cmdTimeoutMs      = nullptr;

  uint32_t* isSoftWarnMv      = nullptr;
  uint32_t* isHardStopMv      = nullptr;

  // Details Stromueberwachung
  uint32_t* isGraceMs         = nullptr;
  uint32_t* isHardHoldMs      = nullptr;
  uint8_t*  isFilterLen       = nullptr;

  int32_t*  arriveTolDeg01    = nullptr;

  float*    rampDistDeg        = nullptr;

  float*    minPwm             = nullptr;

  uint32_t* stallTimeoutMs    = nullptr;

  // Stall-Parameter (Blockade-Erkennung)
  bool*     stallMonitorEnabled = nullptr;
  float*    minStallPwm         = nullptr;
  uint32_t* stallMinCounts      = nullptr;

  // Homing: SEEK-MIN PWM (separat, weil nicht identisch zu Fast/Backoff)
  float*    homeSeekMinPwmPercent = nullptr;

  // Homing/Encoder: Erwartete Counts fuer 360deg (je Encoder-Variante)
  // Hinweis:
  // - Diese Parameter wirken beim aktuellen Projekt erst nach einem Neustart,
  //   weil sie in setup() in die Homing-Rampen-Skalierung einfliessen.
  int32_t*  homeExpectedCountsRing  = nullptr;
  int32_t*  homeExpectedCountsMotor = nullptr;

  // EncoderType (1=MOTOR_AXIS, 2=RING_OUTPUT).
  // Wir nutzen hier absichtlich uint8_t*, damit die .ino ihre Enum intern
  // beibehalten kann (Enum basiert auf uint8_t).
  uint8_t*  encTypeU8 = nullptr;

  // Neustart-Anforderung (z.B. nach Aenderung EncoderType/EncoderCounts).
  // Wird von Rs485Dispatcher gesetzt, der eigentliche ESP.restart() erfolgt in loop().
  bool*     restartRequested = nullptr;
  uint32_t* restartAtMs      = nullptr;

  // PWM-Max: runtime (SETPWM) und persistent (SETMAXPWM)
  float*    pwmMaxAbsRuntime   = nullptr;  // wird von Motion genutzt (Geschwindigkeit)
  float*    pwmMaxAbsStored    = nullptr;  // in Preferences gespeichert (Default nach Neustart)

  // ----------------------------------------------------------
  // Anemometer (Windmesser)
  // ----------------------------------------------------------
  // Offset in km/h (kann negativ/positiv sein), wird in Preferences gespeichert.
  float*    anemoOffsetKmh     = nullptr;

  // ----------------------------------------------------------
  // Windrichtung (RS485 Sensor)
  // ----------------------------------------------------------
  // Global Enable/Disable fuer Wind- & Richtungssensor.
  // - true : Sensor wird gepollt, GETANEMO/GETWINDDIR liefern echte Werte
  // - false: keine Abfragen, GETANEMO/GETWINDDIR liefern immer 0
  bool*     windEnable         = nullptr;

  // Offset in Grad (kann negativ/positiv sein), wird in Preferences gespeichert.
  // Wird in HalBoard direkt in die RS485_Anemometer-Lib durchgereicht.
  float*    windDirOffsetDeg   = nullptr;

  // Antennen-Versatz 1..3 in Grad.
  // Diese Werte werden nur im Rotor persistent gespeichert und per RS485 bereitgestellt.
  // Zulaessiger Bereich: 0,0 .. 360,0 Grad.
  float*    antOffset1Deg      = nullptr;
  float*    antOffset2Deg      = nullptr;
  float*    antOffset3Deg      = nullptr;

  // Frei gespeicherte Winkel 1..3 in Grad.
  // Diese Werte werden nur im Rotor persistent gespeichert und per RS485 bereitgestellt.
  // Zulaessiger Bereich: 0,0 .. 360,0 Grad.
  float*    angle1Deg          = nullptr;
  float*    angle2Deg          = nullptr;
  float*    angle3Deg          = nullptr;

  // ----------------------------------------------------------
  // Temperatur / LoadMonitor / Kalibrierung
  // ----------------------------------------------------------
  float*   tempWarnAmbientC = nullptr;
  float*   tempWarnMotorC   = nullptr;

  // Temperatur-Sensoren: logische Zuordnung tauschen (Device0/Device1)
  // - false: Umgebung=Device0, Motor=Device1
  // - true : Umgebung=Device1, Motor=Device0
  bool*    tempSwapSensors = nullptr;

  float*   calIgnoreRampDeg = nullptr;
  float*   calRampDeg       = nullptr;
  float*   statMinMoveDeg   = nullptr;
  float*   accIgnoreRampDeg = nullptr;

  float*   coldTempDegC       = nullptr;
  float*   coldExtraDragPct   = nullptr;

  float*   dragWarnPct        = nullptr;
  float*   dragWarnBinsPct    = nullptr;
  uint8_t* dragPersistMoves   = nullptr;

  float*   windPeakPct        = nullptr;
  float*   windCoherenceMin   = nullptr;
};

// ============================================================================
// Rs485Dispatcher
// ============================================================================
// Nimmt eingehende RS485-Frames entgegen und verteilt sie auf die Module
// (Safety/Homing/Motion/Load/Temp).
// ==========================================================================
class Rs485Dispatcher {
public:
  Rs485Dispatcher() = default;

  // Muss in setup() einmalig aufgerufen werden.
  void begin(Rs485Proto* proto,
             HalBoard* board,
             SafetyMonitor* safety,
             HomingController* homing,
             MotionController* motion,
             LoadMonitor* loadMon,
             TempSensors* temps,
             const Rs485DispatcherConfig& cfg);

  // Pollt RS485 und dispatcht alle anstehenden Frames.
  void update(uint32_t nowMs);

  // Optional: Homing-Kick-Retry State-Maschine.
  void updateHomingKickRetry(uint32_t nowMs);

  // Verarbeitung eines einzelnen Frames (loggen + dispatchen).
  void onFrame(const Rs485Frame& f, uint32_t nowMs);

  // Fehlermeldung als Broadcast senden (z.B. bei Safety-Fault).
  void sendErrBroadcast(uint8_t errCode);

  // ----------------------------------------------------------
  // Deadman / Keepalive
  // ----------------------------------------------------------
  // Zeitpunkt des letzten gueltigen Kommandos vom Master (jede Anfrage zaehlt als Keepalive).
  // Wird in der .ino fuer die Safety-Timeout-Ueberwachung verwendet.
  uint32_t getLastGetPosCmdMs() const { return _lastGetPosCmdMs; }

private:
  // Kommandos
  void handleCommand(const Rs485Frame& f, uint32_t nowMs);

  // Senden
  void sendAck(uint8_t master, const String& cmd, const String& params);
  void sendNak(uint8_t master, const String& cmd, const String& params);

  // Debug / Logging
  void logRs485FrameToSerial(const Rs485Frame& f);
  void serialEventState(const char* tag);

  // Homing-Kick
  void requestHomingKick(uint32_t nowMs, const char* reason);

  // Parser
  int32_t parseDeg01FromParam(const String& p) const;

  // Formatter
  String formatDeg01ToString(int32_t deg01) const;

private:
  Rs485Proto* _rs485 = nullptr;
  HalBoard* _board = nullptr;
  SafetyMonitor* _safety = nullptr;
  HomingController* _homing = nullptr;
  MotionController* _motion = nullptr;
  LoadMonitor* _loadMon = nullptr;
  TempSensors* _temps = nullptr;
  Rs485DispatcherConfig _cfg;

  // Deadman: letzter gueltiger Befehl vom Master (jedes Kommando zaehlt als Keepalive)
  uint32_t _lastGetPosCmdMs = 0;
};
