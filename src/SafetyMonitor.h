#pragma once

#include <Arduino.h>
#include "HalBoard.h"

// ------------------------------------------------------------
// Warn-/Error Nummern (hier spaeter erweitern)
// ------------------------------------------------------------

// Warnungen (werden gesammelt)
enum SafetyWarnCode : uint8_t {
  SW_NONE     = 0,
  SW_IS_SOFT  = 1,   // Strom: Soft-Warnung (avg >= Soft-Limit)
  SW_WIND_GUST = 2,  // Windboe / kurzfristig stark erhoeht
  SW_DRAG_INCREASE = 3, // Gleichmaessig mehr Reibung (Getriebe/Kaelte)
  SW_DRAG_DECREASE = 4, // Gleichmaessig weniger Reibung (z.B. Antenne fehlt)
  SW_TEMP_AMBIENT_HIGH = 5, // Umgebung zu warm
  SW_TEMP_MOTOR_HIGH   = 6, // Motor zu warm
};

// Fehler (werden gelatched, Motor stoppt)
enum SafetyErrorCode : uint8_t {
  SE_NONE       = 0,
  SE_TIMEOUT    = 10,  // Motion-Command Timeout (Deadman)
  SE_ENDSTOP    = 11,  // Endschalter blockiert Fahrtrichtung
  SE_NSTOP_CMD  = 12,  // NSTOP per RS485 (Not-Aus Kommando)
  SE_IS_HARD    = 15,  // Strom: Hard-Limit
  SE_STALL      = 16,  // Encoder: keine nennenswerte Bewegung trotz Ansteuerung
  SE_HOME_FAIL  = 17,  // Homing: Segment-Timeout / interner Homing-Fehler
  SE_POS_TIMEOUT = 18, // Positionsfahrt: Ziel nicht innerhalb posTimeoutMs erreicht
};

struct SafetyConfig {
  // Endschalter beachten
  bool restrictEndstops = true;

  // ----------------------------------------------------------
  // Richtungskonvention fuer Endschalter-Logik
  // ----------------------------------------------------------
  // Safety muss wissen, welche PWM-Richtung (Vorzeichen) auf welche
  // mechanische Richtung (links/rechts) wirkt.
  //
  // Standard-Annahme (true):
  //   duty > 0  -> Bewegung Richtung RIGHT-Endschalter (MAX)
  //   duty < 0  -> Bewegung Richtung LEFT-Endschalter  (MIN)
  //
  // Wenn bei dir beim Homing das Backoff aus dem Endschalter mit ERR11
  // (SE_ENDSTOP) blockiert wird, obwohl der Motor physisch aus dem
  // Endschalter heraus fahren soll, dann ist die Richtungskonvention
  // invertiert -> hier false setzen.
  bool dutyPositiveMovesRightEndstop = true;

  // ----------------------------------------------------------
  // Endschalter-Entprellung / Stoerunterdrueckung
  // ----------------------------------------------------------
  // Problem:
  // - Endschalter-Eingaenge koennen beim PWM-Schalten/EMV kurzzeitig "glitchen".
  // - Ohne Entprellung kann das besonders beim Homing (Endschalter gedrueckt,
  //   dann Backoff) zu falschen Endstop-Fehlern fuehren.
  //
  // Loesung:
  // - Rohzustand muss endstopDebounceMs lang stabil sein, bevor er als "aktiv" gilt.
  // - 0 = Entprellung aus (nicht empfohlen).
  //
  // Richtwert:
  // - 20..50 ms ist ein guter Start.
  uint32_t endstopDebounceMs = 30;

  // Wenn true: Endschalter-Verletzung wird Error-latched (Stop + ERR)
  // Wenn false: nur clamp (Duty->0), aber kein Error.
  bool latchErrorOnEndstop = true;

  // ----------------------------------------------------------
  // Endschalter-Ueberfahren (nur fuer Homing-Sonderfaelle)
  // ----------------------------------------------------------
  // Standard: false
  // Wenn true, darf Safety die PWM NICHT auf 0 klemmen, obwohl der Endschalter
  // in der betreffenden Richtung bereits aktiv ist. Das ist z.B. beim Homing
  // sinnvoll, wenn der Schalter bewusst noch ein kleines Stueck ueberfahren
  // werden soll, um sanft abzubremsen.
  //
  // WICHTIG:
  // - Nur sehr kurzzeitig aktivieren!
  // - Nur in den Homing-States, in denen das Absicht ist.
  // - Danach wieder false setzen.
  bool allowPushIntoEndMin = false;
  bool allowPushIntoEndMax = false;

  // Deadman / Timeout (0 = aus)
  uint32_t cmdTimeoutMs = 3000;

  // Stromueberwachung aktiv
  bool currentMonitorEnabled = true;

  // IS Grenzwerte in mV (Mittelwert)
  uint32_t isSoftWarnMv = 900;
  uint32_t isHardStopMv = 1200;

  // Nach Start/Richtungswechsel: Hardlimit ignorieren
  uint32_t isGraceMs = 200;

  // Hardlimit muss so lange anliegen
  uint32_t isHardHoldMs = 60;

  // Filter
  uint8_t  isFilterLen = 16;          // 1..32
  uint32_t isSampleIntervalMs = 5;    // Sampling-Rate


  // IS-Offsets (mV) - werden von den Rohwerten abgezogen (clamp >=0)
  // Damit kannst du einen Grundoffset der IS-Eingaenge kompensieren.
  uint32_t isOffset1Mv = 0;
  uint32_t isOffset2Mv = 0;

  // ----------------------------------------------------------
  // Blockade-/Stall-Erkennung ueber Encoder-Counts
  // Idee: Wenn der Controller Bewegung anfordert (Duty != 0), muss innerhalb
  // eines Zeitfensters eine Mindestanzahl Encoder-Counts eintreffen.
  // Das ist bewusst unabhaengig von der Strommessung, weil eine Blockade
  // beim Losfahren (kleine PWM) einen zu kleinen Strom verursachen kann.
  //
  // stallMonitorEnabled:
  //   true  = Ueberwachung aktiv
  //   false = aus
  // stallTimeoutMs:
  //   Zeitfenster, in dem Bewegung erkennbar sein muss (0 = aus)
  // stallMinCounts:
  //   Mindest-Counts, die innerhalb stallTimeoutMs eintreffen muessen (0 = aus)
  bool     stallMonitorEnabled = true;

  // stallArmDutyAbs:
  //   Ab welcher PWM (absolut, in %) wir ueberhaupt "Bewegung gewollt" annehmen und das Stall-Fenster starten.
  //   Hintergrund: Bei sehr langsamer Rampe / kleiner Anforderung kann es passieren, dass der Motor erst nach
  //   >2s ueberhaupt genug PWM erreicht, um die Haftreibung zu ueberwinden. Wuerden wir das Stall-Timeout schon
  //   ab 2% PWM zaehlen, gibt es faelschlich SE_STALL obwohl das Getriebe nur "zaeh" ist.
  //
  //   Empfehlung:
  //   - typischerweise in der Naehe von pwmKickMinAbs oder etwas darueber (z.B. 10..25%).
  //   - 0 => Default (2%).
  float    stallArmDutyAbs = 10.0f;
  uint32_t stallTimeoutMs = 2000;
  uint32_t stallMinCounts = 10;
};

struct SafetyIsSnapshot {
  // ADC-Rohwerte (mV) VOR Offset-Abzug
  // (Das ist der kalibrierte ADC-Wert, direkt von HalBoard::readIsXmV()).
  uint32_t adc1 = 0;
  uint32_t adc2 = 0;

  // Rohwerte (mV) NACH Offset-Abzug (clamp >= 0)
  // Das sind die Werte, die fuer Filter/Schwellen verwendet werden.
  uint32_t raw1 = 0;
  uint32_t raw2 = 0;

  // Mittelwerte (mV)
  uint32_t avg1 = 0;
  uint32_t avg2 = 0;

  // Peaks seit Start (mV)
  uint32_t peak1 = 0;
  uint32_t peak2 = 0;

  // Flags
  bool softWarn = false;
  bool hardOver = false;
  bool graceActive = false;
};

class SafetyMonitor {
public:
  SafetyMonitor() = default;

  void begin(HalBoard* board, const SafetyConfig& cfg);

  // Konfiguration zur Laufzeit aktualisieren (z.B. nach RS485 SETIWARN/SETIMAX/SETDEADMAN).
  // Fehler/Warnungen bleiben dabei unveraendert (gelatched). Nur die Grenzwerte/Timeouts werden angepasst.
  void updateConfig(const SafetyConfig& cfg);

  // ----------------------------------------------------------
  // Serielle Ereignis-Ausgaben (Arduino IDE)
  // ----------------------------------------------------------
  void setSerialLogging(bool on) { _serialLogEnabled = on; }
  bool getSerialLogging() const { return _serialLogEnabled; }

  // Laufzeit-Settings (spaeter EEPROM/NVS aus INO reinreichen)
  void setRestrictEndstops(bool on);
  bool getRestrictEndstops() const { return _cfg.restrictEndstops; }

  // Endschalter-Ueberfahren gezielt erlauben (Homing-Sonderfall)
  void setAllowPushIntoEndMin(bool on);
  void setAllowPushIntoEndMax(bool on);

  void setLatchErrorOnEndstop(bool on);
  bool getLatchErrorOnEndstop() const { return _cfg.latchErrorOnEndstop; }

  void setCmdTimeoutMs(uint32_t ms);
  uint32_t getCmdTimeoutMs() const { return _cfg.cmdTimeoutMs; }

  void setCurrentMonitorEnabled(bool on);
  bool getCurrentMonitorEnabled() const { return _cfg.currentMonitorEnabled; }

  void setIsSoftWarnMv(uint32_t mv);
  void setIsHardStopMv(uint32_t mv);
  uint32_t getIsSoftWarnMv() const { return _cfg.isSoftWarnMv; }
  uint32_t getIsHardStopMv() const { return _cfg.isHardStopMv; }

  // Vollstaendige Kopie der aktuellen Konfiguration (fuer RS485-SET-Kommandos).
  SafetyConfig getConfig() const { return _cfg; }

  void setIsGraceMs(uint32_t ms);
  void setIsHardHoldMs(uint32_t ms);


  // Blockade-/Stall-Erkennung
  void setStallMonitorEnabled(bool on);
  void setStallTimeoutMs(uint32_t ms);
  void setStallMinCounts(uint32_t c);
  // Quittierung
  void clearFault();        // Fehler quittieren
  void clearWarnings();     // Warnungen loeschen

  // Extern (z.B. LoadMonitor): Warnung setzen (ohne Duplikate)
  void raiseWarning(uint8_t warnCode);

  // ----------------------------------------------------------
  // Externer Not-Aus (z.B. RS485-NSTOP)
  // - Latcht einen Safety-Fehler und zwingt Duty auf 0
  // - Der Motor wird dadurch in der .ino sofort abgeschaltet
  // ----------------------------------------------------------
  void triggerEmergencyStop(uint8_t errCode);

  // Fuer Richtungswechsel/Start: Grace-Fenster neu setzen
  void notifyMotionEdge(uint32_t nowMs);

  // Update zyklisch aufrufen
  // nowMs: millis()
  // desiredDutySigned: geplanter Duty (-100..+100)
  // rsMotionActive: true => Deadman aktiv (nur wenn Controller zyklisch senden muss)
  // lastMotionCmdMs: Zeitpunkt des letzten gueltigen Kommandos vom Master (Keepalive)
  //
  // Rueckgabe: erlaubter Duty (kann 0 sein)
  float update(uint32_t nowMs,
               float desiredDutySigned,
               bool rsMotionActive,
               uint32_t lastMotionCmdMs,
               long encoderCounts);

  // Status
  bool isFault() const { return _faultActive; }
  uint8_t getErrorCode() const { return _errorCode; }

  // Warnungen (max 8 Codes)
  uint8_t getWarnCount() const { return _warnCount; }
  uint8_t getWarnAt(uint8_t idx) const;

  // IS Snapshot
  SafetyIsSnapshot getIsSnapshot() const { return _isSnap; }

  // ----------------------------------------------------------
  // Letztes von Safety freigegebenes Duty-Vorzeichen (Richtung)
  // ----------------------------------------------------------
  // Wird u.a. fuer RS485 GETIS genutzt, um automatisch den passenden IS-Kanal
  // (IS1 bei positiver, IS2 bei negativer Bewegung) auszugeben.
  //
  // Rueckgabe:
  //  +1 = positive Richtung
  //  -1 = negative Richtung
  //   0 = aktuell keine PWM (Stillstand / geklemmt)
  int8_t getLastOutputDutySign() const { return _lastOutputDutySign; }

  // Wie oben, aber wenn aktuell 0, dann der letzte Nicht-Null-Wert.
  // Das ist praktisch, wenn der Regler kurzzeitig PWM=0 setzt (z.B. in der Feinphase),
  // man aber trotzdem den "aktiven" IS-Kanal passend zur letzten Bewegungsrichtung sehen will.
  int8_t getLastOutputDutySignNonZero() const { return _lastOutputDutySignNonZero; }

  // Peaks zuruecksetzen
  void resetPeaks();

private:
  void updateIsSampling(uint32_t nowMs);
  void updateIsProtection(uint32_t nowMs);

  // Endschalter entprellen (Roh -> stabil)
  void updateEndstopDebounce(uint32_t nowMs);

  void updateStallProtection(uint32_t nowMs, float desiredDutySigned, long encoderCounts);
  void addWarn(uint8_t code);
  void latchError(uint8_t errCode);

  // Serial-Helfer
  void logWarn(uint8_t code, uint32_t nowMs);
  void logError(uint8_t errCode, uint32_t nowMs);
  const char* warnToText(uint8_t code) const;
  const char* errToText(uint8_t code) const;

private:
  HalBoard* _board = nullptr;
  SafetyConfig _cfg;

  // Serial logging
  bool _serialLogEnabled = false;

  // Fault/Error
  bool _faultActive = false;
  uint8_t _errorCode = SE_NONE;

  // Warns (persistent)
  uint8_t _warns[8] = {0};
  uint8_t _warnCount = 0;

  // ----------------------------------------------------------
  // Endschalter-Entprellung
  // ----------------------------------------------------------
  bool _endLRaw = false;
  bool _endRRaw = false;
  bool _endLStable = false;
  bool _endRStable = false;
  uint32_t _endLChangeMs = 0;
  uint32_t _endRChangeMs = 0;

  // IS Filter Ringbuffer (max 32)
  uint32_t _buf1[32] = {0};
  uint32_t _buf2[32] = {0};
  uint8_t  _idx = 0;
  uint32_t _sum1 = 0;
  uint32_t _sum2 = 0;

  // Timing
  uint32_t _lastIsSampleMs = 0;
  uint32_t _graceUntilMs = 0;
  uint32_t _hardOverSinceMs = 0;

  // Joerg: MotionController-Bremssequenz aktiv (STOP / Richtungswechsel).
  // Wenn true, wird die Encoder-Stall-Erkennung ausgesetzt.


  // ----------------------------------------------------------
  // Letztes von Safety freigegebenes Duty (Ausgabe)
  // ----------------------------------------------------------
  // Diese Werte werden in update() aktualisiert und werden u.a. von RS485 GETIS genutzt.
  float  _lastOutputDutySigned = 0.0f;
  int8_t _lastOutputDutySign = 0;
  int8_t _lastOutputDutySignNonZero = 0;


  // Stall-/Blockade-Erkennung
  bool _stallPrevMoveWanted = false;  // Flanke: wollen wir Bewegung (Duty ueber Schwelle)?
  int8_t _stallPrevSign = 0;          // Vorzeichen der letzten "Bewegung gewollt" PWM (+1 / -1)
  bool _stallArmed = false;           // Timer laeuft
  uint32_t _stallStartMs = 0;         // Startzeit
  long _stallStartCounts = 0;         // Counts-Basis

  // "Micro"-Stall: Fuer extrem kleine Schritte (z.B. 0,01deg) kann es sein, dass wir
  // absichtlich NICHT die normale Stall-Schwelle (stallArmDutyAbs) erreichen.
  // Trotzdem wollen wir bei echter Blockade irgendwann sicher ausloesen.
  // -> Wir sammeln daher "keine Bewegung" Zeit, sobald eine Mindest-PWM anliegt,
  //    und loesen erst mit deutlich laengerer Zeit aus.
  bool _stallMicroArmed = false;
  uint32_t _stallMicroStartMs = 0;
  long _stallMicroStartCounts = 0;

  // Snapshot
  SafetyIsSnapshot _isSnap;
};
