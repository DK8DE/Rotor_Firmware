#pragma once

#include <Arduino.h>
#include "HalBoard.h"
#include "MotorMcpwm.h"
#include "EncoderAxis.h"

/*
  HomingController - 2-Leg + Backoff + Slow-Approach + optional Return-to-Zero

  Endschalter-Interpretation:
  - HomingController nutzt NUR HalBoard::readEndLeft()/readEndRight()
  - Diese Funktionen muessen "true = gedrueckt" liefern (egal ob elektrisch LOW oder HIGH)

  Sequenz (immer):
  1) LEFT FAST         -> bis END_MIN (debounced)
  2) BACKOFF MIN FAST  -> bis END_MIN frei
  3) LEFT SLOW (Touch1)-> bis END_MIN
  4) BACKOFF MIN SLOW  -> bis END_MIN frei (Release merken)
  5) LEFT SLOW (Touch2)-> bis END_MIN -> setCountsZero() + backlashCounts messen
      (Counts-Strecke frei->wieder MIN; Deg01-Umrechnung erst in Schritt 10 mit
       backlashMeasuredToModelScale, typ. 0.5, weil die Messstrecke ~2x einseitiges Spiel enthaelt)

  6) RIGHT FAST        -> bis END_MAX
  7) BACKOFF MAX FAST  -> bis END_MAX frei
  8) RIGHT SLOW (Touch1)-> bis END_MAX
  9) BACKOFF MAX SLOW  -> bis END_MAX frei
 10) RIGHT SLOW (Touch2)-> bis END_MAX -> CPR lernen
 11) BACKOFF MAX FINAL -> bis END_MAX frei

  Optional (returnToZero=true):
 12) RETURN LEFT FAST       -> bis END_MIN
 13) BACKOFF MIN RT FAST    -> bis END_MIN frei
 14) RETURN LEFT SLOW (Touch1)-> bis END_MIN
 15) BACKOFF MIN RT SLOW    -> bis END_MIN frei
 16) RETURN LEFT SLOW (Touch2)-> bis END_MIN -> setCountsZero()
 17) BACKOFF MIN FINAL      -> bis END_MIN frei
-> bis END_MIN frei
  10) DONE
*/

enum HomingState : uint8_t {
  HOME_IDLE = 0,

  // MIN / LEFT
  HOME_SEEK_MIN_FAST,       // schnell anfahren bis END_MIN gedrueckt (debounced)
  // Optionales "Ueberfahren" in den Endschalter hinein (falls mechanisch erlaubt),
  // um nach dem Ausloesen weich abzubremsen.
  HOME_SEEK_MIN_OVERRUN,
  HOME_BACKOFF_MIN_FAST,    // freifahren bis END_MIN frei (debounced)
  HOME_SEEK_MIN_SLOW_1,     // langsam anfahren bis END_MIN gedrueckt (1. Touch)
  HOME_BACKOFF_MIN_SLOW,    // langsam freifahren bis END_MIN frei (Release fuer Backlash)
  HOME_SEEK_MIN_SLOW_2,     // langsam anfahren bis END_MIN gedrueckt (2. Touch) -> Nullpunkt setzen

  // MAX / RIGHT
  HOME_SEEK_MAX_FAST,       // schnell anfahren bis END_MAX gedrueckt
  HOME_BACKOFF_MAX_FAST,    // freifahren bis END_MAX frei
  HOME_SEEK_MAX_SLOW_1,     // langsam anfahren bis END_MAX gedrueckt (1. Touch)
  HOME_BACKOFF_MAX_SLOW,    // langsam freifahren bis END_MAX frei
  HOME_SEEK_MAX_SLOW_2,     // langsam anfahren bis END_MAX gedrueckt (2. Touch) -> CPR lernen
  HOME_BACKOFF_MAX_FINAL,   // sicher freifahren, damit END_MAX am Ende nicht gedrueckt ist

  // OPTIONAL: Return-to-Zero (links) fuer definierte Endposition am Ende
  HOME_RETURN_MIN_FAST,     // schnell nach links bis END_MIN gedrueckt
  HOME_BACKOFF_MIN_RT_FAST, // freifahren bis END_MIN frei
  HOME_RETURN_MIN_SLOW_1,   // langsam anfahren (1. Touch)
  HOME_BACKOFF_MIN_RT_SLOW, // langsam freifahren (Release)
  HOME_RETURN_MIN_SLOW_2,   // langsam anfahren (2. Touch) -> Nullpunkt setzen
  HOME_BACKOFF_MIN_FINAL,   // final freifahren bis END_MIN frei, dann DONE

  // Optional: Nach dem finalen Freifahren noch auf die logische 0-Position
  // (Offset/2 weg vom Endschalter) fahren.
  // Hintergrund: Durch das symmetrische Range-Offset kann die Position nach dem
  // Freifahren noch in der Clamping-Zone liegen (logisch 0), obwohl wir
  // mechanisch noch sehr nah am Endschalter stehen.
  // Dieser Schritt bringt uns reproduzierbar auf logisches 0, ohne erneut
  // in den Endschalter zu fahren.
  HOME_GOTO_LOGICAL_ZERO,

  HOME_DONE,
  HOME_ERROR
};

struct HomingConfig {
  float fastPwmPercent = 60.0f;       // schnelle Fahrt
  float approachPwmPercent = 20.0f;   // langsames Anschleichen
  float backoffPwmPercent = 20.0f;    // Freifahren vom Endschalter

  // ------------------------------------------------------------
  // Ramping (Homing)
  // ------------------------------------------------------------
  // SEEK_MIN_FAST: Wir kennen die Start-Position nicht. Daher:
  // - von approachPwmPercent (typ. g_minPwm) auf seekMinPwmPercent rampen
  // - danach konstant mit seekMinPwmPercent bis END_MIN
  float seekMinPwmPercent = 60.0f;   // konstante PWM nach der Anfahr-Rampe nach links
  int32_t seekMinRampCounts = 5000;  // Rampenlaenge in Encoder-Counts (ab Start SEEK_MIN_FAST)

  // Optional: Nach dem Ausloesen von END_MIN noch ein kleines Stueck "weiter druecken",
  // dabei die PWM weich auf approachPwmPercent herunter rampen.
  // Hintergrund: Viele Endschalter lassen sich ein Stueck ueberfahren, bevor ein harter Anschlag kommt.
  // Mit dieser Rampe wird der Stoß am Ende deutlich sanfter.
  // 0 = deaktiviert (sofort stoppen und backoff).
  int32_t seekMinOverrunCounts = 0;

  // SEEK_MAX_FAST: Wir erwarten ungefaehr einen kompletten Umdrehungsweg.
  // - Beschleunigen ueber seekMaxAccelRampCounts auf fastPwmPercent
  // - ab seekMaxDecelStartCounts wieder auf approachPwmPercent herunter rampen
  int32_t expectedCountsPerRevHint = 160000; // Erwartungswert fuer 360deg (zur Rampen-Skalierung)
  int32_t seekMaxAccelRampCounts = 13300;    // Rampenlaenge fuer Beschleunigung in Counts
  int32_t seekMaxDecelStartCounts = 150000;  // ab hier Abbremsrampe Richtung approachPwmPercent
  int32_t seekMaxDecelRampCounts = 10000;    // Rampenlaenge fuer Abbremsen (Counts)

  // RETURN_MIN_FAST (Rueckweg auf 0 / linker Endschalter): Rampen in Grad.
  // - zu Beginn ueber returnRampDeg auf fastPwmPercent beschleunigen
  // - returnRampDeg vor Ziel (Counts==0) wieder auf approachPwmPercent abbremsen
  float returnRampDeg = 30.0f;

  bool returnToZero = true;          // optional wieder zurueck auf 0

  // Skalierung MIN-Backlash-Messstrecke (END_MIN frei -> wieder gedrueckt) auf
  // das einseitige b fuer MotionController (typ. 0.5, NVS bms). Ohne Skalierung
  // wirkt Umkehrspiel bei SETPOS oft zu gross (Messung: zwei Umkehrungen + Schalter).
  float backlashMeasuredToModelScale = 0.5f;

  // Nach dem finalen Freifahren (END_MIN frei) noch auf logische 0 fahren.
  // 0 = aus (wie bisher)
  bool gotoLogicalZeroAfterHoming = true;

  uint32_t segmentTimeoutMs = 30000; // Timeout pro Segment
};

class HomingController {
public:
  HomingController() = default;

  void begin(HalBoard* board, MotorMcpwm* motor, EncoderAxis* enc, const HomingConfig& cfg);

  // Konfiguration zur Laufzeit aktualisieren (wird z.B. nach RS485 SETHOMEPWM genutzt).
  // Wirkt sofort fuer naechste Schritte/naechsten Homing-Run.
  void updateConfig(const HomingConfig& cfg);

  // Aktuelle Konfiguration (Kopie).
  // Wird im Rs485Dispatcher genutzt, um bei partiellen Updates keine
  // nicht per RS485 einstellbaren Parameter zu verlieren.
  HomingConfig getConfig() const { return _cfg; }

  void start();               // SETREF
  void abort();               // STOP/NSTOP
  void update(uint32_t nowMs);

  bool isActive() const;
  bool isReferenced() const { return _referenced; }
  HomingState getState() const { return _state; }

  int32_t getCountsPerRevLearned() const { return _countsPerRevLearned; }
  int32_t getBacklashDeg01() const { return _backlashDeg01; }
  // Debug/Diagnose: gemessene Backlash-Counts (Encoder-Counts) zwischen "frei" und "wieder gedrueckt".
  // Sinnvoll nur bei ENCTYPE_MOTOR_AXIS (Encoder auf Motorachse).
  long getBacklashCounts() const { return _backlashCounts; }
  const char* getLastErrorText() const { return _lastErr; }

private:
  // Endstop debounce (mechanische Entprellung).
  // Wir arbeiten mit stabilen Zustaenden, damit der "Schalt-Count" reproduzierbar wird.
  void updateEndstopDebounce(uint32_t nowMs);
  bool endL() const { return _endLStable; }
  bool endR() const { return _endRStable; }

  void setState(HomingState s, uint32_t nowMs);
  void fail(const char* msg, uint32_t nowMs);

  void motorStop();
  void motorDrive(float dutySigned);

private:
  HalBoard* _board = nullptr;
  MotorMcpwm* _motor = nullptr;
  EncoderAxis* _enc = nullptr;

  HomingConfig _cfg;
  // Endstop debounce: raw und stabile Zustaende
  bool _endLRaw = false;
  bool _endRRaw = false;
  bool _endLStable = false;
  bool _endRStable = false;
  uint32_t _endLChangedMs = 0;
  uint32_t _endRChangedMs = 0;


  HomingState _state = HOME_IDLE;
  uint32_t _stateStartMs = 0;
  // Encoder-Counts beim Eintritt in den aktuellen Zustand.
  // Wird u.a. fuer rampenbasierte Fahrprofile (Counts/Grad) verwendet.
  long _stateStartCounts = 0;

  // Zuletzt gesetztes PWM-Soll (signed, -100..+100).
  // Wird fuer sanfte Ueberlauf-Rampen am Endschalter genutzt.
  float _lastDutyCmdSigned = 0.0f;

  // Merker: Start-PWM (Betrag) beim Eintritt in HOME_SEEK_MIN_OVERRUN.
  float _overrunStartAbsPwm = 0.0f;

  // ------------------------------------------------------------
  // Optional: Ziel-Counts fuer HOME_GOTO_LOGICAL_ZERO.
  // Wird beim Eintritt in den State berechnet.
  // ------------------------------------------------------------
  long _gotoLogicalZeroTargetCounts = 0;
  bool _gotoLogicalZeroValid = false;

  bool _referenced = false;
  int32_t _countsPerRevLearned = 0;

  // ------------------------------------------------------------
  // Umkehrspiel / Backlash (wird waehrend des Homings ermittelt)
  // ------------------------------------------------------------
  // Merker, ob wir einen gueltigen Startpunkt fuer die Messung haben.
  // Der Startpunkt wird gesetzt, sobald END_MIN beim BACKOFF MIN2 frei wird.
  bool _haveBacklashBase = false;

  // Encoder-Counts an der Stelle, an der END_MIN "frei" geworden ist.
  long _backlashReleaseCounts = 0;

  // Gemessene Counts-Strecke bis END_MIN beim langsamen Wieder-Anfahren
  // wieder gedrueckt wird.
  long _backlashCounts = 0;

  // Umkehrspiel in 0,01deg (Deg01). Wird nach CPR-Lernen aus Counts berechnet.
  int32_t _backlashDeg01 = 0;

  char _lastErr[48] = {0};
};
