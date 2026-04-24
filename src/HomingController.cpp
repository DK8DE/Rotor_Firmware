#include "HomingController.h"

#include <math.h>
#include <string.h>

static float clampFloat(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static float lerpFloat(float a, float b, float t) {
  return a + (b - a) * t;
}

void HomingController::begin(HalBoard* board, MotorMcpwm* motor, EncoderAxis* enc, const HomingConfig& cfg) {
  _board = board;
  _motor = motor;
  _enc = enc;
  _cfg = cfg;

  _state = HOME_IDLE;
  _stateStartMs = 0;

  _referenced = false;
  _countsPerRevLearned = 0;

  _haveBacklashBase = false;
  _backlashReleaseCounts = 0;
  _backlashCounts = 0;
  _backlashDeg01 = 0;

  _lastErr[0] = 0;
}

void HomingController::updateConfig(const HomingConfig& cfg) {
  _cfg = cfg;
}


void HomingController::start() {
  if (!_board || !_motor || !_enc) return;

  _referenced = false;
  _countsPerRevLearned = 0;

  _haveBacklashBase = false;
  _backlashReleaseCounts = 0;
  _backlashCounts = 0;
  _backlashDeg01 = 0;

  _lastErr[0] = 0;

  const uint32_t nowMs = millis();

  // Endstop-Debounce initialisieren (Startwerte als stabil annehmen)
  _endLRaw = _board->readEndLeft();
  _endRRaw = _board->readEndRight();
  _endLStable = _endLRaw;
  _endRStable = _endRRaw;
  _endLChangedMs = nowMs;
  _endRChangedMs = nowMs;

  setState(HOME_SEEK_MIN_FAST, nowMs);
}

void HomingController::abort() {
  motorStop();
  setState(HOME_IDLE, millis());
}

bool HomingController::isActive() const {
  return (_state != HOME_IDLE && _state != HOME_DONE && _state != HOME_ERROR);
}

void HomingController::setState(HomingState s, uint32_t nowMs) {
  _state = s;
  _stateStartMs = nowMs;

  // Start-Counts fuer Rampen-Profile merken.
  // Achtung: Encoder kann bei begin()/start() noch 0 liefern; das ist ok.
  _stateStartCounts = (_enc) ? _enc->getCountsDefault() : 0;

  // Zusatz-Merker fuer spezielle States
  if (s == HOME_SEEK_MIN_OVERRUN) {
    // Start-PWM (Betrag) beim Eintritt merken, damit die Rampe sauber
    // vom aktuell gefahrenen Wert auf approachPwmPercent herunter laufen kann.
    float a = _lastDutyCmdSigned;
    if (a < 0.0f) a = -a;
    _overrunStartAbsPwm = a;
    if (_overrunStartAbsPwm < 0.0f) _overrunStartAbsPwm = 0.0f;
    if (_overrunStartAbsPwm > 100.0f) _overrunStartAbsPwm = 100.0f;
  }

  if (s == HOME_GOTO_LOGICAL_ZERO) {
    // Ziel-Counts fuer logisches 0 (Offset/2 weg vom linken Endschalter) berechnen.
    // Wichtig: Wir nutzen absichtlich die Deg01->Counts-Umrechnung der EncoderAxis,
    // damit Range-Offset/CPR-Lernen konsistent bleiben.
    _gotoLogicalZeroTargetCounts = 0;
    _gotoLogicalZeroValid = false;

    if (_enc) {
      int32_t tarCounts = 0;
      if (_enc->deg01ToCounts(0, tarCounts)) {
        // Ziel muss im positiven Bereich liegen (wir sind nach finalem Backoff bereits frei).
        if (tarCounts > 0) {
          _gotoLogicalZeroTargetCounts = (long)tarCounts;
          _gotoLogicalZeroValid = true;
        }
      }
    }
  }
}

void HomingController::fail(const char* msg, uint32_t nowMs) {
  strncpy(_lastErr, msg, sizeof(_lastErr) - 1);
  _lastErr[sizeof(_lastErr) - 1] = 0;
  setState(HOME_ERROR, nowMs);
  motorStop();
}

void HomingController::motorStop() {
  if (_motor) {
    _motor->stopPwm();
  }
  _lastDutyCmdSigned = 0.0f;
}

void HomingController::motorDrive(float dutySigned) {
  if (!_motor) return;

  float d = clampFloat(dutySigned, -100.0f, +100.0f);

  if (fabsf(d) < 0.01f) {
    _motor->stopPwm();
    _lastDutyCmdSigned = 0.0f;
    return;
  }

  _motor->enable(true);
  _motor->setDutySigned(d);
  _lastDutyCmdSigned = d;
}


void HomingController::updateEndstopDebounce(uint32_t nowMs) {
  if (!_board) return;

  // Diese Entprellung sorgt dafuer, dass mechanisches Prellen keinen "falschen"
  // Schalt-Count erzeugt. Erst wenn der Rohzustand fuer >= debounceMs stabil ist,
  // wird der stabile Zustand umgeschaltet.
  const uint32_t debounceMs = 15;

  const bool rawL = _board->readEndLeft();
  if (rawL != _endLRaw) {
    _endLRaw = rawL;
    _endLChangedMs = nowMs;
  }
  if ((nowMs - _endLChangedMs) >= debounceMs) {
    _endLStable = _endLRaw;
  }

  const bool rawR = _board->readEndRight();
  if (rawR != _endRRaw) {
    _endRRaw = rawR;
    _endRChangedMs = nowMs;
  }
  if ((nowMs - _endRChangedMs) >= debounceMs) {
    _endRStable = _endRRaw;
  }
}

void HomingController::update(uint32_t nowMs) {
  if (!_board || !_motor || !_enc) return;

  // ------------------------------------------------------------
  // WICHTIG:
  // Nach HOME_DONE darf Homing den Motor NICHT mehr dauerhaft stoppen,
  // sonst zerhackt Homing die PWM der Positionsfahrt (stopPwm in jeder Loop).
  // In HOME_ERROR soll der Motor dagegen sicher aus bleiben.
  // ------------------------------------------------------------
  if (_state == HOME_DONE) {
    return;
  }
  if (_state == HOME_ERROR) {
    motorStop();
    return;
  }

  if (_state == HOME_IDLE) {
    return;
  }

  // Timeout pro Segment
  if ((nowMs - _stateStartMs) > _cfg.segmentTimeoutMs) {
    fail("HOME_TIMEOUT", nowMs);
    return;
  }

  const float pwmFast = clampFloat(_cfg.fastPwmPercent, 0.0f, 100.0f);
  const float pwmSlow = clampFloat(_cfg.approachPwmPercent, 0.0f, 100.0f);
  const float pwmBack = clampFloat(_cfg.backoffPwmPercent, 0.0f, 100.0f);

  // Ramping-Parameter
  const float pwmSeekMin = clampFloat(_cfg.seekMinPwmPercent, 0.0f, 100.0f);
  long rampSeekMinCounts = (long)_cfg.seekMinRampCounts;
  if (rampSeekMinCounts < 1) rampSeekMinCounts = 1;
  long seekMinOverrunCounts = (long)_cfg.seekMinOverrunCounts;
  if (seekMinOverrunCounts < 0) seekMinOverrunCounts = 0;

  long expectedCounts = (long)_cfg.expectedCountsPerRevHint;
  if (expectedCounts < 1) expectedCounts = 1;
  long rampSeekMaxAccel = (long)_cfg.seekMaxAccelRampCounts;
  if (rampSeekMaxAccel < 1) rampSeekMaxAccel = 1;
  long rampSeekMaxDecel = (long)_cfg.seekMaxDecelRampCounts;
  if (rampSeekMaxDecel < 1) rampSeekMaxDecel = 1;
  long seekMaxDecelStart = (long)_cfg.seekMaxDecelStartCounts;
  if (seekMaxDecelStart < 0) seekMaxDecelStart = 0;
  if (seekMaxDecelStart > expectedCounts) seekMaxDecelStart = expectedCounts;

  float returnRampDeg = _cfg.returnRampDeg;
  if (returnRampDeg < 0.0f) returnRampDeg = 0.0f;

  // ------------------------------------------------------------
  // Endschalter (debounced) aktualisieren
  // ------------------------------------------------------------
  updateEndstopDebounce(nowMs);
  const bool endMin = endL();
  const bool endMax = endR();

  // ------------------------------------------------------------
  // 1) LEFT FAST -> END_MIN (1. Kontakt)
  // ------------------------------------------------------------
  if (_state == HOME_SEEK_MIN_FAST) {
    if (endMin) {
      // Optional: noch ein kleines Stueck "ueberfahren" und dabei weich abbremsen,
      // bevor wir den eigentlichen Rueckzug (Backoff) starten.
      if (seekMinOverrunCounts > 0) {
        setState(HOME_SEEK_MIN_OVERRUN, nowMs);
      } else {
        motorStop();
        setState(HOME_BACKOFF_MIN_FAST, nowMs);
      }
      return;
    }

    // Wir kennen die Start-Position nicht -> nur Anfahr-Rampe nach links.
    // Rampenlaenge ist in Counts definiert.
    const long cur = _enc->getCountsDefault();
    long moved = cur - _stateStartCounts;
    if (moved < 0) moved = -moved;

    float pwm = pwmSeekMin;
    if (moved < rampSeekMinCounts) {
      const float t = (float)moved / (float)rampSeekMinCounts;
      pwm = lerpFloat(pwmSlow, pwmSeekMin, t);
    }
    motorDrive(-pwm);
    return;
  }

  // ------------------------------------------------------------
  // 1b) OPTIONAL: END_MIN ist bereits gedrueckt -> noch kurz weiterfahren
  //     und dabei sanft auf pwmSlow herunter rampen.
  // ------------------------------------------------------------
  if (_state == HOME_SEEK_MIN_OVERRUN) {
    // Falls der Schalter doch wieder frei wird (Debounce), nicht weiter druecken.
    if (!endMin) {
      motorStop();
      setState(HOME_BACKOFF_MIN_FAST, nowMs);
      return;
    }

    if (seekMinOverrunCounts <= 0) {
      motorStop();
      setState(HOME_BACKOFF_MIN_FAST, nowMs);
      return;
    }

    const long cur = _enc->getCountsDefault();
    long moved = cur - _stateStartCounts;
    if (moved < 0) moved = -moved;

    if (moved >= seekMinOverrunCounts) {
      motorStop();
      setState(HOME_BACKOFF_MIN_FAST, nowMs);
      return;
    }

    // Rampe vom aktuellen Wert (Betrag) sanft auf pwmSlow.
    // Wichtig: niemals "hoch" rampen - wenn der Schalter frueh ausloest,
    // soll die PWM nicht steigen.
    float startAbs = _overrunStartAbsPwm;
    if (startAbs < pwmSlow) startAbs = pwmSlow;
    if (startAbs > 100.0f) startAbs = 100.0f;

    const float t = (float)moved / (float)seekMinOverrunCounts;
    const float td = clampFloat(t, 0.0f, 1.0f);
    const float pwm = lerpFloat(startAbs, pwmSlow, td);

    motorDrive(-pwm);
    return;
  }

  // ------------------------------------------------------------
  // 2) BACKOFF MIN FAST -> bis END_MIN frei
  // ------------------------------------------------------------
  if (_state == HOME_BACKOFF_MIN_FAST) {
    if (!endMin) {
      motorStop();
      setState(HOME_SEEK_MIN_SLOW_1, nowMs);
      return;
    }
    motorDrive(+pwmBack);
    return;
  }

  // ------------------------------------------------------------
  // 3) LEFT SLOW -> END_MIN (1. Touch)
  // ------------------------------------------------------------
  if (_state == HOME_SEEK_MIN_SLOW_1) {
    if (endMin) {
      motorStop();
      setState(HOME_BACKOFF_MIN_SLOW, nowMs);
      return;
    }
    motorDrive(-pwmSlow);
    return;
  }

  // ------------------------------------------------------------
  // 4) BACKOFF MIN SLOW -> bis END_MIN frei (Release fuer Backlash)
  // ------------------------------------------------------------
  if (_state == HOME_BACKOFF_MIN_SLOW) {
    if (!endMin) {
      // Endschalter MIN ist jetzt frei -> Startpunkt fuer Umkehrspiel-Messung merken
      _haveBacklashBase = true;
      _backlashReleaseCounts = _enc->getCountsDefault();

      motorStop();
      setState(HOME_SEEK_MIN_SLOW_2, nowMs);
      return;
    }
    motorDrive(+pwmSlow);
    return;
  }

  // ------------------------------------------------------------
  // 5) LEFT SLOW -> END_MIN (2. Touch) -> Nullpunkt setzen
  // ------------------------------------------------------------
  if (_state == HOME_SEEK_MIN_SLOW_2) {
    if (endMin) {
      // Encoder-Strecke zwischen END_MIN „frei“ (+Backoff) und „wieder gedrueckt“
      // (zweiter −Anlauf). Wird spaeter mit backlashMeasuredToModelScale in Deg01
      // umgerechnet (siehe HOME_SEEK_MAX_SLOW_2), damit es zum einseitigen b
      // in der Positionsregelung passt.
      if (_haveBacklashBase) {
        const long pressCounts = _enc->getCountsDefault();
        long d = pressCounts - _backlashReleaseCounts;
        if (d < 0) d = -d;
        _backlashCounts = d;
        // Umrechnung in Deg01 erfolgt spaeter, sobald CPR gelernt ist.
        _backlashDeg01 = 0;
      }

      motorStop();

      // Linker Anschlag = 0deg
      _enc->setCountsZero();

      setState(HOME_SEEK_MAX_FAST, nowMs);
      return;
    }
    motorDrive(-pwmSlow);
    return;
  }

  // ------------------------------------------------------------
  // 6) RIGHT FAST -> END_MAX
  // ------------------------------------------------------------
  if (_state == HOME_SEEK_MAX_FAST) {
    if (endMax) {
      motorStop();
      setState(HOME_BACKOFF_MAX_FAST, nowMs);
      return;
    }

    // Homing-Fahrt rechts / rechter Endschalter:
    // -------------------------------------------------
    // Zielverhalten (wie besprochen):
    // - Die \"Schaetz-Strecke\" (z.B. ~160000 Counts) wird mit g_homeFastPwmPercent gefahren.
    // - Wenn diese Strecke erreicht ist, NICHT springen, sondern mit einer Rampe
    //   sauber auf g_minPwm herunter fahren.
    // - Danach wird der rechte Endschalter mit KONSTANT g_minPwm gesucht,
    //   OHNE weitere Rampen/Abbremsungen, bis der Endschalter wirklich gedrueckt ist.
    //
    // Wichtig:
    // - g_minPwm muss bis zum Schalter anliegen, sonst kann der Motor kurz davor
    //   "verhungern" (Drehmoment zu klein) und die Stall-Erkennung kann ausloesen.
    //
    // Umsetzung:
    // - "Cruise" rechts = pwmFast (kommt aus g_homeFastPwmPercent)
    // - Beschleunigungsrampe am Anfang: pwmSlow -> pwmCruiseRight
    // - Ab seekMaxDecelStartCounts: Abbremsrampe pwmCruiseRight -> pwmSlow
    // - Nach Ende der Abbremsrampe: konstant pwmSlow bis END_MAX
    const long cur = _enc->getCountsDefault();
    long moved = cur - _stateStartCounts;
    if (moved < 0) moved = -moved;

    // Rechts-Cruise: explizit g_homeFastPwmPercent (ueber _cfg.fastPwmPercent)
    const float pwmCruiseRight = pwmFast;

    // Abbremsrampe rechts: definierte Laenge (Counts), damit die Rampe vor dem Endschalter sichtbar ist.
    // Danach wird mit pwmSlow (g_minPwm) ohne weitere Rampe bis END_MAX gesucht.
    const long rampDecelCounts = rampSeekMaxDecel;
    float pwm = pwmCruiseRight;

    // 1) Beschleunigungsrampe am Anfang (pwmSlow -> pwmCruiseRight)
    if (moved < rampSeekMaxAccel) {
      const float t = (float)moved / (float)rampSeekMaxAccel;
      pwm = lerpFloat(pwmSlow, pwmCruiseRight, t);
    }

    // 2) Ab expectedCounts: Abbremsrampe auf pwmSlow (g_minPwm)
    //    Danach konstant pwmSlow bis zum Endschalter (keine weitere Rampe!)
    if (moved >= seekMaxDecelStart) {
      const long decelMoved = moved - seekMaxDecelStart;
      if (decelMoved >= rampDecelCounts) {
        pwm = pwmSlow;
      } else {
        const float t = (float)decelMoved / (float)rampDecelCounts;
        pwm = lerpFloat(pwmCruiseRight, pwmSlow, clampFloat(t, 0.0f, 1.0f));
      }
    }

    // Harte Untergrenze: sobald wir "fahren", niemals unter pwmSlow (g_minPwm)
    // (0 bleibt 0, aber hier fahren wir immer >0)
    if (pwm < pwmSlow) pwm = pwmSlow;

    motorDrive(+pwm);
    return;
  }

  // ------------------------------------------------------------
  // 7) BACKOFF MAX FAST -> bis END_MAX frei
  // ------------------------------------------------------------
  if (_state == HOME_BACKOFF_MAX_FAST) {
    if (!endMax) {
      motorStop();
      setState(HOME_SEEK_MAX_SLOW_1, nowMs);
      return;
    }
    motorDrive(-pwmBack);
    return;
  }

  // ------------------------------------------------------------
  // 8) RIGHT SLOW -> END_MAX (1. Touch)
  // ------------------------------------------------------------
  if (_state == HOME_SEEK_MAX_SLOW_1) {
    if (endMax) {
      motorStop();
      setState(HOME_BACKOFF_MAX_SLOW, nowMs);
      return;
    }
    motorDrive(+pwmBack);
    return;
  }

  // ------------------------------------------------------------
  // 9) BACKOFF MAX SLOW -> bis END_MAX frei
  // ------------------------------------------------------------
  if (_state == HOME_BACKOFF_MAX_SLOW) {
    if (!endMax) {
      motorStop();
      setState(HOME_SEEK_MAX_SLOW_2, nowMs);
      return;
    }
    motorDrive(-pwmBack);
    return;
  }

  // ------------------------------------------------------------
  // 10) RIGHT SLOW -> END_MAX (2. Touch) -> Range/CPR lernen
  // ------------------------------------------------------------
  if (_state == HOME_SEEK_MAX_SLOW_2) {
    if (endMax) {
      motorStop();

      long c = _enc->getCountsDefault();
      if (c < 0) c = -c;

      if (c < 100) {
        fail("HOME_CPR_TOO_SMALL", nowMs);
        return;
      }

      _countsPerRevLearned = (int32_t)c;
      _enc->setCountsPerRevActual(_countsPerRevLearned);

      // Jetzt koennen wir auch die Backlash-Counts in Deg01 umrechnen.
      // WICHTIG:
      // - Bei mechanisch versetztem rechten Endschalter kann die gemessene Strecke
      //   groesser als 360deg sein (z.B. +2,5deg).
      // - Die eigentliche Grad-Skalierung in der Positionsregelung wird dann ueber
      //   den globalen DGOFFSET korrigiert (EncoderAxis nutzt CPR_effektiv).
      // - Damit Backlash in Deg01 konsistent zur Positionsskalierung bleibt,
      //   rechnen wir hier ebenfalls mit dem effektiven CPR.
      // - backlashMeasuredToModelScale (typ. 0.5, NVS bms): siehe HomingConfig.
      const int32_t cprEff = _enc->getCountsPerRevEffective();
      if (_backlashCounts > 0 && cprEff > 0) {
        const float degPerCount = 360.0f / (float)cprEff;
        const float backlashDeg = (float)_backlashCounts * degPerCount;
        const float scale = clampFloat(_cfg.backlashMeasuredToModelScale, 0.1f, 1.0f);
        _backlashDeg01 = (int32_t)lroundf(backlashDeg * 100.0f * scale);
        if (_backlashDeg01 < 0) _backlashDeg01 = -_backlashDeg01;
      }

      setState(HOME_BACKOFF_MAX_FINAL, nowMs);
      return;
    }
    motorDrive(+pwmBack);
    return;
  }

  // ------------------------------------------------------------
  // 11) BACKOFF MAX FINAL -> bis END_MAX frei, dann Return-to-Zero oder DONE
  // ------------------------------------------------------------
  if (_state == HOME_BACKOFF_MAX_FINAL) {
    if (!endMax) {
      motorStop();

      if (_cfg.returnToZero) {
        setState(HOME_RETURN_MIN_FAST, nowMs);
      } else {
        _referenced = true;
        setState(HOME_DONE, nowMs);
      }
      return;
    }
    motorDrive(-pwmBack);
    return;
  }

  // ------------------------------------------------------------
  // 12) OPTIONAL: Return-to-Zero (links) mit Double-Touch
  // ------------------------------------------------------------
  if (_state == HOME_RETURN_MIN_FAST) {
    if (endMin) {
      motorStop();
      setState(HOME_BACKOFF_MIN_RT_FAST, nowMs);
      return;
    }

    // Rueckweg nach links: jetzt kennen wir die Skalierung und koennen in Grad rampen.
    // Umsetzung in Counts:
    // - Beschleunigen ueber returnRampDeg
    // - returnRampDeg vor Ziel (Counts==0) abbremsen
    int32_t cprEff = _enc->getCountsPerRevEffective();
    if (cprEff <= 0) cprEff = (int32_t)expectedCounts;

    long rampCounts = 0;
    if (returnRampDeg > 0.01f) {
      rampCounts = (long)lroundf(((float)cprEff) * returnRampDeg / 360.0f);
    }
    if (rampCounts < 1) rampCounts = 1;

    const long cur = _enc->getCountsDefault();
    long traveled = _stateStartCounts - cur; // wir fahren nach links, Counts sinken
    if (traveled < 0) traveled = -traveled;
    long remaining = cur; // Ziel ist 0 (linker Endschalter)
    if (remaining < 0) remaining = -remaining;

    float pwm = pwmFast;

    // Beschleunigen am Anfang
    if (traveled < rampCounts) {
      const float t = (float)traveled / (float)rampCounts;
      pwm = lerpFloat(pwmSlow, pwmFast, t);
    }

    // Abbremsen vor Ziel
    if (remaining < rampCounts) {
      const float t = 1.0f - ((float)remaining / (float)rampCounts);
      const float td = clampFloat(t, 0.0f, 1.0f);
      const float pwmD = lerpFloat(pwmFast, pwmSlow, td);
      if (pwmD < pwm) pwm = pwmD;
    }

    motorDrive(-pwm);
    return;
  }

  if (_state == HOME_BACKOFF_MIN_RT_FAST) {
    if (!endMin) {
      motorStop();
      setState(HOME_RETURN_MIN_SLOW_1, nowMs);
      return;
    }
    motorDrive(+pwmBack);
    return;
  }

  if (_state == HOME_RETURN_MIN_SLOW_1) {
    if (endMin) {
      motorStop();
      setState(HOME_BACKOFF_MIN_RT_SLOW, nowMs);
      return;
    }
    motorDrive(-pwmSlow);
    return;
  }

  if (_state == HOME_BACKOFF_MIN_RT_SLOW) {
    if (!endMin) {
      motorStop();
      setState(HOME_RETURN_MIN_SLOW_2, nowMs);
      return;
    }
    motorDrive(+pwmSlow);
    return;
  }

  if (_state == HOME_RETURN_MIN_SLOW_2) {
    if (endMin) {
      motorStop();

      // final: links wieder exakt 0deg
      _enc->setCountsZero();

      // WICHTIG: Jetzt noch freifahren, damit der Endschalter am Ende NICHT gedrueckt ist
      setState(HOME_BACKOFF_MIN_FINAL, nowMs);
      return;
    }
    motorDrive(-pwmSlow);
    return;
  }

  // ------------------------------------------------------------
  // 13) BACKOFF MIN FINAL -> bis END_MIN frei, dann DONE
  // ------------------------------------------------------------
  if (_state == HOME_BACKOFF_MIN_FINAL) {
    if (!endMin) {
      motorStop();

      // Optional: nach dem Freifahren noch auf logisches 0 fahren
      // (Offset/2 Abstand zum Endschalter). Das verhindert, dass wir
      // nach dem Homing "logisch 0" melden, aber mechanisch noch sehr
      // nah am Endschalter stehen.
      setState(HOME_GOTO_LOGICAL_ZERO, nowMs);
      return;
    }
    motorDrive(+pwmBack);
    return;
  }

  // ------------------------------------------------------------
  // 14) Optional: Auf logisches 0 fahren (Offset/2 weg vom linken Endschalter)
  // ------------------------------------------------------------
  if (_state == HOME_GOTO_LOGICAL_ZERO) {
    // Wenn kein Ziel berechnet werden konnte (z.B. fehlender Encoder), sofort fertig.
    if (!_gotoLogicalZeroValid) {
      motorStop();
      _referenced = true;
      setState(HOME_DONE, nowMs);
      return;
    }

    const long cur = _enc->getCountsDefault();
    const long tar = _gotoLogicalZeroTargetCounts;

    // Ziel erreicht oder bereits darueber
    if (cur >= tar) {
      motorStop();
      _referenced = true;
      setState(HOME_DONE, nowMs);
      return;
    }

    // Kleine Strecke: bewusst langsam und weich fahren.
    // Wir nutzen die approachPwmPercent (typ. g_minPwm), damit wir nicht
    // mit hoher Geschwindigkeit in die Offset-Zone "hineinspringen".
    motorDrive(+pwmSlow);
    return;
  }

  // Unbekannter Zustand
  fail("HOME_STATE_INVALID", nowMs);
}
