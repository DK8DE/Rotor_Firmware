#include "SafetyMonitor.h"
#include <math.h>

static float clampFloat(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// ------------------------------------------------------------
// Text-Mapping (fuer Serial-Ausgabe)
// ------------------------------------------------------------
const char* SafetyMonitor::warnToText(uint8_t code) const {
  switch (code) {
    case SW_IS_SOFT: return "IS_SOFT_WARN";
    case SW_WIND_GUST: return "WIND_GUST";
    case SW_DRAG_INCREASE: return "DRAG_INCREASE";
    case SW_DRAG_DECREASE: return "DRAG_DECREASE";
    case SW_TEMP_AMBIENT_HIGH: return "TEMP_AMBIENT_HIGH";
    case SW_TEMP_MOTOR_HIGH: return "TEMP_MOTOR_HIGH";
    default: return "UNKNOWN_WARN";
  }
}

void SafetyMonitor::raiseWarning(uint8_t warnCode) {
  addWarn(warnCode);
}

const char* SafetyMonitor::errToText(uint8_t code) const {
  switch (code) {
    case SE_TIMEOUT: return "TIMEOUT_DEADMAN";
    case SE_ENDSTOP: return "ENDSTOP_BLOCK";
    case SE_NSTOP_CMD: return "NSTOP_CMD";
    case SE_IS_HARD: return "IS_HARD_STOP";
    case SE_STALL: return "STALL_NO_ENCODER_MOVE";
    case SE_HOME_FAIL: return "HOMING_FAIL";
    case SE_POS_TIMEOUT: return "POS_TIMEOUT";
    default: return "UNKNOWN_ERROR";
  }
}

// ------------------------------------------------------------
// Externer Not-Aus (z.B. RS485-NSTOP)
// ------------------------------------------------------------
void SafetyMonitor::triggerEmergencyStop(uint8_t errCode) {
  // Latcht den Fehler (bis clearFault()) und sorgt so dafuer,
  // dass update() dauerhaft Duty=0 liefert.
  latchError(errCode);
}

void SafetyMonitor::logWarn(uint8_t code, uint32_t nowMs) {
  if (!_serialLogEnabled) return;

  // Moeglichst kompakt, aber aussagekraeftig
  Serial.print("[SAFETY][WARN] t=");
  Serial.print(nowMs);
  Serial.print("ms code=");
  Serial.print((int)code);
  Serial.print(" ");
  Serial.print(warnToText(code));

  // Kurzdiagnose Strom
  Serial.print(" | ISavg=");
  Serial.print(_isSnap.avg1);
  Serial.print(",");
  Serial.print(_isSnap.avg2);

  Serial.println();
}

void SafetyMonitor::logError(uint8_t errCode, uint32_t nowMs) {
  if (!_serialLogEnabled) return;

  Serial.print("[SAFETY][ERROR] t=");
  Serial.print(nowMs);
  Serial.print("ms code=");
  Serial.print((int)errCode);
  Serial.print(" ");
  Serial.print(errToText(errCode));

  Serial.print(" | ISavg=");
  Serial.print(_isSnap.avg1);
  Serial.print(",");
  Serial.print(_isSnap.avg2);

  // Endschalter: Roh + stabil (entprellt)
  Serial.print(" | ENDL=");
  Serial.print(_endLRaw ? 1 : 0);
  Serial.print("->");
  Serial.print(_endLStable ? 1 : 0);

  Serial.print(" ENDR=");
  Serial.print(_endRRaw ? 1 : 0);
  Serial.print("->");
  Serial.print(_endRStable ? 1 : 0);

  Serial.println();
}

// ------------------------------------------------------------

void SafetyMonitor::begin(HalBoard* board, const SafetyConfig& cfg) {
  _board = board;
  _cfg = cfg;

  if (_cfg.isFilterLen < 1) _cfg.isFilterLen = 1;
  if (_cfg.isFilterLen > 32) _cfg.isFilterLen = 32;

  clearFault();
  clearWarnings();

  _idx = 0;
  _sum1 = 0;
  _sum2 = 0;
  for (uint8_t i = 0; i < 32; i++) {
    _buf1[i] = 0;
    _buf2[i] = 0;
  }


  _isSnap = SafetyIsSnapshot{};
  _lastIsSampleMs = 0;
  _graceUntilMs = 0;
  _hardOverSinceMs = 0;

  // ----------------------------------------------------------
  // Endschalter-Entprellung initialisieren
  // ----------------------------------------------------------
  // Beim Start initial auf Rohzustand setzen, damit sofort ein konsistenter
  // Zustand vorhanden ist.
  if (_board) {
    _endLRaw = _board->readEndLeft();
    _endRRaw = _board->readEndRight();
    _endLStable = _endLRaw;
    _endRStable = _endRRaw;
    _endLChangeMs = millis();
    _endRChangeMs = millis();
  } else {
    _endLRaw = _endRRaw = false;
    _endLStable = _endRStable = false;
    _endLChangeMs = _endRChangeMs = millis();
  }


  // Stall-/Blockade-Erkennung zuruecksetzen
  _stallPrevMoveWanted = false;
  _stallPrevSign = 0;
  _stallArmed = false;
  _stallStartMs = 0;
  _stallStartCounts = 0;

  // Letztes Duty-Vorzeichen fuer GETIS etc. initialisieren
  _lastOutputDutySigned = 0.0f;
  _lastOutputDutySign = 0;
  _lastOutputDutySignNonZero = 0;

  _stallStartCounts = 0;
  if (_serialLogEnabled) {
    Serial.println("[SAFETY] begin()");
  }
}


void SafetyMonitor::updateConfig(const SafetyConfig& cfg) {
  // Filterlaenge clampen wie in begin()
  SafetyConfig c = cfg;
  if (c.isFilterLen < 1) c.isFilterLen = 1;
  if (c.isFilterLen > 32) c.isFilterLen = 32;

  const bool filterLenChanged = (c.isFilterLen != _cfg.isFilterLen);

  _cfg = c;

  // Wenn sich die Filterlaenge aendert, muessen wir den gleitenden Mittelwert neu initialisieren,
  // sonst stimmt die Summe/Index nicht mehr.
  if (filterLenChanged) {
    _idx = 0;
    _sum1 = 0;
    _sum2 = 0;
    for (uint8_t i = 0; i < 32; i++) {
      _buf1[i] = 0;
      _buf2[i] = 0;
    }
  }
}

// ------------------------------------------------------------
// Endschalter-Entprellung (Roh -> stabil)
// ------------------------------------------------------------
void SafetyMonitor::updateEndstopDebounce(uint32_t nowMs) {
  if (!_board) {
    _endLRaw = _endRRaw = false;
    _endLStable = _endRStable = false;
    return;
  }

  const uint32_t db = _cfg.endstopDebounceMs;
  // 0 = Entprellung aus: stable folgt sofort raw
  const bool rawL = _board->readEndLeft();
  const bool rawR = _board->readEndRight();

  if (db == 0) {
    _endLRaw = rawL;
    _endRRaw = rawR;
    _endLStable = rawL;
    _endRStable = rawR;
    return;
  }

  // Links
  if (rawL != _endLRaw) {
    _endLRaw = rawL;
    _endLChangeMs = nowMs;
  }
  if ((nowMs - _endLChangeMs) >= db) {
    _endLStable = _endLRaw;
  }

  // Rechts
  if (rawR != _endRRaw) {
    _endRRaw = rawR;
    _endRChangeMs = nowMs;
  }
  if ((nowMs - _endRChangeMs) >= db) {
    _endRStable = _endRRaw;
  }
}

void SafetyMonitor::setRestrictEndstops(bool on) {
  _cfg.restrictEndstops = on;
}

void SafetyMonitor::setAllowPushIntoEndMin(bool on) {
  _cfg.allowPushIntoEndMin = on;
}

void SafetyMonitor::setAllowPushIntoEndMax(bool on) {
  _cfg.allowPushIntoEndMax = on;
}

void SafetyMonitor::setLatchErrorOnEndstop(bool on) {
  _cfg.latchErrorOnEndstop = on;
}

void SafetyMonitor::setCmdTimeoutMs(uint32_t ms) {
  _cfg.cmdTimeoutMs = ms;
}

void SafetyMonitor::setCurrentMonitorEnabled(bool on) {
  _cfg.currentMonitorEnabled = on;
}

void SafetyMonitor::setIsSoftWarnMv(uint32_t mv) {
  _cfg.isSoftWarnMv = mv;
}

void SafetyMonitor::setIsHardStopMv(uint32_t mv) {
  _cfg.isHardStopMv = mv;
}

void SafetyMonitor::setIsGraceMs(uint32_t ms) {
  _cfg.isGraceMs = ms;
}

void SafetyMonitor::setIsHardHoldMs(uint32_t ms) {
  _cfg.isHardHoldMs = ms;
}


void SafetyMonitor::setStallMonitorEnabled(bool on) {
  _cfg.stallMonitorEnabled = on;
}

void SafetyMonitor::setStallTimeoutMs(uint32_t ms) {
  _cfg.stallTimeoutMs = ms;
}

void SafetyMonitor::setStallMinCounts(uint32_t c) {
  _cfg.stallMinCounts = c;
}

void SafetyMonitor::clearFault() {
  _faultActive = false;
  _errorCode = SE_NONE;
  _hardOverSinceMs = 0;

  // Stall-/Blockade-Erkennung ebenfalls zuruecksetzen
  _stallPrevMoveWanted = false;
  _stallPrevSign = 0;
  _stallArmed = false;
  _stallStartMs = 0;
  _stallStartCounts = 0;
  if (_serialLogEnabled) {
    Serial.println("[SAFETY] clearFault()");
  }
}

void SafetyMonitor::clearWarnings() {
  _warnCount = 0;
  for (uint8_t i = 0; i < 8; i++) _warns[i] = 0;

  if (_serialLogEnabled) {
    Serial.println("[SAFETY] clearWarnings()");
  }
}

void SafetyMonitor::resetPeaks() {
  _isSnap.peak1 = 0;
  _isSnap.peak2 = 0;

  if (_serialLogEnabled) {
    Serial.println("[SAFETY] resetPeaks()");
  }
}

void SafetyMonitor::notifyMotionEdge(uint32_t nowMs) {
  _graceUntilMs = nowMs + _cfg.isGraceMs;
  _hardOverSinceMs = 0;

  // Neue Bewegung startet: Stall-Ueberwachung neu vorbereiten
  _stallPrevMoveWanted = false;
  _stallPrevSign = 0;
  _stallArmed = false;
  _stallStartMs = 0;
  _stallStartCounts = 0;
  if (_serialLogEnabled) {
    Serial.print("[SAFETY] motionEdge -> graceUntil=");
    Serial.print(_graceUntilMs);
    Serial.println("ms");
  }
}

void SafetyMonitor::addWarn(uint8_t code) {
  if (code == 0) return;

  // keine Duplikate
  for (uint8_t i = 0; i < _warnCount; i++) {
    if (_warns[i] == code) return;
  }

  if (_warnCount < 8) {
    _warns[_warnCount] = code;
    _warnCount++;

    // Logging beim erstmaligen Auftreten
    logWarn(code, millis());
  }
}

void SafetyMonitor::latchError(uint8_t errCode) {
  if (errCode == 0) return;

  // Nur beim ersten Latch loggen
  bool first = (!_faultActive);

  _faultActive = true;
  if (_errorCode == SE_NONE) {
    _errorCode = errCode;
  }

  if (first) {
    logError(_errorCode, millis());
  }
}

uint8_t SafetyMonitor::getWarnAt(uint8_t idx) const {
  if (idx >= _warnCount) return 0;
  return _warns[idx];
}

void SafetyMonitor::updateIsSampling(uint32_t nowMs) {
  if (!_cfg.currentMonitorEnabled) {
    _isSnap.softWarn = false;
    _isSnap.hardOver = false;
    _isSnap.graceActive = false;
    return;
  }
  if (!_board) return;

  if ((nowMs - _lastIsSampleMs) < _cfg.isSampleIntervalMs) return;
  _lastIsSampleMs = nowMs;

  // ADC-Rohwerte (mV) direkt von der Hardware
  uint32_t raw1 = _board->readIs1mV();
  uint32_t raw2 = _board->readIs2mV();

  // Fuer Debug/Diagnose: Rohwerte VOR Offset-Abzug merken
  _isSnap.adc1 = raw1;
  _isSnap.adc2 = raw2;

  // Offsets abziehen (clamp >=0), damit Grundoffset der IS-Eingaenge nicht stoert.
  uint32_t v1 = (raw1 > _cfg.isOffset1Mv) ? (raw1 - _cfg.isOffset1Mv) : 0;
  uint32_t v2 = (raw2 > _cfg.isOffset2Mv) ? (raw2 - _cfg.isOffset2Mv) : 0;

  _isSnap.raw1 = v1;
  _isSnap.raw2 = v2;

  uint8_t len = _cfg.isFilterLen;
  if (len < 1) len = 1;
  if (len > 32) len = 32;

  _sum1 -= _buf1[_idx];
  _sum2 -= _buf2[_idx];

  _buf1[_idx] = v1;
  _buf2[_idx] = v2;

  _sum1 += v1;
  _sum2 += v2;

  _idx++;
  if (_idx >= len) _idx = 0;

  _isSnap.avg1 = _sum1 / len;
  _isSnap.avg2 = _sum2 / len;

  if (v1 > _isSnap.peak1) _isSnap.peak1 = v1;
  if (v2 > _isSnap.peak2) _isSnap.peak2 = v2;

  uint32_t maxAvg = (_isSnap.avg1 > _isSnap.avg2) ? _isSnap.avg1 : _isSnap.avg2;
  _isSnap.softWarn = (maxAvg >= _cfg.isSoftWarnMv);

  _isSnap.graceActive = (nowMs < _graceUntilMs);
}

void SafetyMonitor::updateIsProtection(uint32_t nowMs) {
  if (!_cfg.currentMonitorEnabled) return;
  if (_faultActive) return;

  uint32_t maxAvg = (_isSnap.avg1 > _isSnap.avg2) ? _isSnap.avg1 : _isSnap.avg2;

  if (maxAvg >= _cfg.isSoftWarnMv) {
    addWarn(SW_IS_SOFT);
  }

  // Hard erst nach Grace
  if (nowMs < _graceUntilMs) {
    _isSnap.hardOver = false;
    _hardOverSinceMs = 0;
    return;
  }

  if (maxAvg >= _cfg.isHardStopMv) {
    _isSnap.hardOver = true;

    if (_hardOverSinceMs == 0) {
      _hardOverSinceMs = nowMs;
    } else {
      uint32_t dt = nowMs - _hardOverSinceMs;
      if (dt >= _cfg.isHardHoldMs) {
        latchError(SE_IS_HARD);
      }
    }
  } else {
    _isSnap.hardOver = false;
    _hardOverSinceMs = 0;
  }
}


void SafetyMonitor::updateStallProtection(uint32_t nowMs, float desiredDutySigned, long encoderCounts) {
  if (!_cfg.stallMonitorEnabled) {
    // Wenn deaktiviert: Zustand sauber zuruecksetzen, damit spaetere Aktivierung sauber startet
    _stallPrevMoveWanted = false;
    _stallPrevSign = 0;
    _stallArmed = false;
    _stallStartMs = 0;
    _stallStartCounts = 0;

    // Micro-Stall ebenfalls zuruecksetzen
    _stallMicroArmed = false;
    _stallMicroStartMs = 0;
    _stallMicroStartCounts = 0;
    return;
  }
  if (_faultActive) return;

  // Wir betrachten "Bewegung gewollt", wenn der Duty eine kleine Schwelle ueberschreitet.
  // (Damit wir NICHT in Ziel-Hold/Fine-Phasen mit sehr kleinen PWMs faelschlich ausloesen.)
  const float armDutyAbsCfg = _cfg.stallArmDutyAbs;
  // "Schnell"-Stall: ab dieser PWM erwarten wir innerhalb stallTimeoutMs eine relevante Bewegung
  const float armDutyAbs = (armDutyAbsCfg > 0.0f) ? armDutyAbsCfg : 2.0f; // % PWM

  const float absDuty = fabsf(desiredDutySigned);

  // ----------------------------------------------------------
  // Micro-Stall (sehr kleine Schritte / sehr kleine Bewegungsanforderung)
  // ----------------------------------------------------------
  // Hintergrund:
  // - Bei extrem kleinen Sollbewegungen (z.B. 0,01deg) kann es sein, dass der Controller
  //   bewusst nur mit der Mindest-PWM (Anlauf/Creep) arbeitet.
  // - Wenn stallArmDutyAbs == Mindest-PWM waere, wuerde SE_STALL bei solchen Mini-Schritten
  //   schnell ausloesen, obwohl es in der Praxis okay ist, wenn der Schritt nicht sofort "greift".
  //
  // Loesung:
  // - "Schnell"-Stall startet erst ab armDutyAbs (typischerweise etwas ueber Mindest-PWM).
  // - Zusaetzlich ueberwachen wir in einem deutlich laengeren Fenster, ob ueberhaupt *irgendeine*
  //   Encoderbewegung erfolgt, sobald eine Mindest-PWM anliegt.
  //
  // microArmDutyAbs: etwas unter der Schnell-Schwelle. Empfehlung in der INO:
  //   g_minStallPwm = g_minPwm + 2..5
  // Dann liegt microArmDutyAbs typischerweise in der Naehe von g_minPwm,
  // ohne dass wir dafuer einen extra Parameter brauchen.
  const float microArmDutyAbs = (armDutyAbs > 6.0f) ? (armDutyAbs - 5.0f) : armDutyAbs;
  const bool wantMicroWatch = (absDuty >= microArmDutyAbs);

  if (wantMicroWatch) {
    if (!_stallMicroArmed) {
      _stallMicroArmed = true;
      _stallMicroStartMs = nowMs;
      _stallMicroStartCounts = encoderCounts;
    } else {
      long d = encoderCounts - _stallMicroStartCounts;
      if (d < 0) d = -d;

      // Jede reale Bewegung (>= 1 Count) zaehlt als "nicht blockiert" und setzt die Micro-Uhr zurueck
      if (d >= 1) {
        _stallMicroArmed = false;
        _stallMicroStartMs = 0;
        _stallMicroStartCounts = encoderCounts;
      } else {
        // Deutlich laengeres Fenster als der normale Stall-Timeout
        const uint32_t baseTout = _cfg.stallTimeoutMs;
        uint32_t microTout = baseTout * 2;
        if (microTout < 3000) microTout = 3000;

        const uint32_t dtMicro = nowMs - _stallMicroStartMs;
        if (baseTout > 0 && dtMicro >= microTout) {
          latchError(SE_STALL);
        }
      }
    }
  } else {
    // Keine Bewegung gefordert (oder PWM zu klein) -> Micro-Ueberwachung stoppen
    _stallMicroArmed = false;
    _stallMicroStartMs = 0;
    _stallMicroStartCounts = encoderCounts;
  }

  // ----------------------------------------------------------
  // "Schnell"-Stall: nur wenn PWM gross genug ist
  // ----------------------------------------------------------
  // Hintergrund (Praxis):
  // - In der Feinphase / beim Anfahren (KICK) kann die PWM um die Mindest-PWM herum liegen.
  // - Bei sehr langsamer Bewegung koennen innerhalb stallTimeoutMs ggf. weniger als stallMinCounts
  //   anfallen, obwohl sich der Rotor *tatsaechlich* bewegt.
  // - Wenn g_minStallPwm versehentlich gleich g_minPwm konfiguriert ist, wuerde das besonders haeufig
  //   zu SE_STALL fuehren (z.B. beim Umkehren in der Feinphase).
  //
  // Loesung:
  // - Micro-Stall bleibt wie gehabt aktiv (>= microArmDutyAbs, lange Zeit, 1 Count reicht).
  // - Der "Schnell"-Stall wird erst oberhalb einer zusaetzlichen Margin armiert.
  //   Dadurch bleibt die Erkennung bei hoher PWM schnell/streng, ist aber nahe der Mindest-PWM robust.
  const float fastMargin = 5.0f; // %PWM zusaetzlich ueber armDutyAbs
  const float fastArmDutyAbs = armDutyAbs + fastMargin;

  const bool wantMoveFast = (absDuty >= fastArmDutyAbs);

  if (!wantMoveFast) {
    // Kein Move gewollt -> Stall-Fenster stoppen/reset
    _stallPrevMoveWanted = false;
    _stallPrevSign = 0;
    _stallArmed = false;
    _stallStartMs = 0;
    _stallStartCounts = 0;
    return;
  }

  // Vorzeichen der aktuell gewuenschten Bewegung
  const int8_t signNow = (desiredDutySigned > 0.0f) ? 1 : -1;

  // Armieren:
  // - Rising-Edge: von "kein Move" -> "Move"
  //
  // WICHTIG (Fix fuer Homing):
  // Beim Homing kann die Richtung zwischen Segmenten wechseln (Fast/Backoff/Approach).
  // Wenn wir bei jedem Richtungswechsel die Startzeit zuruecksetzen, kann das Stall-Timeout
  // nie ausloesen (Timer wird staendig neu gestartet).
  //
  // Deshalb:
  // - Bei Richtungswechsel aktualisieren wir nur die Counts-Basis (neue Sollrichtung),
  //   behalten aber die urspruengliche Startzeit bei, solange noch KEINE Bewegung erkannt wurde.
  if (!_stallPrevMoveWanted) {
    _stallArmed = true;
    _stallStartMs = nowMs;
    _stallStartCounts = encoderCounts;
    _stallPrevSign = signNow;
  } else {
    // Richtungswechsel waehrend Move: nur Basis neu setzen, Zeit NICHT resetten
    const bool signChanged = (_stallPrevSign != 0) && (signNow != _stallPrevSign);
    if (signChanged) {
      _stallStartCounts = encoderCounts;
      _stallPrevSign = signNow;
      // _stallStartMs bleibt unveraendert!
      if (!_stallArmed) {
        _stallArmed = true;
        _stallStartMs = nowMs;
      }
    }
  }

  if (_stallArmed) {
    // Fortschritt nur in der gewollten Richtung zaehlen.
    // -> Bewegung in Gegenrichtung ignorieren (EMV/Elastizitaet).
    long prog = (signNow > 0) ? (encoderCounts - _stallStartCounts)
                              : (_stallStartCounts - encoderCounts);
    if (prog < 0) prog = 0;
    const uint32_t dC = (uint32_t)prog;

    const uint32_t minC = _cfg.stallMinCounts;
    const uint32_t tout = _cfg.stallTimeoutMs;

    // 0 bedeutet "aus"
    if (minC == 0 || tout == 0) {
      _stallArmed = false;
      _stallStartMs = 0;
      _stallStartCounts = encoderCounts;
    } else if (dC >= minC) {
      // Bewegung erkannt -> Stall-Fenster beenden
      _stallArmed = false;
      _stallStartMs = 0;
      _stallStartCounts = encoderCounts;
    } else {
      // Noch keine Bewegung -> Timeout pruefen
      const uint32_t dt = nowMs - _stallStartMs;
      if (dt >= tout) {
        latchError(SE_STALL);
      }
    }
  }

  _stallPrevMoveWanted = true;
}

float SafetyMonitor::update(uint32_t nowMs,
                            float desiredDutySigned,
                            bool rsMotionActive,
                            uint32_t lastMotionCmdMs,
                            long encoderCounts) {
  float duty = clampFloat(desiredDutySigned, -100.0f, 100.0f);

  // ----------------------------------------------------------
  // Deadman-Logik (Joerg):
  // - Deadman soll NUR dann zuschlagen, wenn WAEHREND EINER BEWEGUNG
  //   (d.h. es wird eine PWM != 0 angefordert) laenger als cmdTimeoutMs
  //   KEIN gueltiges Kommando vom Master empfangen wurde.
  // - Wenn der Rotor gerade NICHT bewegt wird (Duty ~ 0), darf der Deadman
  //   nicht ausloesen (z.B. kurze Haltephasen im Homing, Segmentwechsel,
  //   Entprellfenster, interne Zustandswechsel).
  //
  // Hinweis:
  // - "rsMotionActive" bleibt true waehrend Homing/Positionsfahrt.
  // - Das alleine reicht aber nicht, weil es Phasen ohne Motorbewegung gibt.
  // - Daher koppeln wir den Timeout zusaetzlich an "moveWanted".
  // ----------------------------------------------------------
  const bool moveWanted = (fabsf(duty) > 0.01f);

  // Endschalter entprellen, bevor wir ihn auswerten
  updateEndstopDebounce(nowMs);

  updateIsSampling(nowMs);
  updateIsProtection(nowMs);

  // Deadman Timeout (nur wenn explizit aktiv UND Bewegung angefordert)
  if ((_cfg.cmdTimeoutMs > 0) && rsMotionActive && moveWanted) {
    const uint32_t age = nowMs - lastMotionCmdMs;
    if (age > _cfg.cmdTimeoutMs) {
      latchError(SE_TIMEOUT);
      duty = 0.0f;
    }
  }

  // Endschalter je Richtung
  if (_cfg.restrictEndstops) {
    const bool endL = _endLStable;
    const bool endR = _endRStable;
    const bool allowMin = _cfg.allowPushIntoEndMin;
    const bool allowMax = _cfg.allowPushIntoEndMax;

    // Richtungskonvention waehlbar (je nach Verdrahtung/Motorrichtung)
    // dutyPositiveMovesRightEndstop=true (Standard):
    //   duty<0 -> Richtung LEFT (MIN), duty>0 -> Richtung RIGHT (MAX)
    // dutyPositiveMovesRightEndstop=false (invertiert):
    //   duty<0 -> Richtung RIGHT (MAX), duty>0 -> Richtung LEFT (MIN)
    if (_cfg.dutyPositiveMovesRightEndstop) {
      if ((duty < 0.0f) && endL) {
        // Richtung LEFT (MIN)
        if (!allowMin) {
          duty = 0.0f;
          if (_cfg.latchErrorOnEndstop && rsMotionActive) {
            latchError(SE_ENDSTOP);
          }
        }
      } else if ((duty > 0.0f) && endR) {
        // Richtung RIGHT (MAX)
        if (!allowMax) {
          duty = 0.0f;
          if (_cfg.latchErrorOnEndstop && rsMotionActive) {
            latchError(SE_ENDSTOP);
          }
        }
      }
    } else {
      // invertiert
      if ((duty < 0.0f) && endR) {
        // duty<0 bedeutet Richtung RIGHT (MAX)
        if (!allowMax) {
          duty = 0.0f;
          if (_cfg.latchErrorOnEndstop && rsMotionActive) {
            latchError(SE_ENDSTOP);
          }
        }
      } else if ((duty > 0.0f) && endL) {
        // duty>0 bedeutet Richtung LEFT (MIN)
        if (!allowMin) {
          duty = 0.0f;
          if (_cfg.latchErrorOnEndstop && rsMotionActive) {
            latchError(SE_ENDSTOP);
          }
        }
      }
    }
  }

  // Stall-/Blockade-Erkennung ueber Encoder (unabhaengig von Strommessung)
  // -> schuetzt besonders beim Anfahren/Rampenstart, wenn der Strom noch niedrig ist.
  // WICHTIG:
  // Stall MUSS NACH der Endschalter-Restriktion laufen.
  // Sonst koennte es passieren, dass der Endschalter die PWM auf 0 klemmt,
  // aber der Stall-Monitor trotzdem "Bewegung gewollt" sieht und faelschlich
  // SE_STALL latches.
  updateStallProtection(nowMs, duty, encoderCounts);
  if (_faultActive) {
    duty = 0.0f;
  }


  // ----------------------------------------------------------
  // Letztes von Safety freigegebenes Duty fuer Diagnose/RS485 merken
  // ----------------------------------------------------------
  _lastOutputDutySigned = duty;

  if (duty > 0.01f) {
    _lastOutputDutySign = 1;
    _lastOutputDutySignNonZero = 1;
  } else if (duty < -0.01f) {
    _lastOutputDutySign = -1;
    _lastOutputDutySignNonZero = -1;
  } else {
    _lastOutputDutySign = 0;
    // _lastOutputDutySignNonZero bleibt unveraendert
  }

  return duty;
}
