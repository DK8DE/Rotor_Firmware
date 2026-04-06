#include "MotorMcpwm.h"
#include <math.h>

static inline uint32_t clamp_u32(uint32_t v, uint32_t lo, uint32_t hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

void MotorMcpwm::cleanup() {
  if (_pinEnable >= 0) {
    pinMode(_pinEnable, OUTPUT);
    digitalWrite(_pinEnable, _enableActiveHigh ? LOW : HIGH);
  }

  if (_genA) { mcpwm_del_generator(_genA); _genA = nullptr; }
  if (_genB) { mcpwm_del_generator(_genB); _genB = nullptr; }

  if (_cmpA) { mcpwm_del_comparator(_cmpA); _cmpA = nullptr; }
  if (_cmpB) { mcpwm_del_comparator(_cmpB); _cmpB = nullptr; }

  if (_oper) { mcpwm_del_operator(_oper); _oper = nullptr; }

  if (_timer) {
    (void)mcpwm_timer_start_stop(_timer, MCPWM_TIMER_STOP_EMPTY);
    (void)mcpwm_timer_disable(_timer);
    (void)mcpwm_del_timer(_timer);
    _timer = nullptr;
  }

  _ready = false;
}

void MotorMcpwm::setTezAction(mcpwm_gen_handle_t gen, bool high) {
  if (!gen) return;

  esp_err_t e = mcpwm_generator_set_action_on_timer_event(
    gen,
    (mcpwm_gen_timer_event_action_t){
      .direction = MCPWM_TIMER_DIRECTION_UP,
      .event = MCPWM_TIMER_EVENT_EMPTY,
      .action = high ? MCPWM_GEN_ACTION_HIGH : MCPWM_GEN_ACTION_LOW
    }
  );
  if (e != ESP_OK) _lastErr = e;
}

void MotorMcpwm::setCompare(mcpwm_cmpr_handle_t cmp, uint32_t ticks) {
  if (!cmp) return;
  esp_err_t e = mcpwm_comparator_set_compare_value(cmp, (int)ticks);
  if (e != ESP_OK) _lastErr = e;
}

bool MotorMcpwm::begin(int pinIn1,
                      int pinIn2,
                      int pinEnable,
                      uint32_t pwmHz,
                      uint32_t timerResolutionHz,
                      bool enableActiveHigh) {
  _pinIn1 = pinIn1;
  _pinIn2 = pinIn2;
  _pinEnable = pinEnable;

  _pwmHz = pwmHz;
  _resolutionHz = timerResolutionHz;
  _enableActiveHigh = enableActiveHigh;

  _lastErr = ESP_OK;

  if (_pinIn1 < 0 || _pinIn2 < 0 || _pinEnable < 0) return false;
  if (_pwmHz == 0 || _resolutionHz == 0) return false;

  _periodTicks = _resolutionHz / _pwmHz;
  if (_periodTicks < 10) return false;

  pinMode(_pinEnable, OUTPUT);
  digitalWrite(_pinEnable, _enableActiveHigh ? LOW : HIGH);
  _enabled = false;

  mcpwm_timer_config_t timer_cfg = {};
  timer_cfg.group_id = 0;
  timer_cfg.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
  timer_cfg.resolution_hz = _resolutionHz;
  timer_cfg.period_ticks = _periodTicks;
  timer_cfg.count_mode = MCPWM_TIMER_COUNT_MODE_UP;

  esp_err_t err = mcpwm_new_timer(&timer_cfg, &_timer);
  if (err != ESP_OK || !_timer) { _lastErr = err; cleanup(); return false; }

  err = mcpwm_timer_enable(_timer);
  if (err != ESP_OK) { _lastErr = err; cleanup(); return false; }

  err = mcpwm_timer_start_stop(_timer, MCPWM_TIMER_START_NO_STOP);
  if (err != ESP_OK) { _lastErr = err; cleanup(); return false; }

  mcpwm_operator_config_t oper_cfg = {};
  oper_cfg.group_id = 0;
  oper_cfg.intr_priority = 0;

  err = mcpwm_new_operator(&oper_cfg, &_oper);
  if (err != ESP_OK || !_oper) { _lastErr = err; cleanup(); return false; }

  err = mcpwm_operator_connect_timer(_oper, _timer);
  if (err != ESP_OK) { _lastErr = err; cleanup(); return false; }

  mcpwm_comparator_config_t cmp_cfg = {};
  cmp_cfg.intr_priority = 0;
  cmp_cfg.flags.update_cmp_on_tez = true;

  err = mcpwm_new_comparator(_oper, &cmp_cfg, &_cmpA);
  if (err != ESP_OK || !_cmpA) { _lastErr = err; cleanup(); return false; }

  err = mcpwm_new_comparator(_oper, &cmp_cfg, &_cmpB);
  if (err != ESP_OK || !_cmpB) { _lastErr = err; cleanup(); return false; }

  setCompare(_cmpA, 0);
  setCompare(_cmpB, 0);
  _cmpATicks = 0;
  _cmpBTicks = 0;

  mcpwm_generator_config_t gen_cfg_a = {};
  gen_cfg_a.gen_gpio_num = (gpio_num_t)_pinIn1;

  err = mcpwm_new_generator(_oper, &gen_cfg_a, &_genA);
  if (err != ESP_OK || !_genA) { _lastErr = err; cleanup(); return false; }

  mcpwm_generator_config_t gen_cfg_b = {};
  gen_cfg_b.gen_gpio_num = (gpio_num_t)_pinIn2;

  err = mcpwm_new_generator(_oper, &gen_cfg_b, &_genB);
  if (err != ESP_OK || !_genB) { _lastErr = err; cleanup(); return false; }

  err = mcpwm_generator_set_action_on_compare_event(
    _genA,
    (mcpwm_gen_compare_event_action_t){
      .direction = MCPWM_TIMER_DIRECTION_UP,
      .comparator = _cmpA,
      .action = MCPWM_GEN_ACTION_LOW
    }
  );
  if (err != ESP_OK) { _lastErr = err; cleanup(); return false; }

  err = mcpwm_generator_set_action_on_compare_event(
    _genB,
    (mcpwm_gen_compare_event_action_t){
      .direction = MCPWM_TIMER_DIRECTION_UP,
      .comparator = _cmpB,
      .action = MCPWM_GEN_ACTION_LOW
    }
  );
  if (err != ESP_OK) { _lastErr = err; cleanup(); return false; }

  err = mcpwm_generator_set_action_on_timer_event(
    _genA,
    (mcpwm_gen_timer_event_action_t){
      .direction = MCPWM_TIMER_DIRECTION_UP,
      .event = MCPWM_TIMER_EVENT_FULL,
      .action = MCPWM_GEN_ACTION_LOW
    }
  );
  if (err != ESP_OK) { _lastErr = err; cleanup(); return false; }

  err = mcpwm_generator_set_action_on_timer_event(
    _genB,
    (mcpwm_gen_timer_event_action_t){
      .direction = MCPWM_TIMER_DIRECTION_UP,
      .event = MCPWM_TIMER_EVENT_FULL,
      .action = MCPWM_GEN_ACTION_LOW
    }
  );
  if (err != ESP_OK) { _lastErr = err; cleanup(); return false; }

  // Start: beide LOW
  setTezAction(_genA, false);
  setTezAction(_genB, false);

  _dutySigned = 0.0f;
  _ready = true;
  return true;
}

void MotorMcpwm::stop() {
  stopPwm();
  enable(false);
  cleanup();
}

void MotorMcpwm::setEnableActiveHigh(bool activeHigh) {
  _enableActiveHigh = activeHigh;
  enable(_enabled);
}

int MotorMcpwm::getEnablePinLevel() const {
  if (_pinEnable < 0) return -1;
  return digitalRead(_pinEnable);
}

void MotorMcpwm::enable(bool en) {
  _enabled = en;
  if (_pinEnable < 0) return;

  if (_enableActiveHigh) {
    digitalWrite(_pinEnable, en ? HIGH : LOW);
  } else {
    digitalWrite(_pinEnable, en ? LOW : HIGH);
  }
}

void MotorMcpwm::stopPwm() {
  if (!_ready) return;

  // Falls aktive Bremse anliegt: erst loesen, dann sauber auf LOW.
  if (_brakeActive) {
    brake(false);
  }

  // Wenn PWM bereits aus ist, nicht dauernd Generator/Compare neu programmieren
  // (reduziert Glitches).
  if (_activeChan == 0 && _cmpATicks == 0 && _cmpBTicks == 0) {
    _dutySigned = 0.0f;
    return;
  }

  setTezAction(_genA, false);
  setTezAction(_genB, false);

  if (_cmpATicks != 0) setCompare(_cmpA, 0);
  if (_cmpBTicks != 0) setCompare(_cmpB, 0);

  _cmpATicks = 0;
  _cmpBTicks = 0;
  _dutySigned = 0.0f;

  _activeChan = 0;
  _activeTicks = 0;
}

void MotorMcpwm::brake(bool on) {
  if (!_ready) return;

  // Wenn wir schon im gewuenschten Zustand sind: nichts tun.
  if (on == _brakeActive) {
    return;
  }

  if (on) {
    // ----------------------------------------------------------
    // Bremse EIN: beide Ausgaenge dauerhaft HIGH
    // ----------------------------------------------------------
    // PWM-Status zuruecksetzen (wir wollen kein "Richtungs-PWM" mehr)
    _activeChan = 0;
    _activeTicks = 0;

    // Compare auf 0, damit keine Pulse entstehen
    if (_cmpATicks != 0) setCompare(_cmpA, 0);
    if (_cmpBTicks != 0) setCompare(_cmpB, 0);
    _cmpATicks = 0;
    _cmpBTicks = 0;

    // Timer EMPTY -> HIGH
    setTezAction(_genA, true);
    setTezAction(_genB, true);

    // Timer FULL -> HIGH (damit er nicht am Periodenende wieder LOW wird)
    esp_err_t e = mcpwm_generator_set_action_on_timer_event(
      _genA,
      (mcpwm_gen_timer_event_action_t){
        .direction = MCPWM_TIMER_DIRECTION_UP,
        .event = MCPWM_TIMER_EVENT_FULL,
        .action = MCPWM_GEN_ACTION_HIGH
      }
    );
    if (e != ESP_OK) _lastErr = e;

    e = mcpwm_generator_set_action_on_timer_event(
      _genB,
      (mcpwm_gen_timer_event_action_t){
        .direction = MCPWM_TIMER_DIRECTION_UP,
        .event = MCPWM_TIMER_EVENT_FULL,
        .action = MCPWM_GEN_ACTION_HIGH
      }
    );
    if (e != ESP_OK) _lastErr = e;

    // Compare-Event -> HIGH (Sicherheitshalber)
    e = mcpwm_generator_set_action_on_compare_event(
      _genA,
      (mcpwm_gen_compare_event_action_t){
        .direction = MCPWM_TIMER_DIRECTION_UP,
        .comparator = _cmpA,
        .action = MCPWM_GEN_ACTION_HIGH
      }
    );
    if (e != ESP_OK) _lastErr = e;

    e = mcpwm_generator_set_action_on_compare_event(
      _genB,
      (mcpwm_gen_compare_event_action_t){
        .direction = MCPWM_TIMER_DIRECTION_UP,
        .comparator = _cmpB,
        .action = MCPWM_GEN_ACTION_HIGH
      }
    );
    if (e != ESP_OK) _lastErr = e;

    _dutySigned = 0.0f;
    _brakeActive = true;
    return;
  }

  // ------------------------------------------------------------
  // Bremse AUS: Normalzustand wiederherstellen (PWM/STOP)
  // ------------------------------------------------------------
  esp_err_t e = mcpwm_generator_set_action_on_compare_event(
    _genA,
    (mcpwm_gen_compare_event_action_t){
      .direction = MCPWM_TIMER_DIRECTION_UP,
      .comparator = _cmpA,
      .action = MCPWM_GEN_ACTION_LOW
    }
  );
  if (e != ESP_OK) _lastErr = e;

  e = mcpwm_generator_set_action_on_compare_event(
    _genB,
    (mcpwm_gen_compare_event_action_t){
      .direction = MCPWM_TIMER_DIRECTION_UP,
      .comparator = _cmpB,
      .action = MCPWM_GEN_ACTION_LOW
    }
  );
  if (e != ESP_OK) _lastErr = e;

  e = mcpwm_generator_set_action_on_timer_event(
    _genA,
    (mcpwm_gen_timer_event_action_t){
      .direction = MCPWM_TIMER_DIRECTION_UP,
      .event = MCPWM_TIMER_EVENT_FULL,
      .action = MCPWM_GEN_ACTION_LOW
    }
  );
  if (e != ESP_OK) _lastErr = e;

  e = mcpwm_generator_set_action_on_timer_event(
    _genB,
    (mcpwm_gen_timer_event_action_t){
      .direction = MCPWM_TIMER_DIRECTION_UP,
      .event = MCPWM_TIMER_EVENT_FULL,
      .action = MCPWM_GEN_ACTION_LOW
    }
  );
  if (e != ESP_OK) _lastErr = e;

  // Timer EMPTY -> LOW
  setTezAction(_genA, false);
  setTezAction(_genB, false);

  // Compare auf 0
  if (_cmpATicks != 0) setCompare(_cmpA, 0);
  if (_cmpBTicks != 0) setCompare(_cmpB, 0);
  _cmpATicks = 0;
  _cmpBTicks = 0;

  _activeChan = 0;
  _activeTicks = 0;
  _dutySigned = 0.0f;

  _brakeActive = false;
}

void MotorMcpwm::setChannelPwmActive(bool chanAActive, float dutyAbsPercent) {
  if (!_ready) return;

  if (dutyAbsPercent < 0.0f) dutyAbsPercent = 0.0f;
  if (dutyAbsPercent > 100.0f) dutyAbsPercent = 100.0f;

  uint32_t ticks = (uint32_t)((dutyAbsPercent * (float)_periodTicks) / 100.0f);

  if (ticks == 0) {
    stopPwm();
    return;
  }

  // 100% -> periodTicks, wir klemmen auf periodTicks-1 (FULL setzt ohnehin LOW)
  ticks = clamp_u32(ticks, 1, _periodTicks - 1);

  uint8_t wantChan = chanAActive ? 1 : 2;

  // Wenn Kanal und Duty unveraendert sind: gar nichts tun (wichtig gegen Glitches)
  if (_activeChan == wantChan && _activeTicks == ticks) {
    return;
  }

  // Wenn nur Duty geaendert wurde (gleiche Richtung): nur Compare anpassen
  if (_activeChan == wantChan && _activeChan != 0) {
    if (wantChan == 1) {
      if (_cmpATicks != ticks) {
        setCompare(_cmpA, ticks);
        _cmpATicks = ticks;
      }
    } else {
      if (_cmpBTicks != ticks) {
        setCompare(_cmpB, ticks);
        _cmpBTicks = ticks;
      }
    }
    _activeTicks = ticks;
    return;
  }

  // Kanalwechsel oder von STOP -> aktiv: nur dann TEZ-Aktionen umschalten
  if (chanAActive) {
    // IN1 PWM, IN2 LOW
    setTezAction(_genA, true);
    setCompare(_cmpA, ticks);
    _cmpATicks = ticks;

    setTezAction(_genB, false);
    if (_cmpBTicks != 0) setCompare(_cmpB, 0);
    _cmpBTicks = 0;
  } else {
    // IN2 PWM, IN1 LOW
    setTezAction(_genA, false);
    if (_cmpATicks != 0) setCompare(_cmpA, 0);
    _cmpATicks = 0;

    setTezAction(_genB, true);
    setCompare(_cmpB, ticks);
    _cmpBTicks = ticks;
  }

  _activeChan = wantChan;
  _activeTicks = ticks;
}


void MotorMcpwm::setDutySigned(float dutyPercentSigned) {
  if (!_ready) return;

  // Wenn zuvor aktiv gebremst wurde, muessen wir die Bremse loesen,
  // bevor wieder PWM ausgegeben wird.
  if (_brakeActive) {
    brake(false);
  }

  if (dutyPercentSigned > 100.0f) dutyPercentSigned = 100.0f;
  if (dutyPercentSigned < -100.0f) dutyPercentSigned = -100.0f;

  _dutySigned = dutyPercentSigned;

  float absDuty = fabsf(dutyPercentSigned);
  if (absDuty < 0.001f) {
    stopPwm();
    return;
  }

  if (dutyPercentSigned > 0.0f) {
    setChannelPwmActive(true, absDuty);
  } else {
    setChannelPwmActive(false, absDuty);
  }
}
