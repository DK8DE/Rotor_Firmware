#pragma once

#include <Arduino.h>
#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"
#include "esp_err.h"

class MotorMcpwm {
public:
  MotorMcpwm() = default;

  bool begin(int pinIn1,
             int pinIn2,
             int pinEnable,
             uint32_t pwmHz,
             uint32_t timerResolutionHz,
             bool enableActiveHigh = true);

  void stop();

  void enable(bool en);
  bool isEnabled() const { return _enabled; }

  void setEnableActiveHigh(bool activeHigh);
  bool getEnableActiveHigh() const { return _enableActiveHigh; }
  int getEnablePinLevel() const;

  // Stop nur PWM (beide LOW), Enable bleibt unveraendert
  void stopPwm();

  // Aktive Bremse (dynamisch): beide Ausgaenge HIGH.
  // on=true  => Bremse aktiv (H-Bruecke kurzgeschlossen -> bremst schnell)
  // on=false => Bremse aus (Ausgaenge wieder im normalen PWM/STOP-Modus)
  void brake(bool on);
  bool isBraking() const { return _brakeActive; }

  void setDutySigned(float dutyPercentSigned);
  float getLastDutySigned() const { return _dutySigned; }

  uint32_t getCmpATicks() const { return _cmpATicks; }
  uint32_t getCmpBTicks() const { return _cmpBTicks; }
  esp_err_t getLastEspErr() const { return _lastErr; }

private:
  void cleanup();

  void setTezAction(mcpwm_gen_handle_t gen, bool high);
  void setCompare(mcpwm_cmpr_handle_t cmp, uint32_t ticks);
  void setChannelPwmActive(bool chanAActive, float dutyAbsPercent);

private:
  int _pinIn1 = -1;
  int _pinIn2 = -1;
  int _pinEnable = -1;

  bool _enableActiveHigh = true;

  uint32_t _pwmHz = 20000;
  uint32_t _resolutionHz = 10000000;
  uint32_t _periodTicks = 0;

  bool  _ready = false;
  bool  _enabled = false;
  float _dutySigned = 0.0f;
  bool  _brakeActive = false; // true => beide Ausgaenge HIGH (aktive Bremse)

  // ------------------------------------------------------------
  // Cache: verhindert, dass wir in jedem loop die MCPWM-Generator-
  // Aktionen/Compare-Werte neu programmieren (kann sonst zu Glitches
  // und hoerbarem 'Kratzen' fuehren).
  // 0=aus, 1=IN1(A) aktiv, 2=IN2(B) aktiv
  // ------------------------------------------------------------
  uint8_t  _activeChan = 0;
  uint32_t _activeTicks = 0;

  mcpwm_timer_handle_t _timer = nullptr;
  mcpwm_oper_handle_t  _oper  = nullptr;
  mcpwm_cmpr_handle_t  _cmpA  = nullptr;
  mcpwm_cmpr_handle_t  _cmpB  = nullptr;
  mcpwm_gen_handle_t   _genA  = nullptr;
  mcpwm_gen_handle_t   _genB  = nullptr;

  uint32_t _cmpATicks = 0;
  uint32_t _cmpBTicks = 0;

  esp_err_t _lastErr = ESP_OK;
};