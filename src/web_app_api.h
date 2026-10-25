#pragma once

#include <Arduino.h>
#include <stddef.h>

// Bridge SoftAP-Web-UI ↔ main.cpp (Implementierung in main.cpp)

/** SoftAP-UID aus MAC (z. B. "A1B2C3"). */
void webAppDeviceUid(char* out, size_t outLen);

/** Firmware-Version string (FW_VERSION_STRING). */
const char* webAppFwVersion();

/**
 * Live-JSON fuer /api/live und /api/status.
 * Felder: angle,tempA,tempM,fw,homed,encType,pwm,minPwm,maxPwm,uptime,heap,ssid,ip,uid
 */
void webAppFillLiveJson(char* buf, size_t buflen, const char* ssid, const char* ip);

/** Setup-Wizard: kompletter Ist-Zustand als JSON. */
void webAppFillSetupStateJson(char* buf, size_t buflen);

/**
 * Setup-Wizard Apply: RS485-Kommando lokal ausfuehren.
 * bodyJson: {"cmd":"SETMINPWM","params":"40"} (params optional)
 * outJson: {"ok":true,"v":"..."} oder {"ok":false,"err":"..."}
 */
void webAppSetupApplyJson(const char* bodyJson, char* outJson, size_t outLen);

/**
 * SoftAP-JOG: dir 0/1/2 (aktiviert Service-JOG).
 * pastLimits=true: Setup-Modus (auch unter Amin/ueber Amax).
 */
void webAppSetupJogEx(uint8_t dir, bool pastLimits, char* outJson, size_t outLen);

/** Setup-Wizard JOG (pastLimits=true). */
void webAppSetupJog(uint8_t dir, char* outJson, size_t outLen);

/** Neustart anfordern (bestehender g_restartRequested-Pfad). */
void webAppRequestReboot();

/** Werksreset: FRAM + NVS Preferences, danach Neustart. */
void webAppFactoryReset();

/** SoftAP Idle-Shutdown: SETWLAN-Wunschflag auf aus setzen. */
void webAppOnSoftApAutoOff();
