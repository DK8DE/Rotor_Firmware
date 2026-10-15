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
 * Felder: angle,tempA,tempM,fw,homed,pwm,uptime,heap,ssid,ip,uid
 */
void webAppFillLiveJson(char* buf, size_t buflen, const char* ssid, const char* ip);

/** Neustart anfordern (bestehender g_restartRequested-Pfad). */
void webAppRequestReboot();

/** Werksreset: FRAM + NVS Preferences, danach Neustart. */
void webAppFactoryReset();

/** SoftAP Idle-Shutdown: SETWLAN-Wunschflag auf aus setzen. */
void webAppOnSoftApAutoOff();
