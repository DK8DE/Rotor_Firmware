#pragma once

#include <Arduino.h>

/** SoftAP starten; PSK kommt aus NVS (Default Rotorconfig). */
void webUiBegin(const char* ssid);

/** Aktuelles SoftAP-Passwort (NVS/Default), fuer Debug-Log. */
const char* webUiSoftApPass();

/** Web-Login + SoftAP-PSK auf Werkswerte (NVS loeschen). */
void webUiClearCredentials();

void webUiEnd();
void webUiLoop();
bool webUiIsActive();
