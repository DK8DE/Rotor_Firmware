#pragma once

#include <Arduino.h>

void webUiBegin(const char* ssid, const char* password);
void webUiEnd();
void webUiLoop();
bool webUiIsActive();
