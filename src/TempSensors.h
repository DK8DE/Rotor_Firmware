#pragma once

#include <Arduino.h>

// DS18B20 (OneWire) Bibliothek (wie von Joerg getestet)
#include <7semi_DS18B20.h>

// ============================================================================
// TempSensors
// ============================================================================
// - Liest bis zu 2 DS18B20 Sensoren ein:
//
//   Physische Reihenfolge (wie die OneWire-Library sie "findet"):
//   * Device 0: erster gefundener Sensor
//   * Device 1: zweiter gefundener Sensor
//
//   Logische Zuordnung (wie sie im RS485-Protokoll genutzt wird):
//   * Umgebung: GETTEMPA
//   * Motor   : GETTEMPM  (optional deaktivierbar)
//
// - Problem: Je nach Sensor/Bus/Timing kann die physische Reihenfolge variieren.
//   Deshalb gibt es einen "Swap"-Schalter, um die logische Zuordnung zu tauschen.
//   -> Wenn Swap aktiv ist, werden Umgebung/Motor vertauscht.
//
// - Wichtig: Alle Limits/Warnings (z.B. tempWarnAmbientC/tempWarnMotorC) arbeiten
//   immer auf der *logischen* Zuordnung. Durch Swap landen die Limits damit am
//   richtigen Sensor, auch wenn die physische Reihenfolge anders ist.
//
// - Die Werte werden zyklisch in einem kleinen Task aktualisiert.
// - Wenn der Motor-Sensor deaktiviert ist, liefert GETTEMPM immer 0.
// - Wenn kein DS18B20 gefunden wird, liefern beide 0.
// ============================================================================
class TempSensors {
public:
  TempSensors() = default;

  // pin: OneWire Data Pin
  // enableMotorSensor: true => logischer Motor-Sensor ist aktiv (GETTEMPM)
  // intervalMs: Messintervall
  bool begin(uint8_t pin, bool enableMotorSensor, uint32_t intervalMs);

  // Motor-Sensor (logisch) aktivieren/deaktivieren
  void setMotorSensorEnabled(bool on);
  bool isMotorSensorEnabled() const { return _motorEnabled; }

  // Logische Zuordnung tauschen:
  // - false: Umgebung=Device0, Motor=Device1
  // - true : Umgebung=Device1, Motor=Device0
  void setSwapTemp(bool on);
  bool isSwapTemp() const { return _swapTemp; }

  uint8_t getDeviceCount() const { return _devCount; }

  // Temperaturen in Grad Celsius
  // - Wenn nicht verfuegbar -> 0
  float getAmbientC() const;
  float getMotorC() const;

  // Temperatur in 0,01 Grad (scaled100)
  int32_t getAmbientScaled100() const;
  int32_t getMotorScaled100() const;

  // Sensor-Praesenz bezogen auf die logische Zuordnung
  bool hasAmbient() const;
  bool hasMotor() const;

private:
  static void taskThunk(void* arg);
  void taskLoop();

  void doOneRead();

  // Mapping: welcher physische Index gehoert logisch zu Umgebung/Motor?
  uint8_t mapAmbientIndex() const;
  uint8_t mapMotorIndex() const;

private:
  // Bibliothek-Objekt (dynamisch, weil der Pin im Konstruktor sitzt)
  DS18B20_7semi* _sensor = nullptr;
  uint8_t _pin = 2;

  // Gefundene Devices (physisch)
  uint8_t _devCount = 0;
  uint8_t _addr0[8] = {0};
  uint8_t _addr1[8] = {0};

  bool _dev0Present = false;
  bool _dev1Present = false;

  // Logik-Schalter
  bool _motorEnabled = false;
  bool _swapTemp = false;

  uint32_t _intervalMs = 1000;

  // Cache (physische Devices) in 0,01C
  volatile int32_t _dev0Scaled100 = 0;
  volatile int32_t _dev1Scaled100 = 0;

  // Task
  TaskHandle_t _task = nullptr;
};
