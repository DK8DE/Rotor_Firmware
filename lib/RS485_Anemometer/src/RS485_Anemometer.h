#pragma once

#include <Arduino.h>

// ============================================================
// RS485_Anemometer
// ------------------------------------------------------------
// Modbus-RTU (RS485) anemometer library for ESP32.
// - Wind speed (m/s)
// - Wind level (1..12)
// - Wind direction in degrees (0..359.9)
// - Wind direction as text (N, NNE, NE, ENE, E / N, NNO, NO, ONO, O ...)
//
// Note:
// - The library itself does NOT print to Serial.
// - The example sketch is responsible for Serial output.
// ============================================================

class RS485_Anemometer
{
public:
  RS485_Anemometer();

  // ----------------------------------------------------------
  // Language for cardinal directions
  // ----------------------------------------------------------
  // DE: German (O instead of E)
  // EN: English (E for East)
  enum class DirLanguage : uint8_t
  {
    DE = 0,
    EN = 1
  };

  // ----------------------------------------------------------
  // Initialisierung
  // ----------------------------------------------------------
  // port  : HardwareSerial (z.B. Serial2)
  // rxPin : RX-Pin des ESP32
  // txPin : ESP32 TX pin
  // deRePin: DE/RE pin of the RS485 transceiver (HIGH=transmit, LOW=receive)
  // address: Modbus address of the sensor (typically 0x01)
  // baud   : default 9600
  void begin(HardwareSerial& port,
             int rxPin,
             int txPin,
             int deRePin,
             uint8_t address = 0x01,
             uint32_t baud = 9600);

  // ----------------------------------------------------------
  // Offsets (werden bei update() auf die Rohwerte addiert)
  // ----------------------------------------------------------
  void setAngleOffsetDeg(float offsetDeg);
  void setSpeedOffsetMps(float offsetMps); 

  // ----------------------------------------------------------
  // Wind lesen (schnelles Timing)
  // ----------------------------------------------------------
  // update() reads the sensor and stores the latest reading internally.
  // Returns: true if a valid reading was received.
  bool update();

  // Komfort-Getter (nutzen den letzten Messwert)
  float getWindSpeedMps() const;      // z.B. 3.2
  int   getWindLevel() const;         // 1..12
  float getWindAngleDeg() const;      // z.B. 274.5
  uint8_t getWindDirCode() const;     // 0..15

  // Direction als Text
  // Default: German (O statt E)
  const char* getWindDirText(DirLanguage lang = DirLanguage::DE) const;

private:
  // Letzter Messwert (nach update())
  struct Reading
  {
    float    windSpeed_mps;   // Wind speed in m/s
    uint16_t windLevel;       // Level 1..12 (as reported by the sensor)
    float    windAngle_deg;   // Wind direction in degrees
    uint8_t  windDirCode;     // 0..15 (16-point rose)
  };

  // ----------------------------------------------------------
  // Modbus / RS485 intern
  // ----------------------------------------------------------
  HardwareSerial* _ser;
  int _deRePin;
  uint8_t _addr;

  float _angleOffsetDeg;
  float _speedOffsetMps;
  uint32_t _lastFrameMs;

  // Letzter Messwert
  Reading _last;

  // Timing-Profile
  static const uint16_t WIND_INTERFRAME_DELAY_MS = 5;
  static const uint16_t WIND_PRE_TX_US  = 80;
  static const uint16_t WIND_POST_TX_US = 120;
  static const uint16_t WIND_TIMEOUT_MS = 160;
  // Low-Level Helper
  static uint16_t modbusCRC16(const uint8_t* data, size_t len);
  static uint16_t be16(uint8_t hi, uint8_t lo);

  // Statische Helfer (intern)
  static float normalizeDeg(float deg);
  static uint8_t dirCodeFromAngle16(float angleDeg);
  static const char* windDir16TextDE(uint8_t code);
  static const char* windDir16TextEN(uint8_t code);

  void flushInput();
  void interFrameDelay(uint16_t interMs);
  void markFrame();
  void txEnable(uint16_t preUs);
  void txDisable(uint16_t postUs);
  bool readBytes(uint8_t* buf, size_t len, uint16_t timeoutMs);

  // Modbus Funktionen
  bool readHoldingRegisters03(uint16_t startReg,
                              uint16_t count,
                              uint16_t* outRegs,
                              uint16_t interMs,
                              uint16_t preUs,
                              uint16_t postUs,
                              uint16_t timeoutMs);

  // Wind lesen (fast) in out
  bool readWindFast(Reading& out);
};
