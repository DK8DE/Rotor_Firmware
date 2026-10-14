#pragma once

#include <Arduino.h>
#include <Wire.h>

// ------------------------------------------------------------
// FM24CL16B-G — 16-Kbit (2048 x 8) I2C F-RAM
// ------------------------------------------------------------
// Slave-Adresse: 1010 | page[2:0] | R/W
// 11-Bit-Adresse: page (3) + word (8). Kein Write-Delay noetig.
// ------------------------------------------------------------
class Fm24cl16b {
public:
  static constexpr uint16_t kSizeBytes = 2048;
  static constexpr uint8_t  kBaseAddr  = 0x50; // 0b1010000 (page 0, 7-bit)

  void begin(TwoWire& wire = Wire, uint32_t clockHz = 400000);

  // true wenn ein ACK auf Page-0-Adresse kommt
  bool probe();

  bool read(uint16_t addr, uint8_t* buf, size_t len);
  bool write(uint16_t addr, const uint8_t* buf, size_t len);

  bool readU8(uint16_t addr, uint8_t& out);
  bool writeU8(uint16_t addr, uint8_t v);

  bool ok() const { return _ok; }

private:
  TwoWire* _wire = nullptr;
  bool     _ok = false;

  static uint8_t slaveAddrFor(uint16_t addr) {
    // page = addr[10:8], in Bits 3..1 der 7-Bit-Adresse
    return (uint8_t)(kBaseAddr | ((addr >> 8) & 0x07));
  }

  static uint8_t wordAddr(uint16_t addr) {
    return (uint8_t)(addr & 0xFF);
  }
};
