/*************************************************************************************************** 
//  7semi_DS18B20.cpp - DS18B20 Temperature Sensor Library Implementation
//  Written for the 7semi sensor platform
//
//  Implementation uses OneWire.h to perform ROM & memory functions,
//  supports multi-device, alarms, scratchpad/EEPROM operations, parasite power handling.
//
 // Author: 7semi
//  License: MIT
*****************************************************************************************************/

#include "7semi_DS18B20.h"

/**
// Constructor: store pins and init OneWire instance
**/
DS18B20_7semi::DS18B20_7semi(uint8_t dataPin, int8_t strongPullupPin)
  : oneWire(dataPin) {
  _dataPin = dataPin;
  _strongPullupPin = strongPullupPin;
  _devices = 0;
}

/**
// begin(): reset search and scan devices
**/
bool DS18B20_7semi::begin() {
  oneWire.reset_search();
  _devices = searchDevices();
  return (_devices > 0);
}

/**
// searchDevices(): search and store addresses up to DS18B20_MAX_DEVICES
**/
uint8_t DS18B20_7semi::searchDevices() {
  oneWire.reset_search();
  _devices = 0;
  uint8_t addr[8];
  while (oneWire.search(addr)) {
    if (_devices < DS18B20_MAX_DEVICES) {
      memcpy(_addresses[_devices], addr, 8);
      // CRC check: OneWire::crc8 helper used here
      if (OneWire::crc8(addr, 7) != addr[7]) {
        // CRC failed -> skip storing this device
      } else {
        _devices++;
      }
    } else {
      break;
    }
  }
  oneWire.reset_search();
  return _devices;
}

/**
// getAddress(): copy stored address by index
**/
bool DS18B20_7semi::getAddress(uint8_t index, uint8_t addr[8]) {
  if (index >= _devices) return false;
  memcpy(addr, _addresses[index], 8);
  return true;
}

/**
// readTemperature(): start conversion, wait appropriate time, read scratchpad and compute °C.
**/
float DS18B20_7semi::readTemperature(const uint8_t addr[8]) {
  uint8_t scratch[9];
  // Start conversion by selecting device and issuing Convert T (0x44)
  oneWire.reset();
  oneWire.select(addr);
  oneWire.write(0x44, 0);  // don't use parasite power flag here (we handle strong pull-up manually)

  // Wait conversion time according to resolution
  uint8_t res = getResolution(addr);
  if (res < 9 || res > 12) res = 12;  // default
  // If parasite power expected, enable strong pullup (if pin provided)
  bool external = true;
  bool extKnown = readPowerSupply((uint8_t *)addr, external);  // try reading power supply; if fails assume external
  if (extKnown && !external && _strongPullupPin >= 0) {
    _strongPullup(true);  // enable MOSFET/strong pullup
  }

  delay(_conversionDelayMs(res));

  if (extKnown && !external && _strongPullupPin >= 0) {
    _strongPullup(false);  // disable strong pullup after conversion
  }

  // Read scratchpad
  if (!readScratchpad(addr, scratch)) return NAN;

  int16_t raw = (int16_t)((scratch[1] << 8) | scratch[0]);
  // default scaling 1/16 (12-bit). If lower resolution, lower bits are zeroed already.
  float tempC = raw / 16.0f;
  return tempC;
}

/**
// readRawTemperature(): read raw 16-bit signed temp register
**/
bool DS18B20_7semi::readRawTemperature(const uint8_t addr[8], int16_t &raw) {
  uint8_t scratch[9];
  if (!readScratchpad(addr, scratch)) return false;
  raw = (int16_t)((scratch[1] << 8) | scratch[0]);
  return true;
}

/**
// setResolution(): set R1/R0 bits in config byte (9..12). Optionally persist to EEPROM.
**/
bool DS18B20_7semi::setResolution(const uint8_t addr[8], uint8_t resolution, bool persistToEeprom) {
  if (resolution < 9 || resolution > 12) return false;
  // read current scratchpad to keep TH/TL
  uint8_t sp[9];
  if (!readScratchpad(addr, sp)) return false;
  int8_t th = (int8_t)sp[2];
  int8_t tl = (int8_t)sp[3];
  uint8_t config = 0;
  // config byte: R1 R0 at bits 6:5 (datasheet representation — but in DS18B20 it's bits 6 and 5)
  // In datasheet the config is: 0 R1 R0 1 1 1 1 1 -> bits 5 and 6 are R0 and R1 respectively (we'll set accordingly)
  uint8_t rbits = 0;
  switch (resolution) {
    case 9: rbits = 0x00; break;   // R1=0 R0=0
    case 10: rbits = 0x20; break;  // R1=0 R0=1 -> bit5 = 1 (0x20)
    case 11: rbits = 0x40; break;  // R1=1 R0=0 -> bit6 = 1 (0x40)
    case 12: rbits = 0x60; break;  // R1=1 R0=1 -> bit6 & bit5
  }
  config = rbits | 0x1F;  // lower bits read as 1 per datasheet; but we will keep the rbits for writing
  // Write scratchpad TH, TL, config (Write Scratchpad 0x4E)
  if (!writeScratchpad(addr, th, tl, config)) return false;
  if (persistToEeprom) {
    if (!copyScratchpad(addr)) return false;
  }
  return true;
}

/**
// getResolution(): parse scratchpad config byte and convert to resolution 9..12
**/
uint8_t DS18B20_7semi::getResolution(const uint8_t addr[8]) {
  uint8_t sp[9];
  if (!readScratchpad(addr, sp)) return 0;
  uint8_t cfg = sp[4];
  uint8_t r = (cfg & 0x60);  // bits 6 and 5
  if (r == 0x00) return 9;
  if (r == 0x20) return 10;
  if (r == 0x40) return 11;
  if (r == 0x60) return 12;
  return 12;
}

/**
// setAlarms(): write TH and TL into scratchpad; optionally persist
**/
bool DS18B20_7semi::setAlarms(const uint8_t addr[8], int8_t th, int8_t tl, bool persistToEeprom) {
  // read config
  uint8_t sp[9];
  if (!readScratchpad(addr, sp)) return false;
  uint8_t cfg = sp[4];
  if (!writeScratchpad(addr, th, tl, cfg)) return false;
  if (persistToEeprom) {
    return copyScratchpad(addr);
  }
  return true;
}

/**
// getAlarms(): read TH/TL from scratchpad
**/
bool DS18B20_7semi::getAlarms(const uint8_t addr[8], int8_t &th, int8_t &tl) {
  uint8_t sp[9];
  if (!readScratchpad(addr, sp)) return false;
  th = (int8_t)sp[2];
  tl = (int8_t)sp[3];
  return true;
}

/**
// alarmSearch(): perform Alarm Search and return first found device address
**/
bool DS18B20_7semi::alarmSearch(uint8_t foundAddr[8]) {
  oneWire.reset_search();
  if (!oneWire.search(foundAddr)) return false;
  // The OneWire library does not provide a direct alarm-search API; use command 0xEC via search.
  // Simpler approach: use ow.reset() + issue Alarm Search command sequence via search with alarm flag:
  // Unfortunately OneWire library supports search with alarm bit via search(address,bool alarmSearch)
  oneWire.reset_search();
  if (!oneWire.search(foundAddr, true)) return false;  // alarmSearch = true (OneWire extension)
  if (OneWire::crc8(foundAddr, 7) != foundAddr[7]) return false;
  return true;
}

/**
// isParasitePower(): issue Read Power Supply (0xB4) on device; returns true for parasite (0) else true external
**/
bool DS18B20_7semi::isParasitePower(const uint8_t addr[8]) {
  bool external = true;
  if (!readPowerSupply((uint8_t *)addr, external)) return false;  // if can't determine, return false
  return !external;
}

/**
// readScratchpad(): read scratchpad bytes and verify CRC
**/
bool DS18B20_7semi::readScratchpad(const uint8_t addr[8], uint8_t buffer[9]) {
  oneWire.reset();
  oneWire.select(addr);
  oneWire.write(0xBE);  // Read Scratchpad
  for (uint8_t i = 0; i < 9; i++) buffer[i] = oneWire.read();
  uint8_t crcCalculated = OneWire::crc8(buffer, 8);
  if (crcCalculated != buffer[8]) return false;
  return true;
}

/**
// writeScratchpad(): write TH,Tl,config into scratchpad (3 bytes)
**/
bool DS18B20_7semi::writeScratchpad(const uint8_t addr[8], int8_t th, int8_t tl, uint8_t config) {
  oneWire.reset();
  oneWire.select(addr);
  oneWire.write(0x4E);  // Write Scratchpad
  oneWire.write((uint8_t)th);
  oneWire.write((uint8_t)tl);
  oneWire.write(config);
  // no immediate CRC check possible for scratchpad write; read back to confirm
  uint8_t sp[9];
  if (!readScratchpad(addr, sp)) return false;
  // Verify match of bytes 2-4 in scratchpad
  if (sp[2] != (uint8_t)th || sp[3] != (uint8_t)tl || sp[4] != (uint8_t)config) return false;
  return true;
}

/**
// copyScratchpad(): copy scratchpad to EEPROM (command 0x48). If parasite, master must provide strong pull-up.
**/
bool DS18B20_7semi::copyScratchpad(const uint8_t addr[8]) {
  oneWire.reset();
  oneWire.select(addr);
  oneWire.write(0x48);  // Copy Scratchpad
  // wait up to 10ms for copy
  // If parasite-powered, enable strong pullup
  bool external = true;
  readPowerSupply((uint8_t *)addr, external);
  if (!external && _strongPullupPin >= 0) _strongPullup(true);
  delay(11);
  if (!external && _strongPullupPin >= 0) _strongPullup(false);
  // Optionally read scratchpad to confirm copy (recallE2 does that)
  return true;
}

/**
// recallE2(): recall EEPROM into scratchpad (0xB8)
**/
bool DS18B20_7semi::recallE2(const uint8_t addr[8]) {
  oneWire.reset();
  oneWire.select(addr);
  oneWire.write(0xB8);  // Recall E2
  // After recall, read scratchpad
  uint8_t sp[9];
  if (!readScratchpad(addr, sp)) return false;
  return true;
}

/**
// readPowerSupply(): issue Read Power Supply (0xB4). returns externalPowered in parameter.
// If the device returns 1 => external power, 0 => parasite.
**/
bool DS18B20_7semi::readPowerSupply(const uint8_t addr[8], bool &externalPowered) {
  oneWire.reset();
  oneWire.select(addr);
  oneWire.write(0xB4);         // Read Power Supply
  uint8_t v = oneWire.read();  // read one bit/time slot
  // OneWire.read returns a byte but the device outputs 0 or 1 as single bit clocked
  externalPowered = (v == 1);
  return true;
}

/**
// getROM64(): pack addr[8] (LSB first) into uint64_t
**/
uint64_t DS18B20_7semi::getROM64(const uint8_t addr[8]) {
  uint64_t v = 0;
  for (int i = 7; i >= 0; --i) {
    v = (v << 8) | addr[i];
  }
  return v;
}

/**
// crc8(): helper (wrap OneWire::crc8)
**/
uint8_t DS18B20_7semi::crc8(const uint8_t *data, uint8_t len) {
  return OneWire::crc8(data, len);
}

/**
// _strongPullup(): control strong pullup MOSFET pin (active HIGH).
**/
void DS18B20_7semi::_strongPullup(bool on) {
  if (_strongPullupPin < 0) return;
  pinMode(_strongPullupPin, OUTPUT);
  digitalWrite(_strongPullupPin, on ? HIGH : LOW);
}

/**
// _conversionDelayMs(): return conversion delay (ms) for resolution
**/
uint16_t DS18B20_7semi::_conversionDelayMs(uint8_t resolution) {
  switch (resolution) {
    case 9: return 94;
    case 10: return 188;
    case 11: return 375;
    case 12:
    default: return 750;
  }
}
