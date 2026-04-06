/*************************************************************************************************** 
//  7semi_DS18B20.h - DS18B20 Temperature Sensor Library
//  Written for the 7semi sensor platform
//
//  Full-feature DS18B20 library using OneWire only.
//  Supports multi-device, alarms, EEPROM, parasite power strong pull-up, CRC checks.
// 
//  Author: 7semi
//  License: MIT
*****************************************************************************************************/

#ifndef _7SEMI_DS18B20_H_
#define _7SEMI_DS18B20_H_

#include <Arduino.h>
#include <OneWire.h>

#define DS18B20_MAX_DEVICES 16

class DS18B20_7semi {
public:
  // Constructor: dataPin is the 1-Wire bus pin. strongPullupPin optional for parasite power.
  DS18B20_7semi(uint8_t dataPin, int8_t strongPullupPin = -1);

  // begin(): Initialize OneWire and discover devices (returns true if at least one found).
  bool begin();

  // searchDevices(): scans bus and stores up to DS18B20_MAX_DEVICES addresses. Returns count.
  uint8_t searchDevices();

  // getAddress(): copy address of device 'index' (0-based) into addr[8]. Returns true if valid.
  bool getAddress(uint8_t index, uint8_t addr[8]);

  // readTemperature(): read temperature (°C) from device address; uses device's configured resolution.
  float readTemperature(const uint8_t addr[8]);

  // readRawTemperature(): read raw 16-bit temperature register (signed).
  bool readRawTemperature(const uint8_t addr[8], int16_t &raw);

  // setResolution(): set resolution (9..12) for device, writes to scratchpad and optionally copies to EEPROM.
  bool setResolution(const uint8_t addr[8], uint8_t resolution, bool persistToEeprom = false);

  // getResolution(): return resolution (9..12) from scratchpad (or 0 on error).
  uint8_t getResolution(const uint8_t addr[8]);

  // setAlarms(): set TH/TL (int8) in scratchpad; optionally persist to EEPROM.
  bool setAlarms(const uint8_t addr[8], int8_t th, int8_t tl, bool persistToEeprom = false);

  // getAlarms(): read TH/TL from scratchpad (returns true if successful).
  bool getAlarms(const uint8_t addr[8], int8_t &th, int8_t &tl);

  // alarmSearch(): finds next device in alarm state and copies its address; returns true if found.
  bool alarmSearch(uint8_t foundAddr[8]);

  // isParasitePower(): returns true if device reports parasite power mode.
  bool isParasitePower(const uint8_t addr[8]);

  // readScratchpad(): read 9 bytes scratchpad into buffer[9]; returns true if CRC OK.
  bool readScratchpad(const uint8_t addr[8], uint8_t buffer[9]);

  // writeScratchpad(): write TH, TL, config to scratchpad (3 bytes).
  bool writeScratchpad(const uint8_t addr[8], int8_t th, int8_t tl, uint8_t config);

  // copyScratchpad(): copy scratchpad to EEPROM (TH/TL/config). Use strong pull-up or VDD.
  bool copyScratchpad(const uint8_t addr[8]);

  // recallE2(): recall EEPROM TH/TL/config into scratchpad.
  bool recallE2(const uint8_t addr[8]);

  // readPowerSupply(): issues Read Power Supply command; returns true for external, false for parasite.
  bool readPowerSupply(const uint8_t addr[8], bool &externalPowered);

  // getROM64(): convert address[8] to uint64_t (LSB first).
  uint64_t getROM64(const uint8_t addr[8]);

  // crc8(): helper to compute 1-Wire CRC8
  static uint8_t crc8(const uint8_t *data, uint8_t len);

private:
  OneWire oneWire;
  uint8_t _devices;
  uint8_t _addresses[DS18B20_MAX_DEVICES][8];
  int8_t _strongPullupPin;
  uint8_t _dataPin;

  // internal helpers
  void _strongPullup(bool on);
  uint16_t _conversionDelayMs(uint8_t resolution);
};

#endif
