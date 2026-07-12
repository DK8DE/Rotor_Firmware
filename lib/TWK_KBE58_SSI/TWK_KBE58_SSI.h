/*
  TWK_KBE58_SSI.h

  Arduino library for reading a TWK KBE 58 - K 4096 G K E06 SSI absolute
  encoder inside a Rohde & Schwarz RD130 rotor.

  Copyright (C) 2026 Joerg Koerner DK8DE

  This library is free software: you can redistribute it and/or modify it under
  the terms of the GNU General Public License as published by the Free Software
  Foundation, either version 3 of the License, or any later version.

  This library is distributed in the hope that it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
  FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
*/

#ifndef TWK_KBE58_SSI_H
#define TWK_KBE58_SSI_H

#include <Arduino.h>

class SPIClass;

class TWK_KBE58_SSI
{
public:
  enum class Mode
  {
    BitBang,
    ArduinoSPI,
    ESP32PreciseSPI
  };

  struct Reading
  {
    int dataIdleLevel;
    uint32_t rawValue;
    uint32_t trailingBits;
    uint32_t grayValue;
    uint32_t position;
    uint32_t stepsPerRevolution;
    float angleDeg;
    float angleDegRounded;
    bool valid;
    uint32_t readCounter;
    uint32_t errorCounter;
  };

  TWK_KBE58_SSI(uint8_t clockPin, uint8_t dataPin);
  TWK_KBE58_SSI(uint8_t clockPin, uint8_t dataPin, uint8_t usefulBits, uint8_t totalClocks, uint8_t trailingBits);
  ~TWK_KBE58_SSI();

  void begin();
  void beginBitBang();

  void beginSPI(SPIClass &spi, uint8_t sckPin, uint8_t misoPin, uint32_t frequencyHz = 100000);

#if defined(ESP32)
  bool beginESP32PreciseSPI(uint8_t sckPin, uint8_t misoPin, uint32_t frequencyHz = 100000);
  bool startBackgroundRead(uint32_t intervalMs = 20, BaseType_t core = 1);
  void stopBackgroundRead();
  Reading getLastReading();
  bool hasNewReading();
  uint32_t getReadCounter() const;
  uint32_t getErrorCounter() const;
  void setRawBitShift(int8_t shift);
#endif

  void setHalfPeriodUs(uint32_t halfPeriodUs);
  void setFramePauseUs(uint32_t framePauseUs);
  void setClockIdleHigh(bool clockIdleHigh);
  void setInvertData(bool invertData);

  void setSpiFrequency(uint32_t frequencyHz);
  void setSpiMode(uint8_t spiMode);
  void setSpiTransferBits(uint8_t transferBits);
  void setSpiRightShift(uint8_t rightShift);
  void setSpiDummySsPin(int8_t ssPin);
  void setStartupDiscardCount(uint8_t discardCount);
  void configureZeroPin(int8_t pin, uint32_t pulseMs = 200);
  void setZero();
  void update();
  bool isZeroPulseActive() const;

  Mode mode() const;

  Reading read();

  uint32_t readRaw();
  uint32_t grayToBinary(uint32_t grayValue) const;

  uint8_t usefulBits() const;
  uint8_t totalClocks() const;
  uint8_t trailingBits() const;

  uint32_t stepsPerRevolution() const;

private:
  uint8_t _clockPin;
  uint8_t _dataPin;

  uint8_t _usefulBits;
  uint8_t _totalClocks;
  uint8_t _trailingBits;

  bool _clockIdleHigh;
  bool _invertData;

  uint32_t _halfPeriodUs;
  uint32_t _framePauseUs;

  Mode _mode;

  SPIClass *_spi;
  uint32_t _spiFrequencyHz;
  uint8_t _spiMode;
  uint8_t _spiTransferBits;
  uint8_t _spiRightShift;
  int8_t _spiDummySsPin;

  bool _lastReadValid;
  uint32_t _readCounter;
  uint32_t _errorCounter;
  uint8_t _startupDiscardRemaining;
  int8_t _zeroPin;
  uint32_t _zeroPulseMs;
  bool _zeroPulseActive;
  uint32_t _zeroPulseStartMs;

#if defined(ESP32)
  void *_spiHandle;
  int _spiHost;
  int8_t _rawBitShift;
  bool _esp32SpiInitialized;

  TaskHandle_t _backgroundTaskHandle;
  volatile bool _backgroundTaskRunning;
  uint32_t _backgroundIntervalMs;
  Reading _lastReading;
  volatile bool _newReading;
  portMUX_TYPE _mux;

  static void _backgroundTaskThunk(void *parameter);
  void _backgroundTask();
  void _releaseESP32Spi();
#endif

  uint32_t readRawBitBang();
  uint32_t readRawArduinoSPI();

#if defined(ESP32)
  uint32_t readRawESP32PreciseSPI();
#endif

  int _sampleDataIdleLevel() const;
  uint32_t _applyRawShiftAndMask(uint32_t rawValue) const;
  void _fillReading(Reading &reading, uint32_t rawValue, bool valid);
  void _runSilentWarmup(uint8_t count);
  void _armStartupDiscard(uint8_t count);
  uint32_t _totalClocksMask() const;
  uint32_t _usefulBitMask() const;
  uint32_t _roundAngleTo01(float angleDeg) const;
};

#endif
