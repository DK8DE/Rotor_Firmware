/*
  TWK_KBE58_SSI.cpp

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

#include "TWK_KBE58_SSI.h"

#include <SPI.h>

#if defined(ESP32)
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

namespace
{
  void initCommonDefaults(
    uint8_t &usefulBits,
    uint8_t &totalClocks,
    uint8_t &trailingBits,
    bool &clockIdleHigh,
    bool &invertData,
    uint32_t &halfPeriodUs,
    uint32_t &framePauseUs)
  {
    usefulBits = 12;
    totalClocks = 13;
    trailingBits = 1;
    clockIdleHigh = true;
    invertData = false;
    halfPeriodUs = 5;
    framePauseUs = 80;
  }
}

TWK_KBE58_SSI::TWK_KBE58_SSI(uint8_t clockPin, uint8_t dataPin)
{
  _clockPin = clockPin;
  _dataPin = dataPin;

  initCommonDefaults(_usefulBits, _totalClocks, _trailingBits, _clockIdleHigh, _invertData, _halfPeriodUs, _framePauseUs);

  _mode = Mode::BitBang;
  _spi = nullptr;
  _spiFrequencyHz = 100000;
  _spiMode = SPI_MODE2;
  _spiTransferBits = 16;
  _spiRightShift = 3;
  _spiDummySsPin = -1;
  _lastReadValid = true;
  _readCounter = 0;
  _errorCounter = 0;
  _startupDiscardRemaining = 0;
  _zeroPin = -1;
  _zeroPulseMs = 200;
  _zeroPulseActive = false;
  _zeroPulseStartMs = 0;

#if defined(ESP32)
  _spiHandle = nullptr;
  _spiHost = SPI2_HOST;
  _rawBitShift = 0;
  _esp32SpiInitialized = false;
  _backgroundTaskHandle = nullptr;
  _backgroundTaskRunning = false;
  _backgroundIntervalMs = 20;
  _newReading = false;
  _mux = portMUX_INITIALIZER_UNLOCKED;
  _lastReading = {};
#endif
}

TWK_KBE58_SSI::TWK_KBE58_SSI(uint8_t clockPin, uint8_t dataPin, uint8_t usefulBits, uint8_t totalClocks, uint8_t trailingBits)
{
  _clockPin = clockPin;
  _dataPin = dataPin;

  _usefulBits = usefulBits;
  _totalClocks = totalClocks;
  _trailingBits = trailingBits;
  _clockIdleHigh = true;
  _invertData = false;
  _halfPeriodUs = 5;
  _framePauseUs = 80;

  _mode = Mode::BitBang;
  _spi = nullptr;
  _spiFrequencyHz = 100000;
  _spiMode = SPI_MODE2;
  _spiTransferBits = 16;
  _spiRightShift = 3;
  _spiDummySsPin = -1;
  _lastReadValid = true;
  _readCounter = 0;
  _errorCounter = 0;
  _startupDiscardRemaining = 0;
  _zeroPin = -1;
  _zeroPulseMs = 200;
  _zeroPulseActive = false;
  _zeroPulseStartMs = 0;

#if defined(ESP32)
  _spiHandle = nullptr;
  _spiHost = SPI2_HOST;
  _rawBitShift = 0;
  _esp32SpiInitialized = false;
  _backgroundTaskHandle = nullptr;
  _backgroundTaskRunning = false;
  _backgroundIntervalMs = 20;
  _newReading = false;
  _mux = portMUX_INITIALIZER_UNLOCKED;
  _lastReading = {};
#endif
}

TWK_KBE58_SSI::~TWK_KBE58_SSI()
{
#if defined(ESP32)
  stopBackgroundRead();
  _releaseESP32Spi();
#endif
}

void TWK_KBE58_SSI::begin()
{
  beginBitBang();
}

void TWK_KBE58_SSI::beginBitBang()
{
#if defined(ESP32)
  stopBackgroundRead();
  _releaseESP32Spi();
#endif

  _mode = Mode::BitBang;
  _spi = nullptr;

  pinMode(_clockPin, OUTPUT);
  pinMode(_dataPin, INPUT);

  if (_clockIdleHigh)
  {
    digitalWrite(_clockPin, HIGH);
  }
  else
  {
    digitalWrite(_clockPin, LOW);
  }

#if defined(ESP32)
  _runSilentWarmup(1);
  _armStartupDiscard(1);
#endif
}

void TWK_KBE58_SSI::beginSPI(SPIClass &spi, uint8_t sckPin, uint8_t misoPin, uint32_t frequencyHz)
{
#if defined(ESP32)
  stopBackgroundRead();
  _releaseESP32Spi();
#endif

  _mode = Mode::ArduinoSPI;
  _spi = &spi;
  _clockPin = sckPin;
  _dataPin = misoPin;
  _spiFrequencyHz = frequencyHz;

  pinMode(_dataPin, INPUT);

#if defined(ESP32)
  _spi->begin(_clockPin, _dataPin, -1, -1);
#else
  _spi->begin();

  pinMode(_clockPin, OUTPUT);
  pinMode(_dataPin, INPUT);

  int8_t ssPin = _spiDummySsPin;

#if defined(SS)
  if (ssPin < 0)
  {
    ssPin = SS;
  }
#endif

  if (ssPin >= 0)
  {
    pinMode((uint8_t)ssPin, OUTPUT);
    digitalWrite((uint8_t)ssPin, HIGH);
  }

  _runSilentWarmup(2);
  _armStartupDiscard(2);
#endif

#if defined(ESP32)
  _runSilentWarmup(1);
  _armStartupDiscard(1);
#endif
}

#if defined(ESP32)
bool TWK_KBE58_SSI::beginESP32PreciseSPI(uint8_t sckPin, uint8_t misoPin, uint32_t frequencyHz)
{
  stopBackgroundRead();
  _releaseESP32Spi();

  _mode = Mode::ESP32PreciseSPI;
  _clockPin = sckPin;
  _dataPin = misoPin;
  _spiFrequencyHz = frequencyHz;
  _spi = nullptr;

  pinMode(_dataPin, INPUT);

  spi_bus_config_t busConfig = {};
  busConfig.miso_io_num = (int)_dataPin;
  busConfig.mosi_io_num = -1;
  busConfig.sclk_io_num = (int)_clockPin;
  busConfig.quadwp_io_num = -1;
  busConfig.quadhd_io_num = -1;
  busConfig.max_transfer_sz = 4;

  spi_device_interface_config_t deviceConfig = {};
  deviceConfig.clock_speed_hz = _spiFrequencyHz;
  deviceConfig.mode = _spiMode;
  deviceConfig.spics_io_num = -1;
  deviceConfig.queue_size = 1;
  deviceConfig.flags = SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_NO_DUMMY;

  esp_err_t busResult = spi_bus_initialize((spi_host_device_t)_spiHost, &busConfig, SPI_DMA_DISABLED);

  if (busResult != ESP_OK)
  {
    return false;
  }

  esp_err_t deviceResult = spi_bus_add_device((spi_host_device_t)_spiHost, &deviceConfig, (spi_device_handle_t *)&_spiHandle);

  if (deviceResult != ESP_OK)
  {
    spi_bus_free((spi_host_device_t)_spiHost);
    _spiHandle = nullptr;
    return false;
  }

  _esp32SpiInitialized = true;
  _runSilentWarmup(1);
  _armStartupDiscard(1);
  return true;
}

void TWK_KBE58_SSI::_releaseESP32Spi()
{
  if (!_esp32SpiInitialized)
  {
    return;
  }

  if (_spiHandle != nullptr)
  {
    spi_bus_remove_device((spi_device_handle_t)_spiHandle);
    _spiHandle = nullptr;
  }

  spi_bus_free((spi_host_device_t)_spiHost);
  _esp32SpiInitialized = false;
}

bool TWK_KBE58_SSI::startBackgroundRead(uint32_t intervalMs, BaseType_t core)
{
  if (_backgroundTaskRunning)
  {
    return true;
  }

  _backgroundIntervalMs = intervalMs;
  _backgroundTaskRunning = true;
  _newReading = false;

  BaseType_t created = xTaskCreatePinnedToCore(
    _backgroundTaskThunk,
    "TWK_KBE58_SSI",
    4096,
    this,
    1,
    &_backgroundTaskHandle,
    core);

  return created == pdPASS;
}

void TWK_KBE58_SSI::stopBackgroundRead()
{
  if (!_backgroundTaskRunning)
  {
    return;
  }

  _backgroundTaskRunning = false;

  while (_backgroundTaskHandle != nullptr)
  {
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

TWK_KBE58_SSI::Reading TWK_KBE58_SSI::getLastReading()
{
  Reading copy;

  portENTER_CRITICAL(&_mux);
  copy = _lastReading;
  _newReading = false;
  portEXIT_CRITICAL(&_mux);

  return copy;
}

bool TWK_KBE58_SSI::hasNewReading()
{
  bool hasReading;

  portENTER_CRITICAL(&_mux);
  hasReading = _newReading;
  portEXIT_CRITICAL(&_mux);

  return hasReading;
}

uint32_t TWK_KBE58_SSI::getReadCounter() const
{
  return _readCounter;
}

uint32_t TWK_KBE58_SSI::getErrorCounter() const
{
  return _errorCounter;
}

void TWK_KBE58_SSI::setRawBitShift(int8_t shift)
{
  _rawBitShift = shift;
}

void TWK_KBE58_SSI::_backgroundTaskThunk(void *parameter)
{
  TWK_KBE58_SSI *self = static_cast<TWK_KBE58_SSI *>(parameter);
  self->_backgroundTask();
}

void TWK_KBE58_SSI::_backgroundTask()
{
  while (_backgroundTaskRunning)
  {
    Reading reading = read();

    if (!reading.valid)
    {
      vTaskDelay(pdMS_TO_TICKS(_backgroundIntervalMs));
      continue;
    }

    portENTER_CRITICAL(&_mux);
    _lastReading = reading;
    _newReading = true;
    portEXIT_CRITICAL(&_mux);

    vTaskDelay(pdMS_TO_TICKS(_backgroundIntervalMs));
  }

  _backgroundTaskHandle = nullptr;
  vTaskDelete(nullptr);
}
#endif

void TWK_KBE58_SSI::setHalfPeriodUs(uint32_t halfPeriodUs)
{
  _halfPeriodUs = halfPeriodUs;
}

void TWK_KBE58_SSI::setFramePauseUs(uint32_t framePauseUs)
{
  _framePauseUs = framePauseUs;
}

void TWK_KBE58_SSI::setClockIdleHigh(bool clockIdleHigh)
{
  _clockIdleHigh = clockIdleHigh;

  if (_mode != Mode::BitBang)
  {
    return;
  }

  if (_clockIdleHigh)
  {
    digitalWrite(_clockPin, HIGH);
  }
  else
  {
    digitalWrite(_clockPin, LOW);
  }
}

void TWK_KBE58_SSI::setInvertData(bool invertData)
{
  _invertData = invertData;
}

void TWK_KBE58_SSI::setSpiFrequency(uint32_t frequencyHz)
{
  _spiFrequencyHz = frequencyHz;
}

void TWK_KBE58_SSI::setSpiMode(uint8_t spiMode)
{
  _spiMode = spiMode;
}

void TWK_KBE58_SSI::setSpiTransferBits(uint8_t transferBits)
{
  _spiTransferBits = transferBits;
}

void TWK_KBE58_SSI::setSpiRightShift(uint8_t rightShift)
{
  _spiRightShift = rightShift;
}

void TWK_KBE58_SSI::setSpiDummySsPin(int8_t ssPin)
{
  _spiDummySsPin = ssPin;
}

void TWK_KBE58_SSI::setStartupDiscardCount(uint8_t discardCount)
{
  _startupDiscardRemaining = discardCount;
}

void TWK_KBE58_SSI::configureZeroPin(int8_t pin, uint32_t pulseMs)
{
  if (_zeroPin >= 0 && _zeroPin != pin)
  {
    digitalWrite((uint8_t)_zeroPin, HIGH);
  }

  _zeroPin = pin;
  _zeroPulseMs = pulseMs;
  _zeroPulseActive = false;
  _zeroPulseStartMs = 0;

  if (_zeroPin >= 0)
  {
    pinMode((uint8_t)_zeroPin, OUTPUT);
    digitalWrite((uint8_t)_zeroPin, HIGH);
  }
}

void TWK_KBE58_SSI::setZero()
{
  if (_zeroPin < 0)
  {
    return;
  }

  digitalWrite((uint8_t)_zeroPin, LOW);
  _zeroPulseStartMs = millis();
  _zeroPulseActive = true;
}

void TWK_KBE58_SSI::update()
{
  if (!_zeroPulseActive || _zeroPin < 0)
  {
    return;
  }

  if ((uint32_t)(millis() - _zeroPulseStartMs) >= _zeroPulseMs)
  {
    digitalWrite((uint8_t)_zeroPin, HIGH);
    _zeroPulseActive = false;
  }
}

bool TWK_KBE58_SSI::isZeroPulseActive() const
{
  return _zeroPulseActive;
}

TWK_KBE58_SSI::Mode TWK_KBE58_SSI::mode() const
{
  return _mode;
}

TWK_KBE58_SSI::Reading TWK_KBE58_SSI::read()
{
  update();

  Reading reading;

  reading.dataIdleLevel = _sampleDataIdleLevel();
  reading.rawValue = readRaw();

  bool valid = _lastReadValid;

  if (_startupDiscardRemaining > 0)
  {
    _startupDiscardRemaining--;
    valid = false;
  }

  _fillReading(reading, reading.rawValue, valid);

  if (valid)
  {
    _readCounter++;
  }

  reading.readCounter = _readCounter;
  reading.errorCounter = _errorCounter;

  return reading;
}

uint32_t TWK_KBE58_SSI::readRaw()
{
  _lastReadValid = true;

  switch (_mode)
  {
    case Mode::ArduinoSPI:
      return readRawArduinoSPI();

#if defined(ESP32)
    case Mode::ESP32PreciseSPI:
      return readRawESP32PreciseSPI();
#endif

    default:
      return readRawBitBang();
  }
}

uint32_t TWK_KBE58_SSI::readRawBitBang()
{
  uint32_t rawValue = 0;

  if (_clockIdleHigh)
  {
    digitalWrite(_clockPin, HIGH);
  }
  else
  {
    digitalWrite(_clockPin, LOW);
  }

  delayMicroseconds(_framePauseUs);

  for (uint8_t bitIndex = 0; bitIndex < _totalClocks; bitIndex++)
  {
    digitalWrite(_clockPin, LOW);
    delayMicroseconds(_halfPeriodUs);

    digitalWrite(_clockPin, HIGH);
    delayMicroseconds(_halfPeriodUs);

    int dataBit = digitalRead(_dataPin);

    if (_invertData)
    {
      dataBit = !dataBit;
    }

    rawValue <<= 1;

    if (dataBit)
    {
      rawValue |= 1;
    }
  }

  delayMicroseconds(_framePauseUs);

  return rawValue;
}

uint32_t TWK_KBE58_SSI::readRawArduinoSPI()
{
  if (_spi == nullptr)
  {
    _lastReadValid = false;
    _errorCounter++;
    return 0;
  }

  delayMicroseconds(_framePauseUs);

  SPISettings settings(_spiFrequencyHz, MSBFIRST, _spiMode);
  _spi->beginTransaction(settings);

  uint32_t rx = 0;

  if (_spiTransferBits <= 8)
  {
    rx = _spi->transfer(0x00);
  }
  else if (_spiTransferBits <= 16)
  {
    rx = ((uint32_t)_spi->transfer(0x00)) << 8;
    rx |= (uint32_t)_spi->transfer(0x00);
  }
  else
  {
    for (uint8_t byteIndex = 0; byteIndex < (_spiTransferBits + 7) / 8; byteIndex++)
    {
      rx = (rx << 8) | (uint32_t)_spi->transfer(0x00);
    }
  }

  _spi->endTransaction();

  delayMicroseconds(_framePauseUs);

  return _applyRawShiftAndMask(rx);
}

#if defined(ESP32)
uint32_t TWK_KBE58_SSI::readRawESP32PreciseSPI()
{
  if (!_esp32SpiInitialized || _spiHandle == nullptr)
  {
    _lastReadValid = false;
    _errorCounter++;
    return 0;
  }

  uint8_t rxBytes[4] = {0, 0, 0, 0};
  const uint8_t rxByteCount = (uint8_t)((_totalClocks + 7) / 8);

  spi_transaction_t transaction = {};
  transaction.length = 0;
  transaction.rxlength = _totalClocks;
  transaction.tx_buffer = nullptr;
  transaction.rx_buffer = rxBytes;

  delayMicroseconds(_framePauseUs);

  esp_err_t result = spi_device_transmit((spi_device_handle_t)_spiHandle, &transaction);

  delayMicroseconds(_framePauseUs);

  if (result != ESP_OK)
  {
    _lastReadValid = false;
    _errorCounter++;
    return 0;
  }

  uint32_t rx = 0;

  for (uint8_t byteIndex = 0; byteIndex < rxByteCount; byteIndex++)
  {
    rx = (rx << 8) | (uint32_t)rxBytes[byteIndex];
  }

  const uint8_t bitRemainder = _totalClocks % 8;

  if (bitRemainder != 0)
  {
    rx >>= (8 - bitRemainder);
  }

  if (_rawBitShift > 0)
  {
    rx >>= (uint8_t)_rawBitShift;
  }
  else if (_rawBitShift < 0)
  {
    rx <<= (uint8_t)(-_rawBitShift);
  }

  return rx & _totalClocksMask();
}
#endif

int TWK_KBE58_SSI::_sampleDataIdleLevel() const
{
  return digitalRead(_dataPin);
}

void TWK_KBE58_SSI::_runSilentWarmup(uint8_t count)
{
  for (uint8_t index = 0; index < count; index++)
  {
    readRaw();
    delayMicroseconds(_framePauseUs * 2);
  }
}

void TWK_KBE58_SSI::_armStartupDiscard(uint8_t count)
{
  _startupDiscardRemaining = count;
}

uint32_t TWK_KBE58_SSI::_applyRawShiftAndMask(uint32_t rawValue) const
{
  if (_mode == Mode::ArduinoSPI && _spiRightShift > 0)
  {
    rawValue >>= _spiRightShift;
  }

  return rawValue & _totalClocksMask();
}

void TWK_KBE58_SSI::_fillReading(Reading &reading, uint32_t rawValue, bool valid)
{
  uint32_t trailingMask = 0;

  if (_trailingBits > 0)
  {
    trailingMask = (1UL << _trailingBits) - 1;
  }

  reading.rawValue = rawValue;
  reading.valid = valid;
  reading.trailingBits = rawValue & trailingMask;
  reading.grayValue = (rawValue >> _trailingBits) & _usefulBitMask();
  reading.position = grayToBinary(reading.grayValue);
  reading.stepsPerRevolution = stepsPerRevolution();
  reading.angleDeg = ((float)reading.position * 360.0f) / (float)reading.stepsPerRevolution;
  reading.angleDegRounded = ((float)_roundAngleTo01(reading.angleDeg)) / 10.0f;
}

uint32_t TWK_KBE58_SSI::grayToBinary(uint32_t grayValue) const
{
  uint32_t binaryValue = grayValue;

  while (grayValue >>= 1)
  {
    binaryValue ^= grayValue;
  }

  return binaryValue;
}

uint8_t TWK_KBE58_SSI::usefulBits() const
{
  return _usefulBits;
}

uint8_t TWK_KBE58_SSI::totalClocks() const
{
  return _totalClocks;
}

uint8_t TWK_KBE58_SSI::trailingBits() const
{
  return _trailingBits;
}

uint32_t TWK_KBE58_SSI::stepsPerRevolution() const
{
  return 1UL << _usefulBits;
}

uint32_t TWK_KBE58_SSI::_totalClocksMask() const
{
  if (_totalClocks >= 32)
  {
    return 0xFFFFFFFFUL;
  }

  return (1UL << _totalClocks) - 1;
}

uint32_t TWK_KBE58_SSI::_usefulBitMask() const
{
  return stepsPerRevolution() - 1;
}

uint32_t TWK_KBE58_SSI::_roundAngleTo01(float angleDeg) const
{
  return (uint32_t)((angleDeg * 10.0f) + 0.5f);
}
