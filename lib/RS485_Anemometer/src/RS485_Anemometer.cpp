#include "RS485_Anemometer.h"

// ============================================================
// Konstruktor
// ============================================================
RS485_Anemometer::RS485_Anemometer()
: _ser(nullptr),
  _deRePin(-1),
  _addr(0x01),
  _angleOffsetDeg(0.0f),
  _speedOffsetMps(0.0f),
  _lastFrameMs(0),
  _last{0.0f, 0, 0.0f, 0}
{
}

// ============================================================
// Public API
// ============================================================
void RS485_Anemometer::begin(HardwareSerial& port,
                            int rxPin,
                            int txPin,
                            int deRePin,
                            uint8_t address,
                            uint32_t baud)
{
  _ser = &port;
  _deRePin = deRePin;
  _addr = address;

  pinMode(_deRePin, OUTPUT);
  digitalWrite(_deRePin, LOW); // Empfangen

  // Modbus RTU Standard: 8N1
  _ser->begin(baud, SERIAL_8N1, rxPin, txPin);

  flushInput();
  _lastFrameMs = millis();
}

void RS485_Anemometer::setAngleOffsetDeg(float offsetDeg)
{
  _angleOffsetDeg = offsetDeg;
}

void RS485_Anemometer::setSpeedOffsetMps(float offsetMps)
{
  _speedOffsetMps = offsetMps;
}
 
 
 

bool RS485_Anemometer::update()
{
  Reading r;

  if (!readWindFast(r))
  {
    return false;
  }

  _last = r;
  return true;
}

float RS485_Anemometer::getWindSpeedMps() const
{
  return _last.windSpeed_mps;
}

int RS485_Anemometer::getWindLevel() const
{
  return (int)_last.windLevel;
}

float RS485_Anemometer::getWindAngleDeg() const
{
  return _last.windAngle_deg;
}

uint8_t RS485_Anemometer::getWindDirCode() const
{
  return _last.windDirCode;
}

const char* RS485_Anemometer::getWindDirText(DirLanguage lang) const
{
  if (lang == DirLanguage::EN)
  {
    return windDir16TextEN(_last.windDirCode);
  }

  return windDir16TextDE(_last.windDirCode);
}

// ============================================================
// Statische Hilfsfunktionen
// ============================================================
uint16_t RS485_Anemometer::modbusCRC16(const uint8_t* data, size_t len)
{
  // Standard Modbus CRC16 (LSB first)
  uint16_t crc = 0xFFFF;

  for (size_t i = 0; i < len; i++)
  {
    crc ^= (uint16_t)data[i];

    for (uint8_t b = 0; b < 8; b++)
    {
      if ((crc & 0x0001) != 0)
      {
        crc >>= 1;
        crc ^= 0xA001;
      }
      else
      {
        crc >>= 1;
      }
    }
  }

  return crc;
}

uint16_t RS485_Anemometer::be16(uint8_t hi, uint8_t lo)
{
  // Big-Endian (High zuerst)
  return (uint16_t)((hi << 8) | lo);
}

float RS485_Anemometer::normalizeDeg(float deg)
{
  // Winkel auf 0..360 normalisieren
  while (deg < 0.0f)
  {
    deg += 360.0f;
  }

  while (deg >= 360.0f)
  {
    deg -= 360.0f;
  }

  return deg;
}

uint8_t RS485_Anemometer::dirCodeFromAngle16(float angleDeg)
{
  // 16-point rose: 360/16 = 22.5° per sector
  // Add +11.25° to round to the nearest sector
  float a = normalizeDeg(angleDeg);

  float shifted = a + 11.25f;
  if (shifted >= 360.0f)
  {
    shifted -= 360.0f;
  }

  return (uint8_t)(((uint8_t)(shifted / 22.5f)) & 0x0F);
}

const char* RS485_Anemometer::windDir16TextDE(uint8_t code)
{
  // German: O instead of E
  switch (code & 0x0F)
  {
    case 0x00: return "N";
    case 0x01: return "NNO";
    case 0x02: return "NO";
    case 0x03: return "ONO";
    case 0x04: return "O";
    case 0x05: return "OSO";
    case 0x06: return "SO";
    case 0x07: return "SSO";
    case 0x08: return "S";
    case 0x09: return "SSW";
    case 0x0A: return "SW";
    case 0x0B: return "WSW";
    case 0x0C: return "W";
    case 0x0D: return "WNW";
    case 0x0E: return "NW";
    case 0x0F: return "NNW";
    default:   return "?";
  }
}

const char* RS485_Anemometer::windDir16TextEN(uint8_t code)
{
  // English: E statt O
  switch (code & 0x0F)
  {
    case 0x00: return "N";
    case 0x01: return "NNE";
    case 0x02: return "NE";
    case 0x03: return "ENE";
    case 0x04: return "E";
    case 0x05: return "ESE";
    case 0x06: return "SE";
    case 0x07: return "SSE";
    case 0x08: return "S";
    case 0x09: return "SSW";
    case 0x0A: return "SW";
    case 0x0B: return "WSW";
    case 0x0C: return "W";
    case 0x0D: return "WNW";
    case 0x0E: return "NW";
    case 0x0F: return "NNW";
    default:   return "?";
  }
}

// ============================================================
// Private: RS485/Modbus Hilfsroutinen
// ============================================================
void RS485_Anemometer::flushInput()
{
  // Eingangsbuffer leeren (alte Bytes entfernen)
  while (_ser && _ser->available() > 0)
  {
    (void)_ser->read();
  }
}

void RS485_Anemometer::interFrameDelay(uint16_t interMs)
{
  // Modbus RTU: Minimaler Abstand zwischen Frames
  uint32_t now = millis();
  uint32_t diff = now - _lastFrameMs;

  if (diff < interMs)
  {
    delay((uint32_t)interMs - diff);
  }
}

void RS485_Anemometer::markFrame()
{
  _lastFrameMs = millis();
}

void RS485_Anemometer::txEnable(uint16_t preUs)
{
  // RS485 auf Senden schalten
  digitalWrite(_deRePin, HIGH);
  delayMicroseconds(preUs);
}

void RS485_Anemometer::txDisable(uint16_t postUs)
{
  // Nachlaufzeit geben und dann auf Empfangen
  delayMicroseconds(postUs);
  digitalWrite(_deRePin, LOW);

  // Kleines Extra-Delay, damit der Transceiver sauber umschaltet
  delayMicroseconds(50);
}

bool RS485_Anemometer::readBytes(uint8_t* buf, size_t len, uint16_t timeoutMs)
{
  // Blockierendes Lesen mit Timeout
  size_t got = 0;
  uint32_t start = millis();

  while (got < len)
  {
    if (_ser->available() > 0)
    {
      buf[got++] = (uint8_t)_ser->read();
    }
    else
    {
      if ((millis() - start) > timeoutMs)
      {
        return false;
      }

      // Kurzes Yield
      delay(1);
    }
  }

  return true;
}

// ============================================================
// Modbus: Read Holding Registers (0x03)
// ============================================================
bool RS485_Anemometer::readHoldingRegisters03(uint16_t startReg,
                                             uint16_t count,
                                             uint16_t* outRegs,
                                             uint16_t interMs,
                                             uint16_t preUs,
                                             uint16_t postUs,
                                             uint16_t timeoutMs)
{
  interFrameDelay(interMs);

  // Request: Addr, Func, StartHi, StartLo, CountHi, CountLo, CRCLo, CRCHi
  uint8_t req[8];
  req[0] = _addr;
  req[1] = 0x03;
  req[2] = (uint8_t)((startReg >> 8) & 0xFF);
  req[3] = (uint8_t)(startReg & 0xFF);
  req[4] = (uint8_t)((count >> 8) & 0xFF);
  req[5] = (uint8_t)(count & 0xFF);

  uint16_t crc = modbusCRC16(req, 6);
  req[6] = (uint8_t)(crc & 0xFF);
  req[7] = (uint8_t)((crc >> 8) & 0xFF);

  // Alte Bytes weg
  flushInput();

  // Senden
  txEnable(preUs);
  _ser->write(req, sizeof(req));
  _ser->flush();
  txDisable(postUs);

  markFrame();

  // Antwortkopf lesen (Addr, Func, ByteCount)
  uint8_t head[3];
  if (!readBytes(head, 3, timeoutMs)) return false;
  if (head[0] != _addr) return false;
  if (head[1] != 0x03)  return false;

  uint8_t byteCount = head[2];
  if (byteCount != (uint8_t)(count * 2)) return false;

  // Total length: 3 + byteCount + 2 (CRC)
  size_t respLen = (size_t)3 + (size_t)byteCount + 2;
  uint8_t resp[64];

  resp[0] = head[0];
  resp[1] = head[1];
  resp[2] = head[2];

  if (!readBytes(&resp[3], respLen - 3, timeoutMs)) return false;

  // Check CRC
  uint16_t respCrc = (uint16_t)resp[respLen - 2] | ((uint16_t)resp[respLen - 1] << 8);
  uint16_t calcCrc = modbusCRC16(resp, respLen - 2);
  if (respCrc != calcCrc) return false;

  // Register extrahieren
  for (uint16_t i = 0; i < count; i++)
  {
    outRegs[i] = be16(resp[3 + (i * 2)], resp[3 + (i * 2) + 1]);
  }

  return true;
}

// ============================================================
// Wind: schnelles Profil
// ============================================================
bool RS485_Anemometer::readWindFast(Reading& out)
{
  // Sensor-Layout wie Original:
  // Reg0: Speed (0.1 m/s)
  // Reg1: Level
  // Reg3: Winkel (0.1°)
  uint16_t regs[4];

  if (!readHoldingRegisters03(
        0x0000,
        4,
        regs,
        WIND_INTERFRAME_DELAY_MS,
        WIND_PRE_TX_US,
        WIND_POST_TX_US,
        WIND_TIMEOUT_MS))
  {
    return false;
  }

  // Speed
  float speed = (float)regs[0] / 10.0f;
  speed = speed + _speedOffsetMps;

  // Winkel
  float angle = (float)regs[3] / 10.0f;
  angle = angle + _angleOffsetDeg;
  angle = normalizeDeg(angle);

  out.windSpeed_mps = speed;
  out.windLevel = regs[1];
  out.windAngle_deg = angle;
  out.windDirCode = dirCodeFromAngle16(angle);

  return true;
}
