#include <Arduino.h>
#include <RS485_Anemometer.h>

/*
  RS485_Anemometer_Example

  This example reads an RS485 / Modbus-RTU anemometer with an ESP32 and prints
  the measured values periodically to the Serial Monitor.

  Output:
  - Wind speed in m/s (1 decimal place)
  - Level as integer
  - Angle in degrees (1 decimal place)
  - Cardinal direction as text
    * German: O instead of E (e.g. ONO, O, OSO)
    * English: E (e.g. ENE, E, ESE)

  Typical settings to adjust below:
  - UART port (Serial2)
  - RS485 pins (TX, RX, DE/RE)
  - Modbus address of the sensor
  - Optional offsets (speed and direction)
*/

static HardwareSerial& RS485_PORT = Serial2;

static const int PIN_RS485_DE_RE = 25;
static const int PIN_RS485_TX    = 32;
static const int PIN_RS485_RX    = 33;

static const uint8_t WIND_ADDR = 0x01;

/* Optional: Korrekturwerte */
static float WIND_ANGLE_OFFSET_DEG = 0.0f;
static float WIND_SPEED_OFFSET_MPS = 0.0f;

RS485_Anemometer wind;

void setup()
{
  /* USB serial for output */
  Serial.begin(115200);
  delay(200);

  Serial.println();
  Serial.println("RS485_Anemometer example started");

  /* Initialize RS485/Modbus */
  wind.begin(RS485_PORT, PIN_RS485_RX, PIN_RS485_TX, PIN_RS485_DE_RE, WIND_ADDR);

  /* Optional: Korrekturwerte setzen */
  wind.setAngleOffsetDeg(WIND_ANGLE_OFFSET_DEG);
  wind.setSpeedOffsetMps(WIND_SPEED_OFFSET_MPS);
}

void loop()
{
  /* Neue Messung einlesen */
  if (wind.update())
  {
    /*
      Output format:
      Wind: <m/s> m/s, Level <n>, Angle <deg> deg, Direction <DE>/<EN> (Code 0x..)
    */
    Serial.print("Wind: ");
    Serial.print(wind.getWindSpeedMps(), 1);
    Serial.print(" m/s, Level ");
    Serial.print(wind.getWindLevel());

    Serial.print(", Angle ");
    Serial.print(wind.getWindAngleDeg(), 1);
    Serial.print(" deg");

    Serial.print(", Direction ");
    Serial.print(wind.getWindDirText(RS485_Anemometer::DirLanguage::DE));
    Serial.print("/");
    Serial.print(wind.getWindDirText(RS485_Anemometer::DirLanguage::EN));
    Serial.print(" (Code 0x");

    uint8_t code = wind.getWindDirCode();
    if (code < 16)
    {
      Serial.print("0");
    }
    Serial.print(code, HEX);
    Serial.println(")");
  }

  /* Lesetakt */
  delay(500);
}
