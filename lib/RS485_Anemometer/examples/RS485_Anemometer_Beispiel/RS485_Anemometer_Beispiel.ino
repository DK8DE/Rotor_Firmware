#include <Arduino.h>
#include <RS485_Anemometer.h>

/*
  RS485_Anemometer_Beispiel

  Dieses Beispiel liest einen RS485/Modbus-RTU Windmesser am ESP32 aus und gibt
  die Messwerte periodisch über den Seriell-Monitor aus.

  Ausgegeben werden:
  - Windgeschwindigkeit in m/s (1 Nachkommastelle)
  - Level als Ganzzahl
  - Winkel in Grad (1 Nachkommastelle)
  - Himmelsrichtung als Text
    * Deutsch: O statt E (z.B. ONO, O, OSO)
    * Englisch: E (z.B. ENE, E, ESE)

  Einstellungen, die typischerweise angepasst werden müssen:
  1) RS485_PORT: Welche serielle Schnittstelle am ESP32 verwendet wird
  2) PIN_RS485_DE_RE, PIN_RS485_TX, PIN_RS485_RX: Pins für den RS485-Transceiver
  3) WIND_ADDR: Modbus-Adresse des Windmessers
  4) Offsets (optional): Korrekturwerte für Winkel und Geschwindigkeit
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
  /* USB-Seriell für Ausgabe */
  Serial.begin(115200);
  delay(200);

  Serial.println();
  Serial.println("RS485_Anemometer Beispiel gestartet");

  /* RS485/Modbus initialisieren */
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
      Ausgabeformat:
      Wind: <m/s> m/s, Level <n>, Winkel <deg> deg, Richtung <DE>/<EN> (Code 0x..)
    */
    Serial.print("Wind: ");
    Serial.print(wind.getWindSpeedMps(), 1);
    Serial.print(" m/s, Level ");
    Serial.print(wind.getWindLevel());

    Serial.print(", Winkel ");
    Serial.print(wind.getWindAngleDeg(), 1);
    Serial.print(" deg");

    Serial.print(", Richtung ");
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
