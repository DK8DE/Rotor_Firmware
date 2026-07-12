/*
  RD130_ArduinoSPI.ino

  Portable Arduino SPI example for the TWK_KBE58_SSI library.

  The encoder is read continuously so SCK pulses stay visible on a scope.
  Serial output is printed every 200 ms only.

  SPI note: portable Arduino SPI usually generates 16 clock pulses. The library
  keeps the 13 relevant SSI bits with setSpiRightShift(3).

  This example reads the SSI absolute encoder inside a Rohde & Schwarz RD130
  rotor. The encoder is assumed to be a TWK KBE 58 - K 4096 G K E06 with Gray
  code, 12 useful position bits and 13 SSI clocks.

  The interface between the microcontroller and the encoder uses one full duplex
  RS422 transceiver. For 3.3 V boards such as ESP32, the ADM3490ARZ can be used.
  For 5 V Arduino boards such as Uno, Nano or Mega, use a 5 V full duplex RS422
  transceiver such as the MAX490.

  Copyright (C) 2026 Joerg Koerner DK8DE

  This example is free software: you can redistribute it and/or modify it under
  the terms of the GNU General Public License as published by the Free Software
  Foundation, either version 3 of the License, or any later version.

  This example is distributed in the hope that it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
  FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

  ADM3490ARZ connection for ESP32 or another 3.3 V board:

                   ADM3490ARZ
            +----------------------+
   +3V3 ----| 1 VCC            A 8 |---- DATA+ ---- Encoder Pin 4
   IO9  ----| 2 RO             B 7 |---- DATA- ---- Encoder Pin 5
   IO8  ----| 3 DI             Z 6 |---- CLOCK- ---- Encoder Pin 3
   GND  ----| 4 GND            Y 5 |---- CLOCK+ ---- Encoder Pin 2
            +----------------------+

  MAX490 connection for Arduino Uno, Nano or Mega (5 V):

                   MAX490
         +----------------------+
 +5V ----| VCC              TX+ |---- CLOCK+ -> Encoder Pin 2
 D13 ----| DI               TX- |---- CLOCK- -> Encoder Pin 3
 D12 ----| RO               RX+ |---- DATA+  -> Encoder Pin 4
 GND ----| GND              RX- |---- DATA-  -> Encoder Pin 5
         +----------------------+
            DE and RE tied for always-on transmit and receive.

  Arduino Uno or Nano (BitBang example pins D8 and D9):

    D8  -> MAX490 DI or TXD  -> encoder CLOCK+ and CLOCK-
    D9  <- MAX490 RO or RXD  <- encoder DATA+ and DATA-
    GND -> MAX490 GND        -> encoder GND

  Arduino Uno or Nano (hardware SPI):

    D13 (SCK)  -> MAX490 DI or TXD  -> encoder CLOCK+ and CLOCK-
    D12 (MISO) <- MAX490 RO or RXD  <- encoder DATA+ and DATA-
    D10 (SS)   kept OUTPUT HIGH for SPI master mode
    D11 (MOSI) not connected for SSI

  Arduino Mega 2560 (hardware SPI):

    D52 (SCK)  -> MAX490 DI or TXD  -> encoder CLOCK+ and CLOCK-
    D50 (MISO) <- MAX490 RO or RXD  <- encoder DATA+ and DATA-
    D53 (SS)   kept OUTPUT HIGH for SPI master mode
    D51 (MOSI) not connected for SSI

  ESP32 or ESP32-S3 (BitBang or SPI with FSPI, example pins):

    IO8  -> ADM3490 DI  -> encoder CLOCK+ and CLOCK-
    IO9  <- ADM3490 RO  <- encoder DATA+ and DATA-
    GND  -> ADM3490 GND -> encoder GND

  Connection at the Rohde & Schwarz RD130 rotor:

  Data connector:

  1 ---> VCC 11-30 V + SET input via push button to VCC -- set encoder to 0
  2 ---> CLOCK IN +
  3 ---> CLOCK IN -
  4 ---> DATA OUT +
  5 ---> DATA OUT -
  6 ---> GND + Code Sense 0 = CW 1 = CCW
  8 ---> Shield

  Motor connector:

  1 ---> Motor 1 +
  2 ---> Motor 1 -
  3 ---> Filter 1 ground
  4 ---> Motor 2 -
  5 ---> Motor 2 +
  6 ---> Filter 2 ground

  Important notes:
  - Never connect CLOCK+ and CLOCK- or DATA+ and DATA- directly to Arduino or ESP32 GPIO pins.
  - Use the ADM3490ARZ for 3.3 V boards or another suitable full duplex RS422 transceiver.
  - For 5 V Arduino boards, use a 5 V full duplex RS422 transceiver such as the MAX490.
  - Connect microcontroller GND, RS422 transceiver GND and encoder 0 V together.
  - Keep CLOCK+ and CLOCK- and DATA+ and DATA- as twisted or closely coupled pairs.
  - Use a 100 nF decoupling capacitor close to the RS422 transceiver VCC and GND pins.
  - For the DATA pair, a 120 ohm termination close to the receiver is recommended.
  - For longer CLOCK lines, a 120 ohm termination at the encoder side may be required.
*/

#include <SPI.h>
#include <TWK_KBE58_SSI.h>

const uint32_t SERIAL_PRINT_INTERVAL_MS = 200;
const int8_t PIN_SET_ZERO = 4;
const uint32_t ZERO_PULSE_MS = 200;

#if defined(ESP32)

// ESP32: free SPI pins via FSPI, connected to ADM3490 or another RS422 module.
const uint8_t PIN_SSI_CLOCK = 8;
const uint8_t PIN_SSI_DATA = 9;

SPIClass ssiSPI(FSPI);
TWK_KBE58_SSI encoder(PIN_SSI_CLOCK, PIN_SSI_DATA);

#else

// Arduino Uno / Nano: D13=SCK, D12=MISO, D10=SS
// Arduino Mega 2560:  D52=SCK, D50=MISO, D53=SS
const uint8_t PIN_SSI_CLOCK = SCK;
const uint8_t PIN_SSI_DATA = MISO;
const uint8_t PIN_SSI_SPI_SS = SS;

TWK_KBE58_SSI encoder(PIN_SSI_CLOCK, PIN_SSI_DATA);

#endif

TWK_KBE58_SSI::Reading g_lastReading;
unsigned long g_lastPrintMs = 0;
String g_serialCommand;

void handleSerialCommand()
{
  while (Serial.available() > 0)
  {
    char c = (char)Serial.read();

    if (c == '\n' || c == '\r')
    {
      g_serialCommand.trim();
      g_serialCommand.toLowerCase();

      if (g_serialCommand == "zero")
      {
        encoder.setZero();
        Serial.println("Zero pulse started (LOW 200 ms)");
      }

      g_serialCommand = "";
      continue;
    }

    if (isPrintable(c) && g_serialCommand.length() < 32)
    {
      g_serialCommand += c;
    }
  }
}

void setup()
{
  Serial.begin(115200);

#if defined(ESP32)
  encoder.beginSPI(ssiSPI, PIN_SSI_CLOCK, PIN_SSI_DATA, 100000);
#else
  encoder.beginSPI(SPI, PIN_SSI_CLOCK, PIN_SSI_DATA, 100000);
  encoder.setSpiDummySsPin(PIN_SSI_SPI_SS);
#endif

  encoder.setSpiMode(SPI_MODE2);
  encoder.setSpiTransferBits(16);
  encoder.setSpiRightShift(3);
  encoder.setFramePauseUs(80);
  encoder.configureZeroPin(PIN_SET_ZERO, ZERO_PULSE_MS);

  Serial.println("RD130 Arduino SPI started");
  Serial.print("Zero pin: ");
  Serial.println(PIN_SET_ZERO);
  Serial.println("Send 'zero' to set encoder hardware zero");

#if defined(ESP32)
  Serial.print("ESP32 SCK:  IO");
  Serial.println(PIN_SSI_CLOCK);
  Serial.print("ESP32 MISO: IO");
  Serial.println(PIN_SSI_DATA);
#elif defined(__AVR_ATmega2560__)
  Serial.println("Mega2560 SCK:  D52 -> MAX490 DI");
  Serial.println("Mega2560 MISO: D50 <- MAX490 RO");
  Serial.println("Mega2560 SS:   D53 (OUTPUT, HIGH)");
#else
  Serial.println("Uno/Nano SCK:  D13 -> MAX490 DI");
  Serial.println("Uno/Nano MISO: D12 <- MAX490 RO");
  Serial.println("Uno/Nano SS:   D10 (OUTPUT, HIGH)");
#endif

  Serial.println("Continuous SPI reads, Serial every 200 ms");
  Serial.println("SPI sends 16 clock pulses per read (13 used after shift)");
}

void loop()
{
  encoder.update();
  handleSerialCommand();

  g_lastReading = encoder.read();

  if (!g_lastReading.valid)
  {
    return;
  }

  unsigned long nowMs = millis();

  if (nowMs - g_lastPrintMs < SERIAL_PRINT_INTERVAL_MS)
  {
    return;
  }

  g_lastPrintMs = nowMs;

  Serial.print("Raw: ");
  for (int8_t bit = encoder.totalClocks() - 1; bit >= 0; bit--)
  {
    Serial.print((g_lastReading.rawValue >> bit) & 1);
  }

  Serial.print(" Position: ");
  Serial.print(g_lastReading.position);
  Serial.print(" Angle: ");
  Serial.println(g_lastReading.angleDegRounded, 1);
}
