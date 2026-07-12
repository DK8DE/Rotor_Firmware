/*
  RD130_ESP32_SPI_Background.ino

  ESP32 background read example for the TWK_KBE58_SSI library.

  Uses ESP32 precise SPI (ESP-IDF SPI master) so exactly 13 SSI clock pulses
  are generated. A FreeRTOS task reads the encoder in the background;
  loop() prints results.

  For minimal ESP32 precise SPI in loop(), see RD130_ESP32_PreciseSPI.

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

  ESP32 or ESP32-S3 (BitBang or SPI, example pins):

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

#include <TWK_KBE58_SSI.h>

const uint8_t PIN_SSI_CLOCK = 8;
const uint8_t PIN_SSI_DATA = 9;
const uint32_t BACKGROUND_INTERVAL_MS = 10;
const uint32_t SPI_FREQUENCY_HZ = 100000;
const int8_t PIN_SET_ZERO = 4;
const uint32_t ZERO_PULSE_MS = 200;

TWK_KBE58_SSI encoder(PIN_SSI_CLOCK, PIN_SSI_DATA);
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
#if defined(ESP32)
        Serial0.println("Zero pulse started (LOW 200 ms)");
#endif
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

void printReading(const TWK_KBE58_SSI::Reading &reading, Stream &out)
{
  out.print("Position: ");
  out.print(reading.position);
  out.print(" / ");
  out.print(reading.stepsPerRevolution);
  out.print(" Angle: ");
  out.print(reading.angleDegRounded, 1);
  out.println(" deg");
}

void setup()
{
  Serial.begin(115200);

#if defined(ESP32)
  Serial0.begin(115200);

  if (!encoder.beginESP32PreciseSPI(PIN_SSI_CLOCK, PIN_SSI_DATA, SPI_FREQUENCY_HZ))
  {
    Serial.println("ESP32 precise SPI init failed");
    Serial0.println("ESP32 precise SPI init failed");
    while (true)
    {
      delay(1000);
    }
  }

  // RD130 timing matches BitBang sampling best with CPOL=1, CPHA=1 (SPI_MODE3).
  // If your hardware differs, test SPI_MODE2.
  encoder.setSpiMode(SPI_MODE3);
  encoder.setRawBitShift(0);
  encoder.setFramePauseUs(80);
  encoder.configureZeroPin(PIN_SET_ZERO, ZERO_PULSE_MS);

  if (!encoder.startBackgroundRead(BACKGROUND_INTERVAL_MS))
  {
    Serial.println("Background read start failed");
    Serial0.println("Background read start failed");
    while (true)
    {
      delay(1000);
    }
  }

  Serial.println("RD130 background read started (ESP32 precise SPI, 10 ms)");
  Serial0.println("RD130 background read started (ESP32 precise SPI, 10 ms)");
  Serial.print("Zero pin: ");
  Serial.println(PIN_SET_ZERO);
  Serial.println("Send 'zero' to set encoder hardware zero");
#else
  Serial.println("This example requires an ESP32 board.");
#endif
}

void loop()
{
  encoder.update();
  handleSerialCommand();

#if defined(ESP32)
  if (encoder.hasNewReading())
  {
    TWK_KBE58_SSI::Reading reading = encoder.getLastReading();

    if (!reading.valid)
    {
      return;
    }

    printReading(reading, Serial);
    printReading(reading, Serial0);
  }
#else
  delay(1000);
#endif
}
