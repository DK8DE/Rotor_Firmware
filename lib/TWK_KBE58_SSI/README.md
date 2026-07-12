# TWK_KBE58_SSI Unofficial Arduino and ESP32 Library for Rohde & Schwarz RD130

Code for the `TWK_KBE58_SSI` Arduino library.

## Description

This library reads the SSI absolute encoder inside a Rohde & Schwarz RD130 rotor.

The encoder is assumed to be a TWK KBE 58 - K 4096 G K E06 with:

- Gray code output
- 12 useful position bits
- 13 SSI clock pulses
- 4096 positions per revolution
- Single-turn absolute position

The library reads and provides a `Reading` structure with:

- SSI raw value
- Gray value
- Binary position
- DATA idle level
- Trailing bits
- Angle in degrees (full precision and rounded to 0.1 degrees)
- `valid` flag (false during startup discard or after failed reads)
- `readCounter` and `errorCounter` statistics

Always check `reading.valid` before using position or angle values. The example sketches ignore invalid readings during startup.

The angle output can be rounded to 0.1 degrees in the example sketch.

## Supported Boards

The library is written for Arduino-compatible boards and can be used with ESP32, ESP32-S3, Arduino and other microcontrollers, as long as the board can generate a stable SSI clock and read a digital input.

The Arduino `library.properties` file may use:

```ini
architectures=*
```

This makes the library visible for all Arduino architectures.

## Logic Level and RS422 Interface

The encoder does not use direct TTL or CMOS logic signals.

The SSI interface uses differential RS422/RS485-style signals:

- CLOCK IN +
- CLOCK IN -
- DATA OUT +
- DATA OUT -

Never connect these differential encoder signals directly to Arduino, ESP32 or other microcontroller GPIO pins.

A suitable full duplex RS422 transceiver is required.

## Ready-Made RS422 Module for Testing

For a quick test setup, you can use a ready-made RS422 TTL full duplex module.

Search for:

```text
MAX490 RS422 TTL Full Duplex Modul
```

A module based on the MAX490 usually provides one differential driver and one differential receiver. This is suitable for the SSI interface because the microcontroller must send the SSI clock to the encoder and receive the SSI data from the encoder at the same time.

Typical module pin names:

```text
TTL side:
VCC
GND
TXD or DI
RXD or RO

RS422 side:
TX+
TX-
RX+
RX-
```

Typical connection to the RD130 encoder:

```text
Microcontroller clock output -> TXD / DI

TX+ -> RD130 data connector pin 2 CLOCK IN+
TX- -> RD130 data connector pin 3 CLOCK IN-

RX+ -> RD130 data connector pin 4 DATA OUT+
RX- -> RD130 data connector pin 5 DATA OUT-

RXD / RO -> Microcontroller data input
GND      -> Microcontroller GND and RD130 data connector pin 6 GND
```

For 5 V Arduino boards, a MAX490 module is a practical choice because it is intended for 5 V logic.

For ESP32 or other 3.3 V boards, check the module carefully. Many MAX490 modules use 5 V logic and are not directly compatible with 3.3 V GPIO pins. For 3.3 V boards, use the ADM3490ARZ circuit shown below or a 3.3 V compatible RS422 TTL full duplex module.

Do not use simple MAX485 RS485 modules for this SSI interface. Most MAX485 modules are half duplex and provide only one A/B pair. The SSI encoder needs full duplex operation with two differential pairs: one pair for CLOCK and one pair for DATA.

## Recommended Transceiver for 3.3 V Boards

For ESP32, ESP32-S3 and other 3.3 V Arduino-compatible boards, the recommended transceiver is:

```text
ADM3490ARZ
```

The ADM3490ARZ is a 3.3 V full duplex RS422 transceiver. It can send the SSI clock to the encoder and receive the SSI data from the encoder at the same time.

## Recommended Transceiver for 5 V Arduino Boards

For classic 5 V Arduino boards such as Arduino Uno, Arduino Nano or Arduino Mega, do not use the ADM3490ARZ directly with 5 V logic.

For 5 V boards, use a 5 V full duplex RS422 transceiver, for example:

```text
MAX490
```

The MAX490 is suitable for 5 V logic and provides one differential driver and one differential receiver. It can be used for the SSI clock and data lines.

Typical use with 5 V Arduino boards:

- Arduino digital output -> MAX490 driver input -> CLOCK+ and CLOCK-
- DATA+ and DATA- -> MAX490 receiver input -> Arduino digital input

Typical MAX490 module wiring for Arduino Uno, Nano or Mega:

```text
                 MAX490
         +----------------------+
 +5V ----| VCC              TX+ |---- CLOCK+ -> Encoder Pin 2
 D13 ----| DI               TX- |---- CLOCK- -> Encoder Pin 3
 D12 ----| RO               RX+ |---- DATA+  -> Encoder Pin 4
 GND ----| GND              RX- |---- DATA-  -> Encoder Pin 5
         +----------------------+
          DE and RE tied for always-on transmit and receive.
```

BitBang example pins on Uno or Nano: D8 = clock, D9 = data.

Hardware SPI on classic Arduino boards:

```text
Uno / Nano:  D13 = SCK, D12 = MISO, D10 = SS (OUTPUT, HIGH)
Mega 2560:   D52 = SCK, D50 = MISO, D53 = SS (OUTPUT, HIGH)
MOSI is not used for SSI.
```

If a 3.3 V RS422 transceiver is used with a 5 V Arduino board, proper level shifting is required.

## ADM3490ARZ Connection for ESP32 / 3.3 V Boards

```text
                 ADM3490ARZ
          +----------------------+
 +3V3 ----| 1 VCC            A 8 |---- DATA+  -> Encoder Pin 4
 IO9  ----| 2 RO             B 7 |---- DATA-  -> Encoder Pin 5
 IO8  ----| 3 DI             Z 6 |---- CLOCK- -> Encoder Pin 3
 GND  ----| 4 GND            Y 5 |---- CLOCK+ -> Encoder Pin 2
          +----------------------+
```

Default example pin assignment:

```text
IO8  -> SSI clock output
IO9  -> SSI data input
```

## Connection at the Rohde & Schwarz RD130 Rotor

### Data Connector

```text
Data connector:

1 ---> VCC 11-30 V + SET input via push button to VCC -- set encoder to 0
2 ---> CLOCK IN +
3 ---> CLOCK IN -
4 ---> DATA OUT +
5 ---> DATA OUT -
6 ---> GND + Code Sense 0 = CW 1 = CCW
8 ---> Shield
```

### Motor Connector

```text
Motor connector:

1 ---> Motor 1 +
2 ---> Motor 1 -
3 ---> Filter 1 ground
4 ---> Motor 2 -
5 ---> Motor 2 +
6 ---> Filter 2 ground
```

## Important Notes

```text
Important notes:
- This is an unofficial project and is not affiliated with Rohde & Schwarz.
- Never connect CLOCK+/CLOCK- or DATA+/DATA- directly to Arduino or ESP32 GPIO pins.
- Use the ADM3490ARZ for 3.3 V boards or another suitable full duplex RS422 transceiver.
- For 5 V Arduino boards, use a 5 V full duplex RS422 transceiver such as the MAX490.
- For quick tests, search for a ready-made MAX490 RS422 TTL Full Duplex Modul.
- Connect microcontroller GND, RS422 transceiver GND and encoder 0 V together.
- Keep CLOCK+/CLOCK- and DATA+/DATA- as twisted or closely coupled pairs.
- Use a 100 nF decoupling capacitor close to the RS422 transceiver VCC and GND pins.
- For the DATA pair, a 120 ohm termination close to the receiver is recommended.
- For longer CLOCK lines, a 120 ohm termination at the encoder side may be required.
```

## Reading Modes

The library supports three SSI read modes plus an optional ESP32 background task:

| Mode | Boards | Clock pulses | API | Example sketch |
|------|--------|--------------|-----|----------------|
| BitBang | All Arduino-compatible | Exactly 13 | `begin()` / `beginBitBang()` | `RD130_BitBang`, `RD130_ADM3490_Diagnose` |
| Portable Arduino SPI | Arduino AVR, ESP32 | Usually 16 (use `setSpiRightShift(3)`) | `beginSPI(...)` | `RD130_ArduinoSPI` |
| ESP32 precise SPI | ESP32 only | Exactly 13 | `beginESP32PreciseSPI(...)` | `RD130_ESP32_PreciseSPI` |
| Background read + BitBang | ESP32 only | Exactly 13 | `startBackgroundRead(...)` after `beginBitBang()` | `RD130_ESP32_Background` |
| Background read + ESP32 precise SPI | ESP32 only | Exactly 13 | `startBackgroundRead(...)` after `beginESP32PreciseSPI(...)` | `RD130_ESP32_SPI_Background` |

SSI is not a continuous PWM-like clock. The clock is generated only during a read telegram and must be followed by a frame pause.

Quick selection guide:

- **Arduino Uno / Nano / Mega:** BitBang or portable Arduino SPI with MAX490
- **ESP32 / ESP32-S3:** BitBang, portable Arduino SPI (FSPI), ESP32 precise SPI, or either background example
- **Exactly 13 SSI clocks on scope:** BitBang, ESP32 precise SPI, or background examples (not portable Arduino SPI without bit shifting)
- **Non-blocking read on ESP32:** `RD130_ESP32_Background` (BitBang) or `RD130_ESP32_SPI_Background` (hardware SPI via ESP-IDF)

### Reading Structure

```cpp
TWK_KBE58_SSI::Reading reading = encoder.read();

if (reading.valid)
{
  Serial.println(reading.position);
  Serial.println(reading.angleDegRounded, 1);
}
```

Fields:

- `dataIdleLevel`, `rawValue`, `grayValue`, `position`, `trailingBits`
- `stepsPerRevolution` (4096 for the default encoder)
- `angleDeg`, `angleDegRounded`
- `valid` — false for discarded startup reads or failed transfers
- `readCounter`, `errorCounter`

### Startup Warmup and Discard

After `begin*()`, the library performs silent warmup reads and marks the first user-visible reads as invalid (`valid = false`) so startup noise does not appear as real position jumps:

- **BitBang on ESP32:** 1 warmup, 1 discarded read
- **Arduino SPI on AVR (Uno, Nano, Mega):** 2 warmup, 2 discarded reads
- **Arduino SPI on ESP32:** 1 warmup, 1 discarded read
- **ESP32 precise SPI:** 1 warmup, 1 discarded read

Override with:

```cpp
encoder.setStartupDiscardCount(0);
```

### BitBang Mode

BitBang mode is the default and works on all Arduino-compatible boards.

```cpp
encoder.begin();          // same as beginBitBang()
encoder.beginBitBang();
```

Advantages:

- Maximum portability
- Exactly 13 SSI clock pulses
- No SPI pin binding
- Good diagnostic mode
- Used by `RD130_BitBang`, `RD130_ADM3490_Diagnose` and `RD130_ESP32_Background` (BitBang)

Timing defaults: `setHalfPeriodUs(5)`, `setFramePauseUs(80)`.

### Portable Arduino SPI Mode

Arduino SPI mode uses the standard Arduino `SPI` library on **5 V AVR boards** (Uno, Nano, Mega) and on **ESP32** boards.

```cpp
#include <SPI.h>

encoder.beginSPI(SPI, SCK, MISO, 100000);
encoder.setSpiMode(SPI_MODE2);
encoder.setSpiTransferBits(16);
encoder.setSpiRightShift(3);
encoder.setFramePauseUs(80);
```

On ESP32 with free pins:

```cpp
SPIClass ssiSPI(FSPI);
encoder.beginSPI(ssiSPI, PIN_SSI_CLOCK, PIN_SSI_DATA, 100000);
```

On classic Arduino boards, keep SS as output so SPI stays in master mode:

```cpp
encoder.setSpiDummySsPin(10);   // Uno / Nano: D10
// Mega 2560: D53
```

On AVR boards the library uses the board default `SCK`, `MISO` and `SS` symbols.

The `RD130_ArduinoSPI` example reads continuously so SCK pulses stay visible on a scope and prints to Serial every 200 ms.

**Important:** Portable Arduino SPI usually generates 16 clock pulses. The library discards the unused bits with `setSpiRightShift(3)`. For strict 13-clock SSI operation, use BitBang mode or ESP32 precise SPI mode.

Test `SPI_MODE0` through `SPI_MODE3` on your hardware. With clock idle high, start with `SPI_MODE2` or `SPI_MODE3`.

Example sketch: `examples/RD130_ArduinoSPI`

### ESP32 Precise SPI Mode

ESP32 precise SPI mode uses the ESP-IDF SPI master API directly (not `SPIClass`) and generates exactly 13 SSI clock pulses.

```cpp
encoder.setSpiMode(SPI_MODE3);   // RD130 default (CPOL=1, CPHA=1)

if (!encoder.beginESP32PreciseSPI(PIN_SSI_CLOCK, PIN_SSI_DATA, 100000))
{
  Serial.println("ESP32 precise SPI init failed");
}

encoder.setRawBitShift(0);       // optional fine tuning of raw bit alignment
```

Use this mode when portable Arduino SPI produces too many clock edges or unstable raw values on ESP32.

Example sketch: `examples/RD130_ESP32_PreciseSPI`  
Background variant: `examples/RD130_ESP32_SPI_Background`

### ESP32 Background Read

On ESP32, a FreeRTOS task can read the encoder in the background while `loop()` stays free. The task works with whichever read mode is active (`beginBitBang()`, `beginSPI()` or `beginESP32PreciseSPI()`).

Two background examples are provided:

**BitBang** — `RD130_ESP32_Background`:

```cpp
encoder.beginBitBang();
encoder.setHalfPeriodUs(5);
encoder.setFramePauseUs(80);
encoder.startBackgroundRead(10);
```

**ESP32 precise SPI** — `RD130_ESP32_SPI_Background`:

```cpp
encoder.setSpiMode(SPI_MODE3);
encoder.beginESP32PreciseSPI(PIN_SSI_CLOCK, PIN_SSI_DATA, 100000);
encoder.setRawBitShift(0);
encoder.setFramePauseUs(80);
encoder.startBackgroundRead(10);
```

Common background API:

```cpp
if (encoder.hasNewReading())
{
  TWK_KBE58_SSI::Reading reading = encoder.getLastReading();

  if (reading.valid)
  {
    // use reading.position, reading.angleDegRounded, ...
  }
}

encoder.stopBackgroundRead();      // optional cleanup
```

Additional ESP32-only helpers:

```cpp
encoder.getReadCounter();
encoder.getErrorCounter();
```

Example sketches:

- `examples/RD130_BitBang` — minimal BitBang
- `examples/RD130_ArduinoSPI` — portable SPI on Uno, Mega or ESP32
- `examples/RD130_ESP32_PreciseSPI` — minimal ESP32 precise SPI (13 clocks)
- `examples/RD130_ESP32_Background` — BitBang with FreeRTOS background read
- `examples/RD130_ESP32_SPI_Background` — ESP32 precise SPI with FreeRTOS background read
- `examples/RD130_ADM3490_Diagnose` — BitBang with full serial diagnosis

Each example contains the full RD130 wiring notes (ADM3490, MAX490, connector pinout) in the sketch header comment.

### Configuration Reference

Common settings for all modes:

```cpp
encoder.setHalfPeriodUs(5);
encoder.setFramePauseUs(80);
encoder.setClockIdleHigh(true);
encoder.setInvertData(false);
encoder.setStartupDiscardCount(1);
```

Hardware zero-set output (optional):

```cpp
encoder.configureZeroPin(4, 200);  // idle HIGH, pulse LOW for 200 ms
encoder.setZero();                 // trigger hardware encoder reset to zero
encoder.update();                  // call regularly (non-blocking timer)
```

If a relay is connected to the encoder SET input, keep the GPIO HIGH by default.
`setZero()` pulls the pin LOW for the configured pulse duration and then returns HIGH
without blocking `loop()`.

SPI-specific settings:

```cpp
encoder.setSpiFrequency(100000);
encoder.setSpiMode(SPI_MODE2);
encoder.setSpiTransferBits(16);
encoder.setSpiRightShift(3);
encoder.setSpiDummySsPin(10);
```

Low-level access:

```cpp
uint32_t raw = encoder.readRaw();
uint32_t binary = encoder.grayToBinary(grayValue);
TWK_KBE58_SSI::Mode mode = encoder.mode();
```

### Serial Command "zero" in Example Sketches

All shipped examples parse a serial command from the PC:

```text
zero
```

On reception, the sketch calls `encoder.setZero()` and starts a non-blocking LOW pulse
on the configured zero pin (default pin 4 in examples).

## PlatformIO Build

Library version **1.2.0**.

### Root project (default: Diagnose example on ESP32)

```powershell
cd TWK_KBE58_SSI
pio run -e esp32-s3-diagnose
pio run -e esp32dev-diagnose
```

### Individual examples

Each example has its own `platformio.ini`. Build from the example directory:

```powershell
cd examples/RD130_BitBang
pio run -e uno
pio run -e esp32-s3-devkitc-1

cd ../RD130_ArduinoSPI
pio run -e uno
pio run -e mega2560
pio run -e esp32dev

cd ../RD130_ESP32_PreciseSPI
pio run -e esp32-s3-devkitc-1
pio run -e esp32dev

cd ../RD130_ESP32_Background
pio run -e esp32-s3-devkitc-1
pio run -e esp32dev

cd ../RD130_ESP32_SPI_Background
pio run -e esp32-s3-devkitc-1
pio run -e esp32dev

cd ../RD130_ADM3490_Diagnose
pio run -e esp32-s3-devkitc-1
```

Upload and monitor (example: Arduino Uno on COM20):

```powershell
cd examples/RD130_ArduinoSPI
pio run -e uno -t upload --upload-port COM20
pio device monitor -p COM20 -b 115200
```

## Hardware Validation

After wiring the encoder and RS422 transceiver, verify each mode:

1. **BitBang** — position 0…4095 and angle 0.0…359.9 change plausibly when rotating; skip the first invalid startup reads
2. **Arduino SPI (Uno / Mega / ESP32)** — test `SPI_MODE0`…`SPI_MODE3` and `setSpiRightShift(0…3)`; choose stable settings; AVR discards the first two user reads after `beginSPI()`
3. **ESP32 precise SPI** — compare raw values with BitBang mode; exactly 13 clock pulses on the scope
4. **Background read** — `hasNewReading()` updates regularly while `loop()` stays free; compare BitBang (`RD130_ESP32_Background`) and ESP32 precise SPI (`RD130_ESP32_SPI_Background`) at 10 ms interval

## Example Output

The example sketch prints diagnostic values to the serial console:

```text
DATA-Idle: 1 | Raw: 0b0110111110110 | Trailing: 0 | Gray: 0b011011111011 | Position: 1197 / 4096 | Angle: 105.2 deg
```

The output contains:

- `DATA-Idle`: logic level on the data input before the SSI telegram starts
- `Raw`: complete SSI raw value
- `Trailing`: trailing bits after the useful data bits
- `Gray`: extracted Gray code value
- `Position`: binary position after Gray-to-binary conversion
- `Angle`: calculated angle in degrees, rounded to 0.1 degrees

## Encoder Resolution

The assumed encoder resolution is:

```text
4096 positions per revolution
```

This means:

```text
0      -> 0.0 degrees
1024   -> 90.0 degrees
2048   -> 180.0 degrees
3072   -> 270.0 degrees
4095   -> just below 360.0 degrees
```

## Unofficial Project Disclaimer

This is an unofficial, independent open source project.

This project is not developed, published, approved, endorsed, sponsored or supported by Rohde & Schwarz.

There is no partnership, cooperation or official relationship between this project and Rohde & Schwarz.

The names "Rohde & Schwarz", "R&S" and "RD130" are used only to identify the equipment this library was developed and tested for.

All trademarks, product names and company names belong to their respective owners.

## License

Copyright (C) 2026 Joerg Koerner DK8DE

This library is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or any later version.

This library is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
