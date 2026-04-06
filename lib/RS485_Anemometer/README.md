# RS485_Anemometer

Arduino/ESP32 library to read an RS485 / Modbus-RTU anemometer.

## Features

- Wind speed in **m/s** (0.1 resolution)
- Wind level as an **integer** (typically 1..12)
- Wind direction in **degrees** (0.1°)
- Wind direction as **text**
  - German (default): **O** instead of E, e.g. `ONO`, `O`, `OSO`
  - English: **E**, e.g. `ENE`, `E`, `ESE`

## Wiring (typical)

You need an RS485 transceiver (e.g. MAX485) between the ESP32 and the anemometer.

- ESP32 TX -> RS485 DI
- ESP32 RX -> RS485 RO
- ESP32 DE/RE -> RS485 DE + RE (often tied together)
- GND common
- A/B lines to the sensor

## Usage

See the example sketch: **File → Examples → RS485_Anemometer → RS485_Anemometer_Example**

## Configuration

In the example you can set:

- UART port (`Serial2`)
- RS485 pins (TX/RX/DE+RE)
- Modbus address of the sensor
- Optional offsets:
  - direction offset in degrees
  - speed offset in m/s

## License

MIT
