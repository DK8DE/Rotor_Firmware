# 7Semi-DS18B20-Arduino-Library

This Arduino library provides support for the **7Semi DS18B20 Temperature Sensor Module**, a waterproof 1-wire digital sensor capable of measuring temperatures from -55°C to +125°C with 9–12-bit resolution. It’s ideal for weather monitoring, thermal logging, and embedded sensing applications.

![Arduino](https://img.shields.io/badge/platform-arduino-blue.svg)
![License](https://img.shields.io/badge/license-MIT-green.svg)
![Status](https://img.shields.io/badge/status-active-brightgreen.svg)

---

## Hardware Required

- 7Semi DS18B20 Digital Temperature Sensor Module  
- Arduino-compatible board  
- Digital GPIO pin  

---

## Getting Started

### 1. Installation via Arduino Library Manager

1. Open **Arduino IDE**
2. Go to:
   - `Sketch > Include Library > Manage Libraries…` (Arduino IDE 1.x), or  
   - `Tools > Library Manager…` or click 📚 icon (IDE 2.x)
3. Search for:
   - 7Semi DS18B20
4. Click **Install**

Then in your sketch:

#include <7semi_ds18b20.h>

### 2. Wiring
| DS18B20 Pin | Arduino Pin                                       |
| ----------- | ------------------------------------------------- |
| VCC         | 5V or 3.3V                                        |
| GND         | GND                                               |
| DQ (Data)   | D2                                                |

