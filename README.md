# Rotor_Firmware

Firmware für einen motorisierten Antennenrotor auf Basis des ESP32-S3 (PlatformIO/Arduino).

Dieses Projekt steuert den Antennenrotor, verarbeitet Endschalter und Encoder, regelt die Bewegung mit Rampenprofilen und stellt die Kommunikation über RS485 bereit.

## Überblick

Die Firmware läuft auf der Steuerhardware und bietet unter anderem:

- **Präzise Positionsregelung** mit Encoder-Rückführung (Motor- oder Ring-Encoder)
- **Automatisches Homing** mit Endschalter-Erkennung und Backlash-Kompensation
- **RS485-Kommunikation** für Fernsteuerung und Statusabfragen
- **Stromüberwachung** (IS-Messung) zur Blockadeerkennung
- **Temperaturüberwachung** (DS18B20 Sensoren)
- **Windmessung** über Anemometer und optionalen RS485-Windsensor
- **Sicherheitsfunktionen**: Stall-Erkennung, Endschalter-Überwachung, Deadman/Keepalive
- **Persistente Parameter** über NVS/Preferences

## Zusammenspiel mit PC und Controller

Zum Gesamtsystem gehören neben dieser Firmware zwei weitere Projekte:

- **PC-Software (Desktop):** [RotorTcpBridge](https://github.com/DK8DE/RotorTcpBridge)  
  Verbindet Anwendungen am PC mit dem Rotor-System (z. B. über TCP/UDP/seriell, je nach Setup).

- **Controller-Firmware (USB-Bridge + Bedienung):** [Rotor_Display_5](https://github.com/DK8DE/Rotor_Display_5)  
  Stellt über USB eine Brücke zu RS485 bereit und dient als lokale Bedieneinheit.

Für reproduzierbare Ergebnisse sollten Firmware, Controller und PC-Software zueinander passen.

## Hardware

- **MCU**: ESP32-S3 (8MB Flash, 8MB PSRAM)
- **Motorsteuerung**: H-Brücke mit PWM-Ansteuerung
- **Encoder**: Quadratur-Encoder (optional mit Z-Index)
- **Kommunikation**: RS485 (Half-Duplex)
- **Sensoren**: DS18B20 (Temperatur), Anemometer (Windgeschwindigkeit)

## Bauen

### Voraussetzungen

- [PlatformIO](https://platformio.org/) (VS Code Extension oder CLI)
- USB-Kabel für ESP32-S3

### Kompilieren und Flashen

```bash
# Build
pio run

# Build und Upload
pio run --target upload

# Serial Monitor
pio device monitor
```

### Umgebung

Standard-Umgebung: `esp32-s3-n8r8`

```bash
pio run --environment esp32-s3-n8r8
```

Die Zielplattform ist in `platformio.ini` definiert.

## Konfiguration

Die Firmware speichert Konfigurationswerte persistent im NVS (Non-Volatile Storage). Wichtige Parameter:

| Parameter | Beschreibung | Standard |
|-----------|--------------|----------|
| `slaveId` | RS485 Slave-ID | 20 |
| `axisMinDeg01` | Minimale Achsposition (0.01°) | 0 |
| `axisMaxDeg01` | Maximale Achsposition (0.01°) | 36000 (360°) |
| `homeFastPwmPercent` | Homing-Geschwindigkeit (%) | 100 |
| `minPwm` | Mindest-PWM für Bewegung (%) | 25 |
| `stallTimeoutMs` | Timeout für Stall-Erkennung (ms) | 2000 |

### Werksreset

Beide Handspeed-Taster beim Booten gedrückt halten → NVS wird gelöscht und das Gerät startet mit Standardwerten neu.

## RS485-Kommandos

Die Kommunikation erfolgt über RS485 mit konfigurierbarer Slave-ID.

### Grundlegende Kommandos

| Kommando | Beschreibung |
|----------|--------------|
| `GETPOSDG` | Aktuelle Position in 0.01° abfragen |
| `SETPOSDG:<wert>` | Zielposition in 0.01° setzen |
| `GETERR` | Aktuellen Fehlercode abfragen |
| `SETREF` | Fehler quittieren / Referenz setzen |
| `HOME` | Homing-Sequenz starten |
| `STOP` | Bewegung stoppen |

### Konfigurations-Kommandos

| Kommando | Beschreibung |
|----------|--------------|
| `SETID:<id>` | Slave-ID setzen (1–247) |
| `SETROTORID:<id>` | Wie `SETID`, aber **nur** als Broadcast an Adresse `255` (wenn die Slave-ID unbekannt ist). Keine RS485-Antwort. Adressiert man eine konkrete ID, antwortet der Slave mit NAK `NBCAST`. |
| `SETPWM:<prozent>` | Maximale PWM setzen (0–100%) |
| `SETMINPWM:<prozent>` | Mindest-PWM setzen |
| `SETSTALLTO:<ms>` | Stall-Timeout setzen |
| `GETTEMPA` | Umgebungstemperatur abfragen |
| `GETTEMPM` | Motortemperatur abfragen |
| `GETIS` | Strommesswerte abfragen |

## Firmware herunterladen

Fertige Firmware-Binaries werden über GitHub Actions gebaut und als Artefakte bereitgestellt.

### Schritte zum Download

1. Öffne **[Actions → PlatformIO Build & Release](https://github.com/DK8DE/Rotor_Firmware/actions/workflows/platformio-build.yml)**
2. Wähle den neuesten erfolgreichen Workflow-Run
3. Unter **Artifacts** das Archiv `firmware-bin` herunterladen
4. ZIP entpacken

### Dateien im Artifact

| Datei | Beschreibung |
|-------|--------------|
| `firmware.bin` | Haupt-Firmware |
| `bootloader.bin` | Bootloader |
| `partitions.bin` | Partitionstabelle |
| `firmware.elf` | Debug-Symbole |

### Flashen der heruntergeladenen Firmware

Mit **esptool.py** (Python erforderlich; seriellen Port anpassen):

```bash
esptool.py --chip esp32-s3 --port /dev/ttyUSB0 write_flash \
  0x0000 bootloader.bin \
  0x8000 partitions.bin \
  0x10000 firmware.bin
```

Unter Windows z. B. `--port COM7` statt `/dev/ttyUSB0`.

Mit **PlatformIO** (typisch nur `firmware.bin`; Bootloader/Partitionen bleiben erhalten, wenn sie schon passen):

```bash
pio run --target upload --upload-port COM7
```

## Projektstruktur

```
Rotor_Firmware/
├── src/
│   └── main.cpp              # Hauptanwendung
├── lib/                      # Lokale Bibliotheken
│   ├── HalBoard/             # Hardware-Abstraktion
│   ├── MotorMcpwm/           # Motorsteuerung
│   ├── EncoderAxis/          # Encoder-Verarbeitung
│   ├── Rs485Proto/           # RS485-Protokoll
│   ├── MotionController/     # Positionsregelung
│   ├── HomingController/     # Homing-Logik
│   ├── SafetyMonitor/        # Sicherheitsüberwachung
│   ├── LoadMonitor/          # Last-/Wind-Statistik
│   └── TempSensors/          # Temperaturmessung
├── platformio.ini            # PlatformIO-Konfiguration
└── .github/workflows/        # CI/CD (GitHub Actions)
```

## Lizenz

Dieses Projekt ist Open Source. Einzelheiten siehe die Lizenzdateien im Repository.

## Hinweise

- Bei Verbindungsproblemen zuerst Verkabelung, RS485-Adressierung und Baudrate prüfen.
- Bei Fragen oder Fehlern ein Issue im [GitHub-Repository](https://github.com/DK8DE/Rotor_Firmware) erstellen.
