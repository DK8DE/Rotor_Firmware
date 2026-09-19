# Rotor_Firmware

**Version: 1.3.1**

Firmware für einen motorisierten Antennenrotor auf Basis des ESP32-S3 (PlatformIO/Arduino).

Dieses Projekt steuert den Antennenrotor, verarbeitet Endschalter und Encoder, regelt die Bewegung mit Rampenprofilen und stellt die Kommunikation über RS485 bereit.

## Überblick

Die Firmware läuft auf der Steuerhardware und bietet unter anderem:

- **Präzise Positionsregelung** mit Encoder-Rückführung
  - Typ 1: Motorachsen-Encoder (PCNT)
  - Typ 2: Ring-/Abtriebs-Encoder (PCNT, optional Z-Index)
  - Typ 3: TWK KBE58 SSI-Absolutencoder (kein Homing über Endschalter)
- **Arbeitsbereich bis 720°** bei Typ 3 (Turn-Zähler 0/1, NVS-gesichert)
- **Feinjustage-Offset** `SETDGCAL` / `GETDGCAL` (−360…+360°) für GETPOSDG/SETPOSDG
- **Automatisches Homing** (Typ 1/2) mit Endschaltern und Backlash-Kompensation
- **RS485-Kommunikation** für Fernsteuerung und Statusabfragen
- **Stromüberwachung** (IS-Messung) zur Blockadeerkennung
- **Temperaturüberwachung** (DS18B20)
- **Windmessung** über Anemometer / optionalen RS485-Windsensor (bei Typ 3 eingeschränkt, da Pins für SSI genutzt werden)
- **Last-/Baseline-Analyse** (LoadMonitor, Kalibrierfahrt über eine Getriebe-Umdrehung)
- **Sicherheitsfunktionen**: Stall-Erkennung, Endschalter, Deadman/Keepalive
- **Persistente Parameter** über NVS/Preferences

## Versionierung

Die Firmware-Versionsnummer (Semantic Versioning: `MAJOR.MINOR.PATCH`) wird an **einer** zentralen Stelle gepflegt: [`src/Version.h`](src/Version.h).

- Wird beim Boot unabhängig von `g_debug` einmal über USB-Serial ausgegeben (`Rotor_Firmware v1.3.1`).
- Über RS485 per **`GETVERSION`** abfragbar (`ACK_GETVERSION:1.3.1`).
- `build.ps1` liest die Version aus `src/Version.h` und übernimmt sie (zusammen mit dem Build-Zeitstempel) in `IMGs/manifest.json` (Feld `version`).

### Version setzen (`build.ps1 -Version`)

Die Version wird **nicht** manuell in `src/Version.h` editiert, sondern über den Build-Skript-Parameter `-Version` gesetzt:

```powershell
.\build.ps1 -Version "1.3.1"
```

Das erledigt automatisch:

1. `src/Version.h` wird auf `1.3.1` aktualisiert (`FW_VERSION_MAJOR/MINOR/PATCH`).
2. **`README.md` wird automatisch mitaktualisiert** — alle Stellen, die die alte Versionsnummer enthalten (Badge oben, Beispiele im Versionierungs-Abschnitt), werden auf die neue Version umgeschrieben.
3. Firmware wird gebaut, `IMGs/` aktualisiert und (ohne `-SkipUpload`) geflasht.

Auch ohne `-Version`-Parameter gleicht `build.ps1` bei jedem Lauf die `README.md` automatisch mit dem aktuellen Stand von `src/Version.h` ab (z. B. falls die Datei direkt bearbeitet wurde).

Änderungen sollten committet und nach `main`/`master` gepusht werden, damit sie versioniert nachvollziehbar sind:

```powershell
git add src/Version.h README.md
git commit -m "Version 1.3.1"
git push
```

### Automatisches GitHub-Release bei Versionsänderung

Der Workflow [`.github/workflows/platformio-build.yml`](.github/workflows/platformio-build.yml) baut die Firmware bei jedem Push/PR (`build`-Job) und prüft danach (`release`-Job), ob sich die Version in `src/Version.h` seit dem letzten Release geändert hat:

- Bei einem Push nach `main`/`master` wird die aktuelle Version ausgelesen und geprüft, ob dafür bereits ein Git-Tag `vMAJOR.MINOR.PATCH` existiert.
- **Existiert kein Tag** (= Version wurde erhöht) → es wird automatisch der Tag `vX.Y.Z` erstellt und gepusht **und** ein GitHub-Release mit `firmware.bin`, `bootloader.bin`, `partitions.bin`, `boot_app0.bin`, `manifest.json` (ESP Web Tools) sowie einem gepackten ZIP angehängt.
- **Existiert der Tag bereits** (= Version unverändert) → es passiert nichts weiter, es gibt kein doppeltes Release.
- Alternativ kann ein Release auch klassisch per manuellem Tag-Push ausgelöst werden: `git tag v1.3.1 && git push --tags`.

Beim Ändern der Firmware sollte die Version daher immer erhöht werden (`PATCH` für Bugfixes, `MINOR` für neue Features, `MAJOR` für inkompatible Änderungen an RS485-Protokoll oder NVS-Layout) — sonst wird beim Push kein neues Release erzeugt.

## Zusammenspiel mit PC und Controller

Zum Gesamtsystem gehören neben dieser Firmware zwei weitere Projekte:

- **PC-Software (Desktop):** [RotorTcpBridge](https://github.com/DK8DE/RotorTcpBridge)  
  Verbindet Anwendungen am PC mit dem Rotor-System (z. B. über TCP/UDP/seriell, je nach Setup).

- **Controller-Firmware (USB-Bridge + Bedienung):** [Rotor_Display_5](https://github.com/DK8DE/Rotor_Display_5)  
  Stellt über USB eine Brücke zu RS485 bereit und dient als lokale Bedieneinheit.

Für reproduzierbare Ergebnisse sollten Firmware, Controller und PC-Software zueinander passen.

## Hardware

- **MCU**: ESP32-S3 (8 MB Flash, 8 MB PSRAM) — Umgebung `esp32-s3-n8r8`
- **Motorsteuerung**: H-Brücke mit MCPWM
- **Encoder**:
  - Quadratur (A/B, optional Z) — Typ 1/2
  - TWK KBE58 SSI absolut — Typ 3
- **Kommunikation**: RS485 (Half-Duplex)
- **Sensoren**: DS18B20 (Temperatur), Anemometer / Windrichtung (soweit Hardware/Pinbelegung es zulässt)

## Bauen

### Voraussetzungen

- [PlatformIO](https://platformio.org/) (VS Code Extension oder CLI)
- USB-Kabel für ESP32-S3
- Unter Windows: PowerShell 5.1+

### Empfohlen: `build.ps1`

```powershell
# Build, IMGs aktualisieren, Upload
.\build.ps1

# Nur Build + IMGs (kein Flash) — z. B. für ESP Web Tools
.\build.ps1 -SkipUpload

# Zuerst clean
.\build.ps1 -Clean
.\build.ps1 -Clean -SkipUpload

# Version setzen (aktualisiert src/Version.h + README.md), dann bauen/flashen
.\build.ps1 -Version "1.3.1"
```

Details zur Versionsvergabe und zum automatischen GitHub-Release: siehe [Versionierung](#versionierung).

`build.ps1` legt bzw. erneuert den Ordner **`IMGs/`** mit den aktuellen Images und einer `manifest.json` für [ESP Web Tools](https://esphome.github.io/esp-web-tools/):

| Datei | Offset (ESP32-S3) |
|-------|-------------------|
| `bootloader.bin` | `0x0000` |
| `partitions.bin` | `0x8000` |
| `boot_app0.bin` | `0xE000` |
| `firmware.bin` | `0x10000` |
| `manifest.json` | — |

### PlatformIO CLI

```bash
# Build
pio run -e esp32-s3-n8r8

# Build und Upload
pio run -t upload -e esp32-s3-n8r8

# Serial Monitor
pio device monitor
```

Upload-/Monitor-Port stehen in `platformio.ini` (bei Bedarf anpassen).

## Konfiguration

Die Firmware speichert Konfigurationswerte persistent im NVS. Wichtige Parameter:

| Parameter / Key | Beschreibung | Hinweis |
|-----------------|--------------|---------|
| Slave-ID | RS485-Adresse | `SETID` / `SETROTORID` (Broadcast 255) |
| `rty` | Rotor-Typ (Identifikation) | 1=Rotation/Azimut (Default), 2=Elevation 90°, 3=Elevation 180°; wird von dieser Firmware nicht ausgewertet |
| `an1` / `an2` / `an3` | Antennennamen 1..3 | max. 9 Zeichen; nur Identifikation fuer Controller, keine Logik im Rotor |
| `amin` / `amax` | Achsgrenzen (Deg01) | Typ 1/2: max. 360°; Typ 3: bis **720°**; ohne gespeichertes `amax` → **360°** |
| `dgcal` | Feinjustage-Offset | −360…+360°; wirkt auf GETPOSDG/SETPOSDG |
| `dgo` | DGOFFSET | Endschalter-Versatz, nur Typ 1/2 |
| `hpos` | Home-/Parkposition | Ziel des Kommandos `HOME`; Default 0,00°; GETHOMEPOS/SETHOMEPOS in Kalibrier-Koordinaten wie GETPOSDG |
| `sturn` / `sldeg` | SSI-Turn + letzte logische Lage | Sicherheitskritisch bei >360° |
| `ect` | Encoder-Typ | 1 / 2 / 3 (Neustart nach Änderung) |

### Werksreset

Beide Handspeed-Taster beim Booten gedrückt halten → NVS wird gelöscht und das Gerät startet mit Standardwerten neu.

## RS485-Kommandos (Auswahl)

Frame-Format: `#src:dst:CMD:params:checksum$`  
Checksumme: `(src + dst) × 100 + Wert` (**vorzeichenbehaftet**, z. B. bei negativen Params).

Winkelangaben typisch als Grad mit Komma (`12,50` = 12,50°). Intern: Deg01 (= Grad × 100).

### Bewegung / Status

| Kommando | Beschreibung |
|----------|--------------|
| `GETPOSDG` | Position (inkl. DGCAL) |
| `SETPOSDG:<grad>` | Zielposition in Kalibrier-Koordinaten; intern `phys = cal − DGCAL`; ACK = akzeptiertes Cal-Ziel |
| `GETHOMEPOS` / `SETHOMEPOS:<grad>` | Home-/Parkposition lesen/setzen (Kalibrier-Koordinaten wie GETPOSDG/SETPOSDG); NVS `hpos`, Default 0,00° |
| `HOME` | Faehrt zur gespeicherten Home-Position (`SETHOMEPOS`); erfordert Referenz (NAK `NOREF` sonst) |
| `STOP` | Bewegung stoppen |
| `GETREF` / `SETREF` | Referenzstatus / Fehler quittieren; `SETREF:1` startet das Endschalter-Homing (Typ 1/2) |
| `GETERR` / `GETWARN` | Fehler- / Warncodes |
| `GETHOMING` | Homing aktiv? |

### Achse / Feinjustage

| Kommando | Beschreibung |
|----------|--------------|
| `GETBEGINDG` / `SETBEGINDG` | Achsminimum |
| `GETMAXDG` / `SETMAXDG` | Achsmaximum (Typ 3 bis 720°); bei Typ 1/2 werden die beim Homing gelernten Encoder-Counts (Endschalter zu Endschalter) auf diesen Winkel verteilt (z.B. 180° fuer einen Elevationsrotor mit 180° realem Hub), NICHT fest auf 360° |
| `GETDGOFFSET` / `SETDGOFFSET` | Endschalter-Offset (Typ 1/2) |
| `GETDGCAL` / `SETDGCAL` | Feinjustage (−360…+360°), NVS `dgcal` |

### Encoder (Typ 3 / SSI)

| Kommando | Beschreibung |
|----------|--------------|
| `GETENCTYPE` / `SETENCTYPE` | 1=Motor, 2=Ring, 3=SSI (Neustart) |
| `SETENCZERO` | SSI-Hardware-Null (SET0), Turn zurücksetzen |
| `GETENCTURN` / `SETENCTURN` | Soft-Turn 0/1 (Überdrehen >360°) |

### Identifikation / PWM / Sensorik (Auszug)

| Kommando | Beschreibung |
|----------|--------------|
| `GETVERSION` | Firmware-Version (z. B. `1.3.1`), siehe [Versionierung](#versionierung) |
| `GETID` / `SETID` | Slave-ID |
| `SETROTORID` | ID nur per Broadcast `255` setzen |
| `GETROTORTYPE` / `SETROTORTYPE` | Rotor-Typ: 1=Rotation/Azimut, 2=Elevation 90°, 3=Elevation 180° (nur Identifikation, NVS `rty`, keine Logik in dieser Firmware) |
| `GETANTNAME1`/`2`/`3` / `SETANTNAME1`/`2`/`3` | Antennennamen (max. 9 Zeichen, keine `:`/`;`); NVS `an1`/`an2`/`an3`, nur fuer den Controller |
| `GETTEMPA` / `GETTEMPM` | Umgebungs- / Motortemperatur |
| `GETIS` | Strommesswert (nach Offset) |
| `GETWIND` / `SETWINDENABLE` | Wind (soweit Hardware) |

Weitere Kommandos (Homing-PWM, Stall, Load-Bins, Antennenanzeige, …) siehe Implementierung in `src/Rs485Dispatcher.cpp`.

## Encoder Typ 3: Bereich >360°

Der SSI-Encoder liefert nur 0…360°. Ein Soft-Turn (0/1) erweitert den logischen Bereich bis `amax` (max. 720°):

- `GETPOSDG` = Rohwinkel + Turn×360° (geclampt auf `amax`)
- Turn wird bei Wrap erkannt und **sofort** in NVS geschrieben (`sturn`, Fallback `sldeg`)
- Nach Stromausfall im Überdrehbereich muss wieder die logische Lage (>360°) erscheinen — sonst Kabelbruch-Risiko

## Firmware herunterladen

Fertige Binaries kommen über GitHub Actions oder lokal aus `IMGs/`.

### GitHub Actions

1. **[Actions → PlatformIO Build & Release](https://github.com/DK8DE/Rotor_Firmware/actions/workflows/platformio-build.yml)**
2. Neuesten erfolgreichen Run öffnen
3. Artifact `firmware-bin` herunterladen und entpacken

| Datei | Beschreibung |
|-------|--------------|
| `firmware.bin` | Haupt-Firmware |
| `bootloader.bin` | Bootloader |
| `partitions.bin` | Partitionstabelle |
| `firmware.elf` | Debug-Symbole |

### Flashen

**ESP Web Tools:** `IMGs/manifest.json` bzw. die Images aus `.\build.ps1 -SkipUpload` verwenden.

**esptool.py:**

```bash
esptool.py --chip esp32-s3 --port /dev/ttyUSB0 write_flash \
  0x0000 bootloader.bin \
  0x8000 partitions.bin \
  0xe000 boot_app0.bin \
  0x10000 firmware.bin
```

Unter Windows z. B. `--port COM7`. `boot_app0.bin` liegt nach dem Build in `IMGs/`.

**PlatformIO:**

```bash
pio run -t upload -e esp32-s3-n8r8 --upload-port COM7
```

## Projektstruktur

```
Rotor_Firmware/
├── src/                      # Anwendung (main, Motion, Encoder, RS485, Safety, …)
├── lib/                      # Lokale Bibliotheken (falls vorhanden)
├── IMGs/                     # Aktuelle Flash-Images + manifest.json (via build.ps1)
├── build.ps1                 # Build, IMGs, optional Upload
├── platformio.ini
└── .github/workflows/        # CI/CD
```

## Lizenz

Dieses Projekt ist Open Source. Einzelheiten siehe die Lizenzdateien im Repository.

## Hinweise

- Bei Verbindungsproblemen zuerst Verkabelung, RS485-Adressierung, Baudrate und Checksumme prüfen (auch bei negativen Parametern).
- Nach `SETENCTYPE` ist ein Neustart nötig.
- Bei Fragen oder Fehlern ein Issue im [GitHub-Repository](https://github.com/DK8DE/Rotor_Firmware) erstellen.
