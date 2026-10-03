# Rotor_Firmware

**Version: 1.5.0**

Firmware fÃƒÂ¼r einen motorisierten Antennenrotor auf Basis des ESP32-S3 (PlatformIO/Arduino).

Dieses Projekt steuert den Antennenrotor, verarbeitet Endschalter und Encoder, regelt die Bewegung mit Rampenprofilen und stellt die Kommunikation ÃƒÂ¼ber RS485 bereit.

## ÃƒÅ“berblick

Die Firmware lÃƒÂ¤uft auf der Steuerhardware und bietet unter anderem:

- **PrÃƒÂ¤zise Positionsregelung** mit Encoder-RÃƒÂ¼ckfÃƒÂ¼hrung
  - Typ 1: Motorachsen-Encoder (PCNT)
  - Typ 2: Ring-/Abtriebs-Encoder (PCNT, optional Z-Index)
  - Typ 3: TWK KBE58 SSI-Absolutencoder (kein Homing ÃƒÂ¼ber Endschalter)
- **Arbeitsbereich bis 720Ã‚Â°** bei Typ 3 (Turn-ZÃƒÂ¤hler 0/1, NVS-gesichert)
- **Feinjustage-Offset** `SETDGCAL` / `GETDGCAL` (Ã¢Ë†â€™360Ã¢â‚¬Â¦+360Ã‚Â°) fÃƒÂ¼r GETPOSDG/SETPOSDG
- **Automatisches Homing** (Typ 1/2) mit Endschaltern und Backlash-Kompensation
- **RS485-Kommunikation** fÃƒÂ¼r Fernsteuerung und Statusabfragen
- **StromÃƒÂ¼berwachung** (IS-Messung) zur Blockadeerkennung
- **TemperaturÃƒÂ¼berwachung** (DS18B20)
- **Windmessung** ÃƒÂ¼ber Anemometer / optionalen RS485-Windsensor (bei Typ 3 eingeschrÃƒÂ¤nkt, da Pins fÃƒÂ¼r SSI genutzt werden)
- **Last-/Baseline-Analyse** (LoadMonitor, Kalibrierfahrt ÃƒÂ¼ber eine Getriebe-Umdrehung)
- **Sicherheitsfunktionen**: Stall-Erkennung, Endschalter, Deadman/Keepalive
- **Persistente Parameter** ÃƒÂ¼ber NVS/Preferences

## Versionierung

Die Firmware-Versionsnummer (Semantic Versioning: `MAJOR.MINOR.PATCH`) wird an **einer** zentralen Stelle gepflegt: [`src/Version.h`](src/Version.h).

- Wird beim Boot unabhÃƒÂ¤ngig von `g_debug` einmal ÃƒÂ¼ber USB-Serial ausgegeben (`Rotor_Firmware v1.5.0`).
- ÃƒÅ“ber RS485 per **`GETVERSION`** abfragbar (`ACK_GETVERSION:1.5.0`).
- `build.ps1` liest die Version aus `src/Version.h` und ÃƒÂ¼bernimmt sie (zusammen mit dem Build-Zeitstempel) in `IMGs/manifest.json` (Feld `version`).

### Version setzen (`build.ps1 -Version`)

Die Version wird **nicht** manuell in `src/Version.h` editiert, sondern ÃƒÂ¼ber den Build-Skript-Parameter `-Version` gesetzt:

```powershell
.\build.ps1 -Version "1.5.0"
```

Das erledigt automatisch:

1. `src/Version.h` wird auf `1.5.0` aktualisiert (`FW_VERSION_MAJOR/MINOR/PATCH`).
2. **`README.md` wird automatisch mitaktualisiert** Ã¢â‚¬â€ alle Stellen, die die alte Versionsnummer enthalten (Badge oben, Beispiele im Versionierungs-Abschnitt), werden auf die neue Version umgeschrieben.
3. Firmware wird gebaut, `IMGs/` aktualisiert und (ohne `-SkipUpload`) geflasht.

Auch ohne `-Version`-Parameter gleicht `build.ps1` bei jedem Lauf die `README.md` automatisch mit dem aktuellen Stand von `src/Version.h` ab (z. B. falls die Datei direkt bearbeitet wurde).

Ãƒâ€žnderungen sollten committet und nach `main`/`master` gepusht werden, damit sie versioniert nachvollziehbar sind:

```powershell
git add src/Version.h README.md
git commit -m "Version 1.5.0"
git push
```

### Automatisches GitHub-Release bei VersionsÃƒÂ¤nderung

Der Workflow [`.github/workflows/platformio-build.yml`](.github/workflows/platformio-build.yml) baut die Firmware bei jedem Push/PR (`build`-Job) und prÃƒÂ¼ft danach (`release`-Job), ob sich die Version in `src/Version.h` seit dem letzten Release geÃƒÂ¤ndert hat:

- Bei einem Push nach `main`/`master` wird die aktuelle Version ausgelesen und geprÃƒÂ¼ft, ob dafÃƒÂ¼r bereits ein Git-Tag `vMAJOR.MINOR.PATCH` existiert.
- **Existiert kein Tag** (= Version wurde erhÃƒÂ¶ht) Ã¢â€ â€™ es wird automatisch der Tag `vX.Y.Z` erstellt und gepusht **und** ein GitHub-Release mit `firmware.bin`, `bootloader.bin`, `partitions.bin`, `boot_app0.bin`, `manifest.json` (ESP Web Tools) sowie einem gepackten ZIP angehÃƒÂ¤ngt.
- **Existiert der Tag bereits** (= Version unverÃƒÂ¤ndert) Ã¢â€ â€™ es passiert nichts weiter, es gibt kein doppeltes Release.
- Alternativ kann ein Release auch klassisch per manuellem Tag-Push ausgelÃƒÂ¶st werden: `git tag v1.5.0 && git push --tags`.

Beim Ãƒâ€žndern der Firmware sollte die Version daher immer erhÃƒÂ¶ht werden (`PATCH` fÃƒÂ¼r Bugfixes, `MINOR` fÃƒÂ¼r neue Features, `MAJOR` fÃƒÂ¼r inkompatible Ãƒâ€žnderungen an RS485-Protokoll oder NVS-Layout) Ã¢â‚¬â€ sonst wird beim Push kein neues Release erzeugt.

## Zusammenspiel mit PC und Controller

Zum Gesamtsystem gehÃƒÂ¶ren neben dieser Firmware zwei weitere Projekte:

- **PC-Software (Desktop):** [RotorTcpBridge](https://github.com/DK8DE/RotorTcpBridge)  
  Verbindet Anwendungen am PC mit dem Rotor-System (z. B. ÃƒÂ¼ber TCP/UDP/seriell, je nach Setup).

- **Controller-Firmware (USB-Bridge + Bedienung):** [Rotor_Display_5](https://github.com/DK8DE/Rotor_Display_5)  
  Stellt ÃƒÂ¼ber USB eine BrÃƒÂ¼cke zu RS485 bereit und dient als lokale Bedieneinheit.

FÃƒÂ¼r reproduzierbare Ergebnisse sollten Firmware, Controller und PC-Software zueinander passen.

## Hardware

- **MCU**: ESP32-S3 (8Ã¢â‚¬Â¯MB Flash, 8Ã¢â‚¬Â¯MB PSRAM) Ã¢â‚¬â€ Umgebung `esp32-s3-n8r8`
- **Motorsteuerung**: H-BrÃƒÂ¼cke mit MCPWM
- **Encoder**:
  - Quadratur (A/B, optional Z) Ã¢â‚¬â€ Typ 1/2
  - TWK KBE58 SSI absolut Ã¢â‚¬â€ Typ 3
- **Kommunikation**: RS485 (Half-Duplex)
- **Sensoren**: DS18B20 (Temperatur), Anemometer / Windrichtung (soweit Hardware/Pinbelegung es zulÃƒÂ¤sst)

## Bauen

### Voraussetzungen

- [PlatformIO](https://platformio.org/) (VS Code Extension oder CLI)
- USB-Kabel fÃƒÂ¼r ESP32-S3
- Unter Windows: PowerShell 5.1+

### Empfohlen: `build.ps1`

```powershell
# Build, IMGs aktualisieren, Upload
.\build.ps1

# Nur Build + IMGs (kein Flash) Ã¢â‚¬â€ z. B. fÃƒÂ¼r ESP Web Tools
.\build.ps1 -SkipUpload

# Zuerst clean
.\build.ps1 -Clean
.\build.ps1 -Clean -SkipUpload

# Version setzen (aktualisiert src/Version.h + README.md), dann bauen/flashen
.\build.ps1 -Version "1.5.0"
```

Details zur Versionsvergabe und zum automatischen GitHub-Release: siehe [Versionierung](#versionierung).

`build.ps1` legt bzw. erneuert den Ordner **`IMGs/`** mit den aktuellen Images und einer `manifest.json` fÃƒÂ¼r [ESP Web Tools](https://esphome.github.io/esp-web-tools/):

| Datei | Offset (ESP32-S3) |
|-------|-------------------|
| `bootloader.bin` | `0x0000` |
| `partitions.bin` | `0x8000` |
| `boot_app0.bin` | `0xE000` |
| `firmware.bin` | `0x10000` |
| `manifest.json` | Ã¢â‚¬â€ |

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
| `rty` | Rotor-Typ (Identifikation) | 1=Rotation/Azimut (Default), 2=Elevation 90Ã‚Â°, 3=Elevation 180Ã‚Â°; wird von dieser Firmware nicht ausgewertet |
| `an1` / `an2` / `an3` | Antennennamen 1..3 | max. 9 Zeichen; nur Identifikation fuer Controller, keine Logik im Rotor |
| `asel` | Antennen-Auswahl | 1/2/3 (Default 1); nur Identifikation/Zustand, keine Logik im Rotor; nur bei RotorType 1 (AZ) nutzbar, sonst `DISABLED` |
| `amin` / `amax` | Achsgrenzen (Deg01) | Typ 1/2: max. 360Ã‚Â°; Typ 3: bis **720Ã‚Â°**; ohne gespeichertes `amax` Ã¢â€ â€™ **360Ã‚Â°** |
| `dgcal` | Feinjustage-Offset | Ã¢Ë†â€™360Ã¢â‚¬Â¦+360Ã‚Â°; wirkt auf GETPOSDG/SETPOSDG |
| `dgo` | DGOFFSET | Endschalter-Versatz, nur Typ 1/2 |
| `hpos` | Home-/Parkposition | Ziel des Kommandos `HOME`; Default 0,00Ã‚Â°; GETHOMEPOS/SETHOMEPOS in Kalibrier-Koordinaten wie GETPOSDG |
| `sturn` / `sldeg` | SSI-Turn + letzte logische Lage | Sicherheitskritisch bei >360Ã‚Â° |
| `ect` | Encoder-Typ | 1 / 2 / 3 (Neustart nach Ãƒâ€žnderung) |

### Werksreset

Beide Handspeed-Taster beim Booten gedrÃƒÂ¼ckt halten Ã¢â€ â€™ NVS wird gelÃƒÂ¶scht und das GerÃƒÂ¤t startet mit Standardwerten neu.

## RS485-Kommandos (Auswahl)

Frame-Format: `#src:dst:CMD:params:checksum$`  
Checksumme: `(src + dst) Ãƒâ€” 100 + Wert` (**vorzeichenbehaftet**, z.Ã¢â‚¬Â¯B. bei negativen Params).

Winkelangaben typisch als Grad mit Komma (`12,50` = 12,50Ã‚Â°). Intern: Deg01 (= Grad Ãƒâ€” 100).

### Bewegung / Status

| Kommando | Beschreibung |
|----------|--------------|
| `GETPOSDG` | Position (inkl. DGCAL) |
| `SETPOSDG:<grad>` | Zielposition in Kalibrier-Koordinaten; intern `phys = cal Ã¢Ë†â€™ DGCAL`; ACK = akzeptiertes Cal-Ziel |
| `GETHOMEPOS` / `SETHOMEPOS:<grad>` | Home-/Parkposition lesen/setzen (Kalibrier-Koordinaten wie GETPOSDG/SETPOSDG); NVS `hpos`, Default 0,00Ã‚Â° |
| `HOME` | Faehrt zur gespeicherten Home-Position (`SETHOMEPOS`); erfordert Referenz (NAK `NOREF` sonst) |
| `STOP` | Bewegung stoppen |
| `GETREF` / `SETREF` | Referenzstatus / Fehler quittieren; `SETREF:1` startet das Endschalter-Homing (Typ 1/2) |
| `GETERR` / `GETWARN` | Fehler- / Warncodes |
| `GETHOMING` | Homing aktiv? |

### Achse / Feinjustage

| Kommando | Beschreibung |
|----------|--------------|
| `GETBEGINDG` / `SETBEGINDG` | Achsminimum |
| `GETMAXDG` / `SETMAXDG` | Achsmaximum (Typ 3 bis 720Ã‚Â°); bei Typ 1/2 werden die beim Homing gelernten Encoder-Counts (Endschalter zu Endschalter) auf diesen Winkel verteilt (z.B. 180Ã‚Â° fuer einen Elevationsrotor mit 180Ã‚Â° realem Hub), NICHT fest auf 360Ã‚Â° |
| `GETDGOFFSET` / `SETDGOFFSET` | Endschalter-Offset (Typ 1/2) |
| `GETDGCAL` / `SETDGCAL` | Feinjustage (Ã¢Ë†â€™360Ã¢â‚¬Â¦+360Ã‚Â°), NVS `dgcal` |

### Encoder (Typ 3 / SSI)

| Kommando | Beschreibung |
|----------|--------------|
| `GETENCTYPE` / `SETENCTYPE` | 1=Motor, 2=Ring, 3=SSI (Neustart) |
| `SETENCZERO` | SSI-Hardware-Null (SET0), Turn zurÃƒÂ¼cksetzen |
| `GETENCTURN` / `SETENCTURN` | Soft-Turn 0/1 (ÃƒÅ“berdrehen >360Ã‚Â°) |

### Identifikation / PWM / Sensorik (Auszug)

| Kommando | Beschreibung |
|----------|--------------|
| `GETVERSION` | Firmware-Version (z. B. `1.5.0`), siehe [Versionierung](#versionierung) |
| `GETID` / `SETID` | Slave-ID |
| `SETROTORID` | ID nur per Broadcast `255` setzen |
| `GETROTORTYPE` / `SETROTORTYPE` | Rotor-Typ: 1=Rotation/Azimut, 2=Elevation 90Ã‚Â°, 3=Elevation 180Ã‚Â° (nur Identifikation, NVS `rty`, keine Logik in dieser Firmware) |
| `GETANTNAME1`/`2`/`3` / `SETANTNAME1`/`2`/`3` | Antennennamen (max. 9 Zeichen, keine `:`/`;`); NVS `an1`/`an2`/`an3`, nur fuer den Controller |
| `GETASELECT` / `SETASELECT` | Aktuell gewaehlte Antenne (1/2/3, Default 1); nur Identifikation/Zustand, NVS `asel`, keine Logik in dieser Firmware. Nur bei RotorType 1 (AZ) aktiv Ã¢â‚¬â€ bei RotorType 2/3 (EL) `NAK ...:DISABLED` |
| `GETTEMPA` / `GETTEMPM` | Umgebungs- / Motortemperatur |
| `GETIS` | Strommesswert (nach Offset) |
| `GETWIND` / `SETWINDENABLE` | Wind (soweit Hardware) |

Weitere Kommandos (Homing-PWM, Stall, Load-Bins, Antennenanzeige, Ã¢â‚¬Â¦) siehe Implementierung in `src/Rs485Dispatcher.cpp`.

## Encoder Typ 3: Bereich >360Ã‚Â°

Der SSI-Encoder liefert nur 0Ã¢â‚¬Â¦360Ã‚Â°. Ein Soft-Turn (0/1) erweitert den logischen Bereich bis `amax` (max. 720Ã‚Â°):

- `GETPOSDG` = Rohwinkel + TurnÃƒâ€”360Ã‚Â° (geclampt auf `amax`)
- Turn wird bei Wrap erkannt und **sofort** in NVS geschrieben (`sturn`, Fallback `sldeg`)
- Nach Stromausfall im ÃƒÅ“berdrehbereich muss wieder die logische Lage (>360Ã‚Â°) erscheinen Ã¢â‚¬â€ sonst Kabelbruch-Risiko

## Firmware herunterladen

Fertige Binaries kommen ÃƒÂ¼ber GitHub Actions oder lokal aus `IMGs/`.

### GitHub Actions

1. **[Actions Ã¢â€ â€™ PlatformIO Build & Release](https://github.com/DK8DE/Rotor_Firmware/actions/workflows/platformio-build.yml)**
2. Neuesten erfolgreichen Run ÃƒÂ¶ffnen
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

Unter Windows z.Ã¢â‚¬Â¯B. `--port COM7`. `boot_app0.bin` liegt nach dem Build in `IMGs/`.

**PlatformIO:**

```bash
pio run -t upload -e esp32-s3-n8r8 --upload-port COM7
```

## Projektstruktur

```
Rotor_Firmware/
Ã¢â€Å“Ã¢â€â‚¬Ã¢â€â‚¬ src/                      # Anwendung (main, Motion, Encoder, RS485, Safety, Ã¢â‚¬Â¦)
Ã¢â€Å“Ã¢â€â‚¬Ã¢â€â‚¬ lib/                      # Lokale Bibliotheken (falls vorhanden)
Ã¢â€Å“Ã¢â€â‚¬Ã¢â€â‚¬ IMGs/                     # Aktuelle Flash-Images + manifest.json (via build.ps1)
Ã¢â€Å“Ã¢â€â‚¬Ã¢â€â‚¬ build.ps1                 # Build, IMGs, optional Upload
Ã¢â€Å“Ã¢â€â‚¬Ã¢â€â‚¬ platformio.ini
Ã¢â€â€Ã¢â€â‚¬Ã¢â€â‚¬ .github/workflows/        # CI/CD
```

## Lizenz

Dieses Projekt ist Open Source. Einzelheiten siehe die Lizenzdateien im Repository.

## Hinweise

- Bei Verbindungsproblemen zuerst Verkabelung, RS485-Adressierung, Baudrate und Checksumme prÃƒÂ¼fen (auch bei negativen Parametern).
- Nach `SETENCTYPE` ist ein Neustart nÃƒÂ¶tig.
- Bei Fragen oder Fehlern ein Issue im [GitHub-Repository](https://github.com/DK8DE/Rotor_Firmware) erstellen.
