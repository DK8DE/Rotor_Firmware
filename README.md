# Rotor_Firmware

**Version: 1.6.5**

Firmware fÃƒÆ’Ã‚Â¼r einen motorisierten Antennenrotor auf Basis des ESP32-S3 (PlatformIO/Arduino).

Dieses Projekt steuert den Antennenrotor, verarbeitet Endschalter und Encoder, regelt die Bewegung mit Rampenprofilen und stellt die Kommunikation ÃƒÆ’Ã‚Â¼ber RS485 bereit.

## ÃƒÆ’Ã…â€œberblick

Die Firmware lÃƒÆ’Ã‚Â¤uft auf der Steuerhardware und bietet unter anderem:

- **PrÃƒÆ’Ã‚Â¤zise Positionsregelung** mit Encoder-RÃƒÆ’Ã‚Â¼ckfÃƒÆ’Ã‚Â¼hrung
  - Typ 1: Motorachsen-Encoder (PCNT)
  - Typ 2: Ring-/Abtriebs-Encoder (PCNT, optional Z-Index)
  - Typ 3: TWK KBE58 SSI-Absolutencoder (kein Homing ÃƒÆ’Ã‚Â¼ber Endschalter)
- **Arbeitsbereich bis 720Ãƒâ€šÃ‚Â°** bei Typ 3 (Turn-ZÃƒÆ’Ã‚Â¤hler 0/1, NVS-gesichert)
- **Feinjustage-Offset** `SETDGCAL` / `GETDGCAL` (ÃƒÂ¢Ã‹â€ Ã¢â‚¬â„¢360ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â¦+360Ãƒâ€šÃ‚Â°) fÃƒÆ’Ã‚Â¼r GETPOSDG/SETPOSDG
- **Automatisches Homing** (Typ 1/2) mit Endschaltern und Backlash-Kompensation
- **RS485-Kommunikation** fÃƒÆ’Ã‚Â¼r Fernsteuerung und Statusabfragen
- **StromÃƒÆ’Ã‚Â¼berwachung** (IS-Messung) zur Blockadeerkennung
- **TemperaturÃƒÆ’Ã‚Â¼berwachung** (DS18B20)
- **Windmessung** ÃƒÆ’Ã‚Â¼ber Anemometer / optionalen RS485-Windsensor (bei Typ 3 eingeschrÃƒÆ’Ã‚Â¤nkt, da Pins fÃƒÆ’Ã‚Â¼r SSI genutzt werden)
- **Last-/Baseline-Analyse** (LoadMonitor, Kalibrierfahrt ÃƒÆ’Ã‚Â¼ber eine Getriebe-Umdrehung)
- **Sicherheitsfunktionen**: Stall-Erkennung, Endschalter, Deadman/Keepalive
- **Persistente Parameter** ÃƒÆ’Ã‚Â¼ber NVS/Preferences
- **SoftAP-Konfigurationsseite** (`SETWLAN`): Status, HTTP-OTA-Update, Reboot/Werksreset (Hotspot `Rotor_<UID>`, nicht persistent)

## Versionierung

Die Firmware-Versionsnummer (Semantic Versioning: `MAJOR.MINOR.PATCH`) wird an **einer** zentralen Stelle gepflegt: [`src/Version.h`](src/Version.h).

- Wird beim Boot unabhÃƒÆ’Ã‚Â¤ngig von `g_debug` einmal ÃƒÆ’Ã‚Â¼ber USB-Serial ausgegeben (`Rotor_Firmware v1.6.5`).
- ÃƒÆ’Ã…â€œber RS485 per **`GETVERSION`** abfragbar (`ACK_GETVERSION:1.6.5`).
- `build.ps1` liest die Version aus `src/Version.h` und ÃƒÆ’Ã‚Â¼bernimmt sie in `IMGs/manifest.json` (Feld `version`).

### Version setzen (`build.ps1 -Version`)

Die Version wird **nicht** manuell in `src/Version.h` editiert, sondern ÃƒÆ’Ã‚Â¼ber den Build-Skript-Parameter `-Version` gesetzt:

```powershell
.\build.ps1 -Version "1.6.5"
```

Das erledigt automatisch:

1. `src/Version.h` wird auf `1.6.5` aktualisiert (`FW_VERSION_MAJOR/MINOR/PATCH`).
2. **`README.md` wird automatisch mitaktualisiert** ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â alle Stellen, die die alte Versionsnummer enthalten (Badge oben, Beispiele im Versionierungs-Abschnitt), werden auf die neue Version umgeschrieben.
3. Firmware wird gebaut, `IMGs/` aktualisiert und (ohne `-SkipUpload`) geflasht.

Auch ohne `-Version`-Parameter gleicht `build.ps1` bei jedem Lauf die `README.md` automatisch mit dem aktuellen Stand von `src/Version.h` ab (z. B. falls die Datei direkt bearbeitet wurde).

ÃƒÆ’Ã¢â‚¬Å¾nderungen sollten committet und nach `main`/`master` gepusht werden, damit sie versioniert nachvollziehbar sind:

```powershell
git add src/Version.h README.md
git commit -m "Version 1.6.5"
git push
```

### Automatisches GitHub-Release & Webflasher

Der Workflow [`.github/workflows/platformio-build.yml`](.github/workflows/platformio-build.yml) baut bei jedem Push/PR die Firmware und bereitet Flash-Images vor (wie beim [Inklinometer](https://github.com/DK8DE/Inklinometer_IIS2ICLXTR/releases/tag/v1.3.3)):

- Bei Push nach `main`/`master`: GitHub Pages mit USB-Webflasher unter https://dk8de.github.io/Rotor_Firmware/
- Wenn fÃƒÆ’Ã‚Â¼r die Version aus `src/Version.h` noch kein Release `vMAJOR.MINOR.PATCH` existiert ÃƒÂ¢Ã¢â‚¬Â Ã¢â‚¬â„¢ Release mit:
  - `Rotor-X.Y.Z-factory.bin` ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Å“ Komplettflash ab 0x0 (USB-Webflasher / Erstinstallation)
  - `Rotor-X.Y.Z-firmware.bin` ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Å“ SoftAP-OTA (App, Web-Tab Update)
- Existiert das Release bereits ÃƒÂ¢Ã¢â‚¬Â Ã¢â‚¬â„¢ kein doppeltes Release; Pages/Artifacts werden trotzdem aktualisiert

Beim ÃƒÆ’Ã¢â‚¬Å¾ndern der Firmware die Version erhÃƒÆ’Ã‚Â¶hen (`PATCH` / `MINOR` / `MAJOR`) ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â sonst entsteht kein neues Release.

## Zusammenspiel mit PC und Controller

Zum Gesamtsystem gehÃƒÆ’Ã‚Â¶ren neben dieser Firmware zwei weitere Projekte:

- **PC-Software (Desktop):** [RotorTcpBridge](https://github.com/DK8DE/RotorTcpBridge)  
  Verbindet Anwendungen am PC mit dem Rotor-System (z. B. ÃƒÆ’Ã‚Â¼ber TCP/UDP/seriell, je nach Setup).

- **Controller-Firmware (USB-Bridge + Bedienung):** [Rotor_Display_5](https://github.com/DK8DE/Rotor_Display_5)  
  Stellt ÃƒÆ’Ã‚Â¼ber USB eine BrÃƒÆ’Ã‚Â¼cke zu RS485 bereit und dient als lokale Bedieneinheit.

FÃƒÆ’Ã‚Â¼r reproduzierbare Ergebnisse sollten Firmware, Controller und PC-Software zueinander passen.

## Hardware

- **MCU**: ESP32-S3 (8ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â¯MB Flash, 8ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â¯MB PSRAM) ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â Umgebung `esp32-s3-n8r8`
- **Motorsteuerung**: H-BrÃƒÆ’Ã‚Â¼cke mit MCPWM
- **Encoder**:
  - Quadratur (A/B, optional Z) ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â Typ 1/2
  - TWK KBE58 SSI absolut ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â Typ 3
- **Kommunikation**: RS485 (Half-Duplex)
- **Sensoren**: DS18B20 (Temperatur), Anemometer / Windrichtung (soweit Hardware/Pinbelegung es zulÃƒÆ’Ã‚Â¤sst)

## Bauen

### Voraussetzungen

- [PlatformIO](https://platformio.org/) (VS Code Extension oder CLI)
- USB-Kabel fÃƒÆ’Ã‚Â¼r ESP32-S3
- Unter Windows: PowerShell 5.1+

### Empfohlen: `build.ps1`

```powershell
# Build, IMGs aktualisieren, Upload
.\build.ps1

# Nur Build + IMGs (kein Flash) ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â z. B. fÃƒÆ’Ã‚Â¼r ESP Web Tools
.\build.ps1 -SkipUpload

# Zuerst clean
.\build.ps1 -Clean
.\build.ps1 -Clean -SkipUpload

# Version setzen (aktualisiert src/Version.h + README.md), dann bauen/flashen
.\build.ps1 -Version "1.6.5"
```

Details zur Versionsvergabe und zum automatischen GitHub-Release: siehe [Versionierung](#versionierung).

`build.ps1` legt bzw. erneuert den Ordner **`IMGs/`** und die `manifest.json` fÃƒÆ’Ã‚Â¼r [ESP Web Tools](https://esphome.github.io/esp-web-tools/) (USB-Webflasher, ein Part):

| Datei | Verwendung |
|-------|------------|
| `factory.bin` | Komplettimage ab `0x0` (USB-Webflasher / Erstinstallation) |
| `firmware.bin` | nur App fÃƒÆ’Ã‚Â¼r SoftAP-OTA (Web-Tab Update) |
| `manifest.json` | verweist auf `factory.bin` @ Offset 0 |
| `bootloader.bin` / `partitions.bin` / `boot_app0.bin` | Einzelteile (Debug / manuelles Flashen) |

Die Flasher-Seite liegt unter [`webflasher/`](webflasher/) und wird von CI nach GitHub Pages deployed.

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
| `rty` | Rotor-Typ (Identifikation) | 1=Rotation/Azimut (Default), 2=Elevation 90Ãƒâ€šÃ‚Â°, 3=Elevation 180Ãƒâ€šÃ‚Â°; wird von dieser Firmware nicht ausgewertet |
| `an1` / `an2` / `an3` | Antennennamen 1..3 | max. 9 Zeichen; nur Identifikation fuer Controller, keine Logik im Rotor |
| `asel` | Antennen-Auswahl | 1/2/3 (Default 1); nur Identifikation/Zustand, keine Logik im Rotor; nur bei RotorType 1 (AZ) nutzbar, sonst `DISABLED` |
| `amin` / `amax` | Achsgrenzen (Deg01) | Typ 1/2: max. 360Ãƒâ€šÃ‚Â°; Typ 3: bis **720Ãƒâ€šÃ‚Â°**; ohne gespeichertes `amax` ÃƒÂ¢Ã¢â‚¬Â Ã¢â‚¬â„¢ **360Ãƒâ€šÃ‚Â°** |
| `dgcal` | Feinjustage-Offset | ÃƒÂ¢Ã‹â€ Ã¢â‚¬â„¢360ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â¦+360Ãƒâ€šÃ‚Â°; wirkt auf GETPOSDG/SETPOSDG |
| `dgo` | DGOFFSET | Endschalter-Versatz, nur Typ 1/2 |
| `hpos` | Home-/Parkposition | Ziel des Kommandos `HOME`; Default 0,00Ãƒâ€šÃ‚Â°; GETHOMEPOS/SETHOMEPOS in Kalibrier-Koordinaten wie GETPOSDG |
| `sturn` / `sldeg` | SSI-Turn + letzte logische Lage | Sicherheitskritisch bei >360Ãƒâ€šÃ‚Â° |
| `ect` | Encoder-Typ | 1 / 2 / 3 (Neustart nach ÃƒÆ’Ã¢â‚¬Å¾nderung) |

### Werksreset

Beide Handspeed-Taster beim Booten gedrÃƒÆ’Ã‚Â¼ckt halten ÃƒÂ¢Ã¢â‚¬Â Ã¢â‚¬â„¢ NVS wird gelÃƒÆ’Ã‚Â¶scht und das GerÃƒÆ’Ã‚Â¤t startet mit Standardwerten neu.

## RS485-Kommandos (Auswahl)

Frame-Format: `#src:dst:CMD:params:checksum$`  
Checksumme: `(src + dst) ÃƒÆ’Ã¢â‚¬â€ 100 + Wert` (**vorzeichenbehaftet**, z.ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â¯B. bei negativen Params).

Winkelangaben typisch als Grad mit Komma (`12,50` = 12,50Ãƒâ€šÃ‚Â°). Intern: Deg01 (= Grad ÃƒÆ’Ã¢â‚¬â€ 100).

### Bewegung / Status

| Kommando | Beschreibung |
|----------|--------------|
| `GETPOSDG` | Position (inkl. DGCAL) |
| `SETPOSDG:<grad>` | Zielposition in Kalibrier-Koordinaten; intern `phys = cal ÃƒÂ¢Ã‹â€ Ã¢â‚¬â„¢ DGCAL`; ACK = akzeptiertes Cal-Ziel |
| `GETHOMEPOS` / `SETHOMEPOS:<grad>` | Home-/Parkposition lesen/setzen (Kalibrier-Koordinaten wie GETPOSDG/SETPOSDG); NVS `hpos`, Default 0,00Ãƒâ€šÃ‚Â° |
| `HOME` | Faehrt zur gespeicherten Home-Position (`SETHOMEPOS`); erfordert Referenz (NAK `NOREF` sonst) |
| `STOP` | Bewegung stoppen |
| `GETREF` / `SETREF` | Referenzstatus / Fehler quittieren; `SETREF:1` startet das Endschalter-Homing (Typ 1/2) |
| `GETERR` / `GETWARN` | Fehler- / Warncodes |
| `GETHOMING` | Homing aktiv? |

### Achse / Feinjustage

| Kommando | Beschreibung |
|----------|--------------|
| `GETBEGINDG` / `SETBEGINDG` | Achsminimum |
| `GETMAXDG` / `SETMAXDG` | Achsmaximum (Typ 3 bis 720Ãƒâ€šÃ‚Â°); bei Typ 1/2 werden die beim Homing gelernten Encoder-Counts (Endschalter zu Endschalter) auf diesen Winkel verteilt (z.B. 180Ãƒâ€šÃ‚Â° fuer einen Elevationsrotor mit 180Ãƒâ€šÃ‚Â° realem Hub), NICHT fest auf 360Ãƒâ€šÃ‚Â° |
| `GETDGOFFSET` / `SETDGOFFSET` | Endschalter-Offset (Typ 1/2) |
| `GETDGCAL` / `SETDGCAL` | Feinjustage (ÃƒÂ¢Ã‹â€ Ã¢â‚¬â„¢360ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â¦+360Ãƒâ€šÃ‚Â°), NVS `dgcal` |

### Encoder (Typ 3 / SSI)

| Kommando | Beschreibung |
|----------|--------------|
| `GETENCTYPE` / `SETENCTYPE` | 1=Motor, 2=Ring, 3=SSI (Neustart) |
| `SETENCZERO` | SSI-Hardware-Null (SET0), Turn zurÃƒÆ’Ã‚Â¼cksetzen |
| `GETENCTURN` / `SETENCTURN` | Soft-Turn 0/1 (ÃƒÆ’Ã…â€œberdrehen >360Ãƒâ€šÃ‚Â°) |

### Identifikation / PWM / Sensorik (Auszug)

| Kommando | Beschreibung |
|----------|--------------|
| `GETVERSION` | Firmware-Version (z. B. `1.6.5`), siehe [Versionierung](#versionierung) |
| `GETWLAN` / `SETWLAN:0` / `SETWLAN:1` | SoftAP-Web-UI: `1` = Hotspot `Rotor_<UID>` an (Passwort `Rotorconfig`, UI `admin`/`Rotorconfig` @ 192.168.4.1), `0` = aus. Nicht persistent ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â nach Reboot immer aus. HTTP-OTA unter Tab Update. Hinweis: WiFi kann die Strommessung (ADC2) beeintrÃƒÆ’Ã‚Â¤chtigen. |
| `GETID` / `SETID` | Slave-ID |
| `SETROTORID` | ID nur per Broadcast `255` setzen |
| `GETROTORTYPE` / `SETROTORTYPE` | Rotor-Typ: 1=Rotation/Azimut, 2=Elevation 90Ãƒâ€šÃ‚Â°, 3=Elevation 180Ãƒâ€šÃ‚Â° (nur Identifikation, NVS `rty`, keine Logik in dieser Firmware) |
| `GETANTNAME1`/`2`/`3` / `SETANTNAME1`/`2`/`3` | Antennennamen (max. 9 Zeichen, keine `:`/`;`); NVS `an1`/`an2`/`an3`, nur fuer den Controller |
| `GETASELECT` / `SETASELECT` | Aktuell gewaehlte Antenne (1/2/3, Default 1); nur Identifikation/Zustand, NVS `asel`, keine Logik in dieser Firmware. Nur bei RotorType 1 (AZ) aktiv ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â bei RotorType 2/3 (EL) `NAK ...:DISABLED` |
| `GETTEMPA` / `GETTEMPM` | Umgebungs- / Motortemperatur |
| `GETIS` | Strommesswert (nach Offset) |
| `GETWIND` / `SETWINDENABLE` | Wind (soweit Hardware) |

Weitere Kommandos (Homing-PWM, Stall, Load-Bins, Antennenanzeige, ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â¦) siehe Implementierung in `src/Rs485Dispatcher.cpp`.

## Encoder Typ 3: Bereich >360Ãƒâ€šÃ‚Â°

Der SSI-Encoder liefert nur 0ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â¦360Ãƒâ€šÃ‚Â°. Ein Soft-Turn (0/1) erweitert den logischen Bereich bis `amax` (max. 720Ãƒâ€šÃ‚Â°):

- `GETPOSDG` = Rohwinkel + TurnÃƒÆ’Ã¢â‚¬â€360Ãƒâ€šÃ‚Â° (geclampt auf `amax`)
- Turn wird bei Wrap erkannt und **sofort** in NVS geschrieben (`sturn`, Fallback `sldeg`)
- Nach Stromausfall im ÃƒÆ’Ã…â€œberdrehbereich muss wieder die logische Lage (>360Ãƒâ€šÃ‚Â°) erscheinen ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â sonst Kabelbruch-Risiko

## Firmware herunterladen & flashen

### USB-Webflasher (GitHub Pages)

https://dk8de.github.io/Rotor_Firmware/ ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â Chrome/Edge, USB, schreibt `factory.bin` (Komplettflash).

Lokal: `webflasher/index.html` + `IMGs/` (nach `.\build.ps1 -SkipUpload`).

### SoftAP-OTA (WLAN-Update ÃƒÆ’Ã‚Â¼ber die interne Seite)

1. RS485: `SETWLAN:1` ÃƒÂ¢Ã¢â‚¬Â Ã¢â‚¬â„¢ Hotspot `Rotor_<UID>` / Passwort `Rotorconfig`
2. Browser: http://192.168.4.1 (Login `admin` / `Rotorconfig`)
3. Tab **Update** ÃƒÂ¢Ã¢â‚¬Â Ã¢â‚¬â„¢ nur `firmware.bin` hochladen (nicht `factory.bin`)

### GitHub Releases / Actions

- Releases: [github.com/DK8DE/Rotor_Firmware/releases](https://github.com/DK8DE/Rotor_Firmware/releases) (`*-factory.bin`, `*-firmware.bin`)
- Artifacts: [Actions ÃƒÂ¢Ã¢â‚¬Â Ã¢â‚¬â„¢ Build Webflasher & Release](https://github.com/DK8DE/Rotor_Firmware/actions/workflows/platformio-build.yml)

### esptool / PlatformIO

```bash
# Komplettflash (wie Webflasher)
esptool.py --chip esp32s3 --port COM7 write_flash 0x0 factory.bin

# oder PlatformIO-Upload
pio run -t upload -e esp32-s3-n8r8 --upload-port COM7
```

## Projektstruktur

```
Rotor_Firmware/
ÃƒÂ¢Ã¢â‚¬ÂÃ…â€œÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ src/                      # Anwendung (main, Motion, Encoder, RS485, Safety, ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â¦)
ÃƒÂ¢Ã¢â‚¬ÂÃ…â€œÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ lib/                      # Lokale Bibliotheken
ÃƒÂ¢Ã¢â‚¬ÂÃ…â€œÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ webflasher/               # USB-Webflasher-Seite (ÃƒÂ¢Ã¢â‚¬Â Ã¢â‚¬â„¢ GitHub Pages)
ÃƒÂ¢Ã¢â‚¬ÂÃ…â€œÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ IMGs/                     # factory.bin, firmware.bin, manifest.json (via build.ps1)
ÃƒÂ¢Ã¢â‚¬ÂÃ…â€œÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ scripts/read_version.sh   # Version aus src/Version.h (CI)
ÃƒÂ¢Ã¢â‚¬ÂÃ…â€œÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ build.ps1                 # Build, IMGs, optional Upload
ÃƒÂ¢Ã¢â‚¬ÂÃ…â€œÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ platformio.ini
ÃƒÂ¢Ã¢â‚¬ÂÃ¢â‚¬ÂÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ .github/workflows/        # CI: Build, Release, Pages
```

## Lizenz

Dieses Projekt ist Open Source. Einzelheiten siehe die Lizenzdateien im Repository.

## Hinweise

- Bei Verbindungsproblemen zuerst Verkabelung, RS485-Adressierung, Baudrate und Checksumme prÃƒÆ’Ã‚Â¼fen (auch bei negativen Parametern).
- Nach `SETENCTYPE` ist ein Neustart nÃƒÆ’Ã‚Â¶tig.
- Bei Fragen oder Fehlern ein Issue im [GitHub-Repository](https://github.com/DK8DE/Rotor_Firmware) erstellen.
