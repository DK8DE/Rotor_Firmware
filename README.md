# Rotor_Firmware

**Version: 1.7.1**

Firmware für einen motorisierten Antennenrotor auf Basis des ESP32-S3 (PlatformIO/Arduino).

Dieses Projekt steuert den Antennenrotor, verarbeitet Endschalter und Encoder, regelt die Bewegung mit Rampenprofilen und stellt die Kommunikation über RS485 bereit.

## Überblick

Die Firmware läuft auf der Steuerhardware und bietet unter anderem:

- **Präzise Positionsregelung** mit Encoder-Rückführung
  - Typ 1: Motorachsen-Encoder (PCNT)
  - Typ 2: Ring-/Abtriebs-Encoder (PCNT, optional Z-Index)
  - Typ 3: TWK KBE58 SSI-Absolutencoder (kein Homing über Endschalter)
  - Typ 4: Inklinometer IIS2ICLX am Wind-RS485 (Elevation, kein Homing, kein Wind)
- **Arbeitsbereich bis 720°** bei Typ 3 (Turn-Zähler 0/1, NVS-gesichert)
- **Feinjustage-Offset** `SETDGCAL` / `GETDGCAL` (−360…+360°) für GETPOSDG/SETPOSDG
- **Automatisches Homing** (Typ 1/2) mit Endschaltern und Backlash-Kompensation
- **RS485-Kommunikation** für Fernsteuerung und Statusabfragen
- **Stromüberwachung** (IS-Messung) zur Blockadeerkennung
- **Temperaturüberwachung** (DS18B20)
- **Windmessung** über Anemometer / optionalen RS485-Windsensor (bei Typ 3 eingeschränkt wegen SSI-Pins; bei Typ 4 entfällt Wind zugunsten des Inklinometers)
- **Last-/Baseline-Analyse** (LoadMonitor, Kalibrierfahrt über eine Getriebe-Umdrehung)
- **Sicherheitsfunktionen**: Stall-Erkennung, Endschalter, Deadman/Keepalive
- **Persistente Parameter** über NVS/Preferences
- **SoftAP-Konfigurationsseite** (`SETWLAN`): Status, HTTP-OTA-Update, Reboot/Werksreset (Hotspot `Rotor_<UID>`, nicht persistent)

## Versionierung

Die Firmware-Versionsnummer (Semantic Versioning: `MAJOR.MINOR.PATCH`) wird an **einer** zentralen Stelle gepflegt: [`src/Version.h`](src/Version.h).

- Wird beim Boot unabhängig von `g_debug` einmal über USB-Serial ausgegeben (`Rotor_Firmware v1.7.1`).
- Über RS485 per **`GETVERSION`** abfragbar (`ACK_GETVERSION:1.7.1`).
- `build.ps1` liest die Version aus `src/Version.h` und übernimmt sie in `IMGs/manifest.json` (Feld `version`).

### Version setzen (`build.ps1 -Version`)

Die Version wird **nicht** manuell in `src/Version.h` editiert, sondern über den Build-Skript-Parameter `-Version` gesetzt:

```powershell
.\build.ps1 -Version "1.7.1"
```

Das erledigt automatisch:

1. `src/Version.h` wird auf `1.7.1` aktualisiert (`FW_VERSION_MAJOR/MINOR/PATCH`).
2. **`README.md` wird automatisch mitaktualisiert** — alle Stellen, die die alte Versionsnummer enthalten (Badge oben, Beispiele im Versionierungs-Abschnitt), werden auf die neue Version umgeschrieben.
3. Firmware wird gebaut, `IMGs/` aktualisiert und (ohne `-SkipUpload`) geflasht.

Auch ohne `-Version`-Parameter gleicht `build.ps1` bei jedem Lauf die `README.md` automatisch mit dem aktuellen Stand von `src/Version.h` ab (z. B. falls die Datei direkt bearbeitet wurde).

Änderungen sollten committet und nach `main`/`master` gepusht werden, damit sie versioniert nachvollziehbar sind:

```powershell
git add src/Version.h README.md
git commit -m "Version 1.7.1"
git push
```

### Automatisches GitHub-Release & Webflasher

Der Workflow [`.github/workflows/platformio-build.yml`](.github/workflows/platformio-build.yml) baut bei jedem Push/PR die Firmware und bereitet Flash-Images vor (wie beim [Inklinometer](https://github.com/DK8DE/Inklinometer_IIS2ICLXTR/releases/tag/v1.3.3)):

- Bei Push nach `main`/`master`: GitHub Pages mit USB-Webflasher unter https://dk8de.github.io/Rotor_Firmware/
- Wenn für die Version aus `src/Version.h` noch kein Release `vMAJOR.MINOR.PATCH` existiert → Release mit:
  - `Rotor-X.Y.Z-factory.bin` – Komplettflash ab 0x0 (USB-Webflasher / Erstinstallation)
  - `Rotor-X.Y.Z-firmware.bin` – SoftAP-OTA (App, Web-Tab Update)
- Existiert das Release bereits → kein doppeltes Release; Pages/Artifacts werden trotzdem aktualisiert

Beim Ändern der Firmware die Version erhöhen (`PATCH` / `MINOR` / `MAJOR`) — sonst entsteht kein neues Release.

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
.\build.ps1 -Version "1.7.1"
```

Details zur Versionsvergabe und zum automatischen GitHub-Release: siehe [Versionierung](#versionierung).

`build.ps1` legt bzw. erneuert den Ordner **`IMGs/`** und die `manifest.json` für [ESP Web Tools](https://esphome.github.io/esp-web-tools/) (USB-Webflasher, ein Part):

| Datei | Verwendung |
|-------|------------|
| `factory.bin` | Komplettimage ab `0x0` (USB-Webflasher / Erstinstallation) |
| `firmware.bin` | nur App für SoftAP-OTA (Web-Tab Update) |
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
| `rty` | Rotor-Typ (Identifikation) | 1=Rotation/Azimut (Default), 2=Elevation 90°, 3=Elevation 180°; wird von dieser Firmware nicht ausgewertet |
| `an1` / `an2` / `an3` | Antennennamen 1..3 | max. 9 Zeichen; nur Identifikation fuer Controller, keine Logik im Rotor |
| `asel` | Antennen-Auswahl | 1/2/3 (Default 1); nur Identifikation/Zustand, keine Logik im Rotor; nur bei RotorType 1 (AZ) nutzbar, sonst `DISABLED` |
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

### Encoder

| Kommando | Beschreibung |
|----------|--------------|
| `GETENCTYPE` / `SETENCTYPE` | 1=Motorachse, 2=Ring/Abtrieb, 3=SSI, 4=Inklinometer (Neustart) |
| `GETENDSTC` / `SETENDSTC` | Anzahl Endschalter: `0`=keine, `1`=ein Kontakt (L oder R), `2`=zwei Kontakte. Default Typ 1/2 = 2, SSI/Inklinometer = 0. Typ 4 erlaubt nur `0` oder `2` |
| `GETENCCOUNT` | Aktuelle Encoder-Counts (Rohwert) |
| `SETENCZERO` | Typ 1/2: Soft-Zero. Typ 3: SSI-Hardware-Null (SET0) + Turn zurücksetzen. Typ 4: NAK (Kalibrierung am Inklinometer) |
| `GETENCCAX` / `SETENCCAX` | Counts pro 360° für Motorachse (Typ 1) |
| `GETENCCRI` / `SETENCCRI` | Counts pro 360° für Ring/Abtrieb (Typ 2) |
| `GETENCTURN` / `SETENCTURN` | Soft-Turn 0/1 (nur SSI, Überdrehen >360°) |


### Identifikation / PWM / Sensorik (Auszug)

| Kommando | Beschreibung |
|----------|--------------|
| `GETVERSION` | Firmware-Version (z. B. `1.7.1`), siehe [Versionierung](#versionierung) |
| `GETWLAN` / `SETWLAN:0` / `SETWLAN:1` | SoftAP-Web-UI: `1` = Hotspot `Rotor_<UID>` an (Passwort `Rotorconfig`, UI `admin`/`Rotorconfig` @ 192.168.4.1), `0` = aus. Nicht persistent — nach Reboot immer aus. HTTP-OTA unter Tab Update. Hinweis: WiFi kann die Strommessung (ADC2) beeinträchtigen. |
| `GETID` / `SETID` | Slave-ID |
| `SETROTORID` | ID nur per Broadcast `255` setzen |
| `GETROTORTYPE` / `SETROTORTYPE` | Rotor-Typ: 1=Rotation/Azimut, 2=Elevation 90°, 3=Elevation 180° (nur Identifikation, NVS `rty`, keine Logik in dieser Firmware) |
| `GETANTNAME1`/`2`/`3` / `SETANTNAME1`/`2`/`3` | Antennennamen (max. 9 Zeichen, keine `:`/`;`); NVS `an1`/`an2`/`an3`, nur fuer den Controller |
| `GETASELECT` / `SETASELECT` | Aktuell gewaehlte Antenne (1/2/3, Default 1); nur Identifikation/Zustand, NVS `asel`, keine Logik in dieser Firmware. Nur bei RotorType 1 (AZ) aktiv — bei RotorType 2/3 (EL) `NAK ...:DISABLED` |
| `GETTEMPA` / `GETTEMPM` | Umgebungs- / Motortemperatur |
| `GETIS` | Strommesswert (nach Offset) |
| `GETWIND` / `SETWINDENABLE` | Wind (soweit Hardware) |

Weitere Kommandos (Homing-PWM, Stall, Load-Bins, Antennenanzeige, …) siehe Implementierung in `src/Rs485Dispatcher.cpp`.

## Encoder Typ 4: Inklinometer (Elevation)

Absolutwinkel vom [Inklinometer_IIS2ICLXTR](https://github.com/DK8DE/Inklinometer_IIS2ICLXTR) am gleichen RS485 wie der Windsensor (GPIO 8/9, Serial2).

- Bus zum Sensor: `#GETDG$` → `#ACK_GETDG,<float>$`, Baud **115200 8N1** (kein Rotor-Checksum-Frame)
- Wind und Inklinometer sind **gegenseitig ausschliessend** (bei Typ 4 kein Wind-Task)
- Kein Homing; `GETREF=1` sobald der Winkel frisch/gueltig ist
- Position fuer den Master: `GETPOSDG` (intern GETDG-Polling)
- Default Achse **0..90°** (`amax=9000`), sofern `amax` nicht explizit gesetzt
- Endschalter: `SETENDSTC:0` oder `:2` (`:1` → NAK `BADVAL`)
- Stall nutzt synthetische Counts (1 Count = 0,01°)
- Konfiguration des Sensors: Master-Befehle `SETINC*` / `GETINC*` (siehe unten); Werte liegen im **NVS des Inklinometers**, nicht im Rotor-FRAM

### Inklinometer-Konfiguration (SETINC / GETINC)

Alle folgenden Kommandos gelten **nur bei** `SETENCTYPE:4`. Bei anderem Encoder-Typ antwortet der Rotor mit `NAK_…:ENCTYPE`. Der Rotor leitet den Befehl ueber Serial2 an das Inklinometer weiter und mappt die Sensor-Antwort auf das Rotor-Checksum-Frame.

**Allgemeine Regeln**

- Float-Parameter akzeptieren Komma oder Punkt (`SETINCCAL:-2,50` oder `-2.50`).
- GET-Antwort: `ACK_<CMD>:<wert>` (Dezimaltrenner Komma).
- SET Erfolg: `ACK_<CMD>:1`.
- Sensor lehnt Wert ab / Bus stoert: `NAK_…:ERR` bzw. `NAK_…:TIMEOUT`.
- Unbekanntes Suffix: `NAK_…:NOSUP`. Keine Hardware: `NAK_…:NOHW`.

#### Lage / Montage

**`GETINCSWAPXY` / `SETINCSWAPXY`** — Achsen X/Y tauschen (`swapXY` am Sensor).  
Werte: `0` = aus, `1` = ein. Nuetzlich, wenn die Sensorplatine um 90° verdreht zur Elevationsebene montiert ist. Persistiert im Inklinometer-NVS.

```
SETINCSWAPXY:1
→ ACK_SETINCSWAPXY:1
GETINCSWAPXY
→ ACK_GETINCSWAPXY:1
```

**`GETINCINVREF` / `SETINCINVREF`** — Referenzachse invertieren.  
Werte: `0` / `1`. Aendert die Vorzeichenlage der berechneten Elevation relativ zur Sensor-Referenz.

```
SETINCINVREF:1
→ ACK_SETINCINVREF:1
```

**`GETINCINVSEN` / `SETINCINVSEN`** — Sense-Achse invertieren.  
Werte: `0` / `1`. Ergaenzt `INVREF`/`INVROT` fuer die Montageanpassung.

```
SETINCINVSEN:0
→ ACK_SETINCINVSEN:1
GETINCINVSEN
→ ACK_GETINCINVSEN:0
```

**`GETINCINVROT` / `SETINCINVROT`** — Drehrichtung umkehren.  
Werte: `0` = Winkel waechst im Uhrzeigersinn (Sensor-Default), `1` = gegen den Uhrzeigersinn. Wenn Elevation beim Heben faellt statt steigt, hier umschalten.

```
SETINCINVROT:1
→ ACK_SETINCINVROT:1
```

**`GETINCMOUNT` / `SETINCMOUNT`** — Montage-Drehung der 0°-Seite.  
Werte: `0`, `90`, `180`, `270` (Grad). Rotiert das Winkelsystem in 90°-Schritten, ohne die Platine mechanisch umzubauen. Andere Werte → `NAK_…:BADVAL`.

```
SETINCMOUNT:90
→ ACK_SETINCMOUNT:1
GETINCMOUNT
→ ACK_GETINCMOUNT:90
```

**`GETINCLIM180` / `SETINCLIM180`** — Soft-Limit auf 0…180°.  
Werte: `0` = voller Sensorbereich, `1` = Ausgabe auf 0…180 begrenzen (Winkel >180…≤270 → 180, >270…<360 → 0). Fuer typische Elevationsmechanik oft sinnvoll.

```
SETINCLIM180:1
→ ACK_SETINCLIM180:1
```

**`GETINCCAL` / `SETINCCAL`** — Kalibrier-Offset in Grad.  
Bereich: ca. −180,0 … +180,0. Wird zum gefilterten Winkel addiert (Nullpunkt / Ebenkalibrierung). Persistiert im Inklinometer-NVS. Alternativ 2-Punkt-Ebenkalibrierung ueber die SoftAP-Web-UI des Sensors.

```
SETINCCAL:-2,50
→ ACK_SETINCCAL:1
GETINCCAL
→ ACK_GETINCCAL:-2,50
```

#### Sensor / Filter / Anzeige

**`GETINCFS1G` / `SETINCFS1G`** — Full-Scale des IIS2ICLX.  
`1` = ±1 g, `0` = ±2 g. Fuer senkrechte Elevationsmontage ±0,5 g nicht nutzen (Saettigung). Aenderung wirkt auf Empfindlichkeit und Rauschverhalten.

```
SETINCFS1G:1
→ ACK_SETINCFS1G:1
GETINCFS1G
→ ACK_GETINCFS1G:1
```

**`GETINCDEC` / `SETINCDEC`** — Nachkommastellen der Elevation am Sensor.  
Werte: `0`, `1`, `2`. Beeinflusst die Aufloesung in der Sensor-Antwort (und der Web-Anzeige); der Rotor speichert intern weiterhin Deg01 (0,01°).

```
SETINCDEC:2
→ ACK_SETINCDEC:1
```

**`GETINCFILT` / `SETINCFILT`** — EMA-Filter-Alpha.  
Bereich: 0,01 … 1,0 (groesser = schneller / weniger Glaettung). Bei stabiler Mechanik daempft der Sensor zusaetzlich (×0,35), bei Ruck beschleunigt er (×1,8). Ungueltige Werte → Sensor-NACK → `NAK_…:ERR`.

```
SETINCFILT:0,35
→ ACK_SETINCFILT:1
GETINCFILT
→ ACK_GETINCFILT:0,35
```

#### Ruck- / Stoss-Ueberwachung

Der Sensor zaehlt Rucke (INT2). Erreichen von `SHOCKLIM` Ereignissen innerhalb von `SHOCKWIN` Sekunden setzt einen Ruck-Fehler (nicht im NVS; nach Sensor-Neustart geloescht).

**`GETINCSHKTH` / `SETINCSHKTH`** — Stossschwelle (WAKE_UP_THS).  
Werte: 1…63 (kleiner = empfindlicher; Default typisch 25).

```
SETINCSHKTH:25
→ ACK_SETINCSHKTH:1
```

**`GETINCSHKWN` / `SETINCSHKWN`** — Zeitfenster fuer die Ruckzaehlung in Sekunden.  
Werte: 1…3600 (Default typisch 60).

```
SETINCSHKWN:60
→ ACK_SETINCSHKWN:1
```

**`GETINCSHKLM` / `SETINCSHKLM`** — Anzahl Rucke bis Fehler.  
Werte: 0…60; `0` = Ueberwachung aus (Default typisch 5).

```
SETINCSHKLM:5
→ ACK_SETINCSHKLM:1
```

**`GETINCSHKCT`** — aktuelle Ruck-Anzahl im laufenden Fenster (nur GET).

```
GETINCSHKCT
→ ACK_GETINCSHKCT:2
```

**`GETINCSHKER` / `SETINCSHKER`** — Ruck-Fehler lesen bzw. loeschen.  
GET: `0` = kein Fehler, `1` = Fehler aktiv. SET nur `:0` (loeschen und Zaehler neu starten); anderer Wert → `NAK_…:BADVAL`.

```
GETINCSHKER
→ ACK_GETINCSHKER:1
SETINCSHKER:0
→ ACK_SETINCSHKER:1
```

**`GETINCSTL`** — Mechanik stabil (INT2, kein Stoss).  
`1` = stabil, `0` = Ruck/Unruhe. Nur GET. Nuetzlich zur Diagnose neben `GETPOSDG`.

```
GETINCSTL
→ ACK_GETINCSTL:1
```

**`GETINCSHK`** — Stoss/Ruck erkannt (invertiert zu SETTLED).  
`1` = Ruck/Stoss, `0` = stabil. Nur GET.

```
GETINCSHK
→ ACK_GETINCSHK:0
```

#### Diagnose / SoftAP am Inklinometer

**`GETINCWLAN` / `SETINCWLAN`** — SoftAP-Web-UI des Inklinometers ein/aus.  
Werte: `0` / `1`. Entspricht Sensor-`SETWIFI`/`GETWIFI`. **Nicht** im Sensor-NVS; nach Neustart des Inklinometers wieder aus. SSID typisch `INKLINO-<UID>`, IP `192.168.4.1` (siehe Sensor-README). Unabhaengig von Rotor-`SETWLAN` (Rotor-SoftAP).

```
SETINCWLAN:1
→ ACK_SETINCWLAN:1
GETINCWLAN
→ ACK_GETINCWLAN:1
```

**`GETINCDBG` / `SETINCDBG`** — USB-Debug-Ausgabe am Inklinometer.  
Werte: `0` / `1`. Schaltet Debug auf dem nativen USB-CDC des Sensors.

```
SETINCDBG:1
→ ACK_SETINCDBG:1
```

## Encoder Typ 3: Bereich >360°

Der SSI-Encoder liefert nur 0…360°. Ein Soft-Turn (0/1) erweitert den logischen Bereich bis `amax` (max. 720°):

- `GETPOSDG` = Rohwinkel + Turn×360° (geclampt auf `amax`)
- Turn wird bei Wrap erkannt und **sofort** in NVS geschrieben (`sturn`, Fallback `sldeg`)
- Nach Stromausfall im Überdrehbereich muss wieder die logische Lage (>360°) erscheinen — sonst Kabelbruch-Risiko

## Firmware herunterladen & flashen

### USB-Webflasher (GitHub Pages)

https://dk8de.github.io/Rotor_Firmware/ — Chrome/Edge, USB, schreibt `factory.bin` (Komplettflash).

Lokal: `webflasher/index.html` + `IMGs/` (nach `.\build.ps1 -SkipUpload`).

### SoftAP-OTA (WLAN-Update über die interne Seite)

1. RS485: `SETWLAN:1` → Hotspot `Rotor_<UID>` / Passwort `Rotorconfig`
2. Browser: http://192.168.4.1 (Login `admin` / `Rotorconfig`)
3. Tab **Update** → nur `firmware.bin` hochladen (nicht `factory.bin`)

### GitHub Releases / Actions

- Releases: [github.com/DK8DE/Rotor_Firmware/releases](https://github.com/DK8DE/Rotor_Firmware/releases) (`*-factory.bin`, `*-firmware.bin`)
- Artifacts: [Actions → Build Webflasher & Release](https://github.com/DK8DE/Rotor_Firmware/actions/workflows/platformio-build.yml)

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
├── src/                      # Anwendung (main, Motion, Encoder, RS485, Safety, …)
├── lib/                      # Lokale Bibliotheken
├── webflasher/               # USB-Webflasher-Seite (→ GitHub Pages)
├── IMGs/                     # factory.bin, firmware.bin, manifest.json (via build.ps1)
├── scripts/read_version.sh   # Version aus src/Version.h (CI)
├── build.ps1                 # Build, IMGs, optional Upload
├── platformio.ini
└── .github/workflows/        # CI: Build, Release, Pages
```

## Lizenz

Dieses Projekt ist Open Source. Einzelheiten siehe die Lizenzdateien im Repository.

## Hinweise

- Bei Verbindungsproblemen zuerst Verkabelung, RS485-Adressierung, Baudrate und Checksumme prüfen (auch bei negativen Parametern).
- Nach `SETENCTYPE` ist ein Neustart nötig.
- Bei Fragen oder Fehlern ein Issue im [GitHub-Repository](https://github.com/DK8DE/Rotor_Firmware) erstellen.
