# Rotor_Firmware

**Version: 1.10.12**

Firmware fÃƒÂ¼r einen motorisierten Antennenrotor auf Basis des ESP32-S3 (PlatformIO/Arduino).

Dieses Projekt steuert den Antennenrotor, verarbeitet Endschalter und Encoder, regelt die Bewegung mit Rampenprofilen und stellt die Kommunikation ÃƒÂ¼ber RS485 bereit.

## ÃƒÅ“berblick

Die Firmware lÃƒÂ¤uft auf der Steuerhardware und bietet unter anderem:

- **PrÃƒÂ¤zise Positionsregelung** mit Encoder-RÃƒÂ¼ckfÃƒÂ¼hrung
  - Typ 1: Motorachsen-Encoder (PCNT)
  - Typ 2: Ring-/Abtriebs-Encoder (PCNT, optional Z-Index)
  - Typ 3: TWK KBE58 SSI-Absolutencoder (kein Homing ÃƒÂ¼ber Endschalter)
  - Typ 4: Inklinometer IIS2ICLX am Wind-RS485 (Elevation, kein Homing, kein Wind)
- **Arbeitsbereich bis 720Ã‚Â°** bei Typ 3 (Turn-ZÃƒÂ¤hler 0/1, NVS-gesichert)
- **Feinjustage-Offset** `SETDGCAL` / `GETDGCAL` (Ã¢Ë†â€™360Ã¢â‚¬Â¦+360Ã‚Â°) fÃƒÂ¼r GETPOSDG/SETPOSDG
- **Automatisches Homing** (Typ 1/2) mit Endschaltern und Backlash-Kompensation
- **RS485-Kommunikation** fÃƒÂ¼r Fernsteuerung und Statusabfragen
- **StromÃƒÂ¼berwachung** (IS-Messung) zur Blockadeerkennung
- **TemperaturÃƒÂ¼berwachung** (DS18B20)
- **Windmessung** ÃƒÂ¼ber Anemometer / optionalen RS485-Windsensor (bei Typ 3 eingeschrÃƒÂ¤nkt wegen SSI-Pins; bei Typ 4 entfÃƒÂ¤llt Wind zugunsten des Inklinometers)
- **Last-/Baseline-Analyse** (LoadMonitor, Kalibrierfahrt ÃƒÂ¼ber eine Getriebe-Umdrehung)
- **Sicherheitsfunktionen**: Stall-Erkennung, Endschalter, Deadman/Keepalive
- **Persistente Parameter** ÃƒÂ¼ber NVS/Preferences
- **SoftAP-Konfigurationsseite** (`SETWLAN`): Status, gefÃ¼hrte **Inbetriebnahme (Setup)**, **Benutzer**/Login, **WLAN**-SchlÃ¼ssel, HTTP-OTA-Update, Reboot/Werksreset (Hotspot `Rotor_<UID>`; SoftAP-Zustand nicht persistent, Zugangsdaten NVS-persistent)
- **Service-JOG** (`SETSRVJOG`): JOG ohne Referenz/Stall fÃƒÂ¼r CPR-Einmessen (Deadman bleibt; nicht persistent)
- **Setup-Modus** (`SETSETUP`): JOG ohne Amin/Amax (auch negativ) fÃƒÂ¼r Einmessen ohne Endschalter; Unref + Stall aus; nicht persistent

## Versionierung

Die Firmware-Versionsnummer (Semantic Versioning: `MAJOR.MINOR.PATCH`) wird an **einer** zentralen Stelle gepflegt: [`src/Version.h`](src/Version.h).

- Wird beim Boot unabhÃƒÂ¤ngig von `g_debug` einmal ÃƒÂ¼ber USB-Serial ausgegeben (`Rotor_Firmware v1.10.12`).
- ÃƒÅ“ber RS485 per **`GETVERSION`** abfragbar (`ACK_GETVERSION:1.10.12`).
- `build.ps1` liest die Version aus `src/Version.h` und ÃƒÂ¼bernimmt sie in `IMGs/manifest.json` (Feld `version`).

### Version setzen (`build.ps1 -Version`)

Die Version wird **nicht** manuell in `src/Version.h` editiert, sondern ÃƒÂ¼ber den Build-Skript-Parameter `-Version` gesetzt:

```powershell
.\build.ps1 -Version "1.10.12"
```

Das erledigt automatisch:

1. `src/Version.h` wird auf `1.10.12` aktualisiert (`FW_VERSION_MAJOR/MINOR/PATCH`).
2. **`README.md` wird automatisch mitaktualisiert** Ã¢â‚¬â€ alle Stellen, die die alte Versionsnummer enthalten (Badge oben, Beispiele im Versionierungs-Abschnitt), werden auf die neue Version umgeschrieben.
3. Firmware wird gebaut, `IMGs/` aktualisiert und (ohne `-SkipUpload`) geflasht.

Auch ohne `-Version`-Parameter gleicht `build.ps1` bei jedem Lauf die `README.md` automatisch mit dem aktuellen Stand von `src/Version.h` ab (z. B. falls die Datei direkt bearbeitet wurde).

Ãƒâ€žnderungen sollten committet und nach `main`/`master` gepusht werden, damit sie versioniert nachvollziehbar sind:

```powershell
git add src/Version.h README.md
git commit -m "Version 1.10.12"
git push
```

### Automatisches GitHub-Release & Webflasher

Der Workflow [`.github/workflows/platformio-build.yml`](.github/workflows/platformio-build.yml) baut bei jedem Push/PR die Firmware und bereitet Flash-Images vor (wie beim [Inklinometer](https://github.com/DK8DE/Inklinometer_IIS2ICLXTR/releases/tag/v1.3.3)):

- Bei Push nach `main`/`master`: GitHub Pages mit USB-Webflasher unter https://dk8de.github.io/Rotor_Firmware/
- Wenn fÃƒÂ¼r die Version aus `src/Version.h` noch kein Release `vMAJOR.MINOR.PATCH` existiert Ã¢â€ â€™ Release mit:
  - `Rotor-X.Y.Z-factory.bin` Ã¢â‚¬â€œ Komplettflash ab 0x0 (USB-Webflasher / Erstinstallation)
  - `Rotor-X.Y.Z-firmware.bin` Ã¢â‚¬â€œ SoftAP-OTA (App, Web-Tab Update)
- Existiert das Release bereits Ã¢â€ â€™ kein doppeltes Release; Pages/Artifacts werden trotzdem aktualisiert

Beim Ãƒâ€žndern der Firmware die Version erhÃƒÂ¶hen (`PATCH` / `MINOR` / `MAJOR`) Ã¢â‚¬â€ sonst entsteht kein neues Release.

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
.\build.ps1 -Version "1.10.12"
```

Details zur Versionsvergabe und zum automatischen GitHub-Release: siehe [Versionierung](#versionierung).

`build.ps1` legt bzw. erneuert den Ordner **`IMGs/`** und die `manifest.json` fÃƒÂ¼r [ESP Web Tools](https://esphome.github.io/esp-web-tools/) (USB-Webflasher, ein Part):

| Datei | Verwendung |
|-------|------------|
| `factory.bin` | Komplettimage ab `0x0` (USB-Webflasher / Erstinstallation) |
| `firmware.bin` | nur App fÃƒÂ¼r SoftAP-OTA (Web-Tab Update) |
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
| `GETHOMING` | Homing aktiv? (`1`/`0`) |
| `GETMOVE` | Rotor bewegt sich? (`1` bei Positionsfahrt, JOG oder Homing, sonst `0`) |
| `GETARRTOL` / `SETARRTOL:<grad>` | Ankunftstoleranz (NVS `atol`); steuert Zielfenster und Fine-Deadband |

### Achse / Feinjustage

| Kommando | Beschreibung |
|----------|--------------|
| `GETBEGINDG` / `SETBEGINDG` | Achsminimum |
| `GETMAXDG` / `SETMAXDG` | Achsmaximum (Typ 3 bis 720Ã‚Â°); bei Typ 1/2 werden die beim Homing gelernten Encoder-Counts (Endschalter zu Endschalter) auf diesen Winkel verteilt (z.B. 180Ã‚Â° fuer einen Elevationsrotor mit 180Ã‚Â° realem Hub), NICHT fest auf 360Ã‚Â° |
| `GETDGOFFSET` / `SETDGOFFSET` | Endschalter-Offset (Typ 1/2) |
| `GETDGCAL` / `SETDGCAL` | Feinjustage (Ã¢Ë†â€™360Ã¢â‚¬Â¦+360Ã‚Â°), NVS `dgcal` |

### Encoder

| Kommando | Beschreibung |
|----------|--------------|
| `GETENCTYPE` / `SETENCTYPE` | 1=Motorachse, 2=Ring/Abtrieb, 3=SSI, 4=Inklinometer (Neustart) |
| `GETENCINV` / `SETENCINV` | Encoder-Richtung invertieren (`0`/`1`). Nur Typ 1/2/3; Typ 4 â†’ `NAK â€¦:ENCTYPE` (dort `SETINCINVROT` etc.). Sofort wirksam (Positionssprung moeglich). FRAM-persistent |
| `GETMOTINV` / `SETMOTINV` | Motor-PWM-Polaritaet invertieren (`0`/`1`, alle Encoder-Typen). Nur H-Bruecke; Encoder unveraendert. FRAM-persistent |
| `GETENDSWAP` / `SETENDSWAP` | Endschalter L/R logisch tauschen (`0`/`1`). FRAM-persistent |
| `GETENDINV` / `SETENDINV` | Endschalter-Pegel: `0` = aktiv LOW (Default, gedrueckt=LOW), `1` = aktiv HIGH (gedrueckt=HIGH; z. B. NO vs. NC). Wirkt auf L und R. FRAM-persistent |
| `GETENDSTAT` | Live Endschalter: `L;R` als `0`/`1` (gedrueckt=1), nach `ENDSWAP`/`ENDINV` logisch. Auch bei `ENDSTC=0` (Verdrahtungs-Check) |
| `GETENDSTC` / `SETENDSTC` | Anzahl Endschalter: `0`=keine, `1`=ein Kontakt (L oder R), `2`=zwei Kontakte. Default Typ 1/2 = 2, SSI/Inklinometer = 0. Typ 4 erlaubt nur `0` oder `2` |
| `GETENCCOUNT` | Aktuelle Encoder-Counts (Rohwert) |
| `SETENCCOUNT` | Counts live setzen; **Winkel folgt automatisch** aus Counts+CPR (+DGOFFSET). Speichert Counts **und** berechneten Winkel in FRAM-POS. Typ 4: NAK. Checksumme `(src+dst)Ãƒâ€”100+countÃƒâ€”100` Ã¢â‚¬â€ Beispiel `#0:20:SETENCCOUNT:12345:12365$` |
| `SETPOSHERE` | Ã¢â‚¬Å¾Hier ist Winkel XÃ¢â‚¬Å“: setzt die **passenden Counts** zu X und speichert beides in FRAM (braucht CPR + `SETMAXDG`). Checksumme wie `SETPOSDG` Ã¢â‚¬â€ Beispiel 0Ã‚Â°: `#0:20:SETPOSHERE:0,00:20$` |
| `SETENCZERO` | Typ 1/2: Soft-Zero. Typ 3: SSI-Hardware-Null (SET0) + Turn zurÃƒÂ¼cksetzen. Typ 4: NAK (Kalibrierung am Inklinometer) |
| `GETENCCAX` / `SETENCCAX` | Counts pro 360Ã‚Â° fÃƒÂ¼r Motorachse (Typ 1) |
| `GETENCCRI` / `SETENCCRI` | Counts pro 360Ã‚Â° fÃƒÂ¼r Ring/Abtrieb (Typ 2) |
| `GETENCTURN` / `SETENCTURN` | Soft-Turn 0/1 (nur SSI, ÃƒÅ“berdrehen >360Ã‚Â°) |


### Identifikation / PWM / Sensorik (Auszug)

| Kommando | Beschreibung |
|----------|--------------|
| `GETVERSION` | Firmware-Version (z. B. `1.10.0`), siehe [Versionierung](#versionierung) |
| `GETWLAN` / `SETWLAN:0` / `SETWLAN:1` | SoftAP-Web-UI: `1` = Hotspot `Rotor_<UID>` an (Default-Passwort `Rotorconfig`, UI Default `admin`/`Rotorconfig` @ 192.168.4.1; beides im Tab **Benutzer** / **WLAN** Ã¤nderbar und NVS-persistent), `0` = aus. SoftAP-Zustand nicht persistent â€” nach Reboot immer aus. LB-LED bei SoftAP: **200 ms an / 200 ms aus / 200 ms an / 600 ms Pause** (wie Inklinometer). Tabs: Status, Setup, Benutzer, WLAN, Update, Reset. Hinweis: WiFi kann die Strommessung (ADC2) beeintrÃ¤chtigen. |
| `GETSRVJOG` / `SETSRVJOG:0\|1` | Service-JOG: JOG ohne `GETREF`/Encoder-GÃƒÂ¼ltigkeit, Stall aus; Endschalter-GPIO und JOG-Deadman bleiben. Nicht persistent (nach Reboot/SoftAP-Ende aus). |
| `GETSETUP` / `SETSETUP:0\|1` | Setup-Modus (Inbetriebnahme): `SETSETUP:1` = JOG ohne Amin/Amax-Stopp (auch negativ/ÃƒÂ¼ber Max), ohne Ref, Stall aus; `GETPOSDG` zeigt Rohwinkel. **`SETSETUP:0`** = Service aus: JOG stoppt, **Achsgrenzen (Amin/Amax) fÃƒÂ¼r JOG wieder aktiv**, Stall wieder wie konfiguriert. Nicht persistent (auch Reboot/`SETWLAN:0` beendet Setup). |
| `GETID` / `SETID` | Slave-ID lesen / setzen (gerichtet an aktuelle ID; ACK; gÃƒÂ¼ltig 1Ã¢â‚¬Â¦254). Nach ID-Wechsel bleibt die **alte** Adresse ca. 5 s als Alias (nachfolgendes `SETID` an alte ID kommt noch an). |
| `SETROTORID` | ID per Broadcast setzen: Zieladresse **255**, Params = neue ID (1Ã¢â‚¬Â¦254). ACK von der **alten** ID (`ACK_SETROTORID:1`) Ã¢â‚¬â€ nur ein Slave am Bus. Danach `GETID` an die **neue** Adresse. Beispiel Master=0, ID=21: `#0:255:SETROTORID:21:276$` (Checksumme `(0+255)Ãƒâ€”100+21Ãƒâ€”100`) |
| `GETROTORTYPE` / `SETROTORTYPE` | Rotor-Typ: 1=Rotation/Azimut, 2=Elevation 90Ã‚Â°, 3=Elevation 180Ã‚Â° (nur Identifikation, NVS `rty`, keine Logik in dieser Firmware) |
| `GETMASTHEIGHT` / `SETMASTHEIGHT` | MasthÃƒÂ¶he in Metern (0Ã¢â‚¬Â¦9999, 1 Nachkommastelle). Nur Identifikation fÃƒÂ¼r den Controller; persistent. Checksumme `(src+dst)Ãƒâ€”100+hÃƒÂ¶heÃƒâ€”100` Ã¢â‚¬â€ Beispiel 25 m: `#0:20:SETMASTHEIGHT:25:45$` |
| `GETANTNAME1`/`2`/`3` / `SETANTNAME1`/`2`/`3` | Antennennamen (max. 9 Zeichen, keine `:`/`;`); NVS `an1`/`an2`/`an3`, nur fuer den Controller |
| `GETASELECT` / `SETASELECT` | Aktuell gewaehlte Antenne (1/2/3, Default 1); nur Identifikation/Zustand, NVS `asel`, keine Logik in dieser Firmware. Nur bei RotorType 1 (AZ) aktiv Ã¢â‚¬â€ bei RotorType 2/3 (EL) `NAK ...:DISABLED` |
| `GETTEMPA` / `GETTEMPM` | Umgebungs- / Motortemperatur |
| `GETIS` | Strommesswert (nach Offset) |
| `GETWIND` / `SETWINDENABLE` | Wind (soweit Hardware) |

Weitere Kommandos (Homing-PWM, Stall, Load-Bins, Antennenanzeige, Ã¢â‚¬Â¦) siehe Implementierung in `src/Rs485Dispatcher.cpp`.

## Encoder Typ 4: Inklinometer (Elevation)

Absolutwinkel vom [Inklinometer_IIS2ICLXTR](https://github.com/DK8DE/Inklinometer_IIS2ICLXTR) am gleichen RS485 wie der Windsensor (GPIO 8/9, Serial2).

- Bus zum Sensor: `#GETDG$` Ã¢â€ â€™ `#ACK_GETDG,<float>$`, Baud **115200 8N1** (kein Rotor-Checksum-Frame)
- Wind und Inklinometer sind **gegenseitig ausschliessend** (bei Typ 4 kein Wind-Task)
- Kein Homing; `GETREF=1` sobald der Winkel frisch/gueltig ist
- Position fuer den Master: `GETPOSDG` (intern GETDG-Polling)
- Default Achse **0..90Ã‚Â°** (`amax=9000`), sofern `amax` nicht explizit gesetzt
- Endschalter: `SETENDSTC:0` oder `:2` (`:1` Ã¢â€ â€™ NAK `BADVAL`)
- Stall nutzt synthetische Counts (1 Count = 0,01Ã‚Â°)
- Ankunft wegen Sensorlag (~80 ms Poll): `SETARRTOL` / `GETARRTOL` steuern Ankunft **und** Fine-Rueckfahrt-Deadband (NVS `atol`, z. B. `SETARRTOL:1,00`). Hold mindestens ~250 ms, Fine-Window ohne SSI-Kuerzung Ã¢â‚¬â€ verhindert Pendeln am Zielende
- Konfiguration des Sensors: Master-Befehle `SETINC*` / `GETINC*` (siehe unten); Werte liegen im **NVS des Inklinometers**, nicht im Rotor-FRAM

### Inklinometer-Konfiguration (SETINC / GETINC)

Alle folgenden Kommandos gelten **nur bei** `SETENCTYPE:4`. Bei anderem Encoder-Typ antwortet der Rotor mit `NAK_Ã¢â‚¬Â¦:ENCTYPE`. Der Rotor leitet den Befehl ueber Serial2 an das Inklinometer weiter und mappt die Sensor-Antwort auf das Rotor-Checksum-Frame.

**Allgemeine Regeln**

- Float-Parameter akzeptieren Komma oder Punkt (`SETINCCAL:-2,50` oder `-2.50`).
- GET-Antwort: `ACK_<CMD>:<wert>` (Dezimaltrenner Komma).
- SET Erfolg: `ACK_<CMD>:1`.
- Sensor lehnt Wert ab / Bus stoert: `NAK_Ã¢â‚¬Â¦:ERR` bzw. `NAK_Ã¢â‚¬Â¦:TIMEOUT`.
- Unbekanntes Suffix: `NAK_Ã¢â‚¬Â¦:NOSUP`. Keine Hardware: `NAK_Ã¢â‚¬Â¦:NOHW`.

#### Lage / Montage

**`GETINCSWAPXY` / `SETINCSWAPXY`** Ã¢â‚¬â€ Achsen X/Y tauschen (`swapXY` am Sensor).  
Werte: `0` = aus, `1` = ein. Nuetzlich, wenn die Sensorplatine um 90Ã‚Â° verdreht zur Elevationsebene montiert ist. Persistiert im Inklinometer-NVS.

```
SETINCSWAPXY:1
Ã¢â€ â€™ ACK_SETINCSWAPXY:1
GETINCSWAPXY
Ã¢â€ â€™ ACK_GETINCSWAPXY:1
```

**`GETINCINVREF` / `SETINCINVREF`** Ã¢â‚¬â€ Referenzachse invertieren.  
Werte: `0` / `1`. Aendert die Vorzeichenlage der berechneten Elevation relativ zur Sensor-Referenz.

```
SETINCINVREF:1
Ã¢â€ â€™ ACK_SETINCINVREF:1
```

**`GETINCINVSEN` / `SETINCINVSEN`** Ã¢â‚¬â€ Sense-Achse invertieren.  
Werte: `0` / `1`. Ergaenzt `INVREF`/`INVROT` fuer die Montageanpassung.

```
SETINCINVSEN:0
Ã¢â€ â€™ ACK_SETINCINVSEN:1
GETINCINVSEN
Ã¢â€ â€™ ACK_GETINCINVSEN:0
```

**`GETINCINVROT` / `SETINCINVROT`** Ã¢â‚¬â€ Drehrichtung umkehren.  
Werte: `0` = Winkel waechst im Uhrzeigersinn (Sensor-Default), `1` = gegen den Uhrzeigersinn. Wenn Elevation beim Heben faellt statt steigt, hier umschalten.

```
SETINCINVROT:1
Ã¢â€ â€™ ACK_SETINCINVROT:1
```

**`GETINCMOUNT` / `SETINCMOUNT`** Ã¢â‚¬â€ Montage-Drehung der 0Ã‚Â°-Seite.  
Werte: `0`, `90`, `180`, `270` (Grad). Rotiert das Winkelsystem in 90Ã‚Â°-Schritten, ohne die Platine mechanisch umzubauen. Andere Werte Ã¢â€ â€™ `NAK_Ã¢â‚¬Â¦:BADVAL`.

```
SETINCMOUNT:90
Ã¢â€ â€™ ACK_SETINCMOUNT:1
GETINCMOUNT
Ã¢â€ â€™ ACK_GETINCMOUNT:90
```

**`GETINCLIM180` / `SETINCLIM180`** Ã¢â‚¬â€ Soft-Limit auf 0Ã¢â‚¬Â¦180Ã‚Â°.  
Werte: `0` = voller Sensorbereich, `1` = Ausgabe auf 0Ã¢â‚¬Â¦180 begrenzen (Winkel >180Ã¢â‚¬Â¦Ã¢â€°Â¤270 Ã¢â€ â€™ 180, >270Ã¢â‚¬Â¦<360 Ã¢â€ â€™ 0). Fuer typische Elevationsmechanik oft sinnvoll.

```
SETINCLIM180:1
Ã¢â€ â€™ ACK_SETINCLIM180:1
```

**`GETINCCAL` / `SETINCCAL`** Ã¢â‚¬â€ Kalibrier-Offset in Grad.  
Bereich: ca. Ã¢Ë†â€™180,0 Ã¢â‚¬Â¦ +180,0. Wird zum gefilterten Winkel addiert (Nullpunkt / Ebenkalibrierung). Persistiert im Inklinometer-NVS. Alternativ 2-Punkt-Ebenkalibrierung ueber die SoftAP-Web-UI des Sensors.

```
SETINCCAL:-2,50
Ã¢â€ â€™ ACK_SETINCCAL:1
GETINCCAL
Ã¢â€ â€™ ACK_GETINCCAL:-2,50
```

#### Sensor / Filter / Anzeige

**`GETINCFS1G` / `SETINCFS1G`** Ã¢â‚¬â€ Full-Scale des IIS2ICLX.  
`1` = Ã‚Â±1 g, `0` = Ã‚Â±2 g. Fuer senkrechte Elevationsmontage Ã‚Â±0,5 g nicht nutzen (Saettigung). Aenderung wirkt auf Empfindlichkeit und Rauschverhalten.

```
SETINCFS1G:1
Ã¢â€ â€™ ACK_SETINCFS1G:1
GETINCFS1G
Ã¢â€ â€™ ACK_GETINCFS1G:1
```

**`GETINCDEC` / `SETINCDEC`** Ã¢â‚¬â€ Nachkommastellen der Elevation am Sensor.  
Werte: `0`, `1`, `2`. Beeinflusst die Aufloesung in der Sensor-Antwort (und der Web-Anzeige); der Rotor speichert intern weiterhin Deg01 (0,01Ã‚Â°).

```
SETINCDEC:2
Ã¢â€ â€™ ACK_SETINCDEC:1
```

**`GETINCFILT` / `SETINCFILT`** Ã¢â‚¬â€ EMA-Filter-Alpha.  
Bereich: 0,01 Ã¢â‚¬Â¦ 1,0 (groesser = schneller / weniger Glaettung). Bei stabiler Mechanik daempft der Sensor zusaetzlich (Ãƒâ€”0,35), bei Ruck beschleunigt er (Ãƒâ€”1,8). Ungueltige Werte Ã¢â€ â€™ Sensor-NACK Ã¢â€ â€™ `NAK_Ã¢â‚¬Â¦:ERR`.

```
SETINCFILT:0,35
Ã¢â€ â€™ ACK_SETINCFILT:1
GETINCFILT
Ã¢â€ â€™ ACK_GETINCFILT:0,35
```

#### Ruck- / Stoss-Ueberwachung

Der Sensor zaehlt Rucke (INT2). Erreichen von `SHOCKLIM` Ereignissen innerhalb von `SHOCKWIN` Sekunden setzt einen Ruck-Fehler (nicht im NVS; nach Sensor-Neustart geloescht).

**`GETINCSHKTH` / `SETINCSHKTH`** Ã¢â‚¬â€ Stossschwelle (WAKE_UP_THS).  
Werte: 1Ã¢â‚¬Â¦63 (kleiner = empfindlicher; Default typisch 25).

```
SETINCSHKTH:25
Ã¢â€ â€™ ACK_SETINCSHKTH:1
```

**`GETINCSHKWN` / `SETINCSHKWN`** Ã¢â‚¬â€ Zeitfenster fuer die Ruckzaehlung in Sekunden.  
Werte: 1Ã¢â‚¬Â¦3600 (Default typisch 60).

```
SETINCSHKWN:60
Ã¢â€ â€™ ACK_SETINCSHKWN:1
```

**`GETINCSHKLM` / `SETINCSHKLM`** Ã¢â‚¬â€ Anzahl Rucke bis Fehler.  
Werte: 0Ã¢â‚¬Â¦60; `0` = Ueberwachung aus (Default typisch 5).

```
SETINCSHKLM:5
Ã¢â€ â€™ ACK_SETINCSHKLM:1
```

**`GETINCSHKCT`** Ã¢â‚¬â€ aktuelle Ruck-Anzahl im laufenden Fenster (nur GET).

```
GETINCSHKCT
Ã¢â€ â€™ ACK_GETINCSHKCT:2
```

**`GETINCSHKER` / `SETINCSHKER`** Ã¢â‚¬â€ Ruck-Fehler lesen bzw. loeschen.  
GET: `0` = kein Fehler, `1` = Fehler aktiv. SET nur `:0` (loeschen und Zaehler neu starten); anderer Wert Ã¢â€ â€™ `NAK_Ã¢â‚¬Â¦:BADVAL`.

```
GETINCSHKER
Ã¢â€ â€™ ACK_GETINCSHKER:1
SETINCSHKER:0
Ã¢â€ â€™ ACK_SETINCSHKER:1
```

**`GETINCSTL`** Ã¢â‚¬â€ Mechanik stabil (INT2, kein Stoss).  
`1` = stabil, `0` = Ruck/Unruhe. Nur GET. Nuetzlich zur Diagnose neben `GETPOSDG`.

```
GETINCSTL
Ã¢â€ â€™ ACK_GETINCSTL:1
```

**`GETINCSHK`** Ã¢â‚¬â€ Stoss/Ruck erkannt (invertiert zu SETTLED).  
`1` = Ruck/Stoss, `0` = stabil. Nur GET.

```
GETINCSHK
Ã¢â€ â€™ ACK_GETINCSHK:0
```

#### Identitaet / Messung am Sensor

**`GETINCVER`** Ã¢â‚¬â€ Firmware-Version des Inklinometers (SemVer, z. B. `1.3.5`). Nur GET. Entspricht Sensor-`#GETVERSION$`. Benoetigt Inklinometer-Firmware ab **1.3.5**.

```
GETINCVER
Ã¢â€ â€™ ACK_GETINCVER:1.3.5
```

**`GETINCUID`** Ã¢â‚¬â€ Geraete-UID (6 Hex-Zeichen, SoftAP-Suffix `INKLINO-<UID>`). Nur GET. Entspricht Sensor-`#GETUID$`. Ab Sensor-FW **1.3.5**.

```
GETINCUID
Ã¢â€ â€™ ACK_GETINCUID:C3A1B2
```

**`GETINCDG`** Ã¢â‚¬â€ gefilterter Elevationswinkel direkt vom Sensor (Grad). Nur GET. Fuer den Master-Betrieb weiter `GETPOSDG` nutzen; `GETINCDG` dient der Diagnose (Sensor vs. Rotor).

```
GETINCDG
Ã¢â€ â€™ ACK_GETINCDG:45,12
```

#### Diagnose / SoftAP am Inklinometer

**`GETINCWLAN` / `SETINCWLAN`** Ã¢â‚¬â€ SoftAP-Web-UI des Inklinometers ein/aus.  
Werte: `0` / `1`. Entspricht Sensor-`SETWIFI`/`GETWIFI`. **Nicht** im Sensor-NVS; nach Neustart des Inklinometers wieder aus. SSID typisch `INKLINO-<UID>`, IP `192.168.4.1` (siehe Sensor-README). Unabhaengig von Rotor-`SETWLAN` (Rotor-SoftAP).

```
SETINCWLAN:1
Ã¢â€ â€™ ACK_SETINCWLAN:1
GETINCWLAN
Ã¢â€ â€™ ACK_GETINCWLAN:1
```

**`GETINCDBG` / `SETINCDBG`** Ã¢â‚¬â€ USB-Debug-Ausgabe am Inklinometer.  
Werte: `0` / `1`. Schaltet Debug auf dem nativen USB-CDC des Sensors.

```
SETINCDBG:1
Ã¢â€ â€™ ACK_SETINCDBG:1
```

## Encoder Typ 3: Bereich >360Ã‚Â°

Der SSI-Encoder liefert nur 0Ã¢â‚¬Â¦360Ã‚Â°. Ein Soft-Turn (0/1) erweitert den logischen Bereich bis `amax` (max. 720Ã‚Â°):

- `GETPOSDG` = Rohwinkel + TurnÃƒâ€”360Ã‚Â° (geclampt auf `amax`)
- Turn wird bei Wrap erkannt und **sofort** in NVS geschrieben (`sturn`, Fallback `sldeg`)
- Nach Stromausfall im ÃƒÅ“berdrehbereich muss wieder die logische Lage (>360Ã‚Â°) erscheinen Ã¢â‚¬â€ sonst Kabelbruch-Risiko

## Firmware herunterladen & flashen

### USB-Webflasher (GitHub Pages)

https://dk8de.github.io/Rotor_Firmware/ Ã¢â‚¬â€ Chrome/Edge, USB, schreibt `factory.bin` (Komplettflash).

Lokal: `webflasher/index.html` + `IMGs/` (nach `.\build.ps1 -SkipUpload`).

### SoftAP: Inbetriebnahme + OTA

1. SoftAP starten: RS485 `SETWLAN:1` **oder** beim Boot den linken Service-Taster **GPIO47** gedrÃƒÂ¼ckt halten (beide Taster = Werksreset)
2. Hotspot `Rotor_<UID>` / Passwort (Default `Rotorconfig`, Ã¤nderbar unter Tab **WLAN**) â†’ Browser http://192.168.4.1 (Login Default `admin` / `Rotorconfig`, Ã¤nderbar unter Tab **Benutzer**)
3. Tab **Setup** â€” gefÃ¼hrte Inbetriebnahme nur mit Rotorwerten (NVS/FRAM, Typ 4 zusÃ¤tzlich Inklinometer-NVS):
   - START â†’ ENCODER â†’ DRIVE â†’ ENDSTOPS â†’ WEATHER (nur Typ 1/2) â†’ TEMP â†’ CAL â†’ ZERO â†’ VERIFY â†’ ANTENNA (Azimut) â†’ DONE
   - Service-JOG in CAL/ZERO bei Typ 1/2 ohne Endschalter (Taste halten = fahren)
   - API: `GET /api/setup/state`, `POST /api/setup/apply`, `POST /api/setup/jog`
4. Tab **Benutzer** â€” Web-Login Ã¤ndern (`GET/POST /api/user`); Tab **WLAN** â€” SoftAP-SchlÃ¼ssel Ã¤ndern (`GET/POST /api/wifi`, 8â€“63 Zeichen). Werkseinstellung setzt beides zurÃ¼ck.
5. Tab **Update** â†’ nur `firmware.bin` hochladen (nicht `factory.bin`)

### GitHub Releases / Actions

- Releases: [github.com/DK8DE/Rotor_Firmware/releases](https://github.com/DK8DE/Rotor_Firmware/releases) (`*-factory.bin`, `*-firmware.bin`)
- Artifacts: [Actions Ã¢â€ â€™ Build Webflasher & Release](https://github.com/DK8DE/Rotor_Firmware/actions/workflows/platformio-build.yml)

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
Ã¢â€Å“Ã¢â€â‚¬Ã¢â€â‚¬ src/                      # Anwendung (main, Motion, Encoder, RS485, Safety, Ã¢â‚¬Â¦)
Ã¢â€Å“Ã¢â€â‚¬Ã¢â€â‚¬ lib/                      # Lokale Bibliotheken
Ã¢â€Å“Ã¢â€â‚¬Ã¢â€â‚¬ webflasher/               # USB-Webflasher-Seite (Ã¢â€ â€™ GitHub Pages)
Ã¢â€Å“Ã¢â€â‚¬Ã¢â€â‚¬ IMGs/                     # factory.bin, firmware.bin, manifest.json (via build.ps1)
Ã¢â€Å“Ã¢â€â‚¬Ã¢â€â‚¬ scripts/read_version.sh   # Version aus src/Version.h (CI)
Ã¢â€Å“Ã¢â€â‚¬Ã¢â€â‚¬ build.ps1                 # Build, IMGs, optional Upload
Ã¢â€Å“Ã¢â€â‚¬Ã¢â€â‚¬ platformio.ini
Ã¢â€â€Ã¢â€â‚¬Ã¢â€â‚¬ .github/workflows/        # CI: Build, Release, Pages
```

## Lizenz

Dieses Projekt ist Open Source. Einzelheiten siehe die Lizenzdateien im Repository.

## Hinweise

- Bei Verbindungsproblemen zuerst Verkabelung, RS485-Adressierung, Baudrate und Checksumme prÃƒÂ¼fen (auch bei negativen Parametern).
- Nach `SETENCTYPE` ist ein Neustart nÃƒÂ¶tig.
- Bei Fragen oder Fehlern ein Issue im [GitHub-Repository](https://github.com/DK8DE/Rotor_Firmware) erstellen.
