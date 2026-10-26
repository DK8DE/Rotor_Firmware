# Inklinometer (Encoder-Typ 4) — Sensorprotokoll und Rotor-Proxy

Dieses Dokument beschreibt den [Inklinometer_IIS2ICLXTR](https://github.com/DK8DE/Inklinometer_IIS2ICLXTR) am Rotor: das **direkte Sensor-Protokoll** auf Serial2 sowie die **Master-Befehle** `GETINC*` / `SETINC*` über den Rotor-RS485-Bus (Proxy).

Sensor-Befehle sind aus dem Quellcode (`handleFrame()` in der Inklinometer-Firmware) abgeleitet. Rotor-Proxy-Zuordnung: `kIncCmds[]` in `src/Rs485Dispatcher.cpp`.

---

## 1. Einbindung am Rotor

| Thema | Verhalten |
|--------|-----------|
| Encoder-Typ | `SETENCTYPE:4` (Neustart) |
| Bus zum Sensor | Serial2, GPIO 8/9, **115200 8N1**, Frame `#…$` (kein Rotor-Checksum) |
| Wind | bei Typ 4 aus (gegenseitig ausschließend) |
| Homing | keines; `GETREF=1`, sobald der Winkel frisch/gültig ist |
| Master-Position | **`GETPOSDG`** (Rotor pollt intern `#GETDG$`) |
| Sensor-Konfig | **`GETINC*` / `SETINC*`** — Werte im **NVS des Inklinometers**, nicht im Rotor-FRAM |
| Achse Default | oft 0…90° (`amax`), Endschalter `SETENDSTC:0` oder `:2` |

Proxy gilt **nur bei** `ENCTYPE=4`. Sonst: `NAK_…:ENCTYPE`. Keine Hardware: `NAK_…:NOHW`. Unbekanntes Suffix: `NAK_…:NOSUP`. Sensor-Timeout/NACK: `NAK_…:TIMEOUT` / `NAK_…:ERR`.

**Antworten am Rotor-Bus (Checksum-Frame):**

- GET: `ACK_<CMD>:<wert>` (Dezimaltrenner **Komma**)
- SET Erfolg: `ACK_<CMD>:1`
- Float-Parameter: Komma oder Punkt erlaubt (`SETINCCAL:-2,50`)

---

## 2. Mapping: Rotor-Proxy ↔ Sensor

| Rotor (Master) | Sensor (Serial2) | SET? | Kurz |
|----------------|------------------|------|------|
| `GETINCVER` | `#GETVERSION$` | — | Sensor-Firmware (SemVer); Sensor-FW ≥ **1.3.5** |
| `GETINCUID` | `#GETUID$` | — | Geräte-UID (6 Hex); SoftAP `INKLINO-<UID>`; ≥ **1.3.5** |
| `GETINCDG` | `#GETDG$` | — | Winkel direkt vom Sensor (Diagnose); Betrieb: `GETPOSDG` |
| `GETINCSWAPXY` / `SETINCSWAPXY` | `GETSWAPXY` / `SETSWAPXY` | ja | Achsen X/Y tauschen |
| `GETINCINVREF` / `SETINCINVREF` | `GETINVREF` / `SETINVREF` | ja | Referenzachse invertieren |
| `GETINCINVSEN` / `SETINCINVSEN` | `GETINVSENS` / `SETINVSENS` | ja | Sense-Achse invertieren |
| `GETINCINVROT` / `SETINCINVROT` | `GETINVROT` / `SETINVROT` | ja | Drehrichtung umkehren |
| `GETINCMOUNT` / `SETINCMOUNT` | `GETMOUNTROT` / `SETMOUNTROT` | ja | Montage 0/90/180/270 |
| `GETINCLIM180` / `SETINCLIM180` | `GETLIMIT180` / `SETLIMIT180` | ja | Soft-Limit 0…180° |
| `GETINCCAL` / `SETINCCAL` | `GETCALIB` / `SETCALIB` | ja | Kalibrier-Offset ° |
| `GETINCFS1G` / `SETINCFS1G` | `GETFS1G` / `SETFS1G` | ja | ±1 g / ±2 g |
| `GETINCDEC` / `SETINCDEC` | `GETELEVDEC` / `SETELEVDEC` | ja | Nachkommastellen 0/1/2 |
| `GETINCFILT` / `SETINCFILT` | `GETFILTER` / `SETFILTER` | ja | EMA-Alpha |
| `GETINCSHKTH` / `SETINCSHKTH` | `GETSHOCKTHS` / `SETSHOCKTHS` | ja | Stoßschwelle 1…63 |
| `GETINCSHKWN` / `SETINCSHKWN` | `GETSHOCKWIN` / `SETSHOCKWIN` | ja | Ruck-Fenster 1…3600 s |
| `GETINCSHKLM` / `SETINCSHKLM` | `GETSHOCKLIM` / `SETSHOCKLIM` | ja | Rucke bis Fehler 0…60 |
| `GETINCSHKCT` | `#GETSHOCKCNT$` | — | Rucke im Fenster |
| `GETINCSHKER` / `SETINCSHKER` | `GETSHOCKERR` / `SETSHOCKERR` | nur `:0` | Fehler lesen / löschen |
| `GETINCSTL` | `#GETSETTLED$` | — | Mechanik stabil |
| `GETINCSHK` | `#GETSHOCK$` | — | Stoß/Ruck erkannt |
| `GETINCWLAN` / `SETINCWLAN` | `GETWIFI` / `SETWIFI` | ja | SoftAP Sensor (nicht NVS) |
| `GETINCDBG` / `SETINCDBG` | `GETDEBUG` / `SETDEBUG` | ja | USB-Debug am Sensor |

Hinweis: Rotor-`GETVERSION` / `SETWLAN` betreffen den **Rotor**, nicht den Sensor. Sensor-Version/WLAN nur über `GETINCVER` / `GETINCWLAN` usw.

---

## 3. Sensor-Protokoll (direkt am Inklinometer)

### 3.1 Grundlagen

| | |
|--|--|
| Schnittstelle | 115200 Baud, 8N1, UART0 (TX GPIO21, RX GPIO20) |
| Frame | `#<BEFEHL>[,<WERT>]$` — z. B. `#GETDG$`, `#SETFILTER,0.35$` |
| Groß-/Kleinschreibung | zählt; Leerzeichen werden **nicht** entfernt |
| `\r` / `\n` | werden ignoriert |
| Frame-Länge | Puffer **95** Zeichen; längere Frames still verworfen (keine Antwort) |
| Neues `#` | mitten im Frame → Frame startet neu |
| Adressierung | **keine** (Punkt-zu-Punkt; mehrere Geräte → Kollision) |
| Erfolg | `#ACK_<BEFEHL>,<Wert>$` (SET: übernommener Wert; Float oft 2 Nachkommastellen) |
| Fehler | `#NACK_<BEFEHL>,ERR$` — ungültig, fehlend, unbekannt, SET auf reinen Lese-Befehl |
| Unbekannt | Frame-Text in NACK, z. B. `#FOO$` → `#NACK_FOO,ERR$` |
| Bereich | zu groß/klein → **NACK**, kein Clamp |
| NVS | jedes erfolgreiche SET speichert die Konfiguration; **Ausnahmen:** `SETWIFI`, `SETSHOCKERR` |
| Flash | SET nur bei Änderung, nicht zyklisch |

### 3.2 Weg des Winkels

1. Sensorwerte `ax`/`ay`, gemittelt aus dem FIFO  
2. `SWAPXY`, dann `INVREF` und `INVSENS`  
3. `atan2` → Wertebereich 0…360°  
4. `INVROT`, dann `+MOUNTROT`, dann `+CALIB`  
5. EMA-Filter mit `FILTER` (Faktor ×0,35 bei stabil, ×1,8 bei Ruck)  
6. `LIMIT180`, dann Rundung mit `ELEVDEC`  

Die ersten fünf Winkel-Einstellungen laufen über den Filter nach (ca. 1–2 s). **`MOUNTROT`** setzt den Filter zurück → `GETDG` sofort neu.

---

## 4. Reine Lese-Befehle (Sensor / Proxy)

| Sensor | Proxy | Antwort / Bedeutung |
|--------|-------|---------------------|
| `#GETDG$` | `GETINCDG` | Gefilterter Winkel °. 0…359,99; mit `LIMIT180` 0…180. Nachkommastellen = `ELEVDEC`. Vor erster Messung oft 0. Sensorausfall: Wert friert ein (kein RS485-Fehlerhinweis); LBLED dauerhaft an. Master-Betrieb: **`GETPOSDG`**. |
| `#GETVERSION$` | `GETINCVER` | Firmware-SemVer (z. B. `1.3.6`). |
| `#GETUID$` | `GETINCUID` | 6 Hex (letzte 3 MAC-Bytes) = Suffix `INKLINO-<UID>`. |
| `#GETSETTLED$` | `GETINCSTL` | INT2 Live: `1` = stabil, `0` = Ruck/Stoß. Nicht gespeichert; kurze Rucke können verpasst werden. Steuert auch den Filter. Zum Zählen: `GETSHOCKCNT` / `GETINCSHKCT`. |
| `#GETSHOCK$` | `GETINCSHK` | Gegenstück zu SETTLED (`1` = Ruck). |
| `#GETSHOCKCNT$` | `GETINCSHKCT` | Rucke in den letzten `SHOCKWIN` Sekunden (gleitendes Fenster). Ruck = fallende INT2-Flanke (stabil → Ruck). Zählen auch bei `SHOCKLIM=0`. |

---

## 5. Winkel-Einstellungen (lesen/schreiben, NVS)

| Sensor GET/SET | Proxy | Werte (Default) | Bedeutung |
|----------------|-------|-----------------|-----------|
| `SWAPXY` | `…SWAPXY` | 0/1 (**1**) | Achsen X/Y tauschen (Einbaulage). |
| `INVREF` | `…INVREF` | 0/1 (**0**) | Vorzeichen Referenzachse. |
| `INVSENS` | `…INVSEN` | 0/1 (**1**) | Vorzeichen Messachse. |
| `INVROT` | `…INVROT` | 0/1 (**1**) | Drehrichtung: Winkel → 360° − Winkel. |
| `MOUNTROT` | `…MOUNT` | nur **0/90/180/270** (**0**) | 0°-Seite verschieben. Andere Werte (auch 45) → NACK. Filter-Reset. |
| `CALIB` | `…CAL` | −180,00…+180,00 (**0,00**) | Offset nach MOUNTROT (Feinjustage). Negativ erlaubt (`-1.5`). |
| `LIMIT180` | `…LIM180` | 0/1 (**0**) | Ausgabe 0…180: 0…180 unverändert; >180…≤270 → 180; >270 → 0. Nur Ausgabe, nicht Filter. |
| `ELEVDEC` | `…DEC` | 0/1/2 (**2**) | Nachkommastellen GETDG/Web. Nur Formatierung. |
| `FILTER` | `…FILT` | 0,01…1,0 (**0,15**) | EMA. Kleiner = stärker glätten. Intern α×0,35 (stabil) / α×1,8 (Ruck), begrenzt 0,01…1,0. Bei 1,0 im Ruck kaum Glättung. Antwort oft 2 Stellen. |
| `FS1G` | `…FS1G` | 0/1 (**1**) | `1` = ±1 g, `0` = ±2 g. Sofort in den Sensor. Ändert Stoßschwelle (1 LSB = FS/64). |

---

## 6. Stoß- / Ruck-Überwachung

| Sensor | Proxy | Werte (Default) | Bedeutung |
|--------|-------|-----------------|-----------|
| `SHOCKTHS` | `…SHKTH` | 1…63 (**25**) | INT2-Empfindlichkeit (`WAKE_UP_THS`). Kleiner = empfindlicher. 1 LSB ≈ FS/64 (~15,6 mg bei ±1 g / ~31 mg bei ±2 g). Default 25 ≈ 390 mg (±1 g) bzw. 780 mg (±2 g). Bei 1: fast Dauer-„Ruck“, Filter schnell, CNT hoch. Sensor-Schreibfehler → NACK, kein Speichern. |
| `SHOCKWIN` | `…SHKWN` | 1…3600 s (**60**) | Gleitendes Zeitfenster. Kürzer verwirft ältere Rucke unwiderruflich. |
| `SHOCKLIM` | `…SHKLM` | 0…60 (**5**) | Rucke im Fenster bis Fehler. `0` = Fehler aus, Zählen weiter. Prüfung beim nächsten Ruck; neues Limit setzt nichts rückwirkend. |
| `GETSHOCKERR` | `GETINCSHKER` | 0/1 | Ruck-Fehler aktiv? |
| `SETSHOCKERR,0` | `SETINCSHKER:0` | nur **0** | Fehler löschen + Zähler neu. Anderer Wert → NACK. |

**Ruck-Fehler:** bleibt bis Löschen; **nicht** im Flash; nach Neustart weg; beeinflusst **weder** LBLED **noch** Winkelausgabe. Master muss aktiv abfragen und löschen.

---

## 7. WLAN und Diagnose am Sensor

| Sensor | Proxy | Werte | Bedeutung |
|--------|-------|-------|-----------|
| `WIFI` | `…WLAN` | 0/1 (nach Boot **0**) | SoftAP ein/aus. Kein Toggle: `1` startet nur wenn aus, `0` beendet nur wenn an. Antwort = Ist-Zustand. **Nicht** NVS. SSID `INKLINO-<UID>`, Passwort typisch `Rotorconfig`, `192.168.4.1`, Kanal 1, max. 2 Clients, ~8,5 dBm. LBLED: Doppelblitz **200/200/200/600 ms**. Idle-Abschaltung ~20 s nach letztem Client nur, wenn vorher ≥1 Client verbunden war — sonst bleibt AP bis `SETWIFI,0` / Neustart. Unabhängig von Rotor-`SETWLAN`. |
| `DEBUG` | `…DBG` | 0/1 (**0**) | ~10×/s `Elev: … deg settled=…` auf USB-CDC. Kein Einfluss auf RS485. **Wird** in NVS gespeichert. |

---

## 8. Was über RS485 (Sensor und Proxy) fehlt

Kein Bus-Befehl für: Neustart, Werkreset, Sensorfehler-Status, UI-Sprache, 2-Punkt-Ebenkalibrierung. Das geht über SoftAP-Web-UI bzw. Taster am Sensor.

---

## 9. Beispiele (Rotor-Bus)

```
SETENCTYPE:4
→ (Neustart)

GETINCVER
→ ACK_GETINCVER:1.3.6

GETINCUID
→ ACK_GETINCUID:C3A1B2

GETINCDG
→ ACK_GETINCDG:45,12

SETINCCAL:-2,50
→ ACK_SETINCCAL:1
GETINCCAL
→ ACK_GETINCCAL:-2,50

SETINCMOUNT:90
→ ACK_SETINCMOUNT:1

SETINCWLAN:1
→ ACK_SETINCWLAN:1

SETINCSHKER:0
→ ACK_SETINCSHKER:1
```

Direkt am Sensor (ohne Rotor):

```
#GETDG$
#ACK_GETDG,45.12$

#SETFILTER,0.35$
#ACK_SETFILTER,0.35$

#SETSHOCKERR,0$
#ACK_SETSHOCKERR,0$
```

---

## 10. Kurzüberblick Proxy-Befehle

| Befehl | Beschreibung |
|--------|----------------|
| `GETINCVER` | Sensor-Firmware-Version |
| `GETINCUID` | Geräte-UID / SoftAP-Suffix |
| `GETINCDG` | Elevationswinkel vom Sensor (Diagnose) |
| `GETINCSWAPXY` / `SETINCSWAPXY` | Achsen X/Y tauschen |
| `GETINCINVREF` / `SETINCINVREF` | Referenz invertieren |
| `GETINCINVSEN` / `SETINCINVSEN` | Sense invertieren |
| `GETINCINVROT` / `SETINCINVROT` | Drehrichtung umkehren |
| `GETINCMOUNT` / `SETINCMOUNT` | Montage 0/90/180/270 |
| `GETINCLIM180` / `SETINCLIM180` | Soft-Limit 0…180° |
| `GETINCCAL` / `SETINCCAL` | Kalibrier-Offset ° |
| `GETINCFS1G` / `SETINCFS1G` | Full-Scale ±1 g / ±2 g |
| `GETINCDEC` / `SETINCDEC` | Nachkommastellen 0/1/2 |
| `GETINCFILT` / `SETINCFILT` | EMA-Filter-Alpha |
| `GETINCSHKTH` / `SETINCSHKTH` | Stoßschwelle |
| `GETINCSHKWN` / `SETINCSHKWN` | Ruck-Zeitfenster |
| `GETINCSHKLM` / `SETINCSHKLM` | Rucke bis Fehler |
| `GETINCSHKCT` | Ruck-Zähler (nur GET) |
| `GETINCSHKER` / `SETINCSHKER` | Ruck-Fehler lesen / mit `:0` löschen |
| `GETINCSTL` | Mechanik stabil (nur GET) |
| `GETINCSHK` | Stoß erkannt (nur GET) |
| `GETINCWLAN` / `SETINCWLAN` | SoftAP am Sensor |
| `GETINCDBG` / `SETINCDBG` | USB-Debug am Sensor |
