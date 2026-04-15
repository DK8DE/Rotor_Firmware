# Rotor_Firmware

Firmware fuer die Rotorsteuerung auf ESP32-S3 (PlatformIO/Arduino).

Dieses Projekt steuert den Antennenrotor, verarbeitet Endschalter und Encoder, regelt die Bewegung mit Rampenprofilen und stellt die Kommunikation ueber RS485 bereit.

## Projektueberblick

`Rotor_Firmware` ist die Geraetefirmware fuer den Rotor-Regler.  
Die Firmware laeuft direkt auf der Steuerhardware und uebernimmt:

- Positionsfahrten mit Rampen- und Feinphase
- Homing/Referenzfahrt (`SETREF`)
- Sicherheitsfunktionen (z. B. Timeout, Stall, Endschalter)
- RS485-Protokoll fuer Steuerung und Rueckmeldungen
- Persistente Parameter ueber NVS/Preferences

## Zusammenspiel mit PC und Controller

Zum Gesamtsystem gehoeren neben dieser Firmware zwei weitere Projekte:

- **PC-Software (Desktop):** [RotorTcpBridge](https://github.com/DK8DE/RotorTcpBridge)  
  Diese Software verbindet Anwendungen am PC mit dem Rotor-System (z. B. ueber TCP/UDP/seriell, je nach Setup).

- **Controller-Firmware (USB-Bridge + Bedienung):** [Rotor_Display_5](https://github.com/DK8DE/Rotor_Display_5)  
  Der Controller stellt ueber USB eine Bruecke zu RS485 bereit und dient als lokale Bedieneinheit.

## Build und Upload

Das Projekt nutzt PlatformIO.

Beispiele:

```bash
pio run
pio run -t upload
pio run -t monitor
```

Die Zielplattform/Umgebung ist in `platformio.ini` definiert.

## Hinweise

- Fuer reproduzierbare Ergebnisse sollten alle drei Komponenten (Firmware, Controller, PC-Software) kompatibel zueinander gehalten werden.
- Bei Verbindungsproblemen zuerst Verkabelung, RS485-Adressierung und Baudrate pruefen.

---

Wenn du willst, kann ich als naechsten Schritt noch eine kurze Sektion mit typischen RS485-Kommandos (z. B. `GETPOSDG`, `SETPOSDG`, `SETREF`, `STOP`) in die README aufnehmen.
