Rotor DK8DE Firmware 1.8.1

factory.bin  - Komplettflash ab Adresse 0 (USB-Webflasher / esptool)
firmware.bin - nur App fuer SoftAP-OTA (Web-Tab Update)

USB-Webflasher:
  https://dk8de.github.io/Rotor_Firmware/

SoftAP-OTA:
  SETWLAN:1 -> http://192.168.4.1 (admin / Rotorconfig) -> Update -> firmware.bin

Beispiel Komplettflash:
  esptool.py --chip esp32s3 write_flash 0x0 factory.bin
