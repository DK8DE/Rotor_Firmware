#pragma once

// ============================================================================
// Firmware-Version (Semantic Versioning: MAJOR.MINOR.PATCH)
// ============================================================================
// Dies ist die EINZIGE Stelle, an der die Firmware-Versionsnummer gepflegt wird.
// - main.cpp gibt die Version beim Start aus (Debug-Log).
// - Rs485Dispatcher beantwortet GETVERSION damit (ACK_GETVERSION:<version>).
// - build.ps1 liest diese Datei aus und uebernimmt die Version in IMGs/manifest.json.
//
// Bei jeder Aenderung, die auf ein neues Firmware-Image geflasht wird, sollte
// mindestens PATCH erhoeht werden (MINOR fuer neue Features, MAJOR fuer
// nicht-kompatible Aenderungen an RS485-Protokoll/NVS-Layout).
#define FW_VERSION_MAJOR 1
#define FW_VERSION_MINOR 5
#define FW_VERSION_PATCH 0

#define FW_VERSION_STRINGIFY_(x) #x
#define FW_VERSION_STRINGIFY(x) FW_VERSION_STRINGIFY_(x)

#define FW_VERSION_STRING \
  FW_VERSION_STRINGIFY(FW_VERSION_MAJOR) "." \
  FW_VERSION_STRINGIFY(FW_VERSION_MINOR) "." \
  FW_VERSION_STRINGIFY(FW_VERSION_PATCH)
