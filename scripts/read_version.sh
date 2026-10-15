#!/usr/bin/env bash
# Liest MAJOR.MINOR.PATCH aus src/Version.h
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
H="$ROOT/src/Version.h"
MAJOR=$(grep -oE '#define[[:space:]]+FW_VERSION_MAJOR[[:space:]]+[0-9]+' "$H" | awk '{print $3}')
MINOR=$(grep -oE '#define[[:space:]]+FW_VERSION_MINOR[[:space:]]+[0-9]+' "$H" | awk '{print $3}')
PATCH=$(grep -oE '#define[[:space:]]+FW_VERSION_PATCH[[:space:]]+[0-9]+' "$H" | awk '{print $3}')
echo "${MAJOR}.${MINOR}.${PATCH}"
