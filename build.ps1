#Requires -Version 5.1
<#
.SYNOPSIS
  Rotor_Firmware bauen, Images fuer ESP Web Tools nach IMGs/ kopieren und optional flashen.

.DESCRIPTION
  - Baut die Firmware (PlatformIO, env esp32-s3-n8r8)
  - Legt/aktualisiert den Ordner IMGs/ mit factory.bin (merge), firmware.bin
    und einer manifest.json fuer ESP Web Tools (ein Part ab 0x0)
  - Ohne -SkipUpload: anschliessend Upload auf den in platformio.ini konfigurierten Port

.EXAMPLE
  .\build.ps1
  Bauen, IMGs aktualisieren, upload.

.EXAMPLE
  .\build.ps1 -SkipUpload
  Nur bauen und IMGs aktualisieren (kein Flash).

.EXAMPLE
  .\build.ps1 -Clean -SkipUpload
  Clean, bauen, IMGs aktualisieren.

.EXAMPLE
  .\build.ps1 -Version "1.3.1"
  Setzt die Firmware-Version auf 1.3.1 (src/Version.h + README.md), baut, aktualisiert IMGs, laedt hoch.
  Committen/Pushen dieser Aenderung nach main/master erstellt automatisch ein GitHub-Release (siehe README).
#>
[CmdletBinding()]
param(
    [switch]$Clean,
    [switch]$SkipUpload,
    [string]$Version
)

$ErrorActionPreference = 'Stop'
Set-Location -LiteralPath $PSScriptRoot

$EnvName = 'esp32-s3-n8r8'
$BuildDir = Join-Path $PSScriptRoot ".pio\build\$EnvName"
$ImgsDir = Join-Path $PSScriptRoot 'IMGs'

function Invoke-Step {
    param([string]$Label, [scriptblock]$Action)
    Write-Host "`n=== $Label ===" -ForegroundColor Cyan
    & $Action
    if ($null -ne $LASTEXITCODE -and $LASTEXITCODE -ne 0) {
        throw "Schritt fehlgeschlagen (Exit $LASTEXITCODE): $Label"
    }
}

function Get-FirmwareVersion {
    $versionFile = Join-Path $PSScriptRoot 'src\Version.h'
    if (-not (Test-Path -LiteralPath $versionFile)) {
        return $null
    }

    $content = Get-Content -LiteralPath $versionFile -Raw
    $major = [regex]::Match($content, '#define\s+FW_VERSION_MAJOR\s+(\d+)').Groups[1].Value
    $minor = [regex]::Match($content, '#define\s+FW_VERSION_MINOR\s+(\d+)').Groups[1].Value
    $patch = [regex]::Match($content, '#define\s+FW_VERSION_PATCH\s+(\d+)').Groups[1].Value

    if ([string]::IsNullOrEmpty($major) -or [string]::IsNullOrEmpty($minor) -or [string]::IsNullOrEmpty($patch)) {
        return $null
    }

    return "$major.$minor.$patch"
}

function Set-FirmwareVersion {
    param([Parameter(Mandatory)][string]$NewVersion)

    if ($NewVersion -notmatch '^\d+\.\d+\.\d+$') {
        throw "Ungueltiges Versionsformat '$NewVersion'. Erwartet: MAJOR.MINOR.PATCH (z.B. 1.3.1)"
    }

    $versionFile = Join-Path $PSScriptRoot 'src\Version.h'
    if (-not (Test-Path -LiteralPath $versionFile)) {
        throw "src\Version.h nicht gefunden."
    }

    $parts = $NewVersion.Split('.')
    $content = Get-Content -LiteralPath $versionFile -Raw
    $content = [regex]::Replace($content, '(#define\s+FW_VERSION_MAJOR\s+)\d+', "`${1}$($parts[0])")
    $content = [regex]::Replace($content, '(#define\s+FW_VERSION_MINOR\s+)\d+', "`${1}$($parts[1])")
    $content = [regex]::Replace($content, '(#define\s+FW_VERSION_PATCH\s+)\d+', "`${1}$($parts[2])")
    [System.IO.File]::WriteAllText($versionFile, $content)

    Write-Host "src\Version.h aktualisiert -> $NewVersion" -ForegroundColor Cyan
}

function Sync-ReadmeVersion {
    param([Parameter(Mandatory)][string]$CurrentVersion)

    $readmePath = Join-Path $PSScriptRoot 'README.md'
    if (-not (Test-Path -LiteralPath $readmePath)) {
        return
    }

    $content = Get-Content -LiteralPath $readmePath -Raw
    $match = [regex]::Match($content, '\*\*Version:\s*([\d.]+)\*\*')
    if (-not $match.Success) {
        return
    }

    $readmeVersion = $match.Groups[1].Value
    if ($readmeVersion -eq $CurrentVersion) {
        return
    }

    # Alle "nackten" Vorkommen der alten Versionsnummer (Badge + Beispiele im
    # Versionierungs-Abschnitt, z.B. "v1.3.0" / "ACK_GETVERSION:1.3.0") durch
    # die neue Version ersetzen. Lookaround verhindert Treffer innerhalb
    # anderer Zahlen (z.B. "11.3.0").
    $escapedOld = [regex]::Escape($readmeVersion)
    $pattern = "(?<![\d.])$escapedOld(?![\d.])"
    $updated = [regex]::Replace($content, $pattern, $CurrentVersion)

    if ($updated -ne $content) {
        [System.IO.File]::WriteAllText($readmePath, $updated)
        Write-Host "README.md aktualisiert: Version $readmeVersion -> $CurrentVersion" -ForegroundColor Cyan
    }
}

function Find-BootApp0 {
    $candidates = @(
        (Join-Path $env:USERPROFILE '.platformio\packages\framework-arduinoespressif32\tools\partitions\boot_app0.bin'),
        (Join-Path $env:USERPROFILE '.platformio\packages\framework-arduinoespressif32-libs\tools\partitions\boot_app0.bin')
    )
    foreach ($p in $candidates) {
        if (Test-Path -LiteralPath $p) { return $p }
    }
    $found = Get-ChildItem -Path (Join-Path $env:USERPROFILE '.platformio\packages') `
        -Filter 'boot_app0.bin' -Recurse -ErrorAction SilentlyContinue |
        Select-Object -First 1 -ExpandProperty FullName
    if ($found) { return $found }
    return $null
}

function Update-ImgsFolder {
    Write-Host "`n=== IMGs aktualisieren ===" -ForegroundColor Cyan

    $required = @{
        'bootloader.bin' = (Join-Path $BuildDir 'bootloader.bin')
        'partitions.bin' = (Join-Path $BuildDir 'partitions.bin')
        'firmware.bin'   = (Join-Path $BuildDir 'firmware.bin')
    }

    foreach ($name in $required.Keys) {
        $src = $required[$name]
        if (-not (Test-Path -LiteralPath $src)) {
            throw "Build-Artefakt fehlt: $src"
        }
    }

    $bootApp0Src = Find-BootApp0
    if (-not $bootApp0Src) {
        throw "boot_app0.bin nicht gefunden (PlatformIO-Packages). Bitte einmal erfolgreich bauen/uploaden."
    }

    if (Test-Path -LiteralPath $ImgsDir) {
        Remove-Item -LiteralPath $ImgsDir -Recurse -Force
    }
    New-Item -ItemType Directory -Path $ImgsDir | Out-Null

    Copy-Item -LiteralPath $required['bootloader.bin'] -Destination (Join-Path $ImgsDir 'bootloader.bin') -Force
    Copy-Item -LiteralPath $required['partitions.bin'] -Destination (Join-Path $ImgsDir 'partitions.bin') -Force
    Copy-Item -LiteralPath $bootApp0Src -Destination (Join-Path $ImgsDir 'boot_app0.bin') -Force
    Copy-Item -LiteralPath $required['firmware.bin'] -Destination (Join-Path $ImgsDir 'firmware.bin') -Force

    $factoryPath = Join-Path $ImgsDir 'factory.bin'
    Write-Host "factory.bin mergen (esptool)..." -ForegroundColor Cyan
    $mergeArgs = @(
        '-m', 'esptool', '--chip', 'esp32s3', 'merge_bin',
        '-o', $factoryPath,
        '--flash-mode', 'qio', '--flash-freq', '80m', '--flash-size', '8MB',
        '0x0', $required['bootloader.bin'],
        '0x8000', $required['partitions.bin'],
        '0xe000', $bootApp0Src,
        '0x10000', $required['firmware.bin']
    )
    # esptool schreibt Hinweise auf stderr -> bei 'Stop' wuerde PowerShell abbrechen
    $prevEap = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    & python @mergeArgs 2>&1 | ForEach-Object { Write-Host $_ }
    $mergeExit = $LASTEXITCODE
    $ErrorActionPreference = $prevEap
    $LASTEXITCODE = $mergeExit
    if ($null -ne $LASTEXITCODE -and $LASTEXITCODE -ne 0) {
        throw "esptool merge_bin fehlgeschlagen (Exit $LASTEXITCODE). Ist esptool installiert? (pip install esptool)"
    }

    $fwVersion = Get-FirmwareVersion
    $version = if ($fwVersion) { $fwVersion } else { (Get-Date -Format 'yyyy-MM-dd') }
    $manifest = [ordered]@{
        name                     = 'Rotor DK8DE'
        version                  = $version
        new_install_prompt_erase = $true
        builds                   = @(
            [ordered]@{
                chipFamily = 'ESP32-S3'
                parts      = @(
                    [ordered]@{ path = 'factory.bin'; offset = 0 }
                )
            }
        )
    }

    $manifestPath = Join-Path $ImgsDir 'manifest.json'
    $json = ($manifest | ConvertTo-Json -Depth 6) + "`n"
    # UTF-8 ohne BOM (Web-Flasher / Browser)
    [System.IO.File]::WriteAllText($manifestPath, $json)

    $readmeTxt = @"
Rotor DK8DE Firmware $version

factory.bin  - Komplettflash ab Adresse 0 (USB-Webflasher / esptool)
firmware.bin - nur App fuer SoftAP-OTA (Web-Tab Update)

USB-Webflasher:
  https://dk8de.github.io/Rotor_Firmware/

SoftAP-OTA:
  SETWLAN:1 -> http://192.168.4.1 (admin / Rotorconfig) -> Update -> firmware.bin

Beispiel Komplettflash:
  esptool.py --chip esp32s3 write_flash 0x0 factory.bin
"@
    [System.IO.File]::WriteAllText((Join-Path $ImgsDir 'README.txt'), $readmeTxt + "`n")

    Write-Host "IMGs aktualisiert (Firmware-Version: $version):" -ForegroundColor Green
    Get-ChildItem -LiteralPath $ImgsDir | ForEach-Object {
        Write-Host ("  {0,-18} {1,10:N0} Bytes" -f $_.Name, $_.Length)
    }
    Write-Host "  USB-Webflasher: factory.bin @ 0x0 | SoftAP-OTA: firmware.bin"
}

try {
    if ($Version) {
        $currentVersion = Get-FirmwareVersion
        if ($currentVersion -ne $Version) {
            Set-FirmwareVersion -NewVersion $Version
        }
        else {
            Write-Host "Firmware-Version ist bereits $Version." -ForegroundColor Yellow
        }
    }

    # README.md immer mit src/Version.h abgleichen (auch wenn Version.h manuell
    # geaendert wurde, ohne -Version zu benutzen).
    $fwVersionForReadme = Get-FirmwareVersion
    if ($fwVersionForReadme) {
        Sync-ReadmeVersion -CurrentVersion $fwVersionForReadme
    }

    if ($Clean) {
        Invoke-Step "pio clean ($EnvName)" { pio run -t clean -e $EnvName }
    }

    Invoke-Step "pio build ($EnvName)" { pio run -e $EnvName }

    Update-ImgsFolder

    if (-not $SkipUpload) {
        Invoke-Step "upload Firmware ($EnvName)" { pio run -t upload -e $EnvName }
        Write-Host "`nFertig (Build + IMGs + Upload)." -ForegroundColor Green
    }
    else {
        Write-Host "`nFertig (Build + IMGs, Upload uebersprungen)." -ForegroundColor Green
    }
}
catch {
    Write-Host "`n$($_.Exception.Message)" -ForegroundColor Red
    exit 1
}
