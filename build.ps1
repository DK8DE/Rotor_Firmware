#Requires -Version 5.1
<#
.SYNOPSIS
  Rotor_Firmware bauen und per USB auf ESP32-S3 flashen (PlatformIO).

.EXAMPLE
  .\build.ps1
  pio run -t upload -e esp32-s3-n8r8

.EXAMPLE
  .\build.ps1 -Clean
  Zuerst clean, dann upload.
#>
[CmdletBinding()]
param(
    [switch]$Clean
)

$ErrorActionPreference = 'Stop'
Set-Location -LiteralPath $PSScriptRoot

function Invoke-Step {
    param([string]$Label, [scriptblock]$Action)
    Write-Host "`n=== $Label ===" -ForegroundColor Cyan
    & $Action
    if ($LASTEXITCODE -ne 0) {
        throw "Schritt fehlgeschlagen (Exit $LASTEXITCODE): $Label"
    }
}

try {
    if ($Clean) {
        Invoke-Step "pio clean" { pio run -t clean }
    }
    Invoke-Step "upload Firmware (esp32-s3-n8r8)" { pio run -t upload -e esp32-s3-n8r8 }
    Write-Host "`nFertig." -ForegroundColor Green
}
catch {
    Write-Host "`n$($_.Exception.Message)" -ForegroundColor Red
    exit 1
}
