param(
    [Parameter(Mandatory = $true)]
    [string]$VitisSrc,

    [string]$DataDir = ""
)

$ErrorActionPreference = "Stop"

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$BringupRoot = Resolve-Path (Join-Path $ScriptDir "..")
$RepoRoot = Resolve-Path (Join-Path $BringupRoot "..")

if ($DataDir -eq "") {
    $DataDir = Join-Path $RepoRoot "data"
}

$VectorHeader = Join-Path $BringupRoot "vitis_app_src\turbo_test_vectors.h"
python (Join-Path $ScriptDir "generate_vitis_vectors.py") --data-dir $DataDir --output $VectorHeader

New-Item -ItemType Directory -Force -Path $VitisSrc | Out-Null
Copy-Item -Path (Join-Path $BringupRoot "vitis_app_src\*") -Destination $VitisSrc -Recurse -Force

Write-Host "Copied Vitis app sources to $VitisSrc"
Write-Host "Next: build the application in Vitis and run it on ps7_cortexa9_0."
