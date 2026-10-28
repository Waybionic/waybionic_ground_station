# Window 1: start RViz + UDP joint-state receiver (Docker WSLg).
# Then in Window 2 run start_teleop.ps1
param([int]$VizPort = 47301)

$ErrorActionPreference = "Stop"
$BridgeRoot = Split-Path $PSScriptRoot -Parent
# Package lives inside the ground-station checkout.
$GsRoot = Split-Path $BridgeRoot -Parent

$env:Path = [System.Environment]::GetEnvironmentVariable("Path", "Machine") + ";" +
            [System.Environment]::GetEnvironmentVariable("Path", "User")

if (-not (Get-Command docker -ErrorAction SilentlyContinue)) {
    throw "docker not found - start Docker Desktop first."
}
if (-not (Test-Path (Join-Path $GsRoot "docker-compose.yml")) -and
    -not (Test-Path (Join-Path $GsRoot "compose.yaml")) -and
    -not (Test-Path (Join-Path $GsRoot "compose.yml"))) {
    # compose file may live at repo root under various names; still require repo root.
    if (-not (Test-Path (Join-Path $GsRoot "waybionic_bringup"))) {
        throw "Missing ground station root at $GsRoot"
    }
}

Write-Host "Bridge mount: $BridgeRoot"
Write-Host "Ground station: $GsRoot"
Write-Host "Starting RViz. When it opens, run start_teleop.ps1 in another PowerShell."
Write-Host "Ctrl+C here stops RViz."
Write-Host ""

Set-Location $GsRoot
docker compose run --rm --build `
    -p "127.0.0.1:${VizPort}:${VizPort}/udp" `
    -v "${BridgeRoot}:/embedded:ro" `
    -e "VIZ_PORT=$VizPort" `
    wslg `
    bash /embedded/scripts/container_rviz_embedded.sh
