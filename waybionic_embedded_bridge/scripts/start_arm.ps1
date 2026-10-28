# One command: start RViz twin (Docker) + keyboard teleop (serial).
# Docker cannot open COM ports on Windows, so serial stays on the host.
param(
    [string]$Port = "COM3",
    [int]$VizPort = 47301
)

$ErrorActionPreference = "Stop"
$BridgeRoot = Split-Path $PSScriptRoot -Parent
$GsRoot = Split-Path $BridgeRoot -Parent

$env:Path = [System.Environment]::GetEnvironmentVariable("Path", "Machine") + ";" +
            [System.Environment]::GetEnvironmentVariable("Path", "User")
$env:PYTHONPATH = $BridgeRoot

if (-not (Get-Command docker -ErrorAction SilentlyContinue)) {
    throw "docker not found - start Docker Desktop first."
}
if (-not (Test-Path (Join-Path $GsRoot "waybionic_bringup"))) {
    throw "Missing ground station root at $GsRoot"
}

Get-ChildItem $BridgeRoot -Recurse -Filter "__pycache__" -Directory -ErrorAction SilentlyContinue |
    Remove-Item -Recurse -Force -ErrorAction SilentlyContinue

Write-Host "Starting RViz container in the background..."
$rvizJob = Start-Job -ScriptBlock {
    param($GsRoot, $BridgeRoot, $VizPort)
    Set-Location $GsRoot
    docker compose run --rm `
        -p "127.0.0.1:${VizPort}:${VizPort}/udp" `
        -v "${BridgeRoot}:/embedded:ro" `
        -e "VIZ_PORT=$VizPort" `
        wslg `
        bash /embedded/scripts/container_rviz_embedded.sh
} -ArgumentList $GsRoot, $BridgeRoot, $VizPort

Write-Host "Waiting for RViz / UDP receiver to come up..."
Start-Sleep -Seconds 12

Write-Host ""
Write-Host "Teleop starting. Hold arrows/WASD, release to stop. Ctrl+C stops everything."
Write-Host "Close other apps using $Port first."
Write-Host ""

try {
    Set-Location $BridgeRoot
    python -B -m waybionic_embedded_bridge.keyboard_teleop --port $Port --viz --viz-port $VizPort
}
finally {
    Write-Host "Stopping RViz container / job..."
    Stop-Job $rvizJob -ErrorAction SilentlyContinue
    Receive-Job $rvizJob -ErrorAction SilentlyContinue | Out-Null
    Remove-Job $rvizJob -Force -ErrorAction SilentlyContinue
    docker ps -q --filter "name=waybionic_ground_station-wslg" | ForEach-Object {
        docker stop $_ 2>$null | Out-Null
    }
}
