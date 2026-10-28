# Physical-arm teleop + RViz joint-state UDP (needs start_rviz.ps1 in another window).
param(
    [string]$Port = "COM3",
    [int]$VizPort = 47301,
    [switch]$NoViz
)

$ErrorActionPreference = "Stop"
$BridgeRoot = Split-Path $PSScriptRoot -Parent
$env:PYTHONPATH = $BridgeRoot

Get-ChildItem $BridgeRoot -Recurse -Filter "__pycache__" -Directory -ErrorAction SilentlyContinue |
    Remove-Item -Recurse -Force -ErrorAction SilentlyContinue

Write-Host "Close any other app using $Port (Tk GUI, Arduino Serial Monitor, old teleop)."
Write-Host "Click THIS window. E=ENABLE, hold arrows/WASD to move, release to stop."
if (-not $NoViz) {
    Write-Host "RViz twin ON (UDP $VizPort) - start_rviz.ps1 must be running in another window."
}
Write-Host ""

Set-Location $BridgeRoot
if ($NoViz) {
    python -B -m waybionic_embedded_bridge.keyboard_teleop --port $Port
} else {
    python -B -m waybionic_embedded_bridge.keyboard_teleop --port $Port --viz --viz-port $VizPort
}
