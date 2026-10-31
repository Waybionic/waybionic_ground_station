# Physical-arm teleop + RViz joint-state UDP (needs start_rviz.ps1 in another window).
param(
    [string]$Port = "COM7",
    [int]$VizPort = 47301,
    [switch]$NoViz
)

$ErrorActionPreference = "Stop"
$BridgeRoot = Split-Path $PSScriptRoot -Parent
$env:PYTHONPATH = $BridgeRoot
$env:PYTHONUNBUFFERED = "1"

Get-ChildItem $BridgeRoot -Recurse -Filter "__pycache__" -Directory -ErrorAction SilentlyContinue |
    Remove-Item -Recurse -Force -ErrorAction SilentlyContinue

# Drop stale teleop so COM isn't left locked from a previous run / agent.
Get-CimInstance Win32_Process -ErrorAction SilentlyContinue |
    Where-Object { $_.CommandLine -match "keyboard_teleop" } |
    ForEach-Object {
        Write-Host "Stopping leftover teleop PID $($_.ProcessId)..."
        Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue
    }
Start-Sleep -Milliseconds 400

Write-Host "Close Serial Monitor / Tk GUI using $Port."
Write-Host "E=ENABLE  Q=DISABLE  M=SIM-ALIGN  H=HOME  Z=ZERO  Esc=ESTOP"
Write-Host "Xbox: A=ENABLE B=DISABLE Y=HOME X=ZERO Start=SIM-ALIGN Back=ESTOP"
Write-Host "Hold arrows/WASD or sticks; release=stop. Ctrl+C releases $Port."
if (-not $NoViz) {
    Write-Host "RViz twin ON (UDP $VizPort) - start_rviz.ps1 must be running in another window."
}
Write-Host ""

Set-Location $BridgeRoot
try {
    if ($NoViz) {
        python -u -B -m waybionic_embedded_bridge.keyboard_teleop --port $Port
    } else {
        python -u -B -m waybionic_embedded_bridge.keyboard_teleop --port $Port --viz --viz-port $VizPort
    }
} finally {
    # Belt-and-suspenders: if Python died hard, kill any teleop still holding the port.
    Get-CimInstance Win32_Process -ErrorAction SilentlyContinue |
        Where-Object { $_.CommandLine -match "keyboard_teleop" } |
        ForEach-Object {
            Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue
        }
}
