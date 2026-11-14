# Drives the real arm from Windows in one step: Docker Desktop, the carrier bridge, the
# controller bridge and the ground station. Docker Desktop cannot open COM ports, so the
# carrier bridge relays the carrier's port to 127.0.0.1:47320 and the drive node connects to it.
# Run windows-arm.cmd, or: powershell -ExecutionPolicy Bypass -File scripts\windows-arm.ps1
# The bridge finds the carrier's COM port itself. With several Arduinos or USB serial devices
# plugged in, name it: windows-arm.cmd COM5 (or set WAYBIONIC_CARRIER=COM5).
param([string]$Port = $env:WAYBIONIC_CARRIER)

$root = Split-Path -Parent $PSScriptRoot
$teleop = Join-Path $root 'waybionic_teleop'
$carrierPort = 47320

$docker = Join-Path $env:LOCALAPPDATA 'Programs\DockerDesktop\resources\bin\docker.exe'
if (-not (Test-Path $docker)) {
    $command = Get-Command docker -ErrorAction SilentlyContinue
    if (-not $command) {
        Write-Error 'Docker Desktop is not installed; see BuildInstructions.md.'
        exit 1
    }
    $docker = $command.Source
}

& $docker info *> $null
if ($LASTEXITCODE -ne 0) {
    $app = @(
        (Join-Path $env:LOCALAPPDATA 'Programs\DockerDesktop\Docker Desktop.exe'),
        (Join-Path $env:ProgramFiles 'Docker\Docker\Docker Desktop.exe')
    ) | Where-Object { Test-Path $_ } | Select-Object -First 1
    if (-not $app) {
        Write-Error 'Docker is not running and Docker Desktop was not found.'
        exit 1
    }
    Write-Host 'Starting Docker Desktop...'
    Start-Process $app
    $deadline = (Get-Date).AddMinutes(3)
    do {
        Start-Sleep -Seconds 3
        & $docker info *> $null
    } until ($LASTEXITCODE -eq 0 -or (Get-Date) -gt $deadline)
    if ($LASTEXITCODE -ne 0) {
        Write-Error 'Docker Desktop did not start within 3 minutes.'
        exit 1
    }
}

$python = $null
$pythonArguments = @()
foreach ($name in 'python', 'python3', 'py') {
    $command = Get-Command $name -ErrorAction SilentlyContinue
    if (-not $command) {
        continue
    }
    $prefix = if ($command.Name -eq 'py.exe') { @('-3') } else { @() }
    & $command.Source @prefix -c 'import sys; sys.exit(sys.version_info[0] != 3)' *> $null
    if ($LASTEXITCODE -eq 0) {
        $python, $pythonArguments = $command, $prefix
        break
    }
}
if (-not $python) {
    Write-Error 'The real arm needs Python 3 on Windows for the carrier bridge.'
    exit 1
}
& $python.Source @pythonArguments -c 'import serial' *> $null
if ($LASTEXITCODE -ne 0) {
    Write-Error 'The carrier bridge needs pyserial: python -m pip install pyserial'
    exit 1
}

# The bridge also writes what it says here, so that this script can show it if the bridge fails.
$bridgeLog = Join-Path $env:TEMP 'waybionic-carrier-bridge.log'
Remove-Item $bridgeLog -ErrorAction SilentlyContinue
$bridgeArguments = @('-m', 'waybionic_teleop.carrier_relay', '--log-file', $bridgeLog)
if ($Port) {
    $bridgeArguments += @('--port', $Port)
}

$carrier = $null
$controller = $null
try {
    $carrier = Start-Process -FilePath $python.Source -PassThru -WorkingDirectory $teleop `
        -ArgumentList ($pythonArguments + $bridgeArguments)

    Write-Host ('Waiting for the carrier (up to a minute): plug it in if you have not; the ' +
        'bridge window says what it sees...')
    $deadline = (Get-Date).AddSeconds(60)
    do {
        Start-Sleep -Milliseconds 500
        $listening = Get-NetTCPConnection -LocalPort $carrierPort -State Listen `
            -ErrorAction SilentlyContinue
    } until ($listening -or $carrier.HasExited -or (Get-Date) -gt $deadline)
    if (-not $listening) {
        Write-Host 'The carrier bridge said:' -ForegroundColor Yellow
        if (Test-Path $bridgeLog) {
            Get-Content $bridgeLog -Tail 6 | ForEach-Object { Write-Host "  $_" }
        } else {
            Write-Host '  (nothing)'
        }
        Write-Error ('The carrier bridge is not ready. Plug in only the carrier, or name its ' +
            'port: scripts\windows-arm.cmd COM5')
        exit 1
    }

    $controller = Start-Process -FilePath $python.Source -PassThru -WorkingDirectory $teleop `
        -ArgumentList ($pythonArguments + @('-m', 'waybionic_teleop.xinput_bridge'))

    Push-Location $root
    try {
        Write-Host 'Starting the ground station; press Ctrl+C here to stop.'
        Write-Host ('When RViz opens, follow its Arm setup panel: put the arm in its zero pose, ' +
            'press Zero the arm, then press Start (Menu) on the controller. Close RViz to stop.')
        & $docker compose run --rm --build --service-ports wslg-arm
    } finally {
        Pop-Location
    }
} finally {
    foreach ($process in $controller, $carrier) {
        if ($process -and -not $process.HasExited) {
            Stop-Process -Id $process.Id
        }
    }
}