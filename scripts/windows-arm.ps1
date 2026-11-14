# Drives the real arm from Windows in one step: Docker Desktop, the carrier bridge, the
# controller bridge and the ground station. Docker Desktop cannot open COM ports, so the
# carrier bridge relays the carrier's port to 127.0.0.1:47320 and the drive node connects to it.
# Run windows-arm.cmd, or: powershell -ExecutionPolicy Bypass -File scripts\windows-arm.ps1
# The bridge finds the carrier's COM port itself. With several Arduinos or USB serial devices
# plugged in, name it: windows-arm.cmd COM5 (or set WAYBIONIC_CARRIER=COM5).
# Run from a release bundle, this folder holds its own Python and a compose file that runs the
# tested image instead of building one. windows-arm.cmd -SelfTest checks the prerequisites and
# starts nothing.
param([string]$Port = $env:WAYBIONIC_CARRIER, [switch]$SelfTest)

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

$python = $null
$pythonArguments = @()
$bundledPython = Join-Path $root 'python\python.exe'
if (Test-Path $bundledPython) {
    $python = [pscustomobject]@{ Source = $bundledPython }
}
foreach ($name in 'python', 'python3', 'py') {
    if ($python) {
        break
    }
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

$release = Join-Path $root 'compose.release.yaml'
$composeFiles = @()
$buildFlag = @('--build')
if (Test-Path $release) {
    $composeFiles = @('-f', $release)
    $buildFlag = @()
}

if ($SelfTest) {
    Push-Location $teleop
    try {
        & $python.Source @pythonArguments -c 'import waybionic_teleop.carrier_relay, waybionic_teleop.xinput_bridge' *> $null
        $bridges = $LASTEXITCODE
    } finally {
        Pop-Location
    }
    if ($bridges -ne 0) {
        Write-Error 'The carrier and controller bridges do not start; the bundle may be damaged.'
        exit 1
    }
    Write-Host "Docker: $docker"
    Write-Host "Python: $($python.Source), with pyserial"
    Write-Host 'The carrier and controller bridges start.'
    if ($composeFiles.Count -gt 0) {
        Write-Host ('Image: ' + ((Get-Content (Join-Path $root 'VERSION')) -join '; '))
    }
    Write-Host 'Self-test passed.'
    exit 0
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

if ($composeFiles.Count -gt 0) {
    Write-Host 'Checking the ground station image; the first start downloads it, several GB...'
    & $docker compose @composeFiles pull wslg-arm
    if ($LASTEXITCODE -ne 0) {
        Write-Error ('The ground station image could not be downloaded. Check the internet ' +
            'connection. If it says unauthorized, the image is not public yet: ask a software ' +
            'lead, or log in once with: docker login ghcr.io')
        exit 1
    }
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
        & $docker compose @composeFiles run --rm @buildFlag --service-ports wslg-arm
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