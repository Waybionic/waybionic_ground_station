# Starts the self-running teleop demo on Windows in one step: Docker Desktop, the controller
# bridge (if Python is installed) and the ground station with autoplay. Run windows-demo.cmd,
# or: powershell -ExecutionPolicy Bypass -File scripts\windows-demo.ps1

$root = Split-Path -Parent $PSScriptRoot
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

# The bridge needs only Python 3. Without Python installed, Windows still has a python.exe
# placeholder that fails, so try each command.
$python = $null
foreach ($name in 'py', 'python', 'python3') {
    $command = Get-Command $name -ErrorAction SilentlyContinue
    if (-not $command) {
        continue
    }
    & $command.Source --version *> $null
    if ($LASTEXITCODE -eq 0) {
        $python = $command
        break
    }
}
$bridge = $null
if ($python) {
    $arguments = @('-m', 'waybionic_teleop.xinput_bridge')
    if ($python.Name -eq 'py.exe') {
        $arguments = @('-3') + $arguments
    }
    $bridge = Start-Process -FilePath $python.Source -ArgumentList $arguments -PassThru `
        -WorkingDirectory (Join-Path $root 'waybionic_teleop')
} else {
    Write-Warning 'Python 3 was not found, so the demo runs but a controller cannot take over.'
}

Push-Location $root
try {
    Write-Host 'Starting the ground station; press Ctrl+C here to stop.'
    & $docker compose run --rm --build --service-ports wslg-demo
} finally {
    Pop-Location
    if ($bridge -and -not $bridge.HasExited) {
        Stop-Process -Id $bridge.Id
    }
}
