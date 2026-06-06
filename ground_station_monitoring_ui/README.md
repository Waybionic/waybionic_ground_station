# WayBionic Ground Station Monitoring Interface

This folder contains a first desktop UI proof of concept for the WayBionic engineering and operations ground station. It is a standalone Python + PySide6/Qt dashboard focused on monitoring diagnostic health and live values.

The prototype uses locally generated mock diagnostic data immediately. It does not depend on robot hardware, sensors, ROS topics, PCBs, cameras, or the doctor/surgeon controller.

## Scope

This app is monitoring-only.

- It does not send motor commands.
- It does not participate in the real-time safety-critical control loop.
- It does not implement video streaming.
- It does not implement robot control or arm simulation.

The interface is intended to give operators and engineers an early ground station layout while the live robotics integrations are still being built.

## UI Regions

- Top bar: demo state controls, current NORMAL/FAULT state, title, and last-updated indicator.
- System Status: diagnostic source, future ROS 2 connection status, backend heartbeat, UI mode, and safety note.
- Telemetry + Live Values: compact diagnostic table for current readings.
- Current Alerts: derived from any diagnostic signal whose status is not `OK`.
- Surgeon Camera View: reserved placeholder; no video streaming in this sprint.
- Robot / Arm Visualization: reserved placeholder; monitoring-only for now.

## Install

From this folder:

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
```

On Linux/macOS:

```bash
python -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
```

## Run

```powershell
python app.py
```

## Demo Modes

Use the top-left buttons to switch states:

- `Normal Demo`: all mock diagnostic signals report `OK`, and the alerts panel shows `No active alerts`.
- `Fault Demo`: board temperature reports a high-temperature `FAULT`, IMU heartbeat reports `STALE`, and alerts become visually urgent.

The dashboard refreshes roughly once per second and updates the telemetry table, alert panel, backend heartbeat, state indicator, and last-updated text from the current diagnostic source.

## Future ROS 2 Integration

The UI depends on normalized `DiagnosticMessage` objects, not on mock internals. A future `ROS2DiagnosticsSubscriber` should replace `MockDiagnosticsSource` and convert live ROS 2 diagnostics into the same normalized contract before the data reaches the UI.

See `docs/DIAGNOSTICS_CONTRACT.md` for the expected data shape and backend integration notes.
