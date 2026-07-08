# PR Notes

## Review Screenshots

These images are included only to help reviewers see the UI quickly. They are not part of the runtime package behavior.

| Engineer view — mock normal | Engineer view — mock fault |
| --- | --- |
| ![Engineer mock normal](screenshots/engineer_mock_normal.png) | ![Engineer mock fault](screenshots/engineer_mock_fault.png) |

## Summary

This PR makes `waybionic_rviz_plugins` a focused WayBionic RViz2 diagnostics package. It keeps the engineer monitoring panel, mock/live diagnostics switching, and a temporary `/diagnostics` publisher for local validation.

The branch is rebased onto `main` (clean Waybionic foundation from PR #1). Review cleanup items from the latest review are addressed below.

Camera/doctor placeholder work was removed from this PR and will be handled separately later.

## Merge Review Fixes

| Item | Fix |
|------|-----|
| `temporary_diagnostics_publisher.py` + `--symlink-install` | Source script marked executable in git (`100755`); metadata test asserts `os.X_OK` |
| Live diagnostics override | `DiagnosticsPanel::onInitialize()` defers ROS parameter application until after RViz `load()`; hardcoded `Use Mock Diagnostics` removed from `engineer_monitoring_view.rviz` |
| Archived camera placeholder | `ground_station_monitoring_ui_archived/` removed |
| AR4/Annin demo helper | `engineer_ar4_demo.launch.py` and `engineer_ar4_demo.rviz` removed; docs updated |

## Primary Launch Commands

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths . --ignore-src -r -y
colcon build --packages-select waybionic_rviz_plugins waybionic_description waybionic_bringup --symlink-install
source install/setup.bash
ros2 launch waybionic_rviz_plugins engineer_view.launch.py
```

Mock diagnostics (default):

```bash
ros2 launch waybionic_rviz_plugins engineer_view.launch.py use_mock_diagnostics:=true
```

Live diagnostics:

```bash
# Terminal 1
ros2 launch waybionic_rviz_plugins temporary_diagnostics_publisher.launch.py mode:=cycle

# Terminal 2
ros2 launch waybionic_rviz_plugins engineer_view.launch.py use_mock_diagnostics:=false diagnostics_topic:=/diagnostics
```

Temporary backend/demo publisher:

```bash
ros2 launch waybionic_rviz_plugins temporary_diagnostics_publisher.launch.py
ros2 launch waybionic_rviz_plugins temporary_diagnostics_publisher.launch.py mode:=cycle
```

Foundation coexistence (placeholder robot + diagnostics):

```bash
# Terminal 1
ros2 launch waybionic_bringup display.launch.py

# Terminal 2
ros2 launch waybionic_rviz_plugins temporary_diagnostics_publisher.launch.py mode:=normal
ros2 launch waybionic_rviz_plugins engineer_view.launch.py use_mock_diagnostics:=false
```

## What Works

- Engineer RViz view with the `WayBionic Diagnostics` panel.
- Mock normal and fault validation states.
- Alert banner/table rendering from normalized `DiagnosticMessage` rows.
- Live diagnostics source path that subscribes to a `diagnostic_msgs/msg/DiagnosticArray` topic.
- Launch arguments override saved RViz panel mock settings.
- Temporary `/diagnostics` publisher with `normal`, `fault`, `stale`, and `cycle` modes.
- Package metadata and lint/test wiring via `colcon test`.
- Only `DiagnosticsPanel` is registered in `plugin_description.xml`.
- Coexists in the same workspace as `waybionic_description` and `waybionic_bringup` after rebase onto `main`.

## Mock Diagnostics Mode

Mock mode is enabled by default through `use_mock_diagnostics:=true`. It uses synthetic values and keeps `Mock Normal` and `Mock Fault` controls enabled for UI validation while backend `/diagnostics` publishing is not ready.

## Live `/diagnostics` Path

`RosDiagnosticsSource` subscribes to `/diagnostics` by default, or another topic passed as `diagnostics_topic:=<topic>`, and converts each status into the internal `DiagnosticMessage` model before the panel renders it.

See `DIAGNOSTICS_BACKEND_INTEGRATION.md` for backend replacement guidance.

Expected mapping:

- `DiagnosticStatus.name` -> `signal_name`
- `OK` -> `OK`
- `WARN` -> `WARN`
- `ERROR` -> `FAULT`
- `STALE` -> `STALE`
- `DiagnosticStatus.message` -> `alert_message` for non-OK rows
- `DiagnosticStatus.values` -> `value` and `unit` when those keys are present
- `DiagnosticArray.header.stamp`, or receive time when unset, -> `timestamp`

## Temporary Diagnostics Publisher

`scripts/temporary_diagnostics_publisher.py` publishes sample `DiagnosticArray` messages for local testing while Korede/backend publishing is unavailable.

Modes:

- `normal` — OK telemetry
- `fault` — high board temperature + stale IMU heartbeat
- `stale` — all sample signals STALE
- `cycle` — rotate through normal/fault/stale every 5 seconds

## Lint / Test Wiring

```bash
colcon test --packages-select waybionic_rviz_plugins
colcon test-result --verbose
```

Tests verify:

- `DiagnosticsPanel` is registered in `plugin_description.xml`
- `SurgeonCameraPanel` is not registered
- `package.xml` has no Annin/AR4 dependency
- doctor/camera launch/config files are removed
- archived camera placeholder and AR4 demo helper files are removed
- temporary diagnostics publisher exists and is executable

## Testing Performed (Ubuntu 24.04 / ROS 2 Jazzy / WSL2)

| Check | Result |
|-------|--------|
| `colcon build --packages-select waybionic_rviz_plugins waybionic_description waybionic_bringup --symlink-install` | Pass |
| `colcon test --packages-select waybionic_rviz_plugins` | Pass (14 tests, 0 failures) |
| `temporary_diagnostics_publisher.py` installed executable with symlink-install | Pass |
| `engineer_view.launch.py --show-args` | Pass |
| `temporary_diagnostics_publisher.launch.py --show-args` | Pass |
| Mock diagnostics GUI (`use_mock_diagnostics:=true`) | Needs Ubuntu GUI confirmation |
| Live diagnostics GUI with cycle publisher | Needs Ubuntu GUI confirmation |
| Foundation + diagnostics two-terminal coexistence | Needs Ubuntu GUI confirmation |

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select waybionic_rviz_plugins waybionic_description waybionic_bringup --symlink-install
source install/setup.bash
colcon test --packages-select waybionic_rviz_plugins
colcon test-result --verbose
ros2 launch waybionic_rviz_plugins engineer_view.launch.py --show-args
ros2 launch waybionic_rviz_plugins temporary_diagnostics_publisher.launch.py --show-args
```

## PR Status

Opened as https://github.com/Waybionic/waybionic_ground_station/pull/2 against `main`. Rebased onto merged foundation (`main`). Review screenshots at the top of this file are for reviewer context only.

**Merge recommendation:** Ready for final review after Ubuntu GUI confirmation of live diagnostics with the cycle publisher. Build, symlink-install publisher, and `colcon test` pass on ROS 2 Jazzy.

## Known Limitations

- Meaningful live rows still depend on stable backend publishing once Korede is available.
- Stale handling is currently a simple five-second freshness check.
- The UI is monitoring-only and does not send motor commands.
- Camera/doctor workflow is out of scope for this PR.

## Next Steps

- Merge diagnostics foundation after final GUI review.
- Track real backend work in a follow-up GitHub issue (temperature, current, heartbeat, sensor health).
- Handle camera/doctor low-latency workflow in a separate PR.
- IMU integration continues on `feature/imu-rviz-integration` after this PR merges.
