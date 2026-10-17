# Ground station launch options

After building and sourcing the workspace:

```bash
ros2 launch waybionic_bringup ground_station.launch.py
```

The default starts the new five-joint arm, joint-state GUI, camera follower, and
RViz using `waybionic_unified.rviz` with its internal mock diagnostics panel.

| Argument | Default | Purpose |
| --- | --- | --- |
| `launch_rviz` | `true` | Start RViz. |
| `use_joint_state_publisher_gui` | `true` | Start the separate joint-state GUI. |
| `use_diagnostics` | `true` | Enable the RViz diagnostics panel and permit the demo publisher. |
| `use_mock_diagnostics` | `true` | Select internal mock data rather than live topics in the panel; demo mode, teleop and the camera always use live topics. |
| `start_temporary_diagnostics_publisher` | `false` | Start a demo publisher when diagnostics are enabled. |
| `demo_mode` | `false` | Sweep the five joints in sequence and report pass/fail diagnostics. |
| `demo_speed` | `30.0` | Joint-demo speed in degrees per second. |
| `teleop` | `false` | Drive the arm with a controller through simulated or real CAN drives. |
| `drive_interface` | `sim` | Drive bus: `sim`, or a python-can interface such as `socketcan` or `slcan` for the real MKS drives. |
| `drive_channel` | empty | CAN channel for a real drive bus, such as `can0` or `/dev/ttyACM0`. |
| `joy_source` | `device` | Controller input: local device, UDP host bridge, or none. |
| `autoplay` | `false` | Play a scripted demo whenever the controller is left idle. |
| `joy_udp_bind` | `127.0.0.1` | UDP receiver bind address; use `0.0.0.0` inside Docker. |
| `joy_udp_port` | `47300` | UDP controller bridge port. |
| `follow_camera` | `true` | Keep RViz focused near the tool; false keeps a fixed view. |
| `use_sim_time` | `false` | Use `/clock`; requires an external clock publisher. |
| `camera` | `false` | Show the doctor view sent by `camera_bridge` and report its delay. |
| `camera_bind` | `127.0.0.1` | Camera receiver bind address; use `0.0.0.0` inside Docker. |
| `camera_log` | empty | CSV file for each camera frame's capture time, arrival time and delay. |
| `diagnostics_topic` | `/diagnostics` | Topic for the panel, the demo publisher, demo mode, teleop and the camera. |
| `model` | Package new-arm URDF | Readable URDF or xacro model path; the placeholder can be selected explicitly. |
| `rvizconfig` | Package unified RViz layout | Readable RViz YAML layout; unused when RViz is disabled. |

Boolean arguments accept `true`, `false`, `True`, `False`, `1`, or `0`.
`demo_mode` and `teleop` cannot both be enabled because each publishes joint states.
`autoplay` needs `teleop:=true` with `drive_interface:=sim`.
Invalid switches, unreadable model
files, malformed XML, and invalid active diagnostics topics fail before nodes start.
When RViz is enabled, its configuration must be readable YAML with a mapping at
the root and a list of panel mappings. These checks do not guarantee that all
RViz plugins or model resources will load successfully at runtime.
Startup logs report selected options and file paths; node output is shown on screen.

## Headless operation

Disable **both** windows for a server/container without a display:

```bash
ros2 launch waybionic_bringup ground_station.launch.py \
  launch_rviz:=false use_joint_state_publisher_gui:=false
```

The robot state publisher still runs. With the joint-state GUI disabled, an
external `/joint_states` publisher is needed to update movable joints unless demo mode
or teleop is running. Real drives are used only with `teleop:=true` and a CAN
`drive_interface`; see
[Real MKS Drives over CAN](../BuildInstructions.md#real-mks-drives-over-can).

## Diagnostics

Disable monitoring while keeping robot visualization:

```bash
ros2 launch waybionic_bringup ground_station.launch.py use_diagnostics:=false
```

This removes the Waybionic diagnostics panel from a temporary copy of the selected
RViz layout and suppresses the demo publisher, even if its switch is `true`.
The original layout is unchanged; the temporary copy is removed on normal launch
shutdown. Demo mode, teleop, the drives and the camera still publish on
`diagnostics_topic`, as do tools and hardware publishers started separately.

Try live-topic monitoring with the demo publisher:

```bash
ros2 launch waybionic_bringup ground_station.launch.py \
  use_mock_diagnostics:=false start_temporary_diagnostics_publisher:=true
```

For real diagnostic streams, leave the demo publisher disabled. The real MKS drives
report on the same topic when `drive_interface` selects them.
