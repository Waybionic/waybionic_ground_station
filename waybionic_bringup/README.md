# Ground station launch options

After building and sourcing the workspace:

```bash
ros2 launch waybionic_bringup ground_station.launch.py
```

The default starts the robot state publisher, joint-state GUI, and RViz using
`waybionic_unified.rviz` with its internal mock diagnostics panel.

| Argument | Default | Purpose |
| --- | --- | --- |
| `launch_rviz` | `true` | Start RViz. |
| `use_joint_state_publisher_gui` | `true` | Start the separate joint-state GUI. |
| `use_diagnostics` | `true` | Enable the RViz diagnostics panel and permit the demo publisher. |
| `use_mock_diagnostics` | `true` | Select internal mock data rather than live topics in the panel. |
| `start_temporary_diagnostics_publisher` | `false` | Start a demo publisher when diagnostics are enabled. |
| `use_sim_time` | `false` | Use `/clock`; requires an external clock publisher. |
| `diagnostics_topic` | `/diagnostics` | Topic for live monitoring and the demo publisher. |
| `model` | Package placeholder URDF | Readable URDF or xacro model path. |
| `rvizconfig` | Package unified RViz layout | Readable RViz YAML layout; unused when RViz is disabled. |

Boolean arguments accept `true` or `false`. Invalid switches, unreadable model
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
external `/joint_states` publisher is needed to update movable joints. This
launch does not start hardware drivers.

## Diagnostics

Disable monitoring while keeping robot visualization:

```bash
ros2 launch waybionic_bringup ground_station.launch.py use_diagnostics:=false
```

This removes the Waybionic diagnostics panel from a temporary copy of the selected
RViz layout and suppresses the demo publisher, even if its switch is `true`.
The original layout is unchanged; the temporary copy is removed on normal launch
shutdown. Other tools and hardware publishers started separately are unaffected.

Try live-topic monitoring with the demo publisher:

```bash
ros2 launch waybionic_bringup ground_station.launch.py \
  use_mock_diagnostics:=false start_temporary_diagnostics_publisher:=true
```

For real diagnostic streams, leave the demo publisher disabled and start your
hardware separately. Hardware selection is deferred to the integration with
PR #18's `hardware_mode`; this change does not introduce `use_hardware`.
