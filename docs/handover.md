# Handover: what works, how it was shown, and what to check next

Written on 14 November 2026 when the software leads changed. Gianna and Karis lead; Yassin
and Mujtaba advise. It records what has been shown to work on the arm and what has not, so
nobody has to rediscover it. PR numbers below are a snapshot; the pull requests say what
happened to them.

## What has been shown to work

| Part | Shown how | Not shown |
| --- | --- | --- |
| Carrier link: laptop USB, UNO R4 WiFi, MCP2515 at 500 kbit/s, five MKS drives, 120 Hz | On the arm, from a Mac. Through the Windows relay and Docker: a simulated carrier at 120 Hz over five drives, about 120 replies/s per drive | A long soak of the Mac read path on the real carrier |
| Zero, gear ratios and directions | On the arm: zero, Start, then moving, with all five drives. The tests follow the directions that were checked | The tool (it has no drive) |
| Arm setup panel (the start-up guide) | On the arm from a Mac, and in a real RViz window for the panel's behaviour | |
| Mac app (`WayBionic Ground Station.app`) | Its decisions are tested without a screen (`test_macos_app.py`) and it was tried by hand on a Mac. The station it starts, `./scripts/macos.sh arm`, is what ran the arm | Intel Macs; an install from the release zip on a clean Mac |
| Windows: `scripts\windows-arm.cmd` | On one Windows laptop with the arm, at the version in #57. The first run said "carrier bridge not ready" until the board was replugged, hence the COM port re-detection | The clone-board and log-file changes (#59); the Windows release bundle, which CI unpacks and self-tests but nobody has run on the arm |
| Chicken head and demo modes (#58) | The simulated drives and a real RViz window with real mouse clicks (28 checks) | **The real arm** |
| CAD joint limits (#44) | Geometry against the CAD meshes at 308,026 poses, and tests | **The powered arm** |
| E-stop status and interlock (draft #60) | The simulated source and tests | The real E-stop: the carrier cannot sense it yet |

## Known problems

- **The Joint control panel did not work properly on the bench.** The drive node and teleop
  sides are tested; the panel needs a pass on the fully wired arm. The Arm setup panel is
  separate and fine.
- **The carrier's CAN error counter climbed during bench runs** (from 14 to about 1400 in
  one). The cause is not established. The counters survive a serial reopen, so judge a run by
  the change from a baseline taken at its start, not by the total (CAN errors are 16 bits;
  failed writes and refusals are 8).
- **No gravity compensation and no hand-guiding.** The MKS drives (firmware V1.0.9) take only
  position and speed commands and report no current or torque, so a torque feed-forward is
  not possible with them. See the operator guide, "Powered hand-guiding".
- **Joint 1 and joint 5 are limited to +/-360 degrees in #44 only because nothing models the
  cables.** Check that the cables allow it before the arm uses those limits.
- **`waybionic_can/docs/HARDWARE_BRINGUP_CHECKLIST.md` still has a step that changes the drives
  to 1 Mbit/s.** The arm's MCP2515 carrier tops out at 500 kbit/s (an 8 MHz crystal) and the
  drives stay at their factory 500 kbit/s. Do not follow that step on this arm.
- **The Windows `wslg` service is only a viewer.** It has no teleop and no setup guide. The
  arm needs `scripts\windows-arm.cmd`, which runs the carrier and controller bridges on the
  host, because Docker cannot open a COM port.

## Check before relying on it

1. **Powered limits (#44).** Mechanical confirms each joint's travel, then someone checks each
   limit with the arm powered at the lowest speed with the stop in reach, as the operator
   guide's "Joint limits and collision checks" says. With #44 merged the real arm gets the new
   limits as soon as it is launched; nothing caps it at the old ones.
2. **The motion modes on the arm (#58).** First run at the lowest speed level, with the
   emergency stop within reach. Demo mode moves the arm by itself.
3. **The Windows launcher on a clean laptop**, including the release bundle: run
   `Check WayBionic Setup.cmd`, then start the arm. Try it with the carrier unplugged, with
   two serial devices plugged in, and with a clone board if anyone has one (#59).
4. **A release from a clean Mac**, using the release zip rather than a checkout.
5. **The E-stop.** The software interlock (#60) is no substitute for the physical stop, which
   must be within reach of anyone near the arm. Electrical still has to sense the E-stop
   circuit and the motor supply at the carrier (`TODO(hardware-TBD)` in `carrier_bridge.ino` and
   `BenchTelemetry.h`).

## Hardware facts that cost time to find

- The carrier is an **Arduino UNO R4 WiFi with an MCP2515 module** (CS on D10, INT on D2, 8 MHz
  crystal). Flash `carrier_bridge_mcp2515`, never `carrier_bridge`, which is for the R4's own
  CAN. CAN runs at 500 kbit/s.
- The MCP2515 must receive with RXM=00 and zero acceptance masks. RXM=11 admits partial,
  errored frames and must not be used as an accept-all shortcut.
- The serial read must take a fixed chunk (`READ_CHUNK`), never `in_waiting`: pyserial's
  `socket://` port reports at most one byte waiting, which starved the Windows relay path to
  0.1 to 7 replies a second.
- The MKS drives have no torque or current command or readback. A PID write (`96h`) is
  acknowledged but reads back the factory value and changes nothing.
- The arm profile (`~/.waybionic/arm_profile.yaml` on a Mac, `arm_profile_bench.yaml` on
  Windows) lists the fitted drives and locks the joints that have none: today only the tool.

## Working in the code

- Run what CI runs, from the repository root: `docker compose --progress plain build test`.
- A new RViz panel needs its name in the layout's `QMainWindow State` blob in
  `waybionic_unified.rviz`, once; `test_default_window_geometry` checks it. When a panel is
  removed or renamed, rebuild the blob from one that predates it, or a placeholder with the
  old name stays.
- A new teleop motion mode needs a ROS contract module in `waybionic_teleop` and a `ModeSpec`
  entry in `mode_model.cpp`; `test_motion_modes_panel.py` checks that they agree.
- A node's `main()` that spins must re-raise a `RuntimeError` unless
  `waybionic_teleop.ros_shutdown.is_shutdown_error(error)`; the SIGINT race otherwise fails
  launch tests at random.
- An ament Python package needs at least one pytest file, or CI's `colcon test` fails on
  empty discovery.
- `.gitignore` ignores `*.app`; the Mac app under `packaging/macos` is un-ignored by negation
  rules, so check that new files there are tracked.

## Who and where

- Releases: [RELEASING.md](RELEASING.md). Operating the arm: [operator_guide.md](operator_guide.md).
- Old branches that hold unmerged work are kept on purpose; `git log origin/main..origin/<branch>`
  shows what each has. `feature/estop-status-malik` is the one worth reviving (#60).
- Teammates who ran the Windows launcher from `feature/joint-enable` should switch to `main`
  once #57 is merged; that branch can then be deleted.

## Repository settings changed at the handover (14 November 2026)

- **The container image is public**, so a Windows laptop can pull it without logging in. The
  organisation's Packages policy ("Package creation") still does not allow members to create
  public packages: it was switched on for a minute to make this one package public, then
  switched off again. See [RELEASING.md](RELEASING.md) to redo it.
- **A tag ruleset, "Protect release tags"**, lets only the `maintainers` team create, move or
  delete `v*` tags. "Protect main" is unchanged.
- **Merged branches are deleted automatically** (the repository setting is on), and the 39
  branches that were provably merged were deleted. Branches with unmerged work were kept.
- **PRs #51 and #54 were closed** as superseded by #55 and #57, with comments crediting their
  authors.
- Secret scanning and Dependabot are off on this repository. They were not changed.
