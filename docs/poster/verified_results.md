# Verified numerical results

Task requirement: 3–5 numbers from merged PRs, tests, or reviews, each with value, what was measured, environment, source, and limits. No invented statistics.

**Policy used here:** a result is listed only if a merged PR body or a code-owner review on that PR states the number, or a test still on `main` encodes it. Current local runs are not back-dated as historical PR results. Open PR #33 camera timings are listed only as a gap.

## Poster decision (rechecked 27 Oct 2026)

The PR descriptions for #35, #36, #40, #45, and #53 were read again on this date. The five values below still match those descriptions. The poster prints three of them. The other two stay in this file, with the reason they are not headlines.

Safe to print, with the qualifier on the same line:

1. **81,421 cases, 0 mismatches** — software frame cross-check only. PR #40.
2. **19 passed and 11 passed** — virtual CAN only. PR #45.
3. **30.0 fps, delay 2 ms (min 2, max 5, jitter 0.3) at 1280×720** — test video, not a webcam. Excludes camera and display time. PR #36. `ros2 topic hz` was 29.99 Hz, and the CSV held 397 frames.

Not a headline:

4. **1,305 tests, 0 failures, 11 skipped** — this is what PR #53 reported, and what Actions run `40122601645` reported. The later `main` run `40128067866` failed `test_not_responding_simulated_drive_is_stale_within_one_report_period_and_recovers`. Do not print this as a suite that is green on current `main`.
5. **25 mm ± 1 mm in 1 s at 50%, axis error under 10 µm** — simulation kinematics. PR #35 and `waybionic_teleop/test/test_incision.py`. A footnote, not a claim about tissue or a powered arm.

These cannot be substantiated from merged work: a powered five-drive motion number, a merged webcam timing, a “suite green on HEAD” line, and a Doctor View screenshot from this machine. Doctor View stays a labeled placeholder. A team lead needs to supply a real RViz capture or an approved test-video frame.

---

## Result 1 — MKS frame cross-check

| | |
| --- | --- |
| **Value** | **81,421 cases, 0 mismatches** |
| **What** | Byte-for-byte comparison of C++/Arduino MKS framing (`mks_frame_tool` from `waybionic_can`) against `waybionic_teleop/mks_can.py` (manual V1.0.9 subset: F5h, 31h, F3h, 92h, 98h, F7h, …) |
| **Environment** | Software only. No CAN adapter, carrier, or motor. |
| **Source** | [PR #40](https://github.com/Waybionic/waybionic_ground_station/pull/40) description and Yassin’s 2026-10-17 review. Script: `waybionic_can/tests/cross_check/cross_check_mks.py`. |
| **Limits** | Proves encoder/command bytes match between the two implementations. It does not prove a physical drive accepts the frames. |

---

## Result 2 — Full workspace test suite

| | |
| --- | --- |
| **Value** | **1,305 tests, 0 errors, 0 failures, 11 skipped** |
| **What** | `docker compose --progress plain build test` / `colcon test` of the ROS 2 Jazzy workspace after the README/setup cleanup. |
| **Environment** | Docker Compose `test` stage, ROS 2 Jazzy. The 11 skips are the `waybionic_can` four-node cases that need `vcan0` in that image. |
| **Source** | [PR #53](https://github.com/Waybionic/waybionic_ground_station/pull/53) (“How I tested it”). Same total in GitHub Actions run `40122601645` on [PR #52](https://github.com/Waybionic/waybionic_ground_station/pull/52) (`Summary: 1305 tests, 0 errors, 0 failures, 11 skipped`, 2026-10-24). |
| **Limits** | Automated software tests, including simulated drives and launch checks. Not a hardware trial. The later `main` run on PR #53 (`40128067866`) **failed** one simulated health test (`test_not_responding_simulated_drive_is_stale_within_one_report_period_and_recovers`); treat 1,305/0 as the last published passing total, not as “CI is green on HEAD”. |

Earlier passing totals on the same stack (do not mix with 1,305 on a poster): PR #45 **1,255** tests / 0 failures / 11 skipped; PR #40 review **1,152** tests; PR #39 **1,077** passed.

---

## Result 3 — Virtual CAN (vcan) drive tests

| | |
| --- | --- |
| **Value** | **19 passed** (`test_can_drives.py` over SocketCAN) **and 11 passed** (`test_vcan_four_node`) |
| **What** | Host-to-simulated-drive frames on Linux `vcan0`, plus the four-node gateway/receiver process test. |
| **Environment** | Privileged GitHub Actions job **CAN Drives over vcan** (`can_drives.yml`), not a physical transceiver. |
| **Source** | [PR #45](https://github.com/Waybionic/waybionic_ground_station/pull/45) description. Reconfirmed on `main` Actions run `40117064058` (PR #39 merge, 2026-10-24): `19 passed` and `[  PASSED  ] 11 tests.` [PR #40](https://github.com/Waybionic/waybionic_ground_station/pull/40) review: DelayedReply case **30/30**, whole four-node binary **20/20** in a privileged container. |
| **Limits** | Kernel virtual CAN. No TJA1051T, no MKS drive, no bus-load or error-frame measurement from metal. |

---

## Result 4 — Doctor-view pipeline delay (test video)

| | |
| --- | --- |
| **Value** | **30.0 fps; delay 2 ms (min 2, max 5, jitter 0.3) at 1280×720.** `ros2 topic hz` **29.99 Hz.** CSV held **397** frames. |
| **What** | Capture-to-arrival delay of the doctor-view TCP path: Windows `camera_bridge` → published port → containerised `camera_receiver` / `latency_monitor`. |
| **Environment** | **Test video file**, not a webcam. Windows Store Python 3.13, OpenCV 5.0, ground station in Docker. The monitor window is 5 s (`latency.py`). |
| **Source** | [PR #36](https://github.com/Waybionic/waybionic_ground_station/pull/36) “How I tested it”. |
| **Limits** | Excludes camera integration time and display time (stated in the PR and in `docs/operator_guide.md`). Not measured on RViz’s image panel. Not a Gemini 2 or procedure camera. Open PR #33’s five-minute webcam run is **not merged** and is internally inconsistent on stopwatch units; do not put those numbers on the poster. |

---

## Result 5 — Incision-group geometry (simulation)

| | |
| --- | --- |
| **Value** | **1.0 s at 50% speed → 25 mm insertion (±1 mm)** along the tool axis; pitch/yaw unchanged; incision point stays within **10 µm** of the axis. Tilt: **−15° ±1.5°** in 1.0 s at 50%, depth and axis error within **10 µm**. |
| **What** | Closed-form incision insert and pivot: whether the port stays on the tool axis and whether commanded insert/tilt distances match the configured 50 mm/s and 30 deg/s peaks. |
| **Environment** | Headless `ArmTeleop` unit tests against URDF kinematics. No motor, no tissue model. |
| **Source** | [PR #35](https://github.com/Waybionic/waybionic_ground_station/pull/35) description; still encoded in `waybionic_teleop/test/test_incision.py` (`depth == pytest.approx(0.025, abs=1e-3)`, `distance < 1e-5`). Review note: simulated only, no powered arm. |
| **Limits** | Kinematic/setpoint accuracy in software. Not tracking error of a physical wrist, and not a statement about incision safety in tissue. |

---

## Strong extras (use only with the same caveats)

These are true, merged, and numerical, but weaker as poster “results” because they are either historical test counts, GPU-host observations, or simulated geometry of a solver—not the five above.

| Value | What / where | Limit |
| --- | --- | --- |
| Cartesian lookahead: **45.373 µm** tip error with the old drive-map vs **1.127 µm** with PR #27, ideal pre-quantization error **0** | Simulated geometry, [PR #27](https://github.com/Waybionic/waybionic_ground_station/pull/27) | Not motor accuracy |
| Teleop at **120 Hz** vs 50 Hz: stalled RViz frames **18/110 → 0/119**; GPU **125–145 fps** vs software **62 fps** | Author PC (RTX 5080), [PR #24](https://github.com/Waybionic/waybionic_ground_station/pull/24) | One workstation, not a bench protocol |
| Six drives at 120 Hz ≈ **37%** of 1 Mbit/s (PR #24) / YAML comment ≈ **30%** | Bus-load estimate from frame bit counts | Estimate, not a measured CAN analyser capture |
| DualSense: buttons/sticks/groups **PASS** over USB and Bluetooth; rumble at a limit on both | Physical controller, [PR #26](https://github.com/Waybionic/waybionic_ground_station/pull/26) | Qualitative pass/fail, Windows only |

---

## Explicit gaps (fewer than three would have been the problem; these are the remaining holes)

1. **No merged numerical result from a powered MKS drive** (encoder lag, current, tracking error, e-stop latency). PRs #31, #37, #40, #45 say so. `waybionic_can/docs/HARDWARE_BRINGUP_CHECKLIST.md` is unchecked.
2. **No merged webcam / RViz-on-screen doctor-view timing.** PR #36 used a file; PR #33’s five-minute webcam log is still open and needs a stopwatch-unit correction before anyone should cite it.
3. **Do not quote “1,305 tests on current HEAD” without the #53 CI failure.** Confirm with a lead whether that health test is a flake before putting a “full suite green” badge on the poster.
4. **Joint limits ±90° / ±180° are provisional**, not measured travel. CAD limits live on open PR #44.
5. **Live session bus load** (for example 15% on the simulated bus during a local autoplay run) is **not** a merged-PR result and is not listed above.
