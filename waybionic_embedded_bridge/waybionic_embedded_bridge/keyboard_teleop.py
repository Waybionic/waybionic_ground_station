"""Hold-to-move console teleop with Xbox pad, sim-align, home, and RViz UDP.

Uses GetAsyncKeyState (no console-focus gate — Windows Terminal breaks that).
Hold = move + pulse; release = stop after a short debounce.
With --viz, prefers closed-loop MKS encoder feedback (ENC lines) for the twin;
falls back to open-loop estimation until the first ENC arrives.

Modes:
  drive   — keys/Xbox move the real arm (+ twin)
  sim     — keys/Xbox move ONLY the RViz twin (align sim to IRL)
  coast   — Q: motors soft-hold, motion keys muted, ENC drives twin
"""

from __future__ import annotations

import argparse
import atexit
import ctypes
import math
import signal
import sys
import time

from . import protocol
from .encoder_map import EncoderState, parse_enc_line
from .joint_estimator import LIMITS, JointEstimator
from .serial_client import UnoSerialClient
from .viz_udp import DEFAULT_PORT, JOINT_NAMES, JointStateUdpSender
from .xbox_pad import XboxPad, pad_to_motion_keys

VK = {
    "up": 0x26,
    "down": 0x28,
    "left": 0x25,
    "right": 0x27,
    "w": 0x57,
    "a": 0x41,
    "s": 0x53,
    "d": 0x44,
    "e": 0x45,
    "q": 0x51,
    "p": 0x50,
    "z": 0x5A,
    "h": 0x48,
    "m": 0x4D,
    "escape": 0x1B,
    "space": 0x20,
}

MOTION_KEYS = ("up", "down", "left", "right", "w", "a", "s", "d")
EDGE_KEYS = ("e", "q", "p", "z", "h", "m", "escape", "space")

AXIS_PAIRS = (
    ("up", "down"),
    ("left", "right"),
    ("w", "s"),
    ("a", "d"),
)

PITCH_KEYS = ("w", "s")


def _key_down(vk: int) -> bool:
    return bool(ctypes.windll.user32.GetAsyncKeyState(vk) & 0x8000)


def _resolve_held(raw_down: dict[str, bool]) -> set[str]:
    active: set[str] = set()
    for a, b in AXIS_PAIRS:
        da, db = raw_down.get(a, False), raw_down.get(b, False)
        if da and not db:
            active.add(a)
        elif db and not da:
            active.add(b)
    return active


def _safe_send(client: UnoSerialClient | None, cmd: str) -> bool:
    if client is None:
        return False
    try:
        client.send(cmd)
        return True
    except Exception as exc:
        print(f"! serial send failed ({cmd}): {exc}")
        return False


def _pitch_at_limit(
    key: str,
    estimator: JointEstimator | None,
    enc: EncoderState | None,
    using_enc: bool,
) -> bool:
    if key not in PITCH_KEYS:
        return False
    sign = 1.0 if key == "w" else -1.0
    lo, hi = LIMITS["joint_4"]
    if using_enc and enc is not None and enc.positions:
        pos = enc.positions.get("joint_4", 0.0)
    elif estimator is not None:
        pos = estimator.positions.get("joint_4", 0.0)
    else:
        return False
    return (sign > 0 and pos >= hi - 1e-4) or (sign < 0 and pos <= lo + 1e-4)


def _windows_loop(
    client: UnoSerialClient,
    estimator: JointEstimator | None,
    viz: JointStateUdpSender | None,
    enc: EncoderState | None,
    pad: XboxPad | None,
    pulse_s: float = 0.5,
    release_debounce_s: float = 0.08,
    viz_hz: float = 50.0,
) -> None:
    held: dict[str, str] = {}
    up_since: dict[str, float] = {}
    prev_edge = {name: False for name in EDGE_KEYS}
    last_pulse = 0.0
    last_viz = 0.0
    viz_period = 1.0 / viz_hz
    using_enc = False
    coasting = False
    sim_align = False
    viz_bias = {n: 0.0 for n in JOINT_NAMES}
    last_coast_print = 0.0
    last_coast_deg = {n: 0.0 for n in JOINT_NAMES}
    last_enc_wait_print = 0.0
    last_pitch_block_print = 0.0
    home_saved = False

    print(
        "Keys: arrows=J1/J3  WASD=wrist  Space=STOP  Esc=ESTOP  E=ENABLE  Q=DISABLE"
    )
    print("      M=SIM-ALIGN twin only  H=HOME startup pose  Z=ZERO origin")
    print("Xbox: Lstick=J1/J3  Rstick=pitch/roll  A=ENABLE  B=DISABLE  Y=HOME  X=ZERO")
    print("      Start=SIM-ALIGN toggle  Back=ESTOP  RB=SETHOME")
    print("Hold to move, release to stop. One key per axis (no opposite chords).")
    if viz:
        print("RViz twin: ENC when locked; SIM-ALIGN jogs twin only to match IRL.")
    print("While ENABLED (not SIM-ALIGN), don't type in other apps (keys are global).")

    while True:
        try:
            now = time.monotonic()
            pad_state = pad.poll() if pad is not None else None

            for line in client.poll_lines():
                if line.startswith("ENC_WAIT"):
                    if (now - last_enc_wait_print) >= 2.0:
                        print(f"< {line}")
                        last_enc_wait_print = now
                    continue
                parsed = parse_enc_line(line) if enc is not None else None
                if parsed is not None and enc is not None:
                    enc.update(*parsed)
                    if not using_enc:
                        using_enc = True
                        print("Encoder lock: RViz twin using closed-loop MKS feedback.")
                    if not home_saved and enc.fresh:
                        home_saved = True
                        _safe_send(client, protocol.SETHOME)
                        print("> SETHOME (startup pose captured)")
                    if coasting and enc.positions and (now - last_coast_print) >= 0.25:
                        moved = []
                        for name in ("joint_1", "joint_3", "joint_4", "joint_5"):
                            deg = math.degrees(enc.positions.get(name, 0.0))
                            if abs(deg - last_coast_deg[name]) >= 2.0:
                                moved.append(f"{name}={deg:+.1f}deg")
                                last_coast_deg[name] = deg
                        if moved:
                            print("hand-follow ENC " + " ".join(moved))
                            last_coast_print = now
                elif line.startswith("PITCH_LIMIT"):
                    if (now - last_pitch_block_print) >= 0.5:
                        print(f"< {line}")
                        last_pitch_block_print = now
                elif line.startswith("DISABLED") or line in (
                    "ENABLED",
                    "POWEROFF",
                    "STOPPED",
                    "ESTOP",
                    "HOME",
                    "SETHOME OK",
                    "SETHOME AUTO",
                ):
                    print(f"< {line}")

            if coasting:
                if held:
                    _safe_send(client, protocol.STOP)
                    held.clear()
                    up_since.clear()
                wanted: set[str] = set()
            else:
                raw_down = {name: _key_down(VK[name]) for name in MOTION_KEYS}
                wanted = _resolve_held(raw_down)
                if pad_state is not None and pad_state.connected:
                    wanted |= pad_to_motion_keys(pad_state)
                    wanted = _resolve_held({k: k in wanted for k in MOTION_KEYS})

                blocked = {
                    k for k in wanted if _pitch_at_limit(k, estimator, enc, using_enc)
                }
                if blocked and (now - last_pitch_block_print) >= 0.5:
                    print(f"! pitch limit — not sending {sorted(blocked)}")
                    last_pitch_block_print = now
                wanted -= blocked

                if sim_align:
                    if held:
                        _safe_send(client, protocol.STOP)
                        held.clear()
                        up_since.clear()
                    if estimator is not None:
                        estimator.set_held(wanted)
                else:
                    for name in list(held.keys()):
                        if name not in wanted:
                            if name not in up_since:
                                up_since[name] = now
                            elif (now - up_since[name]) >= release_debounce_s:
                                stop = protocol.KEY_UP[name]
                                if _safe_send(client, stop):
                                    print(f"> {stop}")
                                del held[name]
                                up_since.pop(name, None)
                        else:
                            up_since.pop(name, None)

                    for name in wanted:
                        if name not in held:
                            cmd = protocol.KEY_DOWN[name]
                            if _safe_send(client, cmd):
                                held[name] = cmd
                                last_pulse = now
                                print(f"> {cmd}")
                    if estimator is not None:
                        estimator.set_held(set(held.keys()))

            pad_edge_map = {
                "e": "a",
                "q": "b",
                "z": "x",
                "h": "y",
                "m": "start",
                "escape": "back",
                "space": "lb",
            }

            for name in EDGE_KEYS:
                down = _key_down(VK[name])
                key_edge = down and not prev_edge[name]
                pad_edge = bool(
                    pad_state
                    and pad_state.connected
                    and pad_state.edges.get(pad_edge_map.get(name, ""), False)
                )
                if key_edge or pad_edge:
                    if name == "e":
                        if held:
                            _safe_send(client, protocol.STOP)
                            held.clear()
                            up_since.clear()
                        _safe_send(client, protocol.ENABLE)
                        coasting = False
                        sim_align = False
                        print("> ENABLE (keyboard/Xbox drive on)")
                    elif name == "q":
                        if held:
                            _safe_send(client, protocol.STOP)
                            held.clear()
                            up_since.clear()
                        _safe_send(client, "COASTMA 250")
                        _safe_send(client, protocol.DISABLE)
                        _safe_send(client, "ENCON")
                        coasting = True
                        sim_align = False
                        if enc is not None and enc.positions:
                            last_coast_deg = {
                                n: math.degrees(enc.positions.get(n, 0.0))
                                for n in JOINT_NAMES
                            }
                        print(
                            "> DISABLE hand-follow: keys muted, ENC->RViz, "
                            "Ma=250 soft-coast (E to drive again)"
                        )
                        if enc is None:
                            print("  WARNING: start with --viz for ENC hand-follow")
                        elif not using_enc:
                            print(
                                "  Waiting for ENC (should be <3s). "
                                "If stuck, CAN replies aren't reaching the host."
                            )
                    elif name == "m":
                        if held:
                            _safe_send(client, protocol.STOP)
                            held.clear()
                            up_since.clear()
                        sim_align = not sim_align
                        if sim_align:
                            coasting = False
                            if estimator is not None and enc is not None and enc.positions:
                                for n in JOINT_NAMES:
                                    estimator.positions[n] = (
                                        enc.positions.get(n, 0.0) + viz_bias.get(n, 0.0)
                                    )
                            print(
                                "> SIM-ALIGN ON: keys/Xbox move RViz only "
                                "(match twin to IRL, then M again)"
                            )
                        else:
                            if estimator is not None:
                                if using_enc and enc is not None and enc.positions:
                                    for n in JOINT_NAMES:
                                        viz_bias[n] = (
                                            estimator.positions.get(n, 0.0)
                                            - enc.positions.get(n, 0.0)
                                        )
                                else:
                                    for n in JOINT_NAMES:
                                        viz_bias[n] = estimator.positions.get(n, 0.0)
                            print(
                                "> SIM-ALIGN OFF: twin bias saved; "
                                "drive/ENC moves are offset to keep your alignment"
                            )
                    elif name == "h":
                        if held:
                            _safe_send(client, protocol.STOP)
                            held.clear()
                            up_since.clear()
                        if sim_align:
                            if estimator is not None:
                                estimator.zero()
                            for n in JOINT_NAMES:
                                viz_bias[n] = 0.0
                            print("> HOME (sim twin -> zero)")
                        else:
                            _safe_send(client, protocol.HOME)
                            if estimator is not None:
                                estimator.zero()
                            for n in JOINT_NAMES:
                                viz_bias[n] = 0.0
                            if enc is not None and enc.fresh:
                                enc.capture_zero()
                            print("> HOME (motors -> startup pose)")
                    elif name == "p":
                        if not coasting and not sim_align:
                            _safe_send(client, protocol.POS)
                            print("> POS")
                    elif name == "z":
                        _safe_send(client, protocol.ZERO)
                        if estimator:
                            estimator.zero()
                        if enc is not None and enc.fresh:
                            enc.capture_zero()
                        for n in JOINT_NAMES:
                            viz_bias[n] = 0.0
                        print("> ZERO (encoder origin captured, viz bias cleared)")
                    elif name == "escape":
                        _safe_send(client, protocol.ESTOP)
                        held.clear()
                        up_since.clear()
                        coasting = False
                        sim_align = False
                        print("> ESTOP")
                    elif name == "space":
                        if not coasting and not sim_align:
                            _safe_send(client, protocol.STOP)
                            held.clear()
                            up_since.clear()
                            print("> STOP")
                prev_edge[name] = down

            if (
                pad_state
                and pad_state.connected
                and pad_state.edges.get("rb", False)
            ):
                _safe_send(client, protocol.SETHOME)
                home_saved = True
                print("> SETHOME (pad RB)")

            if (
                (not coasting)
                and (not sim_align)
                and held
                and (now - last_pulse) >= pulse_s
            ):
                cmds = [
                    (n, c)
                    for n, c in held.items()
                    if not _pitch_at_limit(n, estimator, enc, using_enc)
                ]
                if cmds:
                    _safe_send(client, cmds[0][1])
                    last_pulse = now
                else:
                    _safe_send(client, protocol.DIFF["stop"])
                    held.clear()

            if viz and (now - last_viz) >= viz_period:
                if sim_align and estimator is not None:
                    names, positions, velocities = estimator.step()
                    viz.send(positions, velocities, names)
                elif enc is not None and enc.fresh and enc.positions:
                    positions = [
                        enc.positions.get(n, 0.0) + viz_bias.get(n, 0.0)
                        for n in JOINT_NAMES
                    ]
                    velocities = [enc.velocities.get(n, 0.0) for n in JOINT_NAMES]
                    viz.send(positions, velocities, JOINT_NAMES)
                elif (not coasting) and estimator is not None:
                    names, positions, velocities = estimator.step()
                    biased = [
                        positions[i] + viz_bias.get(names[i], 0.0)
                        for i in range(len(names))
                    ]
                    viz.send(biased, velocities, names)
                last_viz = now
            time.sleep(0.015)
        except KeyboardInterrupt:
            raise
        except Exception as exc:
            print(f"! teleop loop error (continuing): {exc}")
            time.sleep(0.2)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM7")
    parser.add_argument("--baud", type=int, default=protocol.BAUD)
    parser.add_argument("--watchdog", type=float, default=0.0)
    parser.add_argument("--reset-wait", type=float, default=0.0)
    parser.add_argument("--viz", action="store_true", help="Stream joints to RViz over UDP")
    parser.add_argument("--viz-host", default="127.0.0.1")
    parser.add_argument("--viz-port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--no-xbox", action="store_true", help="Disable XInput polling")
    parser.add_argument("--xbox-index", type=int, default=0)
    parser.add_argument("--no-reset-wait", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args(argv)

    settle = 0.0 if args.no_reset_wait else args.reset_wait
    client = UnoSerialClient(
        port=args.port,
        baud=args.baud,
        watchdog_s=0.0,
        reset_settle_s=settle,
        min_gap_s=0.04,
    )
    estimator = JointEstimator()
    enc = EncoderState() if args.viz else None
    viz = JointStateUdpSender(args.viz_host, args.viz_port) if args.viz else None
    pad = None if args.no_xbox else XboxPad(index=args.xbox_index)
    closed = False

    def _release_port(reason: str = "") -> None:
        nonlocal closed
        if closed:
            return
        closed = True
        if viz is not None:
            try:
                viz.close()
            except Exception:
                pass
        try:
            client.close()
        except Exception:
            pass
        msg = "Closed (port released"
        if reason:
            msg += f", {reason}"
        print(msg + ").")

    atexit.register(_release_port, "atexit")

    def _on_signal(signum, _frame) -> None:
        _release_port(f"signal {signum}")
        raise SystemExit(130)

    try:
        signal.signal(signal.SIGINT, _on_signal)
        if hasattr(signal, "SIGBREAK"):
            signal.signal(signal.SIGBREAK, _on_signal)  # type: ignore[attr-defined]
    except Exception:
        pass

    try:
        print(f"Opening {args.port} @ {args.baud} ...")
        print(f"Loaded map from: {protocol.__file__}")
        print(
            "Key map: "
            f"Up={protocol.KEY_DOWN['up']} Down={protocol.KEY_DOWN['down']} | "
            f"Left={protocol.KEY_DOWN['left']} Right={protocol.KEY_DOWN['right']} | "
            f"W={protocol.KEY_DOWN['w']} S={protocol.KEY_DOWN['s']} "
            f"A={protocol.KEY_DOWN['a']} D={protocol.KEY_DOWN['d']}"
        )
        try:
            client.open()
        except Exception as exc:
            print(
                f"! could not open {args.port}: {exc}\n"
                f"  Close Serial Monitor / other teleop, then retry."
            )
            return 2
        if args.viz:
            try:
                client.send("ENCON")
            except Exception:
                pass
        if pad is not None:
            print("Xbox XInput: enabled (A=enable, sticks=move). Use --no-xbox to disable.")
        print("Port open. Press E, then hold/release arrows or WASD. Ctrl+C releases the port.")
        if args.viz and estimator and viz:
            print(f"Viz UDP -> {args.viz_host}:{args.viz_port}")
            names, positions, velocities = estimator.step()
            viz.send(positions, velocities, names)
        if not sys.platform.startswith("win"):
            print("Windows-only console teleop.")
            return 1
        _windows_loop(client, estimator, viz, enc, pad)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        _release_port("shutdown")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
