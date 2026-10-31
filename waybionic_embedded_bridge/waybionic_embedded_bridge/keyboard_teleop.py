"""Hold-to-move console teleop with auto-stop on key release + optional RViz UDP.

Uses GetAsyncKeyState (no console-focus gate — Windows Terminal breaks that).
Hold = move + pulse; release = stop after a short debounce.
With --viz, prefers closed-loop MKS encoder feedback (ENC lines) for the twin;
falls back to open-loop estimation until the first ENC arrives.
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
from .joint_estimator import JointEstimator
from .serial_client import UnoSerialClient
from .viz_udp import DEFAULT_PORT, JOINT_NAMES, JointStateUdpSender

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
    "escape": 0x1B,
    "space": 0x20,
}

MOTION_KEYS = ("up", "down", "left", "right", "w", "a", "s", "d")
EDGE_KEYS = ("e", "q", "p", "z", "escape", "space")

# Opposite keys on the same axis must not both be active (USB flood / crash).
AXIS_PAIRS = (
    ("up", "down"),
    ("left", "right"),
    ("w", "s"),
    ("a", "d"),
)


def _key_down(vk: int) -> bool:
    return bool(ctypes.windll.user32.GetAsyncKeyState(vk) & 0x8000)


def _resolve_held(raw_down: dict[str, bool]) -> set[str]:
    """Keep at most one key per axis pair; prefer the newly pressed if both."""
    active: set[str] = set()
    for a, b in AXIS_PAIRS:
        da, db = raw_down.get(a, False), raw_down.get(b, False)
        if da and not db:
            active.add(a)
        elif db and not da:
            active.add(b)
        # if both: drop both (neutral) to avoid fighting cmds
    return active


def _safe_send(client: UnoSerialClient, cmd: str) -> bool:
    try:
        client.send(cmd)
        return True
    except Exception as exc:
        print(f"! serial send failed ({cmd}): {exc}")
        return False


def _windows_loop(
    client: UnoSerialClient,
    estimator: JointEstimator | None,
    viz: JointStateUdpSender | None,
    enc: EncoderState | None,
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
    last_coast_print = 0.0
    last_coast_deg = {n: 0.0 for n in JOINT_NAMES}
    last_enc_wait_print = 0.0

    print(
        "Keys (ENABLED): arrows=J1/J3  WASD=wrist  Space=STOP  Esc=ESTOP  E=ENABLE  Q=DISABLE"
    )
    print("Hold to move, release to stop. One key per axis (no opposite chords).")
    if viz:
        print("RViz twin: ENC from J1+J3+J4+J5 (joint_2 stays 0).")
        print(
            "Q=DISABLE: motion keys muted (safe to type elsewhere); "
            "hand-move arm -> ENC drives RViz; gravity-hold current on."
        )
        print("E=ENABLE again to drive with keys.")
    print("While ENABLED, don't type in other apps (keys are global).")

    while True:
        try:
            now = time.monotonic()

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
                elif line.startswith("DISABLED") or line in (
                    "ENABLED",
                    "POWEROFF",
                    "STOPPED",
                    "ESTOP",
                ):
                    print(f"< {line}")

            # DISABLE/hand-follow: ignore WASD/arrows/space so typing elsewhere is safe.
            if coasting:
                if held:
                    _safe_send(client, protocol.STOP)
                    held.clear()
                    up_since.clear()
                wanted = set()
            else:
                raw_down = {name: _key_down(VK[name]) for name in MOTION_KEYS}
                wanted = _resolve_held(raw_down)

                # Releases first
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

                # New presses
                for name in wanted:
                    if name not in held:
                        cmd = protocol.KEY_DOWN[name]
                        if _safe_send(client, cmd):
                            held[name] = cmd
                            last_pulse = now
                            print(f"> {cmd}")

            for name in EDGE_KEYS:
                down = _key_down(VK[name])
                if down and not prev_edge[name]:
                    if name == "e":
                        if held:
                            _safe_send(client, protocol.STOP)
                            held.clear()
                            up_since.clear()
                        _safe_send(client, protocol.ENABLE)
                        coasting = False
                        print("> ENABLE (keyboard drive on)")
                    elif name == "q":
                        if held:
                            _safe_send(client, protocol.STOP)
                            held.clear()
                            up_since.clear()
                        # Gravity-hold Ma (hand-movable) + keep ENC streaming.
                        _safe_send(client, "COASTMA 250")
                        _safe_send(client, protocol.DISABLE)
                        _safe_send(client, "ENCON")
                        coasting = True
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
                    elif name == "p":
                        if not coasting:
                            _safe_send(client, protocol.POS)
                            print("> POS")
                    elif name == "z":
                        _safe_send(client, protocol.ZERO)
                        if estimator:
                            estimator.zero()
                        if enc is not None and enc.fresh:
                            enc.capture_zero()
                        print("> ZERO (encoder origin captured)")
                    elif name == "escape":
                        _safe_send(client, protocol.ESTOP)
                        held.clear()
                        up_since.clear()
                        coasting = False
                        print("> ESTOP")
                    elif name == "space":
                        if not coasting:
                            _safe_send(client, protocol.STOP)
                            held.clear()
                            up_since.clear()
                            print("> STOP")
                prev_edge[name] = down

            # Pulse at most one held command per tick to avoid USB CDC flood.
            if (not coasting) and held and (now - last_pulse) >= pulse_s:
                cmd = next(iter(held.values()))
                _safe_send(client, cmd)
                last_pulse = now

            if viz and (now - last_viz) >= viz_period:
                # Prefer ENC always when available (especially hand-follow / Q mode).
                if enc is not None and enc.fresh and enc.positions:
                    positions = [enc.positions.get(n, 0.0) for n in JOINT_NAMES]
                    velocities = [enc.velocities.get(n, 0.0) for n in JOINT_NAMES]
                    viz.send(positions, velocities, JOINT_NAMES)
                elif (not coasting) and estimator is not None:
                    estimator.set_held(held)
                    names, positions, velocities = estimator.step()
                    viz.send(positions, velocities, names)
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
    estimator = JointEstimator() if args.viz else None
    enc = EncoderState() if args.viz else None
    viz = JointStateUdpSender(args.viz_host, args.viz_port) if args.viz else None
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

    # Ctrl+C / console close: always drop the COM handle.
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
        print("Port open. Press E, then hold/release arrows or WASD. Ctrl+C releases the port.")
        if args.viz and estimator and viz:
            print(f"Viz UDP -> {args.viz_host}:{args.viz_port}")
            names, positions, velocities = estimator.step()
            viz.send(positions, velocities, names)
        if not sys.platform.startswith("win"):
            print("Windows-only console teleop.")
            return 1
        _windows_loop(client, estimator, viz, enc)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        _release_port("shutdown")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
