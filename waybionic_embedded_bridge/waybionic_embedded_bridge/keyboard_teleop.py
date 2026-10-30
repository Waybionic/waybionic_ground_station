"""Hold-to-move console teleop with auto-stop on key release + optional RViz UDP.

Uses GetAsyncKeyState (no console-focus gate — Windows Terminal breaks that).
Hold = move + pulse; release = stop after a short debounce.
With --viz, prefers closed-loop MKS encoder feedback (ENC lines) for the twin;
falls back to open-loop estimation until the first ENC arrives.
"""

from __future__ import annotations

import argparse
import ctypes
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


def _key_down(vk: int) -> bool:
    return bool(ctypes.windll.user32.GetAsyncKeyState(vk) & 0x8000)


def _windows_loop(
    client: UnoSerialClient,
    estimator: JointEstimator | None,
    viz: JointStateUdpSender | None,
    enc: EncoderState | None,
    pulse_s: float = 0.4,
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

    print(
        "Keys: arrows=J1/J3  WASD=wrist  Space=STOP  Esc=ESTOP  E=ENABLE  Q=DISABLE"
    )
    print("Hold to move, release to stop. Space=STOP all. Ctrl+C quits.")
    if viz:
        print("RViz twin: closed-loop MKS encoders (26:1 arm / 3:1 wrist).")
        print("Q=DISABLE soft-coast (Ma=250, encoders live) — hand-move updates RViz.")
    print("Tip: don't type in other apps while teleop is running (keys are global).")

    while True:
        now = time.monotonic()

        for line in client.poll_lines():
            parsed = parse_enc_line(line) if enc is not None else None
            if parsed is not None and enc is not None:
                enc.update(*parsed)
                if not using_enc:
                    using_enc = True
                    print("Encoder lock: RViz twin using closed-loop MKS feedback.")

        for name in MOTION_KEYS:
            down = _key_down(VK[name])
            if down:
                up_since.pop(name, None)
                if name not in held:
                    cmd = protocol.KEY_DOWN[name]
                    client.send(cmd)
                    held[name] = cmd
                    last_pulse = now
                    print(f"> {cmd}")
            elif name in held:
                if name not in up_since:
                    up_since[name] = now
                elif (now - up_since[name]) >= release_debounce_s:
                    stop = protocol.KEY_UP[name]
                    client.send(stop)
                    del held[name]
                    up_since.pop(name, None)
                    print(f"> {stop}")

        for name in EDGE_KEYS:
            down = _key_down(VK[name])
            if down and not prev_edge[name]:
                if name == "e":
                    client.enable()
                    print("> ENABLE")
                elif name == "q":
                    client.disable()
                    # Keep ENC stream alive for hand-move closed-loop twin.
                    try:
                        client.send("ENCON")
                    except Exception:
                        pass
                    print("> DISABLE/coast (Ma=250, encoders live for RViz)")
                elif name == "p":
                    client.pos()
                    print("> POS")
                elif name == "z":
                    client.send(protocol.ZERO)
                    if estimator:
                        estimator.zero()
                    if enc is not None and enc.fresh:
                        enc.capture_zero()
                    print("> ZERO (encoder origin captured)")
                elif name == "escape":
                    client.estop()
                    held.clear()
                    up_since.clear()
                    print("> ESTOP")
                elif name == "space":
                    client.stop()
                    held.clear()
                    up_since.clear()
                    print("> STOP")
            prev_edge[name] = down

        if held and (now - last_pulse) >= pulse_s:
            for cmd in held.values():
                client.send(cmd)
            last_pulse = now

        if viz and (now - last_viz) >= viz_period:
            if enc is not None and enc.fresh and enc.positions:
                positions = [enc.positions.get(n, 0.0) for n in JOINT_NAMES]
                velocities = [enc.velocities.get(n, 0.0) for n in JOINT_NAMES]
                viz.send(positions, velocities, JOINT_NAMES)
            elif estimator is not None:
                estimator.set_held(held)
                names, positions, velocities = estimator.step()
                viz.send(positions, velocities, names)
            last_viz = now

        time.sleep(0.01)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM3")
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
    )
    estimator = JointEstimator() if args.viz else None
    enc = EncoderState() if args.viz else None
    viz = JointStateUdpSender(args.viz_host, args.viz_port) if args.viz else None

    try:
        print(f"Opening {args.port} @ {args.baud} ...")
        print(f"Loaded map from: {protocol.__file__}")
        print(
            "Key map: "
            f"Up={protocol.KEY_DOWN['up']} Down={protocol.KEY_DOWN['down']} | "
            f"W/S=pitch ({protocol.KEY_DOWN['w']}/{protocol.KEY_DOWN['s']}) | "
            f"A/D=roll ({protocol.KEY_DOWN['a']}/{protocol.KEY_DOWN['d']})"
        )
        client.open()
        if args.viz:
            try:
                client.send("ENCON")
            except Exception:
                pass
        print("Port open. Press E, then hold/release arrows or WASD.")
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
        if viz:
            viz.close()
        client.close()
        print("Closed (STOP sent).")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
