"""UDP joint-state packets from the Windows host into the Docker RViz container.

JSON line (UTF-8), one datagram:
  {"names":["joint_1",...],"positions":[0.0,...],"velocities":[0.0,...]}

Default port 47301 (host -> container), same pattern as the Xbox bridge on 47300.
"""

from __future__ import annotations

import json
import socket
from typing import Sequence

DEFAULT_PORT = 47301
JOINT_NAMES = ("joint_1", "joint_2", "joint_3", "joint_4", "joint_5")


def pack_joint_state(
    positions: Sequence[float],
    velocities: Sequence[float] | None = None,
    names: Sequence[str] = JOINT_NAMES,
) -> bytes:
    if velocities is None:
        velocities = [0.0] * len(positions)
    payload = {
        "names": list(names),
        "positions": [float(x) for x in positions],
        "velocities": [float(x) for x in velocities],
    }
    return json.dumps(payload, separators=(",", ":")).encode("utf-8")


def unpack_joint_state(data: bytes) -> tuple[list[str], list[float], list[float]]:
    obj = json.loads(data.decode("utf-8"))
    names = list(obj["names"])
    positions = [float(x) for x in obj["positions"]]
    velocities = [float(x) for x in obj.get("velocities", [0.0] * len(positions))]
    return names, positions, velocities


class JointStateUdpSender:
    def __init__(self, host: str = "127.0.0.1", port: int = DEFAULT_PORT) -> None:
        self._addr = (host, port)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(
        self,
        positions: Sequence[float],
        velocities: Sequence[float] | None = None,
        names: Sequence[str] = JOINT_NAMES,
    ) -> None:
        self._sock.sendto(pack_joint_state(positions, velocities, names), self._addr)

    def close(self) -> None:
        self._sock.close()
