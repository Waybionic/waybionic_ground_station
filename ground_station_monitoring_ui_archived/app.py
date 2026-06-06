"""Entry point for the WayBionic ground station monitoring prototype."""

from __future__ import annotations

import sys

from PySide6.QtWidgets import QApplication

from src.main_window import GroundStationMainWindow


def main() -> int:
    app = QApplication(sys.argv)
    window = GroundStationMainWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
