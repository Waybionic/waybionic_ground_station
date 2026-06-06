"""Qt styling constants for the WayBionic monitoring prototype."""

from __future__ import annotations


COLORS = {
    "background": "#071019",
    "panel": "#0d1722",
    "panel_alt": "#101d2a",
    "border": "#284052",
    "text": "#e8f1f8",
    "muted": "#8ea3b1",
    "ok": "#3ddc84",
    "warn": "#ffb020",
    "fault": "#ff4d5e",
    "stale": "#9aa4ad",
    "blue": "#43a6ff",
}


STATUS_COLORS = {
    "OK": COLORS["ok"],
    "WARN": COLORS["warn"],
    "FAULT": COLORS["fault"],
    "STALE": COLORS["stale"],
}


APP_STYLESHEET = f"""
QMainWindow {{
    background: {COLORS["background"]};
}}

QWidget {{
    color: {COLORS["text"]};
    font-family: "Segoe UI", "Inter", Arial, sans-serif;
    font-size: 13px;
}}

QFrame#Panel {{
    background: {COLORS["panel"]};
    border: 1px solid {COLORS["border"]};
    border-radius: 10px;
}}

QLabel#Title {{
    font-size: 24px;
    font-weight: 700;
    letter-spacing: 0.5px;
}}

QLabel#PanelTitle {{
    font-size: 17px;
    font-weight: 700;
}}

QLabel#Muted {{
    color: {COLORS["muted"]};
}}

QLabel#StateNormal {{
    background: rgba(61, 220, 132, 0.16);
    border: 1px solid {COLORS["ok"]};
    border-radius: 8px;
    color: {COLORS["ok"]};
    font-weight: 700;
    padding: 8px 12px;
}}

QLabel#StateFault {{
    background: rgba(255, 77, 94, 0.18);
    border: 1px solid {COLORS["fault"]};
    border-radius: 8px;
    color: {COLORS["fault"]};
    font-weight: 800;
    padding: 8px 12px;
}}

QPushButton {{
    background: {COLORS["panel_alt"]};
    border: 1px solid {COLORS["border"]};
    border-radius: 8px;
    padding: 8px 12px;
    font-weight: 600;
}}

QPushButton:hover {{
    border-color: {COLORS["blue"]};
}}

QPushButton:checked {{
    background: rgba(67, 166, 255, 0.18);
    border-color: {COLORS["blue"]};
    color: {COLORS["text"]};
}}

QTableWidget {{
    background: #09131d;
    border: 1px solid {COLORS["border"]};
    border-radius: 8px;
    gridline-color: #1f3343;
    selection-background-color: rgba(67, 166, 255, 0.25);
}}

QHeaderView::section {{
    background: {COLORS["panel_alt"]};
    color: {COLORS["muted"]};
    border: none;
    border-right: 1px solid {COLORS["border"]};
    padding: 8px;
    font-weight: 700;
}}

QTableWidget::item {{
    padding: 7px;
    border-bottom: 1px solid #142535;
}}
"""
