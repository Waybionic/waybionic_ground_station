"""Main PySide6 dashboard window for the monitoring prototype."""

from __future__ import annotations

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QColor
from PySide6.QtWidgets import (
    QAbstractItemView,
    QButtonGroup,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QMainWindow,
    QPushButton,
    QSizePolicy,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from src.diagnostics_contract import DiagnosticMessage, format_age, seconds_since
from src.mock_diagnostics import MockDiagnosticsSource
from src.styles import APP_STYLESHEET, COLORS, STATUS_COLORS


class GroundStationMainWindow(QMainWindow):
    """Standalone monitoring-first ground station dashboard."""

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("WayBionic Ground Station")
        self.resize(1360, 820)

        self.diagnostics_source = MockDiagnosticsSource()
        self.current_messages: list[DiagnosticMessage] = []

        self.setStyleSheet(APP_STYLESHEET)
        self.setCentralWidget(self._build_central_widget())

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(1000)
        self.refresh()

    def _build_central_widget(self) -> QWidget:
        root = QWidget()
        root_layout = QVBoxLayout(root)
        root_layout.setContentsMargins(18, 14, 18, 18)
        root_layout.setSpacing(14)

        root_layout.addLayout(self._build_top_bar())

        body = QHBoxLayout()
        body.setSpacing(14)
        body.addLayout(self._build_left_column(), 5)
        body.addLayout(self._build_right_column(), 6)
        root_layout.addLayout(body, 1)

        return root

    def _build_top_bar(self) -> QHBoxLayout:
        top_bar = QHBoxLayout()
        top_bar.setSpacing(12)

        state_controls = QHBoxLayout()
        self.normal_button = QPushButton("Normal Demo")
        self.normal_button.setCheckable(True)
        self.normal_button.setChecked(True)
        self.fault_button = QPushButton("Fault Demo")
        self.fault_button.setCheckable(True)

        self.demo_buttons = QButtonGroup(self)
        self.demo_buttons.setExclusive(True)
        self.demo_buttons.addButton(self.normal_button)
        self.demo_buttons.addButton(self.fault_button)
        self.normal_button.clicked.connect(lambda: self._set_demo_mode("normal"))
        self.fault_button.clicked.connect(lambda: self._set_demo_mode("fault"))

        self.state_label = QLabel("Current State: NORMAL")
        self.state_label.setObjectName("StateNormal")
        state_controls.addWidget(self.normal_button)
        state_controls.addWidget(self.fault_button)
        state_controls.addWidget(self.state_label)

        title = QLabel("WayBionic Ground Station")
        title.setObjectName("Title")
        title.setAlignment(Qt.AlignCenter)

        self.last_updated_label = QLabel("Last updated: --")
        self.last_updated_label.setObjectName("Muted")
        self.last_updated_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

        top_bar.addLayout(state_controls, 2)
        top_bar.addWidget(title, 3)
        top_bar.addWidget(self.last_updated_label, 2)
        return top_bar

    def _build_left_column(self) -> QVBoxLayout:
        left = QVBoxLayout()
        left.setSpacing(14)
        left.addWidget(self._build_system_status_panel(), 2)
        left.addWidget(self._build_telemetry_panel(), 5)
        left.addWidget(self._build_alerts_panel(), 3)
        return left

    def _build_right_column(self) -> QVBoxLayout:
        right = QVBoxLayout()
        right.setSpacing(14)
        right.addWidget(
            self._build_placeholder_panel(
                "Surgeon Camera View",
                "Reserved - no video streaming in this sprint",
            ),
            1,
        )
        right.addWidget(
            self._build_placeholder_panel(
                "Robot / Arm Visualization",
                "Reserved - monitoring only",
            ),
            1,
        )
        return right

    def _panel(self) -> QFrame:
        panel = QFrame()
        panel.setObjectName("Panel")
        panel.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        return panel

    def _build_system_status_panel(self) -> QFrame:
        panel = self._panel()
        layout = QGridLayout(panel)
        layout.setContentsMargins(18, 16, 18, 16)
        layout.setHorizontalSpacing(18)
        layout.setVerticalSpacing(10)

        title = QLabel("System Status")
        title.setObjectName("PanelTitle")
        layout.addWidget(title, 0, 0, 1, 2)

        self.status_fields: dict[str, QLabel] = {}
        rows = [
            ("Diagnostic Source", "Mock"),
            ("ROS 2 Connection", "Not connected / Future integration"),
            ("Backend Heartbeat", "OK"),
            ("UI Mode", "Monitoring only"),
            ("Safety Note", "No motor commands sent from this interface"),
        ]
        for row, (label_text, value_text) in enumerate(rows, start=1):
            label = QLabel(label_text)
            label.setObjectName("Muted")
            value = QLabel(value_text)
            value.setWordWrap(True)
            layout.addWidget(label, row, 0)
            layout.addWidget(value, row, 1)
            self.status_fields[label_text] = value

        layout.setColumnStretch(1, 1)
        return panel

    def _build_telemetry_panel(self) -> QFrame:
        panel = self._panel()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(18, 16, 18, 18)
        layout.setSpacing(12)

        title = QLabel("Telemetry + Live Values")
        title.setObjectName("PanelTitle")
        layout.addWidget(title)

        self.telemetry_table = QTableWidget(0, 6)
        self.telemetry_table.setHorizontalHeaderLabels(
            ["Signal", "Status", "Value", "Unit", "Last Updated", "Message"]
        )
        self.telemetry_table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.telemetry_table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.telemetry_table.setFocusPolicy(Qt.NoFocus)
        self.telemetry_table.verticalHeader().setVisible(False)
        self.telemetry_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.telemetry_table.horizontalHeader().setMinimumSectionSize(100)
        layout.addWidget(self.telemetry_table, 1)
        return panel

    def _build_alerts_panel(self) -> QFrame:
        self.alerts_panel = self._panel()
        layout = QVBoxLayout(self.alerts_panel)
        layout.setContentsMargins(18, 16, 18, 18)
        layout.setSpacing(12)

        title_row = QHBoxLayout()
        title = QLabel("Current Alerts")
        title.setObjectName("PanelTitle")
        self.alert_icon = QLabel("")
        self.alert_icon.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.alert_icon.setStyleSheet(f"font-size: 30px; color: {COLORS['fault']}; font-weight: 800;")
        title_row.addWidget(title, 1)
        title_row.addWidget(self.alert_icon)
        layout.addLayout(title_row)

        self.alerts_container = QVBoxLayout()
        self.alerts_container.setSpacing(8)
        layout.addLayout(self.alerts_container)
        layout.addStretch(1)
        return self.alerts_panel

    def _build_placeholder_panel(self, title_text: str, body_text: str) -> QFrame:
        panel = self._panel()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(24, 22, 24, 22)
        layout.setSpacing(12)

        title = QLabel(title_text)
        title.setObjectName("PanelTitle")
        title.setStyleSheet("font-size: 22px;")

        body = QLabel(body_text)
        body.setWordWrap(True)
        body.setStyleSheet(f"color: {COLORS['blue']}; font-size: 22px; font-weight: 650;")

        scope = QLabel("Prototype scope: monitoring surface only; no control, video, or simulation backend attached.")
        scope.setObjectName("Muted")
        scope.setWordWrap(True)

        layout.addWidget(title)
        layout.addWidget(body)
        layout.addStretch(1)
        layout.addWidget(scope)
        return panel

    def _set_demo_mode(self, mode: str) -> None:
        self.diagnostics_source.set_mode(mode)
        self.refresh()

    def refresh(self) -> None:
        self.current_messages = self.diagnostics_source.get_messages()
        self._update_state()
        self._update_system_status()
        self._update_table()
        self._update_alerts()

    def _update_state(self) -> None:
        has_fault = any(message.status in {"FAULT", "STALE"} for message in self.current_messages)
        state = "FAULT" if has_fault else "NORMAL"
        self.state_label.setText(f"Current State: {state}")
        self.state_label.setObjectName("StateFault" if has_fault else "StateNormal")
        self.state_label.style().unpolish(self.state_label)
        self.state_label.style().polish(self.state_label)

        if self.current_messages:
            latest_age = min(seconds_since(message.timestamp) for message in self.current_messages)
            self.last_updated_label.setText(f"Last updated: {latest_age:.1f}s ago")
        else:
            self.last_updated_label.setText("Last updated: --")

    def _update_system_status(self) -> None:
        has_stale = any(message.status == "STALE" for message in self.current_messages)
        has_fault = any(message.status == "FAULT" for message in self.current_messages)
        heartbeat = "STALE" if has_stale else "OK"
        heartbeat_color = STATUS_COLORS["STALE"] if has_stale else STATUS_COLORS["OK"]

        self.status_fields["Diagnostic Source"].setText(self.diagnostics_source.source_name)
        self.status_fields["Backend Heartbeat"].setText(heartbeat)
        self.status_fields["Backend Heartbeat"].setStyleSheet(f"color: {heartbeat_color}; font-weight: 800;")
        self.status_fields["Safety Note"].setStyleSheet(
            f"color: {COLORS['fault'] if has_fault else COLORS['muted']}; font-weight: 700;"
        )

    def _update_table(self) -> None:
        self.telemetry_table.setRowCount(len(self.current_messages))

        for row, message in enumerate(self.current_messages):
            values = [
                message.signal_name,
                message.status,
                self._format_value(message.value),
                message.unit or "-",
                format_age(message.timestamp),
                message.alert_message or "-",
            ]
            status_color = QColor(STATUS_COLORS[message.status])
            row_background = QColor("#170d12" if message.status == "FAULT" else "#101016" if message.status == "STALE" else "#09131d")

            for column, value in enumerate(values):
                item = QTableWidgetItem(value)
                item.setForeground(status_color if column == 1 else QColor(COLORS["text"]))
                item.setBackground(row_background)
                if column in {1, 2, 3, 4}:
                    item.setTextAlignment(Qt.AlignCenter)
                else:
                    item.setTextAlignment(Qt.AlignLeft | Qt.AlignVCenter)
                if message.status != "OK" and column in {0, 1, 5}:
                    font = item.font()
                    font.setBold(True)
                    item.setFont(font)
                self.telemetry_table.setItem(row, column, item)

    def _update_alerts(self) -> None:
        self._clear_layout(self.alerts_container)
        alerts = [message for message in self.current_messages if message.status != "OK"]

        if not alerts:
            self.alert_icon.setText("")
            no_alerts = QLabel("No active alerts")
            no_alerts.setStyleSheet(f"color: {COLORS['ok']}; font-size: 18px; font-weight: 700;")
            self.alerts_container.addWidget(no_alerts)
            return

        self.alert_icon.setText("!")
        for message in alerts:
            label = QLabel(self._alert_text(message))
            label.setWordWrap(True)
            label.setStyleSheet(
                f"""
                background: rgba(255, 77, 94, 0.16);
                border: 1px solid {STATUS_COLORS[message.status]};
                border-radius: 8px;
                color: {COLORS['text']};
                font-size: 16px;
                font-weight: 800;
                padding: 10px 12px;
                """
            )
            self.alerts_container.addWidget(label)

    def _clear_layout(self, layout: QVBoxLayout) -> None:
        while layout.count():
            item = layout.takeAt(0)
            widget = item.widget()
            if widget is not None:
                widget.deleteLater()

    def _alert_text(self, message: DiagnosticMessage) -> str:
        if message.signal_name == "board.temperature":
            return f"FAULT - Board temperature high: {self._format_value(message.value)}{message.unit or ''}"
        if message.signal_name == "imu.heartbeat":
            return "STALE - IMU heartbeat timeout"
        return f"{message.status} - {message.signal_name}: {message.alert_message or 'Attention required'}"

    def _format_value(self, value: float | str | None) -> str:
        if value is None:
            return "-"
        if isinstance(value, float):
            return f"{value:g}"
        return str(value)
