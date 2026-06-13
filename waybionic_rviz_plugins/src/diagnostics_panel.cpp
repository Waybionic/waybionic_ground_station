#include "waybionic_rviz_plugins/diagnostics_panel.hpp"

#include <algorithm>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <QAbstractItemView>
#include <QButtonGroup>
#include <QCheckBox>
#include <QColor>
#include <QFrame>
#include <QGridLayout>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QPushButton>
#include <QSignalBlocker>
#include <QStringList>
#include <QStyle>
#include <QTableWidgetItem>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#include "waybionic_rviz_plugins/ros_diagnostics_source.hpp"

namespace waybionic_rviz_plugins
{
namespace
{

constexpr const char * kPanelStyle = R"(
QWidget {
  background-color: #071019;
  color: #e8f1f8;
  font-family: "Segoe UI", "Ubuntu", sans-serif;
  font-size: 12px;
}
QFrame#Card {
  background-color: #0d1722;
  border: 1px solid #284052;
  border-radius: 8px;
}
QLabel#PanelTitle {
  font-size: 15px;
  font-weight: 700;
}
QLabel#Muted {
  color: #8ea3b1;
}
QLabel#StateNormal {
  background-color: rgba(61, 220, 132, 0.16);
  border: 1px solid #3ddc84;
  border-radius: 6px;
  color: #3ddc84;
  font-weight: 800;
  padding: 6px;
}
QLabel#StateFault {
  background-color: rgba(255, 77, 94, 0.18);
  border: 1px solid #ff4d5e;
  border-radius: 6px;
  color: #ff4d5e;
  font-weight: 800;
  padding: 6px;
}
QPushButton {
  background-color: #101d2a;
  border: 1px solid #284052;
  border-radius: 6px;
  color: #e8f1f8;
  font-weight: 700;
  padding: 6px;
}
QPushButton:checked {
  background-color: rgba(67, 166, 255, 0.22);
  border-color: #43a6ff;
}
QTableWidget {
  background-color: #09131d;
  border: 1px solid #284052;
  gridline-color: #1f3343;
}
QHeaderView::section {
  background-color: #101d2a;
  color: #8ea3b1;
  border: none;
  border-right: 1px solid #284052;
  font-weight: 700;
  padding: 5px;
}
)";

QLabel * makeTitle(const QString & text)
{
  auto * label = new QLabel(text);
  label->setObjectName("PanelTitle");
  return label;
}

QLabel * makeMuted(const QString & text)
{
  auto * label = new QLabel(text);
  label->setObjectName("Muted");
  return label;
}

QFrame * makeCard()
{
  auto * card = new QFrame();
  card->setObjectName("Card");
  return card;
}

}  // namespace

DiagnosticsPanel::DiagnosticsPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  buildUi();
  configureSource(use_mock_diagnostics_);
}

void DiagnosticsPanel::onInitialize()
{
  if (auto ros_node_abstraction = getDisplayContext()->getRosNodeAbstraction().lock()) {
    rviz_node_ = ros_node_abstraction->get_raw_node();
  }

  diagnostics_topic_ = readDiagnosticsTopicParameter(diagnostics_topic_);
  configureSource(readUseMockDiagnosticsParameter(use_mock_diagnostics_));
  refresh_timer_ = new QTimer(this);
  connect(refresh_timer_, &QTimer::timeout, this, [this]() { refresh(); });
  refresh_timer_->start(1000);
  refresh();
}

void DiagnosticsPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("Use Mock Diagnostics", use_mock_diagnostics_);
  config.mapSetValue("Diagnostics Topic", QString::fromStdString(diagnostics_topic_));
}

void DiagnosticsPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);

  bool use_mock_diagnostics = use_mock_diagnostics_;
  QString diagnostics_topic;
  if (config.mapGetString("Diagnostics Topic", &diagnostics_topic)) {
    diagnostics_topic_ = diagnostics_topic.toStdString();
  }
  if (config.mapGetBool("Use Mock Diagnostics", &use_mock_diagnostics)) {
    configureSource(use_mock_diagnostics);
  }
}

void DiagnosticsPanel::buildUi()
{
  setStyleSheet(kPanelStyle);
  setMinimumWidth(420);

  auto * root_layout = new QVBoxLayout(this);
  root_layout->setContentsMargins(10, 10, 10, 10);
  root_layout->setSpacing(10);

  auto * header_card = makeCard();
  auto * header_layout = new QVBoxLayout(header_card);
  header_layout->setSpacing(8);

  auto * title = makeTitle("WayBionic Engineering Monitor");
  state_label_ = new QLabel("Current State: NORMAL");
  state_label_->setObjectName("StateNormal");
  last_updated_label_ = makeMuted("Last updated: --");

  use_mock_diagnostics_checkbox_ = new QCheckBox("Use Mock Diagnostics");
  use_mock_diagnostics_checkbox_->setChecked(use_mock_diagnostics_);
  use_mock_diagnostics_checkbox_->setToolTip(
    "Checked: local mock validation states. Unchecked: subscribe to live ROS 2 diagnostics.");
  connect(use_mock_diagnostics_checkbox_, &QCheckBox::toggled, this, [this](const bool checked) {
    setUseMockDiagnostics(checked);
  });

  auto * button_row = new QHBoxLayout();
  normal_button_ = new QPushButton("Mock Normal");
  normal_button_->setCheckable(true);
  normal_button_->setChecked(true);
  fault_button_ = new QPushButton("Mock Fault");
  fault_button_->setCheckable(true);

  mock_state_button_group_ = new QButtonGroup(this);
  mock_state_button_group_->setExclusive(true);
  mock_state_button_group_->addButton(normal_button_);
  mock_state_button_group_->addButton(fault_button_);
  connect(normal_button_, &QPushButton::clicked, this, [this]() {
    setMockDiagnosticsState(MockDiagnosticsState::Normal);
  });
  connect(fault_button_, &QPushButton::clicked, this, [this]() {
    setMockDiagnosticsState(MockDiagnosticsState::Fault);
  });

  button_row->addWidget(normal_button_);
  button_row->addWidget(fault_button_);

  header_layout->addWidget(title);
  header_layout->addWidget(use_mock_diagnostics_checkbox_);
  header_layout->addLayout(button_row);
  header_layout->addWidget(state_label_);
  header_layout->addWidget(last_updated_label_);
  root_layout->addWidget(header_card);

  auto * system_card = makeCard();
  auto * system_layout = new QGridLayout(system_card);
  system_layout->setVerticalSpacing(6);
  system_layout->addWidget(makeTitle("System Status"), 0, 0, 1, 2);

  source_label_ = new QLabel("Mock");
  ros_connection_label_ = new QLabel("Local mock diagnostics");
  heartbeat_label_ = new QLabel("OK");
  ui_mode_label_ = new QLabel("Monitoring only");
  safety_label_ = new QLabel("No motor commands sent from this RViz panel");
  safety_label_->setWordWrap(true);

  const std::vector<std::pair<QString, QLabel *>> rows = {
    {"Diagnostic Source", source_label_},
    {"ROS 2 Connection", ros_connection_label_},
    {"Backend Heartbeat", heartbeat_label_},
    {"UI Mode", ui_mode_label_},
    {"Safety Note", safety_label_},
  };

  int row = 1;
  for (const auto & [name, value] : rows) {
    system_layout->addWidget(makeMuted(name), row, 0);
    system_layout->addWidget(value, row, 1);
    ++row;
  }
  root_layout->addWidget(system_card);

  auto * table_card = makeCard();
  auto * table_layout = new QVBoxLayout(table_card);
  table_layout->addWidget(makeTitle("Telemetry + Live Values"));

  telemetry_table_ = new QTableWidget(0, 6);
  telemetry_table_->setHorizontalHeaderLabels({"Signal", "Status", "Value", "Unit", "Last Updated", "Message"});
  telemetry_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  telemetry_table_->setSelectionBehavior(QAbstractItemView::SelectRows);
  telemetry_table_->verticalHeader()->setVisible(false);
  telemetry_table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
  telemetry_table_->horizontalHeader()->setMinimumSectionSize(80);
  table_layout->addWidget(telemetry_table_);
  root_layout->addWidget(table_card, 1);

  auto * alerts_card = makeCard();
  auto * alerts_root = new QVBoxLayout(alerts_card);
  auto * alerts_title_row = new QHBoxLayout();
  alerts_title_row->addWidget(makeTitle("Current Alerts"), 1);
  alert_icon_label_ = new QLabel("");
  alert_icon_label_->setStyleSheet("color: #ff4d5e; font-size: 28px; font-weight: 900;");
  alerts_title_row->addWidget(alert_icon_label_);
  alerts_root->addLayout(alerts_title_row);

  alerts_layout_ = new QVBoxLayout();
  alerts_layout_->setSpacing(6);
  alerts_root->addLayout(alerts_layout_);
  alerts_root->addStretch(1);
  root_layout->addWidget(alerts_card);
}

void DiagnosticsPanel::configureSource(const bool use_mock_diagnostics)
{
  use_mock_diagnostics_ = use_mock_diagnostics;
  mock_diagnostics_source_ = nullptr;

  if (use_mock_diagnostics_ || !rviz_node_) {
    auto mock_source = std::make_unique<MockDiagnosticsSource>();
    mock_diagnostics_source_ = mock_source.get();
    diagnostics_source_ = std::move(mock_source);
  } else {
    diagnostics_source_ = std::make_unique<RosDiagnosticsSource>(rviz_node_, diagnostics_topic_);
  }

  updateSourceControls();
}

bool DiagnosticsPanel::readUseMockDiagnosticsParameter(const bool default_value)
{
  if (!rviz_node_) {
    return default_value;
  }

  if (!rviz_node_->has_parameter("use_mock_diagnostics")) {
    return rviz_node_->declare_parameter<bool>("use_mock_diagnostics", default_value);
  }

  return rviz_node_->get_parameter("use_mock_diagnostics").as_bool();
}

std::string DiagnosticsPanel::readDiagnosticsTopicParameter(const std::string & default_value)
{
  if (!rviz_node_) {
    return default_value;
  }

  if (!rviz_node_->has_parameter("diagnostics_topic")) {
    return rviz_node_->declare_parameter<std::string>("diagnostics_topic", default_value);
  }

  return rviz_node_->get_parameter("diagnostics_topic").as_string();
}

void DiagnosticsPanel::refresh()
{
  const auto now = clock_.now();
  const auto messages = diagnostics_source_->messages(now);
  updateSystemStatus(messages, now);
  updateTelemetryTable(messages, now);
  updateAlerts(messages);
}

void DiagnosticsPanel::setMockDiagnosticsState(const MockDiagnosticsState mode)
{
  if (mock_diagnostics_source_ == nullptr) {
    return;
  }

  mock_diagnostics_source_->setMode(mode);
  refresh();
}

void DiagnosticsPanel::setUseMockDiagnostics(const bool use_mock_diagnostics)
{
  configureSource(use_mock_diagnostics);
  refresh();
}

void DiagnosticsPanel::updateSystemStatus(
  const std::vector<DiagnosticMessage> & messages,
  const rclcpp::Time & now)
{
  const bool has_alert =
    std::any_of(messages.begin(), messages.end(), [](const auto & message) {
      return isAlertStatus(message.status);
    });
  const bool has_stale =
    std::any_of(messages.begin(), messages.end(), [](const auto & message) {
      return message.status == DiagnosticStatus::Stale;
    });

  const bool waiting_for_live_diagnostics =
    !use_mock_diagnostics_ && messages.size() == 1 && messages.front().signal_name == "diagnostics.topic";

  if (waiting_for_live_diagnostics) {
    state_label_->setText("Current State: WAITING FOR LIVE DIAGNOSTICS");
  } else {
    state_label_->setText(has_alert ? "Current State: FAULT" : "Current State: NORMAL");
  }
  state_label_->setObjectName(has_alert ? "StateFault" : "StateNormal");
  state_label_->style()->unpolish(state_label_);
  state_label_->style()->polish(state_label_);

  source_label_->setText(QString::fromStdString(diagnostics_source_->sourceName()));
  ros_connection_label_->setText(QString::fromStdString(diagnostics_source_->connectionStatus(now)));
  heartbeat_label_->setText(has_stale ? "STALE" : "OK");
  heartbeat_label_->setStyleSheet(QString("color: %1; font-weight: 800;").arg(has_stale ? "#9aa4ad" : "#3ddc84"));
  safety_label_->setStyleSheet(QString("color: %1; font-weight: 700;").arg(has_alert ? "#ff4d5e" : "#8ea3b1"));

  if (messages.empty()) {
    last_updated_label_->setText("Last updated: --");
    return;
  }

  double latest_age = std::numeric_limits<double>::max();
  for (const auto & message : messages) {
    latest_age = std::min(latest_age, (now - message.timestamp).seconds());
  }
  last_updated_label_->setText(QString("Last updated: %1s ago").arg(latest_age, 0, 'f', 1));
}

void DiagnosticsPanel::updateSourceControls()
{
  if (use_mock_diagnostics_checkbox_ != nullptr) {
    const QSignalBlocker blocker(use_mock_diagnostics_checkbox_);
    use_mock_diagnostics_checkbox_->setChecked(use_mock_diagnostics_);
  }

  const bool mock_enabled = mock_diagnostics_source_ != nullptr;
  if (normal_button_ != nullptr) {
    normal_button_->setEnabled(mock_enabled);
  }
  if (fault_button_ != nullptr) {
    fault_button_->setEnabled(mock_enabled);
  }
}

void DiagnosticsPanel::updateTelemetryTable(
  const std::vector<DiagnosticMessage> & messages,
  const rclcpp::Time & now)
{
  telemetry_table_->setRowCount(static_cast<int>(messages.size()));

  for (int row = 0; row < static_cast<int>(messages.size()); ++row) {
    const auto & message = messages.at(row);
    const QStringList values = {
      QString::fromStdString(message.signal_name),
      toString(message.status),
      optionalText(message.value),
      optionalText(message.unit),
      ageText(message.timestamp, now),
      optionalText(message.alert_message),
    };

    for (int column = 0; column < values.size(); ++column) {
      auto * item = new QTableWidgetItem(values.at(column));
      item->setBackground(QColor(rowBackground(message.status)));
      item->setForeground(QColor(column == 1 ? statusColor(message.status) : "#e8f1f8"));
      if (column == 1 || column == 2 || column == 3 || column == 4) {
        item->setTextAlignment(Qt::AlignCenter);
      }
      if (message.status != DiagnosticStatus::Ok && (column == 0 || column == 1 || column == 5)) {
        auto font = item->font();
        font.setBold(true);
        item->setFont(font);
      }
      telemetry_table_->setItem(row, column, item);
    }
  }
}

void DiagnosticsPanel::updateAlerts(const std::vector<DiagnosticMessage> & messages)
{
  clearAlerts();

  bool has_alert = false;
  for (const auto & message : messages) {
    if (!isAlertStatus(message.status)) {
      continue;
    }

    has_alert = true;
    auto * label = new QLabel(alertText(message));
    label->setWordWrap(true);
    label->setStyleSheet(QString(
      "background-color: rgba(255, 77, 94, 0.18);"
      "border: 1px solid %1;"
      "border-radius: 6px;"
      "color: #e8f1f8;"
      "font-weight: 800;"
      "padding: 8px;").arg(statusColor(message.status)));
    alerts_layout_->addWidget(label);
  }

  if (!has_alert) {
    alert_icon_label_->setText("");
    auto * label = new QLabel("No active alerts");
    label->setStyleSheet("color: #3ddc84; font-size: 15px; font-weight: 800;");
    alerts_layout_->addWidget(label);
    return;
  }

  alert_icon_label_->setText("!");
}

void DiagnosticsPanel::clearAlerts()
{
  while (alerts_layout_->count() > 0) {
    auto * item = alerts_layout_->takeAt(0);
    if (auto * widget = item->widget()) {
      widget->deleteLater();
    }
    delete item;
  }
}

QString DiagnosticsPanel::statusColor(const DiagnosticStatus status) const
{
  switch (status) {
    case DiagnosticStatus::Ok:
      return "#3ddc84";
    case DiagnosticStatus::Warn:
      return "#ffb020";
    case DiagnosticStatus::Fault:
      return "#ff4d5e";
    case DiagnosticStatus::Stale:
      return "#9aa4ad";
  }
  return "#e8f1f8";
}

QString DiagnosticsPanel::rowBackground(const DiagnosticStatus status) const
{
  switch (status) {
    case DiagnosticStatus::Fault:
      return "#241018";
    case DiagnosticStatus::Warn:
      return "#241d0d";
    case DiagnosticStatus::Stale:
      return "#151922";
    case DiagnosticStatus::Ok:
      return "#09131d";
  }
  return "#09131d";
}

QString DiagnosticsPanel::ageText(const rclcpp::Time & timestamp, const rclcpp::Time & now) const
{
  const double age_seconds = std::max(0.0, (now - timestamp).seconds());
  return QString("%1s ago").arg(age_seconds, 0, 'f', 1);
}

QString DiagnosticsPanel::optionalText(const std::optional<std::string> & value) const
{
  if (!value.has_value() || value->empty()) {
    return "-";
  }
  return QString::fromStdString(*value);
}

QString DiagnosticsPanel::alertText(const DiagnosticMessage & message) const
{
  if (message.signal_name == "board.temperature") {
    return QString("FAULT - Board temperature high: %1 %2")
      .arg(optionalText(message.value), optionalText(message.unit));
  }

  if (message.signal_name == "imu.heartbeat") {
    return "STALE - IMU heartbeat timeout";
  }

  return QString("%1 - %2: %3")
    .arg(toString(message.status))
    .arg(QString::fromStdString(message.signal_name))
    .arg(optionalText(message.alert_message));
}

}  // namespace waybionic_rviz_plugins

PLUGINLIB_EXPORT_CLASS(waybionic_rviz_plugins::DiagnosticsPanel, rviz_common::Panel)
