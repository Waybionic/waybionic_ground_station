#ifndef WAYBIONIC_RVIZ_PLUGINS__DIAGNOSTICS_PANEL_HPP_
#define WAYBIONIC_RVIZ_PLUGINS__DIAGNOSTICS_PANEL_HPP_

#include <string>
#include <optional>
#include <vector>

#include <QLabel>
#include <QTableWidget>
#include <QTimer>
#include <QVBoxLayout>

#include <rclcpp/clock.hpp>
#include <rviz_common/panel.hpp>

#include "waybionic_rviz_plugins/diagnostics_contract.hpp"
#include "waybionic_rviz_plugins/mock_diagnostics_source.hpp"

class QButtonGroup;
class QPushButton;

namespace waybionic_rviz_plugins
{

class DiagnosticsPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit DiagnosticsPanel(QWidget * parent = nullptr);

  void onInitialize() override;

private:
  void buildUi();
  void refresh();
  void setDemoMode(DemoMode mode);
  void updateSystemStatus(const std::vector<DiagnosticMessage> & messages, const rclcpp::Time & now);
  void updateTelemetryTable(const std::vector<DiagnosticMessage> & messages, const rclcpp::Time & now);
  void updateAlerts(const std::vector<DiagnosticMessage> & messages);
  void clearAlerts();

  QString statusColor(DiagnosticStatus status) const;
  QString rowBackground(DiagnosticStatus status) const;
  QString ageText(const rclcpp::Time & timestamp, const rclcpp::Time & now) const;
  QString optionalText(const std::optional<std::string> & value) const;
  QString alertText(const DiagnosticMessage & message) const;

  MockDiagnosticsSource diagnostics_source_;
  rclcpp::Clock clock_{RCL_SYSTEM_TIME};

  QTimer * refresh_timer_{nullptr};
  QLabel * state_label_{nullptr};
  QLabel * last_updated_label_{nullptr};
  QLabel * source_label_{nullptr};
  QLabel * ros_connection_label_{nullptr};
  QLabel * heartbeat_label_{nullptr};
  QLabel * ui_mode_label_{nullptr};
  QLabel * safety_label_{nullptr};
  QLabel * alert_icon_label_{nullptr};
  QTableWidget * telemetry_table_{nullptr};
  QVBoxLayout * alerts_layout_{nullptr};
  QPushButton * normal_button_{nullptr};
  QPushButton * fault_button_{nullptr};
  QButtonGroup * demo_button_group_{nullptr};
};

}  // namespace waybionic_rviz_plugins

#endif  // WAYBIONIC_RVIZ_PLUGINS__DIAGNOSTICS_PANEL_HPP_
