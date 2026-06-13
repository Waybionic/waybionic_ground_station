#ifndef WAYBIONIC_RVIZ_PLUGINS__SURGEON_CAMERA_PANEL_HPP_
#define WAYBIONIC_RVIZ_PLUGINS__SURGEON_CAMERA_PANEL_HPP_

#include <QString>

#include <rviz_common/config.hpp>
#include <rviz_common/panel.hpp>

class QLabel;

namespace waybionic_rviz_plugins
{

class SurgeonCameraPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit SurgeonCameraPanel(QWidget * parent = nullptr);

  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config & config) override;

private:
  void buildUi();
  void refreshLabels();

  QString primary_topic_{"/camera/camera/color/image_raw"};
  QString secondary_topic_{"/surgeon/secondary/image_raw"};

  QLabel * primary_topic_label_{nullptr};
  QLabel * secondary_topic_label_{nullptr};
};

}  // namespace waybionic_rviz_plugins

#endif  // WAYBIONIC_RVIZ_PLUGINS__SURGEON_CAMERA_PANEL_HPP_
