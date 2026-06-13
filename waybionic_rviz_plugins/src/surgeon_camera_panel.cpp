#include "waybionic_rviz_plugins/surgeon_camera_panel.hpp"

#include <QFrame>
#include <QGridLayout>
#include <QLabel>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>

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
QLabel#StatusWaiting {
  background-color: rgba(255, 176, 32, 0.16);
  border: 1px solid #ffb020;
  border-radius: 6px;
  color: #ffb020;
  font-weight: 800;
  padding: 6px;
}
QLabel#Muted {
  color: #8ea3b1;
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

SurgeonCameraPanel::SurgeonCameraPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  buildUi();
  refreshLabels();
}

void SurgeonCameraPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("Primary Camera Topic", primary_topic_);
  config.mapSetValue("Secondary Camera Topic", secondary_topic_);
}

void SurgeonCameraPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);

  QString primary_topic;
  if (config.mapGetString("Primary Camera Topic", &primary_topic)) {
    primary_topic_ = primary_topic;
  }

  QString secondary_topic;
  if (config.mapGetString("Secondary Camera Topic", &secondary_topic)) {
    secondary_topic_ = secondary_topic;
  }

  refreshLabels();
}

void SurgeonCameraPanel::buildUi()
{
  setStyleSheet(kPanelStyle);
  setMinimumWidth(360);

  auto * root_layout = new QVBoxLayout(this);
  root_layout->setContentsMargins(10, 10, 10, 10);
  root_layout->setSpacing(10);

  auto * header_card = makeCard();
  auto * header_layout = new QVBoxLayout(header_card);
  header_layout->setSpacing(8);

  header_layout->addWidget(makeTitle("WayBionic Surgeon Camera Placeholder"));

  auto * status_label = new QLabel("Waiting for camera feed");
  status_label->setObjectName("StatusWaiting");
  header_layout->addWidget(status_label);

  auto * scope_label = makeMuted(
    "Placeholder layout only. Camera hardware, drivers, and final ROS topics are not selected yet.");
  scope_label->setWordWrap(true);
  header_layout->addWidget(scope_label);

  root_layout->addWidget(header_card);

  auto * topics_card = makeCard();
  auto * topics_layout = new QGridLayout(topics_card);
  topics_layout->setVerticalSpacing(6);
  topics_layout->addWidget(makeTitle("Configured Placeholder Topics"), 0, 0, 1, 2);

  primary_topic_label_ = new QLabel();
  secondary_topic_label_ = new QLabel();

  topics_layout->addWidget(makeMuted("Primary Camera"), 1, 0);
  topics_layout->addWidget(primary_topic_label_, 1, 1);
  topics_layout->addWidget(makeMuted("Secondary Camera"), 2, 0);
  topics_layout->addWidget(secondary_topic_label_, 2, 1);
  root_layout->addWidget(topics_card);

  auto * safety_card = makeCard();
  auto * safety_layout = new QVBoxLayout(safety_card);
  safety_layout->addWidget(makeTitle("Scope"));

  auto * safety_label = makeMuted(
    "Monitoring-only panel. It does not subscribe to images directly, start camera nodes, or send motor commands.");
  safety_label->setWordWrap(true);
  safety_layout->addWidget(safety_label);
  root_layout->addWidget(safety_card);

  root_layout->addStretch(1);
}

void SurgeonCameraPanel::refreshLabels()
{
  if (primary_topic_label_ != nullptr) {
    primary_topic_label_->setText(primary_topic_);
  }
  if (secondary_topic_label_ != nullptr) {
    secondary_topic_label_->setText(secondary_topic_);
  }
}

}  // namespace waybionic_rviz_plugins

PLUGINLIB_EXPORT_CLASS(waybionic_rviz_plugins::SurgeonCameraPanel, rviz_common::Panel)
