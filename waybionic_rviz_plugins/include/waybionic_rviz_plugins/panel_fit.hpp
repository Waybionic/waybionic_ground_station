#ifndef WAYBIONIC_RVIZ_PLUGINS__PANEL_FIT_HPP_
#define WAYBIONIC_RVIZ_PLUGINS__PANEL_FIT_HPP_

#include <algorithm>

#include <QLayout>
#include <QWidget>

namespace waybionic_rviz_plugins
{

/// Asks for the height a panel's layout needs at the panel's current width, up to `max_height`.
///
/// A word-wrapped label reports a one-line minimum height, and a dock gives its panel no height
/// for its width, so a long message was squeezed over the row below it. A dock or scroll area
/// that is shorter than the panel scrolls it instead. With `at_most_that`, the panel also
/// refuses more height than it needs, so a small panel leaves the rest of the dock to the panels
/// beside it.
inline void fitPanelHeight(QWidget * panel, const int max_height, const bool at_most_that = false)
{
  // Before the dock has laid the panel out it is far narrower than it will be, so the height it
  // would need there is meaningless, and as a minimum it would stop the window from maximizing.
  if (panel->layout() == nullptr || panel->width() < panel->minimumWidth()) {
    return;
  }
  const int needed = std::min(panel->layout()->totalHeightForWidth(panel->width()), max_height);
  if (needed > 0 && needed != panel->minimumHeight()) {
    panel->setMinimumHeight(needed);
  }
  if (at_most_that && needed > 0 && needed != panel->maximumHeight()) {
    panel->setMaximumHeight(needed);
  }
}

}  // namespace waybionic_rviz_plugins

#endif  // WAYBIONIC_RVIZ_PLUGINS__PANEL_FIT_HPP_
