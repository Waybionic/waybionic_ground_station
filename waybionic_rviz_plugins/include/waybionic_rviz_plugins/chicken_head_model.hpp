#ifndef WAYBIONIC_RVIZ_PLUGINS__CHICKEN_HEAD_MODEL_HPP_
#define WAYBIONIC_RVIZ_PLUGINS__CHICKEN_HEAD_MODEL_HPP_

#include <chrono>
#include <optional>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>

namespace waybionic_rviz_plugins
{

/// Latched topic on which the teleop node reports chicken head mode.
constexpr const char * kChickenHeadTopic = "/arm/chicken_head";

/// The teleop node's std_srvs/SetBool service that switches chicken head mode on or off.
constexpr const char * kChickenHeadService = "/arm/chicken_head/set_enabled";

/// The teleop node republishes every 0.5 s and on every change, so this long without a message
/// means it is gone.
constexpr std::chrono::milliseconds kTeleopQuietAfter{3000};

enum class HeadState
{
  /// The mode is off.
  Off,
  /// The tip is moving to the head spot.
  Approach,
  /// The tip is held there while the sticks tilt and roll the tool.
  Hold,
  /// The mode is on but the way to the head spot is blocked.
  Blocked,
  /// The teleop node has no arm to move yet.
  Unavailable
};

struct HeadSnapshot
{
  bool valid{false};
  HeadState state{HeadState::Unavailable};
  /// What is going on, or what is wrong; may be empty while the mode is off.
  std::string message;
  /// Summary of the problem when the message is not valid; empty otherwise.
  std::string error;
};

/// Reads one /arm/chicken_head message. Never throws: a malformed message is reported through
/// `valid` and `error` so the caller can keep showing the last good one.
HeadSnapshot parseChickenHead(const diagnostic_msgs::msg::DiagnosticArray & array);

/// True while the mode is switched on, whether it is moving, holding or blocked.
bool headIsOn(HeadState state);

/// Word shown for a state: "off", "moving to the head spot", "holding", "blocked" or
/// "unavailable".
const char * headStateWord(HeadState state);

/// True when the teleop node should be treated as gone: nothing has been received, or the last
/// message is at least kTeleopQuietAfter old.
bool teleopQuiet(
  const std::optional<std::chrono::steady_clock::time_point> & last_message,
  std::chrono::steady_clock::time_point now);

/// What the panel says while it has no report to show. `seen_report` is whether a valid report
/// has ever arrived, and `teleop_quiet` is teleopQuiet().
std::string headWaitingMessage(bool seen_report, bool teleop_quiet);

}  // namespace waybionic_rviz_plugins

#endif  // WAYBIONIC_RVIZ_PLUGINS__CHICKEN_HEAD_MODEL_HPP_
