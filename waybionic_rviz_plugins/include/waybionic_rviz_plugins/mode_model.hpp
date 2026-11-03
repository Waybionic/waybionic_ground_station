#ifndef WAYBIONIC_RVIZ_PLUGINS__MODE_MODEL_HPP_
#define WAYBIONIC_RVIZ_PLUGINS__MODE_MODEL_HPP_

#include <chrono>
#include <optional>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>

namespace waybionic_rviz_plugins
{

/// The teleop node republishes every mode's state every 0.5 s and on every change, so this long
/// without a message means it is gone.
constexpr std::chrono::milliseconds kTeleopQuietAfter{3000};

/// How a state is shown beside its checkbox.
enum class Tone
{
  Quiet,
  Active,
  Warning
};

/// One state a mode reports on its topic.
struct ModeState
{
  /// As published, for example "hold".
  std::string name;
  /// Shown beside the checkbox.
  std::string word;
  /// The mode counts as switched on, so its checkbox is ticked.
  bool on{false};
  Tone tone{Tone::Quiet};
};

/// A mode the teleop node switches on and off for the operator: where it reports and what it says.
struct ModeSpec
{
  /// Name of the row on the topic, for example "chicken_head".
  std::string row;
  /// Text of the checkbox.
  std::string label;
  /// What the mode does; shown while the node has nothing else to say about it.
  std::string hint;
  /// Latched topic with one row, and the std_srvs/SetBool service that switches the mode.
  std::string topic;
  std::string service;
  std::vector<ModeState> states;
};

struct ModeSnapshot
{
  bool valid{false};
  /// The state reported; meaningful only when `valid`.
  ModeState state;
  /// What is going on, or what is wrong; may be empty while the mode is off.
  std::string message;
  /// The row's level was a warning: the mode cannot be used, or stopped by itself.
  bool warning{false};
  /// Summary of the problem when the report is not valid; empty otherwise.
  std::string error;
};

/// Chicken head mode: the tool tip is held at a spot while the sticks tilt and roll the tool.
const ModeSpec & chickenHeadSpec();

/// The modes in the order the panel lists them.
const std::vector<const ModeSpec *> & modeSpecs();

/// Reads one mode's message. Never throws: a malformed message is reported through `valid` and
/// `error` so the caller can keep showing the last good one.
ModeSnapshot parseMode(const diagnostic_msgs::msg::DiagnosticArray & array, const ModeSpec & spec);

/// True when the teleop node should be treated as gone: nothing has been received, or the last
/// message is at least kTeleopQuietAfter old.
bool teleopQuiet(
  const std::optional<std::chrono::steady_clock::time_point> & last_message,
  std::chrono::steady_clock::time_point now);

/// What the panel says while a mode has no report to show. `seen_report` is whether a valid
/// report has ever arrived, and `teleop_quiet` is teleopQuiet().
std::string modeWaitingMessage(bool seen_report, bool teleop_quiet);

}  // namespace waybionic_rviz_plugins

#endif  // WAYBIONIC_RVIZ_PLUGINS__MODE_MODEL_HPP_
