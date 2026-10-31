#ifndef WAYBIONIC_RVIZ_PLUGINS__JOINT_CONTROL_MODEL_HPP_
#define WAYBIONIC_RVIZ_PLUGINS__JOINT_CONTROL_MODEL_HPP_

#include <chrono>
#include <optional>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>

namespace waybionic_rviz_plugins
{

/// Latched topic on which the drive node reports the enable state of every joint.
constexpr const char * kJointControlTopic = "/arm/joint_control";

/// The drive node republishes every 0.5 s and on every state change, so this long
/// without a message means the node is gone.
constexpr std::chrono::milliseconds kDriveNodeQuietAfter{3000};

enum class JointState
{
  Enabled,
  Disabled,
  NoDrive
};

struct JointRow
{
  std::string joint_name;
  std::string label;
  JointState state{JointState::NoDrive};
  std::vector<std::string> drives;
  double angle_rad{0.0};
  std::string message;
  /// True when the drive node offers a set_enabled service for this joint.
  bool serviceable{false};
  /// False when the status broke the contract; `problem` then says how.
  bool valid{false};
  std::string problem;
};

struct JointControlSnapshot
{
  /// True only when the array is non-empty and every row is valid.
  bool valid{false};
  /// Rows in display order: the known joints first, then any others as received.
  std::vector<JointRow> rows;
  /// Summary of the first problem; empty when the snapshot is valid.
  std::string error;
};

/// Reads one /arm/joint_control message. Never throws: malformed statuses are
/// reported through `valid`, `problem` and `error` so the caller can keep showing
/// the last good snapshot.
JointControlSnapshot parseJointControl(const diagnostic_msgs::msg::DiagnosticArray & array);

/// Names of the joints the drive node normally reports, in display order.
std::vector<std::string> knownJointNames();

/// Short display label: joint_1..joint_5 are J1..J5 and tool_grip is Tool.
/// Other names are returned unchanged.
std::string jointLabel(const std::string & joint_name);

/// Name of the SetBool service that enables or disables a joint.
std::string setEnabledServiceName(const std::string & joint_name);

/// Word shown for a state: "enabled", "disabled" or "no drive".
const char * stateWord(JointState state);

/// True when the drive node should be treated as gone: nothing has been received,
/// or the last message is at least kDriveNodeQuietAfter old.
bool driveNodeQuiet(
  const std::optional<std::chrono::steady_clock::time_point> & last_message,
  std::chrono::steady_clock::time_point now);

}  // namespace waybionic_rviz_plugins

#endif  // WAYBIONIC_RVIZ_PLUGINS__JOINT_CONTROL_MODEL_HPP_
