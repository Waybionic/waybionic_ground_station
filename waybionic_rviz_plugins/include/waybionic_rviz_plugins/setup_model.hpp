#ifndef WAYBIONIC_RVIZ_PLUGINS__SETUP_MODEL_HPP_
#define WAYBIONIC_RVIZ_PLUGINS__SETUP_MODEL_HPP_

#include <chrono>
#include <optional>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>

namespace waybionic_rviz_plugins
{

/// Latched topic on which setup_guide publishes the start-up steps.
constexpr const char * kSetupTopic = "/arm/setup";

/// The drive node's zero service (std_srvs/Trigger): the setup panel's Zero button calls it.
constexpr const char * kZeroServiceName = "/sim_arm_drives/zero";

/// setup_guide publishes every 0.5 s, so this long without a message means it is gone.
constexpr std::chrono::milliseconds kSetupGuideQuietAfter{3000};

enum class StepState
{
  Ok,
  Todo,
  Blocked,
  Working
};

struct SetupStep
{
  /// "carrier", "drives", "controller", "zero" or "ready": the row name without "setup.".
  std::string name;
  std::string title;
  StepState state{StepState::Todo};
  /// What the operator is to do, or what is going on.
  std::string message;
  /// "zero" when the panel is to offer the Zero button; empty otherwise.
  std::string action;
  /// True for the first step that is not done.
  bool current{false};
  bool valid{false};
  std::string problem;
};

struct SetupSnapshot
{
  /// True only when the array is non-empty and every step is valid.
  bool valid{false};
  /// Known steps in the order the operator does them, then any others as received.
  std::vector<SetupStep> steps;
  /// Summary of the first problem; empty when the snapshot is valid.
  std::string error;

  /// True when there is at least one step and every one is done.
  bool ready() const;
  /// The step marked current, or nullptr once everything is done.
  const SetupStep * currentStep() const;
};

/// Reads one /arm/setup message. Never throws: malformed statuses are reported through
/// `valid`, `problem` and `error` so the caller can keep showing the last good snapshot.
SetupSnapshot parseSetup(const diagnostic_msgs::msg::DiagnosticArray & array);

/// Word shown for a state: "done", "to do", "blocked" or "working".
const char * stepStateWord(StepState state);

/// True when setup_guide should be treated as gone: nothing has been received, or the last
/// message is at least kSetupGuideQuietAfter old.
bool setupGuideQuiet(
  const std::optional<std::chrono::steady_clock::time_point> & last_message,
  std::chrono::steady_clock::time_point now);

}  // namespace waybionic_rviz_plugins

#endif  // WAYBIONIC_RVIZ_PLUGINS__SETUP_MODEL_HPP_
