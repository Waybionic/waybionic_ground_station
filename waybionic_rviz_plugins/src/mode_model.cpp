#include "waybionic_rviz_plugins/mode_model.hpp"

#include <algorithm>
#include <cstddef>
#include <string>
#include <vector>

namespace waybionic_rviz_plugins
{
namespace
{

// Problem texts quote the offending message, which ends up in the panel, so keep them short
// and printable whatever the message contains.
std::string shorten(const std::string & text)
{
  constexpr std::size_t kMaxLength = 32;
  std::string result = text.substr(0, kMaxLength);
  for (auto & character : result) {
    const auto byte = static_cast<unsigned char>(character);
    if (byte < 0x20 || byte >= 0x7f) {
      character = '?';
    }
  }
  if (text.size() > kMaxLength) {
    result += "...";
  }
  return result;
}

}  // namespace

// The states, row, topic and service here are the ones waybionic_teleop's chicken_head module
// publishes; a test in waybionic_teleop keeps the two in step.
const ModeSpec & chickenHeadSpec()
{
  static const ModeSpec spec{
    "chicken_head", "Chicken head mode",
    "Moves the tool tip to a preset spot and holds it there while the sticks tilt and roll the "
    "tool.",
    "/arm/chicken_head", "/arm/chicken_head/set_enabled",
    {
      {"off", "off", false, Tone::Quiet},
      {"approach", "moving to the head spot", true, Tone::Active},
      {"hold", "holding", true, Tone::Active},
      {"blocked", "blocked", true, Tone::Warning},
      {"unavailable", "unavailable", false, Tone::Quiet},
    }};
  return spec;
}

const std::vector<const ModeSpec *> & modeSpecs()
{
  static const std::vector<const ModeSpec *> specs{&chickenHeadSpec()};
  return specs;
}

ModeSnapshot parseMode(const diagnostic_msgs::msg::DiagnosticArray & array, const ModeSpec & spec)
{
  ModeSnapshot snapshot;
  const diagnostic_msgs::msg::DiagnosticStatus * row = nullptr;
  for (const auto & status : array.status) {
    if (status.name != spec.row) {
      continue;
    }
    if (row != nullptr) {
      snapshot.error = "duplicate " + spec.row + " status";
      return snapshot;
    }
    row = &status;
  }
  if (row == nullptr) {
    snapshot.error = "no " + spec.row + " status";
    return snapshot;
  }

  const std::string * state_text = nullptr;
  for (const auto & item : row->values) {
    if (item.key != "state") {
      continue;
    }
    if (state_text != nullptr) {
      snapshot.error = "duplicate 'state' value";
      return snapshot;
    }
    state_text = &item.value;
  }
  if (state_text == nullptr) {
    snapshot.error = "missing 'state' value";
    return snapshot;
  }
  const auto known = std::find_if(
    spec.states.begin(), spec.states.end(),
    [state_text](const ModeState & state) {return state.name == *state_text;});
  if (known == spec.states.end()) {
    snapshot.error = "unknown state '" + shorten(*state_text) + "'";
    return snapshot;
  }

  snapshot.state = *known;
  snapshot.message = row->message;
  snapshot.warning = row->level >= diagnostic_msgs::msg::DiagnosticStatus::WARN;
  snapshot.valid = true;
  return snapshot;
}

bool teleopQuiet(
  const std::optional<std::chrono::steady_clock::time_point> & last_message,
  const std::chrono::steady_clock::time_point now)
{
  return !last_message.has_value() || now - *last_message >= kTeleopQuietAfter;
}

std::string modeWaitingMessage(const bool seen_report, const bool teleop_quiet)
{
  if (!teleop_quiet) {
    return "Waiting for a valid report...";
  }
  if (seen_report) {
    return "The teleop node (xbox_teleop) has stopped reporting.";
  }
  return "Waiting for the teleop node (xbox_teleop). It runs when the station is started for "
         "the arm (teleop:=true).";
}

}  // namespace waybionic_rviz_plugins
