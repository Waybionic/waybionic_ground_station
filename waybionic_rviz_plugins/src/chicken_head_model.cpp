#include "waybionic_rviz_plugins/chicken_head_model.hpp"

#include <cstddef>
#include <string>

namespace waybionic_rviz_plugins
{
namespace
{

constexpr const char * kRowName = "chicken_head";

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

bool parseState(const std::string & text, HeadState * state)
{
  if (text == "off") {
    *state = HeadState::Off;
  } else if (text == "approach") {
    *state = HeadState::Approach;
  } else if (text == "hold") {
    *state = HeadState::Hold;
  } else if (text == "blocked") {
    *state = HeadState::Blocked;
  } else if (text == "unavailable") {
    *state = HeadState::Unavailable;
  } else {
    return false;
  }
  return true;
}

}  // namespace

HeadSnapshot parseChickenHead(const diagnostic_msgs::msg::DiagnosticArray & array)
{
  HeadSnapshot snapshot;
  const diagnostic_msgs::msg::DiagnosticStatus * row = nullptr;
  for (const auto & status : array.status) {
    if (status.name != kRowName) {
      continue;
    }
    if (row != nullptr) {
      snapshot.error = "duplicate chicken_head status";
      return snapshot;
    }
    row = &status;
  }
  if (row == nullptr) {
    snapshot.error = "no chicken_head status";
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
  if (!parseState(*state_text, &snapshot.state)) {
    snapshot.error = "unknown state '" + shorten(*state_text) + "'";
    return snapshot;
  }

  snapshot.message = row->message;
  snapshot.valid = true;
  return snapshot;
}

bool headIsOn(const HeadState state)
{
  return state == HeadState::Approach || state == HeadState::Hold || state == HeadState::Blocked;
}

const char * headStateWord(const HeadState state)
{
  switch (state) {
    case HeadState::Off:
      return "off";
    case HeadState::Approach:
      return "moving to the head spot";
    case HeadState::Hold:
      return "holding";
    case HeadState::Blocked:
      return "blocked";
    case HeadState::Unavailable:
      return "unavailable";
  }
  return "unknown";
}

bool teleopQuiet(
  const std::optional<std::chrono::steady_clock::time_point> & last_message,
  const std::chrono::steady_clock::time_point now)
{
  return !last_message.has_value() || now - *last_message >= kTeleopQuietAfter;
}

std::string headWaitingMessage(const bool seen_report, const bool teleop_quiet)
{
  if (!teleop_quiet) {
    return "Waiting for a valid chicken head report...";
  }
  if (seen_report) {
    return "The teleop node (xbox_teleop) has stopped reporting.";
  }
  return "Waiting for the teleop node (xbox_teleop). It runs when the station is started for "
         "the arm (teleop:=true).";
}

}  // namespace waybionic_rviz_plugins
