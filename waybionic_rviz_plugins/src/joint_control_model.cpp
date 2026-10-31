#include "waybionic_rviz_plugins/joint_control_model.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <locale>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace waybionic_rviz_plugins
{
namespace
{

struct KnownJoint
{
  const char * name;
  const char * label;
};

constexpr KnownJoint kKnownJoints[] = {
  {"joint_1", "J1"},
  {"joint_2", "J2"},
  {"joint_3", "J3"},
  {"joint_4", "J4"},
  {"joint_5", "J5"},
  {"tool_grip", "Tool"},
};

std::size_t displayOrder(const std::string & joint_name)
{
  for (std::size_t index = 0; index < std::size(kKnownJoints); ++index) {
    if (joint_name == kKnownJoints[index].name) {
      return index;
    }
  }
  return std::size(kKnownJoints);
}

// The name becomes part of a service name, so it must be a valid ROS name token.
bool isValidJointName(const std::string & name)
{
  if (name.empty()) {
    return false;
  }
  for (std::size_t index = 0; index < name.size(); ++index) {
    const char character = name[index];
    const bool letter =
      (character >= 'a' && character <= 'z') ||
      (character >= 'A' && character <= 'Z') || character == '_';
    const bool digit = character >= '0' && character <= '9';
    if (!letter && !(digit && index > 0)) {
      return false;
    }
  }
  return true;
}

std::string trim(const std::string & text)
{
  const auto first = text.find_first_not_of(" \t");
  if (first == std::string::npos) {
    return {};
  }
  const auto last = text.find_last_not_of(" \t");
  return text.substr(first, last - first + 1);
}

// Problem texts quote the offending message, which ends up in the status line, so
// keep them short and printable whatever the message contains.
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

std::vector<std::string> splitDrives(const std::string & text)
{
  std::vector<std::string> drives;
  std::size_t start = 0;
  while (start <= text.size()) {
    const auto end = text.find(',', start);
    const auto drive = trim(text.substr(start, end == std::string::npos ? end : end - start));
    if (!drive.empty()) {
      drives.push_back(drive);
    }
    if (end == std::string::npos) {
      break;
    }
    start = end + 1;
  }
  return drives;
}

bool parseState(const std::string & text, JointState * state)
{
  if (text == "enabled") {
    *state = JointState::Enabled;
  } else if (text == "disabled") {
    *state = JointState::Disabled;
  } else if (text == "no_drive") {
    *state = JointState::NoDrive;
  } else {
    return false;
  }
  return true;
}

// Plain decimal numbers only. Hex floats, "nan" and "inf" are rejected up front
// because stream parsing of them differs between standard libraries.
bool parseDecimal(const std::string & text, double * value)
{
  if (text.empty() || text.find_first_not_of("0123456789+-.eE") != std::string::npos) {
    return false;
  }
  std::istringstream stream(text);
  stream.imbue(std::locale::classic());
  double parsed = 0.0;
  stream >> parsed;
  if (stream.fail() || !stream.eof() || !std::isfinite(parsed)) {
    return false;
  }
  *value = parsed;
  return true;
}

// Returns the value stored under `key`, or nullptr. `duplicate` is set when the
// key appears more than once, because the intended value is then ambiguous.
const std::string * findValue(
  const diagnostic_msgs::msg::DiagnosticStatus & status, const char * key, bool * duplicate)
{
  const std::string * found = nullptr;
  *duplicate = false;
  for (const auto & item : status.values) {
    if (item.key != key) {
      continue;
    }
    if (found != nullptr) {
      *duplicate = true;
    } else {
      found = &item.value;
    }
  }
  return found;
}

JointRow parseRow(const diagnostic_msgs::msg::DiagnosticStatus & status)
{
  JointRow row;
  row.joint_name = status.name;
  row.label = jointLabel(status.name);
  row.message = status.message;

  // Only the first problem is kept; it is enough to find the fault.
  const auto fail = [&row](const std::string & problem) {
      if (row.problem.empty()) {
        row.problem = problem;
      }
    };

  if (status.name.empty()) {
    fail("status without a joint name");
  } else if (!isValidJointName(status.name)) {
    fail("invalid joint name '" + shorten(status.name) + "'");
  }

  bool duplicate = false;
  const auto * state_text = findValue(status, "state", &duplicate);
  if (duplicate) {
    fail("duplicate 'state' value");
  } else if (state_text == nullptr) {
    fail("missing 'state' value");
  } else if (!parseState(*state_text, &row.state)) {
    fail("unknown state '" + shorten(*state_text) + "'");
  }

  const auto * angle_text = findValue(status, "angle", &duplicate);
  if (duplicate) {
    fail("duplicate 'angle' value");
  } else if (angle_text == nullptr) {
    fail("missing 'angle' value");
  } else if (!parseDecimal(*angle_text, &row.angle_rad)) {
    fail("angle '" + shorten(*angle_text) + "' is not a number");
  }

  // The drive names are informational, so a status without them is still usable.
  const auto * drives_text = findValue(status, "drives", &duplicate);
  if (drives_text != nullptr) {
    row.drives = splitDrives(*drives_text);
  }

  row.valid = row.problem.empty();
  row.serviceable = row.valid && row.state != JointState::NoDrive;
  return row;
}

std::string describeProblem(const JointRow & row, const std::size_t position)
{
  const auto subject =
    row.joint_name.empty() ? "status " + std::to_string(position + 1) : shorten(row.joint_name);
  return subject + ": " + row.problem;
}

}  // namespace

JointControlSnapshot parseJointControl(const diagnostic_msgs::msg::DiagnosticArray & array)
{
  JointControlSnapshot snapshot;
  snapshot.rows.reserve(array.status.size());
  for (const auto & status : array.status) {
    snapshot.rows.push_back(parseRow(status));
  }

  std::set<std::string> seen;
  for (auto & row : snapshot.rows) {
    if (!seen.insert(row.joint_name).second) {
      row.valid = false;
      row.serviceable = false;
      if (row.problem.empty()) {
        row.problem = "duplicate joint name";
      }
    }
  }

  if (snapshot.rows.empty()) {
    snapshot.error = "no joint statuses";
  } else {
    std::size_t problems = 0;
    for (std::size_t position = 0; position < snapshot.rows.size(); ++position) {
      const auto & row = snapshot.rows[position];
      if (row.valid) {
        continue;
      }
      if (problems == 0) {
        snapshot.error = describeProblem(row, position);
      }
      ++problems;
    }
    if (problems > 1) {
      snapshot.error += " (and " + std::to_string(problems - 1) + " more)";
    }
  }
  snapshot.valid = snapshot.error.empty();

  std::stable_sort(
    snapshot.rows.begin(), snapshot.rows.end(), [](const JointRow & left, const JointRow & right) {
      return displayOrder(left.joint_name) < displayOrder(right.joint_name);
    });
  return snapshot;
}

std::vector<std::string> knownJointNames()
{
  std::vector<std::string> names;
  for (const auto & joint : kKnownJoints) {
    names.emplace_back(joint.name);
  }
  return names;
}

std::string jointLabel(const std::string & joint_name)
{
  for (const auto & joint : kKnownJoints) {
    if (joint_name == joint.name) {
      return joint.label;
    }
  }
  return joint_name;
}

std::string setEnabledServiceName(const std::string & joint_name)
{
  return "/arm/" + joint_name + "/set_enabled";
}

const char * stateWord(const JointState state)
{
  switch (state) {
    case JointState::Enabled:
      return "enabled";
    case JointState::Disabled:
      return "disabled";
    case JointState::NoDrive:
      return "no drive";
  }
  return "unknown";
}

bool driveNodeQuiet(
  const std::optional<std::chrono::steady_clock::time_point> & last_message,
  const std::chrono::steady_clock::time_point now)
{
  return !last_message.has_value() || now - *last_message >= kDriveNodeQuietAfter;
}

}  // namespace waybionic_rviz_plugins
