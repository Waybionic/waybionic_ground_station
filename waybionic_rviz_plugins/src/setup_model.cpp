#include "waybionic_rviz_plugins/setup_model.hpp"

#include <algorithm>
#include <cstddef>
#include <iterator>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace waybionic_rviz_plugins
{
namespace
{

constexpr const char * kPrefix = "setup.";
constexpr const char * kKnownSteps[] = {"carrier", "drives", "controller", "zero", "ready"};

std::size_t displayOrder(const std::string & name)
{
  for (std::size_t index = 0; index < std::size(kKnownSteps); ++index) {
    if (name == kKnownSteps[index]) {
      return index;
    }
  }
  return std::size(kKnownSteps);
}

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

// Returns the value stored under `key`, or nullptr. `duplicate` is set when the key appears
// more than once, because the intended value is then ambiguous.
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

bool parseState(const std::string & text, StepState * state)
{
  if (text == "ok") {
    *state = StepState::Ok;
  } else if (text == "todo") {
    *state = StepState::Todo;
  } else if (text == "blocked") {
    *state = StepState::Blocked;
  } else if (text == "working") {
    *state = StepState::Working;
  } else {
    return false;
  }
  return true;
}

SetupStep parseStep(const diagnostic_msgs::msg::DiagnosticStatus & status)
{
  SetupStep step;
  step.message = status.message;

  // Only the first problem is kept; it is enough to find the fault.
  const auto fail = [&step](const std::string & problem) {
      if (step.problem.empty()) {
        step.problem = problem;
      }
    };

  const std::string prefix = kPrefix;
  if (status.name.compare(0, prefix.size(), prefix) != 0 || status.name.size() == prefix.size()) {
    fail("not a setup step");
    step.name = status.name;
  } else {
    step.name = status.name.substr(prefix.size());
  }
  step.title = step.name;

  bool duplicate = false;
  const auto * title = findValue(status, "title", &duplicate);
  if (title != nullptr && !duplicate && !title->empty()) {
    step.title = *title;
  }

  const auto * state_text = findValue(status, "state", &duplicate);
  if (duplicate) {
    fail("duplicate 'state' value");
  } else if (state_text == nullptr) {
    fail("missing 'state' value");
  } else if (!parseState(*state_text, &step.state)) {
    fail("unknown state '" + shorten(*state_text) + "'");
  }

  // Only the zero action exists; an unknown one must not become a button that does something else.
  const auto * action = findValue(status, "action", &duplicate);
  if (duplicate) {
    fail("duplicate 'action' value");
  } else if (action != nullptr && !action->empty()) {
    if (*action == "zero") {
      step.action = *action;
    } else {
      fail("unknown action '" + shorten(*action) + "' (only zero)");
    }
  }

  const auto * current = findValue(status, "current", &duplicate);
  step.current = !duplicate && current != nullptr && *current == "true";

  step.valid = step.problem.empty();
  return step;
}

}  // namespace

bool SetupSnapshot::ready() const
{
  return valid && !steps.empty() &&
         std::all_of(steps.begin(), steps.end(), [](const SetupStep & step) {
           return step.state == StepState::Ok;
         });
}

const SetupStep * SetupSnapshot::currentStep() const
{
  const auto found = std::find_if(
    steps.begin(), steps.end(), [](const SetupStep & step) {return step.current;});
  return found == steps.end() ? nullptr : &*found;
}

SetupSnapshot parseSetup(const diagnostic_msgs::msg::DiagnosticArray & array)
{
  SetupSnapshot snapshot;
  snapshot.steps.reserve(array.status.size());
  for (const auto & status : array.status) {
    snapshot.steps.push_back(parseStep(status));
  }

  std::set<std::string> seen;
  for (auto & step : snapshot.steps) {
    if (!seen.insert(step.name).second) {
      step.valid = false;
      if (step.problem.empty()) {
        step.problem = "duplicate step";
      }
    }
  }

  if (snapshot.steps.empty()) {
    snapshot.error = "no setup steps";
  } else {
    std::size_t problems = 0;
    for (const auto & step : snapshot.steps) {
      if (step.valid) {
        continue;
      }
      if (problems == 0) {
        snapshot.error = shorten(step.name) + ": " + step.problem;
      }
      ++problems;
    }
    if (problems > 1) {
      snapshot.error += " (and " + std::to_string(problems - 1) + " more)";
    }
  }
  snapshot.valid = snapshot.error.empty();

  std::stable_sort(
    snapshot.steps.begin(), snapshot.steps.end(), [](const SetupStep & left, const SetupStep & right) {
      return displayOrder(left.name) < displayOrder(right.name);
    });
  return snapshot;
}

const char * stepStateWord(const StepState state)
{
  switch (state) {
    case StepState::Ok:
      return "done";
    case StepState::Todo:
      return "to do";
    case StepState::Blocked:
      return "blocked";
    case StepState::Working:
      return "working";
  }
  return "unknown";
}

bool setupGuideQuiet(
  const std::optional<std::chrono::steady_clock::time_point> & last_message,
  const std::chrono::steady_clock::time_point now)
{
  return !last_message.has_value() || now - *last_message >= kSetupGuideQuietAfter;
}

}  // namespace waybionic_rviz_plugins
