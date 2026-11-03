// Tests for the Qt-free parsing of the mode topics. They build fabricated DiagnosticArray
// messages, so no teleop node or ROS graph is needed.

#include <chrono>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

#include "waybionic_rviz_plugins/mode_model.hpp"

namespace waybionic_rviz_plugins
{
namespace
{

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;
using namespace std::chrono_literals;

KeyValue keyValue(const std::string & key, const std::string & value)
{
  KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}

DiagnosticStatus makeStatus(
  const std::string & row, const std::string & state, const std::string & message = "",
  const unsigned char level = DiagnosticStatus::OK)
{
  DiagnosticStatus status;
  status.name = row;
  status.message = message;
  status.level = level;
  status.values.push_back(keyValue("state", state));
  return status;
}

DiagnosticArray makeArray(const std::vector<DiagnosticStatus> & statuses)
{
  DiagnosticArray array;
  array.status = statuses;
  return array;
}

// The first state of the mode that is on, and its first state that is off.
const ModeState & anOnState(const ModeSpec & spec)
{
  for (const auto & state : spec.states) {
    if (state.on) {
      return state;
    }
  }
  ADD_FAILURE() << spec.row << " has no state that is on";
  return spec.states.front();
}

TEST(ModeModel, ListsChickenHeadThenDemo)
{
  const auto & specs = modeSpecs();
  ASSERT_EQ(specs.size(), 2u);
  EXPECT_EQ(specs[0], &chickenHeadSpec());
  EXPECT_EQ(specs[1], &demoSpec());
}

TEST(ModeModel, EveryModeHasItsOwnRowTopicAndService)
{
  std::vector<std::string> names;
  for (const auto * spec : modeSpecs()) {
    for (const auto & name : {spec->row, spec->topic, spec->service}) {
      EXPECT_FALSE(name.empty());
      for (const auto & other : names) {
        EXPECT_NE(name, other) << "shared by two modes";
      }
    }
    names.push_back(spec->row);
    names.push_back(spec->topic);
    names.push_back(spec->service);
  }
}

TEST(ModeModel, EveryModeCanBeOffAndSaysSoWithoutBeingOn)
{
  for (const auto * spec : modeSpecs()) {
    bool found = false;
    for (const auto & state : spec->states) {
      if (state.name == "off") {
        found = true;
        EXPECT_FALSE(state.on) << spec->row;
      }
    }
    EXPECT_TRUE(found) << spec->row << " has no off state";
  }
}

TEST(ModeModel, NoModeHasTwoStatesOfOneName)
{
  for (const auto * spec : modeSpecs()) {
    for (std::size_t first = 0; first < spec->states.size(); ++first) {
      for (std::size_t second = first + 1; second < spec->states.size(); ++second) {
        EXPECT_NE(spec->states[first].name, spec->states[second].name) << spec->row;
      }
    }
  }
}

TEST(ModeModel, ParsesEveryStateOfEveryMode)
{
  for (const auto * spec : modeSpecs()) {
    for (const auto & expected : spec->states) {
      const auto snapshot = parseMode(makeArray({makeStatus(spec->row, expected.name)}), *spec);
      ASSERT_TRUE(snapshot.valid) << spec->row << " " << expected.name << ": " << snapshot.error;
      EXPECT_EQ(snapshot.state.name, expected.name);
      EXPECT_EQ(snapshot.state.on, expected.on);
      EXPECT_TRUE(snapshot.error.empty());
    }
  }
}

TEST(ModeModel, KeepsTheMessageForTheOperator)
{
  const auto & spec = chickenHeadSpec();
  const auto snapshot = parseMode(
    makeArray({makeStatus(spec.row, "blocked", "The way to the head position is blocked")}), spec);

  ASSERT_TRUE(snapshot.valid);
  EXPECT_EQ(snapshot.message, "The way to the head position is blocked");
}

TEST(ModeModel, AnOffReportNeedsNoMessage)
{
  for (const auto * spec : modeSpecs()) {
    const auto snapshot = parseMode(makeArray({makeStatus(spec->row, "off")}), *spec);

    ASSERT_TRUE(snapshot.valid) << spec->row;
    EXPECT_TRUE(snapshot.message.empty());
    EXPECT_FALSE(snapshot.warning);
  }
}

TEST(ModeModel, ReportsWhetherTheRowIsAWarning)
{
  const auto & spec = chickenHeadSpec();
  const auto calm = parseMode(makeArray({makeStatus(spec.row, "hold", "x")}), spec);
  const auto warn = parseMode(
    makeArray({makeStatus(spec.row, "off", "stopped", DiagnosticStatus::WARN)}), spec);
  const auto error = parseMode(
    makeArray({makeStatus(spec.row, "off", "failed", DiagnosticStatus::ERROR)}), spec);

  EXPECT_FALSE(calm.warning);
  EXPECT_TRUE(warn.warning);
  EXPECT_TRUE(error.warning);
}

TEST(ModeModel, IgnoresRowsWithOtherNames)
{
  const auto & spec = chickenHeadSpec();
  const auto snapshot = parseMode(
    makeArray({makeStatus("teleop", "off", "other"), makeStatus("demo", "running"),
      makeStatus(spec.row, "hold", "Holding the tip")}), spec);

  ASSERT_TRUE(snapshot.valid) << snapshot.error;
  EXPECT_EQ(snapshot.state.name, "hold");
  EXPECT_EQ(snapshot.message, "Holding the tip");
}

TEST(ModeModel, RejectsAnEmptyArray)
{
  const auto & spec = chickenHeadSpec();
  const auto snapshot = parseMode(DiagnosticArray(), spec);

  EXPECT_FALSE(snapshot.valid);
  EXPECT_EQ(snapshot.error, "no chicken_head status");
}

TEST(ModeModel, RejectsAnArrayWithoutTheRow)
{
  const auto & spec = chickenHeadSpec();
  const auto snapshot = parseMode(makeArray({makeStatus("joint_1", "hold")}), spec);

  EXPECT_FALSE(snapshot.valid);
  EXPECT_EQ(snapshot.error, "no chicken_head status");
}

TEST(ModeModel, RejectsDuplicateRows)
{
  const auto & spec = chickenHeadSpec();
  const auto snapshot = parseMode(
    makeArray({makeStatus(spec.row, "off"), makeStatus(spec.row, "hold")}), spec);

  EXPECT_FALSE(snapshot.valid);
  EXPECT_EQ(snapshot.error, "duplicate chicken_head status");
}

TEST(ModeModel, RejectsAMissingState)
{
  const auto & spec = chickenHeadSpec();
  auto status = makeStatus(spec.row, "hold");
  status.values.clear();

  const auto snapshot = parseMode(makeArray({status}), spec);

  EXPECT_FALSE(snapshot.valid);
  EXPECT_EQ(snapshot.error, "missing 'state' value");
}

TEST(ModeModel, RejectsADuplicateState)
{
  const auto & spec = chickenHeadSpec();
  auto status = makeStatus(spec.row, "hold");
  status.values.push_back(keyValue("state", "off"));

  const auto snapshot = parseMode(makeArray({status}), spec);

  EXPECT_FALSE(snapshot.valid);
  EXPECT_EQ(snapshot.error, "duplicate 'state' value");
}

TEST(ModeModel, RejectsAStateTheModeDoesNotHave)
{
  const auto & spec = chickenHeadSpec();
  const auto snapshot = parseMode(makeArray({makeStatus(spec.row, "dancing")}), spec);

  EXPECT_FALSE(snapshot.valid);
  EXPECT_EQ(snapshot.error, "unknown state 'dancing'");
}

TEST(ModeModel, ShortensAndCleansAnUnknownStateInTheError)
{
  const auto & spec = chickenHeadSpec();
  const auto snapshot = parseMode(
    makeArray({makeStatus(spec.row, std::string("bad\nstate") + std::string(60, 'x'))}), spec);

  EXPECT_FALSE(snapshot.valid);
  EXPECT_EQ(snapshot.error.find('\n'), std::string::npos);
  EXPECT_LT(snapshot.error.size(), 80u);
  EXPECT_NE(snapshot.error.find("..."), std::string::npos);
}

TEST(ModeModel, ChickenHeadStatesAreOnWhileItMovesHoldsOrIsBlocked)
{
  const auto & spec = chickenHeadSpec();
  const auto on = [&spec](const std::string & name) {
      return parseMode(makeArray({makeStatus(spec.row, name)}), spec).state.on;
    };

  EXPECT_FALSE(on("off"));
  EXPECT_TRUE(on("approach"));
  EXPECT_TRUE(on("hold"));
  EXPECT_TRUE(on("blocked"));
  EXPECT_FALSE(on("unavailable"));
}

TEST(ModeModel, ChickenHeadStatesAreNamedInAWord)
{
  const auto & spec = chickenHeadSpec();
  const auto word = [&spec](const std::string & name) {
      return parseMode(makeArray({makeStatus(spec.row, name)}), spec).state.word;
    };

  EXPECT_EQ(word("off"), "off");
  EXPECT_EQ(word("approach"), "moving to the head spot");
  EXPECT_EQ(word("hold"), "holding");
  EXPECT_EQ(word("blocked"), "blocked");
  EXPECT_EQ(word("unavailable"), "unavailable");
}

TEST(ModeModel, ABlockedModeIsShownAsAWarningAndAHoldingOneAsActive)
{
  const auto & spec = chickenHeadSpec();
  const auto tone = [&spec](const std::string & name) {
      return parseMode(makeArray({makeStatus(spec.row, name)}), spec).state.tone;
    };

  EXPECT_EQ(tone("blocked"), Tone::Warning);
  EXPECT_EQ(tone("hold"), Tone::Active);
  EXPECT_EQ(tone("off"), Tone::Quiet);
}

TEST(ModeModel, DemoIsOnWhileItRunsAndNotWhileItReturns)
{
  const auto & spec = demoSpec();
  const auto parse = [&spec](const std::string & name) {
      return parseMode(makeArray({makeStatus(spec.row, name)}), spec).state;
    };

  EXPECT_FALSE(parse("off").on);
  EXPECT_TRUE(parse("running").on);
  EXPECT_FALSE(parse("stopping").on);
  EXPECT_FALSE(parse("unavailable").on);
  EXPECT_EQ(parse("running").word, "running");
  EXPECT_EQ(parse("stopping").word, "returning");
  // Returning is something going on, so it is not shown as quiet, but it is not a problem.
  EXPECT_EQ(parse("stopping").tone, Tone::Active);
  EXPECT_EQ(parse("off").tone, Tone::Quiet);
}

TEST(ModeModel, ADemoThatStoppedItselfIsAnOffStateWithAWarning)
{
  const auto & spec = demoSpec();
  const auto snapshot = parseMode(
    makeArray({makeStatus(
        spec.row, "off", "Demo stopped: blocked by forearm_link: table",
        DiagnosticStatus::WARN)}), spec);

  ASSERT_TRUE(snapshot.valid);
  EXPECT_FALSE(snapshot.state.on);
  EXPECT_TRUE(snapshot.warning);
  EXPECT_EQ(snapshot.message, "Demo stopped: blocked by forearm_link: table");
}

TEST(ModeModel, EachModeOnlyReadsItsOwnRowAndStates)
{
  const auto array = makeArray({makeStatus("chicken_head", "hold"), makeStatus("demo", "running")});

  const auto head = parseMode(array, chickenHeadSpec());
  const auto demo = parseMode(array, demoSpec());
  ASSERT_TRUE(head.valid);
  ASSERT_TRUE(demo.valid);
  EXPECT_EQ(head.state.name, "hold");
  EXPECT_EQ(demo.state.name, "running");

  // A state of one mode is not a state of the other.
  const auto wrong = parseMode(makeArray({makeStatus("demo", "hold")}), demoSpec());
  EXPECT_FALSE(wrong.valid);
  EXPECT_EQ(wrong.error, "unknown state 'hold'");
  EXPECT_EQ(parseMode(makeArray({makeStatus("demo", "running")}), chickenHeadSpec()).error,
    "no chicken_head status");
}

TEST(ModeModel, AnOnStateExistsForEveryMode)
{
  for (const auto * spec : modeSpecs()) {
    EXPECT_TRUE(anOnState(*spec).on);
  }
}

TEST(ModeModel, TheTeleopNodeIsQuietUntilItHasReported)
{
  const auto now = std::chrono::steady_clock::now();

  EXPECT_TRUE(teleopQuiet(std::nullopt, now));
  EXPECT_FALSE(teleopQuiet(now - 100ms, now));
  EXPECT_FALSE(teleopQuiet(now - (kTeleopQuietAfter - 1ms), now));
  EXPECT_TRUE(teleopQuiet(now - kTeleopQuietAfter, now));
  EXPECT_TRUE(teleopQuiet(now - 1h, now));
}

TEST(ModeModel, TheWaitingMessageSaysWhatIsMissing)
{
  EXPECT_EQ(modeWaitingMessage(false, false), "Waiting for a valid report...");
  EXPECT_EQ(modeWaitingMessage(true, false), "Waiting for a valid report...");
  EXPECT_NE(modeWaitingMessage(false, true).find("teleop:=true"), std::string::npos);
  EXPECT_NE(modeWaitingMessage(true, true).find("stopped reporting"), std::string::npos);
}

}  // namespace
}  // namespace waybionic_rviz_plugins
