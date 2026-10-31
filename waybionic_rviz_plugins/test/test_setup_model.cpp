// Tests for the Qt-free parsing of /arm/setup. They build fabricated DiagnosticArray
// messages, so no setup_guide node or ROS graph is needed.

#include <chrono>
#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

#include "waybionic_rviz_plugins/setup_model.hpp"

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

DiagnosticStatus makeStep(
  const std::string & name, const std::string & title, const std::string & state,
  const std::string & message, const std::string & action = "",
  const std::string & current = "false")
{
  DiagnosticStatus status;
  status.name = name;
  status.message = message;
  status.values.push_back(keyValue("title", title));
  status.values.push_back(keyValue("state", state));
  status.values.push_back(keyValue("action", action));
  status.values.push_back(keyValue("current", current));
  return status;
}

DiagnosticArray makeArray(const std::vector<DiagnosticStatus> & statuses)
{
  DiagnosticArray array;
  array.status = statuses;
  return array;
}

// What setup_guide publishes while the arm waits to be zeroed.
DiagnosticArray zeroingArray()
{
  return makeArray(
    {
      makeStep("setup.carrier", "Carrier", "ok", "Carrier connected"),
      makeStep("setup.drives", "Drives", "ok", "4 drives answer"),
      makeStep("setup.controller", "Controller", "ok", "Controller connected"),
      makeStep(
        "setup.zero", "Zero", "todo",
        "Put the arm in its zero pose (upright, as RViz shows it), then press Zero", "zero",
        "true"),
      makeStep("setup.ready", "Ready", "todo", "Waits for the steps above"),
    });
}

DiagnosticArray readyArray()
{
  return makeArray(
    {
      makeStep("setup.carrier", "Carrier", "ok", "Carrier connected"),
      makeStep("setup.drives", "Drives", "ok", "4 drives answer"),
      makeStep("setup.controller", "Controller", "ok", "Controller connected"),
      makeStep("setup.zero", "Zero", "ok", "Zeroed"),
      makeStep("setup.ready", "Ready", "ok", "Teleop is on"),
    });
}

}  // namespace

TEST(SetupModel, ParsesTheFiveStepsInOrder)
{
  const auto snapshot = parseSetup(zeroingArray());
  ASSERT_TRUE(snapshot.valid) << snapshot.error;
  ASSERT_EQ(5u, snapshot.steps.size());
  EXPECT_EQ("carrier", snapshot.steps[0].name);
  EXPECT_EQ("Carrier", snapshot.steps[0].title);
  EXPECT_EQ(StepState::Ok, snapshot.steps[0].state);
  EXPECT_EQ("zero", snapshot.steps[3].name);
  EXPECT_EQ(StepState::Todo, snapshot.steps[3].state);
  EXPECT_EQ("zero", snapshot.steps[3].action);
  EXPECT_TRUE(snapshot.steps[3].current);
  EXPECT_FALSE(snapshot.steps[4].current);
  EXPECT_EQ("Waits for the steps above", snapshot.steps[4].message);
}

TEST(SetupModel, ReadsEveryState)
{
  const auto snapshot = parseSetup(
    makeArray(
      {
        makeStep("setup.carrier", "Carrier", "ok", ""),
        makeStep("setup.drives", "Drives", "todo", ""),
        makeStep("setup.controller", "Controller", "blocked", ""),
        makeStep("setup.zero", "Zero", "working", ""),
      }));
  ASSERT_TRUE(snapshot.valid) << snapshot.error;
  EXPECT_EQ(StepState::Ok, snapshot.steps[0].state);
  EXPECT_EQ(StepState::Todo, snapshot.steps[1].state);
  EXPECT_EQ(StepState::Blocked, snapshot.steps[2].state);
  EXPECT_EQ(StepState::Working, snapshot.steps[3].state);
}

TEST(SetupModel, KnowsWhenEverythingIsReady)
{
  EXPECT_FALSE(parseSetup(zeroingArray()).ready());
  EXPECT_TRUE(parseSetup(readyArray()).ready());
  EXPECT_FALSE(parseSetup(DiagnosticArray()).ready());
}

TEST(SetupModel, FindsTheCurrentStep)
{
  const auto snapshot = parseSetup(zeroingArray());
  ASSERT_NE(nullptr, snapshot.currentStep());
  EXPECT_EQ("zero", snapshot.currentStep()->name);
  EXPECT_EQ(nullptr, parseSetup(readyArray()).currentStep());
}

TEST(SetupModel, OnlyTheZeroActionIsKnown)
{
  auto array = zeroingArray();
  array.status[3].values[2].value = "reboot";
  const auto snapshot = parseSetup(array);
  EXPECT_FALSE(snapshot.valid);
  EXPECT_NE(std::string::npos, snapshot.error.find("action"));
  EXPECT_NE(std::string::npos, snapshot.error.find("zero"));
}

TEST(SetupModel, AMissingActionMeansNone)
{
  auto array = zeroingArray();
  array.status[3].values.erase(array.status[3].values.begin() + 2);
  const auto snapshot = parseSetup(array);
  ASSERT_TRUE(snapshot.valid) << snapshot.error;
  EXPECT_EQ("", snapshot.steps[3].action);
}

TEST(SetupModel, AMissingTitleFallsBackToTheStepName)
{
  auto array = zeroingArray();
  array.status[1].values.erase(array.status[1].values.begin());
  const auto snapshot = parseSetup(array);
  ASSERT_TRUE(snapshot.valid) << snapshot.error;
  EXPECT_EQ("drives", snapshot.steps[1].title);
}

TEST(SetupModel, UnknownOrMissingStatesInvalidateTheSnapshot)
{
  auto array = zeroingArray();
  array.status[0].values[1].value = "fine";
  auto snapshot = parseSetup(array);
  EXPECT_FALSE(snapshot.valid);
  EXPECT_NE(std::string::npos, snapshot.error.find("carrier"));
  EXPECT_NE(std::string::npos, snapshot.error.find("fine"));

  array = zeroingArray();
  array.status[0].values.erase(array.status[0].values.begin() + 1);
  snapshot = parseSetup(array);
  EXPECT_FALSE(snapshot.valid);
  EXPECT_NE(std::string::npos, snapshot.error.find("state"));
}

TEST(SetupModel, StepsMustBeNamedSetupSomething)
{
  auto array = zeroingArray();
  array.status[2].name = "controller";
  const auto snapshot = parseSetup(array);
  EXPECT_FALSE(snapshot.valid);
  EXPECT_NE(std::string::npos, snapshot.error.find("controller"));
}

TEST(SetupModel, DuplicateStepsInvalidateTheSnapshot)
{
  auto array = zeroingArray();
  array.status[1].name = "setup.carrier";
  const auto snapshot = parseSetup(array);
  EXPECT_FALSE(snapshot.valid);
  EXPECT_NE(std::string::npos, snapshot.error.find("duplicate"));
}

TEST(SetupModel, AnEmptyArrayIsInvalid)
{
  const auto snapshot = parseSetup(DiagnosticArray());
  EXPECT_FALSE(snapshot.valid);
  EXPECT_FALSE(snapshot.error.empty());
}

TEST(SetupModel, KeepsKnownStepsInOrderAndUnknownOnesLast)
{
  const auto snapshot = parseSetup(
    makeArray(
      {
        makeStep("setup.extra", "Extra", "ok", ""),
        makeStep("setup.ready", "Ready", "todo", ""),
        makeStep("setup.carrier", "Carrier", "ok", ""),
      }));
  ASSERT_TRUE(snapshot.valid) << snapshot.error;
  ASSERT_EQ(3u, snapshot.steps.size());
  EXPECT_EQ("carrier", snapshot.steps[0].name);
  EXPECT_EQ("ready", snapshot.steps[1].name);
  EXPECT_EQ("extra", snapshot.steps[2].name);
}

TEST(SetupModel, SurvivesHostileStrings)
{
  auto array = zeroingArray();
  array.status[0].message = std::string("a\0b\xff\n", 5);
  array.status[1].values[0].value = std::string(10000, 'x');
  array.status[2].values[1].value = std::string("\x01\x02", 2);
  const auto snapshot = parseSetup(array);
  EXPECT_FALSE(snapshot.valid);
  EXPECT_LT(snapshot.error.size(), 200u);
}

TEST(SetupModel, StateWordsAreForTheOperator)
{
  EXPECT_STREQ("done", stepStateWord(StepState::Ok));
  EXPECT_STREQ("to do", stepStateWord(StepState::Todo));
  EXPECT_STREQ("blocked", stepStateWord(StepState::Blocked));
  EXPECT_STREQ("working", stepStateWord(StepState::Working));
}

TEST(SetupModel, TheGuideIsQuietFromThreeSecondsWithoutAMessage)
{
  const auto now = std::chrono::steady_clock::now();
  EXPECT_TRUE(setupGuideQuiet(std::nullopt, now));
  EXPECT_FALSE(setupGuideQuiet(now - 2900ms, now));
  EXPECT_TRUE(setupGuideQuiet(now - 3000ms, now));
  EXPECT_FALSE(setupGuideQuiet(now + 1s, now));
}

TEST(SetupModel, TheZeroServiceIsTheDriveNodes)
{
  EXPECT_STREQ("/sim_arm_drives/zero", kZeroServiceName);
  EXPECT_STREQ("/arm/setup", kSetupTopic);
}

}  // namespace waybionic_rviz_plugins
