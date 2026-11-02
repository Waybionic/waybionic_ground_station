// Tests for the Qt-free parsing of /arm/chicken_head. They build fabricated DiagnosticArray
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

#include "waybionic_rviz_plugins/chicken_head_model.hpp"

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
  const std::string & state, const std::string & message = "",
  const std::string & name = "chicken_head")
{
  DiagnosticStatus status;
  status.name = name;
  status.message = message;
  status.values.push_back(keyValue("state", state));
  return status;
}

DiagnosticArray makeArray(const std::vector<DiagnosticStatus> & statuses)
{
  DiagnosticArray array;
  array.status = statuses;
  return array;
}

TEST(ChickenHeadModel, ParsesEveryState)
{
  const std::vector<std::pair<std::string, HeadState>> states = {
    {"off", HeadState::Off},
    {"approach", HeadState::Approach},
    {"hold", HeadState::Hold},
    {"blocked", HeadState::Blocked},
    {"unavailable", HeadState::Unavailable},
  };
  for (const auto & [text, expected] : states) {
    const auto snapshot = parseChickenHead(makeArray({makeStatus(text)}));
    ASSERT_TRUE(snapshot.valid) << text << ": " << snapshot.error;
    EXPECT_EQ(snapshot.state, expected) << text;
    EXPECT_TRUE(snapshot.error.empty());
  }
}

TEST(ChickenHeadModel, KeepsTheMessageForTheOperator)
{
  const auto snapshot = parseChickenHead(
    makeArray({makeStatus("blocked", "The way to the head position is blocked by joint_4")}));

  ASSERT_TRUE(snapshot.valid);
  EXPECT_EQ(snapshot.message, "The way to the head position is blocked by joint_4");
}

TEST(ChickenHeadModel, AnOffReportNeedsNoMessage)
{
  const auto snapshot = parseChickenHead(makeArray({makeStatus("off")}));

  ASSERT_TRUE(snapshot.valid);
  EXPECT_TRUE(snapshot.message.empty());
}

TEST(ChickenHeadModel, IgnoresRowsWithOtherNames)
{
  const auto snapshot = parseChickenHead(
    makeArray({makeStatus("hold", "other", "teleop"), makeStatus("hold", "Holding the tip")}));

  ASSERT_TRUE(snapshot.valid) << snapshot.error;
  EXPECT_EQ(snapshot.message, "Holding the tip");
}

TEST(ChickenHeadModel, RejectsAnEmptyArray)
{
  const auto snapshot = parseChickenHead(DiagnosticArray());

  EXPECT_FALSE(snapshot.valid);
  EXPECT_EQ(snapshot.error, "no chicken_head status");
}

TEST(ChickenHeadModel, RejectsAnArrayWithoutTheHeadRow)
{
  const auto snapshot = parseChickenHead(makeArray({makeStatus("hold", "", "joint_1")}));

  EXPECT_FALSE(snapshot.valid);
  EXPECT_EQ(snapshot.error, "no chicken_head status");
}

TEST(ChickenHeadModel, RejectsDuplicateHeadRows)
{
  const auto snapshot = parseChickenHead(makeArray({makeStatus("off"), makeStatus("hold")}));

  EXPECT_FALSE(snapshot.valid);
  EXPECT_EQ(snapshot.error, "duplicate chicken_head status");
}

TEST(ChickenHeadModel, RejectsAMissingState)
{
  auto status = makeStatus("hold");
  status.values.clear();

  const auto snapshot = parseChickenHead(makeArray({status}));

  EXPECT_FALSE(snapshot.valid);
  EXPECT_EQ(snapshot.error, "missing 'state' value");
}

TEST(ChickenHeadModel, RejectsADuplicateState)
{
  auto status = makeStatus("hold");
  status.values.push_back(keyValue("state", "off"));

  const auto snapshot = parseChickenHead(makeArray({status}));

  EXPECT_FALSE(snapshot.valid);
  EXPECT_EQ(snapshot.error, "duplicate 'state' value");
}

TEST(ChickenHeadModel, RejectsAnUnknownState)
{
  const auto snapshot = parseChickenHead(makeArray({makeStatus("dancing")}));

  EXPECT_FALSE(snapshot.valid);
  EXPECT_EQ(snapshot.error, "unknown state 'dancing'");
}

TEST(ChickenHeadModel, ShortensAndCleansAnUnknownStateInTheError)
{
  const auto snapshot = parseChickenHead(
    makeArray({makeStatus(std::string("bad\nstate") + std::string(60, 'x'))}));

  EXPECT_FALSE(snapshot.valid);
  EXPECT_EQ(snapshot.error.find('\n'), std::string::npos);
  EXPECT_LT(snapshot.error.size(), 80u);
  EXPECT_NE(snapshot.error.find("..."), std::string::npos);
}

TEST(ChickenHeadModel, OnlyAnActiveModeIsOn)
{
  EXPECT_FALSE(headIsOn(HeadState::Off));
  EXPECT_TRUE(headIsOn(HeadState::Approach));
  EXPECT_TRUE(headIsOn(HeadState::Hold));
  EXPECT_TRUE(headIsOn(HeadState::Blocked));
  EXPECT_FALSE(headIsOn(HeadState::Unavailable));
}

TEST(ChickenHeadModel, NamesEveryStateInAWord)
{
  EXPECT_STREQ(headStateWord(HeadState::Off), "off");
  EXPECT_STREQ(headStateWord(HeadState::Approach), "moving to the head spot");
  EXPECT_STREQ(headStateWord(HeadState::Hold), "holding");
  EXPECT_STREQ(headStateWord(HeadState::Blocked), "blocked");
  EXPECT_STREQ(headStateWord(HeadState::Unavailable), "unavailable");
}

TEST(ChickenHeadModel, TheTeleopNodeIsQuietUntilItHasReported)
{
  const auto now = std::chrono::steady_clock::now();

  EXPECT_TRUE(teleopQuiet(std::nullopt, now));
  EXPECT_FALSE(teleopQuiet(now - 100ms, now));
  EXPECT_FALSE(teleopQuiet(now - (kTeleopQuietAfter - 1ms), now));
  EXPECT_TRUE(teleopQuiet(now - kTeleopQuietAfter, now));
  EXPECT_TRUE(teleopQuiet(now - 1h, now));
}

TEST(ChickenHeadModel, TheWaitingMessageSaysWhatIsMissing)
{
  EXPECT_EQ(headWaitingMessage(false, false), "Waiting for a valid chicken head report...");
  EXPECT_EQ(headWaitingMessage(true, false), "Waiting for a valid chicken head report...");
  EXPECT_NE(headWaitingMessage(false, true).find("teleop:=true"), std::string::npos);
  EXPECT_NE(headWaitingMessage(true, true).find("stopped reporting"), std::string::npos);
}

TEST(ChickenHeadModel, TheTopicAndServiceMatchTheTeleopNode)
{
  EXPECT_STREQ(kChickenHeadTopic, "/arm/chicken_head");
  EXPECT_STREQ(kChickenHeadService, "/arm/chicken_head/set_enabled");
}

}  // namespace
}  // namespace waybionic_rviz_plugins
