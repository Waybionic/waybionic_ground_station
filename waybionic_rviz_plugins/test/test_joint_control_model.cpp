// Tests for the Qt-free parsing of /arm/joint_control. They build fabricated
// DiagnosticArray messages, so no drive node or ROS graph is needed.

#include <chrono>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

#include "waybionic_rviz_plugins/joint_control_model.hpp"

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
  const std::string & name, const std::string & state, const std::string & angle = "0.0",
  const std::string & drives = "", const std::string & message = "")
{
  DiagnosticStatus status;
  status.name = name;
  status.message = message;
  status.values.push_back(keyValue("state", state));
  status.values.push_back(keyValue("drives", drives));
  status.values.push_back(keyValue("angle", angle));
  return status;
}

DiagnosticArray makeArray(const std::vector<DiagnosticStatus> & statuses)
{
  DiagnosticArray array;
  array.status = statuses;
  return array;
}

// The array the drive node publishes for an arm with J2 and the tool unfitted.
DiagnosticArray healthyArray()
{
  return makeArray(
    {
      makeStatus("joint_1", "enabled", "0.1", "base_yaw", "enabled"),
      makeStatus("joint_2", "no_drive", "0.0", "", "no drive fitted: locked at 0.0 deg"),
      makeStatus("joint_3", "disabled", "0.2146755", "elbow", "disabled: held at +12.3 deg"),
      makeStatus("joint_4", "enabled", "-0.3490658503988659", "wrist_left, wrist_right", "enabled"),
      makeStatus("joint_5", "enabled", "0.0", "wrist_left,wrist_right", "enabled"),
      makeStatus("tool_grip", "no_drive", "0.0", "", "no drive fitted: locked at 0.0 deg"),
    });
}

std::vector<std::string> names(const JointControlSnapshot & snapshot)
{
  std::vector<std::string> result;
  for (const auto & row : snapshot.rows) {
    result.push_back(row.joint_name);
  }
  return result;
}

const JointRow & rowFor(const JointControlSnapshot & snapshot, const std::string & name)
{
  for (const auto & row : snapshot.rows) {
    if (row.joint_name == name) {
      return row;
    }
  }
  ADD_FAILURE() << "no row for " << name;
  static const JointRow missing;
  return missing;
}

TEST(JointControlModel, ParsesAllThreeStates)
{
  const auto snapshot = parseJointControl(healthyArray());

  ASSERT_TRUE(snapshot.valid) << snapshot.error;
  EXPECT_TRUE(snapshot.error.empty());
  ASSERT_EQ(snapshot.rows.size(), 6u);

  const auto & enabled = rowFor(snapshot, "joint_1");
  EXPECT_EQ(enabled.state, JointState::Enabled);
  EXPECT_TRUE(enabled.valid);
  EXPECT_EQ(enabled.message, "enabled");
  EXPECT_DOUBLE_EQ(enabled.angle_rad, 0.1);

  const auto & disabled = rowFor(snapshot, "joint_3");
  EXPECT_EQ(disabled.state, JointState::Disabled);
  EXPECT_EQ(disabled.message, "disabled: held at +12.3 deg");
  EXPECT_DOUBLE_EQ(disabled.angle_rad, 0.2146755);

  const auto & no_drive = rowFor(snapshot, "joint_2");
  EXPECT_EQ(no_drive.state, JointState::NoDrive);
  EXPECT_EQ(no_drive.message, "no drive fitted: locked at 0.0 deg");
  EXPECT_DOUBLE_EQ(no_drive.angle_rad, 0.0);
}

TEST(JointControlModel, ReadsNegativeAnglesWithFullPrecision)
{
  const auto snapshot = parseJointControl(healthyArray());

  ASSERT_TRUE(snapshot.valid);
  EXPECT_DOUBLE_EQ(rowFor(snapshot, "joint_4").angle_rad, -0.3490658503988659);
}

TEST(JointControlModel, SplitsAndTrimsDriveNames)
{
  const auto snapshot = parseJointControl(healthyArray());

  ASSERT_TRUE(snapshot.valid);
  EXPECT_EQ(rowFor(snapshot, "joint_1").drives, std::vector<std::string>{"base_yaw"});
  EXPECT_EQ(
    rowFor(snapshot, "joint_4").drives, (std::vector<std::string>{"wrist_left", "wrist_right"}));
  EXPECT_EQ(
    rowFor(snapshot, "joint_5").drives, (std::vector<std::string>{"wrist_left", "wrist_right"}));
  EXPECT_TRUE(rowFor(snapshot, "joint_2").drives.empty());
}

TEST(JointControlModel, DropsEmptyDriveNames)
{
  const auto snapshot =
    parseJointControl(makeArray({makeStatus("joint_1", "enabled", "0.0", " ,base_yaw,, ")}));

  ASSERT_TRUE(snapshot.valid);
  EXPECT_EQ(snapshot.rows.front().drives, std::vector<std::string>{"base_yaw"});
}

TEST(JointControlModel, ToleratesAMissingDrivesEntry)
{
  auto status = makeStatus("joint_1", "enabled");
  status.values.erase(status.values.begin() + 1);  // "drives"
  const auto snapshot = parseJointControl(makeArray({status}));

  ASSERT_TRUE(snapshot.valid) << snapshot.error;
  EXPECT_TRUE(snapshot.rows.front().drives.empty());
}

TEST(JointControlModel, KeepsTheKnownJointOrderWhateverTheArrivalOrder)
{
  const auto snapshot = parseJointControl(
    makeArray(
      {
        makeStatus("tool_grip", "no_drive"),
        makeStatus("joint_5", "enabled"),
        makeStatus("joint_3", "enabled"),
        makeStatus("joint_1", "enabled"),
        makeStatus("joint_4", "enabled"),
        makeStatus("joint_2", "no_drive"),
      }));

  ASSERT_TRUE(snapshot.valid);
  EXPECT_EQ(
    names(snapshot),
    (std::vector<std::string>{"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "tool_grip"}));
}

TEST(JointControlModel, PutsUnknownJointsLastInArrivalOrder)
{
  const auto snapshot = parseJointControl(
    makeArray(
      {
        makeStatus("wrist_roll", "enabled"),
        makeStatus("tool_grip", "no_drive"),
        makeStatus("joint_6", "disabled"),
        makeStatus("joint_1", "enabled"),
      }));

  ASSERT_TRUE(snapshot.valid);
  EXPECT_EQ(
    names(snapshot),
    (std::vector<std::string>{"joint_1", "tool_grip", "wrist_roll", "joint_6"}));
}

TEST(JointControlModel, LabelsKnownJointsAndKeepsRawNamesOtherwise)
{
  EXPECT_EQ(jointLabel("joint_1"), "J1");
  EXPECT_EQ(jointLabel("joint_2"), "J2");
  EXPECT_EQ(jointLabel("joint_3"), "J3");
  EXPECT_EQ(jointLabel("joint_4"), "J4");
  EXPECT_EQ(jointLabel("joint_5"), "J5");
  EXPECT_EQ(jointLabel("tool_grip"), "Tool");
  EXPECT_EQ(jointLabel("joint_6"), "joint_6");
  EXPECT_EQ(jointLabel("wrist_roll"), "wrist_roll");

  const auto snapshot = parseJointControl(
    makeArray({makeStatus("joint_3", "enabled"), makeStatus("wrist_roll", "enabled")}));
  ASSERT_TRUE(snapshot.valid);
  EXPECT_EQ(rowFor(snapshot, "joint_3").label, "J3");
  EXPECT_EQ(rowFor(snapshot, "wrist_roll").label, "wrist_roll");
}

TEST(JointControlModel, ListsTheKnownJointsInDisplayOrder)
{
  EXPECT_EQ(
    knownJointNames(),
    (std::vector<std::string>{"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "tool_grip"}));
}

TEST(JointControlModel, OnlyEnabledAndDisabledJointsAreServiceable)
{
  const auto snapshot = parseJointControl(healthyArray());

  ASSERT_TRUE(snapshot.valid);
  EXPECT_TRUE(rowFor(snapshot, "joint_1").serviceable);
  EXPECT_FALSE(rowFor(snapshot, "joint_2").serviceable);
  EXPECT_TRUE(rowFor(snapshot, "joint_3").serviceable);
  EXPECT_TRUE(rowFor(snapshot, "joint_4").serviceable);
  EXPECT_TRUE(rowFor(snapshot, "joint_5").serviceable);
  EXPECT_FALSE(rowFor(snapshot, "tool_grip").serviceable);
}

TEST(JointControlModel, IgnoresTheDiagnosticLevel)
{
  auto status = makeStatus("joint_1", "enabled");
  status.level = DiagnosticStatus::ERROR;
  auto locked = makeStatus("joint_2", "disabled");
  locked.level = DiagnosticStatus::OK;
  const auto snapshot = parseJointControl(makeArray({status, locked}));

  ASSERT_TRUE(snapshot.valid);
  EXPECT_EQ(rowFor(snapshot, "joint_1").state, JointState::Enabled);
  EXPECT_EQ(rowFor(snapshot, "joint_2").state, JointState::Disabled);
}

TEST(JointControlModel, IgnoresEntriesItDoesNotKnow)
{
  auto status = makeStatus("joint_1", "enabled");
  status.values.push_back(keyValue("firmware", "1.2"));
  const auto snapshot = parseJointControl(makeArray({status}));

  EXPECT_TRUE(snapshot.valid) << snapshot.error;
}

TEST(JointControlModel, MissingStateInvalidatesTheArray)
{
  auto broken = makeStatus("joint_3", "enabled");
  broken.values.erase(broken.values.begin());  // "state"
  const auto snapshot = parseJointControl(makeArray({makeStatus("joint_1", "enabled"), broken}));

  EXPECT_FALSE(snapshot.valid);
  EXPECT_NE(snapshot.error.find("joint_3"), std::string::npos) << snapshot.error;
  EXPECT_NE(snapshot.error.find("state"), std::string::npos) << snapshot.error;
  EXPECT_TRUE(rowFor(snapshot, "joint_1").valid);
  const auto & row = rowFor(snapshot, "joint_3");
  EXPECT_FALSE(row.valid);
  EXPECT_FALSE(row.serviceable);
  EXPECT_FALSE(row.problem.empty());
}

TEST(JointControlModel, UnknownStateInvalidatesTheArray)
{
  for (const std::string bad : {"faulted", "Enabled", "ENABLED", "no drive", "", " enabled"}) {
    const auto snapshot =
      parseJointControl(makeArray({makeStatus("joint_1", "enabled"), makeStatus("joint_2", bad)}));

    EXPECT_FALSE(snapshot.valid) << "state '" << bad << "' was accepted";
    EXPECT_FALSE(rowFor(snapshot, "joint_2").valid);
    EXPECT_FALSE(rowFor(snapshot, "joint_2").serviceable);
    EXPECT_NE(snapshot.error.find("joint_2"), std::string::npos) << snapshot.error;
  }
}

TEST(JointControlModel, DuplicateJointNamesInvalidateTheArray)
{
  const auto snapshot = parseJointControl(
    makeArray(
      {
        makeStatus("joint_1", "enabled"),
        makeStatus("joint_3", "enabled"),
        makeStatus("joint_3", "disabled"),
      }));

  EXPECT_FALSE(snapshot.valid);
  EXPECT_NE(snapshot.error.find("joint_3"), std::string::npos) << snapshot.error;
  EXPECT_NE(snapshot.error.find("duplicate"), std::string::npos) << snapshot.error;
  ASSERT_EQ(snapshot.rows.size(), 3u);
  EXPECT_TRUE(snapshot.rows[0].valid);
  EXPECT_TRUE(snapshot.rows[1].valid);
  EXPECT_FALSE(snapshot.rows[2].valid);
}

TEST(JointControlModel, NonNumericAngleInvalidatesTheRow)
{
  const std::vector<std::string> bad_angles = {
    "", "abc", "nan", "inf", "-inf", "1.5x", "0x10", " 1.0", "1.0 ", "1,5", "1e999", "--1", "1.2.3",
    "e5", "+", "."};
  for (const auto & bad : bad_angles) {
    const auto snapshot = parseJointControl(
      makeArray({makeStatus("joint_1", "enabled"), makeStatus("joint_4", "enabled", bad)}));

    EXPECT_FALSE(snapshot.valid) << "angle '" << bad << "' was accepted";
    EXPECT_TRUE(rowFor(snapshot, "joint_1").valid);
    EXPECT_FALSE(rowFor(snapshot, "joint_4").valid);
    EXPECT_NE(snapshot.error.find("joint_4"), std::string::npos) << snapshot.error;
    EXPECT_NE(snapshot.error.find("angle"), std::string::npos) << snapshot.error;
  }
}

TEST(JointControlModel, AcceptsPlainDecimalAngles)
{
  const std::vector<std::pair<std::string, double>> good_angles = {
    {"0.0", 0.0}, {"-0.3490658503988659", -0.3490658503988659}, {"1.5707963267948966",
      1.5707963267948966}, {"+0.5", 0.5}, {"-3", -3.0}, {"2.", 2.0}, {".25", 0.25}, {"1e-3", 0.001},
    {"-1.5E2", -150.0}};
  for (const auto & [text, expected] : good_angles) {
    const auto snapshot = parseJointControl(makeArray({makeStatus("joint_1", "enabled", text)}));

    ASSERT_TRUE(snapshot.valid) << "angle '" << text << "' was rejected: " << snapshot.error;
    EXPECT_DOUBLE_EQ(snapshot.rows.front().angle_rad, expected);
  }
}

TEST(JointControlModel, MissingAngleInvalidatesTheRow)
{
  auto status = makeStatus("joint_1", "enabled");
  status.values.pop_back();  // "angle"
  const auto snapshot = parseJointControl(makeArray({status}));

  EXPECT_FALSE(snapshot.valid);
  EXPECT_NE(snapshot.error.find("angle"), std::string::npos) << snapshot.error;
}

TEST(JointControlModel, RepeatedStateOrAngleEntriesAreAmbiguous)
{
  auto twice_state = makeStatus("joint_1", "enabled");
  twice_state.values.push_back(keyValue("state", "disabled"));
  auto twice_angle = makeStatus("joint_2", "enabled");
  twice_angle.values.push_back(keyValue("angle", "1.0"));
  const auto snapshot = parseJointControl(makeArray({twice_state, twice_angle}));

  EXPECT_FALSE(snapshot.valid);
  EXPECT_FALSE(rowFor(snapshot, "joint_1").valid);
  EXPECT_FALSE(rowFor(snapshot, "joint_2").valid);
}

TEST(JointControlModel, JointNamesMustBeUsableInAServiceName)
{
  for (const std::string bad : {"", "arm/joint_1", "/joint_1", "1joint", "joint 1", "joint-1"}) {
    const auto snapshot = parseJointControl(makeArray({makeStatus(bad, "enabled")}));

    EXPECT_FALSE(snapshot.valid) << "name '" << bad << "' was accepted";
    EXPECT_FALSE(snapshot.rows.front().serviceable);
  }
}

TEST(JointControlModel, AnEmptyArrayIsInvalid)
{
  const auto snapshot = parseJointControl(DiagnosticArray());

  EXPECT_FALSE(snapshot.valid);
  EXPECT_TRUE(snapshot.rows.empty());
  EXPECT_FALSE(snapshot.error.empty());
}

TEST(JointControlModel, ReportsTheFirstProblemAndCountsTheRest)
{
  const auto snapshot = parseJointControl(
    makeArray(
      {
        makeStatus("joint_1", "enabled"),
        makeStatus("joint_2", "bogus"),
        makeStatus("joint_3", "enabled", "x"),
        makeStatus("joint_4", "enabled", "y"),
      }));

  EXPECT_FALSE(snapshot.valid);
  EXPECT_NE(snapshot.error.find("joint_2"), std::string::npos) << snapshot.error;
  EXPECT_NE(snapshot.error.find("2 more"), std::string::npos) << snapshot.error;
}

TEST(JointControlModel, NamesAnonymousStatusesByPosition)
{
  const auto snapshot =
    parseJointControl(makeArray({makeStatus("joint_1", "enabled"), makeStatus("", "enabled")}));

  EXPECT_FALSE(snapshot.valid);
  EXPECT_NE(snapshot.error.find("status 2"), std::string::npos) << snapshot.error;
}

TEST(JointControlModel, SurvivesHostileStrings)
{
  const std::string long_text(100000, 'x');
  const std::string with_nul("1.0\0junk", 8);
  auto hostile = makeStatus(long_text, long_text, long_text, long_text, long_text);
  hostile.values.push_back(keyValue(long_text, long_text));
  hostile.values.push_back(keyValue("", ""));
  const auto snapshot = parseJointControl(
    makeArray(
      {
        hostile,
        makeStatus(std::string("joint_1\0x", 9), "enabled", with_nul),
        makeStatus("joint_2", with_nul),
      }));

  EXPECT_FALSE(snapshot.valid);
  EXPECT_EQ(snapshot.rows.size(), 3u);
  for (const auto & row : snapshot.rows) {
    EXPECT_FALSE(row.valid);
    EXPECT_FALSE(row.serviceable);
  }
}

TEST(JointControlModel, ServiceNamesFollowTheContract)
{
  EXPECT_EQ(setEnabledServiceName("joint_3"), "/arm/joint_3/set_enabled");
  EXPECT_EQ(setEnabledServiceName("tool_grip"), "/arm/tool_grip/set_enabled");
  EXPECT_STREQ(kJointControlTopic, "/arm/joint_control");
}

TEST(JointControlModel, StateWordsAreForTheOperator)
{
  EXPECT_STREQ(stateWord(JointState::Enabled), "enabled");
  EXPECT_STREQ(stateWord(JointState::Disabled), "disabled");
  EXPECT_STREQ(stateWord(JointState::NoDrive), "no drive");
}

TEST(JointControlModel, DriveNodeIsQuietWithoutAnyMessage)
{
  EXPECT_TRUE(driveNodeQuiet(std::nullopt, std::chrono::steady_clock::now()));
}

TEST(JointControlModel, DriveNodeIsQuietFromThreeSecondsWithoutAMessage)
{
  const auto last = std::chrono::steady_clock::now();

  EXPECT_FALSE(driveNodeQuiet(last, last));
  EXPECT_FALSE(driveNodeQuiet(last, last + 500ms));
  EXPECT_FALSE(driveNodeQuiet(last, last + 2999ms));
  EXPECT_TRUE(driveNodeQuiet(last, last + 3000ms));
  EXPECT_TRUE(driveNodeQuiet(last, last + 10s));
}

TEST(JointControlModel, AMessageFromTheFutureIsNotQuiet)
{
  const auto now = std::chrono::steady_clock::now();

  EXPECT_FALSE(driveNodeQuiet(now + 1s, now));
}

}  // namespace
}  // namespace waybionic_rviz_plugins
