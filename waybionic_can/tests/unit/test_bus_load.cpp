// Bus-load estimate used by the UNO R4 sketches' 'status' command.

#include <gtest/gtest.h>

#include <string>

#include "common/BusLoad.h"

namespace waybionic
{
namespace
{

Frame frameWithDlc(uint8_t dlc)
{
  Frame frame;
  frame.id = 1;
  frame.dlc = dlc;
  return frame;
}

TEST(BusLoad, FrameBitsCoverNoStuffingAndWorstCaseStuffing)
{
  EXPECT_EQ(BusLoad::frameBitsMin(0), 47u);
  EXPECT_EQ(BusLoad::frameBitsMax(0), 55u);
  EXPECT_EQ(BusLoad::frameBitsMin(2), 63u);   // 31h read request
  EXPECT_EQ(BusLoad::frameBitsMax(2), 75u);
  EXPECT_EQ(BusLoad::frameBitsMin(8), 111u);  // F5h move, 31h reply
  EXPECT_EQ(BusLoad::frameBitsMax(8), 135u);
}

TEST(BusLoad, NothingIsReportedBeforeTheFirstWindowCloses)
{
  BusLoad load(1000);
  load.reset(500000, 0);
  load.note(frameWithDlc(8), 10);
  char text[160];
  load.format(999, text, sizeof(text));
  EXPECT_FALSE(load.hasWindow());
  EXPECT_EQ(std::string(text), "bus load: no 1000 ms window closed yet; 1 frames total");
}

TEST(BusLoad, OneWindowGivesTheLoadRange)
{
  BusLoad load(1000);
  load.reset(500000, 0);
  for (uint32_t t = 0; t < 100; ++t) {
    load.note(frameWithDlc(8), t * 10);
  }
  char text[160];
  load.format(1000, text, sizeof(text));
  ASSERT_TRUE(load.hasWindow());
  EXPECT_EQ(load.lastFrames(), 100u);
  EXPECT_EQ(load.lastMinPermille(), 22u);  // 11100 bits / 500000 bit/s
  EXPECT_EQ(load.lastMaxPermille(), 27u);  // 13500 bits
  EXPECT_EQ(std::string(text),
    "bus load last 1000 ms: 100 frames, 2.2-2.7 % of 500000 bit/s; peak 2.7 %; "
    "100 frames total");
}

TEST(BusLoad, IdleGapAveragesAndKeepsThePeak)
{
  BusLoad load(1000);
  load.reset(1000000, 0);
  for (uint32_t i = 0; i < 1000; ++i) {
    load.note(frameWithDlc(8), i);
  }
  load.update(1000);
  EXPECT_EQ(load.lastMaxPermille(), 135u);
  load.update(5000);
  EXPECT_EQ(load.lastFrames(), 0u);
  EXPECT_EQ(load.lastDurationMs(), 4000u);
  EXPECT_EQ(load.lastMaxPermille(), 0u);
  EXPECT_EQ(load.peakMaxPermille(), 135u);
  EXPECT_EQ(load.totalFrames(), 1000u);
}

TEST(BusLoad, HandlesClockWraparoundAndReset)
{
  BusLoad load(1000);
  const uint32_t start = 0xFFFFFF00u;
  load.reset(500000, start);
  load.note(frameWithDlc(2), start + 10);
  load.update(start + 999);
  EXPECT_FALSE(load.hasWindow());
  load.update(start + 1000);
  ASSERT_TRUE(load.hasWindow());
  EXPECT_EQ(load.lastFrames(), 1u);

  load.reset(500000, 0);
  EXPECT_FALSE(load.hasWindow());
  EXPECT_EQ(load.totalFrames(), 0u);
  EXPECT_EQ(load.peakMaxPermille(), 0u);
}

TEST(BusLoad, UnknownBitrateReportsZero)
{
  BusLoad load(1000);
  load.reset(0, 0);
  load.note(frameWithDlc(8), 0);
  load.update(1000);
  EXPECT_EQ(load.lastFrames(), 1u);
  EXPECT_EQ(load.lastMaxPermille(), 0u);
}

}  // namespace
}  // namespace waybionic
