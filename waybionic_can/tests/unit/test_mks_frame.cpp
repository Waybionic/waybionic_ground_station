// Frame layout checks. The vectors are copied from waybionic_teleop/test/test_mks_can.py
// (PR #24), which checks them against the MKS SERVO42D/57D CAN manual V1.0.9.

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "common/MksFrame.h"

namespace waybionic
{
namespace
{

std::string dataHex(const Frame & frame)
{
  static const char kHex[] = "0123456789ABCDEF";
  std::string out;
  for (uint8_t i = 0; i < frame.dlc; ++i) {
    out += kHex[frame.data[i] >> 4];
    out += kHex[frame.data[i] & 0x0F];
  }
  return out;
}

Frame fromHex(const uint16_t id, const std::string & hex)
{
  Frame frame;
  frame.id = id;
  frame.dlc = static_cast<uint8_t>(hex.size() / 2);
  for (uint8_t i = 0; i < frame.dlc; ++i) {
    frame.data[i] = static_cast<uint8_t>(std::stoul(hex.substr(2 * i, 2), nullptr, 16));
  }
  return frame;
}

Frame built(bool ok, const Frame & frame)
{
  EXPECT_TRUE(ok);
  return frame;
}

TEST(MksFrame, FramesMatchTheManual)
{
  Frame f;
  const std::vector<std::pair<Frame, std::string>> cases = {
    {built(mks::absoluteAxis(1, 0x4000, 600, 2, f), f), "F502580200400092"},
    {built(mks::absoluteAxis(1, -0x4000, 600, 2, f), f), "F5025802FFC00011"},
    {built(mks::absoluteAxis(1, 0x28000, 300, 2, f), f), "F5012C02028000A7"},
    {built(mks::absoluteAxis(1, 0x7F8000, 300, 2, f), f), "F5012C027F800024"},
    {built(mks::absoluteAxis(1, 0, 0, 4, f), f), "F5000004000000FA"},
    {built(mks::readEncoder(1, f), f), "3132"},
    {built(mks::setMode(1, mks::kModeSrVfoc, f), f), "820588"},
    {built(mks::enable(1, true, f), f), "F301F5"},
    {built(mks::setResponse(1, true, false, f), f), "8C01008E"},
    {built(mks::setHeartbeat(1, 500, f), f), "98000001F48E"},
  };
  for (const auto & [frame, expected] : cases) {
    EXPECT_EQ(frame.id, 1u);
    EXPECT_EQ(dataHex(frame), expected);
  }
}

TEST(MksFrame, EncoderReplyDecodesNegativeValues)
{
  const uint8_t value[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0};
  Frame frame;
  ASSERT_TRUE(mks::buildFrame(1, mks::kReadEncoder, value, 6, frame));
  mks::Message message;
  ASSERT_EQ(mks::parseFrame(frame, message), mks::ParseResult::kOk);
  EXPECT_EQ(message.code, mks::kReadEncoder);
  int64_t decoded = 0;
  ASSERT_TRUE(mks::decodeEncoderValue(message, decoded));
  EXPECT_EQ(decoded, -16);
}

TEST(MksFrame, ParseRejectsBadChecksumsAndLengths)
{
  mks::Message message;
  EXPECT_EQ(mks::parseFrame(fromHex(1, "3133"), message), mks::ParseResult::kBadChecksum);
  EXPECT_EQ(mks::parseFrame(fromHex(1, "31"), message), mks::ParseResult::kBadDlc);
  Frame for_node_1;
  ASSERT_TRUE(mks::readEncoder(1, for_node_1));
  for_node_1.id = 2;  // the checksum covers the CAN ID
  EXPECT_EQ(mks::parseFrame(for_node_1, message), mks::ParseResult::kBadChecksum);
}

TEST(MksFrame, AbsoluteAxisRejectsOutOfRangeArguments)
{
  Frame frame;
  EXPECT_FALSE(mks::absoluteAxis(1, 0x800000, 100, 2, frame));
  EXPECT_FALSE(mks::absoluteAxis(1, 0, 3001, 2, frame));
  EXPECT_FALSE(mks::absoluteAxis(1, 0, 100, 256, frame));
  EXPECT_FALSE(mks::absoluteAxis(1, 0, -1, 2, frame));
  EXPECT_TRUE(mks::absoluteAxis(1, mks::kMaxAxis, mks::kMaxSpeedRpm, 255, frame));
  EXPECT_TRUE(mks::absoluteAxis(1, mks::kMinAxis, 0, 0, frame));
}

TEST(MksFrame, FormatMatchesCandump)
{
  Frame frame;
  ASSERT_TRUE(mks::readEncoder(1, frame));
  char text[32];
  mks::formatFrame(frame, text, sizeof(text));
  EXPECT_STREQ(text, "001#3132");
}

TEST(MksFrame, BuildRejectsNonStandardIdsAndTooManyArguments)
{
  Frame frame;
  EXPECT_TRUE(mks::readEncoder(0x7FF, frame));
  EXPECT_FALSE(mks::readEncoder(0x800, frame));
  const uint8_t seven[7] = {};
  EXPECT_FALSE(mks::buildFrame(1, mks::kReadEncoder, seven, 7, frame));
  const uint8_t six[6] = {};
  ASSERT_TRUE(mks::buildFrame(1, mks::kReadEncoder, six, 6, frame));
  EXPECT_EQ(frame.dlc, 8u);
}

TEST(MksFrame, AbsoluteAxisRoundTrip)
{
  for (const int32_t axis : {
      0, 1, -1, 0x4000, -0x4000, mks::kMaxAxis, mks::kMinAxis})
  {
    Frame frame;
    ASSERT_TRUE(mks::absoluteAxis(7, axis, 1234, 200, frame));
    mks::Message message;
    ASSERT_EQ(mks::parseFrame(frame, message), mks::ParseResult::kOk);
    mks::AbsoluteAxisArgs args;
    ASSERT_TRUE(mks::decodeAbsoluteAxis(message, args));
    EXPECT_EQ(args.axis, axis);
    EXPECT_EQ(args.speed_rpm, 1234);
    EXPECT_EQ(args.acc, 200);
  }
}

TEST(MksFrame, EncoderReplyRoundTrip)
{
  for (const int64_t value : {0LL, 16LL, -16LL, 0x7FFFFFFFFFFFLL, -0x800000000000LL}) {
    Frame frame;
    ASSERT_TRUE(mks::encoderReply(3, value, frame));
    mks::Message message;
    ASSERT_EQ(mks::parseFrame(frame, message), mks::ParseResult::kOk);
    int64_t decoded = 0;
    ASSERT_TRUE(mks::decodeEncoderValue(message, decoded));
    EXPECT_EQ(decoded, value);
  }
}

TEST(MksFrame, ChecksumIsIdPlusBodyLowByte)
{
  const uint8_t body[] = {0xF5, 0x02, 0x58, 0x02, 0x00, 0x40, 0x00};
  EXPECT_EQ(mks::checksum(1, body, sizeof(body)), 0x92);
  // The whole ID is added, so IDs above 0xFF only contribute their low byte.
  EXPECT_EQ(mks::checksum(0x101, body, sizeof(body)), 0x92);
}

TEST(MksFrame, ValidChecksumParses)
{
  mks::Message message;
  ASSERT_EQ(mks::parseFrame(fromHex(1, "F301F5"), message), mks::ParseResult::kOk);
  EXPECT_EQ(message.code, mks::kEnable);
  ASSERT_EQ(message.argument_count, 1u);
  EXPECT_EQ(message.arguments[0], 1u);
}

TEST(MksFrame, InvalidDlcIsRejected)
{
  mks::Message message;
  Frame empty;
  empty.id = 1;
  EXPECT_EQ(mks::parseFrame(empty, message), mks::ParseResult::kBadDlc);
  Frame too_long = fromHex(1, "3132");
  too_long.dlc = 9;
  EXPECT_EQ(mks::parseFrame(too_long, message), mks::ParseResult::kBadDlc);
  Frame bad_id = fromHex(0x800, "3132");
  EXPECT_EQ(mks::parseFrame(bad_id, message), mks::ParseResult::kBadId);
}

TEST(MksFrame, DecodersRejectWrongArgumentCounts)
{
  mks::Message message;
  ASSERT_EQ(mks::parseFrame(fromHex(1, "3132"), message), mks::ParseResult::kOk);
  int64_t value = 0;
  EXPECT_FALSE(mks::decodeEncoderValue(message, value));  // a request, not a 6-byte reply
  mks::AbsoluteAxisArgs args;
  EXPECT_FALSE(mks::decodeAbsoluteAxis(message, args));
  uint8_t status = 0;
  EXPECT_FALSE(mks::decodeStatus(message, status));
}

}  // namespace
}  // namespace waybionic
