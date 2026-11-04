#include "vakt/asterix.hpp"

#include "vakt/types.hpp"

#include <gtest/gtest.h>

using namespace vakt;
using namespace vakt::asterix;

TEST(Asterix, EncodesKnownBytes) {
  Cat048Plot p;
  p.sac = 0x07;
  p.sic = 0x2A;
  p.time_of_day = 1.0;                       // 128 -> 0x000080
  p.range_m = 10.0 * kMetresPerNm;           // 10 NM -> 2560 -> 0x0A00
  p.azimuth_rad = deg2rad(90.0);             // 0x4000
  // CAT, LEN (whole block = 3 + FSPEC 1 + 010:2 + 140:3 + 020:1 + 040:4 = 14), FSPEC, items
  const std::vector<std::uint8_t> expected{48,   0x00, 0x0E,             // header
                                           0xF0,                         // FSPEC: FRN 1-4
                                           0x07, 0x2A,                   // I048/010
                                           0x00, 0x00, 0x80,             // I048/140
                                           0x20,                         // I048/020 TYP=1
                                           0x0A, 0x00, 0x40, 0x00};      // I048/040
  EXPECT_EQ(encode_block(std::span(&p, 1)), expected);
}

TEST(Asterix, RoundTripWithinResolution) {
  std::vector<Cat048Plot> plots;
  for (int i = 0; i < 50; ++i) {
    Cat048Plot p;
    p.sac = 1;
    p.sic = 2;
    p.time_of_day = 43200.0 + i * 0.37;
    p.range_m = 1500.0 + i * 3217.3;
    p.azimuth_rad = deg2rad(i * 7.3);
    p.simulated = i % 2;
    plots.push_back(p);
  }
  const auto decoded = decode(encode_block(plots));
  ASSERT_EQ(decoded.size(), plots.size());
  for (std::size_t i = 0; i < plots.size(); ++i) {
    EXPECT_NEAR(decoded[i].range_m, plots[i].range_m, kMetresPerNm / 256.0 / 2 + 1e-9);
    EXPECT_NEAR(decoded[i].azimuth_rad, plots[i].azimuth_rad, deg2rad(360.0 / 65536.0));
    EXPECT_NEAR(decoded[i].time_of_day, plots[i].time_of_day, 1.0 / 128.0);
    EXPECT_EQ(decoded[i].simulated, plots[i].simulated);
    EXPECT_EQ(decoded[i].sic, 2);
  }
}

TEST(Asterix, SkipsOptionalItemsFromRealRadars) {
  // FSPEC 0xFF 0x80: FRN 1-7 present + FRN 8 (aircraft address).
  const std::vector<std::uint8_t> rec{
      0xFF, 0x80,
      0x01, 0x02,              // 010
      0x00, 0x01, 0x00,        // 140
      0x41, 0x00 + 0x00,       // 020: TYP=2 (single SSR) with FX, second octet
      0x05, 0x00, 0x80, 0x00,  // 040: 5 NM, 180 deg
      0x0F, 0xFF,              // 070
      0x01, 0x40,              // 090
      0xC0, 0x10, 0x20,        // 130: SRL + SRR present
      0xAB, 0xCD, 0xEF};       // 220
  std::vector<std::uint8_t> block{48, 0, static_cast<std::uint8_t>(3 + rec.size())};
  block.insert(block.end(), rec.begin(), rec.end());
  const auto plots = decode(block);
  ASSERT_EQ(plots.size(), 1u);
  EXPECT_EQ(plots[0].typ, 2);
  EXPECT_NEAR(plots[0].range_m, 5 * kMetresPerNm, 1e-9);
  EXPECT_NEAR(rad2deg(plots[0].azimuth_rad), 180.0, 1e-9);
}

TEST(Asterix, RejectsTruncatedAndSkipsOtherCategories) {
  Cat048Plot p;
  p.range_m = 1000;
  auto block = encode_block(std::span(&p, 1));
  auto truncated = block;
  truncated.pop_back();
  truncated[2] = static_cast<std::uint8_t>(truncated.size());
  EXPECT_THROW(decode(truncated), DecodeError);

  std::vector<std::uint8_t> other{34, 0, 5, 0xAA, 0xBB};  // a CAT034 block in front
  other.insert(other.end(), block.begin(), block.end());
  EXPECT_EQ(decode(other).size(), 1u);

  const std::vector<std::uint8_t> bad_len{48, 0, 200, 0xF0};
  EXPECT_THROW(decode(bad_len), DecodeError);
}
