#include "vakt/metrics.hpp"

#include <gtest/gtest.h>

#include <cmath>

using namespace vakt;
using V = Eigen::Vector2d;

TEST(Gospa, PerfectEstimateIsZero) {
  const auto g = gospa({V{0, 0}, V{100, 0}}, {V{100, 0}, V{0, 0}}, 500.0);
  EXPECT_DOUBLE_EQ(g.total, 0.0);
  EXPECT_EQ(g.missed, 0);
  EXPECT_EQ(g.false_tracks, 0);
  EXPECT_EQ(g.truth_to_estimate, (std::vector<int>{1, 0}));
}

TEST(Gospa, LocalisationErrorOnly) {
  const auto g = gospa({V{0, 0}}, {V{30, 40}}, 500.0);
  EXPECT_NEAR(g.total, 50.0, 1e-12);
}

TEST(Gospa, MissedAndFalseCostHalfCutoffEach) {
  const double c = 200.0;
  // one missed target: sqrt(c^2 / 2)
  EXPECT_NEAR(gospa({V{0, 0}}, {}, c).total, c / std::sqrt(2.0), 1e-9);
  // one false track, same cost
  EXPECT_NEAR(gospa({}, {V{0, 0}}, c).total, c / std::sqrt(2.0), 1e-9);
  // far apart pair = one missed + one false, not a "match"
  const auto g = gospa({V{0, 0}}, {V{5000, 0}}, c);
  EXPECT_EQ(g.missed, 1);
  EXPECT_EQ(g.false_tracks, 1);
  EXPECT_NEAR(g.total, c, 1e-9);
}

TEST(IdSwitches, CountsSwapsAndFragments) {
  IdSwitchCounter c;
  c.observe(1, 10);
  c.observe(1, 10);
  c.observe(1, 11);  // swap
  c.observe(1, 0);   // lost
  c.observe(1, 12);  // new track after a gap: fragment
  c.observe(1, 12);
  EXPECT_EQ(c.switches(), 1);
  EXPECT_EQ(c.fragments(), 1);
}
