#include "vakt/runner.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <string>

using namespace vakt;

namespace {

Scenario scenario(const std::string& name) { return load_scenario(std::string(VAKT_SCENARIO_DIR) + "/" + name + ".json"); }

// Average over a few seeds so the thresholds test the tracker, not one lucky draw.
RunSummary average(const Scenario& sc, const TrackerConfig& cfg, int seeds) {
  const Timeline tl = make_timeline(sc);
  RunSummary avg;
  for (int s = 0; s < seeds; ++s) {
    const RunSummary r = run_scenario(sc, tl, cfg, 1000 + static_cast<std::uint64_t>(s));
    avg.mean_gospa += r.mean_gospa / seeds;
    avg.mean_false += r.mean_false / seeds;
    avg.mean_missed += r.mean_missed / seeds;
    avg.id_switches += r.id_switches;
    avg.fragments += r.fragments;
  }
  return avg;
}

}  // namespace

TEST(Tracker, StartsAndHoldsTracksOnCleanStraightTarget) {
  Scenario sc;
  sc.duration = 120;
  sc.radar.scan_period = 2.0;
  sc.radar.pd = 1.0;
  sc.radar.clutter_rate = 0.0;
  TargetSpec t;
  t.id = 1;
  t.east = -20000;
  t.north = 25000;
  t.speed = 200;
  t.heading = deg2rad(110);
  sc.targets.push_back(t);

  const Timeline tl = make_timeline(sc);
  int confirmed_after = -1;
  int k = 0;
  const RunSummary r = run_scenario(sc, tl, TrackerConfig{}, 1, 500.0, [&](const ScanRecord& rec) {
    if (confirmed_after < 0 && !rec.tracks.empty()) confirmed_after = k;
    ++k;
  });
  EXPECT_EQ(confirmed_after, 2);  // M = 3 hits: scans 0, 1, 2
  EXPECT_EQ(r.id_switches, 0);
  EXPECT_EQ(r.fragments, 0);
  EXPECT_DOUBLE_EQ(r.mean_false, 0.0);
  // after the first 10 scans the track should sit well inside the radar's own accuracy at 30 km
  double worst = 0.0;
  for (std::size_t i = 10; i < r.rms_pos.size(); ++i) worst = std::max(worst, r.rms_pos[i]);
  EXPECT_LT(worst, 150.0);
}

TEST(Tracker, CrossingTargetsKeepTheirIds) {
  const RunSummary r = average(scenario("crossing"), TrackerConfig{}, 10);
  EXPECT_LE(r.id_switches, 1) << "over 10 runs";
  EXPECT_LT(r.mean_false, 0.3);
  EXPECT_LT(r.mean_gospa, 250.0);
}

TEST(Tracker, ImmBeatsCvOnManoeuvringTarget) {
  const Scenario sc = scenario("manoeuvre");
  TrackerConfig cv;
  cv.filter = FilterKind::CV;
  TrackerConfig imm;
  imm.filter = FilterKind::IMM;
  const RunSummary rcv = average(sc, cv, 6);
  const RunSummary rimm = average(sc, imm, 6);
  EXPECT_LT(rimm.mean_gospa, rcv.mean_gospa);
  EXPECT_LE(rimm.fragments, rcv.fragments);
}

TEST(Tracker, ClutterAloneConfirmsAlmostNothing) {
  Scenario sc;
  sc.duration = 200;
  sc.radar.scan_period = 2.0;
  sc.radar.clutter_rate = 30.0;
  const RunSummary r = run_scenario(sc, make_timeline(sc), TrackerConfig{}, 5);
  EXPECT_LT(r.mean_false, 0.5);
}
