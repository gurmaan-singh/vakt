// Tracker throughput: time per scan as the number of targets and clutter grows.
// The cost is dominated by gating (tracks x plots) and the O(n^3) assignment.
#include "cli.hpp"
#include "vakt/runner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <random>

using namespace vakt;

namespace {

Scenario random_scenario(int targets, double clutter, std::uint64_t seed) {
  Scenario sc;
  sc.name = "bench";
  sc.duration = 120;
  sc.radar.scan_period = 2.0;
  sc.radar.pd = 0.9;
  sc.radar.clutter_rate = clutter;
  sc.radar.max_range = 150000;
  std::mt19937_64 rng(seed);
  std::uniform_real_distribution<double> r(10e3, 140e3), a(0, 2 * std::numbers::pi), v(50, 300);
  for (int i = 0; i < targets; ++i) {
    TargetSpec t;
    t.id = i + 1;
    const double rr = r(rng), aa = a(rng);
    t.east = rr * std::sin(aa);
    t.north = rr * std::cos(aa);
    t.speed = v(rng);
    t.heading = a(rng);
    sc.targets.push_back(t);
  }
  return sc;
}

}  // namespace

int main(int argc, char** argv) {
  cli::Args args(argc, argv, "usage: vakt_bench [--clutter-ratio R]   (clutter per scan = R x targets, default 1)");
  const double ratio = args.number("--clutter-ratio", 1.0);
  std::cout << "| targets | clutter/scan | mean [us/scan] | p99 [us/scan] | max [us/scan] |\n|---:|---:|---:|---:|---:|\n";
  for (int n : {10, 25, 50, 100, 200, 400}) {
    const Scenario sc = random_scenario(n, ratio * n, 7);
    const Timeline tl = make_timeline(sc);
    std::vector<double> us;
    run_scenario(sc, tl, TrackerConfig{}, 1, 500.0, [&](const ScanRecord& rec) {
      if (rec.scan.time > 10.0) us.push_back(rec.process_us);  // skip initiation transient
    });
    std::sort(us.begin(), us.end());
    double mean = 0;
    for (double x : us) mean += x / static_cast<double>(us.size());
    const double p99 = us[static_cast<std::size_t>(0.99 * static_cast<double>(us.size() - 1))];
    std::cout << std::fixed << std::setprecision(0) << "| " << n << " | " << ratio * n << " | " << mean << " | " << p99
              << " | " << us.back() << " |\n";
  }
  return 0;
}
