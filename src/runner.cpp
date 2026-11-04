#include "vakt/runner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace vakt {

Timeline make_timeline(const Scenario& sc) {
  Timeline tl;
  const double T = sc.radar.scan_period;
  for (double t = T; t <= sc.duration + 1e-9; t += T) {
    tl.times.push_back(t);
    tl.truth.push_back(truth_at(sc, t));
  }
  return tl;
}

RunSummary run_scenario(const Scenario& sc, const Timeline& tl, const TrackerConfig& cfg, std::uint64_t seed,
                        double cutoff, const ScanObserver& observer) {
  std::mt19937_64 rng(seed);
  Tracker tracker(cfg, sc.radar.sensor);
  IdSwitchCounter ids;
  RunSummary sum;
  double missed = 0.0, false_tracks = 0.0, total_us = 0.0;

  for (std::size_t k = 0; k < tl.times.size(); ++k) {
    const auto& truth = tl.truth[k];
    const Scan scan = simulate_scan(sc, tl.times[k], truth, rng);

    const auto t0 = std::chrono::steady_clock::now();
    tracker.process(scan);
    const auto t1 = std::chrono::steady_clock::now();
    const double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
    total_us += us;
    sum.max_process_us = std::max(sum.max_process_us, us);

    const std::vector<TrackReport> tracks = tracker.confirmed_tracks();
    std::vector<Eigen::Vector2d> X, Y;
    for (const auto& tp : truth) X.push_back(tp.pos);
    for (const auto& tr : tracks) Y.push_back(tr.x.head<2>());
    const Gospa g = gospa(X, Y, cutoff);

    double se = 0.0;
    int matched = 0;
    for (std::size_t i = 0; i < truth.size(); ++i) {
      const int j = g.truth_to_estimate[i];
      ids.observe(truth[i].id, j >= 0 ? tracks[static_cast<std::size_t>(j)].id : 0u);
      if (j >= 0) {
        se += (X[i] - Y[static_cast<std::size_t>(j)]).squaredNorm();
        ++matched;
      }
    }
    sum.gospa.push_back(g.total);
    sum.rms_pos.push_back(matched ? std::sqrt(se / matched) : std::numeric_limits<double>::quiet_NaN());
    missed += g.missed;
    false_tracks += g.false_tracks;

    if (observer) observer(ScanRecord{scan, truth, tracks, g, us});
  }

  const double n = std::max<double>(1.0, static_cast<double>(tl.times.size()));
  for (double v : sum.gospa) sum.mean_gospa += v / n;
  sum.mean_missed = missed / n;
  sum.mean_false = false_tracks / n;
  sum.mean_process_us = total_us / n;
  sum.id_switches = ids.switches();
  sum.fragments = ids.fragments();
  return sum;
}

}  // namespace vakt
