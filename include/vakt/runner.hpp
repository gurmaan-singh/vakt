#pragma once

#include "vakt/metrics.hpp"
#include "vakt/sim.hpp"
#include "vakt/tracker.hpp"

#include <cstdint>
#include <functional>
#include <vector>

namespace vakt {

// Ground truth at every scan time. Deterministic, so Monte Carlo runs share one copy.
struct Timeline {
  std::vector<double> times;
  std::vector<std::vector<TruthPoint>> truth;
};
Timeline make_timeline(const Scenario& sc);

struct ScanRecord {
  const Scan& scan;
  const std::vector<TruthPoint>& truth;
  const std::vector<TrackReport>& tracks;
  const Gospa& gospa;
  double process_us;
};
using ScanObserver = std::function<void(const ScanRecord&)>;

struct RunSummary {
  std::vector<double> gospa;       // per scan
  std::vector<double> rms_pos;     // per scan, RMS position error of matched tracks (NaN if none)
  double mean_gospa = 0.0;
  double mean_missed = 0.0;        // targets per scan without a track
  double mean_false = 0.0;         // confirmed tracks per scan not on a target
  int id_switches = 0;
  int fragments = 0;
  double mean_process_us = 0.0;
  double max_process_us = 0.0;
};

// Simulates the radar with `seed`, runs the tracker over every scan and scores it with
// GOSPA (cutoff `cutoff` metres) against the truth.
RunSummary run_scenario(const Scenario& sc, const Timeline& tl, const TrackerConfig& cfg, std::uint64_t seed,
                        double cutoff = 500.0, const ScanObserver& observer = {});

}  // namespace vakt
