#pragma once

#include "vakt/imm.hpp"

#include <cstdint>
#include <optional>
#include <vector>

namespace vakt {

enum class FilterKind { CV, IMM };

struct TrackerConfig {
  FilterKind filter = FilterKind::IMM;
  double q_cv = 1.0;           // CV process noise [m^2/s^3]: gentle manoeuvres
  double q_ct = 4.0;           // CT position/velocity noise
  double q_turn = 1e-4;        // CT turn-rate diffusion [rad^2/s^3]
  double switch_rate = 0.05;   // IMM mode switching rate [1/s]

  double gate = 13.8;          // chi-square gate, 2 dof, P_G = 0.999
  int confirm_m = 3;           // confirm after M hits ...
  int confirm_n = 4;           // ... within the first N scans
  int delete_misses = 3;       // confirmed tracks die after this many consecutive misses
  double max_position_sigma = 3000.0;  // or when they are this lost [m]

  double init_speed_sigma = 250.0;     // m/s, one-point initiation, covers jets
  double init_turn_sigma = deg2rad(3.0);  // rad/s
};

enum class TrackStatus : std::uint8_t { Tentative, Confirmed };

struct Track {
  Track(TrackId id_, Imm f) : id(id_), filter(std::move(f)) {}

  TrackId id = 0;
  TrackStatus status = TrackStatus::Tentative;
  Imm filter;
  double born = 0.0;
  double last_update = 0.0;
  int scans = 0;
  int hits = 0;
  int consecutive_misses = 0;
  std::optional<int> last_truth;  // simulation only: truth id of the last associated plot
};

// Snapshot of one confirmed track for publishing / evaluation.
struct TrackReport {
  TrackId id = 0;
  double time = 0.0;
  State x = State::Zero();
  StateCov P = StateCov::Identity();
  double p_turn = 0.0;  // probability of the CT mode (0 for the CV filter)
  int hits = 0;
  double age = 0.0;
};

struct ScanStats {
  int plots = 0;
  int assigned = 0;
  int new_tentative = 0;
  int confirmed = 0;
  int deleted = 0;
};

// Global-nearest-neighbour tracker: predict, gate, solve one assignment per scan,
// update, then M-of-N confirmation and miss-count deletion.
class Tracker {
 public:
  Tracker(TrackerConfig cfg, PolarSensor sensor);

  ScanStats process(const Scan& scan);

  std::vector<TrackReport> confirmed_tracks() const;
  const std::vector<Track>& tracks() const { return tracks_; }
  const TrackerConfig& config() const { return cfg_; }
  const PolarSensor& sensor() const { return sensor_; }

 private:
  Imm make_filter() const;
  Track start_track(const Plot& plot, double time);

  TrackerConfig cfg_;
  PolarSensor sensor_;
  std::vector<Track> tracks_;
  TrackId next_id_ = 1;
  double last_time_ = 0.0;
  bool started_ = false;
  std::vector<Imm::ModelPtr> models_;
};

}  // namespace vakt
