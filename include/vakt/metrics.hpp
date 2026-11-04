#pragma once

#include <Eigen/Dense>

#include <map>
#include <vector>

namespace vakt {

// Generalised OSPA with alpha = 2 (Rahmathullah, Garcia-Fernandez, Svensson 2017).
// Splits the error into localisation, missed targets and false tracks, which is far more
// useful when tuning than a single number.
struct Gospa {
  double total = 0.0;         // GOSPA distance [m]
  double localisation = 0.0;  // sum of d^p over matched pairs (before the 1/p root)
  int missed = 0;
  int false_tracks = 0;
  std::vector<int> truth_to_estimate;  // matched estimate index per truth, -1 if none
};

Gospa gospa(const std::vector<Eigen::Vector2d>& truth, const std::vector<Eigen::Vector2d>& estimates,
            double cutoff, double p = 2.0);

// Per-truth bookkeeping across a run: how often the track id serving a target changed.
class IdSwitchCounter {
 public:
  // matched_track_id: id of the track matched to `truth_id` this scan, or 0 if unmatched.
  void observe(int truth_id, unsigned matched_track_id);
  int switches() const { return switches_; }
  int fragments() const { return fragments_; }

 private:
  struct Last {
    unsigned id = 0;
    bool matched = false;
  };
  std::map<int, Last> last_;
  int switches_ = 0;
  int fragments_ = 0;
};

}  // namespace vakt
