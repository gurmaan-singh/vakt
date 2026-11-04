#include "vakt/metrics.hpp"

#include "vakt/assignment.hpp"

#include <algorithm>
#include <cmath>

namespace vakt {

Gospa gospa(const std::vector<Eigen::Vector2d>& truth, const std::vector<Eigen::Vector2d>& estimates,
            double cutoff, double p) {
  Gospa g;
  const auto nX = static_cast<Eigen::Index>(truth.size());
  const auto nY = static_cast<Eigen::Index>(estimates.size());
  g.truth_to_estimate.assign(truth.size(), -1);
  const double cp = std::pow(cutoff, p);

  // With alpha = 2, a pair further apart than the cutoff costs exactly c^p, the same as
  // calling it one miss (c^p/2) plus one false track (c^p/2). So min(d, c)^p is the right cost.
  Eigen::MatrixXd cost(nX, nY);
  for (Eigen::Index i = 0; i < nX; ++i)
    for (Eigen::Index j = 0; j < nY; ++j)
      cost(i, j) = std::min(std::pow((truth[static_cast<std::size_t>(i)] - estimates[static_cast<std::size_t>(j)]).norm(), p), cp);

  const std::vector<int> a = solve_assignment(cost);
  int matched = 0;
  for (Eigen::Index i = 0; i < nX; ++i) {
    const int j = a[static_cast<std::size_t>(i)];
    if (j < 0 || cost(i, j) >= cp) continue;
    g.localisation += cost(i, j);
    g.truth_to_estimate[static_cast<std::size_t>(i)] = j;
    ++matched;
  }
  g.missed = static_cast<int>(nX) - matched;
  g.false_tracks = static_cast<int>(nY) - matched;
  g.total = std::pow(g.localisation + 0.5 * cp * (g.missed + g.false_tracks), 1.0 / p);
  return g;
}

void IdSwitchCounter::observe(int truth_id, unsigned matched_track_id) {
  Last& last = last_[truth_id];
  if (matched_track_id == 0) {
    last.matched = false;
    return;
  }
  if (last.id != 0 && last.id != matched_track_id) {
    // Same target, different track: a swap if it was matched last scan, otherwise the
    // old track died and a new one was started (fragmentation).
    if (last.matched)
      ++switches_;
    else
      ++fragments_;
  }
  last.id = matched_track_id;
  last.matched = true;
}

}  // namespace vakt
