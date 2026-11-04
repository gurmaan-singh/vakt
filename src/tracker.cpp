#include "vakt/tracker.hpp"

#include "vakt/assignment.hpp"

#include <algorithm>
#include <cmath>

namespace vakt {

namespace {
// Cost for pairs outside the gate. Finite so the Hungarian solver never sees inf - inf.
constexpr double kForbidden = 1e12;
}  // namespace

Tracker::Tracker(TrackerConfig cfg, PolarSensor sensor) : cfg_(cfg), sensor_(std::move(sensor)) {
  models_.push_back(std::make_shared<ConstantVelocity>(cfg_.q_cv));
  if (cfg_.filter == FilterKind::IMM) models_.push_back(std::make_shared<CoordinatedTurn>(cfg_.q_ct, cfg_.q_turn));
}

Imm Tracker::make_filter() const { return Imm(models_, cfg_.filter == FilterKind::IMM ? cfg_.switch_rate : 0.0); }

Track Tracker::start_track(const Plot& plot, double time) {
  Gaussian g;
  g.x.setZero();
  g.x.head<2>() = sensor_.to_cartesian(plot);
  g.P.setZero();
  g.P.topLeftCorner<2, 2>() = sensor_.cartesian_cov(plot);
  g.P(2, 2) = g.P(3, 3) = cfg_.init_speed_sigma * cfg_.init_speed_sigma;
  g.P(4, 4) = cfg_.init_turn_sigma * cfg_.init_turn_sigma;

  Track t(next_id_++, make_filter());
  t.filter.init(g);
  t.born = time;
  t.last_update = time;
  t.scans = 1;
  t.hits = 1;
  if (plot.truth_id >= 0) t.last_truth = plot.truth_id;
  return t;
}

ScanStats Tracker::process(const Scan& scan) {
  ScanStats stats;
  stats.plots = static_cast<int>(scan.plots.size());
  const double dt = started_ ? scan.time - last_time_ : 0.0;
  started_ = true;
  last_time_ = scan.time;

  for (auto& t : tracks_)
    if (dt > 0.0) t.filter.predict(dt);

  // Gating + cost matrix. Columns [0, nP) are plots, column nP + i is "track i misses".
  // Cost = d^2 + ln|S| (negative log likelihood up to a constant), so a track is only
  // assigned a plot when that beats missing it, i.e. when d^2 < gate.
  const auto nT = static_cast<Eigen::Index>(tracks_.size());
  const auto nP = static_cast<Eigen::Index>(scan.plots.size());
  Eigen::MatrixXd cost = Eigen::MatrixXd::Constant(nT, nP + nT, kForbidden);
  for (Eigen::Index i = 0; i < nT; ++i) {
    const Gaussian& pred = tracks_[static_cast<std::size_t>(i)].filter.predicted();
    const MeasJac H = sensor_.jacobian(pred.x);
    const MeasCov S = H * pred.P * H.transpose() + sensor_.noise();
    const auto Sdec = S.ldlt();
    const double log_det = std::log(S.determinant());
    const Meas zhat = sensor_.h(pred.x);
    for (Eigen::Index j = 0; j < nP; ++j) {
      const Meas nu = sensor_.residual(sensor_.to_meas(scan.plots[static_cast<std::size_t>(j)]), zhat);
      const double d2 = nu.dot(Sdec.solve(nu));
      if (d2 < cfg_.gate) cost(i, j) = d2 + log_det;
    }
    cost(i, nP + i) = cfg_.gate + log_det;
  }

  const std::vector<int> assignment = solve_assignment(cost);
  std::vector<char> plot_used(static_cast<std::size_t>(nP), 0);

  for (std::size_t i = 0; i < tracks_.size(); ++i) {
    Track& t = tracks_[i];
    t.scans += 1;
    const int col = assignment[i];
    if (col >= 0 && col < nP) {
      const Plot& plot = scan.plots[static_cast<std::size_t>(col)];
      t.filter.update(sensor_.to_meas(plot), sensor_);
      t.hits += 1;
      t.consecutive_misses = 0;
      t.last_update = scan.time;
      t.last_truth = plot.truth_id >= 0 ? std::optional<int>(plot.truth_id) : std::nullopt;
      plot_used[static_cast<std::size_t>(col)] = 1;
      ++stats.assigned;
    } else {
      t.filter.coast();
      t.consecutive_misses += 1;
    }
  }

  // Track management.
  const auto lost = [&](const Track& t) {
    if (t.status == TrackStatus::Tentative) {
      return t.scans - t.hits > cfg_.confirm_n - cfg_.confirm_m;  // can no longer reach M of N
    }
    const Eigen::Matrix2d Ppos = t.filter.estimate().P.topLeftCorner<2, 2>();
    const double sigma = std::sqrt(Ppos.selfadjointView<Eigen::Upper>().eigenvalues().maxCoeff());
    return t.consecutive_misses >= cfg_.delete_misses || sigma > cfg_.max_position_sigma;
  };
  const auto before = tracks_.size();
  std::erase_if(tracks_, lost);
  stats.deleted = static_cast<int>(before - tracks_.size());

  for (auto& t : tracks_) {
    if (t.status == TrackStatus::Tentative && t.hits >= cfg_.confirm_m) t.status = TrackStatus::Confirmed;
    if (t.status == TrackStatus::Confirmed) ++stats.confirmed;
  }

  for (Eigen::Index j = 0; j < nP; ++j) {
    if (plot_used[static_cast<std::size_t>(j)]) continue;
    tracks_.push_back(start_track(scan.plots[static_cast<std::size_t>(j)], scan.time));
    ++stats.new_tentative;
  }
  return stats;
}

std::vector<TrackReport> Tracker::confirmed_tracks() const {
  std::vector<TrackReport> out;
  for (const auto& t : tracks_) {
    if (t.status != TrackStatus::Confirmed) continue;
    TrackReport r;
    r.id = t.id;
    r.time = last_time_;
    r.x = t.filter.estimate().x;
    r.P = t.filter.estimate().P;
    r.p_turn = t.filter.num_modes() > 1 ? t.filter.mode_probabilities()(1) : 0.0;
    r.hits = t.hits;
    r.age = last_time_ - t.born;
    out.push_back(r);
  }
  return out;
}

}  // namespace vakt
