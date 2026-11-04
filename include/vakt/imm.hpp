#pragma once

#include "vakt/ekf.hpp"

#include <memory>
#include <vector>

namespace vakt {

// Interacting Multiple Model filter (Blom & Bar-Shalom 1988) over EKF mode filters.
// With a single model it reduces to a plain EKF, which is how the CV baseline is run.
//
// Mode switching is a continuous-time Markov chain with rate `switch_rate` [1/s], so the
// transition matrix stays sensible when the revisit time varies between scans.
class Imm {
 public:
  using ModelPtr = std::shared_ptr<const MotionModel>;

  Imm(std::vector<ModelPtr> models, double switch_rate);

  void init(const Gaussian& g);
  void predict(double dt);
  // Moment-matched prediction across modes, used for gating and assignment.
  const Gaussian& predicted() const { return combined_; }
  Innovation innovation(const Meas& z, const PolarSensor& sensor) const;
  void update(const Meas& z, const PolarSensor& sensor);
  // No detection this scan: keep the prediction, mode probabilities = predicted ones.
  void coast();

  const Gaussian& estimate() const { return combined_; }
  const Eigen::VectorXd& mode_probabilities() const { return mu_; }
  std::size_t num_modes() const { return models_.size(); }
  std::string_view mode_name(std::size_t i) const { return models_[i]->name(); }

 private:
  Eigen::MatrixXd transition(double dt) const;
  void combine();

  std::vector<ModelPtr> models_;
  double switch_rate_;
  std::vector<Gaussian> modes_;
  Eigen::VectorXd mu_;
  Eigen::VectorXd mu_pred_;
  Gaussian combined_;
};

}  // namespace vakt
