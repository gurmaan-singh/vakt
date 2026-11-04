#include "vakt/imm.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace vakt {

Imm::Imm(std::vector<ModelPtr> models, double switch_rate)
    : models_(std::move(models)), switch_rate_(switch_rate) {
  if (models_.empty()) throw std::invalid_argument("IMM needs at least one model");
  const auto m = static_cast<Eigen::Index>(models_.size());
  mu_ = Eigen::VectorXd::Constant(m, 1.0 / static_cast<double>(m));
  mu_pred_ = mu_;
  modes_.resize(models_.size());
}

void Imm::init(const Gaussian& g) {
  for (auto& mode : modes_) mode = g;
  mu_.setConstant(1.0 / static_cast<double>(models_.size()));
  mu_pred_ = mu_;
  combined_ = g;
}

Eigen::MatrixXd Imm::transition(double dt) const {
  const auto m = static_cast<Eigen::Index>(models_.size());
  if (m == 1) return Eigen::MatrixXd::Ones(1, 1);
  const double p_leave = 1.0 - std::exp(-switch_rate_ * dt);
  Eigen::MatrixXd T = Eigen::MatrixXd::Constant(m, m, p_leave / static_cast<double>(m - 1));
  T.diagonal().setConstant(1.0 - p_leave);
  return T;
}

void Imm::predict(double dt) {
  const auto m = static_cast<Eigen::Index>(models_.size());
  const Eigen::MatrixXd T = transition(dt);

  // 1. Mixing: cbar_j = sum_i T_ij mu_i,  w_ij = T_ij mu_i / cbar_j.
  mu_pred_ = T.transpose() * mu_;
  std::vector<Gaussian> mixed(models_.size());
  for (Eigen::Index j = 0; j < m; ++j) {
    Gaussian& out = mixed[static_cast<std::size_t>(j)];
    out.x.setZero();
    out.P.setZero();
    for (Eigen::Index i = 0; i < m; ++i) {
      const double w = T(i, j) * mu_(i) / mu_pred_(j);
      out.x += w * modes_[static_cast<std::size_t>(i)].x;
    }
    for (Eigen::Index i = 0; i < m; ++i) {
      const double w = T(i, j) * mu_(i) / mu_pred_(j);
      const auto& mi = modes_[static_cast<std::size_t>(i)];
      const State d = mi.x - out.x;
      out.P += w * (mi.P + d * d.transpose());
    }
  }

  // 2. Mode-matched prediction.
  for (std::size_t j = 0; j < models_.size(); ++j) modes_[j] = ekf_predict(mixed[j], *models_[j], dt);
  mu_ = mu_pred_;
  combine();
}

Innovation Imm::innovation(const Meas& z, const PolarSensor& sensor) const {
  return ekf_innovation(combined_, z, sensor);
}

void Imm::update(const Meas& z, const PolarSensor& sensor) {
  const auto m = static_cast<Eigen::Index>(models_.size());
  Eigen::VectorXd log_w(m);
  for (Eigen::Index j = 0; j < m; ++j) {
    auto& mode = modes_[static_cast<std::size_t>(j)];
    const Innovation inn = ekf_innovation(mode, z, sensor);
    log_w(j) = inn.log_likelihood() + std::log(mu_pred_(j));
    mode = ekf_update(mode, inn, sensor);
  }
  // Normalise in log space: likelihoods of a badly mismatched mode underflow quickly.
  const double mx = log_w.maxCoeff();
  mu_ = (log_w.array() - mx).exp();
  mu_ /= mu_.sum();
  // Keep every mode reachable, otherwise a mode that hits 0 can never come back.
  for (Eigen::Index j = 0; j < m; ++j) mu_(j) = std::max(mu_(j), 1e-6);
  mu_ /= mu_.sum();
  combine();
}

void Imm::coast() {
  mu_ = mu_pred_;
  combine();
}

void Imm::combine() {
  combined_.x.setZero();
  combined_.P.setZero();
  for (std::size_t j = 0; j < modes_.size(); ++j) combined_.x += mu_(static_cast<Eigen::Index>(j)) * modes_[j].x;
  for (std::size_t j = 0; j < modes_.size(); ++j) {
    const State d = modes_[j].x - combined_.x;
    combined_.P += mu_(static_cast<Eigen::Index>(j)) * (modes_[j].P + d * d.transpose());
  }
}

}  // namespace vakt
