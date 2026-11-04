#include "vakt/ekf.hpp"

#include <cmath>
#include <numbers>

namespace vakt {

double Innovation::mahalanobis2() const { return nu.dot(S.ldlt().solve(nu)); }

double Innovation::log_likelihood() const {
  return -0.5 * (mahalanobis2() + std::log(S.determinant()) + 2.0 * std::log(2.0 * std::numbers::pi));
}

Gaussian ekf_predict(const Gaussian& g, const MotionModel& model, double dt) {
  const StateCov F = model.jacobian(g.x, dt);
  Gaussian out;
  out.x = model.f(g.x, dt);
  out.P = F * g.P * F.transpose() + model.noise(g.x, dt);
  out.P = 0.5 * (out.P + out.P.transpose());
  return out;
}

Innovation ekf_innovation(const Gaussian& pred, const Meas& z, const PolarSensor& sensor) {
  Innovation inn;
  inn.H = sensor.jacobian(pred.x);
  inn.nu = sensor.residual(z, sensor.h(pred.x));
  inn.S = inn.H * pred.P * inn.H.transpose() + sensor.noise();
  return inn;
}

Gaussian ekf_update(const Gaussian& pred, const Innovation& inn, const PolarSensor& sensor) {
  // K = P H' S^-1, solved rather than inverted.
  const Eigen::Matrix<double, kStateDim, 2> PHt = pred.P * inn.H.transpose();
  const Eigen::Matrix<double, kStateDim, 2> K = inn.S.ldlt().solve(PHt.transpose()).transpose();
  Gaussian out;
  out.x = pred.x + K * inn.nu;
  const StateCov IKH = StateCov::Identity() - K * inn.H;
  out.P = IKH * pred.P * IKH.transpose() + K * sensor.noise() * K.transpose();
  out.P = 0.5 * (out.P + out.P.transpose());
  return out;
}

}  // namespace vakt
