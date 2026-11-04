#include "vakt/measurement.hpp"

#include <algorithm>
#include <cmath>

namespace vakt {

Meas PolarSensor::h(const State& x) const {
  const double de = x(0) - position(0);
  const double dn = x(1) - position(1);
  return {std::hypot(de, dn), wrap_2pi(std::atan2(de, dn))};
}

MeasJac PolarSensor::jacobian(const State& x) const {
  const double de = x(0) - position(0);
  const double dn = x(1) - position(1);
  // A target sitting on the antenna has no defined azimuth; clamp so the Jacobian stays finite.
  const double r2 = std::max(de * de + dn * dn, 1.0);
  const double r = std::sqrt(r2);
  MeasJac H = MeasJac::Zero();
  H(0, 0) = de / r;
  H(0, 1) = dn / r;
  H(1, 0) = dn / r2;
  H(1, 1) = -de / r2;
  return H;
}

MeasCov PolarSensor::noise() const {
  MeasCov R = MeasCov::Zero();
  R(0, 0) = sigma_range * sigma_range;
  R(1, 1) = sigma_azimuth * sigma_azimuth;
  return R;
}

Meas PolarSensor::residual(const Meas& z, const Meas& zhat) const {
  return {z(0) - zhat(0), wrap_pi(z(1) - zhat(1))};
}

Eigen::Vector2d PolarSensor::to_cartesian(const Plot& p) const {
  return position + Eigen::Vector2d{p.range * std::sin(p.azimuth), p.range * std::cos(p.azimuth)};
}

Eigen::Matrix2d PolarSensor::cartesian_cov(const Plot& p) const {
  // First-order propagation of the polar noise; good enough for initiation, where the
  // velocity uncertainty dominates anyway.
  Eigen::Matrix2d J;
  J << std::sin(p.azimuth), p.range * std::cos(p.azimuth),
       std::cos(p.azimuth), -p.range * std::sin(p.azimuth);
  return J * noise() * J.transpose();
}

}  // namespace vakt
