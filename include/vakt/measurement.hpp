#pragma once

#include "vakt/types.hpp"

namespace vakt {

// Range/azimuth sensor located at `position` in the tracking frame.
struct PolarSensor {
  Eigen::Vector2d position = Eigen::Vector2d::Zero();
  double sigma_range = 50.0;              // m
  double sigma_azimuth = deg2rad(0.25);   // rad

  Meas h(const State& x) const;
  MeasJac jacobian(const State& x) const;
  MeasCov noise() const;
  // z - zhat with the azimuth difference wrapped to (-pi, pi].
  Meas residual(const Meas& z, const Meas& zhat) const;

  Meas to_meas(const Plot& p) const { return {p.range, p.azimuth}; }
  // Converted-measurement position and covariance, used to start new tracks.
  Eigen::Vector2d to_cartesian(const Plot& p) const;
  Eigen::Matrix2d cartesian_cov(const Plot& p) const;
};

}  // namespace vakt
