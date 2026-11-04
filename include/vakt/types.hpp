#pragma once

#include <Eigen/Dense>

#include <cstdint>
#include <numbers>
#include <vector>

namespace vakt {

// Track state in a local east-north tangent plane centred on the radar:
// [east, north, v_east, v_north, turn rate].  Units: m, m/s, rad/s.
// The turn rate is carried by every model so the IMM can mix states of equal size.
inline constexpr int kStateDim = 5;
using State = Eigen::Matrix<double, kStateDim, 1>;
using StateCov = Eigen::Matrix<double, kStateDim, kStateDim>;

// A 2D surveillance radar measures range and azimuth.
using Meas = Eigen::Vector2d;
using MeasCov = Eigen::Matrix2d;
using MeasJac = Eigen::Matrix<double, 2, kStateDim>;

struct Gaussian {
  State x = State::Zero();
  StateCov P = StateCov::Identity();
};

// One detection ("plot" in radar speak).
struct Plot {
  double range = 0.0;    // m
  double azimuth = 0.0;  // rad, clockwise from north, [0, 2pi)
  int truth_id = -1;     // simulation only: which target caused it, -1 = clutter
};

// Plots from one antenna revolution, stamped with a single time.
struct Scan {
  double time = 0.0;  // s
  std::vector<Plot> plots;
};

using TrackId = std::uint32_t;

constexpr double deg2rad(double d) { return d * std::numbers::pi / 180.0; }
constexpr double rad2deg(double r) { return r * 180.0 / std::numbers::pi; }

// Wrap an angle to (-pi, pi].
double wrap_pi(double a);
// Wrap an angle to [0, 2pi).
double wrap_2pi(double a);

}  // namespace vakt
