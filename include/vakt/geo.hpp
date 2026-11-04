#pragma once

#include <Eigen/Dense>

namespace vakt {

struct Geodetic {
  double lat_deg = 0.0;
  double lon_deg = 0.0;
  double alt_m = 0.0;  // above the WGS84 ellipsoid
};

Eigen::Vector3d geodetic_to_ecef(const Geodetic& g);
Geodetic ecef_to_geodetic(const Eigen::Vector3d& ecef);

// Local east-north-up frame with its origin at the radar.
class LocalFrame {
 public:
  explicit LocalFrame(const Geodetic& origin);

  Eigen::Vector3d to_enu(const Geodetic& g) const;
  Geodetic to_geodetic(const Eigen::Vector3d& enu) const;
  // The tracker works in 2D: horizontal position, altitude handled outside the filter.
  Geodetic to_geodetic_2d(double east, double north, double alt_m) const;

  const Geodetic& origin() const { return origin_; }

 private:
  Geodetic origin_;
  Eigen::Vector3d origin_ecef_;
  Eigen::Matrix3d ecef_to_enu_;
};

}  // namespace vakt
