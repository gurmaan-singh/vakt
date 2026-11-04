#include "vakt/geo.hpp"

#include "vakt/types.hpp"

#include <cmath>

namespace vakt {

namespace {
constexpr double kA = 6378137.0;                 // WGS84 semi-major axis
constexpr double kF = 1.0 / 298.257223563;       // flattening
constexpr double kE2 = kF * (2.0 - kF);          // first eccentricity squared
constexpr double kB = kA * (1.0 - kF);           // semi-minor axis
constexpr double kEp2 = (kA * kA - kB * kB) / (kB * kB);  // second eccentricity squared
}  // namespace

Eigen::Vector3d geodetic_to_ecef(const Geodetic& g) {
  const double lat = deg2rad(g.lat_deg);
  const double lon = deg2rad(g.lon_deg);
  const double N = kA / std::sqrt(1.0 - kE2 * std::sin(lat) * std::sin(lat));
  return {(N + g.alt_m) * std::cos(lat) * std::cos(lon),
          (N + g.alt_m) * std::cos(lat) * std::sin(lon),
          (N * (1.0 - kE2) + g.alt_m) * std::sin(lat)};
}

Geodetic ecef_to_geodetic(const Eigen::Vector3d& p) {
  // Bowring's method with two refinement steps: sub-millimetre for anything near the surface.
  const double x = p(0), y = p(1), z = p(2);
  const double lon = std::atan2(y, x);
  const double r = std::hypot(x, y);
  double beta = std::atan2(z, (1.0 - kF) * r);
  double lat = 0.0;
  for (int i = 0; i < 3; ++i) {
    lat = std::atan2(z + kEp2 * kB * std::pow(std::sin(beta), 3), r - kE2 * kA * std::pow(std::cos(beta), 3));
    beta = std::atan2((1.0 - kF) * std::sin(lat), std::cos(lat));
  }
  const double N = kA / std::sqrt(1.0 - kE2 * std::sin(lat) * std::sin(lat));
  const double alt = r * std::cos(lat) + (z + kE2 * N * std::sin(lat)) * std::sin(lat) - N;
  return {rad2deg(lat), rad2deg(lon), alt};
}

LocalFrame::LocalFrame(const Geodetic& origin) : origin_(origin), origin_ecef_(geodetic_to_ecef(origin)) {
  const double lat = deg2rad(origin.lat_deg);
  const double lon = deg2rad(origin.lon_deg);
  const double sl = std::sin(lat), cl = std::cos(lat), so = std::sin(lon), co = std::cos(lon);
  ecef_to_enu_ << -so, co, 0.0,
                  -sl * co, -sl * so, cl,
                  cl * co, cl * so, sl;
}

Eigen::Vector3d LocalFrame::to_enu(const Geodetic& g) const {
  return ecef_to_enu_ * (geodetic_to_ecef(g) - origin_ecef_);
}

Geodetic LocalFrame::to_geodetic(const Eigen::Vector3d& enu) const {
  return ecef_to_geodetic(origin_ecef_ + ecef_to_enu_.transpose() * enu);
}

Geodetic LocalFrame::to_geodetic_2d(double east, double north, double alt_m) const {
  // Walk along the tangent plane, then put the point back at the requested altitude.
  Geodetic g = to_geodetic({east, north, 0.0});
  g.alt_m = alt_m;
  return g;
}

}  // namespace vakt
