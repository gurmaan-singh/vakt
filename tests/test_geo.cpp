#include "vakt/geo.hpp"

#include <gtest/gtest.h>

using namespace vakt;

TEST(Geo, EcefOfKnownPoint) {
  // Equator / prime meridian at sea level sits on the semi-major axis.
  const Eigen::Vector3d p = geodetic_to_ecef({0.0, 0.0, 0.0});
  EXPECT_NEAR(p(0), 6378137.0, 1e-6);
  EXPECT_NEAR(p(1), 0.0, 1e-6);
  EXPECT_NEAR(p(2), 0.0, 1e-6);
  // North pole: semi-minor axis.
  EXPECT_NEAR(geodetic_to_ecef({90.0, 0.0, 0.0})(2), 6356752.314245, 1e-5);
}

TEST(Geo, EcefRoundTrip) {
  for (const Geodetic g : {Geodetic{58.41, 15.62, 120.0}, Geodetic{-33.9, 151.2, 11000.0}, Geodetic{69.65, 18.96, 0.0}}) {
    const Geodetic back = ecef_to_geodetic(geodetic_to_ecef(g));
    EXPECT_NEAR(back.lat_deg, g.lat_deg, 1e-9);
    EXPECT_NEAR(back.lon_deg, g.lon_deg, 1e-9);
    EXPECT_NEAR(back.alt_m, g.alt_m, 1e-4);
  }
}

TEST(Geo, LocalFrameAxes) {
  // Linköping (Saab), 1 km north and 1 km east of the origin.
  const LocalFrame f({58.4108, 15.6214, 0.0});
  const Eigen::Vector3d north = f.to_enu({58.4108 + 1000.0 / 111400.0, 15.6214, 0.0});
  EXPECT_NEAR(north(0), 0.0, 0.5);
  EXPECT_NEAR(north(1), 1000.0, 5.0);
  const Eigen::Vector3d back = f.to_enu(f.to_geodetic({2500.0, -7300.0, 300.0}));
  EXPECT_NEAR(back(0), 2500.0, 1e-6);
  EXPECT_NEAR(back(1), -7300.0, 1e-6);
  EXPECT_NEAR(back(2), 300.0, 1e-6);
}
