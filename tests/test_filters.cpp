#include "vakt/imm.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <random>

using namespace vakt;

namespace {

PolarSensor sensor() {
  PolarSensor s;
  s.sigma_range = 30.0;
  s.sigma_azimuth = deg2rad(0.2);
  return s;
}

Meas measure(const PolarSensor& s, const State& truth, std::mt19937_64& rng) {
  std::normal_distribution<double> n01;
  Meas z = s.h(truth);
  z(0) += s.sigma_range * n01(rng);
  z(1) = wrap_2pi(z(1) + s.sigma_azimuth * n01(rng));
  return z;
}

Gaussian initial(const State& truth) {
  Gaussian g;
  g.x = truth;
  g.x(2) = g.x(3) = g.x(4) = 0.0;  // unknown velocity, like a real one-point start
  g.P = StateCov::Zero();
  g.P(0, 0) = g.P(1, 1) = 200.0 * 200.0;
  g.P(2, 2) = g.P(3, 3) = 200.0 * 200.0;
  g.P(4, 4) = deg2rad(3) * deg2rad(3);
  return g;
}

}  // namespace

TEST(Measurement, JacobianMatchesNumeric) {
  const PolarSensor s = sensor();
  State x;
  x << 12000, -5000, 0, 0, 0;
  const MeasJac H = s.jacobian(x);
  for (int k = 0; k < 2; ++k) {
    State xp = x, xm = x;
    xp(k) += 1.0;
    xm(k) -= 1.0;
    const Meas d = s.residual(s.h(xp), s.h(xm)) / 2.0;
    EXPECT_NEAR(H(0, k), d(0), 1e-9);
    EXPECT_NEAR(H(1, k), d(1), 1e-9);
  }
}

TEST(Measurement, AzimuthResidualWrapsThroughNorth) {
  const PolarSensor s = sensor();
  const Meas r = s.residual({1000, deg2rad(1)}, {1000, deg2rad(359)});
  EXPECT_NEAR(r(1), deg2rad(2), 1e-12);
}

// The filter is "consistent" if its covariance matches its actual errors. Over many runs
// the normalised estimation error squared (NEES) averages to the state dimension.
// The truth has to be driven by the same process noise the filter assumes, otherwise the
// filter is (correctly) pessimistic and NEES comes out low.
TEST(Ekf, CvFilterIsConsistentOnStraightLine) {
  const PolarSensor s = sensor();
  const auto cv = std::make_shared<ConstantVelocity>(0.5);
  std::mt19937_64 rng(7);
  std::normal_distribution<double> n01;
  constexpr int kRuns = 300;
  constexpr int kSteps = 40;
  constexpr double kDt = 2.0;
  const StateCov Lq = cv->noise(State::Zero(), kDt).llt().matrixL();
  double nees_sum = 0.0;
  for (int run = 0; run < kRuns; ++run) {
    State truth;
    truth << 20000, 15000, -180, 60, 0;
    Imm f({cv}, 0.0);
    f.init(initial(truth));
    for (int k = 0; k < kSteps; ++k) {
      State w;
      for (int i = 0; i < kStateDim; ++i) w(i) = n01(rng);
      truth = cv->f(truth, kDt) + Lq * w;
      f.predict(kDt);
      f.update(measure(s, truth, rng), s);
    }
    const Eigen::Vector4d e = (f.estimate().x - truth).head<4>();
    nees_sum += e.dot(f.estimate().P.topLeftCorner<4, 4>().ldlt().solve(e));
  }
  const double anees = nees_sum / kRuns;
  // 99% two-sided chi-square bounds for 4 dof averaged over 300 runs: [3.48, 4.58].
  // Slightly wider on the upper side for the EKF linearisation of the polar measurement.
  EXPECT_GT(anees, 3.3);
  EXPECT_LT(anees, 4.9);
}

TEST(Imm, SwitchesToTurnModelDuringManoeuvre) {
  const PolarSensor s = sensor();
  const auto cv = std::make_shared<ConstantVelocity>(0.5);
  const auto ct = std::make_shared<CoordinatedTurn>(1.0, 1e-4);
  std::mt19937_64 rng(3);
  Imm f({cv, ct}, 0.05);
  State truth;
  truth << 10000, 30000, 250, 0, 0;
  f.init(initial(truth));
  constexpr double kDt = 1.0;
  for (int k = 0; k < 30; ++k) {  // straight
    truth = cv->f(truth, kDt);
    f.predict(kDt);
    f.update(measure(s, truth, rng), s);
  }
  const double p_ct_straight = f.mode_probabilities()(1);
  truth(4) = deg2rad(8.0);  // ~3.6 g at 250 m/s
  for (int k = 0; k < 15; ++k) {
    truth = ct->f(truth, kDt);
    f.predict(kDt);
    f.update(measure(s, truth, rng), s);
  }
  EXPECT_LT(p_ct_straight, 0.5);
  EXPECT_GT(f.mode_probabilities()(1), 0.8);
  EXPECT_NEAR(rad2deg(f.estimate().x(4)), 8.0, 1.0);
  // At ~30 km a 0.2 deg azimuth error alone is ~105 m cross-range, so allow about 2.5 sigma.
  const double cross_range_sigma = s.h(truth)(0) * s.sigma_azimuth;
  EXPECT_LT((f.estimate().x.head<2>() - truth.head<2>()).norm(), 2.5 * cross_range_sigma);
}

TEST(Imm, CoastingGrowsUncertainty) {
  const auto cv = std::make_shared<ConstantVelocity>(1.0);
  Imm f({cv}, 0.0);
  State x;
  x << 0, 10000, 100, 0, 0;
  f.init(initial(x));
  const double before = f.estimate().P.trace();
  f.predict(4.0);
  f.coast();
  EXPECT_GT(f.estimate().P.trace(), before);
}
