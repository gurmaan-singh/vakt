#include "vakt/models.hpp"

#include <gtest/gtest.h>

#include <cmath>

using namespace vakt;

namespace {

State make_state(double e, double n, double ve, double vn, double w) {
  State x;
  x << e, n, ve, vn, w;
  return x;
}

// Central differences, to check the analytic Jacobians.
StateCov numeric_jacobian(const MotionModel& m, const State& x, double dt) {
  StateCov J;
  for (int k = 0; k < kStateDim; ++k) {
    const double h = k == 4 ? 1e-7 : 1e-4;
    State xp = x, xm = x;
    xp(k) += h;
    xm(k) -= h;
    J.col(k) = (m.f(xp, dt) - m.f(xm, dt)) / (2 * h);
  }
  return J;
}

}  // namespace

TEST(Models, WrapAngles) {
  EXPECT_NEAR(wrap_pi(3 * std::numbers::pi), std::numbers::pi, 1e-12);
  EXPECT_NEAR(wrap_pi(-std::numbers::pi / 2), -std::numbers::pi / 2, 1e-12);
  EXPECT_NEAR(wrap_2pi(-0.1), 2 * std::numbers::pi - 0.1, 1e-12);
  EXPECT_NEAR(wrap_2pi(7.0), 7.0 - 2 * std::numbers::pi, 1e-12);
}

TEST(Models, ConstantVelocityMovesStraight) {
  const ConstantVelocity cv(1.0);
  const State x = cv.f(make_state(0, 0, 10, -5, 0), 2.0);
  EXPECT_DOUBLE_EQ(x(0), 20.0);
  EXPECT_DOUBLE_EQ(x(1), -10.0);
}

TEST(Models, CoordinatedTurnClosesFullCircle) {
  const CoordinatedTurn ct(1.0, 1e-4);
  const double w = deg2rad(3.0);  // standard-rate turn
  const double period = 2 * std::numbers::pi / w;
  State x = make_state(1000, 2000, 0, 150, w);
  for (int i = 0; i < 120; ++i) x = ct.f(x, period / 120);
  EXPECT_NEAR(x(0), 1000.0, 1e-6);
  EXPECT_NEAR(x(1), 2000.0, 1e-6);
  EXPECT_NEAR(x(3), 150.0, 1e-9);
}

TEST(Models, CoordinatedTurnIsClockwiseForPositiveRate) {
  const CoordinatedTurn ct(1.0, 1e-4);
  // Flying north, turning right: east velocity must become positive.
  const State x = ct.f(make_state(0, 0, 0, 100, deg2rad(5)), 2.0);
  EXPECT_GT(x(2), 0.0);
  EXPECT_GT(x(0), 0.0);
}

TEST(Models, CoordinatedTurnReducesToCvAtZeroRate) {
  const CoordinatedTurn ct(1.0, 1e-4);
  const ConstantVelocity cv(1.0);
  const State x = make_state(10, 20, 30, -40, 0.0);
  EXPECT_TRUE(ct.f(x, 3.0).isApprox(cv.f(x, 3.0), 1e-12));
}

TEST(Models, AnalyticJacobiansMatchNumeric) {
  const CoordinatedTurn ct(1.0, 1e-4);
  const ConstantVelocity cv(1.0);
  for (double w : {0.0, 1e-6, deg2rad(2.0), deg2rad(-15.0)}) {
    const State x = make_state(500, -300, 120, 80, w);
    EXPECT_TRUE(ct.jacobian(x, 2.5).isApprox(numeric_jacobian(ct, x, 2.5), 1e-5)) << "w = " << w;
    EXPECT_TRUE(cv.jacobian(x, 2.5).isApprox(numeric_jacobian(cv, x, 2.5), 1e-6));
  }
}

TEST(Models, ProcessNoiseIsSymmetricPositiveDefinite) {
  const CoordinatedTurn ct(4.0, 1e-4);
  const StateCov Q = ct.noise(State::Zero(), 2.0);
  EXPECT_TRUE(Q.isApprox(Q.transpose()));
  EXPECT_EQ(Q.llt().info(), Eigen::Success);
}
