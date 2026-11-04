#include "vakt/models.hpp"

#include <cmath>

namespace vakt {

double wrap_pi(double a) {
  a = std::fmod(a + std::numbers::pi, 2.0 * std::numbers::pi);
  if (a <= 0.0) a += 2.0 * std::numbers::pi;
  return a - std::numbers::pi;
}

double wrap_2pi(double a) {
  a = std::fmod(a, 2.0 * std::numbers::pi);
  return a < 0.0 ? a + 2.0 * std::numbers::pi : a;
}

StateCov white_accel_noise(double q, double dt) {
  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  StateCov Q = StateCov::Zero();
  for (int axis = 0; axis < 2; ++axis) {
    const int p = axis;
    const int v = axis + 2;
    Q(p, p) = q * dt3 / 3.0;
    Q(p, v) = q * dt2 / 2.0;
    Q(v, p) = q * dt2 / 2.0;
    Q(v, v) = q * dt;
  }
  return Q;
}

// ---- constant velocity ----

State ConstantVelocity::f(const State& x, double dt) const {
  State out = x;
  out(0) += x(2) * dt;
  out(1) += x(3) * dt;
  return out;
}

StateCov ConstantVelocity::jacobian(const State&, double dt) const {
  StateCov F = StateCov::Identity();
  F(0, 2) = dt;
  F(1, 3) = dt;
  return F;
}

StateCov ConstantVelocity::noise(const State&, double dt) const {
  StateCov Q = white_accel_noise(q_, dt);
  Q(4, 4) = q_turn_ * dt;
  return Q;
}

// ---- coordinated turn ----
//
// With w the turn rate (clockwise positive) the velocity vector rotates by -w*dt
// in the east-north frame. Writing s = sin(w dt), c = cos(w dt):
//   e' = e + (s/w) ve + ((1-c)/w) vn
//   n' = n - ((1-c)/w) ve + (s/w) vn
//   ve' =  c ve + s vn
//   vn' = -s ve + c vn
// For |w| -> 0 the terms are replaced by their Taylor series to avoid 0/0.

namespace {

struct TurnTerms {
  double a;   // sin(wT)/w
  double b;   // (1-cos(wT))/w
  double c;   // cos(wT)
  double s;   // sin(wT)
  double da;  // d a / dw
  double db;  // d b / dw
};

TurnTerms turn_terms(double w, double T) {
  TurnTerms t{};
  const double wT = w * T;
  t.s = std::sin(wT);
  t.c = std::cos(wT);
  if (std::abs(wT) < 1e-4) {
    const double T2 = T * T;
    const double T3 = T2 * T;
    t.a = T - w * w * T3 / 6.0;
    t.b = w * T2 / 2.0;
    t.da = -w * T3 / 3.0;
    t.db = T2 / 2.0 - w * w * T2 * T2 / 8.0;
  } else {
    t.a = t.s / w;
    t.b = (1.0 - t.c) / w;
    t.da = (T * t.c) / w - t.s / (w * w);
    t.db = (T * t.s) / w - (1.0 - t.c) / (w * w);
  }
  return t;
}

}  // namespace

State CoordinatedTurn::f(const State& x, double dt) const {
  const TurnTerms t = turn_terms(x(4), dt);
  const double ve = x(2);
  const double vn = x(3);
  State out;
  out(0) = x(0) + t.a * ve + t.b * vn;
  out(1) = x(1) - t.b * ve + t.a * vn;
  out(2) = t.c * ve + t.s * vn;
  out(3) = -t.s * ve + t.c * vn;
  out(4) = x(4);
  return out;
}

StateCov CoordinatedTurn::jacobian(const State& x, double dt) const {
  const TurnTerms t = turn_terms(x(4), dt);
  const double ve = x(2);
  const double vn = x(3);
  StateCov F = StateCov::Identity();
  F(0, 2) = t.a;
  F(0, 3) = t.b;
  F(1, 2) = -t.b;
  F(1, 3) = t.a;
  F(2, 2) = t.c;
  F(2, 3) = t.s;
  F(3, 2) = -t.s;
  F(3, 3) = t.c;
  // derivatives w.r.t. the turn rate
  F(0, 4) = t.da * ve + t.db * vn;
  F(1, 4) = -t.db * ve + t.da * vn;
  F(2, 4) = dt * (-t.s * ve + t.c * vn);
  F(3, 4) = dt * (-t.c * ve - t.s * vn);
  return F;
}

StateCov CoordinatedTurn::noise(const State&, double dt) const {
  StateCov Q = white_accel_noise(q_, dt);
  Q(4, 4) = q_turn_ * dt;
  return Q;
}

}  // namespace vakt
