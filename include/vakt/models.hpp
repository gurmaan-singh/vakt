#pragma once

#include "vakt/types.hpp"

#include <memory>
#include <string_view>

namespace vakt {

// Discrete-time motion model x_k+1 = f(x_k, dt) + w,  w ~ N(0, Q(dt)).
class MotionModel {
 public:
  virtual ~MotionModel() = default;
  virtual std::string_view name() const = 0;
  virtual State f(const State& x, double dt) const = 0;
  virtual StateCov jacobian(const State& x, double dt) const = 0;
  virtual StateCov noise(const State& x, double dt) const = 0;
};

// Nearly constant velocity, white-noise acceleration with spectral density q [m^2/s^3].
// The turn-rate component is held (and only slowly diffused) so the IMM can mix it.
class ConstantVelocity final : public MotionModel {
 public:
  explicit ConstantVelocity(double q_accel, double q_turn = 1e-8) : q_(q_accel), q_turn_(q_turn) {}
  std::string_view name() const override { return "CV"; }
  State f(const State& x, double dt) const override;
  StateCov jacobian(const State& x, double dt) const override;
  StateCov noise(const State& x, double dt) const override;

 private:
  double q_;
  double q_turn_;
};

// Coordinated turn with unknown turn rate (Bar-Shalom, Li, Kirubarajan 2001, sec. 11.7).
// Positive turn rate turns clockwise, consistent with azimuth measured clockwise from north.
class CoordinatedTurn final : public MotionModel {
 public:
  CoordinatedTurn(double q_accel, double q_turn) : q_(q_accel), q_turn_(q_turn) {}
  std::string_view name() const override { return "CT"; }
  State f(const State& x, double dt) const override;
  StateCov jacobian(const State& x, double dt) const override;
  StateCov noise(const State& x, double dt) const override;

 private:
  double q_;
  double q_turn_;
};

// Shared process noise for the position/velocity block of both models.
StateCov white_accel_noise(double q, double dt);

}  // namespace vakt
