#pragma once

#include "vakt/measurement.hpp"
#include "vakt/models.hpp"

namespace vakt {

struct Innovation {
  Meas nu = Meas::Zero();       // z - h(x_pred)
  MeasCov S = MeasCov::Zero();  // H P H' + R
  MeasJac H = MeasJac::Zero();

  double mahalanobis2() const;
  double log_likelihood() const;
};

Gaussian ekf_predict(const Gaussian& g, const MotionModel& model, double dt);
Innovation ekf_innovation(const Gaussian& pred, const Meas& z, const PolarSensor& sensor);
// Joseph-form update: slower than P - KSK' but keeps P symmetric positive definite
// when the gain is computed from a slightly inconsistent S.
Gaussian ekf_update(const Gaussian& pred, const Innovation& inn, const PolarSensor& sensor);

}  // namespace vakt
