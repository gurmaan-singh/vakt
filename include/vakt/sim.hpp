#pragma once

#include "vakt/measurement.hpp"

#include <random>
#include <string>
#include <vector>

namespace vakt {

// A piece of a target's flight: hold turn rate and tangential acceleration for `duration`.
struct Leg {
  double duration = 0.0;    // s
  double turn_rate = 0.0;   // rad/s, clockwise positive
  double accel = 0.0;       // m/s^2 along the velocity
};

struct TargetSpec {
  int id = 0;
  std::string label;
  double t_start = 0.0;
  double t_end = 1e18;     // target disappears (lands, leaves coverage...) after this
  double east = 0.0, north = 0.0;
  double speed = 0.0;      // m/s
  double heading = 0.0;    // rad, clockwise from north
  std::vector<Leg> legs;   // after the last leg the target flies straight
};

struct RadarSpec {
  PolarSensor sensor;
  double scan_period = 2.0;    // s
  double pd = 0.9;             // detection probability
  double clutter_rate = 5.0;   // mean false alarms per scan (Poisson)
  double min_range = 1000.0;   // m
  double max_range = 80000.0;  // m
};

struct Scenario {
  std::string name;
  std::string description;
  double duration = 0.0;
  RadarSpec radar;
  std::vector<TargetSpec> targets;
};

struct TruthPoint {
  int id = 0;
  Eigen::Vector2d pos = Eigen::Vector2d::Zero();
  Eigen::Vector2d vel = Eigen::Vector2d::Zero();
};

Scenario load_scenario(const std::string& path);

// Ground truth for every target alive at time t (deterministic, integrated at 20 Hz).
std::vector<TruthPoint> truth_at(const Scenario& sc, double t);

// One radar scan: detections with probability pd inside coverage, Gaussian polar noise,
// plus Poisson clutter uniform over the covered annulus (by area, not by range).
Scan simulate_scan(const Scenario& sc, double t, const std::vector<TruthPoint>& truth, std::mt19937_64& rng);

}  // namespace vakt
