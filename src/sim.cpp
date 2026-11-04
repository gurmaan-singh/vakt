#include "vakt/sim.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <numbers>
#include <stdexcept>

namespace vakt {

namespace {

double get_or(const nlohmann::json& j, const char* key, double fallback) {
  return j.contains(key) ? j.at(key).get<double>() : fallback;
}

// Integrate one target from its start to time t. Small fixed step; truth only needs to be
// smooth and reproducible, not fast.
bool propagate(const TargetSpec& tg, double t, TruthPoint& out) {
  if (t < tg.t_start || t > tg.t_end) return false;
  constexpr double kStep = 0.05;
  double e = tg.east, n = tg.north, v = tg.speed, h = tg.heading;
  double now = tg.t_start;
  std::size_t leg = 0;
  double leg_left = tg.legs.empty() ? 0.0 : tg.legs[0].duration;
  while (now < t) {
    const double step = std::min(kStep, t - now);
    double turn = 0.0, acc = 0.0;
    if (leg < tg.legs.size()) {
      turn = tg.legs[leg].turn_rate;
      acc = tg.legs[leg].accel;
    }
    // midpoint integration of heading keeps turns closed to well under a metre
    const double hm = h + 0.5 * turn * step;
    const double vm = v + 0.5 * acc * step;
    e += vm * std::sin(hm) * step;
    n += vm * std::cos(hm) * step;
    h += turn * step;
    v = std::max(0.0, v + acc * step);
    now += step;
    if (leg < tg.legs.size()) {
      leg_left -= step;
      if (leg_left <= 1e-9) {
        ++leg;
        if (leg < tg.legs.size()) leg_left = tg.legs[leg].duration;
      }
    }
  }
  out.id = tg.id;
  out.pos = {e, n};
  out.vel = {v * std::sin(h), v * std::cos(h)};
  return true;
}

}  // namespace

Scenario load_scenario(const std::string& path) {
  std::ifstream in(path);
  if (!in) throw std::runtime_error("cannot open scenario " + path);
  const nlohmann::json j = nlohmann::json::parse(in);

  Scenario sc;
  sc.name = j.value("name", std::string("unnamed"));
  sc.description = j.value("description", std::string());
  sc.duration = j.at("duration").get<double>();

  const auto& r = j.at("radar");
  sc.radar.scan_period = get_or(r, "scan_period", 2.0);
  sc.radar.pd = get_or(r, "pd", 0.9);
  sc.radar.clutter_rate = get_or(r, "clutter_per_scan", 5.0);
  sc.radar.min_range = get_or(r, "min_range_m", 1000.0);
  sc.radar.max_range = get_or(r, "max_range_m", 80000.0);
  sc.radar.sensor.sigma_range = get_or(r, "sigma_range_m", 50.0);
  sc.radar.sensor.sigma_azimuth = deg2rad(get_or(r, "sigma_azimuth_deg", 0.25));

  int next_id = 1;
  for (const auto& t : j.at("targets")) {
    TargetSpec tg;
    tg.id = t.value("id", next_id);
    next_id = tg.id + 1;
    tg.label = t.value("label", std::string());
    tg.t_start = get_or(t, "t_start", 0.0);
    tg.t_end = get_or(t, "t_end", 1e18);
    tg.east = t.at("east_m").get<double>();
    tg.north = t.at("north_m").get<double>();
    tg.speed = t.at("speed_mps").get<double>();
    tg.heading = deg2rad(t.at("heading_deg").get<double>());
    if (t.contains("legs")) {
      for (const auto& l : t.at("legs")) {
        Leg leg;
        leg.duration = l.at("duration_s").get<double>();
        leg.turn_rate = deg2rad(get_or(l, "turn_deg_s", 0.0));
        leg.accel = get_or(l, "accel_mps2", 0.0);
        tg.legs.push_back(leg);
      }
    }
    sc.targets.push_back(std::move(tg));
  }
  return sc;
}

std::vector<TruthPoint> truth_at(const Scenario& sc, double t) {
  std::vector<TruthPoint> out;
  for (const auto& tg : sc.targets) {
    TruthPoint p;
    if (propagate(tg, t, p)) out.push_back(p);
  }
  return out;
}

Scan simulate_scan(const Scenario& sc, double t, const std::vector<TruthPoint>& truth, std::mt19937_64& rng) {
  const RadarSpec& rs = sc.radar;
  const PolarSensor& s = rs.sensor;
  std::uniform_real_distribution<double> u01(0.0, 1.0);
  std::normal_distribution<double> n01(0.0, 1.0);

  Scan scan;
  scan.time = t;
  for (const auto& tp : truth) {
    State x = State::Zero();
    x.head<2>() = tp.pos;
    const Meas z = s.h(x);
    if (z(0) < rs.min_range || z(0) > rs.max_range) continue;
    if (u01(rng) > rs.pd) continue;
    Plot p;
    p.range = z(0) + s.sigma_range * n01(rng);
    p.azimuth = wrap_2pi(z(1) + s.sigma_azimuth * n01(rng));
    p.truth_id = tp.id;
    scan.plots.push_back(p);
  }

  std::poisson_distribution<int> n_clutter(rs.clutter_rate);
  const int k = n_clutter(rng);
  const double r0 = rs.min_range * rs.min_range;
  const double r1 = rs.max_range * rs.max_range;
  for (int i = 0; i < k; ++i) {
    Plot p;
    p.range = std::sqrt(r0 + (r1 - r0) * u01(rng));  // uniform in area
    p.azimuth = 2.0 * std::numbers::pi * u01(rng);
    scan.plots.push_back(p);
  }
  // A real plot extractor does not hand out plots grouped by target.
  std::shuffle(scan.plots.begin(), scan.plots.end(), rng);
  return scan;
}

}  // namespace vakt
