#include "vakt/json_io.hpp"

#include <nlohmann/json.hpp>

#include <cmath>

namespace vakt {

std::string tracks_to_json(const std::vector<TrackReport>& tracks, double time, const LocalFrame& frame,
                           const std::string& source) {
  nlohmann::json j;
  j["type"] = "tracks";
  j["source"] = source;
  j["time"] = time;
  j["radar"] = {{"lat", frame.origin().lat_deg}, {"lon", frame.origin().lon_deg}};
  auto& arr = j["tracks"] = nlohmann::json::array();
  for (const auto& t : tracks) {
    const Geodetic g = frame.to_geodetic_2d(t.x(0), t.x(1), 0.0);
    const Eigen::Matrix2d Ppos = t.P.topLeftCorner<2, 2>();
    const double sigma = std::sqrt(Ppos.selfadjointView<Eigen::Upper>().eigenvalues().maxCoeff());
    const double speed = std::hypot(t.x(2), t.x(3));
    arr.push_back({{"id", t.id},
                   {"lat", g.lat_deg},
                   {"lon", g.lon_deg},
                   {"east", std::round(t.x(0))},
                   {"north", std::round(t.x(1))},
                   {"speed", std::round(speed * 10.0) / 10.0},
                   {"course", std::round(rad2deg(wrap_2pi(std::atan2(t.x(2), t.x(3)))) * 10.0) / 10.0},
                   {"turn_dps", std::round(rad2deg(t.x(4)) * 100.0) / 100.0},
                   {"sigma_m", std::round(sigma)},
                   {"p_turn", std::round(t.p_turn * 1000.0) / 1000.0},
                   {"age", std::round(t.age)},
                   {"hits", t.hits}});
  }
  return j.dump();
}

}  // namespace vakt
