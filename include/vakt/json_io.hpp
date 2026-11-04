#pragma once

#include "vakt/geo.hpp"
#include "vakt/tracker.hpp"

#include <string>
#include <vector>

namespace vakt {

// Track picture as one JSON object per scan, consumed by the lagesbild C2 display:
// {"type":"tracks","source":"vakt","time":..,"radar":{lat,lon},"tracks":[{id,lat,lon,
//   east,north,speed,course,sigma_m,p_turn,age,hits}]}
// Course is degrees clockwise from north, speed in m/s, sigma_m is the 1-sigma
// position error along the worst axis.
std::string tracks_to_json(const std::vector<TrackReport>& tracks, double time, const LocalFrame& frame,
                           const std::string& source = "vakt");

}  // namespace vakt
