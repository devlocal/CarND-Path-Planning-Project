#include "settings.hpp"
#include <algorithm>

namespace planner { namespace settings {

const size_t NumPointsToPredict{std::max({NumPointsKeepLane, NumPointsChangeLane})};  // NOLINT

const std::vector<double> KeepLaneVelocities{  // NOLINT
  settings::MaxVelocity * 1,
  settings::MaxVelocity * 0.9,
  settings::MaxVelocity * 0.8,
  settings::MaxVelocity * 0.7,
  settings::MaxVelocity * 0.6,
  settings::MaxVelocity * 0.5,
  settings::MaxVelocity * 0.4
};

} }