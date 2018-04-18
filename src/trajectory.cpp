#include "trajectory.hpp"
#include "utils.hpp"

namespace planner {

Trajectory::Trajectory(
  VehicleState next_state, double car_theta,
  const std::vector<double> &map_waypoints_x, const std::vector<double> &map_waypoints_y,
  const Traffic &traffic, std::vector<double> &&next_x_vals, std::vector<double> &&next_y_vals,
  int current_lane, int target_lane, double target_speed, double maneuver_length
) : nextState{next_state}, traffic{traffic}, x_vals{std::move(next_x_vals)}, y_vals{std::move(next_y_vals)},
    currentLane{current_lane}, targetLane{target_lane}, targetSpeed{target_speed},
    keepLane{target_lane == current_lane}, changeLeft{target_lane < current_lane}, changeRight{target_lane > current_lane},
    maneuverLength{maneuver_length}
{
  // Covert (x, y) coordinates to (s, d) coordinates
  auto frenet = getFrenet(x_vals, y_vals, car_theta, map_waypoints_x, map_waypoints_y);

  for (const auto &sd : frenet) {
    s_vals.push_back(sd[0]);
    d_vals.push_back(sd[1]);
  }
}

std::ostream& operator<<(std::ostream &os, const Trajectory &trajectory) {
  switch (trajectory.nextState) {
    case VehicleState::Initialize: os << "I"; break;
    case VehicleState::KeepLane: os << "KL"; break;
    case VehicleState::ChangeLeft: os << "CL"; break;
    case VehicleState::ChangeRight: os << "CR"; break;
  }

  os << "-" << static_cast<int>(trajectory.GetTargetSpeed());

  return os;
}

}