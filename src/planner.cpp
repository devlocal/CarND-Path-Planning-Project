#include "planner.hpp"

#include <algorithm>
#include <cmath>

#include "spline/spline.h"
#include "utils.hpp"
#include "waypoints.hpp"

namespace planner {

void PathPlanner::Handle(
  double car_x, double car_y, double car_s, double car_d, double car_yaw,
  const std::vector<double> &previous_path_x, const std::vector<double> &previous_path_y,
  const std::vector<double> &map_waypoints_x, const std::vector<double> &map_waypoints_y,
  const std::vector<double> &map_waypoints_s,
  const json &sensor_fusion
) {
  double car_theta = deg2rad(car_yaw);

  VehicleState prev_state = state;

  Traffic traffic(sensor_fusion, car_s, car_d);

  switch (state) {
    case VehicleState::Initialize:
      lane_index = GetLaneIndex(car_d);
      if (lane_index >= 0) {
        state = SuccessorStates.at(state).back();
      }
      break;

    case VehicleState::KeepLane:
      HandleKeepLane(
        car_x, car_y, car_theta, previous_path_x, previous_path_y,
        map_waypoints_x, map_waypoints_y, map_waypoints_s, traffic
      );
      break;

    case VehicleState::ChangeRight:
    case VehicleState::ChangeLeft:
      HandleChangeLane(
        car_x, car_y, car_d, car_theta, previous_path_x, previous_path_y,
        map_waypoints_x, map_waypoints_y, map_waypoints_s, traffic
      );
      break;
  }

  if (settings::DebugOutput && state != prev_state) {
    std::cout << "State changed: " << prev_state << " -> " << state << std::endl;
  }
}

void PathPlanner::HandleKeepLane(
  double car_x, double car_y, double car_theta,
  const std::vector<double> &previous_path_x, const std::vector<double> &previous_path_y,
  const std::vector<double> &map_waypoints_x, const std::vector<double> &map_waypoints_y,
  const std::vector<double> &map_waypoints_s,
  const Traffic &traffic
) {
  std::vector<std::shared_ptr<Trajectory>> trajectories;

  // Generate Keep Lane trajectories with different target velocity
  for (double velocity : settings::KeepLaneVelocities) {
    auto keepLaneTrajectory = GenerateTrajectory(
      VehicleState::KeepLane, car_x, car_y, car_theta,
      previous_path_x, previous_path_y,
      map_waypoints_x, map_waypoints_y, map_waypoints_s,
      traffic, lane_index, velocity, settings::NumPointsKeepLane, 0
    );
    trajectories.push_back(std::move(keepLaneTrajectory));
  }

  // List of velocities for trajectories that change lane
  std::vector<double> ChangeLaneVelocities{};

  size_t num_prev_points = previous_path_x.size();
  if (num_prev_points >= 2) {
    // Compute linear speed and the end of previously computed path
    double x1 = previous_path_x[num_prev_points - 2];
    double x2 = previous_path_x[num_prev_points - 1];
    double y1 = previous_path_y[num_prev_points - 2];
    double y2 = previous_path_y[num_prev_points - 1];
    double speed = sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2)) / settings::WayPointTimeDistance.count();

    // Allow slightly higher and slightly lower speed than the current speed when changing lane
    double v1 = speed * 1.2;
    double v2 = speed * 1.1;
    double v3 = speed * 1;

    if (v1 < settings::MaxVelocity * 0.9) { ChangeLaneVelocities.push_back(v1); }
    if (v2 < settings::MaxVelocity * 0.9) { ChangeLaneVelocities.push_back(v2); }
    if (v3 < settings::MaxVelocity * 0.9) { ChangeLaneVelocities.push_back(v3); }
    ChangeLaneVelocities.push_back(speed * 0.9);
    ChangeLaneVelocities.push_back(speed * 0.8);
  };

  // Generate Change Left trajectories
  if (lane_index > 0) {
    for (double velocity : ChangeLaneVelocities) {
      // Length of change lane maneuver is proportional to the speed, but at least 1/2 of max length
      double maneuverLength = std::max(
        settings::ChangeLaneMaxManeuverLength * 0.5,
        settings::ChangeLaneMaxManeuverLength * velocity / settings::MaxVelocity
      );

      auto changeLeftTrajectory = GenerateTrajectory(
        VehicleState::ChangeLeft, car_x, car_y, car_theta,
        previous_path_x, previous_path_y,
        map_waypoints_x, map_waypoints_y, map_waypoints_s,
        traffic, lane_index - 1, velocity, settings::NumPointsChangeLane, maneuverLength
      );
      trajectories.push_back(std::move(changeLeftTrajectory));
    }
  }

  // Generate Change Right trajectories
  if (lane_index < settings::NumLanes - 1) {
    for (double velocity : ChangeLaneVelocities) {
      double maneuverLength = std::max(
        settings::ChangeLaneMaxManeuverLength * 0.5,
        settings::ChangeLaneMaxManeuverLength * velocity / settings::MaxVelocity
      );

      auto changeRightTrajectory = GenerateTrajectory(
        VehicleState::ChangeRight, car_x, car_y, car_theta,
        previous_path_x, previous_path_y,
        map_waypoints_x, map_waypoints_y, map_waypoints_s,
        traffic, lane_index + 1, velocity, settings::NumPointsChangeLane, maneuverLength
      );
      trajectories.push_back(std::move(changeRightTrajectory));
    }
  }

  // Run simulations
  std::vector<Simulation> simulations = RunSimulations(trajectories, traffic);

  // Choose the best trajectory
  int bestSimulationIndex = FindBestSimulationIndex(simulations);
  std::shared_ptr<Trajectory> bestTrajectory;

  if (bestSimulationIndex >= 0) {
    // Safe trajectory found
    bestTrajectory = simulations[bestSimulationIndex].GetTrajectory();
  } else {
    // All simulations lead to a collision, fall back to Keep Lane trajectory
    // with the speed slightly lower than the speed of the vehicle ahead
    const VehiclePoints *nextInLane = traffic.GetNextInLane(lane_index);
    double velocity = nextInLane ? nextInLane->linear_speed * 0.8 : settings::MaxVelocity;

    // Generate fallback trajectory
    bestTrajectory = GenerateTrajectory(
      VehicleState::KeepLane, car_x, car_y, car_theta,
      previous_path_x, previous_path_y,
      map_waypoints_x, map_waypoints_y, map_waypoints_s,
      traffic, lane_index, velocity, settings::NumPointsKeepLane, 0
    );
  }

  path_x = std::move(bestTrajectory->GetXVals());
  path_y = std::move(bestTrajectory->GetYVals());

  state = bestTrajectory->GetNextState();
  target_lane_index = bestTrajectory->GetTargetLane();
  target_speed = bestTrajectory->GetTargetSpeed();
}

void PathPlanner::HandleChangeLane(
  double car_x, double car_y, double car_d, double car_theta,
  const std::vector<double> &previous_path_x, const std::vector<double> &previous_path_y,
  const std::vector<double> &map_waypoints_x, const std::vector<double> &map_waypoints_y,
  const std::vector<double> &map_waypoints_s,
  const Traffic &traffic
)
{
  // Generate Keep Lane trajectory, keeping target lane
  std::shared_ptr<Trajectory> changeLaneTrajectory = GenerateTrajectory(
    VehicleState::KeepLane, car_x, car_y, car_theta,
    previous_path_x, previous_path_y,
    map_waypoints_x, map_waypoints_y, map_waypoints_s,
    traffic, target_lane_index, target_speed, settings::NumPointsChangeLane,
    0
  );

  path_x = changeLaneTrajectory->GetXVals();
  path_y = changeLaneTrajectory->GetYVals();

  // In the middle of target lane?
  if (GetLaneIndex(car_d) == target_lane_index) {
    state = SuccessorStates.at(state).back();
    lane_index = target_lane_index;
  }
}

double PathPlanner::GetInitialSpeed(
  const std::vector<double> &previous_path_x, const std::vector<double> &previous_path_y, size_t n_points
) {
  if (n_points < 2) {
    // No previous waypoints, speed is 0
    return 0;
  }

  double x1 = previous_path_x[n_points - 1];
  double y1 = previous_path_y[n_points - 1];
  double x2 = previous_path_x[n_points - 2];
  double y2 = previous_path_y[n_points - 2];

  // Compute linear speed from distance between two waypoints and fixed time
  return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2)) / settings::WayPointTimeDistance.count();
}

std::unique_ptr<Trajectory> PathPlanner::GenerateTrajectory(
  VehicleState nextState, double car_x, double car_y, double car_theta,
  const std::vector<double> &previous_path_x, const std::vector<double> &previous_path_y,
  const std::vector<double> &maps_x, const std::vector<double> &maps_y, const std::vector<double> &maps_s,
  const Traffic &traffic, int target_lane, double target_speed, size_t num_points,
  double maneuverLength
) {
  std::vector<double> next_x_vals;
  std::vector<double> next_y_vals;

  std::vector<double> anchor_points_x;
  std::vector<double> anchor_points_y;

  size_t n_prev_points = std::min(previous_path_x.size(), settings::NumPointsToReuse);

  // Store anchor points from the beginning of future path for spline interpolation
  if (n_prev_points < 2) {
    anchor_points_x.push_back(car_x - cos(car_theta));
    anchor_points_x.push_back(car_x);
    anchor_points_y.push_back(car_y - sin(car_theta));
    anchor_points_y.push_back(car_y);
  } else {
    for (size_t i = 2; i >= 1; i--) {
      double x = previous_path_x[n_prev_points - i];
      double y = previous_path_y[n_prev_points - i];

      anchor_points_x.push_back(x);
      anchor_points_y.push_back(y);
    }
  }

  // Rotation point
  double ref_x = anchor_points_x.back();
  double ref_y = anchor_points_y.back();
  double ref_s = getFrenet(ref_x, ref_y, car_theta, maps_x, maps_y)[0];

  // *** Compute anchor points for lane change maneuver part of the path ***

  double num_anchor_points;
  double delta_s;

  if (maneuverLength > 1e-5) {
    num_anchor_points = ceil(target_speed * settings::WayPointTimeDistance.count() / maneuverLength);
    delta_s = maneuverLength / num_anchor_points;
  } else {
    num_anchor_points = 2;
    delta_s = 90.0 / num_anchor_points;
  }

  // Start from index 1, because the first points have been pushed to anchor_points already
  for (int i = 1; i <= std::max(2, static_cast<int>(num_anchor_points)); i++) {
    double distance = delta_s * i;
    double d;

    if (distance >= maneuverLength) {
      d = 4 * target_lane + 2;
    } else {
      d = 4 * lane_index + 4.0 * (target_lane - lane_index) * distance / maneuverLength + 2;
    }

    std::vector<double> anchor_xy = getXY(ref_s + distance, d, maps_s, maps_x, maps_y);

    anchor_points_x.push_back(anchor_xy[0]);
    anchor_points_y.push_back(anchor_xy[1]);
  }

  // Compute rotation angle
  double kAimingDistance = maneuverLength < 1e-5 ? 30 : maneuverLength;  // meters
  std::vector<double> next_xy = getXY(ref_s + kAimingDistance, 4 * target_lane + 2, maps_s, maps_x, maps_y);
  double target_x = next_xy[0];
  double target_y = next_xy[1];
  double target_yaw = atan2(target_y - ref_y, target_x - ref_x);

  // Rotate all anchor points
  for (int i = 0; i < anchor_points_x.size(); i++) {
    to_local(ref_x, ref_y, target_yaw, &anchor_points_x[i], &anchor_points_y[i]);
  }

  tk::spline spline;
  spline.set_points(anchor_points_x, anchor_points_y);

  // Copy existing points
  std::copy(previous_path_x.begin(), previous_path_x.begin() + n_prev_points, std::back_inserter(next_x_vals));
  std::copy(previous_path_y.begin(), previous_path_y.begin() + n_prev_points, std::back_inserter(next_y_vals));

  // *** Compute x, y coordinates for the path at different intervals to implement acceleration/deceleration ***

  double pre_acceleration_x;
  double pre_acceleration_y;
  double pre_acceleration_speed = GetInitialSpeed(next_x_vals, next_y_vals, next_x_vals.size());

  if (next_x_vals.empty()) {
    pre_acceleration_x = 0;
    pre_acceleration_y = 0;
  } else {
    pre_acceleration_x = next_x_vals.back();
    pre_acceleration_y = next_y_vals.back();
    to_local(ref_x, ref_y, target_yaw, &pre_acceleration_x, &pre_acceleration_y);
  }

  // Compute acceleration track
  WayPointsConstAcceleration acc_track(
    pre_acceleration_x, pre_acceleration_y, pre_acceleration_speed, target_speed,
    [&spline](double x){ return spline(x); },
    num_points - next_x_vals.size()
  );

  // Convert all the points to global coordinates
  for (size_t i = 0; i < acc_track.size(); i++) {
    double x = acc_track.GetX(i);
    double y = acc_track.GetY(i);

    to_global(ref_x, ref_y, target_yaw, &x, &y);

    next_x_vals.push_back(x);
    next_y_vals.push_back(y);
  }

  return std::unique_ptr<Trajectory>(new Trajectory(
    nextState, car_theta, maps_x, maps_y, traffic, std::move(next_x_vals), std::move(next_y_vals),
    lane_index, target_lane, target_speed, maneuverLength
  ));
}

std::vector<Simulation> PathPlanner::RunSimulations(
  std::vector<std::shared_ptr<Trajectory>> &trajectories, const Traffic &traffic
) {
  bool collisionsDetected = false;

  std::vector<Simulation> simulations;

  // Run simulations for all trajectories
  for (const std::shared_ptr<Trajectory> &trajectory : trajectories) {
    Simulation simulation(trajectory, traffic);

    if (!simulation.HasCollision()) {
      // Simulation didn't produce a collision, store it
      simulations.push_back(std::move(simulation));
    } else {
      // Simulation ends with a collision, ignore it
      if (settings::DebugOutput) {
        if (collisionsDetected) {
          std::cout << " ";
        }
        std::cout << *simulation.GetTrajectory() << "-" << simulation.GetCollision().vehicle_id;
      }
      collisionsDetected = true;
    }
  }

  if (settings::DebugOutput) {
    if (trajectories.size() != simulations.size()) {
      std::cout << " | ";
    }
  }

  return simulations;
}

int PathPlanner::FindBestSimulationIndex(const std::vector<Simulation> &simulations) const
{
  // Select best trajectory
  TrajectoryCost bestTrajectoryCost{TrajectoryCost::Max()};
  int bestSimulationIndex = -1;

  for (int i = 0; i < simulations.size(); i++) {
    const Simulation &simulation = simulations[i];

    const TrajectoryCost &cost = simulation.GetCost();

    if (settings::DebugOutput) {
      if (i > 0) { std::cout << " | "; }
      std::cout << *simulation.GetTrajectory() << " " << simulation;
    }

    if (cost < bestTrajectoryCost) {
      bestTrajectoryCost = cost;
      bestSimulationIndex = i;

      if (settings::DebugOutput) { std::cout << "+"; }
    } else {
      if (settings::DebugOutput) { std::cout << "-"; }
    }
  }

  if (settings::DebugOutput) { std::cout << " BSI=" << bestSimulationIndex << std::endl; }

  return bestSimulationIndex;
}

}