#ifndef PATH_PLANNING_PLANNER_HPP
#define PATH_PLANNING_PLANNER_HPP

#include <iostream>
#include <vector>
#include <algorithm>
#include "json/json.hpp"
#include "state.hpp"
#include "settings.hpp"
#include "trajectory.hpp"
#include "traffic.hpp"
#include "cost.hpp"

using json = nlohmann::json;

namespace planner {

/*
 * Path planner, predicts trajectories and picks one with the lowest cost
 */
class PathPlanner {
public:
  /*
   * @param car_x current x coordinate of ego vehicle
   * @param car_y current y coordinate of ego vehicle
   * @param car_s current s coordinate of ego vehicle
   * @param car_d current d coordinate of ego vehicle
   * @param car_yaw current vehicle yaw angle
   * @param previous_path_x x-coordinates of path computed on previous iteration
   * @param previous_path_y y-coordinates of path computed on previous iteration
   * @param map_waypoints_x map data, x-coordinates
   * @param map_waypoints_y map data, y-coordinates
   * @param map_waypoints_s map data, s-coordinates
   * @param sensor_fusion sensor fusion output, positions and speed of other vehicles on the road
   */
  void Handle(
    double car_x, double car_y, double car_s, double car_d, double car_yaw,
    const std::vector<double> &previous_path_x, const std::vector<double> &previous_path_y,
    const std::vector<double> &map_waypoints_x, const std::vector<double> &map_waypoints_y,
    const std::vector<double> &map_waypoints_s,
    const json &sensor_fusion
  );

  /*
   * Get x-coordinates of computed path
   */
  std::vector<double>&& GetPathX() noexcept { return std::move(path_x); }

  /*
   * Get y-coordinates of computed path
   */
  std::vector<double>&& GetPathY() noexcept { return std::move(path_y); }

private:
  // Current vehicle state
  VehicleState state{VehicleState::Initialize};

  // Index of current vehicle lane, 0 - leftmost lane
  int lane_index;

  // Index of target lane for lane change maneuver
  int target_lane_index;

  // Speed to maintain
  double target_speed;

  // Computed path, x-coordinate
  std::vector<double> path_x;

  // Computed path, y-coordinate
  std::vector<double> path_y;

  /*
   * Handle KeepLane state
   */
  void HandleKeepLane(
    double car_x, double car_y, double car_theta,
    const std::vector<double> &previous_path_x, const std::vector<double> &previous_path_y,
    const std::vector<double> &map_waypoints_x, const std::vector<double> &map_waypoints_y,
    const std::vector<double> &map_waypoints_s,
    const Traffic &traffic
  );

  /*
   * Handle ChangeLane state
   */
  void HandleChangeLane(
    double car_x, double car_y, double car_d, double car_theta,
    const std::vector<double> &previous_path_x, const std::vector<double> &previous_path_y,
    const std::vector<double> &map_waypoints_x, const std::vector<double> &map_waypoints_y,
    const std::vector<double> &map_waypoints_s,
    const Traffic &traffic
  );

  /*
   * Get vehicle speed at position specified by n_points
   */
  double GetInitialSpeed(
    const std::vector<double> &previous_path_x, const std::vector<double> &previous_path_y, size_t n_points
  );

  /**
   * Generate a new trajectory
   *
   * @param nextState next vehicle state
   * @param car_x current vehicle position, x-coordinate
   * @param car_y current vehicle position, y-coordinate
   * @param car_theta current vehicle yaw angle
   * @param previous_path_x previously computed path, x-coordinates
   * @param previous_path_y previously computed path, y-coordinates
   * @param maps_x map data, x-coordinates
   * @param maps_y map data, y-coordinates
   * @param maps_s map data, s-coordinates
   * @param traffic traffic predictions
   * @param target_lane target lane index
   * @param target_speed desired speed
   * @param num_points number of waypoints to compute
   * @param maneuverLength length of change lane maneuver, meters
   * @return pointer to Trajectory instance for a newly generated trajectory
   */
  std::unique_ptr<Trajectory> GenerateTrajectory(
    VehicleState nextState, double car_x, double car_y, double car_theta,
    const std::vector<double> &previous_path_x, const std::vector<double> &previous_path_y,
    const std::vector<double> &maps_x, const std::vector<double> &maps_y, const std::vector<double> &maps_s,
    const Traffic &traffic, int target_lane, double target_speed, size_t num_points,
    double maneuverLength
  );

  /*
   * Run simulations, return only simulations without collisions
   */
  std::vector<Simulation> RunSimulations(
    std::vector<std::shared_ptr<Trajectory>> &trajectories, const Traffic &traffic
  );

  /*
   * Find simulation with the lowest cost
   */
  int FindBestSimulationIndex(const std::vector<Simulation> &simulations) const;
};

}

#endif //PATH_PLANNING_PLANNER_HPP
