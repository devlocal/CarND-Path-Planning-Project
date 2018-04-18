#ifndef PATH_PLANNING_TRAJECTORY_HPP
#define PATH_PLANNING_TRAJECTORY_HPP

#include <vector>
#include "state.hpp"
#include "json/json.hpp"
#include "settings.hpp"
#include "traffic.hpp"

using json = nlohmann::json;

namespace planner {

/*
 * Future trajectory of ego vehicle
 */
class Trajectory {
public:
  /*
   * @param next_state next vehicle state
   * @param car_theta car yaw angle
   * @param map_waypoints_x map waypoints (x-coordinate)
   * @param map_waypoints_y map waypoints (y-coordinate)
   * @param traffic predicted positions of other vehicles on the road
   * @param next_x_vals x-coordinates of predicted waypoints
   * @param next_y_vals y-coordinates of predicted waypoints
   * @param current_lane index of current lane
   * @param target_lane index of target lane
   * @param target_speed desired speed
   * @param maneuver_length length of lane change maneuver
   */
  Trajectory(
    VehicleState next_state, double car_theta,
    const std::vector<double> &map_waypoints_x, const std::vector<double> &map_waypoints_y,
    const Traffic &traffic, std::vector<double> &&next_x_vals, std::vector<double> &&next_y_vals,
    int current_lane, int target_lane, double target_speed, double maneuver_length
  );

  /*
   * Get next state of ego vehicle (keep lane, change lane etc.)
   */
  VehicleState GetNextState() const { return nextState; }

  /*
   * Is next state KeepLane?
   */
  bool IsKeepLane() const { return keepLane; }

  /*
   * Is next state ChangeRight?
   */
  bool IsChangeRight() const { return changeRight; }


  /*
   * Get x-coordinates of future waypoints.
   */
  std::vector<double>& GetXVals() { return x_vals; }

  /*
   * Get y-coordinates of future waypoints.
   */
  std::vector<double>& GetYVals() { return y_vals; }

  /*
   * Get s-coordinates of future waypoints.
   */
  std::vector<double>& GetSVals() { return s_vals; }

  /*
   * Get d-coordinates of future waypoints.
   */
  std::vector<double>& GetDVals() { return d_vals; }

  /*
   * Get index of current ego vehicle lane
   */
  int GetCurrentLane() const { return currentLane; }

  /*
   * Get index of target lane of ego vehicle
   */
  int GetTargetLane() const { return targetLane; }

  /*
   * Get length of lane change maneuver, 0 for KeepLane state
   */
  double GetManeuverLength() const { return maneuverLength; }

  /*
   * Get desired speed
   */
  double GetTargetSpeed() const { return targetSpeed; }

protected:
  VehicleState nextState;
  bool keepLane;
  bool changeLeft;
  bool changeRight;

  std::vector<double> x_vals;
  std::vector<double> y_vals;
  std::vector<double> s_vals;
  std::vector<double> d_vals;
  int targetLane;
  int currentLane;
  double maneuverLength;
  double targetSpeed;

  friend std::ostream& operator<<(std::ostream &os, const Trajectory &td);

private:
  const Traffic &traffic;
};

}

#endif //PATH_PLANNING_TRAJECTORY_HPP
