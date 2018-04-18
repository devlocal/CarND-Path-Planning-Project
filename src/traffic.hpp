#ifndef PATH_PLANNING_TRAFFIC_HPP
#define PATH_PLANNING_TRAFFIC_HPP

#include <vector>
#include "json/json.hpp"
#include "settings.hpp"
#include "utils.hpp"

using json = nlohmann::json;

namespace planner {

/*
 * Predicted future locations of of an other vehicle
 */
struct VehiclePoints {
  // Vehicle position relative to ego vehicle
  enum Location {
    Front,
    Rear,
    Side
  };

  // Speed of vehicle (assuming constant)
  double linear_speed;

  // d-coordinate (assuming constant)
  double d;

  Location location;

  // s-coordinates of the vehicle travelling with a constant speed
  std::vector<double> s_points;

  friend std::ostream& operator<<(std::ostream &os, const VehiclePoints::Location &loc);
  friend std::ostream& operator<<(std::ostream &os, const VehiclePoints &vp);
};

/*
 * State of other vehicles on the road at the current moment and future predictions of their state.
 * Uncertainty is ignored when predicting waypoints. Constant speed assumed.
 */
class Traffic {
public:
  /*
   * Construct traffic state and predictions using sensor fusion data.
   *
   * @param sensor_fusion sensor fusion output
   * @param ego_s s-coordinate of ego vehicle
   * @param ego_d d-coordinate of ego vehicle
   * @param num_points number of waypoints to predict
   */
  Traffic(const json &sensor_fusion, double ego_s, double ego_d, size_t num_points = settings::NumPointsToPredict);

  /*
   * Get predicted waypoints for all sensed vehicles.
   */
  const std::map<unsigned long, VehiclePoints>& GetVehiclePoints() const { return vehiclePoints; }

  /*
   * Get the closes vehicle in a lane in front of ego vehicle
   *
   * @param lane_index index of lane to check
   * @return VehiclePoints for the closes vehicle or nullptr if there is no vehicle ahead
   */
  const VehiclePoints* GetNextInLane(int lane_index) const {
    assert(lane_index >= 0 && lane_index < settings::NumLanes); return nextInLane[lane_index];
  }

private:
  // Normalizer of s-coordinates
  SNormalizer s_normalizer;

  // Predicted vehicle positions
  std::map<unsigned long, VehiclePoints> vehiclePoints;

  // Identifiers of vehicles in each lane ahead of ego vehicle
  const VehiclePoints *nextInLane[settings::NumLanes]{nullptr};

  void FindNextInLanes(const json &sensor_fusion, double ego_s);
  void PredictWaypoints(const json &vehicle_sensor_data, size_t num_points, double ego_s, double ego_d);
};

}

#endif //PATH_PLANNING_TRAFFIC_HPP
