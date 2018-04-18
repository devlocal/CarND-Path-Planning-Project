#include "traffic.hpp"
#include "utils.hpp"

namespace planner {

Traffic::Traffic(const json &sensor_fusion, double ego_s, double ego_d, size_t num_points) : s_normalizer{ego_s} {
  for (const auto &data : sensor_fusion) {
    PredictWaypoints(data, num_points, ego_s, ego_d);
  }

  FindNextInLanes(sensor_fusion, ego_s);
}

void Traffic::PredictWaypoints(const json &vehicle_sensor_data, size_t num_points, double ego_s, double ego_d) {
  // JSON data: [id, x, y, vx, vy, s, d]
  unsigned long id = vehicle_sensor_data[0];
  double vx = vehicle_sensor_data[3];
  double vy = vehicle_sensor_data[4];
  double current_s = s_normalizer(vehicle_sensor_data[5]);
  double current_d = vehicle_sensor_data[6];

  // Combine speed along x and y axis to get linear speed
  double linear_speed = sqrt(pow(vx, 2) + pow(vy, 2));

  double distance_s = current_s - ego_s;
  double distance_d = fabs(current_d - ego_d);

  VehiclePoints points;

  points.linear_speed = linear_speed;
  points.d = current_d;

  // Detect vehicle position relative to ego vehicle
  if (distance_d > fabs(distance_s)) {
    points.location = VehiclePoints::Side;
  } else if (distance_s > 0) {
    points.location = VehiclePoints::Front;
  } else {
    points.location = VehiclePoints::Rear;
  }

  for (size_t i = 0; i < num_points; i++) {
    std::chrono::duration<double> t = i * settings::WayPointTimeDistance;

    // Use linear_speed to predict s-coordinate
    double future_s = current_s + linear_speed * t.count();

    points.s_points.push_back(future_s);
  }

  vehiclePoints[id] = std::move(points);
}

void Traffic::FindNextInLanes(const json &sensor_fusion, double ego_s) {
  for (const auto &pair: vehiclePoints) {
    const VehiclePoints &points = pair.second;

    int lane_index = GetLaneIndex(points.d, true);
    double s = points.s_points[0];

    // Ignore vehicles more than 300 meters away from ego vehicle, there's too much uncertainty
    if (lane_index < 0 || lane_index >= settings::NumLanes || fabs(s - ego_s) > 300.0) {
      continue;
    }

    if (s > ego_s && (!nextInLane[lane_index] || nextInLane[lane_index]->s_points[0] > s)) {
      nextInLane[lane_index] = &points;
    }
  }
}

std::ostream& operator<<(std::ostream &os, const VehiclePoints::Location &loc) {
  switch(loc) {
    case VehiclePoints::Front:
      std::cout << "Front";
      break;
    case VehiclePoints::Rear:
      std::cout << "Rear";
      break;
    case VehiclePoints::Side:
      std::cout << "Side";
      break;
  }
  return os;
}

std::ostream& operator<<(std::ostream &os, const VehiclePoints &vp) {
  os << vp.location << ", d=" << vp.d << ", sp=" << vp.linear_speed << std::endl;
  for (size_t i = 0; i < vp.s_points.size(); i++) {
    if (i > 0) {
      std::cout << std::endl;
    }
    std::cout << "s[" << i << "]=" << vp.s_points[i];
  }
  return os;
}

}
