#include "cost.hpp"

namespace planner {

std::ostream& operator<<(std::ostream &os, const Collision::CollisionType &ct) {
  switch (ct) {
    case Collision::FrontImpact:
      std::cout << "FrontImpact";
      break;
    case Collision::RearImpact:
      std::cout << "RearImpact";
      break;
    case Collision::SideImpact:
      std::cout << "SideImpact";
      break;
  }
  return os;
}

std::ostream& operator<<(std::ostream &os, const Collision &c) {
  std::cout
    << c.collisionType
    << ", cds=" << c.current_delta_s
    << ", ids=" << c.impact_delta_s
    << ", l=" << c.lane_index
    << ", ts=" << c.target_speed
    << ", time=" << c.time.count()
    << ", vid=" << c.vehicle_id;
  return os;
}

Simulation::Simulation(std::shared_ptr<Trajectory> trajectory, const Traffic &traffic)
  : trajectory{std::move(trajectory)}, traffic{traffic} {
  DetectCollision();
  ComputeObstacleProximity();

  ComputeCost();
}

std::unordered_set<unsigned long> Simulation::GetVehiclesInEgoLanes() {
  std::unordered_set<unsigned long> result;
  // Get lanes where ego vehicle drives
  int low_lane_index = std::min(trajectory->GetCurrentLane(), trajectory->GetTargetLane());
  int high_lane_index = std::max(trajectory->GetCurrentLane(), trajectory->GetTargetLane());

  // Find vehicles located within interval from source to target lanes
  for (const std::pair<unsigned long, VehiclePoints> pair: traffic.GetVehiclePoints()) {
    const VehiclePoints &points = pair.second;
    int future_lane = GetLaneIndex(points.d);

    // Ignore vehicles not between source and target lanes
    if (future_lane >= low_lane_index && future_lane <= high_lane_index) {
      result.emplace(pair.first);
    }
  }

  return result;
};

void Simulation::DetectCollision() {
  assert(trajectory->GetSVals().size() == trajectory->GetDVals().size());
  assert(trajectory->GetSVals().size() <= settings::NumPointsToPredict);

  std::unordered_set<unsigned long> vehiclesInEgoLanes = GetVehiclesInEgoLanes();

  // Iterate over all trajectory points, check predicted positions of other vehicles
  for (size_t i = 0; i < trajectory->GetSVals().size(); i++) {
    // Future position of ego vehicle
    double future_ego_s = trajectory->GetSVals()[i];

    for (const std::pair<unsigned long, const VehiclePoints> pair: traffic.GetVehiclePoints()) {
      if (vehiclesInEgoLanes.find(pair.first) == vehiclesInEgoLanes.end()) {
        // Vehicle not in source lane or target lane
        continue;
      }

      const VehiclePoints &points = pair.second;

      int future_lane = GetLaneIndex(points.d);  // Lane of other vehicle
      double future_s = points.s_points[i];  // Future s-coordinate of other vehicle
      double initial_linear_speed = points.linear_speed;  // Speed of other vehicle (assuming constant)

      // If other vehicle is in current lane, does ego vehicle have safe distance to it?
      bool safeInCurrentLane =
        future_lane != trajectory->GetCurrentLane() ||
        future_s > future_ego_s + settings::SafeKeepLaneDistanceAhead ||  // Front collision
        future_s < future_ego_s - settings::SafeKeepLaneDistanceBehind;  // Rear collision

      // If other vehicle is in target lane, does ego vehicle have safe distance to it?
      bool safeLaneChange =
        trajectory->IsKeepLane() || future_lane == trajectory->GetCurrentLane() ||
        future_s > future_ego_s + settings::SafeTargetLaneDistanceAhead ||
        future_s < future_ego_s - settings::SafeTargetLaneDistanceBehind;

      if (safeInCurrentLane && safeLaneChange) {
        continue;
      }

      // Ego vehicle collides with the other vehicle in future

      Collision::CollisionType collisionType;
      if (points.location == VehiclePoints::Side) {
        if (trajectory->IsKeepLane()) {
          // Ignore side impacts when keeping lane
          continue;
        }
        collisionType = Collision::SideImpact;
      } else if (points.location == VehiclePoints::Front) {
        collisionType = Collision::FrontImpact;
      } else {
        if (trajectory->IsKeepLane()) {
          // Ignore rear impacts when keeping lane (can be a false positive during acceleration)
          continue;
        }
        collisionType = Collision::RearImpact;
      }

      std::chrono::duration<double> collision_time = i * settings::WayPointTimeDistance;

      if (collision == nullptr || collision->time > collision_time) {
        collision = std::unique_ptr<Collision>(new Collision(
          collisionType,
          points.s_points[0] - trajectory->GetSVals()[0],
          future_s - future_ego_s,
          future_lane,
          initial_linear_speed,
          collision_time,
          pair.first
        ));
      }

      break;
    }
  }
}

void Simulation::ComputeObstacleProximity() {
  const VehiclePoints *nextInLane = traffic.GetNextInLane(trajectory->GetTargetLane());
  // Is there a vehicle in the current lane ahead of ego vehicle?
  if (nextInLane) {
    obstacleProximity = nextInLane->s_points[0] - trajectory->GetSVals()[0];  // How far is the other vehicle
    obstacleSpeed = nextInLane->linear_speed;  // How fast the other vehicle drives
  }
}

void Simulation::ComputeCost() {
  cost.laneChangeCost = abs(trajectory->GetTargetLane() - trajectory->GetCurrentLane());
  if (trajectory->IsChangeRight()) {
    // Give higher cost to changeRight over changeLeft to prefer left passing
    cost.laneChangeCost *= 2;
  }

  double target_speed = trajectory->GetTargetSpeed();
  if (target_speed == 0) {
    cost.speedCost = 1;
  } else {
    cost.speedCost = std::min(1.0, 1.0 / target_speed);
  }

  // Penalize coming too close to the vehicle in front
  if (trajectory->IsKeepLane() && obstacleProximity > 0 && obstacleProximity < settings::SafeDistance &&
      trajectory->GetTargetSpeed() >= obstacleSpeed) {
    cost.speedCost *= 2;
  }

  if (obstacleProximity) {
    cost.obstacleProximityCost = 1.0 / obstacleProximity;
  }

  if (trajectory->GetManeuverLength() < 1e-10) {
    // No maneuver - the lowest cost
    cost.maneuverCost = 0;
  } else {
    // Longer maneuver - lower cost
    cost.maneuverCost = std::min(1.0, 1.0 / trajectory->GetManeuverLength());
  }

  // Non-zero cost of collision allows to compare unsafe trajectories.
  // Currently any trajectory with non-zero collision cost is rejected.
  if (collision) {
    switch (collision->collisionType) {
      case Collision::FrontImpact:
        cost.collisionCost = 1;
        break;

      case Collision::RearImpact:
        cost.collisionCost = 2;
        break;

      case Collision::SideImpact:
        cost.collisionCost = 3;
        break;
    }

    if (collision->time.count() == 0) {
      cost.collisionTimeCost = 1;
    } else {
      cost.collisionTimeCost = std::min(1.0, settings::WayPointTimeDistance / collision->time);
    }
  }
}

bool operator<(const TrajectoryCost &lhs, const TrajectoryCost &rhs) {
  // If only one trajectory leads to a collision, pick the safe one
  if (lhs.collisionCost < rhs.collisionCost) {
    return true;
  } else if (lhs.collisionCost > rhs.collisionCost) {
    return false;
  }

  if (lhs.collisionCost) {
    // If both trajectories lead to a collision, pick the one with the most distant collision point
    return lhs.collisionTimeCost < rhs.collisionTimeCost;
  }

  // Prefer trajectory that allows to drive faster
  if (lhs.speedCost < rhs.speedCost) {
    return true;
  } else if (lhs.speedCost > rhs.speedCost) {
    return false;
  }

  // Prefer trajectory with more distant vehicle in front
  double lhs_proximity_cost = lhs.obstacleProximityCost * (lhs.laneChangeCost == 0 ? 0.5 : 1);
  double rhs_proximity_cost = rhs.obstacleProximityCost * (rhs.laneChangeCost == 0 ? 0.5 : 1);
  if (lhs_proximity_cost < rhs_proximity_cost) {
    return true;
  } else if (lhs_proximity_cost > rhs_proximity_cost) {
    return false;
  }

  // Prefer trajectory with a shorter or no maneuver
  if (lhs.maneuverCost < rhs.maneuverCost) {
    return true;
  } else if (lhs.maneuverCost > rhs.maneuverCost) {
    return false;
  }

  return lhs.laneChangeCost < rhs.laneChangeCost;
}


std::ostream& operator<<(std::ostream &os, const TrajectoryCost &trajectory) {
  os << std::fixed << std::setprecision(2);

  os << "<c" << trajectory.collisionCost;
  os << " s" << trajectory.speedCost * 100.0;
  os << " p" << trajectory.obstacleProximityCost * 100.0;
  os << " m" << trajectory.maneuverCost * 100.0;
  os << " l" << trajectory.laneChangeCost;

  std::cout << ">";

  return os;
}

std::ostream& operator<<(std::ostream &os, const Simulation &sim) {
  os << sim.cost;
  if (sim.collision != nullptr) {
    os << " vid=" << sim.collision->vehicle_id;
  }
  return os;
}

}