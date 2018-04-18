#ifndef PATH_PLANNING_COST_HPP
#define PATH_PLANNING_COST_HPP

#include <unordered_set>
#include "trajectory.hpp"

namespace planner {

/*
 * Information about future collision with another vehicle
 */
struct Collision {
  // Type of collision, determined based on current location of other vehicle
  // relative to the ego vehicle.
  enum CollisionType {
    FrontImpact,
    RearImpact,
    SideImpact
  };

  Collision(
    CollisionType collisionType,
    double current_delta_s,
    double impact_delta_s,
    int lane_index,
    double target_speed, std::chrono::duration<double> time,
    unsigned long vehicle_id
  ) : collisionType{collisionType},
      current_delta_s{current_delta_s},
      impact_delta_s{impact_delta_s}, lane_index{lane_index},
      target_speed{target_speed}, time{time}, vehicle_id{vehicle_id} {}

  CollisionType collisionType;

  // Distance from ego vehicle to other vehicle at current time (time of prediction)
  double current_delta_s;

  // Distance from ego vehicle to other vehicle at impact time (in future)
  double impact_delta_s;

  // Lane index of other vehicle
  int lane_index;

  // Other vehicle speed
  double target_speed;

  // Time to collision
  std::chrono::duration<double> time;

  // Other vehicle id
  unsigned long vehicle_id;

  friend std::ostream& operator<<(std::ostream &os, const Collision::CollisionType &ct);
  friend std::ostream& operator<<(std::ostream &os, const Collision &c);
};

/*
 * Cost of a combination of trajectory and traffic.
 */
class TrajectoryCost {
public:
  // Maximum possible cost
  static TrajectoryCost Max() { TrajectoryCost tc; tc.collisionCost = 9; return tc; }

protected:
  // Cost of collision, non-zero collision cost prohibits further trajectory consideration
  int collisionCost{0};

  // Cost of collision time, allows to compare two trajectories both leading to collisions
  double collisionTimeCost{0.0};

  // Cost of lane change
  int laneChangeCost{0};

  // Cost of max speed. The higher the speed, the lower the cost.
  double speedCost{0.0};

  // Cost of proximity to other vehicle ahead of the ego vehicle
  double obstacleProximityCost{0.0};

  // Cost of making a maneuver
  double maneuverCost{0.0};

  friend class Simulation;

  /**
   * Compare cost of two trajectories.
   */
  friend bool operator<(const TrajectoryCost &lhs, const TrajectoryCost &rhs);

  friend std::ostream& operator<<(std::ostream &os, const TrajectoryCost &td);
};

/*
 * Trajectory simulation
 */
class Simulation {
public:
  Simulation(std::shared_ptr<Trajectory> trajectory, const Traffic &traffic);

  /*
   * Get trajectory for which the simulation has been performed
   */
  std::shared_ptr<Trajectory> GetTrajectory() const { return trajectory; }

  /*
   * Check if simulated ended up with a collision.
   *
   * @return bool true if trajectory leads to a collision, false otherwise
   */
  bool HasCollision() const { return collision != nullptr; }

  /*
   * Get information about collision.
   *
   * @return Collision reference to a Collision instance or nullptr if trajectory is safe
   */
  const Collision& GetCollision() const { return *collision; }

  /*
   * Get trajectory cost
   */
  const TrajectoryCost& GetCost() const { return cost; }

protected:
  std::unique_ptr<Collision> collision;

  friend std::ostream& operator<<(std::ostream &os, const Simulation &sim);

private:
  std::shared_ptr<Trajectory> trajectory;
  const Traffic &traffic;

  TrajectoryCost cost;

  // Distance in meters to the closest vehicle ahead of the ego vehicle in target lane.
  double obstacleProximity{60};

  // Speed of the closest vehicle ahead of the ego vehicle
  double obstacleSpeed{settings::MaxVelocity};

  /*
   * Get a set of vehicle ids that drive either in source or in target lane with the ego vehicle
   */
  std::unordered_set<unsigned long> GetVehiclesInEgoLanes();

  void DetectCollision();
  void ComputeCost();
  void ComputeObstacleProximity();
};

}

#endif //PATH_PLANNING_COST_HPP
