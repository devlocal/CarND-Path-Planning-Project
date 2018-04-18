#ifndef PATH_PLANNING_STATE_HPP
#define PATH_PLANNING_STATE_HPP

#include <map>
#include <vector>
#include <ostream>

namespace planner {

/*
 * Possible vehicle state
 */
enum class VehicleState {
  Initialize,
  KeepLane,
  ChangeRight,
  ChangeLeft,
};

/*
 * Allowed vehicle state transitions
 */
extern const std::map<VehicleState, std::vector<VehicleState>> SuccessorStates;

std::ostream& operator<<(std::ostream &os, VehicleState state);

}

#endif //PATH_PLANNING_STATE_HPP
