#include "state.hpp"

namespace planner {

const std::map<VehicleState, std::vector<VehicleState>> SuccessorStates{  // NOLINT
  {VehicleState::Initialize,  {VehicleState::Initialize, VehicleState::KeepLane}},
  {VehicleState::KeepLane,    {VehicleState::KeepLane, VehicleState::ChangeLeft, VehicleState::ChangeRight}},
  {VehicleState::ChangeLeft,  {VehicleState::ChangeLeft, VehicleState::KeepLane}},
  {VehicleState::ChangeRight, {VehicleState::ChangeRight, VehicleState::KeepLane}},
};

std::ostream& operator<<(std::ostream &os, VehicleState state) {
  switch (state) {
    case VehicleState::Initialize:
      os << "Initialize";
      break;

    case VehicleState::KeepLane:
      os << "KeepLane";
      break;

    case VehicleState::ChangeLeft:
      os << "ChangeLeft";
      break;

    case VehicleState::ChangeRight:
      os << "ChangeRight";
      break;
  }

  return os;
}

}