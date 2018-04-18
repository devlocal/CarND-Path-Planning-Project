#ifndef PATH_PLANNING_SETTINGS_HPP
#define PATH_PLANNING_SETTINGS_HPP

#include <vector>
#include <chrono>
#include <array>

namespace planner {  namespace settings {

// Maximum allowed vehicle speed, miles per hour
constexpr double MaxVelocityMph{48};

// Maximum allowed vehicle speed, meters per second
constexpr double MaxVelocity{MaxVelocityMph * 1600.0 / 3600.0};

// List of allowed velocities for trajectory that continue on the current lane
extern const std::vector<double> KeepLaneVelocities;

// Time interval between two adjacent waypoints
constexpr std::chrono::duration<double> WayPointTimeDistance{std::chrono::milliseconds{20}};  // NOLINT

// Lane width, meters
constexpr double LaneWidth{4.0};

// Number of lanes in a single direction
constexpr int NumLanes{3};

// Number of waypoints points to compute for change lane state
constexpr size_t NumPointsChangeLane{150};

// Number of waypoints to analyze for keep lane state
constexpr size_t NumPointsKeepLane{100};

// Number of waypoints to report to simulator
constexpr size_t NumPointsToReport{100};

// Number of waypoints to reuse from the previous iteration
constexpr size_t NumPointsToReuse{50};

// Number of waypoints to predict for each sensed vehicle
extern const size_t NumPointsToPredict;

// Length of lane change maneuver at maximum speed
constexpr double ChangeLaneMaxManeuverLength{100};

// Maximum allowed distance to a vehicle in front of ego vehicle before it is considered a collision
constexpr double SafeKeepLaneDistanceAhead{7};

// Maximum allowed distance to a vehicle behind ego vehicle before it is considered a collision
constexpr double SafeKeepLaneDistanceBehind{7};

// Vehicle acceleration or deceleration
constexpr double Acceleration{6};

// The value of increment or decrement in speed between two adjacent waypoints when accelerating or decelerating
constexpr double SpeedChangeStep{Acceleration * WayPointTimeDistance.count()};  // NOLINT

// Track length, meters
constexpr double TrackLength{6945.554};

// Safe distance to keep to a vehicle ahead of ego vehicle
constexpr double SafeDistance{25};

// Distance to keep to a vehicle ahead of the ego vehicle at any point while changing lane
constexpr double SafeTargetLaneDistanceAhead{20};

// Distance to keep to a vehicle behind the ego vehicle at any point while changing lane
constexpr double SafeTargetLaneDistanceBehind{15};

// Whether to print debug output to console (when true) or not
constexpr bool DebugOutput{false};

} }

#endif //PATH_PLANNING_SETTINGS_HPP
