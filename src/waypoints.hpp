#ifndef WAYPOINTS_HPP
#define WAYPOINTS_HPP

#include <cassert>
#include <vector>
#include <functional>
#include <chrono>

namespace planner {

/*
 * Computes (x, y) waypoint coordinates spaced by 20 milliseconds using uniform acceleration.
 */
class WayPointsConstAcceleration {
public:
  /*
   * Construct a series of waypoints evenly distributed in time with a fixed interval.
   * Distance between points changes with a fixed acceleration to reach target_speed.
   *
   * @param current_x current x-coordinate of the vehicle
   * @param current_y current y-coordinate of the vehicle
   * @param current_speed current vehicle speed
   * @param target_speed speed that vehicle should reach
   * @param get_y callback function to compute y-coordinate
   * @param n_points number of points to generate
   */
  WayPointsConstAcceleration(
    double current_x, double current_y, double current_speed, double target_speed,
    std::function<double(double)> get_y,
    size_t n_points
  );

  /*
   * Get number of generated points.
   *
   * @return number of points stored in the class
   */
  size_t size() const { assert(x_points.size() == y_points.size()); return x_points.size(); }

  /*
   * Get x-coordinate of a point by point index
   */
  double GetX(size_t n) const { return x_points[n]; }

  /*
   * Get y-coordinate of a point by point index
   */
  double GetY(size_t n) const { return y_points[n]; }

private:
  // Generated waypoints
  std::vector<double> x_points;
  std::vector<double> y_points;
};

}
#endif //WAYPOINTS_HPP
