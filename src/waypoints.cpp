#include <cmath>
#include <iostream>

#include "waypoints.hpp"
#include "settings.hpp"

namespace planner {

WayPointsConstAcceleration::WayPointsConstAcceleration(
  double current_x, double current_y, double current_speed, double target_speed,
  std::function<double(double)> get_y,
  size_t n_points
) {
  double v = current_speed;
  double x = current_x;
  double y = current_y;

  double delta_x = 0;
  double new_x;
  double new_y = 0;
  double speed;

  for (size_t i = 0; i < n_points; i++) {
    if (fabs(target_speed - v) <= settings::SpeedChangeStep) {
      // If speed is close to target speed, set it to target speed
      v = target_speed;
    } else {
      // Increment or decrement speed value to bring it closer towards target speed
      v += std::copysign(settings::SpeedChangeStep, target_speed - v);
    }

    bool pointComputed = false;
    double k = 0;

    // In order to make sure linear speed doesn't exceed maximum allowed speed (which can happen
    // on curved parts of the road due to the nature of selected computation algorithm)
    // run computation in a loop decreasing next point distance when speeding
    while (!pointComputed) {
      // Compute new x-coordinate
      delta_x = v * settings::WayPointTimeDistance.count();
      new_x = x + delta_x;

      // Invoke callback to get y-coordinate (presumably by using spline)
      new_y = get_y(new_x);
      double delta_y = new_y - y;

      // Compute vehicle speed between two points using (x, y) coordinates
      speed = sqrt(pow(delta_x, 2) + pow(delta_y, 2)) / settings::WayPointTimeDistance.count();

      // If the observed speed is higher than maximum allowed speed, decrease speed by 5% and re-compute
      if (speed > settings::MaxVelocity) {
        if (k == 0) {
          v *= settings::MaxVelocity / speed;
          k = 0.95;
        } else {
          v *= k;
          k *= 0.95;

          // Sanity check. If speed is decreased to a very low value, exit the loop
          if (k < 1e-5) {
            std::cerr << "Unable to compute next waypoint, maximum allowed speed exceeded." << std::endl;
            pointComputed = true;
          }
        }
      } else {
        pointComputed = true;
      }
    }

    x += delta_x;
    y = new_y;

    x_points.push_back(x);
    y_points.push_back(y);
  }
}

}