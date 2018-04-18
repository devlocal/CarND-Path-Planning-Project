#ifndef PATH_PLANNING_UTILS_HPP
#define PATH_PLANNING_UTILS_HPP

#include <cmath>
#include <vector>
#include "settings.hpp"

namespace planner {

constexpr double pi() { return M_PI; }

double deg2rad(double x);

double rad2deg(double x);

/*
 * Transform a single point from Cartesian x,y coordinates to Frenet s,d coordinates
 */
std::vector<double> getFrenet(
  double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y
);

/*
 * Transform a list of points from Cartesian x,y coordinates to Frenet s,d coordinates
 */
std::vector<std::vector<double>> getFrenet(const std::vector<double> &x, const std::vector<double> &y,
                                           double theta,
                                           const std::vector<double> &maps_x, const std::vector<double> &maps_y);

/*
 * Transform a single point from Frenet s,d coordinates to Cartesian x,y
 */
std::vector<double> getXY(double s, double d, const std::vector<double> &maps_s, const std::vector<double> &maps_x,
                          const std::vector<double> &maps_y);

/*
 * Transform a point from global coordinates to vehicle coordinates
 */
void to_local(double ref_x, double ref_y, double angle, double *x, double *y);

/*
 * Transform a point from vehicle coordinates to global coordinates
 */
void to_global(double ref_x, double ref_y, double angle, double *x, double *y);

/*
 * Get lane index by d-coordinate.
 *
 * @param relaxed if false requires d-coordinate to be no more than 1m away from lane center.
 *
 * @return -1 if d-coordinate is too far from any lane center, 0-based lane index otherwise
 */
int GetLaneIndex(double car_d, bool relaxed = false);

/*
 * Normalize s-coordinates to handle vehicle positions around the track.
 *
 * When ego vehicle is approaching end of track, add track length to s-positions of vehicles past start line.
 *
 * When ego vehicle is beyond start line, but still close to start, subtract track length from s-positions
 * of vehicles that approach start line.
 */
class SNormalizer {
public:
  explicit SNormalizer(double current_ego_s) :
    afterStart{current_ego_s < afterStartThreshold}, beforeStart{current_ego_s > beforeStartThreshold} {}

  double operator()(double s) const {
    if (afterStart && s > beforeStartThreshold) {
      return s - settings::TrackLength;
    }

    if (beforeStart && s < afterStartThreshold) {
      return s + settings::TrackLength;
    }

    return s;
  }

private:
  static constexpr double afterStartThreshold{settings::TrackLength * 0.1};
  static constexpr double beforeStartThreshold{settings::TrackLength * 0.9};
  bool afterStart;
  bool beforeStart;
};

}

#endif //PATH_PLANNING_UTILS_HPP
