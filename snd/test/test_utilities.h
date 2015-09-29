#ifndef SND_LOCAL_PLANNER_TEST_UTILITIES_H
#define SND_LOCAL_PLANNER_TEST_UTILITIES_H

#include <algorithm> // For std::sort.
#include <cmath> // For std::sqrt, atan2.
#include <sstream>
#include <vector>
#include <utility>

#include <snd/utilities.h>
/* #include "../include/snd_local_planner/utilities.h" */

using snd::normalizeAngle;
using snd::sectorToAngle;
using snd::circularIndex;

typedef std::pair<float, float> CartesianPoint;

struct PolarPoint
{
  float r;
  float theta;

  PolarPoint(float x, float y) :
    r(std::sqrt(x * x + y * y)),
    theta(std::atan2(y, x))
  {
  }

  bool operator<(const PolarPoint& other) const {return normalizeAngle(theta) < normalizeAngle(other.theta);}
};

std::vector<CartesianPoint> sstreamToCartesianPoints(std::stringstream& polygon_sstream)
{
  std::vector<CartesianPoint> points;
  while (!polygon_sstream.eof())
  {
    double x;
    double y;
    polygon_sstream >> x >> y;
    points.push_back(CartesianPoint(x, y));
  }

  return points;
}

/** Return the list of polar points corresponding to the list of Cartesian points.
 *
 * No sorting is done.
 *
 */
std::vector<PolarPoint> sstreamToPolarPoints(std::stringstream& polygon_sstream)
{
  std::vector<CartesianPoint> cart_points = sstreamToCartesianPoints(polygon_sstream);

  std::vector<PolarPoint> pol_points;
  pol_points.reserve(cart_points.size());

  std::vector<CartesianPoint>::const_iterator pt;
  for (pt = cart_points.begin(); pt != cart_points.end(); ++pt)
  {
    PolarPoint pol_pt(pt->first, pt->second);
    pol_points.push_back(pol_pt);
  }
  return pol_points;
}

/** 
 * @param[in] polar_points List of points sorted with ascending angle.
 */
float interpolateRay(const std::vector<PolarPoint>& polar_points, double angle)
{
  const size_t num_points = polar_points.size();

  // The index of the polar_polar with angle closest to angle.
  int right_closest_index = 0;
  while ((right_closest_index < (int)num_points) && (angle > polar_points[right_closest_index].theta))
  {
    right_closest_index++;
  }
  right_closest_index--;

  const double angle_at_right = polar_points[circularIndex(right_closest_index, num_points)].theta;
  const double angle_at_left = polar_points[circularIndex(right_closest_index + 1, num_points)].theta;
  double interpolation_factor = 0.0;
  if (std::abs(angle_at_left - angle_at_right) > 1e-6)
  {
    interpolation_factor = normalizeAngle(angle - angle_at_right) / normalizeAngle(angle_at_left - angle_at_right);
  }
  const double range_at_right = polar_points[circularIndex(right_closest_index, num_points)].r;
  const double range_at_left = polar_points[circularIndex(right_closest_index + 1, num_points)].r;
  return interpolation_factor * (range_at_left - range_at_right) + range_at_right;
}

std::vector<float> scanFromStringstream(std::stringstream& polygon_sstream, size_t num_sectors)
{

  std::vector<PolarPoint> polar_points = sstreamToPolarPoints(polygon_sstream);
  std::sort(polar_points.begin(), polar_points.end());

  std::vector<float> scan;
  scan.reserve(polar_points.size());

  for (int i = 0; i < (int)num_sectors; ++i)
  {
    const double theta = sectorToAngle(i, num_sectors);
    scan.push_back(interpolateRay(polar_points, theta));
  }
  return scan;
}

#endif /* SND_LOCAL_PLANNER_TEST_UTILITIES_H */
