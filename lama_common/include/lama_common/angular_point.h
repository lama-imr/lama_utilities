#pragma once
#ifndef LAMA_COMMON_ANGULAR_POINT_H
#define LAMA_COMMON_ANGULAR_POINT_H

#include <cmath>
#include <vector>

#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>
#include <lama_msgs/PlaceProfile.h>

namespace lama_common
{

/** A pair (index, angle) constructed from a Point32 with comparison by angle value.
*/
struct AngularPoint
{
  AngularPoint(geometry_msgs::Point32 point, int32_t index_) :
    angle(std::atan2(point.y, point.x)),
    index(index_)
  {
  }

  double angle;
  int32_t index;

  bool operator<(const AngularPoint& other) const {return angle < other.angle;}
};

inline std::vector<AngularPoint> toAngularPoints(const geometry_msgs::Polygon& polygon)
{
  std::vector<AngularPoint> angular_points;
  const size_t point_count = polygon.points.size();
  angular_points.reserve(point_count);
  for (size_t i = 0; i < point_count; ++i)
  {
    angular_points.push_back(AngularPoint(polygon.points[i], i));
  }
  return angular_points;
}

inline std::vector<AngularPoint> toAngularPoints(const lama_msgs::PlaceProfile& profile)
{
  std::vector<AngularPoint> angular_points;
  const size_t point_count = profile.polygon.points.size();
  angular_points.reserve(point_count);
  for (size_t i = 0; i < point_count; ++i)
  {
    angular_points.push_back(AngularPoint(profile.polygon.points[i], i));
  }
  return angular_points;
}

} /* namespace lama_common */

#endif /* LAMA_COMMON_ANGULAR_POINT_H */
