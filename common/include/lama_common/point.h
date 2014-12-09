#ifndef _LAMA_COMMON_POINT_H_
#define _LAMA_COMMON_POINT_H_

#include <cmath>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>

namespace lama_common {

struct Point2
{
  double x;
  double y;

  Point2(): x(0), y(0) {}
  Point2(const double x_, const double y_): x(x_), y(y_) {}
  Point2(const Point2& p) : x(p.x), y(p.y) {}
  Point2(const geometry_msgs::Point& p) : x(p.x), y(p.y) {}
  Point2(const geometry_msgs::Point32& p) : x(p.x), y(p.y) {}

  bool operator==(const Point2 &p)
  {
    return (p.x == x && p.y == y);
  }

  friend std::ostream &operator<<(std::ostream &os, const Point2 &p);
};

template<typename T>
double pointDistanceSquared2(const T &pa, const T &pb)
{
  const double dx = pa.x - pb.x;
  const double dy = pa.y - pb.y;
  return dx * dx + dy * dy;
}

template<typename T>
double pointDistance2(const T &pa, const T &pb)
{
  return std::sqrt(pointDistanceSquared2(pa, pb));
}

} // namespace lama_common

#endif // _LAMA_COMMON_POINT_H_
