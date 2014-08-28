#ifndef _LAMA_COMMON_FRONTIER_H_
#define _LAMA_COMMON_FRONTIER_H_

#include <lama_common/point.h>

namespace lama {

/* Frontier is a line segment through which the robot can go
 * p1 First point
 * p2 Second point, so that angle(r-p1, r-p2) is positive, where r is the laser base
 * width Segment length, i.e. width of free space
 * angle Angle between x-axis and line between origin and frontier middle. Where x-axis
 *   and origin refer to the data from which the frontier is computed, in general a LaserScan.
 */
struct Frontier
{
  Frontier(const Point2& p1_, const Point2& p2_, const double width_, const double angle_) :
    p1(p1_),
    p2(p2_),
    width(width_),
    angle(angle_)
  {
  }

  Point2 p1;
  Point2 p2;
  double width;
  double angle;
};

} // namespace lama

#endif // _LAMA_COMMON_FRONTIER_H_

