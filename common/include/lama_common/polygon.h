#ifndef _LAMA_COMMON_POLYGON_H_
#define _LAMA_COMMON_POLYGON_H_

#include <vector>
#include <numeric>

#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>
#include <sensor_msgs/LaserScan.h>

#include <lama_common/point.h>

namespace lama_common {

using std::vector;
using geometry_msgs::Point32;

// Return the length of a given polygon.
template<typename T> 
double getLength(const vector<T> &pts)
{
  double length = 0;
  for(size_t i = 0; i < pts.size() - 1; i++)
  {
    length += pointDistance2(pts[i], pts[i+1]);
  }
  if (pts.size() > 2)
  {
    length += pointDistance2(pts.front(), pts.back());
  }
  return length;
}

/// for each segment in polygon, return it's length
/// polygon: p0, p1,.., pn
/// result:  l0, l1,.., ln
/// l0 = lenght of (p0, p1), l1 = length(p1,p2), ... , ln = length(pn, p0)
template<typename T>
vector<double> getLengths(const vector<T> &pts)
{
  vector<double> result;
  result.reserve(pts.size());

  for (size_t i = 0; i < pts.size(); i++)
  {
    result.push_back(pointDistance2(pts[i], pts[(i + 1) % pts.size()]));
    //		std::cerr << "pts[" << i << "]: " << "(" << pts[i].x << "," << pts[i].y << ") " << result.back() << " ";
    //		std::cerr << "next is " << "(" << pts[(i+1)%pts.size()].x << " " << pts[(i+1)%pts.size()].y << ")\n";
  }
  return result;
}


/**
 * Resample the given polygon
 *
 * Resample the given polygon of 2D or 3D points. The third coordinate of 3D points is ignored.
 * T must have a constructor T(double, double).
 * numOfSamples > 0
 */
template<typename T>
vector<T> resamplePolygon(const vector<T>& polygon, const int numOfSamples, double& delta)
{
  vector<double> lengths(getLengths(polygon));
  const double polygonLength = std::accumulate(lengths.begin(), lengths.end(), 0.0);
  const double dl = polygonLength / (double)numOfSamples;
  delta = dl;

  vector<T> result;
  result.reserve(numOfSamples);

  int last = 0;
  const int size = polygon.size();
  double alength = 0;
  double tmpl;
  int lnext;
  for (double l = 0; l < polygonLength; l += dl)
  {
    while (last < size && (alength + lengths[last]) < l)
    {
      alength += lengths[last];
      last++;
    }
    if (last == size)
    {
      break;
    }
    lnext = (last == (size - 1)) ? 0 : (last + 1);
    tmpl = l - alength;
    result.push_back(T(
          polygon[last].x * (1 - tmpl / lengths[last]) + polygon[lnext].x * (tmpl / lengths[last]),
          polygon[last].y * (1 - tmpl / lengths[last]) + polygon[lnext].y * (tmpl / lengths[last])));		
  }

  return result;
}

/* Resample the given polygon.
 *
 * Specialization for geometry_msgs::Point32 that doesn't have a constructor T(double, double).
 *
 * numOfSamples > 0
 */
// inline is necessary here because of linker error for multiple definition.
template <>
inline vector<Point32> resamplePolygon<Point32>(const vector<Point32>& polygon, const int numOfSamples, double& delta)
{
  vector<Point2> points;
  points.reserve(polygon.size());
  for (size_t i = 0; i < polygon.size(); ++i)
  {
    points.push_back(Point2(polygon[i]));
  }

  vector<Point2> resampledPoints = resamplePolygon(points, numOfSamples, delta);

  vector<Point32> resampledGPoints;
  vector<Point32>::iterator it_gpoint;
  resampledGPoints.reserve(resampledPoints.size());
  for (size_t i = 0; i < resampledPoints.size(); ++i)
  {
    Point32 gpoint;
    gpoint.x = resampledPoints[i].x;
    gpoint.y = resampledPoints[i].y;
    resampledGPoints.push_back(gpoint);
  }

  return resampledGPoints;
}

/* Return a list of points from a LaserScan.
 *
 * T must have a constructor T(double, double).
 */
template <typename T>
vector<T> scanToPolygon(const sensor_msgs::LaserScan& scan)
{
  vector<T> polygon;
  polygon.reserve(scan.ranges.size());
  double angle = scan.angle_min;
  for (size_t i = 0; i <  scan.ranges.size(); ++i)
  {
    const double r = scan.ranges[i];
    polygon.push_back(T(r * std::cos(angle), r * std::sin(angle)));
    angle += scan.angle_increment;
  }
  return polygon;
}

/* Return a geometry_msgs::Polygon from a LaserScan.
 *
 */
inline geometry_msgs::Polygon scanToPolygon(const sensor_msgs::LaserScan& scan)
{
  geometry_msgs::Polygon polygon;
  polygon.points.reserve(scan.ranges.size());
  double angle = scan.angle_min;
  for (size_t i = 0; i <  scan.ranges.size(); ++i)
  {
    const double r = scan.ranges[i];
    Point32 point;
    point.x = r * std::cos(angle);
    point.y = r * std::sin(angle);
    polygon.points.push_back(point);
    angle += scan.angle_increment;
  }
  return polygon;
}

} // namespace lama_common

#endif // _LAMA_COMMON_POLYGON_H_
