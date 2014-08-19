#ifndef _LAMA_COMMON_POLYGON_H_
#define _LAMA_COMMON_POLYGON_H_

#include <vector>
#include <numeric>

#include <geometry_msgs/Point32.h>
#include <lama_common/point.h>

namespace lama {

// Return the length of a given polygon.
template<typename T> 
double getLength(const std::vector<T> &pts)
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
std::vector<double> getLengths(const std::vector<T> &pts)
{
  std::vector<double> result;
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
 * numOfSamples > 0
 */
template<typename T>
std::vector<T> resamplePolygon(const std::vector<T>& polygon, const int numOfSamples, double& delta)
{
  std::vector<double> lengths(getLengths(polygon));
  const double polygonLength = std::accumulate(lengths.begin(), lengths.end(), 0.0);
  const double dl = polygonLength / (double)numOfSamples;
  delta = dl;

  std::vector<T> result;
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

/**
 * Resample the given polygon
 * numOfSamples > 0
 */
std::vector<geometry_msgs::Point32> resamplePolygon(const std::vector<geometry_msgs::Point32>& polygon, const int numOfSamples, double& delta)
{
  std::vector<lama::Point2> points;
  for (auto gpoint : polygon)
  {
    points.push_back(lama::Point2(gpoint));
  }

  auto resampledPoints = resamplePolygon(points, numOfSamples, delta);

  std::vector<geometry_msgs::Point32> resampledGPoints;
  resampledGPoints.reserve(resampledPoints.size());
  for (auto point : resampledPoints)
  {
    geometry_msgs::Point32 gpoint;
    gpoint.x = point.x;
    gpoint.y = point.y;
    resampledGPoints.push_back(gpoint);
  }

  return resampledGPoints;
}

} // namespace lama

#endif // _LAMA_COMMON_POLYGON_H_
