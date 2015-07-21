#include <lama_common/angular_point.h>
#include <lama_common/polygon_utils.h>

#include <ros/ros.h> // DEBUG
#include <fstream> // DEBUG
#include <iostream> // DEBUG

namespace lama_common
{

/** Center a polygon about its center of mass.
 *
 */
void centerPolygon(geometry_msgs::Polygon& polygon)
{
  double dx;
  double dy;
  centerPolygon(polygon, dx, dy);
}

/** Center a polygon about its center of mass.
 *
 * @param[in] polygon Polygon to center.
 * @param[out] dx Applied translation in x-direction.
 * @param[out] dy Applied translation in y-direction.
 */
void centerPolygon(geometry_msgs::Polygon& polygon, double& dx, double& dy)
{
  dx = 0.0;
  dy = 0.0;

  if (polygon.points.empty())
  {
    return;
  }

  // Compute the center of the polygon.
  vector<Point32>::const_iterator cpt;
  for (cpt = polygon.points.begin(); cpt != polygon.points.end(); ++cpt)
  {
    dx -= cpt->x;
    dy -= cpt->y;
  }
  dx /= (double)polygon.points.size();
  dy /= (double)polygon.points.size();

  vector<Point32>::iterator pt;
  for (pt = polygon.points.begin(); pt != polygon.points.end(); ++pt)
  {
    pt->x += dx;
    pt->y += dy;
  }
}

/** Return a copy of the polygon centered about its center of mass.
 *
 * @param[in] polygon Input polygon.
 * @return The centered polygon.
 */
geometry_msgs::Polygon centeredPolygon(const geometry_msgs::Polygon& polygon)
{
  geometry_msgs::Polygon centered_polygon = polygon;
  centerPolygon(centered_polygon);
  return centered_polygon;
}

/** Return a copy of the polygon centered about its center of mass.
 *
 * @param[in] polygon Input polygon.
 * @param[out] dx Applied translation in x-direction.
 * @param[out] dy Applied translation in y-direction.
 * @return The centered polygon.
 */
geometry_msgs::Polygon centeredPolygon(const geometry_msgs::Polygon& polygon, double& dx, double& dy)
{
  geometry_msgs::Polygon centered_polygon = polygon;
  centerPolygon(centered_polygon, dx, dy);
  return centered_polygon;
}

/** Return true if the polygon is normalizable.
 *
 * Return true if the angles are growing or decreasing homogeneously (one jump
 * allowed, because of angle ciclicity).
 */
static bool normalizablePolygon(const vector<AngularPoint>& angular_points)
{
  unsigned int count_plus = 0;
  unsigned int count_minus = 0;

  std::ofstream ofs("/tmp/diff.txt", std::ios_base::out); // DEBUG
  for (size_t i = 0; i < angular_points.size(); ++i)
  {
    const double this_angle = angular_points[i].angle;
    const double next_angle = angular_points[(i + 1) % angular_points.size()].angle;
    ofs << this_angle << " " << next_angle << " " << next_angle - this_angle << std::endl; // DEBUG
    if (this_angle == next_angle)
    {
      return false;
    }
    if (this_angle < next_angle)
    {
      count_plus++;
    }
    else
    {
      count_minus++;
    }
    if (count_plus > 1 && count_minus > 1)
    {
      /* DEBUG */
      std::ofstream ofs2("/tmp/normalizable.txt", std::ios_base::out);
      ofs2 << "count_plus: " << count_plus << "; count_minus: " << count_minus << std::endl; // DEBUG
      ofs2.close();
      /* DEBUG */
      return false;
    }
  }
  ofs.close(); // DEBUG
  return true;
}

/** Change point order in place so that point angles are sorted.
 *
 * Angles within [-pi,pi[ will be sorted from smallest to greatest (i.e.
 * counterclock-wise). The considered angle is from the center of
 * mass to the point.
 *
 * The algorithm is not robust against non-simple polygons.
 *
 * @param[in] poly The polygon to sort.
 * @return true if the polygon was normalized, false if it
 *   was not normalizable (in this case, the polygon is left
 *   untouched).
 */
bool normalizePolygon(geometry_msgs::Polygon& poly)
{
  std::vector<AngularPoint> angular_points = toAngularPoints(poly);
  const size_t point_count = poly.points.size();

  /* DEBUG */
  std::ofstream ofs2("/tmp/points_and_angle.txt");
  for (size_t i = 0; i < poly.points.size(); ++i)
  {
    ofs2 << poly.points[i].x << " " << poly.points[i].y << " " << angular_points[i].angle << std::endl; // DEBUG
  }
  ofs2.close();
  /* DEBUG */
  if (!normalizablePolygon(angular_points))
  {
    return false;
  }

  if (!is_sorted(angular_points.begin(), angular_points.end()))
  {
    // TODO: Improve sort performance.
    // Appart from the position of the step from +pi to -pi (CCW) which can be in
    // the middle of the array instead of being absent, angular_points
    // should be sorted. Use a circular_iterator, if this exists.
    std::sort(angular_points.begin(), angular_points.end());

    geometry_msgs::Polygon old_poly = poly;
    for (size_t i = 0; i < point_count; ++i)
    {
      poly.points[i] = old_poly.points[angular_points[i].index];
    }
  }
  return true;
}

/** Return a copy of a Polygon message where point angles are sorted.
 *
 * Angles within [-pi,pi[ will be sorted.
 *
 * @param[in] poly The polygon on which the returned polygon is based.
 * @param[out] normalized true if the polygon was normalized, false if it
 *   was not normalizable.
 * @return A copy of poly with sorted angles.
 */
geometry_msgs::Polygon normalizedPolygon(const geometry_msgs::Polygon& poly, bool& normalized)
{
  geometry_msgs::Polygon new_poly = poly;
  normalized = normalizePolygon(new_poly);
  return new_poly;
}

/** Return true is a polygon has no holes larger than max_frontier_width
 *
 * @return true is a polygon has no holes larger than max_frontier_width.
 */
bool isClosed(const geometry_msgs::Polygon& poly, double max_frontier_width)
{
  const size_t size = poly.points.size();
  const double width2 = max_frontier_width * max_frontier_width;

  for(size_t i = 0; i < size; ++i)
  {
    Point32 a = poly.points[i];
    Point32 b = poly.points[(i + 1) % size];

    const double dx = b.x - a.x;
    const double dy = b.y - a.y;

    if (dx * dx + dy * dy > width2)
    {
      return false;
    }
  }
  return true;
}

} /* namespace lama_common */
