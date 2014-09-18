#include <crossing_detector/crossing_detector.h>

#define DEBUG_CROSSDETECT

#ifdef DEBUG_CROSSDETECT
#include <fstream>
#include <cassert>
#include <iostream>
#endif

namespace lama {
namespace crossing_detector {

#ifdef DEBUG_CROSSDETECT
/* Save a list of points into a file (comma separation).
 */
template <typename T>
void points_output(const char filename[30], const std::vector<T> points)
{
  ROS_INFO("%s: %zu points", filename, points.size());
	//std::ofstream ofs(filename);
	for (size_t i = 0; i < points.size(); ++i)
	{
    ROS_INFO("%zu: %f %f", i, points[i].x, points[i].y);
		//ofs << points[i].x << " " << points[i].y << "\n";
    std::cout << points[i].x << " " << points[i].y << std::endl;

		//ofs << 5235 << " " << 534 << " " << i << "\n";
	}
	//ofs.close();
  std::cout << std::flush;
}

/* Save a list of points into a file (comma-separated x1, y1, x2, y2).
 */
template <typename T>
void edges_output(const char filename[30], const std::vector<T> points1, const std::vector<T> points2)
{
  assert(points1.size() == points2.size());
	std::ofstream ofs(filename);
	for (size_t i = 0; i < points1.size(); ++i)
	{
		ofs << points1[i].x << " " << points1[i].y << " " << points2[i].x << " " << points2[i].y << "\n";
	}
	ofs.close();
}
#endif

CrossingDetector::CrossingDetector(const double frontier_width, const double max_frontier_angle) :
  frontier_width_(frontier_width),
  max_frontier_angle_(max_frontier_angle)
{
}

Crossing CrossingDetector::crossingDescriptor(const PlaceProfile& profile, const bool normalize)
{
  Crossing crossing;

  if (normalize)
  {
    place_profile_ = normalizedPlaceProfile(profile);
  }
  else
  {
    place_profile_ = profile;
  }
  vector<Point> inputPoints = delaunayInput(place_profile_);

  ROS_INFO("%zu PlaceProfile points", place_profile_.polygon.points.size());
  ROS_INFO("%zu Delaunay points", inputPoints.size());
  
  // Insert points and compute the Delaunay triangulation.
  Delaunay triangulation;
  triangulation.insert(inputPoints.begin(), inputPoints.end());

  // Build a polygon to test if the circumcenter lies inside place_profile_.
  Polygon polygon(inputPoints.begin(), inputPoints.end());

  ROS_DEBUG("Delaunay triangulation is %svalid", triangulation.is_valid() ? "" : "not ");
  ROS_INFO("Number of vertices: %zu", triangulation.number_of_vertices());
  ROS_INFO("Number of faces: %zu", triangulation.number_of_faces());
  // check if the polygon is simple.
  ROS_INFO("The polygon is %ssimple", polygon.is_simple() ? "" : "not ");

  for (Face_iterator face = triangulation.finite_faces_begin(); face != triangulation.finite_faces_end(); ++face)
  {
    const Point c = triangulation.circumcenter(face);
    const Point& p = face->vertex(0)->point();
    if (polygon.bounded_side(p) == CGAL::ON_BOUNDED_SIDE)
    {
      ROS_INFO("Inside");
      // The circumcenter lies inside place_profile_.
      const double circle_radius = std::sqrt((c[0] - p.x()) * (c[0] - p.x()) + (c[1] - p.y()) * (c[1] - p.y()));
      if (circle_radius > crossing.radius)
      {
        crossing.center.x = c[0];
        crossing.center.y = c[1];
        crossing.radius = circle_radius;
      }
    }
    else
      ROS_INFO("Outside");
  }

  crossing.frontiers = frontiers_();

#ifdef DEBUG_CROSSDETECT
	// Cf. tests/debug_plots.py
	// "python $(rospack find crossing_detector)/tests/debug_plots.py"
  // "python $(rospack find crossing_detector)/tests/debug_plots.py place_profile delaunay_input"
  
  // Output place profile.
  points_output("/tmp/place_profile.dat", place_profile_.polygon.points); 
  // Output Delaunay input points.
  vector<geometry_msgs::Point32> inputPointsRos;
  vector<Point>::const_iterator it = inputPoints.begin();
  for (; it != inputPoints.end(); ++it)
  {
    geometry_msgs::Point32 point;
    point.x = it->x();
    point.y = it->y();
    inputPointsRos.push_back(point);
  }
  //points_output("/tmp/delaunay_input.dat", inputPointsRos);
  // Output Delaunay edges.
  vector<geometry_msgs::Point32> edgePoints1;
  vector<geometry_msgs::Point32> edgePoints2;
  Delaunay::Edge_iterator edge_it = triangulation.finite_edges_begin();
  for (; edge_it != triangulation.finite_edges_end(); ++edge_it)
  {
    geometry_msgs::Point32 point0;
    geometry_msgs::Point32 point1;
    point0.x = edge_it->first->vertex(0)->point().x();
    point0.y = edge_it->first->vertex(0)->point().y();
    point1.x = edge_it->first->vertex(1)->point().x();
    point1.y = edge_it->first->vertex(1)->point().y();
    edgePoints1.push_back(point0);
    edgePoints2.push_back(point1);
  }
  edges_output("/tmp/delaunay_edges.dat", edgePoints1, edgePoints2);
#endif

  return crossing;
}

/* Return frontiers.
 *
 * Frontiers can be an excluded segment (or a series of them) or a normal segment.
 * Uses place_profile_ as input.
 */
vector<Frontier> CrossingDetector::frontiers_() const
{
  vector<Frontier> frontiers;

  size_t size = place_profile_.polygon.points.size();

  if (size < 2)
  {
    ROS_ERROR("%s: PlaceProfile message must have at least 2 points",
        ros::this_node::getName().c_str());
    return frontiers;
  }

  double frontier_width2 = frontier_width_ * frontier_width_;

  Frontier frontier;
  for(size_t i = 0; i < size; ++i)
  {
    if (pointIsExcluded(place_profile_, i))
    {
      continue;
    }
    geometry_msgs::Point32 a(place_profile_.polygon.points[i]);
    size_t j = (i + 1) % size;
    while (pointIsExcluded(place_profile_, j))
    {
      j = (j + 1) % size;
      continue;
    }
    geometry_msgs::Point32 b(place_profile_.polygon.points[j]);

    const double dist2 = (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y);

    if (dist2 > frontier_width2)
    {
      const double sx = (a.x + b.x) / 2.0;
      const double sy = (a.y + b.y) / 2.0;

      const double distToFrontierCenter = std::sqrt(sx * sx + sy * sy);
      const double dotProductFrontierSxSy = (b.x - a.x) * sx + (b.y - a.y) * sy;
      const double dist = std::sqrt(dist2);
      const double frontierAngleWithSxSy = std::acos(dotProductFrontierSxSy / dist / distToFrontierCenter);
      if (std::fabs(M_PI_2 - frontierAngleWithSxSy) < max_frontier_angle_)
      {
        frontier.p1.x = a.x;
        frontier.p1.y = a.y;
        frontier.p2.x = b.x;
        frontier.p2.y = b.y;
        frontier.width = dist;
        frontier.angle = std::atan2(sy, sx);
        frontiers.push_back(frontier);
      }
    }
    a = b;
  }
  return frontiers;
}

/* Return frontiers.
 *
 * Frontiers can be an excluded segment (or a series of them) or a normal segment.
 *
 * profile[in] PlaceProfile message.
 * normalize[in] true if the PlaceProfile should be normalized in a first step.
 */
vector<Frontier> CrossingDetector::frontiers(const PlaceProfile& profile, const bool normalize)
{
  if (normalize)
  {
    place_profile_ = normalizedPlaceProfile(profile);
  }
  else
  {
    place_profile_ = profile;
  }
  return frontiers_();
}

vector<Point> CrossingDetector::delaunayInput(const PlaceProfile& profile) const
{
  PlaceProfile delaunayProfile = closedPlaceProfile(profile, frontier_width_);
  vector<Point> points;
  points.reserve(profile.polygon.points.size());
  for (size_t i = 0; i < profile.polygon.points.size(); ++i)
  {
    points.push_back(Point(profile.polygon.points[i].x, profile.polygon.points[i].y));
  }
  return points;
}

} // namespace crossing_detector
} // namespace lama

