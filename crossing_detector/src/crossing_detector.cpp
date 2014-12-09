#include <crossing_detector/crossing_detector.h>

/* #define DEBUG_CROSSDETECT */

#ifdef DEBUG_CROSSDETECT
#include <fstream>
#include <cassert>
#endif

namespace crossing_detector {

#ifdef DEBUG_CROSSDETECT

/* Save a list of geometry_msgs::Point32 into a file (space separation).
 */
void points_output(const std::string& filename, const std::vector<geometry_msgs::Point32>& points)
{
  std::ofstream ofs(filename.c_str());
  if (!ofs.is_open())
  {
    ROS_WARN("Cannot open %s for writing", filename.c_str());
    return;
  }
  for (size_t i = 0; i < points.size(); ++i)
  {
    ofs << points[i].x << " " << points[i].y << "\n";
  }
  ofs.close();
}

/* Save a list of Point into a file (space separation).
*/
void points_output(const std::string& filename, const std::vector<Point>& points)
{
  std::ofstream ofs(filename.c_str());
  if (!ofs.is_open())
  {
    ROS_WARN("Cannot open %s for writing", filename.c_str());
    return;
  }
  for (size_t i = 0; i < points.size(); ++i)
  {
    ofs << points[i].x() << " " << points[i].y() << "\n";
  }
  ofs.close();
}

/* Save a list of edges into a file (space-separated x1, y1, x2, y2).
*/
void edges_output(const std::string& filename, const Delaunay& dt)
{
  std::ofstream ofs(filename.c_str());
  if (!ofs.is_open())
  {
    ROS_WARN("Cannot open %s for writing", filename.c_str());
    return;
  }
  Delaunay::Edge_iterator edge_it = dt.finite_edges_begin();
  for (; edge_it != dt.finite_edges_end(); ++edge_it)
  {
    const Delaunay::Face_handle face = edge_it->first;
    const int vertex_num = edge_it->second;

    ofs <<
      face->vertex(face->cw(vertex_num))->point().x() << " " <<
      face->vertex(face->cw(vertex_num))->point().y() << " " <<
      face->vertex(face->ccw(vertex_num))->point().x() << " " <<
      face->vertex(face->ccw(vertex_num))->point().y() << "\n";
  }
  ofs.close();
}

/* Save a list of (x, z, r) into a file
*/
void candidates_output(const std::string& filename, const Delaunay& dt, const Polygon& polygon, const bool rejected=false)
{
  std::ofstream ofs(filename.c_str());
  if (!ofs.is_open())
  {
    ROS_WARN("Cannot open %s for writing", filename.c_str());
    return;
  }
  for (Face_iterator face = dt.finite_faces_begin(); face != dt.finite_faces_end(); ++face)
  {
    const Point c = dt.circumcenter(face);
    if ((polygon.bounded_side(c) == CGAL::ON_BOUNDED_SIDE) ^ rejected)
    {
      // The circumcenter lies inside place_profile_.
      const Point& p = face->vertex(0)->point();
      const double circle_radius = std::sqrt((c[0] - p.x()) * (c[0] - p.x()) + (c[1] - p.y()) * (c[1] - p.y()));
      ofs << c[0] << " " << c[1] << " " << circle_radius << "\n";
    }
  }
  ofs.close();
}

#endif

CrossingDetector::CrossingDetector(const double frontier_width, const double max_frontier_angle) :
  frontier_width_(frontier_width),
  max_frontier_angle_(max_frontier_angle),
  min_relevance_(0.01)
{
  std::strncpy(node_name_, ros::this_node::getName().c_str(), 30);

  ros::NodeHandle private_nh("~");
  private_nh.getParamCached("max_frontier_angle", max_frontier_angle_);
  private_nh.getParamCached("frontier_width", frontier_width_);
}

/* Return a lama_msgs/Crossing message from analysis of a lama_msgs/PlaceProfile
 *
 * profile[in] PlaceProfile message.
 * normalize[in] true if the PlaceProfile should be normalized in a first step.
 */
Crossing CrossingDetector::crossingDescriptor(const PlaceProfile& profile, const bool normalize)
{
  Crossing crossing;

  if (normalize)
  {
    place_profile_ = lama_common::normalizedPlaceProfile(profile);
  }
  else
  {
    place_profile_ = profile;
  }
  vector<Point> inputPoints = delaunayInput(place_profile_);

  ROS_DEBUG("%s: %zu PlaceProfile points", node_name_, place_profile_.polygon.points.size());
  ROS_DEBUG("%s: %zu Delaunay input points", node_name_, inputPoints.size());

  // Insert points and compute the Delaunay triangulation.
  Delaunay triangulation;
  triangulation.insert(inputPoints.begin(), inputPoints.end());

  // Build a polygon to test if the circumcenter lies inside place_profile_.
  Polygon polygon(inputPoints.begin(), inputPoints.end());

  ROS_DEBUG("%s: Delaunay triangulation is %svalid", node_name_, triangulation.is_valid() ? "" : "not ");
  // check if the polygon is simple (i.e., e.g., if points are sorted according to their angle).
  ROS_DEBUG("%s: the polygon is %ssimple", node_name_, polygon.is_simple() ? "" : "not ");

  for (Face_iterator face = triangulation.finite_faces_begin(); face != triangulation.finite_faces_end(); ++face)
  {
    const Point c = triangulation.circumcenter(face);
    if (polygon.bounded_side(c) == CGAL::ON_BOUNDED_SIDE)
    {
      // The circumcenter lies inside place_profile_.
      const Point& p = face->vertex(0)->point();
      const double circle_radius = std::sqrt((c[0] - p.x()) * (c[0] - p.x()) + (c[1] - p.y()) * (c[1] - p.y()));
      if (circle_radius > crossing.radius)
      {
	crossing.center.x = c[0];
	crossing.center.y = c[1];
	crossing.radius = circle_radius;
      }
    }
  }

  crossing.frontiers = frontiers_();

#ifdef DEBUG_CROSSDETECT
  // Cf. tests/debug_plots.py
  // "python $(rospack find crossing_detector)/tests/debug_plots.py"

  // Output place profile.
  std::string filename = "/tmp/" + ros::this_node::getName() + "_place_profile.dat";
  points_output(filename, place_profile_.polygon.points); 
  // Output Delaunay input points.
  filename = "/tmp/" + ros::this_node::getName() + "_delaunay_input.dat";
  points_output(filename, inputPoints);
  // Output Delaunay edges.
  filename = "/tmp/" + ros::this_node::getName() + "_delaunay_edges.dat";
  edges_output(filename, triangulation);
  // Output crossing center candidates.
  filename = "/tmp/" + ros::this_node::getName() + "_candidates.dat";
  candidates_output(filename, triangulation, polygon);
  // Output excluded crossing center.
  filename = "/tmp/" + ros::this_node::getName() + "_rejected.dat";
  candidates_output(filename, triangulation, polygon, true);
#endif

  return crossing;
}

/* Return frontiers computed from place_profile_.
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
	node_name_);
    return frontiers;
  }

  double frontier_width2 = frontier_width_ * frontier_width_;

  Frontier frontier;
  for(size_t i = 0; i < size; ++i)
  {
    geometry_msgs::Point32 a(place_profile_.polygon.points[i]);
    geometry_msgs::Point32 b(place_profile_.polygon.points[(i + 1) % size]);

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

/* Return a list of frontiers.
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
    place_profile_ = lama_common::normalizedPlaceProfile(profile);
  }
  else
  {
    place_profile_ = profile;
  }
  return frontiers_();
}

/* Return a list of points suited for Delaunay
 *
 * Two operations are realized:
 * - reduce the number of points with a relevance filter (less points on a single "line segment").
 * - fill frontiers so that the crossing center will not be found at frontiers.
 */
vector<Point> CrossingDetector::delaunayInput(const PlaceProfile& profile) const
{
  PlaceProfile delaunayProfile = lama_common::simplifiedPlaceProfile(profile, min_relevance_);
  ROS_DEBUG("%s: %zu relevant points in the PlaceProfile", node_name_, delaunayProfile.polygon.points.size());
  lama_common::closePlaceProfile(delaunayProfile, frontier_width_ / 2);
  vector<Point> points;
  points.reserve(delaunayProfile.polygon.points.size());
  for (size_t i = 0; i < delaunayProfile.polygon.points.size(); ++i)
  {
    points.push_back(Point(delaunayProfile.polygon.points[i].x, delaunayProfile.polygon.points[i].y));
  }
  return points;
}

} // namespace crossing_detector

