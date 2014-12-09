#include <lama_common/crossing_visualization.h>

namespace lama_common {

/* Return the marker for the visualization of the crossing center
 */
visualization_msgs::Marker getCrossingCenterMarker(const std::string& frame_id, const lama_msgs::Crossing& crossing)
{
	visualization_msgs::Marker m;
	m.header.frame_id = frame_id;
	m.ns = "crossing_center";
	m.type = visualization_msgs::Marker::SPHERE;
	m.pose.position.x = crossing.center.x;
	m.pose.position.y = crossing.center.y;
	m.pose.position.z = 0;
	m.pose.orientation.w = 1.0;
	m.scale.x = crossing.radius;
	m.scale.y = crossing.radius;
	m.scale.z = 1;
	m.color.r = 1.0;
	m.color.g = 1.0;
	m.color.a = 0.5;
	return m;
}

/* Return the marker for the visualization of the roads
 */
visualization_msgs::Marker getFrontiersMarker(const std::string& frame_id, const lama_msgs::Crossing& crossing)
{
	visualization_msgs::Marker m;
	m.header.frame_id = frame_id;
	m.ns = "frontiers";
	m.type = visualization_msgs::Marker::LINE_LIST;
	m.pose.orientation.w = 1.0;
	m.scale.x = 0.1;
	m.color.r = 0.0;
	m.color.g = 0.0;
	m.color.b = 1.0;
	m.color.a = 0.5;

  geometry_msgs::Point p;
  for(size_t i = 0; i < crossing.frontiers.size(); ++i)
	{
    // Frontier itself.
    p.x = crossing.frontiers[i].p1.x;
    p.y = crossing.frontiers[i].p1.y;
		m.points.push_back(p);
    p.x = crossing.frontiers[i].p2.x;
    p.y = crossing.frontiers[i].p2.y;
		m.points.push_back(p);
    // Line from robot with angle frontier.angle.
    p.x = 0;
    p.y = 0;
    m.points.push_back(p);
    p.x = 4 * crossing.radius * std::cos(crossing.frontiers[i].angle);
    p.y = 4 * crossing.radius * std::sin(crossing.frontiers[i].angle);
    m.points.push_back(p);
	}
	return m;
}

} // namespace lama_common
