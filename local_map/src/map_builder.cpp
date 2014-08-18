/* Defines a class with a local occupancy grid
 */

#include <cmath>
#include <fstream>

#include "local_map/map_builder.h"

// Probability that a point is occupied when the laser ranger says so.
const double p_occupied_when_laser = 0.70;
// Probability that a point is occupied when the laser ranger says it's free.
const double p_occupied_when_no_laser = 0.05;
// Large log odds used with probability 0 and 1. The greater, the more inertia.
const double large_log_odds = 1000;
// Max log odds used to compute the belief (exp(max_log_odds) should not overflow).
const double max_log_odds = 20;

using std::vector;
using std::abs;
using std::max;
using std::lround;

// A map to store ray castings
typedef std::map<double, vector<size_t>> raycast_dict;

std::string getWorldFrame(const tf::Transformer& tfTransformer, const std::string& child)
{
	std::string last_parent = child;
	std::string parent;
	bool has_parent = tfTransformer.getParent(child, ros::Time(0), parent);
	while (has_parent)
	{
		last_parent = parent;
		has_parent = tfTransformer.getParent(parent, ros::Time(0), parent);
	}
	return last_parent;
}

/* Return the angle from a quaternion representing a rotation around the z-axis
 *
 * The quaternion in ROS is q = (w, x, y, z), so that
 * q = (cos(a/2), ux * sin(a/2), uy * sin(a/2), uz * sin(a/2)),
 *   where a is the rotation angle and (ux, uy, uz) is the unit vector of the
 *   rotation axis.
 * For a rotation around z, we have q = (cos(a/2), 0, 0, sin(a/2)). Thus
 * a = 2 * atan2(z, w).
 */
double angleFromQuaternion(const tf::Quaternion& q)
{
	if (std::fabs(q.x()) > 1e-5 || std::fabs(q.y()) > 1e-5)
	{
		ROS_WARN("Laser frame rotation is not around the z-axis, just pretending it is");
	}
	return 2 * std::atan2(q.z(), q.w());
}

/* Return the list of pixel indexes from map origin to pixel at map border and given angle
 *
 * The Bresenham algorithm is used for rendering.
 *
 * map[in] occupancy grid
 * angle[in] beam angle
 * nrow[in] image height
 * ncol[in] image width
 */
vector<size_t> getRayCastToMapBorder(const nav_msgs::OccupancyGrid& map, const double angle)
{
	vector<size_t> pts;

	// Pixel coordinates of map origin.
	int row_center = lround(-map.info.origin.position.y / map.info.resolution);
	int col_center = lround(-map.info.origin.position.x / map.info.resolution);
	// Diagonal length.
	const size_t nrow = map.info.height;
	const size_t ncol = map.info.width;
	const double r = std::sqrt((double) nrow * nrow + ncol * ncol);
	// Start point, map origin.
	int x0 = col_center;
	int y0 = row_center;
	// End point, outside the map.
	int x1 = (int) lround(x0 + r * std::cos(angle)); // Can be negative
	int y1 = (int) lround(y0 + r * std::sin(angle));
	int dx = x1 - x0;
	int dy = y1 - y0;
	pts.reserve(max(abs(dx), abs(dy)) + 1);
	bool steep = (abs(dy) >= abs(dx));
	if (steep)
	{
		swap(x0, y0);
		swap(x1, y1);
		// recompute Dx, Dy after swap
		dx = x1 - x0;
		dy = y1 - y0;
	}
	int xstep = 1;
	if (dx < 0)
	{
		xstep = -1;
		dx = -dx;
	}
	int ystep = 1;
	if (dy < 0)
	{
		ystep = -1;
		dy = -dy;
	}
	int twoDy = 2 * dy;
	int twoDyTwoDx = twoDy - 2 * dx; // 2*Dy - 2*Dx
	int e = twoDy - dx; //2*Dy - Dx
	int y = y0;
	int xDraw, yDraw;
	for (int x = x0; x != x1; x += xstep)
	{
		if (steep)
		{
			xDraw = y;
			yDraw = x;
		}
		else
		{
			xDraw = x;
			yDraw = y;
		}
		if (pointInMap(yDraw, xDraw, map))
		{
			pts.push_back(offsetFromRowCol(yDraw, xDraw, ncol));
		}
		else
		{
			// We exit when the first point outside the map is encountered.
			return pts;
		}
		// next
		if (e > 0)
		{
			e += twoDyTwoDx; //E += 2*Dy - 2*Dx;
			y = y + ystep;
		}
		else
		{
			e += twoDy; //E += 2*Dy;
		}
	}
}

/* Return the pixel representation of the nearest ray
 *
 * raycast_lookup[in] map (ray angle --> pixel list)
 * angle[in] laser beam angle
 * raycast[out] list of pixel indexes touched by the laser beam
 */
void rayLookup(const raycast_dict& raycast_lookup, const double angle, vector<size_t>& raycast)
{
	double dangle_lower;
	double dangle_upper;

	auto upper_bound = raycast_lookup.upper_bound(angle);
	if (upper_bound == raycast_lookup.begin())
	{

		raycast = upper_bound->second;
		return;
	}
	else if (upper_bound == raycast_lookup.end())
	{
		dangle_upper = raycast_lookup.begin()->first - angle + 2 * M_PI;
		upper_bound--;
		dangle_lower = upper_bound->first - angle;
		if (dangle_lower < dangle_upper)
		{
			raycast = upper_bound->second;
			return;
		}
		else
		{
			raycast = raycast_lookup.begin()->second;
			return;
		}
	}
	else
	{
		dangle_upper = upper_bound->first - angle;
		auto lower_bound = upper_bound;
		lower_bound--;
		dangle_lower = angle - lower_bound->first;
		if (dangle_lower < dangle_upper)
		{
			raycast = lower_bound->second;
			return;
		}
		else
		{
			raycast = upper_bound->second;
			return;
		}
	}
}

/* Return the pixel list by ray casting from map origin to map border
 *
 * map[in] occupancy grid
 * angle[in] laser beam angle
 * raycast[out] list of pixel indexes touched by the laser beam
 */
void getRayCast(const nav_msgs::OccupancyGrid& map, const double angle, vector<size_t>& raycast)
{
	// Store the ray casting up to the map border into a look-up table. Ray
	// casting exclusively depends on the ray angle.
	static std::map<double, vector<size_t>> raycast_lookup;
	static bool raycast_lookup_cached;

	if (!raycast_lookup_cached)
	{
		const double angle_start = -M_PI;
		const double angle_end = angle_start + 2 * M_PI;
		const double angle_resolution = M_PI / 720;  // 0.25 deg
		for (double a = angle_start; a <= angle_end; a += angle_resolution)
		{
			raycast_lookup[a] = getRayCastToMapBorder(map, a);
		}
		raycast_lookup_cached = true;
	}
	return rayLookup(raycast_lookup, angle, raycast);
}

/* Render a line as bitmap.
 *
 * The points are ordered from (x0, y0) to (x1, y1).
 *
 * map[in] occupancy grid
 * angle[in] laser beam angle
 * range[in] laser beam range
 * raycast[out] list of pixel indexes touched by the laser beam
 */
bool bresenham(const nav_msgs::OccupancyGrid& map, const double angle, const double range, vector<size_t>& raycast)
{
	bool obstacle_in_map;

	vector<size_t> ray_to_map_border;
	getRayCast(map, angle, ray_to_map_border);
	const size_t row_origin = rowFromOffset(ray_to_map_border[0], map.info.width);
	const size_t col_origin = colFromOffset(ray_to_map_border[0], map.info.width);
	// range in pixel length. The ray length in pixels corresponds to the number
	// of pixels in the bresenham algorithm.
	const size_t pixel_range = lround(range * max(abs(std::cos(angle)), abs(std::sin(angle))) / map.info.resolution);
	size_t raycast_size;
	if (pixel_range < ray_to_map_border.size())
	{
		obstacle_in_map = true;
		raycast_size = pixel_range;
	}
	else
	{
		obstacle_in_map = false;
		raycast_size = ray_to_map_border.size();
	}
	raycast.clear();
	raycast.reserve(raycast_size);
	for (size_t i = 0; i < raycast_size; ++i)
	{
		raycast.push_back(ray_to_map_border[i]);
	}

	return obstacle_in_map;
}

/* In-place move an image represented as a 1D array
 *
 * The origin of the image moves relativelty to a frame F. All pixels must be
 * moved in the opposite direction, so that what is represented by the pixels
 * is fixed in the frame F.
 *
 * fill Default fill value
 * dx pixel displacement in x (rows)
 * dy pixel displacement in y (columns)
 * ncol number of column
 * map image to be moved
 */
template <typename T>
void moveAndCopyImage(const int fill, const int dx, const int dy, const unsigned int ncol, vector<T>& map)
{
	if (dx == 0 && dy == 0)
	{
		return;
	}
	
	const unsigned int nrow = map.size() / ncol;
	int row_start = 0;
	int row_end = nrow;
	int row_increment = 1;
	if (dy < 0)
	{
		row_start = nrow - 1;
		row_end = -1;
		row_increment = -1;
	}
	int col_start = 0;
	int col_steps = ncol;
	int col_increment = 1;
	if (dx < 0)
	{
		col_start = ncol - 1;
		col_steps = -ncol;
		col_increment = -1;
	}
	for (int new_row = row_start; new_row != row_end; new_row += row_increment)
	{
		const size_t new_idx_start = offsetFromRowCol(new_row, col_start, ncol);
		const int row = new_row + dy;  // row in old map, can be outside old map
		int idx = offsetFromRowColNoRangeCheck(row, col_start + dx, ncol);
		const int min_idx = std::max(0, offsetFromRowColNoRangeCheck(row, 0, ncol));
		const int max_idx = std::min((int) map.size() - 1, offsetFromRowColNoRangeCheck(row, ncol - 1, ncol));
		const size_t new_idx_end = new_idx_start + col_steps;
		for (int new_idx = new_idx_start; new_idx != new_idx_end; )
		{
			if (min_idx <= idx && idx <= max_idx)
			{
				map[new_idx] = map[idx];
			}
			else
			{
				map[new_idx] = fill;
			}
			new_idx += col_increment;
			idx += col_increment;
		}
	}
}

/* Update occupancy and log odds for a point
 *
 * occupied[in] true if the point was measured as occupied
 * idx[in] pixel index
 * ncol[in] map width
 * occupancy[in/out] occupancy map to update
 * log_odds[in/out] log odds to update
 */
void updatePointOccupancy(const bool occupied, const size_t idx, vector<int8_t>& occupancy, vector<double>& log_odds)
{
	if (idx >= occupancy.size())
	{
		return;
	}

	if (occupancy.size() != log_odds.size())
	{
		ROS_ERROR("occupancy and count do not have the same number of elements");
		return;
	}

	// Update log_odds.
	double p;  // Probability of being occupied knowing current measurement.
	if (occupied)
	{
		p = p_occupied_when_laser;
	}
	else
	{
		p = p_occupied_when_no_laser;
	}
	// Original formula: Table 4.2, "Probabilistics robotics", Thrun et al., 2005:
	// log_odds[idx] = log_odds[idx] +
	//     std::log(p * (1 - p_occupancy) / (1 - p) / p_occupancy);
	// With p_occupancy = 0.5, this simplifies to:
	log_odds[idx] += std::log(p / (1 - p));
	if (log_odds[idx] < -large_log_odds)
	{
		log_odds[idx] = -large_log_odds;
	}
	else if(log_odds[idx] > large_log_odds)
	{
		log_odds[idx] = large_log_odds;
	}
	// Update occupancy.
	if (log_odds[idx] < -max_log_odds)
	{
		occupancy[idx] = 0;
	}
	else if (log_odds[idx] > max_log_odds)
	{
		occupancy[idx] = 100;
	}
	else
	{
		occupancy[idx] = (int8_t) lround((1 - 1 / (1 + std::exp(log_odds[idx]))) * 100);
	}
}

/* Update occupancy and log odds for a list of a points
 */
inline void updatePointsOccupancy(const bool occupied, const vector<size_t>& indexes, vector<int8_t>& occupancy, vector<double>& log_odds)
{
	for (const auto& idx : indexes)
	{
		updatePointOccupancy(occupied, idx, occupancy, log_odds);
	}
}

MapBuilder::MapBuilder(const int width, const int height, const double resolution) :
	m_has_frame_id(false)
{
	m_map.header.frame_id = "local_map";
	m_map.info.width = width;
	m_map.info.height = height;
	m_map.info.resolution = resolution;
	m_map.info.origin.position.x = -((double) width) / 2 * resolution;
	m_map.info.origin.position.y = -((double) height) / 2 * resolution;
	m_map.info.origin.orientation.w = 1.0;
	m_map.data.assign(width * height, -1);  // Fill with "unknown" occupancy.
	// log_odds = log(occupancy / (1 - occupancy); prefill with
	// occupancy = 0.5, equiprobability between occupied and free.
	m_log_odds.assign(width * height, 0);
}

nav_msgs::OccupancyGrid MapBuilder::getMap() const
{
	return m_map;
}

/* Update (geometrical transformation + probability update) the map with the current scan
 */
void MapBuilder::grow(const sensor_msgs::LaserScan& scan)
{
	static tf::TransformBroadcaster br;

	if (!m_has_frame_id)
	{
		// Wait for a parent.
		std::string parent;
		bool has_parent = m_tfListener.getParent(scan.header.frame_id, ros::Time(0), parent);
		if (!has_parent)
		{
			return;
		}
		m_world_frame_id = getWorldFrame(m_tfListener, scan.header.frame_id);
		m_has_frame_id = true;

		// Initialize saved positions.
		tf::StampedTransform transform;
		m_tfListener.waitForTransform(m_world_frame_id, scan.header.frame_id,
				scan.header.stamp, ros::Duration(5.0));
		m_tfListener.lookupTransform(m_world_frame_id, scan.header.frame_id,
				scan.header.stamp, transform);
		m_xinit = transform.getOrigin().x();
		m_yinit = transform.getOrigin().y();
		m_last_xmap = lround(m_xinit / m_map.info.resolution);
		m_last_ymap = lround(m_yinit / m_map.info.resolution);

		// Send a map frame with identity transform.
		tf::Transform map_transform;
		map_transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
		map_transform.setRotation(tf::Quaternion(1, 0, 0, 0));
		br.sendTransform(tf::StampedTransform(map_transform, ros::Time::now(), scan.header.frame_id, "local_map"));
	}

	// Get the displacement.
	tf::StampedTransform new_tr;
	try
	{
		m_tfListener.waitForTransform(m_world_frame_id, scan.header.frame_id,
				scan.header.stamp, ros::Duration(0.5));
		m_tfListener.lookupTransform(m_world_frame_id, scan.header.frame_id,
				scan.header.stamp, new_tr);
	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR("%s", ex.what());
	}

	// Map position relative to initialization.
	double x = new_tr.getOrigin().x() - m_xinit;
	double y = new_tr.getOrigin().y() - m_yinit;
	double theta = angleFromQuaternion(new_tr.getRotation());

	// Get the pixel displacement of the map.
	long int xmap = lround(x / m_map.info.resolution);
	long int ymap = lround(y / m_map.info.resolution);
	long int map_dx = xmap - m_last_xmap;
	long int map_dy = ymap - m_last_ymap;

	// Update the map
	bool move = updateMap(scan, map_dx, map_dy, theta);
	if (move)
	{
		//ROS_INFO("Displacement: %ld, %ld pixels", map_dx, map_dy);
		// Record the position only if the map moves.
		m_last_xmap = xmap;
		m_last_ymap = ymap;
	}

	// Update the map frame, so that it's oriented like frame named "m_world_frame_id".
	tf::Transform map_transform;
	map_transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
	tf::Quaternion q;
	q.setRPY(0, 0, -theta);
	map_transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(map_transform, scan.header.stamp, scan.header.frame_id, "local_map"));
}

bool MapBuilder::updateMap(const sensor_msgs::LaserScan& scan, const long int dx, const long int dy, const double theta)
{
	bool has_moved = false;
	int ncol = m_map.info.width;
	if (dx != 0 || dy != 0)
	{
		has_moved = true;
		// Move the map and m_log_odds.
		moveAndCopyImage(-1, dx, dy, ncol, m_map.data);
		moveAndCopyImage(0, dx, dy, ncol, m_log_odds);
	}

	// Update occupancy
	vector<size_t> pts;
	for (size_t i = 0; i < scan.ranges.size(); ++i)
	{
		double angle = scan.angle_min + i * scan.angle_increment + theta;
		bool obstacle_in_map = bresenham(m_map, angle, scan.ranges[i], pts);
		if (obstacle_in_map)
		{
			// Last point is the point with obstacle.
			auto last_pt = pts.back();
			updatePointOccupancy(true, last_pt, m_map.data, m_log_odds);
			pts.pop_back();
		}
		// The remaining points are in free space.
		updatePointsOccupancy(false, pts, m_map.data, m_log_odds);
	}
	return has_moved;
}

bool MapBuilder::saveMap(const std::string& name) const
{
	bool res = true;
	ros::Time time = ros::Time::now();
	const int sec = time.sec;
	const int nsec = time.nsec;

	std::string filename;
	if (name.empty())
	{
		std::stringstream sname;
		sname << "map_";
		sname << std::setw(5) << std::setfill('0') << sec;
		sname << std::setw(0) << "_";
		sname << std::setw(9) << std::setfill('0') << nsec;
		sname << std::setw(0) << ".txt";
		filename = sname.str();
	}
	else
	{
		filename = name;
	}

	std::ofstream ofs;
	ofs.open(filename);
	if (!ofs.is_open())
	{
		ROS_ERROR("Cannot open %s", filename.c_str());
		return false;
	}

	for (uint i = 0; i < m_map.data.size(); ++i)
	{
		ofs << (int) m_map.data[i];
		if ((i % m_map.info.width) == (m_map.info.width - 1))
		{
			ofs << "\n";
		}
		else
		{
			ofs << ",";
		}
	}
	ofs.close();
}

