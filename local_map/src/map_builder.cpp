/* Defines a class with a local occupancy grid
*/

#include <cmath>
#include <fstream>
#include <math.h>  // for round(), std::round() is since C++11.

#include <local_map/map_builder.h>

namespace local_map {

// Angle resolution for the ray cast lookup.
const double angle_resolution = M_PI / 720;  // 0.25 deg

// Probability that a point is occupied when the laser ranger says so.
const double p_occupied_when_laser = 0.90;
// Probability that a point is occupied when the laser ranger says it's free.
const double p_occupied_when_no_laser = 0.3;
// Large log odds used with probability 0 and 1. The greater, the more inertia.
const double large_log_odds = 1000;
// Max log odds used to compute the belief (exp(max_log_odds_for_belief) should not overflow).
const double max_log_odds_for_belief = 20;

/* Return the name of the tf frame that has no parent.
 */
std::string getWorldFrame(const tf::Transformer& tf_transformer, const std::string& child)
{
  std::string last_parent = child;
  std::string parent;
  bool has_parent = tf_transformer.getParent(child, ros::Time(0), parent);
  while (has_parent)
  {
    last_parent = parent;
    has_parent = tf_transformer.getParent(parent, ros::Time(0), parent);
  }
  return last_parent;
}

/* Return the angle from a quaternion representing a rotation around the z-axis
  *
  * The quaternion in ROS is q = (x, y, z, w), so that
  * q = (ux * sin(a/2), uy * sin(a/2), uz * sin(a/2), cos(a/2)),
  *   where a is the rotation angle and (ux, uy, uz) is the unit vector of the
  *   rotation axis.
  * For a rotation around z, we have q = (cos(a/2), 0, 0, sin(a/2)). Thus
  * a = 2 * atan2(z, w).
  */
double angleFromQuaternion(const tf::Quaternion& q)
{
  if (std::fabs(q.x()) > 1e-5 || std::fabs(q.y()) > 1e-5)
  {
    tf::Vector3 axis = q.getAxis();
    ROS_WARN("Laser frame rotation is not around the z-axis (axis = [%f, %f, %f], just pretending it is",
	axis.x(), axis.y(), axis.z());
  }
  return 2 * std::atan2(q.z(), q.w());
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
    const size_t new_idx_start = offsetFromRowColNoRangeCheck(new_row, col_start, ncol);
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
  if (log_odds[idx] < -max_log_odds_for_belief)
  {
    occupancy[idx] = 0;
  }
  else if (log_odds[idx] > max_log_odds_for_belief)
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
  vector<size_t>::const_iterator idx = indexes.begin();
  for (; idx != indexes.end(); ++idx)
  {
    updatePointOccupancy(occupied, *idx, occupancy, log_odds);
  }
}

MapBuilder::MapBuilder(const int width, const int height, const double resolution) :
  has_frame_id_(false)
{
  map_frame_id_ = ros::this_node::getName() + "/local_map";
  map_.header.frame_id = map_frame_id_;
  map_.info.width = width;
  map_.info.height = height;
  map_.info.resolution = resolution;
  map_.info.origin.position.x = -((double) width) / 2 * resolution;
  map_.info.origin.position.y = -((double) height) / 2 * resolution;
  map_.info.origin.orientation.w = 1.0;
  map_.data.assign(width * height, -1);  // Fill with "unknown" occupancy.
  // log_odds = log(occupancy / (1 - occupancy); prefill with
  // occupancy = 0.5, equiprobability between occupied and free.
  log_odds_.assign(width * height, 0);

  // Fill in the lookup cache.
  const double angle_start = -M_PI;
  const double angle_end = angle_start + 2 * M_PI - 1e-6;
  for (double a = angle_start; a <= angle_end; a += angle_resolution)
  {
    ray_caster_.getRayCastToMapBorder(a, height, width, 0.9 * angle_resolution);
  }
}

/* Callback for the LaserScan subscriber.
 *
 * Update (geometrical transformation + probability update) the map with the current scan
 */
void MapBuilder::grow(const sensor_msgs::LaserScan& scan)
{
  if (!has_frame_id_)
  {
    // Wait for a parent.
    std::string parent;
    bool has_parent = tf_listerner_.getParent(scan.header.frame_id, ros::Time(0), parent);
    if (!has_parent)
    {
      ROS_DEBUG_STREAM("Frame " << scan.header.frame_id << " has no parent");
      return;
    }
    world_frame_id_ = getWorldFrame(tf_listerner_, scan.header.frame_id);
    ROS_INFO_STREAM("Found world frame " << world_frame_id_);
    has_frame_id_ = true;

    // Initialize saved positions.
    tf::StampedTransform transform;
    tf_listerner_.waitForTransform(world_frame_id_, scan.header.frame_id,
	scan.header.stamp, ros::Duration(5.0));
    tf_listerner_.lookupTransform(world_frame_id_, scan.header.frame_id,
	scan.header.stamp, transform);
    xinit_ = transform.getOrigin().x();
    yinit_ = transform.getOrigin().y();
    last_xmap_ = lround(xinit_ / map_.info.resolution);
    last_ymap_ = lround(yinit_ / map_.info.resolution);

    // Send a map frame with identity transform.
    tf::Transform map_transform;
    map_transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    map_transform.setRotation(tf::Quaternion(1, 0, 0, 0));
    tr_broadcaster_.sendTransform(tf::StampedTransform(map_transform,
	  scan.header.stamp, scan.header.frame_id, map_frame_id_));
  }

  // Get the displacement.
  tf::StampedTransform new_tr;
  try
  {
    tf_listerner_.waitForTransform(world_frame_id_, scan.header.frame_id,
	scan.header.stamp, ros::Duration(0.2));
    tf_listerner_.lookupTransform(world_frame_id_, scan.header.frame_id,
	scan.header.stamp, new_tr);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
  }

  // Map position relative to initialization.
  const double x = new_tr.getOrigin().x() - xinit_;
  const double y = new_tr.getOrigin().y() - yinit_;
  const double theta = angleFromQuaternion(new_tr.getRotation());

  // Get the pixel displacement of the map.
  const long int xmap = lround(x / map_.info.resolution);
  const long int ymap = lround(y / map_.info.resolution);
  const long int map_dx = xmap - last_xmap_;
  const long int map_dy = ymap - last_ymap_;

  // Update the map
  const bool move = updateMap(scan, map_dx, map_dy, theta);
  if (move)
  {
    //ROS_INFO("Displacement: %ld, %ld pixels", map_dx, map_dy);
    // Record the position only if the map moves.
    last_xmap_ = xmap;
    last_ymap_ = ymap;
  }

  // Update the map frame, so that it's oriented like frame named "world_frame_id_".
  tf::Transform map_transform;
  map_transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  tf::Quaternion q;
  q.setRPY(0, 0, -theta);
  map_transform.setRotation(q);
  tr_broadcaster_.sendTransform(tf::StampedTransform(map_transform, scan.header.stamp, scan.header.frame_id, map_frame_id_));
}

bool MapBuilder::updateMap(const sensor_msgs::LaserScan& scan, const long int dx, const long int dy, const double theta)
{
  const bool has_moved = (dx != 0 || dy != 0);
  const int ncol = map_.info.width;
  if (has_moved)
  {
    // Move the map and log_odds_.
    moveAndCopyImage(-1, dx, dy, ncol, map_.data);
    moveAndCopyImage(0, dx, dy, ncol, log_odds_);
  }

  // Update occupancy.
  for (size_t i = 0; i < scan.ranges.size(); ++i)
  {
    const double angle = angles::normalize_angle(scan.angle_min + i * scan.angle_increment + theta);
    vector<size_t> pts;
    const bool obstacle_in_map = getRayCastToObstacle(map_, angle, scan.ranges[i], pts);
    if (obstacle_in_map)
    {
      // The last point is the point with obstacle.
      const size_t last_pt = pts.back();
      updatePointOccupancy(true, last_pt, map_.data, log_odds_);
      pts.pop_back();
    }
    // The remaining points are in free space.
    updatePointsOccupancy(false, pts, map_.data, log_odds_);
  }
  return has_moved;
}

/* Return the pixel list by ray casting from map origin to map border, first obstacle.
  *
  * Return the pixel list by ray casting from map origin to map border or first obstacle, whichever comes first.
  * Return true if the last point of the pixel list is an obstacle (end of laser beam).
  *
  * map[in] occupancy grid
  * angle[in] laser beam angle
  * range[in] laser beam range
  * raycast[out] list of pixel indexes touched by the laser beam
  */
bool MapBuilder::getRayCastToObstacle(const nav_msgs::OccupancyGrid& map, const double angle, const double range, vector<size_t>& raycast)
{
  const vector<size_t>& ray_to_map_border = ray_caster_.getRayCastToMapBorder(angle,
      map.info.height, map.info.width, 1.1 * angle_resolution);
  // range in pixel length. The ray length in pixels corresponds to the number
  // of pixels in the bresenham algorithm.
  const size_t pixel_range = lround(range * max(abs(std::cos(angle)), abs(std::sin(angle))) / map.info.resolution);
  size_t raycast_size;
  bool obstacle_in_map = pixel_range < ray_to_map_border.size();
  if (obstacle_in_map)
  {
    raycast_size = pixel_range;
  }
  else
  {
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

bool MapBuilder::saveMap(const std::string& name) const
{
  const ros::Time time = ros::Time::now();
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
  ofs.open(filename.c_str());
  if (!ofs.is_open())
  {
    ROS_ERROR("Cannot open %s", filename.c_str());
    return false;
  }

  for (uint i = 0; i < map_.data.size(); ++i)
  {
    ofs << (int) map_.data[i];
    if ((i % map_.info.width) == (map_.info.width - 1))
    {
      ofs << "\n";
    }
    else
    {
      ofs << ",";
    }
  }
  ofs.close();
  return true;
}

} // namespace local_map

