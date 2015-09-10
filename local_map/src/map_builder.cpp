/** Defines a class with a local occupancy grid
*/

#include <cmath>
#include <fstream>
#include <math.h>  // for round(), std::round() is since C++11.

#include <local_map/map_builder.h>

namespace local_map
{

const double g_default_p_occupied_when_laser = 0.9;
const double g_default_p_occupied_when_no_laser = 0.3;
const double g_default_large_log_odds = 100;
const double g_default_max_log_odds_for_belief = 20;

/** Return the name of the tf frame that has no parent.
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

/** Return the angle from a quaternion representing a rotation around the z-axis
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

/** In-place move an image represented as a 1D array
 *
 * The origin of the image moves relativelty to a frame F. All pixels must be
 * moved in the opposite direction, so that what is represented by the pixels
 * is fixed in the frame F.
 *
 * @param[in] fill Default fill value
 * @param[in] dx pixel displacement in x (rows)
 * @param[in] dy pixel displacement in y (columns)
 * @param[in] ncol number of column
 * @param[in,out] map image to be moved
 */
template <typename T>
void moveAndCopyImage(int fill, int dx, int dy, unsigned int ncol, vector<T>& map)
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
    const int new_idx_start = offsetFromRowColNoRangeCheck(new_row, col_start, ncol);
    const int row = new_row + dy;  // row in old map, can be outside old map
    int idx = offsetFromRowColNoRangeCheck(row, col_start + dx, ncol);
    const int min_idx = std::max(0, offsetFromRowColNoRangeCheck(row, 0, ncol));
    const int max_idx = std::min(static_cast<int>(map.size()) - 1, offsetFromRowColNoRangeCheck(row, ncol - 1, ncol));
    const int new_idx_end = new_idx_start + col_steps;
    for (int new_idx = new_idx_start; new_idx != new_idx_end;)
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

MapBuilder::MapBuilder(int width, int height, double resolution) :
  angle_resolution_(M_PI / 720),
  p_occupied_when_laser_(g_default_p_occupied_when_laser),
  p_occupied_when_no_laser_(g_default_p_occupied_when_no_laser),
  large_log_odds_(g_default_large_log_odds),
  max_log_odds_for_belief_(g_default_max_log_odds_for_belief),
  has_frame_id_(false)
{
  map_frame_id_ = ros::this_node::getName() + "/local_map";
  map_.header.frame_id = map_frame_id_;
  map_.info.width = width;
  map_.info.height = height;
  map_.info.resolution = resolution;
  map_.info.origin.position.x = -static_cast<double>(width) / 2 * resolution;
  map_.info.origin.position.y = -static_cast<double>(height) / 2 * resolution;
  map_.info.origin.orientation.w = 1.0;
  map_.data.assign(width * height, -1);  // Fill with "unknown" occupancy.
  // log_odds = log(occupancy / (1 - occupancy); prefill with
  // occupancy = 0.5, equiprobability between occupied and free.
  log_odds_.assign(width * height, 0);

  ros::NodeHandle private_nh("~");
  private_nh.getParam("angle_resolution", angle_resolution_);
  private_nh.getParam("p_occupied_when_laser", p_occupied_when_laser_);
  if (p_occupied_when_laser_ <=0 || p_occupied_when_laser_ >= 1)
  {
    ROS_ERROR_STREAM("Parameter "<< private_nh.getNamespace() <<
        "/p_occupied_when_laser must be within ]0, 1[, setting to default (" <<
        g_default_p_occupied_when_laser << ")");
    p_occupied_when_laser_ = g_default_p_occupied_when_laser;
  }
  private_nh.getParam("p_occupied_when_no_laser", p_occupied_when_no_laser_);
  if (p_occupied_when_no_laser_ <=0 || p_occupied_when_no_laser_ >= 1)
  {
    ROS_ERROR_STREAM("Parameter "<< private_nh.getNamespace() <<
        "/p_occupied_when_no_laser must be within ]0, 1[, setting to default (" <<
        g_default_p_occupied_when_no_laser << ")");
    p_occupied_when_no_laser_ = g_default_p_occupied_when_no_laser;
  }
  private_nh.getParam("large_log_odds", large_log_odds_);
  if (large_log_odds_ <=0)
  {
    ROS_ERROR_STREAM("Parameter "<< private_nh.getNamespace() << "/large_log_odds must be positive, setting to default (" <<
        g_default_large_log_odds << ")");
    large_log_odds_ = g_default_large_log_odds;
  }
  private_nh.getParam("max_log_odds_for_belief", max_log_odds_for_belief_);
  try
  {
    std::exp(max_log_odds_for_belief_);
  }
  catch (std::exception)
  {
    ROS_ERROR_STREAM("Parameter "<< private_nh.getNamespace() << "/max_log_odds_for_belief too large, setting to default (" <<
        g_default_max_log_odds_for_belief << ")");
    max_log_odds_for_belief_ = g_default_max_log_odds_for_belief;
  }


  // Fill in the lookup cache.
  const double angle_start = -M_PI;
  const double angle_end = angle_start + 2 * M_PI - 1e-6;
  for (double a = angle_start; a <= angle_end; a += angle_resolution_)
  {
    ray_caster_.getRayCastToMapBorder(a, height, width, 0.9 * angle_resolution_);
  }
}

/** Update occupancy and log odds for a point
 *
 * @param[in] occupied true if the point was measured as occupied
 * @param[in] idx pixel index
 * @param[in] ncol map width
 * @param[in,out] occupancy occupancy map to update
 * @param[in,out] log_odds log odds to update
 */
void MapBuilder::updatePointOccupancy(bool occupied, size_t idx, vector<int8_t>& occupancy, vector<double>& log_odds) const
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
    p = p_occupied_when_laser_;
  }
  else
  {
    p = p_occupied_when_no_laser_;
  }
  // Original formula: Table 4.2, "Probabilistics robotics", Thrun et al., 2005:
  // log_odds[idx] = log_odds[idx] +
  //     std::log(p * (1 - p_occupancy) / (1 - p) / p_occupancy);
  // With p_occupancy = 0.5, this simplifies to:
  log_odds[idx] += std::log(p / (1 - p));
  if (log_odds[idx] < -large_log_odds_)
  {
    log_odds[idx] = -large_log_odds_;
  }
  else if(log_odds[idx] > large_log_odds_)
  {
    log_odds[idx] = large_log_odds_;
  }
  // Update occupancy.
  if (log_odds[idx] < -max_log_odds_for_belief_)
  {
    occupancy[idx] = 0;
  }
  else if (log_odds[idx] > max_log_odds_for_belief_)
  {
    occupancy[idx] = 100;
  }
  else
  {
    occupancy[idx] = static_cast<int8_t>(lround((1 - 1 / (1 + std::exp(log_odds[idx]))) * 100));
  }
}

/** Callback for the LaserScan subscriber.
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
    try
    {
      tf_listerner_.waitForTransform(world_frame_id_, scan.header.frame_id,
          scan.header.stamp, ros::Duration(1.0));
      tf_listerner_.lookupTransform(world_frame_id_, scan.header.frame_id,
          scan.header.stamp, transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      has_frame_id_ = false;
    }
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
    return;
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

bool MapBuilder::updateMap(const sensor_msgs::LaserScan& scan, long int dx, long int dy, double theta)
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
    if (pts.empty())
    {
      continue;
    }
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

/** Return the pixel list by ray casting from map origin to map border, first obstacle.
 *
 * Return the pixel list by ray casting from map origin to map border or first obstacle, whichever comes first.
 *
 * @param[in] map occupancy grid
 * @param[in] angle laser beam angle
 * @param[in] range laser beam range
 * @param[out] raycast list of pixel indexes touched by the laser beam
 * @return true if the last point of the pixel list is an obstacle (end of laser beam). 
 */
bool MapBuilder::getRayCastToObstacle(const nav_msgs::OccupancyGrid& map, double angle, double range, vector<size_t>& raycast)
{
  // Do not consider a 0-length range.
  if (range < 1e-10)
  {
    raycast.clear();
    return false;
  }

  const vector<size_t>& ray_to_map_border = ray_caster_.getRayCastToMapBorder(angle,
      map.info.height, map.info.width, 1.1 * angle_resolution_);
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
  std::string filename;
  if (name.empty())
  {
    const ros::Time time = ros::Time::now();
    const int sec = time.sec;
    const int nsec = time.nsec;

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

  for (size_t i = 0; i < map_.data.size(); ++i)
  {
    ofs << static_cast<int>(map_.data[i]);
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

