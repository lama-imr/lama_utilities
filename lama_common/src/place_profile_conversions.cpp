#include <lama_common/place_profile_conversions.h>

#define OCCUPIED_THRESHOLD 60 // 0 = free, 100 = occupied.
#define COSTMAP_DISCRETISATION_COUNT 360 // Number of points for the costmap discretisation to obtain a PlaceProfile.

namespace lama_common {

/* Return the polygon corresponding to the PlaceProfile
 */
geometry_msgs::Polygon placeProfileToPolygon(const PlaceProfile& profile)
{
  geometry_msgs::Polygon polygon;

  polygon.points.reserve(profile.polygon.points.size());

  for (size_t i = 0; i < polygon.points.size(); ++i)
  {
    polygon.points.push_back(polygon.points[i]);
  }

  return polygon;
}

/* Return the point cloud corresponding to the PlaceProfile
 */
sensor_msgs::PointCloud placeProfileToPointCloud(const PlaceProfile& profile)
{
  sensor_msgs::PointCloud cloud;
  cloud.header = profile.header;
  for (size_t i = 0; i < profile.polygon.points.size(); ++i)
  {
    cloud.points.push_back(profile.polygon.points[i]);
  }
  return cloud;
}

/* Tranform a LaserScan into a PlaceProfile
 *
 * The laser beams greater than range_cutoff are removed. However, the first and the last beam of
 * a series of beams larger than range_cutoff are kept are their length is reduced to range_cutoff.
 */
PlaceProfile laserScanToPlaceProfile(const sensor_msgs::LaserScan& scan, const double range_cutoff)
{
  // Use LaserProjection for its caching of sine and cosine values.
  static laser_geometry::LaserProjection projector;

  PlaceProfile profile;
  profile.header = scan.header;
  size_t size = scan.ranges.size();
  profile.polygon.points.reserve(size);

  vector<bool> in_range(size, false);
  for (size_t i = 0; i < size; ++i)
  {
    if (scan.ranges[i] < range_cutoff)
    {
      in_range[i] = true;
    }
  }

  const double greater_than_longest = *(std::max_element(scan.ranges.begin(), scan.ranges.end())) + 1;
  sensor_msgs::PointCloud cloud;
  projector.projectLaser(scan, cloud, greater_than_longest, laser_geometry::channel_option::None);

  if (scan.ranges.size() != cloud.points.size())
  {
    // projectLaser removes beams smaller than scan.min_range from the
    // resulting cloud but we need that the point count is the same. We achieve
    // this by modifying a copy of scan.
    sensor_msgs::LaserScan copy_of_scan = scan;
    const double smaller_than_shortest = *(std::min_element(scan.ranges.begin(), scan.ranges.end())) - 1;
    copy_of_scan.range_min = smaller_than_shortest;
    projector.projectLaser(copy_of_scan, cloud, greater_than_longest, laser_geometry::channel_option::None);
  }

  int idx_start;
  int idx_end;
  int idx_increment;
  if (scan.angle_increment > 0)
  {
    idx_start = 0;
    idx_end = size;
    idx_increment = 1;
  }
  else
  {
    idx_start = size - 1;
    idx_end = -1;
    idx_increment = -1;
  }
  for (int i = idx_start; i != idx_end; i += idx_increment)
  {
    if (in_range[i] && in_range[circular_index(i + idx_increment, size)])
    {
      // point i and next point are included.
      profile.polygon.points.push_back(cloud.points[i]);
    }
    else if (in_range[i] && !in_range[circular_index(i + idx_increment, size)])
    {
      // point i is included, next point (which will not be part of
      // profile.polygon) is excluded.
      profile.polygon.points.push_back(cloud.points[i]);
      profile.exclude_segments.push_back(profile.polygon.points.size() - 1);
    }
  }
  
  if (!profile.exclude_segments.empty() && (profile.exclude_segments.front() == -1))
  {
    // If segment between last point and point 0 is excluded.
    // This happens if the first range(s) is(are) out of range.
    profile.exclude_segments[0] = profile.polygon.points.size() - 1;
  }

  normalizePlaceProfile(profile);
  return profile;
}

/* Return true if the map point is occupied.
 */
inline bool pointOccupied(const nav_msgs::OccupancyGrid& map, const int index)
{
  return (map.data[index] > OCCUPIED_THRESHOLD);
}

/* Return true if the map point is unknown.
 */
inline bool pointUnknown(const nav_msgs::OccupancyGrid& map, const int index)
{
  return (map.data[index] == -1);
}

/* Return the first non-free point from map center to map border with given angle.
 *
 * Return the first non-free point (i.e. first occupied or unknown point),
 * calculated by raytracing from the map center to the map border with the
 * Bresenham algorithm and stopping at the first encountered occupied or
 * unknown point.
 * Return true if the first non-free point is an obstacle and false if it is an
 * unknown point or if no non-free point was met (ray tracing up to map
 * border).
 *
 * map[in] occupancy grid 
 * angle[in] angle of the laser ray
 */
bool firstNonFree(const nav_msgs::OccupancyGrid& map, const double angle, geometry_msgs::Point32& point)
{
  static map_ray_caster::MapRayCaster ray_caster;

  const vector<size_t>& ray = ray_caster.getRayCastToMapBorder(angle, map.info.height, map.info.width);
  for (size_t i = 0; i < ray.size(); ++i)
  {
    size_t idx = ray[i];
    if (pointOccupied(map, idx))
    {
      map_ray_caster::indexToReal(map, idx, point);
      return true;
    }
    else if (pointUnknown(map, idx))
    {
      map_ray_caster::indexToReal(map, idx, point);
      return false;
    }
  }
  map_ray_caster::indexToReal(map, ray.back(), point);
  return false;
}

PlaceProfile costmapToPlaceProfile(const nav_msgs::OccupancyGrid& map)
{
  PlaceProfile profile;
  profile.header = map.header;
  profile.polygon.points.reserve(COSTMAP_DISCRETISATION_COUNT);

  // angle_min should be slightly greater than M_PI to be sure that the
  // pixel on the bottom half of the map is chosen if map height is even.
  const double angle_min = -M_PI + 1e-6;
  const double resolution = 2 * M_PI / COSTMAP_DISCRETISATION_COUNT;
  geometry_msgs::Point32 last_point;
  // last_point should be different than any point in the map.
  last_point.x = map.info.width * map.info.resolution;
  geometry_msgs::Point32 this_point;
  geometry_msgs::Point32 next_point;
  bool this_in_range = firstNonFree(map, angle_min, this_point);
  double next_angle = angle_min + resolution;
  for (size_t i = 0; i < COSTMAP_DISCRETISATION_COUNT; ++i)
  {
    bool next_in_range = firstNonFree(map, next_angle, next_point);
    if (this_in_range)
    {
      if ((this_point.x != last_point.x) || (this_point.y != last_point.y))
      {
        profile.polygon.points.push_back(this_point);
      }
    }
    else if (next_in_range)
    {
      profile.exclude_segments.push_back(profile.polygon.points.size() - 1);
    }
    next_angle += resolution;
    last_point = this_point;
    this_point = next_point;
    this_in_range = next_in_range;
  }
  if (!profile.exclude_segments.empty() && (profile.exclude_segments.front() == -1))
  {
    // If segment between last point and point 0 is excluded.
    // This happens if the first point(s) is(are) non-free.
    profile.exclude_segments[0] = profile.polygon.points.size() - 1;
  }

  return profile;
}

} // namespace lama_common

