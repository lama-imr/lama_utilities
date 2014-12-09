/* Utility functions to convert to and from PlaceProfile messages
 */

#ifndef _LAMA_MSGS_PLACE_PROFILE_CONVERSIONS_H_
#define _LAMA_MSGS_PLACE_PROFILE_CONVERSIONS_H_

#include <algorithm>

#include <ros/ros.h>
#include <geometry_msgs/Polygon.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/OccupancyGrid.h>
#include <laser_geometry/laser_geometry.h>

#include <lama_msgs/PlaceProfile.h>
#include <lama_msgs/place_profile_utils.h>

#include <map_ray_caster/map_ray_caster.h>

using std::vector;
using lama_msgs::PlaceProfile;

namespace lama {

/* Return a positive modulo (as Python does, even with negative numbers)
 * 
 * index must be greater than (-size).
 */
inline int32_t circular_index(const int32_t index, const size_t size)
{
  return (index + size) % size;
}

geometry_msgs::Polygon placeProfileToPolygon(const PlaceProfile& profile);

sensor_msgs::PointCloud placeProfileToPointCloud(const PlaceProfile& profile);

PlaceProfile laserScanToPlaceProfile(const sensor_msgs::LaserScan& scan, const double range_cutoff);

PlaceProfile costmapToPlaceProfile(const nav_msgs::OccupancyGrid& map);

} // namespace lama
#endif //  _LAMA_MSGS_PLACE_PROFILE_CONVERSIONS_H_
