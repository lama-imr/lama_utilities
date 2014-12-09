#include <crossing_detector/laser_crossing_detector.h>

namespace crossing_detector {

LaserCrossingDetector::LaserCrossingDetector(const double frontier_width, const double max_frontier_angle) :
  CrossingDetector(frontier_width, max_frontier_angle)
{
  ros::NodeHandle private_nh("~");
  if (!private_nh.getParamCached("max_frontier_distance", max_frontier_dist_))
  {
    max_frontier_dist_ = 0.0;
  }
}

Crossing LaserCrossingDetector::crossingDescriptor(const LaserScan& scan, const bool normalize)
{
  double max_range = max_frontier_dist_;
  if (std::abs(max_frontier_dist_) < 1e-10)
  {
    max_range = 0.9 * scan.range_max;
  }

  PlaceProfile profile = lama_common::laserScanToPlaceProfile(scan, max_range);

  return CrossingDetector::crossingDescriptor(profile, normalize);
}

vector<Frontier> LaserCrossingDetector::frontiers(const LaserScan& scan, const bool normalize)
{
  double max_range = max_frontier_dist_;
  if (max_frontier_dist_ == 0)
  {
    max_range = 0.9 * scan.range_max;
  }

  PlaceProfile profile = lama_common::laserScanToPlaceProfile(scan, max_range);

  return CrossingDetector::frontiers(profile, normalize);
}

} // namespace crossing_detector


