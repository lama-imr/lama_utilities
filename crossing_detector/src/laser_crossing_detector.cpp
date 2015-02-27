#include <crossing_detector/laser_crossing_detector.h>

namespace crossing_detector
{

LaserCrossingDetector::LaserCrossingDetector(const double frontier_width, const double max_frontier_angle) :
  CrossingDetector(frontier_width, max_frontier_angle),
  range_cutoff_(0)
{
  ros::NodeHandle private_nh("~");
  private_nh.getParam("range_cutoff", range_cutoff_);
}

Crossing LaserCrossingDetector::crossingDescriptor(const LaserScan& scan, const bool normalize)
{
  double max_range = range_cutoff_;
  if (std::abs(range_cutoff_) < 1e-10)
  {
    max_range = 0.9 * scan.range_max;
  }

  PlaceProfile profile = lama_common::laserScanToPlaceProfile(scan, max_range);

  return CrossingDetector::crossingDescriptor(profile, normalize);
}

vector<Frontier> LaserCrossingDetector::frontiers(const LaserScan& scan, const bool normalize)
{
  double max_range = range_cutoff_;
  if (std::abs(range_cutoff_) < 1e-10)
  {
    max_range = 0.9 * scan.range_max;
  }

  PlaceProfile profile = lama_common::laserScanToPlaceProfile(scan, max_range);

  return CrossingDetector::frontiers(profile, normalize);
}

} // namespace crossing_detector


