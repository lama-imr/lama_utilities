#include <crossing_detector/laser_crossing_detector.h>

namespace lama {
namespace crossing_detector {

LaserCrossingDetector::LaserCrossingDetector(const double frontier_width, const double max_frontier_angle) :
  CrossingDetector(frontier_width, max_frontier_angle),
  max_frontier_dist_(0.0)
{
}

Crossing LaserCrossingDetector::crossingDescriptor(const LaserScan& scan, const bool normalize)
{
  double max_range = max_frontier_dist_;
  if (max_frontier_dist_ == 0)
  {
    max_range = 0.9 * scan.range_max;
  }

  PlaceProfile profile = laserScanToPlaceProfile(scan, max_range);

  return CrossingDetector::crossingDescriptor(profile, normalize);
}

vector<Frontier> LaserCrossingDetector::frontiers(const LaserScan& scan, const bool normalize)
{
  double max_range = max_frontier_dist_;
  if (max_frontier_dist_ == 0)
  {
    max_range = 0.9 * scan.range_max;
  }

  PlaceProfile profile = laserScanToPlaceProfile(scan, max_range);

  return CrossingDetector::frontiers(profile, normalize);
}

} // namespace crossing_detector
} // namespace lama


