#include <crossing_detector/costmap_crossing_detector.h>

namespace lama {
namespace crossing_detector {

CostmapCrossingDetector::CostmapCrossingDetector(const double frontier_width, const double max_frontier_angle) :
  CrossingDetector(frontier_width, max_frontier_angle)
{
}

Crossing CostmapCrossingDetector::crossingDescriptor(const OccupancyGrid& map, const bool normalize)
{
  PlaceProfile profile = costmapToPlaceProfile(map);

  return CrossingDetector::crossingDescriptor(profile, normalize);
}

vector<Frontier> CostmapCrossingDetector::frontiers(const OccupancyGrid& map, const bool normalize)
{
  PlaceProfile profile = costmapToPlaceProfile(map);

  return CrossingDetector::frontiers(profile, normalize);
}

} // namespace crossing_detector
} // namespace lama

