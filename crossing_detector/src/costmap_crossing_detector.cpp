#include <crossing_detector/costmap_crossing_detector.h>

namespace crossing_detector {

CostmapCrossingDetector::CostmapCrossingDetector(const double frontier_width, const double max_frontier_angle) :
  CrossingDetector(frontier_width, max_frontier_angle)
{
}

Crossing CostmapCrossingDetector::crossingDescriptor(const OccupancyGrid& map)
{
  PlaceProfile profile = lama_common::costmapToPlaceProfile(map);

  return CrossingDetector::crossingDescriptor(profile);
}

vector<Frontier> CostmapCrossingDetector::frontiers(const OccupancyGrid& map)
{
  PlaceProfile profile = lama_common::costmapToPlaceProfile(map);

  return CrossingDetector::frontiers(profile);
}

} // namespace crossing_detector

