#include <crossing_detector/costmap_crossing_detector.h>

namespace crossing_detector
{

CostmapCrossingDetector::CostmapCrossingDetector(double frontier_width, double max_frontier_angle) :
  CrossingDetector(frontier_width, max_frontier_angle)
{
}

Crossing CostmapCrossingDetector::crossingDescriptor(const OccupancyGrid& map, double range_cutoff)
{
  PlaceProfile profile = lama_common::costmapToPlaceProfile(map, range_cutoff);

  return CrossingDetector::crossingDescriptor(profile);
}

vector<Frontier> CostmapCrossingDetector::frontiers(const OccupancyGrid& map, double range_cutoff)
{
  PlaceProfile profile = lama_common::costmapToPlaceProfile(map, range_cutoff);

  return CrossingDetector::frontiers(profile);
}

} // namespace crossing_detector

