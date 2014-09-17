/*
 * Crossing detector based on OccupancyGrid centered on robot.
 */

#ifndef _CROSSING_DETECTOR_COSTMAP_CROSSING_DETECTOR_H_
#define _CROSSING_DETECTOR_COSTMAP_CROSSING_DETECTOR_H_

#include <nav_msgs/OccupancyGrid.h>

#include <lama_msgs/place_profile_conversions.h>

#include <crossing_detector/crossing_detector.h>

namespace lama {
namespace crossing_detector {

using nav_msgs::OccupancyGrid;

class CostmapCrossingDetector : public CrossingDetector
{
  public:
    
    CostmapCrossingDetector(const double frontier_width, const double max_frontier_angle=0.785);

    Crossing crossingDescriptor(const OccupancyGrid& map, const bool normalize=false);
    vector<Frontier> frontiers(const OccupancyGrid& map, const bool normalize=false);

  private:
    
    // Visibility change (not necessary).
    using CrossingDetector::crossingDescriptor;
    using CrossingDetector::frontiers;
};

} // namespace crossing_detector
} // namespace lama

#endif //  _CROSSING_DETECTOR_COSTMAP_CROSSING_DETECTOR_H_

