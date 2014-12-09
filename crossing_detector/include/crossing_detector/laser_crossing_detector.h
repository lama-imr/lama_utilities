/*
 * Crossing detector based on LaserScan
 */

#ifndef _CROSSING_DETECTOR_LASER_CROSSING_DETECTOR_H_
#define _CROSSING_DETECTOR_LASER_CROSSING_DETECTOR_H_

#include <cmath> // for std::abs

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <lama_common/place_profile_conversions.h>

#include <crossing_detector/crossing_detector.h>

namespace crossing_detector {

using sensor_msgs::LaserScan;

class LaserCrossingDetector : public CrossingDetector
{
  public:

    LaserCrossingDetector(const double frontier_width, const double max_frontier_angle=0.785);

    Crossing crossingDescriptor(const LaserScan& scan, const bool normalize=false);
    vector<lama_msgs::Frontier> frontiers(const LaserScan& scan, const bool normalize=false);

    double getMaxFrontierDistance() const {return max_frontier_dist_;}
    void setMaxFrontierDistance(const double value) {max_frontier_dist_ = value;}
    
  private:

    // Parameters shown outside.
    double max_frontier_dist_;  //!> Threshold for a laser beam to be considered infinite.
                                //!> If the user does not set the value, 90% of range_max will be used.

    // Internals.

    // Visibility change (not necessary).
    using CrossingDetector::crossingDescriptor;
    using CrossingDetector::frontiers;
};

} // namespace crossing_detector

#endif // _CROSSING_DETECTOR_LASER_CROSSING_DETECTOR_H_
