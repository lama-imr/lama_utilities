#ifndef SND_SND_PLANNER_H
#define SND_SND_PLANNER_H

#include <algorithm> // For std::max_element, min_element.
#include <cmath> // For cos, sin, sqrt, asin, abs, pow, min.
#include <math.h> // For round().
#include <string>
#include <list>
#include <limits>
#include <vector>
#include <utility> // For std::pair.

#include <boost/function.hpp>
#include <boost/format.hpp>

#include <snd/gap.h>
#include <snd/utilities.h>
#include <snd/valley.h>

// Those are defined somewhere.
#undef logError
#undef logDebug

namespace snd
{

typedef std::pair<double, double> Point2D;
typedef boost::function<void(const std::string&)> log_function_ptr;

class SNDPlanner
{
  public:

    SNDPlanner(double robot_radius, double safety_distance, double safety_distance_quick);

    /** Compute the direction and velocity with the SND algorithm.
     *
     * @param[in] goalx Position of the goal along the x-axis, i.e. in front of
     *                  the robot.
     * @param[in] goaly Position of the goal along the y-axis, positive if on
     *                  the left of the robot.
     * @param[in] scan 360-degree range-finder scan. The first range must be at
     *                 -pi (0 being in front of the robot). The scan
     *                 resolution is computed from the number of ranges, so
     *                 that all ranges are equally spaced (angular space) and
     *                 cover 2 * pi. The data should be prepared so that
     *                 measurement error are represented with a large value.
     *                 The data should be prepared so that the difference
     *                 between the ranges that fall into the sensor maximum
     *                 range and ranges that do no measure any obstacle is
     *                 greater than the minimum gap width
     *                 (2 * (robot_radius + safety_distance)), so that a valley
     *                 can be detected there.
     * @param[out] theta_traj Direction the robot should take to go to goal
     *                        while avoiding obstacles.
     * @param[out] vfactor Linear velocity factor, within [0, 1].
     */
    bool computeAngleAndVelocity(double goalx, double goaly, const std::vector<float>& scan, double& theta_traj, double& vfactor);

    // Algorithm parameters.
    // The radius of the minimum circle which contains the robot (m).
    // If set to 0 the footprint parameter will be used instead.
    // Default: 0.25.
    double robot_radius;

    // The polygon defining the robot footprint. Must be a simple polygon.
    std::vector<Point2D> footprint;

    // Minimum distance between robot bound and obstacle allowed to an obstacle
    // when driving at slow speed (m).
    // Note that this is the distance to the robot bound, as opposed to
    // safety_distance_quick.
    double safety_distance;

    // Minimum distance between robot center and obstacle allowed to an
    // obstacle when driving at high speed (m).
    // Note that this is the distance to the robot center, as opposed to
    // safety_distance.
    double safety_distance_quick;

    // Farouche factor (unitless, positive).
    // The higher, the most will the robot be repelled from obstacles.
    // Default: 2.0 (i.e. square of the threat measure).
    double farouche_factor;

    // Maximum speed allowed (m/s).
    // Default: 1.0.
    /* double max_speed; */

    // Maximum angular speed allowed (rad/s).
    // Default: 1 rad/s (approx. 60 degrees/s).
    /* double max_turn_rate; */

    // Maximum distance allowed from the final goal for the algorithm to stop.
    // Default: robot radius / 2
    /* double goal_position_tol; */

    // Maximum angular error from the final goal position for the algorithm to stop
    // Default: 0.2 (approx. 11 degrees).
    /* double goal_angle_tol; */

    // Maximum laser scan range (m).
    // Default: 5 m.
    double max_range;

    /** Set print error function pointer
     */
    void setLogErrorFunction(log_function_ptr f) {log_error_f_ = f;}

    /** Set print info function pointer
     */
    void setLogInfoFunction(log_function_ptr f) {log_info_f_ = f;}

    /** Set print debug function pointer
     */
    void setLogDebugFunction(log_function_ptr f) {log_debug_f_ = f;}

    /** Set print debug function pointer
     */
    void setLogSuperdebugFunction(log_function_ptr f) {log_superdebug_f_ = f;}

    std::list<Valley> valleys() const {return valleys_;}

    Valley bestValley() const;

  private :

    /** Return the list of gaps.
     * @param[in] scan 360-degree laser scan.
     */
    std::list<Gap> getGaps(const std::vector<float>& scan, size_t goal_sector) const;

    std::list<Valley> getValleys(const std::list<Gap>& gaps, const std::vector<float>& scan) const;
    bool isSpaceTraversable(const Gap& right_gap, const Gap& left_gap, size_t num_sectors) const;
    std::list<Valley>::const_iterator getBestValley(const std::list<Valley>& valleys) const;
    double computeProximityDeflection(const std::vector<float>& scan, double theta_desired, std::vector<double>& threats) const;
    double greatestThreat(double theta_traj, const std::vector<double>& threats);

    void logError(const std::string& text) const;
    void logInfo(const std::string& text) const;
    void logDebug(const std::string& text) const;
    void logSuperdebug(const std::string& text) const;

    // Minimum passage width the algorithm will try to exploit (m).
    // Default: 2 * (robot_radius + safety_distance).
    double min_gap_width_;
    std::list<Valley> valleys_;  //!< The valleys computed during last call to computeAngleAndVelocity.
    std::list<Valley>::const_iterator best_valley_;  //!< Best valley computed during last call to computeAngleAndVelocity.

    /** Print functions for different levels */
    log_function_ptr log_error_f_;
    log_function_ptr log_info_f_;
    log_function_ptr log_debug_f_;
    log_function_ptr log_superdebug_f_;
};

}

#endif /* SND_SND_PLANNER_H */
