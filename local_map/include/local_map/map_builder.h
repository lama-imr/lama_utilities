#ifndef _LOCAL_MAP_MAP_BUILDER_H_
#define _LOCAL_MAP_MAP_BUILDER_H_

#include <cmath> // For std::exp.
#include <exception>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <angles/angles.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>

#include <map_ray_caster/map_ray_caster.h>

namespace local_map
{

using std::vector;
using std::abs;
using std::max;

class MapBuilder
{
  public:

    MapBuilder(int width, int height, double resolution);

    bool saveMap(const std::string& name) const;  //!< Save the map on disk

    void grow(const sensor_msgs::LaserScan& scan);

    nav_msgs::OccupancyGrid getMap() const {return map_;}

  private:

    bool updateMap(const sensor_msgs::LaserScan& scan, long int dx, long int dy, double theta);
    bool getRayCastToObstacle(const nav_msgs::OccupancyGrid& map, double angle, double range, vector<size_t>& raycast);
    void updatePointOccupancy(bool occupied, size_t idx, vector<int8_t>& occupancy, vector<double>& log_odds) const;

    /** Update occupancy and log odds for a list of a points
    */
    inline void updatePointsOccupancy(bool occupied, const vector<size_t>& indexes, vector<int8_t>& occupancy, vector<double>& log_odds)
    {
      vector<size_t>::const_iterator idx = indexes.begin();
      for (; idx != indexes.end(); ++idx)
      {
        updatePointOccupancy(occupied, *idx, occupancy, log_odds);
      }
    }

    // ROS parameters.
    double angle_resolution_;  //!< Angle resolution for the ray cast lookup (rad).
                               //!< Defaults to 0.25 deg equivalent.
    double p_occupied_when_laser_;  //!< Probability that a point is occupied
                                    //!< when the laser ranger says so.
                                    //!< Defaults to 0.9.
    double p_occupied_when_no_laser_ ;  //!< Probability that a point is
                                        //!< occupied when the laser ranger
                                        //!< says it's free.
                                        //!< Defaults to 0.3.
    double large_log_odds_;  //!< Large log odds used with probability 0 and 1.
                             //!< The greater, the more inertia.
                             //!< Defaults to 100.
    double max_log_odds_for_belief_;  //!< Max log odds used to compute the
                                      //!< belief (exp(max_log_odds_for_belief)
                                      //!< should not overflow).
                                      //!< Defaults to 20.


    // Internals.
    tf::TransformListener tf_listerner_;
    std::string world_frame_id_; //!< frame_id of the world frame
    tf::TransformBroadcaster tr_broadcaster_;  //!< To broadcast the transform from LaserScan to local map.
    bool has_frame_id_;  //!< true if map frame_id was already set
    std::string map_frame_id_;  //!< map frame id in tf
    double xinit_;  //!< Map x position at initialization
    double yinit_;  //!< Map y position at initialization
    long int last_xmap_;  //!< Map integer x position at last map move
    long int last_ymap_;  //!< Map integer y position at last map move

    nav_msgs::OccupancyGrid map_; //!< local map with fixed orientation
    std::vector<double> log_odds_;  //!< log odds ratios for the binary Bayes filter
                                    //!< log_odd = log(p(x) / (1 - p(x)))
    map_ray_caster::MapRayCaster ray_caster_;  //!< Ray casting with cache.
};

/* Return the offset from row and column number for a row-major array
 *
 * offset, row and column can be out of the map range.
 */
inline int offsetFromRowColNoRangeCheck(int row, int col, size_t ncol)
{
  return (row * ncol) + col;
}

template <typename T>
void moveAndCopyImage(int fill, int dx, int dy, unsigned int ncol, std::vector<T>& map);

} // namespace local_map

#endif // # ifndef _LOCAL_MAP_MAP_BUILDER_H_
