#ifndef _LOCAL_MAP_MAP_BUILDER_H_
#define _LOCAL_MAP_MAP_BUILDER_H_

#include <string>
#include <vector>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <angles/angles.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>

#include <local_map/map_ray_caster.h>

namespace lama {
namespace local_map {

using std::vector;
using std::abs;
using std::max;

class MapBuilder
{
	public:

		MapBuilder(const int width, const int height, const double resolution);

		nav_msgs::OccupancyGrid getMap() const;
		bool saveMap(const std::string& name) const;  //!> Save the map on disk

		void grow(const sensor_msgs::LaserScan& scan);

	private:

		bool updateMap(const sensor_msgs::LaserScan& scan, const long int dx, const long int dy, const double theta);
    bool getRayCastToObstacle(const nav_msgs::OccupancyGrid& map, const double angle, const double range, vector<size_t>& raycast);

		tf::TransformListener tf_listerner_;
		std::string world_frame_id_; //!> frame_id of the world frame
		bool has_frame_id_;  //!> true if map frame_id was already set
		std::string map_frame_id_;  //!> map frame id in tf
		double xinit_;  //!> Map x position at initialization
		double yinit_;  //!> Map y position at initialization
		long int last_xmap_;  //!> Map integer x position at last map move
		long int last_ymap_;  //!> Map integer y position at last map move

		nav_msgs::OccupancyGrid map_; //!> local map with fixed orientation
		std::vector<double> log_odds_;  //>! log odds ratios for the binary Bayes filter
		                                 //>! log_odd = log(p(x) / (1 - p(x)))
    MapRayCaster ray_caster_;  //!> Ray casting with cache
};

/* Return the offset from row and column number for a row-major array
 *
 * offset, row and column can be out of the map range.
 */
inline int offsetFromRowColNoRangeCheck(const int row, const int col, const size_t ncol)
{
	return (row * ncol) + col;
}

template <typename T>
void moveAndCopyImage(const int fill, const int dx, const int dy, const unsigned int ncol, std::vector<T>& map);

} // namespace local_map
} // namespace lama

#endif // # ifndef _LOCAL_MAP_MAP_BUILDER_H_
