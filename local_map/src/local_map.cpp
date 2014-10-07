/*
 * Local map builder
 * The local_map node takes as input a LaserScan message and outputs
 * a local map as OccupancyGrid. The local map orientation is the same
 * as the one of the global frame.
 *
 * Parameters:
 * - map_width, float, 200, map pixel width (x-direction)
 * - map_width, float, 200, map pixel height (y-direction)
 * - map_resolution, float, 0.020, map resolution (m/pixel)
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>

#include "local_map/map_builder.h"
#include "local_map/SaveMap.h"

ros::Publisher mapPub;
MapBuilder* mapBuilderPtr;

void handleLaserScan(sensor_msgs::LaserScan msg)
{
	mapBuilderPtr->grow(msg);
	mapPub.publish(mapBuilderPtr->getMap());
}

bool save_map(local_map::SaveMap::Request& req,
		local_map::SaveMap::Response& res)
{
	return mapBuilderPtr->saveMap(req.name);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "local_map");
	ros::NodeHandle nh("~");

  double map_width;
  double map_height;
  double map_resolution;
  nh.param<double>("map_width", map_width, 200);
  nh.param<double>("map_height", map_height, 200);
  nh.param<double>("map_resolution", map_resolution, 0.020);
	MapBuilder mapBuilder(map_width, map_height, map_resolution);
	mapBuilderPtr = &mapBuilder;

	ros::Subscriber scanHandler = nh.subscribe<sensor_msgs::LaserScan>("scan", 1, handleLaserScan);
	mapPub = nh.advertise<nav_msgs::OccupancyGrid>("local_map", 1, true);
	ros::ServiceServer service = nh.advertiseService("save_map", save_map);

	ros::spin();
}

