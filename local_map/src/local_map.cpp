/*
 * Local map builder
 * The local_map node takes as input a LaserScan message and outputs
 * a local map as OccupancyGrid. The local map orientation is the same
 * as the one of the global frame.
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
	ros::NodeHandle n;

	MapBuilder mapBuilder(200, 200, 0.020);
	mapBuilderPtr = &mapBuilder;

	ros::Subscriber scanHandler = n.subscribe<sensor_msgs::LaserScan> ("scan", 50, handleLaserScan);
	mapPub = n.advertise<nav_msgs::OccupancyGrid> ("local_map",50, true);
	ros::ServiceServer service = n.advertiseService(n.getNamespace() + "/save_map", save_map);

	//ROS_INFO(n.getNamespace() + " started.");
	ROS_INFO("started.");
	ros::spin();
}

