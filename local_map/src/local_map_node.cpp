/*
 * Local map builder
 * The local_map node takes as input a LaserScan message and outputs
 * a local map as OccupancyGrid. The local map orientation is the same
 * as the one of the global frame. The position of the map is the same
 * as the one of the LaserScan.
 *
 * Parameters:
 * - map_width, float, 200, map pixel width (x-direction)
 * - map_height, float, 200, map pixel height (y-direction)
 * - map_resolution, float, 0.020, map resolution (m/pixel)
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>

#include <local_map/map_builder.h>
#include <local_map/SaveMap.h>

ros::Publisher map_publisher;
local_map::MapBuilder* map_builder_ptr;

void handleLaserScan(sensor_msgs::LaserScan msg)
{
  map_builder_ptr->grow(msg);
  map_publisher.publish(map_builder_ptr->getMap());
}

bool save_map(local_map::SaveMap::Request& req,
    local_map::SaveMap::Response& res)
{
  return map_builder_ptr->saveMap(req.name);
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
  local_map::MapBuilder map_builder(map_width, map_height, map_resolution);
  map_builder_ptr = &map_builder;

  ros::Subscriber scanHandler = nh.subscribe<sensor_msgs::LaserScan>("scan", 1, handleLaserScan);
  map_publisher = nh.advertise<nav_msgs::OccupancyGrid>("local_map", 1, true);
  ros::ServiceServer service = nh.advertiseService("save_map", save_map);

  ros::spin();
}

