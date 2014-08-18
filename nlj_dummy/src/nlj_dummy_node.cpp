/**
 * ROS node using localizing and navigating jockeys for test purposes
 *
 */

#include <ros/ros.h>

#include <nlj_dummy/nj_dummy.h>
#include <nlj_dummy/lj_dummy.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dummy_jockey");
  ros::NodeHandle n;

  LJDummy loc_jockey("localizing_jockey");
  NJDummy nav_jockey("navigating_jockey");

  ROS_DEBUG("dummy_jockey started");
  ros::spin();
  return 0;
}

