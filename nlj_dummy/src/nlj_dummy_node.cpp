/**
 * ROS node using localizing and navigating jockeys for test purposes
 *
 */

#include <ros/ros.h>

#include <nlj_dummy/nj_dummy.h>
#include <nlj_dummy/lj_dummy.h>

#include <lama_interfaces/AddInterface.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dummy_jockey");
  ros::NodeHandle n;

  // Create the getter and setter services for Dummy descriptors.
  ros::ServiceClient client = n.serviceClient<lama_interfaces::AddInterface>("dummy_descriptor",
      "nlj_dummy/dummy_descriptor");
  // Run the jockeys.
  LJDummy loc_jockey("localizing_jockey");
  NJDummy nav_jockey("navigating_jockey");

  ROS_DEBUG("dummy_jockey started");
  ros::spin();
  return 0;
}

