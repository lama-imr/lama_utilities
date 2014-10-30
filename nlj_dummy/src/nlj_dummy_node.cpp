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
  ros::ServiceClient client = n.serviceClient<lama_interfaces::AddInterface>("interface_factory");
  client.waitForExistence();
  lama_interfaces::AddInterface srv;
  srv.request.interface_name = "dummy_descriptor";
  srv.request.interface_type = lama_interfaces::AddInterfaceRequest::SERIALIZED;
  srv.request.get_service_message = "nlj_dummy/GetDummyDescriptor";
  srv.request.set_service_message = "nlj_dummy/SetDummyDescriptor";
  if (!client.call(srv))
  {
    ROS_ERROR("Failed to create the Lama interface %s", srv.request.interface_name.c_str());
    return 1;
  }

  // Run the jockeys.
  LJDummy loc_jockey("localizing_jockey", srv.request.interface_name, srv.response.set_service_name);
  NJDummy nav_jockey("navigating_jockey", srv.response.get_service_name);

  ROS_DEBUG("dummy_jockey started");
  ros::spin();
  return 0;
}

