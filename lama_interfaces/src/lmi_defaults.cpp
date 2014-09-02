#include <string>

#include <ros/ros.h>

#include <lama_interfaces/CLMIZernikeMoments.h>
#include <lama_interfaces/CLMIIntegralInvariant.h>
#include <lama_interfaces/CLMIPolygon.h>
#include <lama_interfaces/CPolygonDescriptor.h>

int main(int argc, char **argv)
{
  std::string map_file;

  ros::init(argc, argv, "lmi_defaults");
  ros::NodeHandle n;

  // TODO: transform to an anonymous parameter.
  n.param<std::string>("lama_map_file", map_file, "./default.dbs");

  CLMIZernikeMoments zm(map_file);
  zm.connect();
  CLMIIntegralInvariant ii(map_file);
  ii.connect();
  CLMIPolygon poly(map_file);
  poly.connect();

  ros::ServiceServer zmgetter = n.advertiseService("lmi_zernike_moments_get", &CLMIZernikeMoments::getter, &zm);
  ros::ServiceServer zmsetter = n.advertiseService("lmi_zernike_moments_set", &CLMIZernikeMoments::setter, &zm);
  ros::ServiceServer iigetter = n.advertiseService("lmi_integral_invariant_get", &CLMIIntegralInvariant::getter, &ii);
  ros::ServiceServer iisetter = n.advertiseService("lmi_integral_invariant_set", &CLMIIntegralInvariant::setter, &ii);
  ros::ServiceServer getterPoly = n.advertiseService("get_polygon", &CLMIPolygon::getter, &poly);
  ros::ServiceServer setterPoly = n.advertiseService("set_polygon", &CLMIPolygon::setter, &poly);
  ROS_INFO("map interface %s is ready with file %s", ros::this_node::getName().c_str(), map_file.c_str());

  ros::spin();

  return 0;
};
