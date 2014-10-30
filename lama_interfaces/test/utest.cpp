#include <cmath>
#include <stdexcept>

#include <gtest/gtest.h>

#include <ros/ros.h>
#include <ros/message.h>
#include <ros/serialization.h>
#include <sensor_msgs/LaserScan.h>

#include <lama_interfaces/AddInterface.h>
#include <lama_interfaces/GetVectorDouble.h>
#include <lama_interfaces/SetVectorDouble.h>
#include <lama_interfaces/GetVectorLaserScan.h>
#include <lama_interfaces/SetVectorLaserScan.h>

#define PREPARE_GETTER_SETTER(iface_name, msg) \
  std::string get_service_name; \
  std::string set_service_name; \
  if (!initMapInterface(#iface_name, "lama_interfaces/Get" #msg, "lama_interfaces/Set" #msg, get_service_name, set_service_name)) \
  { \
    throw std::runtime_error("Could not create map interface iface_name"); \
  } \
  ros::NodeHandle nh; \
  ros::ServiceClient getter = nh.serviceClient<lama_interfaces::Get##msg>(get_service_name); \
  getter.waitForExistence(); \
  ros::ServiceClient setter = nh.serviceClient<lama_interfaces::Set##msg>(set_service_name); \
  setter.waitForExistence();

bool initMapInterface(const std::string& interface_name,
    const std::string& get_service_message,
    const std::string& set_service_message,
    std::string& get_service_name,
    std::string& set_service_name)
{
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<lama_interfaces::AddInterface>("interface_factory");
  client.waitForExistence();
  lama_interfaces::AddInterface srv;
  srv.request.interface_name = interface_name;
  srv.request.interface_type = lama_interfaces::AddInterfaceRequest::SERIALIZED;
  srv.request.get_service_message = get_service_message;
  srv.request.set_service_message = set_service_message;
  if (!client.call(srv))
  {
    ROS_ERROR("Failed to create the Lama interface %s", interface_name.c_str());
    return false;
  }
  get_service_name = srv.response.get_service_name;
  set_service_name = srv.response.set_service_name;
  return true;
}

testing::AssertionResult headerEqual(const std_msgs::Header& a, const std_msgs::Header& b)
{
  if (a.seq != b.seq)
  {
    return testing::AssertionFailure() << "seq differ: " << a.seq << " is not " << b.seq;
  }

  // TODO: test all attributes.

  return testing::AssertionSuccess();
}

testing::AssertionResult laserScanEqual(sensor_msgs::LaserScan& a, sensor_msgs::LaserScan& b)
{
  if (!headerEqual(a.header, b.header))
  {
    return testing::AssertionFailure() << "header differ";
  }

  if (a.angle_min != b.angle_min)
  {
    return testing::AssertionFailure() << "angle_min differ: " << a.angle_min << " is not " << b.angle_min;
  }

  if (a.angle_increment != b.angle_increment)
  {
    return testing::AssertionFailure() << "angle_increment differ: " << a.angle_increment << " is not " << b.angle_increment;
  }

  // TODO: test all attributes.

  if (a.angle_max != b.angle_max)
  {
    return testing::AssertionFailure() << "angle_max differ: " << a.angle_max << " is not " << b.angle_max;
  }

  if (a.ranges != b.ranges)
  {
    return testing::AssertionFailure() << "ranges differ";
  }

  return testing::AssertionSuccess();
}

TEST(TestSuite, testVectorDouble)
{
  PREPARE_GETTER_SETTER(double, VectorDouble);

  lama_interfaces::SetVectorDouble set_srv;
  set_srv.request.descriptor.push_back(45.69);
  set_srv.request.descriptor.push_back(-46.3);

  setter.call(set_srv);
  
  lama_interfaces::GetVectorDouble get_srv;
  get_srv.request.id = set_srv.response.id;
  getter.call(get_srv);

  ASSERT_EQ(set_srv.request.descriptor.size(), get_srv.response.descriptor.size());
  for (size_t i = 0; i < set_srv.request.descriptor.size(); ++i)
  {
    EXPECT_EQ(set_srv.request.descriptor[i], get_srv.response.descriptor[i]);
  }
}
 
TEST(TestSuite, testVectorLaserScan)
{
  PREPARE_GETTER_SETTER(laserscan, VectorLaserScan);

  sensor_msgs::LaserScan scan0;
  scan0.header.seq = 1;
  scan0.header.stamp = ros::Time::now();
  scan0.header.frame_id = "frame0";
  scan0.angle_min = -M_PI;
  scan0.angle_max = M_PI;
  scan0.range_max = 10.;
  scan0.ranges.push_back(0);
  scan0.ranges.push_back(1);
  sensor_msgs::LaserScan scan1;
  scan1.header.seq = 2;
  scan1.header.stamp = ros::Time::now();
  scan1.header.frame_id = "frame1";
  scan1.angle_min = -M_PI / 2;
  scan1.angle_max = M_PI / 2;
  scan1.range_max = 9.;
  scan1.ranges.push_back(2);
  scan1.ranges.push_back(3);
  
  lama_interfaces::SetVectorLaserScan set_srv;
  set_srv.request.descriptor.push_back(scan0);
  set_srv.request.descriptor.push_back(scan1);
  setter.call(set_srv);

  lama_interfaces::GetVectorLaserScan get_srv;
  get_srv.request.id = set_srv.response.id;
  getter.call(get_srv);

  ASSERT_EQ(set_srv.request.descriptor.size(), get_srv.response.descriptor.size());
  for (size_t i = 0; i < set_srv.request.descriptor.size(); ++i)
  {
    EXPECT_TRUE(laserScanEqual(set_srv.request.descriptor[i], get_srv.response.descriptor[i]));
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_cpp_interface_factory");
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

