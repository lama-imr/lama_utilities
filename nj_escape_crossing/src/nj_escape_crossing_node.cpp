#include <ros/ros.h>
#include <ros/console.h>

#include <nj_escape_crossing/crossing_escaper.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "nj_escape_crossing");
  
  // Change log level.
  if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  {
    ros::console::notifyLoggerLevelsChanged();
  }

  lama::nj_escape_crossing::CrossingEscaper jockey(ros::this_node::getName() + "_jockey");
  ROS_INFO("%s started (with server %s)", ros::this_node::getName().c_str(), jockey.getName().c_str());

  ros::spin();
}
