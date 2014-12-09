#include <ros/ros.h>
#include <ros/console.h> // to change the log level to debug

#include <lama_msgs/Crossing.h>

#include <goto_crossing/crossing_goer.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "goto_crossing");
  ros::NodeHandle nh("~");
  
  // Change log level.
  if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  {
    ros::console::notifyLoggerLevelsChanged();
  }

  goto_crossing::CrossingGoer crossing_goer;
  ros::Subscriber sub = nh.subscribe("crossing", 1, &goto_crossing::CrossingGoer::callback_goto_crossing, &crossing_goer);

  ros::spin();
}
