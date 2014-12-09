#include <ros/ros.h>
#include <ros/console.h> // to change the log level to debug

#include <lama_msgs/Crossing.h>
#include <lama_msgs/PlaceProfile.h>

#include <crossing_detector/crossing_detector.h>
#include <crossing_detector/DetectCrossing.h>

crossing_detector::CrossingDetector* detector;

bool detect_crossing_callback(crossing_detector::DetectCrossing::Request& req, crossing_detector::DetectCrossing::Response& res)
{
  detector->setFrontierWidth(req.frontier_width);
  detector->setMaxFrontierAngle(req.max_frontier_angle);
  detector->setMinRelevance(req.min_relevance);
  res.crossing = detector->crossingDescriptor(req.profile, true);
  return true;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "crossing_detector");
  ros::NodeHandle nh("~");
  
  // Change log level.
  if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  {
    ros::console::notifyLoggerLevelsChanged();
  }

  detector = new crossing_detector::CrossingDetector(0);

  ros::ServiceServer service_server = nh.advertiseService("detect_crossing", detect_crossing_callback);

  ros::spin();
  return 0;
}
