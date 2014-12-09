#include <ros/ros.h>
#include <ros/console.h> // to change the log level to debug

#include <lama_msgs/Crossing.h>
#include <lama_msgs/PlaceProfile.h>

#include <crossing_detector/laser_crossing_detector.h>
#include <crossing_detector/LaserDetectCrossing.h>

crossing_detector::LaserCrossingDetector* detector;

bool detect_crossing_callback(crossing_detector::LaserDetectCrossing::Request& req, crossing_detector::LaserDetectCrossing::Response& res)
{
  detector->setFrontierWidth(req.frontier_width);
  detector->setMaxFrontierAngle(req.max_frontier_angle);
  detector->setMinRelevance(req.min_relevance);
  res.crossing = detector->crossingDescriptor(req.scan, true);
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

  detector = new crossing_detector::LaserCrossingDetector(0);

  ros::ServiceServer service_server = nh.advertiseService("detect_crossing", detect_crossing_callback);

  ros::spin();
  return 0;
}
