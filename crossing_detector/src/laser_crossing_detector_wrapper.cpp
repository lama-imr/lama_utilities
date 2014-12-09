/* Wrapper for LaserCrossingDetector
 *
 * Parameter passing occurs through strings. The serialization/deserialization
 * in Python is done in crossing_detector/crossing_detector.py, class CrossingDetector.
 */
#include <algorithm>
#include <string>
#include <vector>
#include <sstream>

#include <boost/python.hpp>
#include <python2.7/Python.h>

#include <ros/serialization.h>

#include <sensor_msgs/LaserScan.h>
#include <lama_msgs/Crossing.h>
#include <lama_msgs/Frontier.h>

#include <crossing_detector/laser_crossing_detector.h>
#include <crossing_detector/wrapper_utils.h>

namespace bp = boost::python;
namespace rs = ros::serialization;

using sensor_msgs::LaserScan;
using crossing_detector::LaserCrossingDetector;
using lama_msgs::Crossing;
using lama_msgs::Frontier;
    
/* Wrapper class for LaserCrossingDetector
 *
 * Functions arguments and returned types are strings and list of strings with
 * serialized content.
 */
class LaserCrossingDetectorWrapper : public LaserCrossingDetector
{
  public:

    LaserCrossingDetectorWrapper(const double frontier_width, const double max_frontier_angle=0.785) :
      LaserCrossingDetector(frontier_width, max_frontier_angle)
    {
    }

    std::string crossingDescriptor(const std::string str_scan, const bool normalize=false)
    {

      LaserScan scan = from_python<LaserScan>(str_scan);
      Crossing crossing = LaserCrossingDetector::crossingDescriptor(scan, normalize);

      return to_python(crossing);
    }

    std::vector<std::string> frontiers(const std::string str_scan, const bool normalize=false)
    {
      LaserScan scan = from_python<LaserScan>(str_scan);
      std::vector<Frontier> frontiers = LaserCrossingDetector::frontiers(scan, normalize);
      std::vector<std::string> str_frontiers;
      str_frontiers.reserve(frontiers.size());
      for (size_t i = 0; i < frontiers.size(); ++i)
      {
        str_frontiers.push_back(to_python(frontiers[i]));
      }
      return str_frontiers;
    }
};

BOOST_PYTHON_MODULE(laser_crossing_detector_wrapper_cpp)
{
  bp::class_<LaserCrossingDetectorWrapper>("LaserCrossingDetectorWrapper", bp::init<double, double>())
    .def("crossingDescriptor", &LaserCrossingDetectorWrapper::crossingDescriptor)
    .def("frontiers", &LaserCrossingDetectorWrapper::frontiers)
    ;

  bp::to_python_converter<std::vector<std::string, std::allocator<std::string> >, vector_to_python<std::string> >();
}
