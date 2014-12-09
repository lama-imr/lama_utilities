/* Wrapper for CrossingDetector
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

#include <lama_msgs/Crossing.h>
#include <lama_msgs/Frontier.h>
#include <lama_msgs/PlaceProfile.h>

#include <crossing_detector/crossing_detector.h>
#include <crossing_detector/wrapper_utils.h>

namespace bp = boost::python;
namespace rs = ros::serialization;

using crossing_detector::CrossingDetector;
using lama_msgs::Crossing;
using lama_msgs::Frontier;
using lama_msgs::PlaceProfile;
    
/* Wrapper class for CrossingDetector
 *
 * Functions arguments and returned types are strings and list of strings with
 * serialized content.
 */
class CrossingDetectorWrapper : public CrossingDetector
{
  public:

    CrossingDetectorWrapper(const double frontier_width, const double max_frontier_angle=0.785) :
      CrossingDetector(frontier_width, max_frontier_angle)
    {
    }

    std::string crossingDescriptor(const std::string str_profile, const bool normalize=false)
    {

      PlaceProfile profile = from_python<PlaceProfile>(str_profile);
      Crossing crossing = CrossingDetector::crossingDescriptor(profile, normalize);

      return to_python(crossing);
    }

    std::vector<std::string> frontiers(const std::string str_profile, const bool normalize=false)
    {
      PlaceProfile profile = from_python<PlaceProfile>(str_profile);
      std::vector<Frontier> frontiers = CrossingDetector::frontiers(profile, normalize);
      std::vector<std::string> str_frontiers;
      str_frontiers.reserve(frontiers.size());
      for (size_t i = 0; i < frontiers.size(); ++i)
      {
        str_frontiers.push_back(to_python(frontiers[i]));
      }
      return str_frontiers;
    }
};

BOOST_PYTHON_MODULE(crossing_detector_wrapper_cpp)
{
  bp::class_<CrossingDetectorWrapper>("CrossingDetectorWrapper", bp::init<double, double>())
    .def("crossingDescriptor", &CrossingDetectorWrapper::crossingDescriptor)
    .def("frontiers", &CrossingDetectorWrapper::frontiers)
    ;

  bp::to_python_converter<std::vector<std::string, std::allocator<std::string> >, vector_to_python<std::string> >();
}
