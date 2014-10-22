/* Wrapper for CrossingDetector
 *
 * Parameter passing occurs through STRING OR STRING BUFFER??. The serialization/deserialization
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

namespace bp = boost::python;
namespace rs = ros::serialization;

using lama::crossing_detector::CrossingDetector;
using lama_msgs::Crossing;
using lama_msgs::Frontier;
using lama_msgs::PlaceProfile;
    
// Extracted from https://gist.github.com/avli/b0bf77449b090b768663.
template<class T>
struct vector_to_python
{
  static PyObject* convert(const std::vector<T>& vec)
  {
    boost::python::list* l = new boost::python::list();
    for(std::size_t i = 0; i < vec.size(); i++)
      (*l).append(vec[i]);

    return l->ptr();
  }
};

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

    /* Read a ROS message from a serialized string.
     */
    template <typename M>
    M from_python(const std::string str_msg)
    {
      size_t serial_size = str_msg.size();
      boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
      for (size_t i = 0; i < serial_size; ++i)
      {
        buffer[i] = str_msg[i];
      }
      rs::IStream stream(buffer.get(), serial_size);
      M msg;
      rs::Serializer<M>::read(stream, msg);
      return msg;
    }

    /* Write a ROS message into a serialized string.
     */
    template <typename M>
    std::string to_python(const M& msg)
    {
      size_t serial_size = rs::serializationLength(msg);
      boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
      rs::OStream stream(buffer.get(), serial_size);
      rs::serialize(stream, msg);
      std::string str_msg;
      str_msg.reserve(serial_size);
      for (size_t i = 0; i < serial_size; ++i)
      {
        str_msg.push_back(buffer[i]);
      }
      return str_msg;
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
