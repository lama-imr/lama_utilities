#include <iostream> // DEBUG
#include <boost/python.hpp>

#include <moveit/py_bindings_tools/roscpp_initializer.h>
#include <crossing_detector/crossing_detector.h>
#include <lama_msgs/Crossing.h>
#include <lama_msgs/PlaceProfile.h>

using boost::python::class_;
using boost::python::def;
using boost::python::init;
using boost::python::return_value_policy;
using boost::python::copy_non_const_reference;
using lama::crossing_detector::CrossingDetector;
using lama_msgs::CrossingConstPtr;
using lama_msgs::PlaceProfile;
using lama_msgs::PlaceProfileConstPtr;
    
class CrossingDetectorWrapper : public CrossingDetector
{
  public:

    CrossingDetectorWrapper(const double frontier_width, const double max_frontier_angle=1.0) : CrossingDetector(frontier_width, max_frontier_angle) {}

    // const lama_msgs::Crossing crossingDescriptor(const PlaceProfileConstPtr& profile, const bool normalize=false)
    // {
    //   lama_msgs::Crossing crossing = CrossingDetector::crossingDescriptor(*profile, normalize);
    //   return crossing;
    // }
    
    bool testCall0() {return true;}
    //bool testCall1(const PlaceProfileConstPtr profile) {return true;}
    bool testCall1(const boost::python::object& profile)
    {
      profile.attr("exclude_segments");
      profile.ptr()
      std::cout << profile.attr("exclude_segments") << std::endl;

      return true;}
};

class CrossingDetectorWrapperMoveit : protected moveit::py_bindings_tools::ROScppInitializer,
  public CrossingDetector
{
  public:

    CrossingDetectorWrapperMoveit(const double frontier_width, const double max_frontier_angle=1.0) :
      moveit::py_bindings_tools::ROScppInitializer(),
      CrossingDetector(frontier_width, max_frontier_angle) {}
};

BOOST_PYTHON_MODULE(crossing_detector)
{
  class_<CrossingDetectorWrapper>("CrossingDetector", init<double, double>())
    .def("crossingDescriptor", &CrossingDetectorWrapper::crossingDescriptor)
    .def("testCall0", &CrossingDetectorWrapper::testCall0)
    .def("testCall1", &CrossingDetectorWrapper::testCall1)
    ;
}
