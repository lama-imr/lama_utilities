#include <boost/python.hpp>

#include <moveit/py_bindings_tools/roscpp_initializer.h>
#include <crossing_detector/crossing_detector.h>

using boost::python::class_;
using boost::python::def;
using boost::python::init;
using lama::crossing_detector::CrossingDetector;
    
class CrossingDetectorWrapper : protected moveit::py_bindings_tools::ROScppInitializer,
                                public CrossingDetector
{
  public:

    CrossingDetectorWrapper(const double frontier_width, const double max_frontier_angle=1.0) : moveit::py_bindings_tools::ROScppInitializer(), CrossingDetector(frontier_width, max_frontier_angle)
  {
  }
};

BOOST_PYTHON_MODULE(crossing_detector)
{
  class_<CrossingDetectorWrapper>("CrossingDetector", init<double, double>())
    .def("crossingDescriptor", &CrossingDetectorWrapper::crossingDescriptor)
    ;
}
