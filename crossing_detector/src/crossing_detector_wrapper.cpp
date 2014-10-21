#include <string>
#include <vector>
#include <iostream> // DEBUG

#include <boost/python.hpp>
#include <python2.7/Python.h>

#include <geometry_msgs/Point32.h>
#include <lama_msgs/Crossing.h>
#include <lama_msgs/PlaceProfile.h>

#include <crossing_detector/crossing_detector.h>

namespace bp = boost::python;
using lama::crossing_detector::CrossingDetector;
using lama_msgs::PlaceProfile;
    
class CrossingDetectorWrapper : public CrossingDetector
{
  public:

    CrossingDetectorWrapper(const double frontier_width, const double max_frontier_angle=1.0) : CrossingDetector(frontier_width, max_frontier_angle) {}

    const lama_msgs::Crossing crossingDescriptor(const bp::object object, const bool normalize=false)
    {
      PlaceProfile profile;
      bp::object header_attr = object.attr("header");
      profile.header.frame_id = bp::extract<std::string>(header_attr.attr("frame_id"));
      profile.header.seq = bp::extract<uint32_t>(header_attr.attr("seq"));
      profile.header.stamp.sec = bp::extract<uint32_t>(header_attr.attr("stamp").attr("secs"));
      profile.header.stamp.nsec = bp::extract<uint32_t>(header_attr.attr("stamp").attr("nsecs"));
      bp::object polygon_attr = object.attr("polygon");
      bp::object points_attr = polygon_attr.attr("points");
      for (size_t i = 0; i < bp::len(points_attr); ++i)
      {
        geometry_msgs::Point32 point;
        point.x = bp::extract<double>(points_attr[i].attr("x"));
        point.y = bp::extract<double>(points_attr[i].attr("y"));
        point.z = bp::extract<double>(points_attr[i].attr("z"));
        profile.polygon.points.push_back(point);
      }

      bp::object exclude_segments_attr = object.attr("exclude_segments");
      for (size_t i = 0; i < bp::len(exclude_segments_attr); ++i)
      {
        profile.exclude_segments.push_back(bp::extract<int32_t>(exclude_segments_attr[i]));
      }
      lama_msgs::Crossing crossing = CrossingDetector::crossingDescriptor(profile, normalize);
      return crossing;
    }
    
    bool testCall0() {return true;}
    bool testCall1(const bp::object object)
    {
      PlaceProfile profile;
      bp::object header_attr = object.attr("header");
      profile.header.frame_id = bp::extract<std::string>(header_attr.attr("frame_id"));
      profile.header.seq = bp::extract<uint32_t>(header_attr.attr("seq"));
      profile.header.stamp.sec = bp::extract<uint32_t>(header_attr.attr("stamp").attr("secs"));
      profile.header.stamp.nsec = bp::extract<uint32_t>(header_attr.attr("stamp").attr("nsecs"));
      bp::object polygon_attr = object.attr("polygon");
      bp::object points_attr = polygon_attr.attr("points");
      for (size_t i = 0; i < bp::len(points_attr); ++i)
      {
        geometry_msgs::Point32 point;
        point.x = bp::extract<double>(points_attr[i].attr("x"));
        point.y = bp::extract<double>(points_attr[i].attr("y"));
        point.z = bp::extract<double>(points_attr[i].attr("z"));
        profile.polygon.points.push_back(point);
      }
      std::cout << profile.header.frame_id << std::endl;
      std::cout << profile.header.seq << std::endl;
      std::cout << profile.header.stamp.sec << std::endl;
      std::cout << profile.header.stamp.nsec << std::endl;
      std::cout << profile.polygon.points[0] << std::endl;
      std::cout << profile.polygon.points[1] << std::endl;
      std::cout << profile.polygon.points.size() << std::endl;
      return true;

      //std::vector<int> test2 = bp::extract<std::vector<int> >(test);
      //std::cout << test2.size() << std::endl;
    }

    bp::object testCall2()
    {
      bp::object crossing;
      return crossing;
    }
};

BOOST_PYTHON_MODULE(crossing_detector)
{
  bp::class_<CrossingDetectorWrapper>("CrossingDetector", bp::init<double, double>())
    .def("crossingDescriptor", &CrossingDetectorWrapper::crossingDescriptor)
    .def("testCall0", &CrossingDetectorWrapper::testCall0)
    .def("testCall1", &CrossingDetectorWrapper::testCall1)
    .def("testCall2", &CrossingDetectorWrapper::testCall2)
    ;
}
