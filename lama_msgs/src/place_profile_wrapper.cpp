#include <boost/python.hpp>

#include <lama_msgs/PlaceProfile.h>

namespace bp = boost::python;

namespace lama {

using lama_msgs::PlaceProfile;

BOOST_PYTHON_MODULE(place_profile_wrapper)
{
  bp::class_<PlaceProfile>("PlaceProfileWrapper", bp::init<>())
  .add_property("polygon", &PlaceProfile::polygon)
  .add_property("exclude_segments", &PlaceProfile::exclude_segments)
  ;
}

} // namespace lama
