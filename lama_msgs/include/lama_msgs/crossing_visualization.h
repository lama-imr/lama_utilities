#ifndef _LAMA_MSGS_CROSSING_VISUALIZATION_H_
#define _LAMA_MSGS_CROSSING_VISUALIZATION_H_

#include <cmath>
#include <string>

#include <visualization_msgs/Marker.h>

#include <lama_msgs/Crossing.h>

namespace lama {

visualization_msgs::Marker getCrossingCenterMarker(const std::string& frame_id, const lama_msgs::Crossing& crossing);

visualization_msgs::Marker getFrontiersMarker(const std::string& frame_id, const lama_msgs::Crossing& crossing);

} // namespace lama

#endif // _LAMA_MSGS_CROSSING_VISUALIZATION_H_
