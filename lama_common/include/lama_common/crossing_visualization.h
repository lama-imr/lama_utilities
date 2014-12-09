#ifndef LAMA_COMMON_CROSSING_VISUALIZATION_H
#define LAMA_COMMON_CROSSING_VISUALIZATION_H

#include <cmath>
#include <string>

#include <visualization_msgs/Marker.h>

#include <lama_msgs/Crossing.h>

namespace lama_common {

visualization_msgs::Marker getCrossingCenterMarker(const std::string& frame_id, const lama_msgs::Crossing& crossing);

visualization_msgs::Marker getFrontiersMarker(const std::string& frame_id, const lama_msgs::Crossing& crossing);

} // namespace lama_common

#endif // LAMA_COMMON_CROSSING_VISUALIZATION_H
