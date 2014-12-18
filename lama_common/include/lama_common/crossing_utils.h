#ifndef LAMA_COMMON_CROSSING_UTILS_H
#define LAMA_COMMON_CROSSING_UTILS_H

#include <cmath>
#include <vector>

#include <angles/angles.h> /* for angles::normalize_angle() */
#include <lama_msgs/Crossing.h>
#include <lama_msgs/Frontier.h>

namespace lama_common {

void rotateCrossing(lama_msgs::Crossing& crossing, const double angle);

} // namespace lama_common

#endif // LAMA_COMMON_CROSSING_UTILS_H
