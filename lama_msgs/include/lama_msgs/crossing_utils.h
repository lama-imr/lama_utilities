#ifndef _LAMA_MSGS_CROSSING_UTILS_H_
#define _LAMA_MSGS_CROSSING_UTILS_H_

#include <cmath>
#include <vector>

#include <lama_msgs/Crossing.h>
#include <lama_msgs/Frontier.h>

namespace lama {

void rotateCrossing(const double angle, lama_msgs::Crossing& crossing);

} // namespace lama
#endif // _LAMA_MSGS_CROSSING_UTILS_H_
