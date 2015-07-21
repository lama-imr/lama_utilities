#ifndef LAMA_COMMON_PLACE_PROFILE_UTILS_H
#define LAMA_COMMON_PLACE_PROFILE_UTILS_H

#include <math.h>  // For round() (std::round() is since C++11).
#include <cmath>
#include <vector>
#include <list>
#include <map>
#include <limits>
#include <algorithm>
#include <stdexcept>
#include <sstream>

#include <lama_msgs/PlaceProfile.h>

#include <lama_common/angular_point.h>
#include <lama_common/is_sorted.h>

namespace lama_common
{

using std::vector;
using geometry_msgs::Point32;
using lama_msgs::PlaceProfile;

void normalizePlaceProfile(PlaceProfile& profile, bool sort=true);
PlaceProfile normalizedPlaceProfile(const PlaceProfile& profile, bool sort=true);

bool isClosed(const PlaceProfile& profile, double max_frontier_width);
void closePlaceProfile(PlaceProfile& profile, double max_frontier_width);
PlaceProfile closedPlaceProfile(const PlaceProfile& profile, double max_frontier_width);

vector<Point32> simplifyPath(const vector<Point32>& points, size_t begin, size_t end, double min_relevance);

void simplifyPlaceProfile(PlaceProfile& profile, double min_relevance);
PlaceProfile simplifiedPlaceProfile(const PlaceProfile& profile, double min_relevance);

void curtailPlaceProfile(PlaceProfile& profile, double max_distance);
PlaceProfile curtailedPlaceProfile(const PlaceProfile& profile, double max_distance);

} // namespace lama_common

#endif // LAMA_COMMON_PLACE_PROFILE_UTILS_H

