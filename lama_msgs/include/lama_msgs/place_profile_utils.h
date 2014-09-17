#ifndef _LAMA_MSGS_PLACE_PROFILE_UTILS_H_
#define _LAMA_MSGS_PLACE_PROFILE_UTILS_H_

#include <math.h>  // For round().
#include <cmath>
#include <vector>
#include <map>
#include <algorithm>

#include <lama_msgs/PlaceProfile.h>

namespace lama {

using std::vector;
using lama_msgs::PlaceProfile;

/* Return true if the index is in the list.
 * 
 * It can be used to determine if a segment index is in the list excluded_segments.
 */
inline bool inList(const vector<int32_t>& list, const int32_t index)
{
  return std::binary_search(list.begin(), list.end(), index);
}

/* Return true if the point is surrounded by two excluded segments.
 *
 * i - 1 is the segment before
 * i is the segment after
 */
inline bool pointIsExcluded(const PlaceProfile& profile, const int32_t index)
{
  return (inList(profile.exclude_segments, (index - 1) % profile.polygon.points.size()) &&
      inList(profile.exclude_segments, index % profile.polygon.points.size()));
}

void normalizePlaceProfile(PlaceProfile& profile);
PlaceProfile normalizedPlaceProfile(const PlaceProfile& profile);

void closePlaceProfile(PlaceProfile& profile, const double max_frontier_width);
PlaceProfile closedPlaceProfile(const PlaceProfile& profile, const double max_frontier_width);

} // namespace lama

#endif // _LAMA_MSGS_PLACE_PROFILE_UTILS_H_

