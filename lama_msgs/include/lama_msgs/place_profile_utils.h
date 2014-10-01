#ifndef _LAMA_MSGS_PLACE_PROFILE_UTILS_H_
#define _LAMA_MSGS_PLACE_PROFILE_UTILS_H_

#include <math.h>  // For round() (std::round() is since C++11).
#include <cassert>
#include <cmath>
#include <vector>
#include <list>
#include <map>
#include <limits>
#include <algorithm>

#include <lama_msgs/PlaceProfile.h>

namespace lama {

using std::vector;
using geometry_msgs::Point32;
using lama_msgs::PlaceProfile;

/* Return a positive modulo (as Python does, even with negative numbers)
 * 
 * index must be greater than (-size).
 */
inline int32_t circular_index(const int32_t index, const size_t size)
{
  return (index + size) % size;
}

/* Return true if the index is in the list.
 * 
 * The list must be sorted.
 *
 * It can be used to determine if a segment index is in the list excluded_segments.
 */
inline bool inList(const vector<int32_t>& list, const int32_t index)
{
  return std::binary_search(list.begin(), list.end(), index);
}

/* Return true if the segment following the point is excluded
 *
 * If true, the point is the end of an included segment.
 *
 * The list of excluded segments must be sorted.
 *
 */
inline bool pointIsBeforeExclusion(const PlaceProfile& profile, const int32_t index)
{
  return inList(profile.exclude_segments, circular_index(index, profile.polygon.points.size()));
}

/* Return true if the segment preceeding the point is excluded
 *
 * If true, the point is either excluded or at the beginning of an included segment.
 *
 * The list of excluded segments must be sorted.
 *
 */
inline bool pointIsAfterExclusion(const PlaceProfile& profile, const int32_t index)
{
  return inList(profile.exclude_segments, circular_index(index - 1, profile.polygon.points.size()));
}

/* Return true if the point is surrounded by two excluded segments.
 *
 * The list of excluded segments must be sorted.
 *
 * i - 1 is the segment before. i is the segment after.
 */
inline bool pointIsExcluded(const PlaceProfile& profile, const int32_t index)
{
  return pointIsAfterExclusion(profile, index) && pointIsBeforeExclusion(profile, index);
}

/* Return true if the point is at the beginning of an included segment.
 *
 * The list of excluded segments must be sorted.
 *
 */
inline bool pointIsAtInclusionStart(const PlaceProfile& profile, const int32_t index)
{
  return pointIsAfterExclusion(profile, index) && !pointIsBeforeExclusion(profile, index);
}

size_t firstIncludedPointFrom(const PlaceProfile& profile, const int32_t index);

size_t lastIncludedPointFrom(const PlaceProfile& profile, const int32_t index);

void normalizePlaceProfile(PlaceProfile& profile);
PlaceProfile normalizedPlaceProfile(const PlaceProfile& profile);

bool isClosed(const PlaceProfile& profile, const double max_frontier_width);
void closePlaceProfile(PlaceProfile& profile, const double max_frontier_width);
PlaceProfile closedPlaceProfile(const PlaceProfile& profile, const double max_frontier_width);

vector<Point32> simplifyPath(const vector<Point32>& points, const size_t begin, const size_t end, const double min_relevance);

void simplifyPlaceProfile(PlaceProfile& profile, const double min_relevance);
PlaceProfile simplifiedPlaceProfile(const PlaceProfile& profile, const double min_relevance);

} // namespace lama

#endif // _LAMA_MSGS_PLACE_PROFILE_UTILS_H_

