#include <lama_msgs/place_profile_utils.h>

#include <iostream> // DEBUG

namespace lama {

/* A pair (index, angle) constructed from a Point32, to get the permutation used for sorting.
 */
struct AngularPoint
{
  AngularPoint(Point32 point, int32_t index_) :
    angle(std::atan2(point.y, point.x)),
    index(index_)
  {
  }

  double angle;
  int32_t index;

  bool operator<(const AngularPoint& other) const {return angle < other.angle;}
};

/* Checks if the elements in range [first, last) are sorted in ascending order
 *
 * is_sorted is in STL since C++11.
 * copied from http://en.cppreference.com/w/cpp/algorithm/is_sorted and
 * http://en.cppreference.com/w/cpp/algorithm/is_sorted_until
 */
template<class ForwardIt>
inline bool is_sorted(ForwardIt first, ForwardIt last)
{
  if (first != last)
  {
    ForwardIt next = first;
    while (++next != last)
    {
      if (*next < *first)
      {
        return false;
      }
      first = next;
    }
  }
  return true;
}

/* Remove out-of-bound exclude_segments
 */
inline void removeWrongSegments(PlaceProfile& profile)
{
  if (profile.exclude_segments.empty())
  {
    return;
  }

  // Sort the segments.
  if (!is_sorted(profile.exclude_segments.begin(), profile.exclude_segments.end()))
  {
    std::sort(profile.exclude_segments.begin(), profile.exclude_segments.end());
  }

  if (profile.exclude_segments[0] < 0)
  {
    // Remove negative elements and too large elements.
    vector<int32_t> old_exclude_segments = profile.exclude_segments;
    profile.exclude_segments.clear();
    for (size_t i = 0; i < old_exclude_segments.size(); ++i)
    {
      if (0 <= old_exclude_segments[i] && old_exclude_segments[i] < profile.polygon.points.size())
      {
        profile.exclude_segments.push_back(old_exclude_segments[i]);
      }
    }
  }
  else
  {
    // Remove too large elements.
    while (profile.exclude_segments.back() >= profile.polygon.points.size())
    {
      profile.exclude_segments.pop_back();
    }
  }
}

/* Modify a PlaceProfile so that the length of exclude_segments will be minimized.
 */
inline void reduceSegmentCount(PlaceProfile& profile)
{
  // Check if the segment count can be reduced (i.e. if there is at least one excluded point and
  // no doubled value in exlude_segments).
  bool doit = false;
  // Check for excluded points.
  for (size_t i = 0; i < profile.polygon.points.size(); ++i)
  {
    if (pointIsExcluded(profile, i))
    {
      doit = true;
      break;
    }
  }
  // Check for doubles in exclude_segments.
  for (int i = 0; i < ((int) profile.exclude_segments.size()) - 1; ++i)
  {
    if (profile.exclude_segments[i] == profile.exclude_segments[i + 1])
    {
      doit = true;
      break;
    }
  }
  if (!doit)
  {
    return;
  }

  PlaceProfile old_profile = profile;
  profile.polygon.points.clear();
  profile.exclude_segments.clear();
  for (size_t i = 0; i < old_profile.polygon.points.size(); ++i)
  {
    if (!pointIsExcluded(old_profile, i))
    {
      profile.polygon.points.push_back(old_profile.polygon.points[i]);
      if (pointIsBeforeExclusion(old_profile, i))
      {
        profile.exclude_segments.push_back(profile.polygon.points.size() - 1);
      }
    }
  }
}

/* Return true if points are in counterclockwise order
 */
inline bool pointOrder(const vector<AngularPoint>& angular_points)
{
  double sum_angle_increments = 0;
  for (size_t i = 0; i < std::min((int)angular_points.size() - 1, 3); ++i)
  {
    const double dtheta = angular_points[i + 1].angle - angular_points[i].angle;
    if (std::abs(dtheta) < 0.99 * M_PI)
    {
      sum_angle_increments += dtheta;
    }
  }
  return sum_angle_increments > 0;
}

/* Change point order in place so that point angles are sorted.
 *
 * Angles within [-pi,pi[ will be sorted from smallest to greatest (i.e.
 * counterclock-wise), optionally.
 * Points fully excluded will be removed.
 * invalid exclude_sements will be removed.
 * exclude_segments will be sorted from smallest index to greatest.
 * exclude_segments will contain the miminum number of values.
 *
 * The algorithm is not robust against non-simple polygons.
 *
 * profile[out] PlaceProfile
 * sort[in] If true, the polygon points will be sorted according to their angle.
 */
void normalizePlaceProfile(PlaceProfile& profile, const bool sort)
{
  if (sort)
  {
    vector<AngularPoint> angular_points;
    size_t point_count = profile.polygon.points.size();
    angular_points.reserve(point_count);
    for (size_t i = 0; i < point_count; ++i)
    {
      angular_points.push_back(AngularPoint(profile.polygon.points[i], i));
    }

    if (!is_sorted(angular_points.begin(), angular_points.end()))
    {
      // If points are clock-wise, exclude_segments needs to be changed because
      // after with clock-wise direction means before with counterclock-wise direction.
      const bool counterclockwise = pointOrder(angular_points);

      // TODO: Improve sort performance.
      // Appart from the position of the step from +pi to -pi (CCW) which can be in
      // the middle of the array instead of being absent, angular_points
      // should be sorted. Use a circular_iterator, if this exists.
      std::sort(angular_points.begin(), angular_points.end());

      PlaceProfile old_profile = profile;
      for (size_t i = 0; i < point_count; ++i)
      {
        profile.polygon.points[i] = old_profile.polygon.points[angular_points[i].index];
      }

      vector<int32_t> inverse_permutation;
      if (!profile.exclude_segments.empty())
      {
        inverse_permutation.resize(point_count);
        for (size_t i = 0; i < point_count; ++i)
        {
          inverse_permutation[angular_points[i].index] = i;
        }
      }
      if (!counterclockwise)
      {
        // v[i] = v[i]+1, does the trick to get the correct renumbering.
        for (size_t i = 0; i < old_profile.exclude_segments.size(); ++i)
        {
          old_profile.exclude_segments[i] = circular_index(old_profile.exclude_segments[i] + 1, point_count);
        }
      }
      for (size_t i = 0; i < profile.exclude_segments.size(); ++i)
      {
        profile.exclude_segments[i] = inverse_permutation[old_profile.exclude_segments[i]];
      }
    }
  }

  removeWrongSegments(profile);

  reduceSegmentCount(profile);
}

/* Return a copy of a PlaceProfile message where point angles are sorted.
 *
 * Angles within [-pi,pi[ will be sorted.
 *
 * profile[in] PlaceProfile
 * sort[in] if true, the polygon points will be sorted according to their angle.
 */
PlaceProfile normalizedPlaceProfile(const PlaceProfile& profile, const bool sort)
{
  PlaceProfile new_profile = profile;
  normalizePlaceProfile(new_profile, sort);
  return new_profile;
}

bool isClosed(const PlaceProfile& profile, const double max_frontier_width)
{
  const size_t size = profile.polygon.points.size();
  const double width2 = max_frontier_width * max_frontier_width;

  for(size_t i = 0; i < size; ++i)
  {
    if (pointIsExcluded(profile, i))
    {
      continue;
    }
    Point32 a = profile.polygon.points[i];
    size_t j = (i + 1) % size;
    while (pointIsExcluded(profile, j))
    {
      j = (j + 1) % size;
      continue;
    }
    Point32 b = profile.polygon.points[j];

    const double dx = b.x - a.x;
    const double dy = b.y - a.y;

    if (dx * dx + dy * dy > width2)
    {
      return false;
    }
  }
  return true;
}


/* Modify a PlaceProfile in place, so that the largest frontier will be bounded.
 *
 * Add some points to the PlaceProfile message so that the distance between two
 * consecutive points will be at most max_frontier_width. The polygon will have
 * no excluded segments.
 *
 * profile[out] PlaceProfile
 * max_frontier_width[in] maximum frontier width
 */
void closePlaceProfile(PlaceProfile& profile, const double max_frontier_width)
{
  const size_t size = profile.polygon.points.size();
  const double width2 = max_frontier_width * max_frontier_width;

  PlaceProfile old_profile = profile;
  profile.polygon.points.clear();
  profile.polygon.points.reserve(size);
  profile.exclude_segments.clear();

  for(size_t i = 0; i < size; ++i)
  {
    if (pointIsExcluded(old_profile, i))
    {
      continue;
    }
    Point32 a = old_profile.polygon.points[i];
    size_t j = (i + 1) % size;
    while (pointIsExcluded(old_profile, j))
    {
      j = (j + 1) % size;
      continue;
    }
    Point32 b = old_profile.polygon.points[j];

    const double dx = b.x - a.x;
    const double dy = b.y - a.y;

    if (dx * dx + dy * dy > width2)
    {
      Point32 point;
      const double norm = std::sqrt(dx * dx + dy * dy);
      // Unit vector from point[i] to point[j].
      const double ux = dx / norm;
      const double uy = dy / norm;
      for (double s = 0; s <= norm; s += max_frontier_width) 
      {
        point.x = a.x + s * ux;
        point.y = a.y + s * uy;
        profile.polygon.points.push_back(point);
      }
    }
    else
    {
      profile.polygon.points.push_back(a);
    }
  }
}

/* Return a PlaceProfile where the largest frontier will be bounded by adding some points.
 *
 * Add some points to the returned PlaceProfile so that the distance between two
 * consecutive points will be at most max_frontier_width. The polygon will have
 * no excluded segments.
 *
 * profile[in] PlaceProfile
 * max_frontier_width[in] maximum frontier width
 */
PlaceProfile closedPlaceProfile(const PlaceProfile& profile, const double max_frontier_width)
{
  // closePlaceProfile is not used here to avoid an extra copy of the polygon points.
 
  PlaceProfile new_profile;
  const size_t size = profile.polygon.points.size();
  const double width2 = max_frontier_width * max_frontier_width;

  new_profile.polygon.points.reserve(size);

  for(size_t i = 0; i < size; ++i)
  {
    if (pointIsExcluded(profile, i))
    {
      continue;
    }
    Point32 a(profile.polygon.points[i]);
    size_t j = (i + 1) % size;
    while (pointIsExcluded(profile, j))
    {
      j = (j + 1) % size;
      continue;
    }
    Point32 b(profile.polygon.points[j]);

    const double dx = b.x - a.x;
    const double dy = b.y - a.y;

    if (dx * dx + dy * dy > width2)
    {
      Point32 point;
      const double norm = std::sqrt(dx * dx + dy * dy);
      // Unit vector from point[i] to point[j].
      const double ux = dx / norm;
      const double uy = dy / norm;
      for (double s = 0; s <= norm; s += max_frontier_width) 
      {
        point.x = a.x + s * ux;
        point.y = a.y + s * uy;
        new_profile.polygon.points.push_back(point);
      }
    }
    else
    {
      new_profile.polygon.points.push_back(a);
    }
  }

  return new_profile;
}

struct IndexedDouble
{
  size_t index;
  double value;
  IndexedDouble(const size_t i, const double v) : index(i), value(v) {}

  bool operator<(const IndexedDouble& other) {return value < other.value;}
};

/* Return the relevance of 3 points.
 *
 * Return |AB| + |BC| - |AC| for points A, B, and C, where |x| is the length of
 * x, point B should be more or less between A and C.
 * Relevance is 0 for colinear points and the farther B from (AC), the greater.
 */
inline double getRelevance(const Point32& pi, const Point32& pj, const Point32& pk)
{
	double relevance;
  double dx;
  double dy;
	dx = pi.x - pj.x;
	dy = pi.y - pj.y;
	relevance = sqrt(dx * dx + dy * dy);
	dx = pj.x - pk.x;
	dy = pj.y - pk.y;
	relevance += sqrt(dx * dx + dy * dy);
	dx = pi.x - pk.x;
	dy = pi.y - pk.y;
	relevance -= sqrt(dx * dx+ dy * dy);
	return std::abs(relevance);
}

/* Return the relevance of point in points with index it->index with its predecessor and successor
 * 
 * l[in] list from which to get the original indexes.
 * it[in] iterator to the adequate element of l.
 * points[int] list of points with original indexing.
 */
double getRelevance(const std::list<IndexedDouble>& l, std::list<IndexedDouble>::const_iterator it,
    const vector<Point32>& points)
{
  std::list<IndexedDouble>::const_iterator pred = it;
  std::list<IndexedDouble>::const_iterator succ = it;
	pred--;
	succ++;

	double relevance = -1;
	if (it != l.begin() && succ != l.end())
  {
		relevance = getRelevance(points[pred->index], points[it->index], points[succ->index]);
	} 
	return relevance;
}

/* Reduce the number of points in a series of points
 *
 * A relevance filter is used.
 *
 * points[in] list of points.
 * begin[in] index of first point to consider.
 * end[in] index of point after the last point to consider.
 * min_relevance[in] relevance threshold, points with relevance smaller than this are removed.
 */
vector<Point32> simplifyPath(const vector<Point32>& points, const size_t begin, const size_t end, const double min_relevance)
{
  assert(begin < points.size());
  assert(end <= points.size());
  if (end - begin < 3)
  {
    vector<Point32> filtered_points;
    for (size_t i = begin; i < end; ++i)
    {
      filtered_points.push_back(points[i]);
    }
    return filtered_points;
  }

  std::list<IndexedDouble> relevances;
  double relevance = std::numeric_limits<double>::min();
	for (size_t j = begin; j < end; ++j)
  {
    if (j != begin && j != end - 1)
    {
      relevance = getRelevance(points[j - 1], points[j], points[j + 1]);
    }
		relevances.push_back(IndexedDouble(j, relevance));
	}
	relevances.front().value = 10 * min_relevance;
	relevances.back().value = 10 * min_relevance;

	do
  {
    if (relevances.size() < 3)
    {
      break;
    }

    std::list<IndexedDouble>::iterator me = std::min_element(relevances.begin(), relevances.end());

		if (me == relevances.end())
    {
      // relevances is empty, finish.
			break;
    }
    if (me->value >= min_relevance)
    {
      // All points are relevant, finish.
      break;
    }
    // Because of the last test, the smallest relevance cannot be on the first
    // nor the last point, so the following is not risky.
    std::list<IndexedDouble>::iterator pred = me;
    --pred;
    std::list<IndexedDouble>::iterator succ = me;
    ++succ;
    relevances.erase(me);
    if (pred != --relevances.end())
    {
      // The point removed was not the before-the-last point.
      relevance = getRelevance(relevances, pred, points);
      if (relevance >= 0)
      {
        pred->value = relevance;
      }
    }
    if (succ != relevances.begin())
    {
      // The point removed was not the 2nd point.
      relevance = getRelevance(relevances, succ, points);
      if (relevance >= 0)
      {
        succ->value = relevance;
      }
    }
	} while (true);

	vector<Point32> filtered_points;
	filtered_points.reserve(relevances.size());
	for(std::list<IndexedDouble>::const_iterator it = relevances.begin(); it != relevances.end(); ++it)
  {
		filtered_points.push_back(points[it->index]);
  }
	return filtered_points;
}

/* Return the next included point (point with index index possibly)
 */
size_t firstIncludedPointFrom(const PlaceProfile& profile, const int32_t index)
{
  for (size_t i = index; i < profile.polygon.points.size(); ++i)
  {
    if (!pointIsExcluded(profile, i))
    {
      return i;
    }
  }
  return profile.polygon.points.size();
}
    
/* Return the index after the last point of the next included segment
 */
size_t lastIncludedPointFrom(const PlaceProfile& profile, const int32_t index)
{
  size_t first_included_from = firstIncludedPointFrom(profile, index);
  
  for (size_t i = first_included_from; i < profile.polygon.points.size(); ++i)
  {
    if (pointIsBeforeExclusion(profile, i))
    {
      return i + 1;
    }
  }
  return profile.polygon.points.size();
}

/* Reduce the number of points in a PlaceProfile message
 *
 * A relevance filter is used for points of non-excluded segments. All points
 * with relevance smaller than the threshold are removed.
 * The number of excluded segments will also be minimized (no more exluded points).
 *
 * profile[in] PlaceProfile
 * min_relevance[in] relevance threshold, points with relevance smaller than this are removed.
 */
void simplifyPlaceProfile(PlaceProfile& profile, const double min_relevance)
{
  // Note: we do not simplify the segment between the last point and the first one.
  
	if (profile.polygon.points.size() < 3)
  {
		return;
  }

  if (profile.exclude_segments.empty())
  {
    profile.polygon.points = simplifyPath(profile.polygon.points, 0, profile.polygon.points.size(), min_relevance);
    return;
  }

  PlaceProfile old_profile = profile;

  // We need sorted excluded segments (points do not need to be sorted).
  if (!is_sorted(old_profile.exclude_segments.begin(), old_profile.exclude_segments.end()))
  {
    std::sort(old_profile.exclude_segments.begin(), old_profile.exclude_segments.end());
  }

  profile.polygon.points.clear();
  profile.exclude_segments.clear();
  size_t path_end = 0;
  do
  {
    const size_t path_start = firstIncludedPointFrom(old_profile, path_end);
    path_end = lastIncludedPointFrom(old_profile, path_start);
    if (path_start < old_profile.polygon.points.size())
    {
      const vector<Point32> filtered_points = simplifyPath(old_profile.polygon.points, path_start, path_end, min_relevance);
      for (size_t i = 0; i < filtered_points.size(); ++i)
      {
        profile.polygon.points.push_back(filtered_points[i]);
      }
      if (path_end < old_profile.polygon.points.size())
      {
        profile.exclude_segments.push_back(profile.polygon.points.size() - 1);
      }
    }
  } while (path_end < old_profile.polygon.points.size());

  if (pointIsBeforeExclusion(old_profile, old_profile.polygon.points.size() - 1) &&
      !pointIsBeforeExclusion(profile, profile.polygon.points.size() -1))
  {
    // Last segment is excluded in old_profile.
    profile.exclude_segments.push_back(profile.polygon.points.size() - 1);
  }

}

/* Return a PlaceProfile with reduced number of points.
 *
 * A relevance filter is used.
 *
 * profile[in] PlaceProfile
 * min_relevance[in] relevance threshold, points with relevance smaller than this are removed.
 */
PlaceProfile simplifiedPlaceProfile(const PlaceProfile& profile, const double min_relevance)
{
  PlaceProfile new_profile = profile;
  simplifyPlaceProfile(new_profile, min_relevance);
  return new_profile;
}

/* Remove points farther than a certain distance.
 *
 * profile[out] PlaceProfile
 * max_distance[in] range cutoff
 */
void curtailPlaceProfile(PlaceProfile& profile, const double max_distance)
{
}

/* Return a PlaceProfile only containing points withing a certain distance.
 *
 * profile[in] PlaceProfile
 * max_distance[in] range cutoff
 */
PlaceProfile curtailedPlaceProfile(const PlaceProfile& profile, const double max_distance)
{
  PlaceProfile new_profile = profile;
  curtailPlaceProfile(new_profile, max_distance);
  return new_profile;
}

} // namespace lama


