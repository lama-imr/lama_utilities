#include <lama_msgs/place_profile_utils.h>

namespace lama {

struct AngularPoint
{
  AngularPoint(geometry_msgs::Point32 point, int32_t index_) :
    angle(std::atan2(point.y, point.x)),
    index(index_)
  {
  }

  double angle;
  int32_t index;

  bool operator<(const AngularPoint& rhs) const {return angle < rhs.angle;}
};

/* Change point order in place so that point angles are sorted.
 *
 * Angles within [-pi,pi[ will be sorted.
 */
void normalizePlaceProfile(PlaceProfile& profile)
{
  vector<AngularPoint> angular_points;
  angular_points.reserve(profile.polygon.points.size());
  for (size_t i = 0; i < profile.polygon.points.size(); ++i)
  {
    angular_points.push_back(AngularPoint(profile.polygon.points[i], i));
  }

#if __cplusplus > 199711L
// is_sorted is only supported from standard C++11 on.
  if (std::is_sorted(angular_points.begin(), angular_points.end()))
  {
    return;
  }
#endif

  // TODO: Improve sort performance.
  // Appart from the position of the step from +pi to -pi (CCW) which can be in
  // the middle of the array instead of being absent, angular_points
  // should be sorted. Use a circular_iterator, if this exists.
  std::sort(angular_points.begin(), angular_points.end());

  // Check if the order has changed.
  bool any_change = false;
  for (size_t i = 0; i < profile.polygon.points.size(); ++i)
  {
    if (i != angular_points[i].angle)
    {
      any_change = true;
      break;
    }
  }

  if (any_change)
  {
    PlaceProfile old_profile = profile;
    for (size_t i = 0; i < profile.polygon.points.size(); ++i)
    {
      profile.polygon.points[i] = old_profile.polygon.points[angular_points[i].index];
    }
    for (size_t i = 0; i < profile.exclude_segments.size(); ++i)
    {
      profile.exclude_segments[i] = old_profile.exclude_segments[angular_points[i].index];
    }
  }
}

/* Returned a copy of a PlaceProfile message where point angles are sorted.
 *
 * Angles within [-pi,pi[ will be sorted.
 */
PlaceProfile normalizedPlaceProfile(const PlaceProfile& profile)
{
  PlaceProfile new_profile = profile;
  normalizePlaceProfile(new_profile);
  return new_profile;
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

  vector<geometry_msgs::Point32> old_points = profile.polygon.points;
  profile.polygon.points.clear();
  profile.polygon.points.reserve(size);

  for(size_t i = 0; i < size; ++i)
  {
    if (pointIsExcluded(profile, i))
    {
      continue;
    }
    geometry_msgs::Point32 a(old_points[i]);
    size_t j = (i + 1) % size;
    while (pointIsExcluded(profile, j))
    {
      j = (j + 1) % size;
      continue;
    }
    geometry_msgs::Point32 b(old_points[j]);

    const double dx = b.x - a.x;
    const double dy = b.y - a.y;

    if (dx * dx + dy * dy > width2)
    {
      geometry_msgs::Point32 point;
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
  profile.exclude_segments.clear();
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
    geometry_msgs::Point32 a(profile.polygon.points[i]);
    size_t j = (i + 1) % size;
    while (pointIsExcluded(profile, j))
    {
      j = (j + 1) % size;
      continue;
    }
    geometry_msgs::Point32 b(profile.polygon.points[j]);

    const double dx = b.x - a.x;
    const double dy = b.y - a.y;

    if (dx * dx + dy * dy > width2)
    {
      geometry_msgs::Point32 point;
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

} // namespace lama


