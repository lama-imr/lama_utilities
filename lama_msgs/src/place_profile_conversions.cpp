#include <lama_msgs/place_profile_conversions.h>

#define OCCUPIED_THRESHOLD 50 // 0 = free, 100 = occupied.
#define COSTMAP_DISCRETISATION_COUNT 360 // Number of points for the costmap discretisation to obtain a PlaceProfile.

namespace lama {

/* Return the polygon that correspond to the PlaceProfile
 *
 * Points surrounded by two excluded segments are excluded.
 */
geometry_msgs::Polygon placeProfileToPolygon(const PlaceProfile& profile)
{
  geometry_msgs::Polygon polygon;

  polygon.points.reserve(profile.polygon.points.size());

  for (size_t i = 0; i < polygon.points.size(); ++i)
  {
    if (!pointIsExcluded(profile, i))
    {
      polygon.points.push_back(polygon.points[i]);
    }
  }

  return polygon;
}

sensor_msgs::PointCloud placeProfileToPointCloud(const PlaceProfile& profile)
{
  sensor_msgs::PointCloud cloud;
  cloud.header = profile.header;
  for (size_t i = 0; i < profile.polygon.points.size(); ++i)
  {
    if (!pointIsExcluded(profile, i))
    {
      cloud.points.push_back(profile.polygon.points[i]);
    }
  }
  return cloud;
}

PlaceProfile laserScanToPlaceProfile(const sensor_msgs::LaserScan& scan, const double max_range)
{
  PlaceProfile profile;
  profile.header = scan.header;
  size_t size = scan.ranges.size();
  profile.polygon.points.reserve(size);

  vector<bool> in_range(size, false);
  for (size_t i = 0; i < size; ++i)
  {
    if (scan.ranges[i] <= max_range)
    {
      in_range[i] = true;
    }
  }

  for (size_t i = 0; i < size; ++i)
  {
    if (in_range[i])
    {
      const double angle = scan.angle_min + ((double) i) * scan.angle_increment;
      geometry_msgs::Point32 point;
      point.x = scan.ranges[i] * std::cos(angle);
      point.y = scan.ranges[i] * std::sin(angle);
      profile.polygon.points.push_back(point);
    }
    else if (in_range[(i + 1) % size])
    {
      profile.exclude_segments.push_back(profile.polygon.points.size() - 1);
    }
  }
  if (!profile.exclude_segments.empty() && (profile.exclude_segments.front() == -1))
  {
    // If segment between last point and point 0 is excluded.
    // This happens if the first range(s) is(are) out of range.
    profile.exclude_segments[0] = profile.polygon.points.size() - 1;
  }

  return profile;
}

/* Return true if the map point is occupied.
 */
inline bool pointOccupied(const nav_msgs::OccupancyGrid& map, const int index)
{
	return (map.data[index] > OCCUPIED_THRESHOLD);
}

/* Return true if the map point is unknown.
 */
inline bool pointUnknown(const nav_msgs::OccupancyGrid& map, const int index)
{
	return (map.data[index] == -1);
}

/* Return true if the point lies in the map
 */
inline bool pointInMap(const int row, const int col, const size_t nrow, const size_t ncol)
{
	return ((0 <= col) && (col < ncol) &&
			(0 <= row) && (row < nrow));
}

/* Return the row number from offset for a row-major array
 */
inline size_t rowFromOffset(const size_t offset, const size_t ncol)
{
	return offset / ncol;
}

/* Return the column number from offset for a row-major array
 */
inline size_t colFromOffset(const size_t offset, const size_t ncol)
{
	return offset % ncol;
}

/* Return the offset from row and column number for a row-major array
 */
inline size_t offsetFromRowCol(const size_t row, const size_t col, const size_t ncol)
{
	return (row * ncol) + col;
}

/* Return the list of pixel indexes from map center to pixel at map border and given angle
 *
 * The Bresenham algorithm is used for rendering.
 *
 * angle[in] beam angle
 * nrow[in] image height
 * ncol[in] image width
 */
vector<size_t> getRayCastToMapBorder(const double angle, const size_t nrow, const size_t ncol)
{
  vector<size_t> pts;

	// Twice the distance from map center to map corner.
	const double r = std::sqrt((double) nrow * nrow + ncol * ncol);
	// Start point, map center.
	// TODO: the sensor position (map origin)  may not be the map center
	int x0 = ncol / 2;
	int y0 = nrow / 2;
	// End point, outside the map.
	int x1 = (int) round(x0 + r * std::cos(angle)); // Can be negative
	int y1 = (int) round(y0 + r * std::sin(angle));
	int dx = x1 - x0;
	int dy = y1 - y0;
	bool steep = (std::abs(dy) >= std::abs(dx));
	if (steep)
	{
    std::swap(x0, y0);
    std::swap(x1, y1);
		// recompute Dx, Dy after swap
		dx = x1 - x0;
		dy = y1 - y0;
	}
	int xstep = 1;
	if (dx < 0)
	{
		xstep = -1;
		dx = -dx;
	}
	int ystep = 1;
	if (dy < 0)
	{
		ystep = -1;
		dy = -dy;
	}
	int twoDy = 2 * dy;
	int twoDyTwoDx = twoDy - 2 * dx; // 2*Dy - 2*Dx
	int e = twoDy - dx; //2*Dy - Dx
	int y = y0;
	int xDraw, yDraw;
	for (int x = x0; x != x1; x += xstep)
	{
		if (steep)
		{
			xDraw = y;
			yDraw = x;
		}
		else
		{
			xDraw = x;
			yDraw = y;
		}
		if (pointInMap(yDraw, xDraw, nrow, ncol))
		{
			pts.push_back(offsetFromRowCol(yDraw, xDraw, ncol));
		}
		else
		{
			// We exit when the first point outside the map is encountered.
			return pts;
		}
		// next
		if (e > 0)
		{
			e += twoDyTwoDx; //E += 2*Dy - 2*Dx;
			y = y + ystep;
		}
		else
		{
			e += twoDy; //E += 2*Dy;
		}
	}
}

/* Return the pixel list by ray casting from map center to map border
 *
 * angle[in] laser beam angle
 * nrow[in] image height in pixel
 * ncol[in] image width in pixel
 */
vector<size_t> getRayCast(const double angle, const size_t nrow, const size_t ncol)
{
	// Store the ray casting up to the map border into a look-up table. Ray
	// casting exclusively depends on (angle, nrow and ncol). We use
  // ((angle + nrow) * ncol) to make a high-probably unique value out of
  // these three values.
	static std::map<double, vector<size_t> > raycast_lookup;

  const double code = (angle + nrow) * ncol;
  if (raycast_lookup.find(code) == raycast_lookup.end())
  {
    raycast_lookup[code] = getRayCastToMapBorder(angle, nrow, ncol);
  }

	return raycast_lookup[code];
}

/* Return the world coordinates of the map point at given index
 *
 * The map center is (0, 0).
 */
inline void indexToReal(const nav_msgs::OccupancyGrid& map, const size_t index, geometry_msgs::Point32& point)
{
  const double xcenter = (map.info.width / 2) * map.info.resolution;
  const double ycenter = (map.info.height / 2) * map.info.resolution;
  const size_t row = rowFromOffset(index, map.info.height);
  const size_t col = colFromOffset(index, map.info.width);
  const double xindex = col * map.info.resolution;
  const double yindex = row * map.info.resolution;
  point.x = xindex - xcenter;
  point.y = yindex - ycenter;
}

/* Return the first non-free point from map center to map border with given angle.
 *
 * Return the first non-free point (i.e. first occupied or unknown point),
 * calculated by raytracing from the map center to the map border with the
 * Bresenham algorithm and stopping at the first encountered occupied or
 * unknown point.
 * Return true if the first non-free point is an obstacle and false if it is an
 * unknown point or if no non-free point was met (ray tracing up to map
 * border).
 *
 * map[in] occupancy grid 
 * angle[in] angle of the laser ray
 */
bool firstNonFree(const nav_msgs::OccupancyGrid& map, const double angle, geometry_msgs::Point32& point)
{
  vector<size_t> ray = getRayCast(angle, map.info.height, map.info.width);
	for (size_t i = 0; i < ray.size(); ++i)
	{
    size_t idx = ray[i];
		if (pointOccupied(map, idx))
		{
      indexToReal(map, idx, point);
			return true;
		}
    else if (pointUnknown(map, idx))
    {
      indexToReal(map, idx, point);
			return false;
    }
	}
  indexToReal(map, ray.back(), point);
	return false;
}

PlaceProfile costmapToPlaceProfile(const nav_msgs::OccupancyGrid& map)
{
  PlaceProfile profile;
  profile.header = map.header;
  profile.polygon.points.reserve(COSTMAP_DISCRETISATION_COUNT);

  // angle_min should be slightly greater than M_PI to be sure that the
  // pixel on the bottom half of the map is chosen if map height is even.
  const double angle_min = -M_PI + 1e-6;
  const double resolution = 2 * M_PI / COSTMAP_DISCRETISATION_COUNT;
  geometry_msgs::Point32 last_point;
  // last_point should be different than any point in the map.
  last_point.x = map.info.width * map.info.resolution;
  geometry_msgs::Point32 this_point;
  geometry_msgs::Point32 next_point;
  bool this_in_range = firstNonFree(map, angle_min, this_point);
  double next_angle = angle_min + resolution;
  for (size_t i = 0; i < COSTMAP_DISCRETISATION_COUNT; ++i)
  {
    bool next_in_range = firstNonFree(map, next_angle, next_point);
    if (this_in_range)
    {
      if ((this_point.x != last_point.x) || (this_point.y != last_point.y))
      {
        profile.polygon.points.push_back(this_point);
      }
    }
    else if (next_in_range)
    {
      profile.exclude_segments.push_back(profile.polygon.points.size() - 1);
    }
    next_angle += resolution;
    last_point = this_point;
    this_point = next_point;
    this_in_range = next_in_range;
  }
  if (!profile.exclude_segments.empty() && (profile.exclude_segments.front() == -1))
  {
    // If segment between last point and point 0 is excluded.
    // This happens if the first point(s) is(are) non-free.
    profile.exclude_segments[0] = profile.polygon.points.size() - 1;
  }

  return profile;
}

} // namespace lama

