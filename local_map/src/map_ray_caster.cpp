#include <local_map/map_ray_caster.h>

namespace lama {
namespace local_map {

void MapRayCaster::laserScanCast(const nav_msgs::OccupancyGrid& map, sensor_msgs::LaserScan& scan)
{
  scan.ranges.clear();
  for (double angle = scan.angle_min; angle <= scan.angle_max; angle += scan.angle_increment)
  {
  }
}

void MapRayCaster::placeProfileCaster(const nav_msgs::OccupancyGrid& map, lama_msgs::PlaceProfile& profile)
{
}

/* Return the list of pixel indexes from map center to pixel at map border and given angle
 *
 * The Bresenham algorithm is used.
 *
 * angle[in] beam angle
 * nrow[in] image height
 * ncol[in] image width
 */
const std::vector<size_t>& MapRayCaster::getRayCastToMapBorder(const double angle, const size_t nrow, const size_t ncol, const double tolerance)
{
  // Check that parameters are compatible with the cache. If not, erase the cache.
  if (nrow != nrow_ || ncol != ncol_)
  {
    raycast_lookup_.clear();
    nrow_ = nrow;
    ncol_ = ncol;
  }

  RayLookup::const_iterator ray = angleLookup(angle, tolerance);
  if (ray != raycast_lookup_.end())
  {
    return ray->second;
  }

  std::vector<size_t> pts;

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
      raycast_lookup_[angle] = pts;
			return raycast_lookup_[angle];
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

/* Return an iterator to the closest key (angle) in the cache.
 *
 * Return an iterator to the closest key (angle) in the cache, if this angle is close enough
 * to the desired angle.
 */
RayLookup::const_iterator MapRayCaster::angleLookup(const double angle, const double tolerance)
{
  if (tolerance == 0)
  {
    return raycast_lookup_.find(angle);
  }

	double dangle_lower;
	double dangle_upper;

  RayLookup::const_iterator upper_bound = raycast_lookup_.upper_bound(angle);
	if (upper_bound == raycast_lookup_.begin())
	{
    if (std::abs(angles::shortest_angular_distance(angle, upper_bound->first)) <= tolerance)
    {
      return upper_bound;
    }
    return raycast_lookup_.end();
	}
	else if (upper_bound == raycast_lookup_.end())
	{
		dangle_upper = raycast_lookup_.begin()->first - angle + 2 * M_PI;
		upper_bound--;
		dangle_lower = upper_bound->first - angle;
		if (dangle_lower < dangle_upper)
		{
      if (std::abs(angles::shortest_angular_distance(angle, upper_bound->first)) <= tolerance)
      {
        return upper_bound;
      }
      return raycast_lookup_.end();
		}
		else
		{
      if (std::abs(angles::shortest_angular_distance(angle, raycast_lookup_.begin()->first)) <= tolerance)
      {
        return raycast_lookup_.begin();
      }
      return raycast_lookup_.end();
		}
	}
	else
	{
		dangle_upper = upper_bound->first - angle;
    RayLookup::const_iterator lower_bound = upper_bound;
		lower_bound--;
		dangle_lower = angle - lower_bound->first;
		if (dangle_lower < dangle_upper)
		{
      if (std::abs(angles::shortest_angular_distance(angle, lower_bound->first)) <= tolerance)
      {
        return lower_bound;
      }
      return raycast_lookup_.end();
		}
		else
		{
      if (std::abs(angles::shortest_angular_distance(angle, upper_bound->first)) <= tolerance)
      {
        return upper_bound;
      }
      return raycast_lookup_.end();
		}
	}
}

} // namespace local_map
} // namespace lama
