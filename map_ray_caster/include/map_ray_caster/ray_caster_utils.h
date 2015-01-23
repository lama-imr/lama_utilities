#ifndef MAP_RAY_CASTER_RAY_CASTER_UTILS_H
#define MAP_RAY_CASTER_RAY_CASTER_UTILS_H

#include <cstddef>

#include <nav_msgs/OccupancyGrid.h>

namespace map_ray_caster {

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

/* Return true if the point lies in the map
 */
inline bool pointInMap(const int row, const int col, const size_t nrow, const size_t ncol)
{
  return ((0 <= col) && ((size_t) col < ncol) && (0 <= row) && ((size_t) row < nrow));
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

} // namespace map_ray_caster

#endif // MAP_RAY_CASTER_RAY_CASTER_UTILS_H
