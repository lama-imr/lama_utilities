#pragma once
#ifndef SND_UTILITIES_H
#define SND_UTILITIES_H

#include <algorithm> // For std::min.
#include <cassert>
#include <cmath> // For std::abs, fmod.
#include <cstddef> // For size_t.

namespace snd
{

/** Return a positive modulo (as Python does, even with negative numbers)
 */
inline size_t circularIndex(int index, size_t size)
{
  while (index < 0)
  {
    index += size;
  }
  return ((size_t)index) % size;
}

/// Limit a value to the range of min, max
template<typename T>
inline T saturate(T val, T min, T max)
{
  if (val < min)
  {
    return min;
  }
  else if (val > max)
  {
    return max;
  }
  return val;
}

// TODO: test with even/odd size.
inline int shortestSectorDistance(int s1, int s2, size_t size)
{
  int int_size = (int)size;
	int s = (int)circularIndex(s2, size) - (int)circularIndex(s1, size);
  if (s < -int_size / 2)
  {
    return s + int_size;
  }
  if (s >= int_size / 2)
  {
    return s - int_size;
  }
  return s;
}

inline double normalizeAngle(double angle)
{
  return std::fmod(std::fmod(angle + M_PI, 2 * M_PI) + 2 * M_PI, 2 * M_PI) - M_PI;
}

inline double sectorToAngle(int sector, size_t num_sectors)
{
  return M_PI * ((double)(2 * sector)) / ((double)num_sectors) - M_PI;
}

inline bool angleInValley(double angle, double right_angle, double left_angle, double margin=0.0)
{
  // No need to normalize more than once.
  const double limit_right = right_angle + margin;
  const double limit_left = left_angle - margin;
  if (normalizeAngle(limit_left - limit_right) <= 0)
  {
    // No space between the two gaps expanded with the margin.
    return false;
  }
  return (normalizeAngle(angle - limit_right) > 0) && (normalizeAngle(limit_left - angle) > 0);
}

} /* namespace snd */

#endif /* SND_UTILITIES_H */

