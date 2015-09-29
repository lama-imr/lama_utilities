#include <snd/gap.h>

namespace snd
{

Gap::Gap(size_t sector, const std::vector<float>& scan, size_t goal_sector, gap_type_t type) :
  sector(sector),
  type(type)
{
  if (scan.empty())
  {
    throw std::runtime_error("Empty scan");
  }
  size_t num_sectors = scan.size();
  angle_ = M_PI * ((double)(2 * sector)) / ((double)num_sectors) - M_PI;
  previous_range = scan[circularIndex(sector - 1, num_sectors)];
  range = scan[circularIndex(sector, num_sectors)];
  next_range = scan[circularIndex(sector + 1, num_sectors)];

  sector_distance_to_goal_ = shortestSectorDistance(sector, goal_sector, num_sectors);

  if (type == BOTH_GAP)
  {
    if (previous_range < next_range)
    {
      obstacle_distance_ = previous_range;
    }
    else
    {
      obstacle_distance_ = next_range;
    }
  }
  else if (type == RISING_GAP)
  {
    obstacle_distance_ = previous_range;
  }
  else
  {
    /* I.e. type == FALLING_GAP */
    obstacle_distance_ = next_range;
  }
}

} /* namespace snd */

