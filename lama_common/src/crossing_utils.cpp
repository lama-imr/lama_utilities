#include <lama_common/crossing_utils.h>

namespace lama_common {

void rotateCrossing(lama_msgs::Crossing& crossing, const double angle)
{
  const double cosangle = std::cos(angle);
  const double sinangle = std::sin(angle);

  // Rotate crossing center.
  const double old_x = crossing.center.x;
  const double old_y = crossing.center.y;
  crossing.center.x = old_x * cosangle - old_y * sinangle;
  crossing.center.y = old_x * sinangle + old_y * cosangle;

  // Rotate frontier points and change frontier angles.
  typedef std::vector<lama_msgs::Frontier>::iterator frontier_it;
  for (frontier_it frontier = crossing.frontiers.begin(); frontier != crossing.frontiers.end(); ++frontier)
  {
    frontier->angle = angles::normalize_angle(frontier->angle + angle);
    double old_x = frontier->p1.x;
    double old_y = frontier->p1.y;
    frontier->p1.x = old_x * cosangle - old_y * sinangle;
    frontier->p1.y = old_x * sinangle + old_y * cosangle;
    old_x = frontier->p2.x;
    old_y = frontier->p2.y;
    frontier->p2.x = old_x * cosangle - old_y * sinangle;
    frontier->p2.y = old_x * sinangle + old_y * cosangle;
  }
}

} // namespace lama_common
