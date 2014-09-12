#include <lama_msgs/crossing_utils.h>

namespace lama {

void rotateCrossing(const double angle, lama_msgs::Crossing& crossing)
{
  const double cosangle = std::cos(angle);
  const double sinangle = std::sin(angle);

  // Rotate crossing center.
  crossing.center.x = crossing.center.x * cosangle + crossing.center.y * sinangle;
  crossing.center.y = -crossing.center.x * sinangle + crossing.center.y * cosangle;

  // Rotate frontier points and change frontier angles.
  typedef std::vector<lama_msgs::Frontier>::iterator frontier_it;
  for (frontier_it frontier = crossing.frontiers.begin(); frontier != crossing.frontiers.end(); ++frontier)
  {
    frontier->angle = angles::normalize_angle(frontier->angle + angle);
    frontier->p1.x = frontier->p1.x * cosangle + frontier->p1.y * sinangle;
    frontier->p1.y = -frontier->p1.x * sinangle + frontier->p1.y * cosangle;
    frontier->p2.x = frontier->p2.x * cosangle + frontier->p2.y * sinangle;
    frontier->p2.y = -frontier->p2.x * sinangle + frontier->p2.y * cosangle;
  }
}

} // namespace lama
