#include <snd/valley.h>

namespace snd
{

Valley::Valley(const Gap& local_right_gap, const Gap& local_left_gap) :
  right_gap(local_right_gap),
  left_gap(local_left_gap)
{
  if (right_gap.type == RISING_GAP && left_gap.type == RISING_GAP)
  {
    best_gap_is_right_ = true;
  }
  else if (right_gap.type == FALLING_GAP && left_gap.type == FALLING_GAP)
  {
    best_gap_is_right_ = false;
  }
  else
  {
    const int right_to_goal = std::abs(right_gap.sectorDistanceToGoal());
    const int left_to_goal = std::abs(left_gap.sectorDistanceToGoal());
    if (left_to_goal < right_to_goal)
    {
      best_gap_is_right_ = false;
    }
    else
    {
      best_gap_is_right_ = true;
    }
  }
}

const Gap& Valley::bestGap() const
{
  if (best_gap_is_right_)
  {
    return right_gap;
  }
  else
  {
    return left_gap;
  }
}

const Gap& Valley::otherGap() const
{
  if (best_gap_is_right_)
  {
    return left_gap;
  }
  else
  {
    return right_gap;
  }
}

bool Valley::angleInValley(double angle, double margin) const
{
  return snd::angleInValley(angle, right_gap.angle(), left_gap.angle(), margin);
}

} /* namespace snd */

