#pragma once
#ifndef SND_GAP_H
#define SND_GAP_H

#include <stdexcept>
#include <vector>

#include <snd/utilities.h>

namespace snd
{

enum gap_type_t
{
  // Descending discontinuity according to Minguez et al.
  // Left gap according to Durham et al.
  // In practical, this range is larger than the next one by at least
  // the threshold.
  FALLING_GAP,

  // Rising discontinuity according to Minguez et al.
  // Right gap according to Durham et al.
  // In practical, this range is larger than the previous one by at least
  // the threshold.
  RISING_GAP,

  // Both FALLING_GAP and RISING_GAP, i.e. one long range between two short
  // ranges.
  BOTH_GAP,

  // To construct an dummy gap.
  UNDEFINED_GAP
};

class Gap
{
  public :

    Gap(size_t sector, const std::vector<float>& scan, size_t goal_sector, gap_type_t type);
    
    size_t sector;
    gap_type_t type;

    double angle() const {return angle_;}
    int sectorDistanceToGoal() const {return sector_distance_to_goal_;}
    double obstacleDistance() const {return obstacle_distance_;}

    double previous_range;
    double range;
    double next_range;

  private :

    double angle_;
    int sector_distance_to_goal_;
    double obstacle_distance_;
};

} /* namespace snd */

#endif /* SND_GAP_H */

