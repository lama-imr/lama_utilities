#include <snd/snd_planner.h>

// The construction of string is avoided if ALLOW_SUPERDEBUG is 0.
#define ALLOW_SUPERDEBUG 1

namespace snd
{

template <typename T>
inline typename std::list<T>::const_iterator circularNext(const std::list<T>& list, const typename std::list<T>::const_iterator& node)
{
  typename std::list<T>::const_iterator next_node;

  next_node = node;
  next_node++;
  if (next_node == list.end())
  {
    next_node = list.begin();
  }
  return next_node;
}

inline double angularPositiveDistance(double a1, double a2)
{
  return std::min(std::abs(normalizeAngle(a1 - a2)), std::abs(normalizeAngle(a2 - a1)));
}

inline double vfactorPossibleObstacle(double dist_to_goal, double dist_to_obstacle, double robot_radius_padded, double safety_distance_quick)
{
  if (dist_to_obstacle < robot_radius_padded)
  {
    // The robot may have a non-null velocity when reaching this, just hope it
    // the robot radius is sufficiently padded.
    return 0.0;
  }
  return saturate(dist_to_goal / safety_distance_quick, 0.0, 1.0);
}

SNDPlanner::SNDPlanner(double robot_radius, double safety_distance, double safety_distance_quick) :
  robot_radius(robot_radius),
  safety_distance(safety_distance),
  safety_distance_quick(safety_distance_quick),
  farouche_factor(2),
  max_range(5),
  min_gap_width_(2 * (robot_radius + safety_distance))
{
  best_valley_ = valleys_.end();
  // TODO: compute min_gap_width_ from footprint.
}

void SNDPlanner::logError(const std::string& text) const
{
  if (log_error_f_)
  {
    log_error_f_(text);
  }
}

void SNDPlanner::logInfo(const std::string& text) const
{
  if (log_info_f_)
  {
    log_info_f_(text);
  }
}

void SNDPlanner::logDebug(const std::string& text) const
{
  if (log_debug_f_)
  {
    log_debug_f_(text);
  }
}

void SNDPlanner::logSuperdebug(const std::string& text) const
{
  if (log_superdebug_f_)
  {
    log_superdebug_f_(text);
  }
}

bool SNDPlanner::computeAngleAndVelocity(double goalx, double goaly, const std::vector<float>& scan, double& theta_traj, double& vfactor)
{
  logSuperdebug(str(boost::format("Robot radius: %f") % robot_radius)); // DEBUG
  logSuperdebug(str(boost::format("Safety distance: %f") % safety_distance)); // DEBUG
  logSuperdebug(str(boost::format("Safety distance (when quick): %f") % safety_distance_quick)); // DEBUG
  const size_t num_sectors = scan.size();
  double scan_resolution = 2 * M_PI / ((double)num_sectors);

  const double dist_to_goal = std::sqrt(goalx * goalx + goaly * goaly);
  const double angle_to_goal = std::atan2(goaly, goalx);
  const int goal_sector = (int)round(((double)num_sectors) / 2.0 + angle_to_goal / scan_resolution);

  logDebug(str(boost::format("Distance to goal: %.3f m; angle to goal: %.4f (%.3f deg)") %
        dist_to_goal % angle_to_goal % (angle_to_goal * 180 / M_PI)));

  const double smallest_observed_range = *(std::min_element(scan.begin(), scan.end()));
  
  /* In the case that the goal is inside all ranges, go to it */
  if (dist_to_goal < smallest_observed_range)
  {
    logDebug("No obstacle, going to goal");
    theta_traj = angle_to_goal;
    vfactor = vfactorPossibleObstacle(dist_to_goal, smallest_observed_range, robot_radius + safety_distance, safety_distance_quick);
    return true;
  }

  /* Locate gaps and valleys */
  std::list<Gap> gaps = getGaps(scan, goal_sector);

  if (gaps.empty())
  {
    const double min_distance = robot_radius + safety_distance;
    if (smallest_observed_range < min_distance)
    {
      // TODO: we do not need to stop if the obstacles are behind.
      logDebug("No gaps and obstacles too close, stopping");
      theta_traj = angle_to_goal;
      vfactor = 0.0;
    }
    else
    {
      logDebug("No gaps but no close obstacle, going towards the goal");
      theta_traj = angle_to_goal;
      vfactor = vfactorPossibleObstacle(dist_to_goal, smallest_observed_range, robot_radius + safety_distance, safety_distance_quick);
    }
    return true;
  }

  valleys_ = getValleys(gaps, scan);

  logSuperdebug(str(boost::format("Found %d valleys") % valleys_.size()));

#if ALLOW_SUPERDEBUG
  {
    std::list<Valley>::const_iterator valley;
    size_t i = 1;
    for (valley = valleys_.begin(); valley != valleys_.end(); ++valley, ++i)
    {
      const double theta_best = valley->bestGap().angle();
      const double theta_other = valley->otherGap().angle();
      double theta_mid;
      if (valley->bestGapIsRightGap())
      {
        theta_mid = normalizeAngle(theta_best + normalizeAngle(theta_other - theta_best) / 2);
      }
      else
      {
        theta_mid = normalizeAngle(theta_best - normalizeAngle(theta_best - theta_other) / 2);
      }
      logSuperdebug(str(boost::format("Valley %d: right, middle, left angles = %.3f, %.3f, %.3f") % i % valley->right_gap.angle() % theta_mid % valley->left_gap.angle()));
    }
  }
#endif

  best_valley_ = getBestValley(valleys_);

  if (best_valley_ == valleys_.end())
  {
    logDebug("No valley, stopping");
    theta_traj = 0.0;
    vfactor = 0.0;
    return false;
  }

  // Compute the margin to avoid touching the obstacle when going
  // in the obstacle direction.
  const double dist = best_valley_->bestGap().obstacleDistance();
  const double angle_margin = std::asin(saturate((robot_radius + safety_distance) / dist, -1.0, 1.0));
  logSuperdebug(str(boost::format("Angle margin: %.3f rad; obstacle distance: %.3f") % angle_margin % dist));

  /* Go to the goal if it is within the best valley */
  if (best_valley_->angleInValley(angle_to_goal, angle_margin))
  {
    logDebug("Goal angle inside the best valley, going toward goal");
    theta_traj = angle_to_goal;
    vfactor = vfactorPossibleObstacle(dist_to_goal, 2 * (robot_radius + safety_distance), robot_radius + safety_distance, safety_distance_quick);
    return true;
  }

  /* Determine the desired heading. */
  /* Best heading within the best valley. */
  const double theta_best = best_valley_->bestGap().angle();
  logSuperdebug(str(boost::format("Best angle of best valley: %.3f rad") % theta_best));
  
  /* Heading along best gap with a clearance. */
  double theta_safe_best;
  if (best_valley_->bestGapIsRightGap())
  {
    theta_safe_best = theta_best + angle_margin;
  }
  else
  {
    theta_safe_best = theta_best - angle_margin;
  }
  logSuperdebug(str(boost::format("Angle with best progress towards goal: %.3f rad") % theta_safe_best));

  /* Heading towards the middle of the valley. */
  // TODO: this should not be necessary because the valley is knwon to be wide enough.
  // TODO: the valley is wide enough at its narrowest place, so its angle range should be even wider.
  double theta_mid_best;
  if (best_valley_->bestGapIsRightGap())
  {
    const double theta_other = best_valley_->otherGap().angle();
    theta_mid_best = normalizeAngle(theta_best + normalizeAngle(theta_other - theta_best) / 2);
  }
  else
  {
    const double theta_other = best_valley_->otherGap().angle();
    theta_mid_best = normalizeAngle(theta_best - normalizeAngle(theta_best - theta_other) / 2);
  }
  logSuperdebug(str(boost::format("Middle angle of best valley: %.3f rad") % theta_mid_best));

  /* Desidered heading. */
  double theta_desired = theta_safe_best;
  if (angularPositiveDistance(theta_best, theta_mid_best) < angularPositiveDistance(theta_best, theta_safe_best))
  {
    theta_desired = theta_mid_best;
  }

  logDebug(str(boost::format("Desired angle (before deflection): %.3f rad") % theta_desired));

  /* Obstacle avoidance, i.e. deflection from theta_desired */
  std::vector<double> threats;
  const double theta_deflection = computeProximityDeflection(scan, theta_desired, threats);

  logDebug(str(boost::format("Deflection: %.3f rad") % theta_deflection));

  /* Final trajectory angle */
  theta_traj = normalizeAngle(theta_desired + theta_deflection);

  logDebug(str(boost::format("Final angle: %.3f rad") % theta_traj));

  /* final velocities */
  const double greatest_threat = greatestThreat(theta_traj, threats);
  vfactor = 1 - greatest_threat;

  logDebug(str(boost::format("Velocity factor: %.3f") % vfactor));

  return true;
}

std::list<Gap> SNDPlanner::getGaps(const std::vector<float>& scan, size_t goal_sector) const
{
  std::list<Gap> gaps;

  for (int i = 0; i < (int)scan.size(); ++i)
  {
    const double previous_range = scan[circularIndex(i - 1, scan.size())];
    const double next_range = scan[circularIndex(i + 1, scan.size())];
    const double step_from_previous = scan[i] - previous_range; 
    const double step_to_next = scan[i] - next_range;
    const bool is_falling = step_to_next > min_gap_width_;
    const bool is_rising = step_from_previous > min_gap_width_;
    if (is_rising && is_falling)
    {
      gaps.push_back(Gap(i, scan, goal_sector, BOTH_GAP));
#if ALLOW_SUPERDEBUG
      /* logSuperdebug(str(boost::format("Sector %d (%.3f rad): BOTH_GAP") % i % ((double)i / scan.size() * 2 * M_PI - M_PI))); */
#endif
    }
    else if (is_rising)
    {
      gaps.push_back(Gap(i, scan, goal_sector, RISING_GAP));
#if ALLOW_SUPERDEBUG
      /* logSuperdebug(str(boost::format("Sector %d (%.3f rad): RISING_GAP") % i % ((double)i / scan.size() * 2 * M_PI - M_PI))); */
#endif
    }
    else if (is_falling)
    {
      gaps.push_back(Gap(i, scan, goal_sector, FALLING_GAP));
#if ALLOW_SUPERDEBUG
      /* logSuperdebug(str(boost::format("Sector %d (%.3f rad): FALLING_GAP") % i % ((double)i / scan.size() * 2 * M_PI - M_PI))); */
#endif
    }
  }
  return gaps;
}

std::list<Valley> SNDPlanner::getValleys(const std::list<Gap>& gaps, const std::vector<float>& scan) const
{
  std::list<Valley> valleys;

  /* this_gap is the right gap */
  std::list<Gap>::const_iterator this_gap;
  for (this_gap = gaps.begin(); this_gap != gaps.end(); ++this_gap)
  {
#if ALLOW_SUPERDEBUG
    /* logSuperdebug(str(boost::format("Examining region with right gap %d") % this_gap->sector)); */
    if (this_gap->type == BOTH_GAP)
    {
      logSuperdebug(str(boost::format("Examining region with both gap at %d") % this_gap->sector));
    }
#endif
    if (this_gap->type == BOTH_GAP && isSpaceTraversable(*this_gap, *this_gap, scan.size()))
    {
      valleys.push_back(Valley(*this_gap, *this_gap));
      continue;
    }
 
    /* next_gap is the left gap */
    std::list<Gap>::const_iterator next_gap = circularNext(gaps, this_gap);

    const bool rising_falling = this_gap->type == RISING_GAP && next_gap->type == FALLING_GAP;
    const bool rising_rising = this_gap->type == RISING_GAP && next_gap->type == RISING_GAP;
    const bool falling_falling = this_gap->type == FALLING_GAP && next_gap->type == FALLING_GAP;
#if ALLOW_SUPERDEBUG
    logSuperdebug(str(boost::format("Examining region between %d (%s) and %d (%s)") % this_gap->sector %
          (this_gap->type == RISING_GAP ? "rising" : "falling") %
          next_gap->sector %
          (next_gap->type == RISING_GAP ? "rising" : "falling")));
    logSuperdebug(str(boost::format("  rising on right or falling on left: %s") % ((rising_falling || rising_rising || falling_falling)? "true" : "false")));
#endif

    if (rising_falling || rising_rising || falling_falling)
    {
      const bool space_traversable = isSpaceTraversable(*this_gap, *next_gap, scan.size());
      if (space_traversable)
      {
        /* We have a valley. On the opposite, a region which is not a valley would be falling_rising. */
        valleys.push_back(Valley(*this_gap, *next_gap));
      }
#if ALLOW_SUPERDEBUG
      logSuperdebug(str(boost::format("  space_traversable: %s") % (space_traversable ? "true" : "false")));
#endif
    }
  }

  return valleys;
}

bool SNDPlanner::isSpaceTraversable(const Gap& right_gap, const Gap& left_gap, size_t num_sectors) const
{
  const double right_angle = sectorToAngle(right_gap.sector - 1, num_sectors);
  const double right_x = right_gap.previous_range * std::cos(right_angle);
  const double right_y = right_gap.previous_range * std::sin(right_angle);
  const double left_angle = sectorToAngle(left_gap.sector + 1, num_sectors);
  const double left_x = left_gap.next_range * std::cos(left_angle);
  const double left_y = left_gap.next_range * std::sin(left_angle);

  const double dist2 = (right_x - left_x) * (right_x - left_x) + (right_y - left_y) * (right_y - left_y);

#if ALLOW_SUPERDEBUG
  logSuperdebug(str(boost::format("  isSpaceTraversable: right point = (%.3f, %.3f), angle = %.3f") % right_x % right_y % right_angle));
  logSuperdebug(str(boost::format("  isSpaceTraversable: left point = (%.3f, %.3f), angle = %.3f") % left_x % left_y % left_angle));
  logSuperdebug(str(boost::format("  isSpaceTraversable: Region between %d and %d is %straversable: %.3f %s %.3f") %
        right_gap.sector %
        left_gap.sector %
        ((dist2 > min_gap_width_ * min_gap_width_) ? "" : "not ") %
        std::sqrt(dist2) %
        ((dist2 > min_gap_width_ * min_gap_width_) ? ">" : "<") %
        min_gap_width_));
#endif
  return dist2 > min_gap_width_ * min_gap_width_;
}

std::list<Valley>::const_iterator SNDPlanner::getBestValley(const std::list<Valley>& valleys) const
{
  std::list<Valley>::const_iterator best_valley = valleys.end();
  unsigned int best_dist = std::numeric_limits<unsigned int>::max();

  std::list<Valley>::const_iterator this_valley = valleys.begin();
#if ALLOW_SUPERDEBUG
    size_t i = 1;
#endif
  for (; this_valley != valleys.end(); ++this_valley)
  {
    const unsigned int this_dist = std::abs(this_valley->bestGap().sectorDistanceToGoal());
#if ALLOW_SUPERDEBUG
    logSuperdebug(str(boost::format("Valley %d: sector distance to goal: %d") % i % this_dist));
    i++;
#endif

    if (this_dist < best_dist)
    {
      best_dist = this_dist;
      best_valley = this_valley;
    }
  }
  return best_valley;
}

double SNDPlanner::computeProximityDeflection(const std::vector<float>& scan, double theta_desired, std::vector<double>& threats) const
{
  double threat_total = 0.0;
  threats.resize(scan.size());
  std::vector<double> deflections(scan.size());
  for (size_t i = 0; i < scan.size(); ++i)
  {
    double this_threat;
    if ((safety_distance_quick - safety_distance - robot_radius) > 0)
    {
      this_threat = saturate((safety_distance_quick - scan[i]) / (safety_distance_quick - safety_distance - robot_radius), 0.0, 1.0);
    }
    else
    {
      const double shortest_safety_distance = std::min(safety_distance_quick, robot_radius + safety_distance);
      if (scan[i] < shortest_safety_distance)
      {
        this_threat = 1.0;
      }
      else
      {
        this_threat = 0.0;
      }
    }
    const double opposite_angle = normalizeAngle(sectorToAngle(i, scan.size()) + M_PI);
    const double this_deflection = this_threat * normalizeAngle(opposite_angle - theta_desired);
    threats[i] = this_threat;
    deflections[i] = this_deflection;
    threat_total += std::pow(this_threat, farouche_factor);
#if ALLOW_SUPERDEBUG
    /* logSuperdebug(str(boost::format("scan %d: angle = %f, range = %f, threat = %f, deflection = %f") % i % sectorToAngle(i, scan.size()) % scan[i] % this_threat % this_deflection)); */
#endif
  }

  if (threat_total == 0.0)
  {
    // No deflection.
    return 0.0;
  }

  double deflection_total = 0.0;
  for (size_t i = 0; i < scan.size(); ++i)
  {
    deflection_total += threats[i] * deflections[i];
  }
  return deflection_total / threat_total;
}

double SNDPlanner::greatestThreat(double theta_traj, const std::vector<double>& threats)
{
  // No need to normalize here.
  const double right_angle = theta_traj - M_PI_2;
  const double left_angle = theta_traj + M_PI_2;
  const size_t num_sectors = threats.size();
  double max_threat = 0.0;
  for (size_t i = 0; i < num_sectors; ++i)
  {
    const bool in_dangerous_zone = angleInValley(sectorToAngle(i, num_sectors), right_angle, left_angle);
    if (in_dangerous_zone && (threats[i] > max_threat))
    {
      max_threat = threats[i];
    }
  }
  return max_threat;
}

Valley SNDPlanner::bestValley() const
{
  if (best_valley_ != valleys_.end())
  {
    return *best_valley_;
  }
  else
  {
    std::vector<float> minimum_scan(3);
    Gap dummy_gap(0, minimum_scan, 0, UNDEFINED_GAP);
    return Valley(dummy_gap, dummy_gap);
  }
}

} /* namespace snd */

