#include <goto_crossing/crossing_goer.h>

namespace lama {
namespace goto_crossing {

const double CrossingGoer::threshold_w_only_ = 1.0;  // (rad), ~60 deg.

CrossingGoer::CrossingGoer() :
  nh_("~")
{
  if (!nh_.getParamCached("kp_v", kp_v_))
    kp_v_ = 0.05;
  if (!nh_.getParamCached("kp_w", kp_w_))
    kp_w_ = 0.2;
  if (!nh_.getParamCached("min_velocity", min_velocity_))
    min_velocity_ = 0.020;
  if (!nh_.getParamCached("reach_distance", reach_distance_))
    reach_distance_ = 0.050;

  twist_publisher_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  goal_reached_publisher_ = nh_.advertise<std_msgs::Bool>("goal_reached", 1);
  // TODO: add a timeout mechanism for Crossing reception and set twist to 0.
}

bool CrossingGoer::goto_crossing(const lama_msgs::Crossing& crossing, geometry_msgs::Twist& twist) const
{
  geometry_msgs::Point goal;

  if (crossing.frontiers.size() == 0)
  {
    // Move straight forward.
    goal.x = crossing.radius;
  }
  else if (crossing.frontiers.size() == 1)
  {
    // Move to the sole frontier.
    goal.x = (crossing.frontiers[0].p1.x + crossing.frontiers[0].p2.x) / 2;
    goal.y = (crossing.frontiers[0].p1.y + crossing.frontiers[0].p2.y) / 2;
  }
  else if (crossing.frontiers.size() == 2)
  {
    // Move to the frontier that is the closest to forward.
    double dtheta0 = std::fmod(crossing.frontiers[0].angle + M_PI, 2 * M_PI) - M_PI;
    double dtheta1 = std::fmod(crossing.frontiers[1].angle + M_PI, 2 * M_PI) - M_PI;
    
    if (std::abs(dtheta0) < std::abs(dtheta1))
    {
      goal.x = (crossing.frontiers[0].p1.x + crossing.frontiers[0].p2.x) / 2;
      goal.y = (crossing.frontiers[0].p1.y + crossing.frontiers[0].p2.y) / 2;
    }
    else
    {
      goal.x = (crossing.frontiers[1].p1.x + crossing.frontiers[1].p2.x) / 2;
      goal.y = (crossing.frontiers[1].p1.y + crossing.frontiers[1].p2.y) / 2;
    }
  }
  else
  {
    // Move to the crossing center.
    goal.x = crossing.center.x;
    goal.y = crossing.center.y;
  }
  return goToGoal(goal, twist);
}

/* Callback for the Crossing topic.
 */
void CrossingGoer::goto_crossing_callback(const lama_msgs::Crossing& crossing) const
{
  geometry_msgs::Twist twist;
  bool goal_reached = goto_crossing(crossing, twist);

  twist_publisher_.publish(twist);
  goal_reached_publisher_.publish((uint8_t)goal_reached);
}

/* Return the twist to reach the given goal pose
 * 
 * There is no cycle in this function, so it should be called periodically by class instances.
 *
 * goal[in] position of the goal relative to the robot
 */
bool CrossingGoer::goToGoal(const geometry_msgs::Point& goal, geometry_msgs::Twist& twist) const
{
  double distance = std::sqrt(goal.x * goal.x + goal.y * goal.y);

  if (distance < reach_distance_)
  {
    ROS_DEBUG("Goal (%f, %f) reached", goal.x, goal.y);
    twist = geometry_msgs::Twist();
    // Return true to indicate that the goal is reached.
    return true;
  }

  double dtheta = std::atan2(goal.y, goal.x);
  ROS_DEBUG("distance to goal: %f, dtheta to goal: %f", distance, dtheta);

  if (std::fabs(dtheta) > threshold_w_only_)
  {
    // Do no go forward because the goal is not well in front of the robot.
    distance = 0.0;
  }

  // TODO: add a parameter "allow_backward" so that we don't need to turn 180 deg.
  
  double vx = kp_v_ * distance; 
  double wz = kp_w_ * dtheta;
  if (vx < min_velocity_)
  {
    vx = min_velocity_;
  }

  twist.linear.x = vx;
  twist.angular.z = wz;

  return false;
}

} // namespace goto_crossing
} // namespace lama
