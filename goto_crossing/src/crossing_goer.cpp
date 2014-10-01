#include <goto_crossing/crossing_goer.h>

namespace lama {
namespace goto_crossing {

const double CrossingGoer::threshold_w_only_ = 1.0;  // (rad), ~60 deg.

CrossingGoer::CrossingGoer() :
  last_t_(ros::Time::now()),
  sum_v_(0),
  sum_w_(0)
{
  // Log level
  if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
  {
    ros::console::notifyLoggerLevelsChanged();
  }

  ros::NodeHandle private_nh;
  if (!private_nh.getParamCached("kp_v", kp_v_))
    kp_v_ = 0.1;
  if (!private_nh.getParamCached("kp_w", kp_w_))
    kp_w_ = 0.2;
  if (!private_nh.getParamCached("ki_v", ki_v_))
    ki_v_ = 0.00;
  if (!private_nh.getParamCached("ki_w", ki_w_))
    ki_w_ = 0.00;
  if (!private_nh.getParamCached("min_linear_velocity", min_linear_velocity_))
    min_linear_velocity_ = 0.020;
  if (!private_nh.getParamCached("min_angular_velocity", min_angular_velocity_))
    min_angular_velocity_ = 0.1;
  if (!private_nh.getParamCached("reach_distance", reach_distance_))
    reach_distance_ = 0.050;

  twist_publisher_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  goal_reached_publisher_ = nh_.advertise<std_msgs::Bool>("goal_reached", 1);
  
  ROS_INFO("%s: CrossingGoer initialized", ros::this_node::getName().c_str());
}

/* Compute the twist needed to reach the goal
 *
 * Return true if the goal is reached, false otherwise.
 *
 * crossing[in] crossing descriptor relative to the robot
 * twist[out] twist to apply (all null except linear.x and angular.z).
 */
bool CrossingGoer::goto_crossing(const lama_msgs::Crossing& crossing, geometry_msgs::Twist& twist)
{
  geometry_msgs::Point goal;
  //!> can only reach the goal if at least 3 exits.
  bool can_reach = false;

  if (crossing.frontiers.size() == 0)
  {
    // Move straight forward.
    goal.x = crossing.radius;
  }
  else if (crossing.frontiers.size() == 1)
  {
    // Move to the sole frontier.
    // DOES NOT WORK VERY WELL.
    // goal.x = (crossing.frontiers[0].p1.x + crossing.frontiers[0].p2.x) / 2;
    // goal.y = (crossing.frontiers[0].p1.y + crossing.frontiers[0].p2.y) / 2;
    goal.x = std::cos(crossing.frontiers[0].angle) * crossing.radius;
    goal.y = std::sin(crossing.frontiers[0].angle) * crossing.radius;
  }
  else if (crossing.frontiers.size() == 2)
  {
    // Move to the frontier that is the closest to forward.
    double dtheta0 = angles::normalize_angle(crossing.frontiers[0].angle);
    double dtheta1 = angles::normalize_angle(crossing.frontiers[1].angle);
    
    if (std::abs(dtheta0) < std::abs(dtheta1))
    {
      // DOES NOT WORK VERY WELL.
      // goal.x = (crossing.frontiers[0].p1.x + crossing.frontiers[0].p2.x) / 2;
      // goal.y = (crossing.frontiers[0].p1.y + crossing.frontiers[0].p2.y) / 2;
      goal.x = std::cos(dtheta0) * crossing.radius;
      goal.y = std::sin(dtheta0) * crossing.radius;
    }
    else
    {
      // DOES NOT WORK VERY WELL.
      // goal.x = (crossing.frontiers[1].p1.x + crossing.frontiers[1].p2.x) / 2;
      // goal.y = (crossing.frontiers[1].p1.y + crossing.frontiers[1].p2.y) / 2;
      goal.x = std::cos(dtheta1) * crossing.radius;
      goal.y = std::sin(dtheta1) * crossing.radius;
    }
  }
  else
  {
    can_reach = true;
    // Move to the crossing center.
    goal.x = crossing.center.x;
    goal.y = crossing.center.y;
  }
  ROS_DEBUG("%s: goal = (%.3f, %.3f)", ros::this_node::getName().c_str(), goal.x, goal.y);
  return goToGoal(goal, twist) && can_reach;
}

/* Callback for the Crossing topic.
 */
void CrossingGoer::goto_crossing_callback(const lama_msgs::Crossing& crossing)
{
  // TODO: add a timeout mechanism for Crossing reception and set twist to 0.
  geometry_msgs::Twist twist;
  bool goal_reached = goto_crossing(crossing, twist);

  twist_publisher_.publish(twist);
  std_msgs::Bool goal_reached_msg;
  goal_reached_msg.data = goal_reached;
  goal_reached_publisher_.publish(goal_reached_msg);
}

/* Return the twist to reach the given goal pose
 * 
 * There is no cycle in this function, so it should be called periodically by class instances.
 *
 * goal[in] position of the goal relative to the robot.
 * twist[out] twist to apply (all null except linear.x and angular.z).
 */
bool CrossingGoer::goToGoal(const geometry_msgs::Point& goal, geometry_msgs::Twist& twist)
{
  ros::Time t = ros::Time::now();

  double distance = std::sqrt(goal.x * goal.x + goal.y * goal.y);

  if (distance < reach_distance_)
  {
    ROS_DEBUG("%s: Goal (%f, %f) reached", ros::this_node::getName().c_str(), goal.x, goal.y);
    twist = geometry_msgs::Twist();
    // Return true to indicate that the goal is reached.
    return true;
  }

  double dtheta = std::atan2(goal.y, goal.x);
  ROS_DEBUG("%s: distance to goal: %f, dtheta to goal: %f", ros::this_node::getName().c_str(), distance, dtheta);

  if (std::abs(dtheta) > threshold_w_only_)
  {
    // Do no go forward because the goal is not well in front of the robot.
    distance = 0.0;
  }

  // Compute the integrals.
  // TODO: Add anti wind-up.
  double dt = (t - last_t_).toSec();
  if (dt != 0)
  {
    sum_v_ += distance / dt;
    sum_w_ += dtheta / dt;
  }
  last_t_ = t;
  
  // TODO: add a parameter "allow_backward" so that we don't need to turn 180 deg.
  
  double vx = kp_v_ * distance + ki_v_ * sum_v_; 
  double wz = kp_w_ * dtheta + ki_w_ * sum_w_;

  // Dead-zone management (not need if ki_v_ and ki_w_ non null).
  if ((vx < min_linear_velocity_) && (distance != 0) && (std::abs(wz) < min_angular_velocity_))
  {
    vx = min_linear_velocity_;
  }
  if ((wz > 0) && (wz < min_angular_velocity_) && (vx < min_linear_velocity_))
  {
    wz = min_angular_velocity_;
  }
  else if ((wz < 0) && (wz > -min_angular_velocity_) && (vx < min_linear_velocity_))
  {
    wz = -min_angular_velocity_;
  }

  twist.linear.x = vx;
  twist.angular.z = wz;

  return false;
}

} // namespace goto_crossing
} // namespace lama
