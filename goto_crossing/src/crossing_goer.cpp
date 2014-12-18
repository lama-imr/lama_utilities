#include <goto_crossing/crossing_goer.h>

namespace goto_crossing {

CrossingGoer::CrossingGoer() :
  kp_v_(0.1),
  kp_w_(0.2),
  ki_v_(0),
  ki_w_(0),
  min_linear_velocity_(0.020),
  min_angular_velocity_(0.1),
  reach_distance_(0.050),
  dtheta_force_left_(0),
  threshold_w_only_(0.35),
  max_sum_v_(10),
  max_sum_w_(30),
  last_t_(ros::Time::now()),
  sum_v_(0),
  sum_w_(0)
{
  // Log level
  if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
  {
    ros::console::notifyLoggerLevelsChanged();
  }

  ros::NodeHandle private_nh("~");
  private_nh.getParam("kp_v", kp_v_);
  private_nh.getParam("kp_w", kp_w_);
  private_nh.getParam("ki_v", ki_v_);
  private_nh.getParam("ki_w", ki_w_);
  private_nh.getParam("min_linear_velocity", min_linear_velocity_);
  private_nh.getParam("min_angular_velocity", min_angular_velocity_);
  private_nh.getParam("reach_distance", reach_distance_);
  private_nh.getParam("dtheta_force_left", dtheta_force_left_);
  private_nh.getParam("threshold_w_only", threshold_w_only_);
  private_nh.getParam("max_sum_v", max_sum_v_);
  private_nh.getParam("max_sum_w", max_sum_w_);

  twist_publisher_ = private_nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  goal_reached_publisher_ = private_nh.advertise<std_msgs::Bool>("goal_reached", 1);
  
  reset_integral_server_ = private_nh.advertiseService("reset_integrals", &CrossingGoer::callback_resetIntegrals, this);

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

  // Can only reach the goal if at least 3 exits.
  bool can_reach = (crossing.frontiers.size() >= 3);

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
    // Move to the crossing center.
    goal.x = crossing.center.x;
    goal.y = crossing.center.y;
  }
  ROS_DEBUG("%s: goal: (%.3f, %.3f)", ros::this_node::getName().c_str(), goal.x, goal.y);
  return goToGoal(goal, twist) && can_reach;
}

/* Callback for the Crossing topic.
 *
 * Compute and publish the Twist and Bool messages.
 */
void CrossingGoer::callback_goto_crossing(const lama_msgs::Crossing& crossing)
{
  // TODO: add a timeout mechanism for Crossing reception and set twist to 0.
  geometry_msgs::Twist twist;
  bool goal_reached = goto_crossing(crossing, twist);

  twist_publisher_.publish(twist);
  std_msgs::Bool goal_reached_msg;
  goal_reached_msg.data = goal_reached;
  goal_reached_publisher_.publish(goal_reached_msg);
}

bool CrossingGoer::callback_resetIntegrals(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  resetIntegrals();
  return true;
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
  // TODO: use dynamic_reconfigure instead of getParamCached.
  ros::NodeHandle private_nh("~");
  private_nh.getParamCached("kp_v", kp_v_);
  private_nh.getParamCached("kp_w", kp_w_);
  private_nh.getParamCached("ki_v", ki_v_);
  private_nh.getParamCached("ki_w", ki_w_);
  private_nh.getParamCached("min_linear_velocity", min_linear_velocity_);
  private_nh.getParamCached("min_angular_velocity", min_angular_velocity_);
  private_nh.getParamCached("reach_distance", reach_distance_);
  private_nh.getParamCached("dtheta_force_left", dtheta_force_left_);
  private_nh.getParamCached("threshold_w_only", threshold_w_only_);
  private_nh.getParamCached("max_sum_v", max_sum_v_);
  private_nh.getParamCached("max_sum_w", max_sum_w_);

  double distance = std::sqrt(goal.x * goal.x + goal.y * goal.y);

  if (distance < reach_distance_)
  {
    ROS_DEBUG("%s: Goal (%f, %f) reached", ros::this_node::getName().c_str(), goal.x, goal.y);
    twist = geometry_msgs::Twist();
    // Return true to indicate that the goal is reached.
    return true;
  }

  double dtheta = std::atan2(goal.y, goal.x);

  if (std::abs(dtheta) > threshold_w_only_)
  {
    // Do no go forward because the goal is not well in front of the robot.
    ROS_DEBUG("%s: Goal angle too large, just turning...", ros::this_node::getName().c_str());
    distance = 0.0;
  }

  if ((M_PI - dtheta_force_left_ < dtheta) && (dtheta < M_PI))
  {
    // If the goal is behind the robot, force the robot to turn left.
    dtheta -= 2 * M_PI;
  }

  // Compute the integrals.
  const ros::Time t = ros::Time::now();
  const double dt = (t - last_t_).toSec();
  sum_v_ += distance * dt;
  sum_w_ += dtheta * dt;

  // Anti wind-up.
  if (sum_v_ < -max_sum_v_)
  {
    sum_v_ = -max_sum_v_;
  }
  else if (sum_v_ > max_sum_v_)
  {
    sum_v_ = max_sum_v_;
  }
  if (sum_w_ < -max_sum_w_)
  {
    sum_w_ = -max_sum_w_;
  }
  else if (sum_w_ > max_sum_w_)
  {
    sum_w_ = max_sum_w_;
  }
  
  // TODO: add a parameter "allow_backward" so that we don't need to turn 180 deg.
  
  double vx = kp_v_ * distance + ki_v_ * sum_v_; 
  double wz = kp_w_ * dtheta + ki_w_ * sum_w_;

  // Linear velocity throttle depending on dtheta (full velocity
  // when dtheta = 0, 0 when dtheta = threshold_w_only_).
  const double vx_throttle_dtheta = std::max(std::min(1.0, 1.0 - std::abs(dtheta) / threshold_w_only_), 0.0);
  vx = vx * vx_throttle_dtheta;

  // Dead-zone management (not needed if ki_v_ and ki_w_ non null).
  if ((ki_v_ < 1e-10) && (vx < min_linear_velocity_) &&
      (std::abs(distance) > 1e-10) && (std::abs(wz) <= min_angular_velocity_))
  {
    vx = min_linear_velocity_;
  }
  if ((ki_w_ < 1e-10) && (wz > 0) && (wz < min_angular_velocity_) && (vx <= min_linear_velocity_))
  {
    wz = min_angular_velocity_;
  }
  else if ((ki_w_ < 1e-10) && (wz < 0) && (wz > -min_angular_velocity_) && (vx <= min_linear_velocity_))
  {
    wz = -min_angular_velocity_;
  }
  ROS_DEBUG("%s: distance to goal: %f, dtheta to goal: %f, twist: (%f, %f)", ros::this_node::getName().c_str(),
      distance, dtheta, vx, wz);

  ROS_DEBUG_NAMED("superdebug", "sum_v_: %f", sum_v_);
  ROS_DEBUG_NAMED("superdebug", "sum_w_: %f", sum_w_);

  twist.linear.x = vx;
  twist.angular.z = wz;

  last_t_ = t;
  return false;
}

} // namespace goto_crossing
