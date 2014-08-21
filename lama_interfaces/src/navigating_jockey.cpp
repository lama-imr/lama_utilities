/* Base class for navigating jockeys
 *
 */

#include <lama_interfaces/navigating_jockey.h>
#include <lama_interfaces/LamaObjectIdentifier.h>
#include <lama_interfaces/LamaDescriptorIdentifier.h>

namespace lama
{
namespace interfaces
{

NavigatingJockey::NavigatingJockey(std::string name) :
  Jockey(name),
  server_(nh_, name, false),
  goal_reached_(false)
{
  server_.registerGoalCallback(boost::bind(&NavigatingJockey::goalCallback, this));
  server_.registerPreemptCallback(boost::bind(&NavigatingJockey::preemptCallback, this));

  server_.start();

  ros::NodeHandle n("~");
  if (!n.getParamCached("max_goal_distance", max_goal_distance_))
    max_goal_distance_ = 10.0;
  if (!n.getParamCached("max_goal_dtheta", max_goal_dtheta_))
    max_goal_dtheta_ = 0.785;  // 45 deg
  if (!n.getParamCached("kp_v", kp_v_))
    kp_v_ = 0.05;
  if (!n.getParamCached("kp_w", kp_w_))
    kp_w_ = 0.2;
  if (!n.getParamCached("min_velocity", min_velocity_))
    min_velocity_ = 0.020;
  if (!n.getParamCached("reach_distance", reach_distance_))
    reach_distance_ = 0.050;
}

void NavigatingJockey::goalCallback()
{
  lama_interfaces::NavigateGoalConstPtr current_goal = server_.acceptNewGoal();
  goal_.action = current_goal->action;

  // Check that preempt has not been requested by the client.
  if (server_.isPreemptRequested() || !ros::ok())
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    server_.setPreempted();
    return;
  }

  switch (goal_.action)
  {
    case lama_interfaces::NavigateGoal::STOP:
      initAction();
      // Reset the goal, just in case.
      goal_.edge = lama_interfaces::LamaObjectIdentifier();
      goal_.descriptor = lama_interfaces::LamaDescriptorIdentifier();
      onStop();
      break;
    case lama_interfaces::NavigateGoal::TRAVERSE:
      initAction();
      goal_.edge = current_goal->edge;
      goal_.descriptor = current_goal->descriptor;
      onTraverse();
      break;
    case lama_interfaces::NavigateGoal::INTERRUPT:
      interrupt();
      onInterrupt();
      break;
    case lama_interfaces::NavigateGoal::CONTINUE:
      resume();
      onContinue();
      break;
  }
}

void NavigatingJockey::preemptCallback()
{
  ROS_INFO("%s: Preempted", action_name_.c_str());
  // set the action state to preempted
  server_.setPreempted();
}

void NavigatingJockey::onInterrupt()
{
  ROS_DEBUG("%s: navigating goal %d interrupted", action_name_.c_str(), goal_.edge.object_id);
}

void NavigatingJockey::onContinue()
{
  ROS_DEBUG("%s: navigating goal %d resumed", action_name_.c_str(), goal_.edge.object_id);
}

/* Return the twist to reach the given goal pose
 * 
 * There is no cycle in this function, so it should be called periodically by class instances.
 *
 * goal[in] position of the goal relative to the robot
 */
geometry_msgs::Twist NavigatingJockey::goToGoal(const geometry_msgs::Point& goal)
{
  geometry_msgs::Twist twist;

  if (isGoalReached())
    return twist;

  double distance = std::sqrt(goal.x * goal.x + goal.y * goal.y);
  if (distance > max_goal_distance_)
  {
    ROS_DEBUG("distance to goal (%f) is greater than max (%f)", distance, max_goal_distance_);
    return twist;
  }

  if (distance < reach_distance_)
  {
    setGoalReached();
    return twist;
  }

  double dtheta = std::atan2(goal.y, goal.x);
  ROS_DEBUG("distance to goal: %f, dtheta to goal: %f", distance, dtheta);
  if (dtheta > max_goal_dtheta_) dtheta = max_goal_dtheta_;
  if (dtheta < -max_goal_dtheta_) dtheta = -max_goal_dtheta_;

  // Only move forward if the goal is in front of the robot (+/- max_goal_dtheta_).
  // The linear velocity is max if the goal is in front of the robot and 0 if at max_goal_dtheta_.
  double vx = kp_v_ * distance * (max_goal_dtheta_ - std::fabs(dtheta)) / max_goal_dtheta_; 
  double wz = kp_w_ * dtheta;
  if (vx < min_velocity_)
  {
    vx = min_velocity_;
  }

  twist.linear.x = vx;
  twist.angular.z = wz;
  return twist;
}

} // namespace interfaces
} // namespace lama

