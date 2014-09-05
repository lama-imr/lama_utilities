#include <nj_escape_crossing/crossing_escaper.h>

namespace lama {
namespace nj_escape_crossing {

const double CrossingEscaper::reach_angular_distance_ = 0.017;  // (rad), 1 deg
const double CrossingEscaper::threshold_w_only_ = 1.0;  // (rad), ~60 deg.
const double CrossingEscaper::max_data_age_ = 0.050;

CrossingEscaper::CrossingEscaper(std::string name, double escape_distance) :
  lama::NavigatingJockey(name),
  escape_distance_(escape_distance),
  twist_advertised_(false),
  angle_reached_(false),
  goal_reached_(false),
  has_data_(false)
{
  if (!nh_.getParamCached("kp_v", kp_v_))
    kp_v_ = 0.05;
  if (!nh_.getParamCached("kp_w", kp_w_))
    kp_w_ = 0.2;
  if (!nh_.getParamCached("min_velocity", min_velocity_))
    min_velocity_ = 0.020;
  if (!nh_.getParamCached("min_angular_velocity", min_angular_velocity_))
    min_velocity_ =  0.1;
  nh_.getParamCached("escape_distance", escape_distance_);
}

/* First orient the robot, then travel at least "escape_distance_".
 */
void CrossingEscaper::onTraverse()
{
  // TODO: Add a mechanism to stop before escape_distance_ if
  // the robot sees only 2 crossing exits on a certain distance.
  
  twist_publisher_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  twist_advertised_ = true;
  crossing_subscriber_ = nh_.subscribe("crossing", 1, &CrossingEscaper::crossing_callback, this);
  direction_subscriber_ =  nh_.subscribe("direction", 1, &CrossingEscaper::crossing_callback, this);
  odometry_subscriber_ = nh_.subscribe("odometry", 1, &CrossingEscaper::odometry_callback, this);

  // Wait for data.
  ros::Rate r_data(100);
  while (!has_data_)
  {
    r_data.sleep();
  }
  start_position_ = odometry_;
  feedback_.completion = 0.05;
  server_.publishFeedback(feedback_);

  geometry_msgs::Twist twist;

  ros::Rate r(50);
  while (ros::ok() && goal_.action == lama_jockeys::NavigateGoal::TRAVERSE)
  {
    // TODO: add a timeout mechanism if odometry_ is not updated.
    if (!angle_reached_)
    {
      angle_reached_ = turnToAngle(direction_, twist);
      twist_publisher_.publish(twist);
      if (angle_reached_)
      {
        feedback_.completion = 0.25;
        server_.publishFeedback(feedback_);
      }
    }
    else if (!goal_reached_)
    {
      geometry_msgs::Point goal = goalFromOdometry();
      goal_reached_ = goToGoal(goal, twist);
      twist_publisher_.publish(twist);
    }
    else
    {
      // The robot reached its goal.
      feedback_.completion = 1;
      server_.publishFeedback(feedback_);
      result_.final_state = lama_jockeys::NavigateResult::DONE;
      result_.completion_time = ros::Time::now() - getStartTime() - getInterruptionsDuration();
      server_.setSucceeded(result_);
      break;
    }
    r.sleep();
  }
  twist_publisher_.shutdown();
  twist_advertised_ = false;
  crossing_subscriber_.shutdown();
  direction_subscriber_.shutdown();
  odometry_subscriber_.shutdown();
  angle_reached_ = false;
  goal_reached_ = false;
}

void CrossingEscaper::onStop()
{
  if (twist_advertised_)
  {
    twist_publisher_.publish(geometry_msgs::Twist());
    twist_publisher_.shutdown();
    twist_advertised_ = false;
  }
  result_.final_state = lama_jockeys::NavigateResult::DONE;
  result_.completion_time = ros::Duration(0);
  server_.setSucceeded(result_);
}

void CrossingEscaper::crossing_callback(const lama_msgs::Crossing& crossing)
{
  crossing_ = crossing;
  crossing_timestamp_ = ros::Time::now();

  has_data_ = (direction_timestamp_.toSec() < max_data_age_) &&
      (odometry_.header.stamp.toSec() < max_data_age_);
}

void CrossingEscaper::direction_callback(const std_msgs::Float32& direction)
{
  direction_ = direction.data;
  direction_timestamp_ = ros::Time::now();

  has_data_ = (crossing_timestamp_.toSec() < max_data_age_) &&
      (odometry_.header.stamp.toSec() < max_data_age_);
}

void CrossingEscaper::odometry_callback(const nav_msgs::Odometry& odometry)
{
  odometry_ = odometry;
  has_data_ = (crossing_timestamp_.toSec() < max_data_age_) &&
    (direction_timestamp_.toSec() < max_data_age_);
}

bool CrossingEscaper::turnToAngle(const double direction, geometry_msgs::Twist& twist) const
{
  double start_yaw = tf::getYaw(start_position_.pose.pose.orientation);
  double yaw = tf::getYaw(odometry_.pose.pose.orientation);
  double already_turned_angle = angles::shortest_angular_distance(start_yaw, yaw);
  double dtheta = angles::shortest_angular_distance(already_turned_angle, direction_);
  ROS_DEBUG("dtheta to goal: %f", dtheta);

  double wz = kp_w_ * dtheta;

  twist.linear.x = 0;
  twist.angular.z = wz;
  return (std::abs(dtheta) < reach_angular_distance_);
}

bool CrossingEscaper::goToGoal(const geometry_msgs::Point& goal, geometry_msgs::Twist& twist) const
{
  if (traveled_distance_ > escape_distance_)
  {
    // Goal reached.
    twist = geometry_msgs::Twist();
    return true;
  }

  double distance = std::sqrt(goal.x * goal.x + goal.y * goal.y);

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

geometry_msgs::Point CrossingEscaper::goalFromOdometry()
{
  geometry_msgs::Point goal_from_start;
  goal_from_start.x = odometry_.pose.pose.position.x + escape_distance_ * std::cos(direction_);
  goal_from_start.y = odometry_.pose.pose.position.y + escape_distance_ * std::sin(direction_);

  geometry_msgs::Point goal;
  goal.x = goal_from_start.x - odometry_.pose.pose.position.x;
  goal.y = goal_from_start.y - odometry_.pose.pose.position.y;

  traveled_distance_ = std::sqrt((goal.x - goal_from_start.x) * (goal.x * goal_from_start.x) +
      (goal.x - goal_from_start.x) * (goal.x * goal_from_start.x));
  return goal;
}

} // namespace nj_escape_crossing
} // namespace lama
