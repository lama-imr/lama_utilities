#include <nj_escape_crossing/crossing_escaper.h>

namespace nj_escape_crossing {

const double CrossingEscaper::reach_angular_distance_ = 0.017;  // (rad), 1 deg
const double CrossingEscaper::threshold_w_only_ = 1.0;  // (rad), ~60 deg.

// Timeout for Odometry.
const ros::Duration CrossingEscaper::max_odometry_age_ = ros::Duration(0.1);

CrossingEscaper::CrossingEscaper(std::string name, double escape_distance) :
  lama_jockeys::NavigatingJockey(name),
  kp_v_(0.05),
  kp_w_(0.2),
  min_linear_velocity_(0.050),
  min_angular_velocity_(0.1),
  escape_distance_(escape_distance),
  exit_angle_topic_name_("exit_angle"),
  angle_reached_(false),
  goal_reached_(false),
  has_odometry_(false),
  has_direction_(false),
  crossing_interface_name_("crossing"),
  exit_angle_interface_name_("exit_angle")
{
  ros::NodeHandle private_nh("~");
  private_nh.getParam("kp_v", kp_v_);
  private_nh.getParam("kp_w", kp_w_);
  private_nh.getParam("min_linear_velocity", min_linear_velocity_);
  private_nh.getParam("min_angular_velocity", min_angular_velocity_);
  private_nh.getParam("escape_distance", escape_distance_);
  private_nh.getParam("crossing_interface_name", crossing_interface_name_);
  private_nh.getParam("exit_angle_interface_name", exit_angle_interface_name_);
  private_nh.getParam("exit_angle_topic_name", exit_angle_topic_name_);

  crossing_getter_ = nh_.serviceClient<lama_msgs::GetCrossing>(crossing_interface_name_ + "_getter");
  exit_angle_getter_ = nh_.serviceClient<lama_interfaces::GetDouble>(exit_angle_interface_name_ + "_getter");
}

/* First orient the robot, then travel at least "distance_to_escape_".
 */
void CrossingEscaper::onTraverse()
{
  // TODO: Add a mechanism to stop before distance_to_escape_ if
  // the robot sees only 2 crossing exits on a certain distance.
  
  ROS_DEBUG("%s: received action TRAVERSE", jockey_name_.c_str());
  
  if (escape_distance_ == 0)
  {
    // escape_distance_ not set, try to get escape distance from crossing_.
    if (!getCrossing())
    {
      ROS_ERROR("%s: escape_distance not set and no Crossing found, aborting...",
          jockey_name_.c_str());
      server_.setAborted();
    }
    if (crossing_.radius == 0)
    {
      // crossing_.radius set to 0, don't need to travel very far.
      ROS_WARN("Crossing descriptor to be used as escape distance but radius is 0, DONE");
      twist_publisher_.publish(geometry_msgs::Twist());
      result_.final_state = lama_jockeys::NavigateResult::DONE;
      result_.completion_time = ros::Duration(0);
      server_.setSucceeded(result_);
      return;
    }
    distance_to_escape_ = crossing_.radius;
  }
  else
  {
    distance_to_escape_ = escape_distance_;
  }
  ROS_DEBUG("Distance to escape: %.3f m", distance_to_escape_);

  twist_publisher_ = private_nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  odometry_subscriber_ = private_nh_.subscribe("odometry", 1, &CrossingEscaper::odometry_callback, this);

  if (!getExitAngle())
  {
    // No exit angle descriptor available, get the travel direction from topic exit_angle_topic_name_.
    ros::Subscriber direction_subscriber = private_nh_.subscribe(exit_angle_topic_name_, 1,
        &CrossingEscaper::direction_callback, this);

    // Wait for odometry (for the start position) and exit_angle data.
    ros::Duration(0.2).sleep();
    ros::spinOnce();
    while (!(has_odometry_ && has_direction_))
    {
      ros::spinOnce();
      ROS_WARN_STREAM_THROTTLE(5, "Waiting for odometry on topic " << odometry_subscriber_.getTopic() <<
          " and exit_angle on topic " << direction_subscriber.getTopic());
      ros::Duration(0.01).sleep();
    }
    has_odometry_ = false;
    has_direction_ = false;
    // Do not allow update of direction_ because we can't handle it.
    // shutdown is not necessary because direction_subscriber goes out of scope.
  }
  else
  {
    // Wait for odometry data (no exit_angle topic necessary).
    ros::Duration(0.2).sleep();
    ros::spinOnce();
    while (!has_odometry_)
    {
      ros::spinOnce();
      ROS_WARN_STREAM_THROTTLE(5, "Waiting for odometry on topic " << odometry_subscriber_.getTopic());
      ros::Duration(0.01).sleep();
    }
    has_odometry_ = false;
  }

  start_position_ = odometry_;
  feedback_.completion = 0.01;
  server_.publishFeedback(feedback_);

  geometry_msgs::Twist twist;

  ros::Rate r(50);
  while (ros::ok())
  {
    if (server_.isPreemptRequested() && !ros::ok())
    {
      ROS_INFO("%s: Preempted", jockey_name_.c_str());
      // set the action state to preempted
      server_.setPreempted();
      break;
    }

    // Odometry timeout mechanism.
    if ((ros::Time::now() - odometry_.header.stamp) > max_odometry_age_)
    {
      ROS_WARN("No Odometry received within %.3f s, setting Twist to 0 (odometry age: %.3f s)",
          max_odometry_age_.toSec(), odometry_.header.stamp.toSec());
        twist_publisher_.publish(geometry_msgs::Twist());
      continue;
    }

    if (!angle_reached_)
    {
      angle_reached_ = turnToAngle(direction_, twist);
      twist_publisher_.publish(twist);
      if (angle_reached_)
      {
        ROS_DEBUG("%s: angle reached", jockey_name_.c_str());
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
      // As for now, the jockey is pretty stupid and does not stop before the
      // distance is traveled. In the future, it should be able to recognize if
      // the current place (Crossing-type) is the same as the place it started
      // from.
      ROS_DEBUG("%s: goal reached", jockey_name_.c_str());
      result_.final_state = lama_jockeys::NavigateResult::DONE;
      result_.completion_time = ros::Time::now() - getStartTime() - getInterruptionsDuration();
      server_.setSucceeded(result_);
      break;
    }
    ros::spinOnce();
    r.sleep();
  }
  angle_reached_ = false;
  goal_reached_ = false;
}

void CrossingEscaper::onStop()
{
  twist_publisher_.publish(geometry_msgs::Twist());
  twist_publisher_.shutdown();
  result_.final_state = lama_jockeys::NavigateResult::DONE;
  result_.completion_time = ros::Duration(0);
  server_.setSucceeded(result_);
  angle_reached_ = false;
  goal_reached_ = false;
  has_odometry_ = false;
  has_direction_ = false;
}

void CrossingEscaper::odometry_callback(const nav_msgs::Odometry& odometry)
{
  odometry_ = odometry;
  has_odometry_ = true;
}

void CrossingEscaper::direction_callback(const std_msgs::Float32& direction)
{
  direction_ = direction.data;
  has_direction_ = true;
}

/* Set crossing_ to the descriptor found in the databse from information in goal.
 *
 * Set crossing_ to the descriptor associated with the first vertex of
 * goal_.edge, if goal._edge.id non-null. If goal._edge.id is null, get the
 * crossing with id goal_.descriptor_links[0].descriptor_id.
 *
 * Return false if edge does not exist or if no Crossing descriptor was found,
 * true otherwise.
 */
bool CrossingEscaper::getCrossing()
{
  if (goal_.edge.id != 0)
  {
    lama_interfaces::ActOnMap map_action;
    map_action.request.action = lama_interfaces::ActOnMapRequest::GET_DESCRIPTOR_LINKS;
    map_action.request.interface_name = crossing_interface_name_;
    if (goal_.edge.references.empty())
    {
      ROS_INFO("goal.edge (id: %d) should be have references, probably not an edge",
          goal_.edge.id);
      return false;
    }
    map_action.request.object.id = goal_.edge.references[0];
    map_action.request.interface_name = crossing_interface_name_;
    if (!map_agent_.call(map_action))
    {
      ROS_ERROR("Failed to call map agent");
      return false;
    }
    if (map_action.response.descriptor_links.empty())
    {
      ROS_DEBUG("No crossing descriptor for vertex %d", map_action.request.object.id);
      return false;
    }
    if (map_action.response.descriptor_links.size() > 1)
    {
      ROS_WARN("More than one crossing descriptor for vertex %d, taking the first one (id %d)",
          map_action.request.object.id, map_action.response.descriptor_links[0].descriptor_id);
    }
    return retrieveCrossingFromMap(map_action.response.descriptor_links[0].descriptor_id);
  }
  else
  {
    ROS_DEBUG("Goal.edge.id not set, getting descriptor id from goal.descriptor_link (id %d)",
        goal_.descriptor_link.descriptor_id);
    return retrieveCrossingFromMap(goal_.descriptor_link.descriptor_id);
  }
}

/* Retrieve a Crossing message from the map from its id.
 *
 * The message will be saved in crossing_.
 * Return false if the message in not found in the database.
 *
 * descriptor_id[in] id of the Crossing message in the database.
 */
bool CrossingEscaper::retrieveCrossingFromMap(const int32_t descriptor_id)
{
  lama_msgs::GetCrossing crossing_srv;
  crossing_srv.request.id = descriptor_id;
  if (!crossing_getter_.call(crossing_srv))
  {
    ROS_ERROR_STREAM(jockey_name_ << ": failed to get Crossing with id " <<
        descriptor_id << " and interface " << crossing_interface_name_ <<
        " (service " << crossing_getter_.getService() << ")");
    return false;
  }
  crossing_ = crossing_srv.response.descriptor;
  return true;
}

/* Set direction_ to the exit angle descriptor associated with goal_.edge
 *
 * Return false if no such descriptor was found, true otherwise.
 */
bool CrossingEscaper::getExitAngle()
{
  if (goal_.edge.id == 0)
  {
    // goal_.edge not set.
    return false;
  }

  lama_interfaces::ActOnMap map_action;
  map_action.request.action = lama_interfaces::ActOnMapRequest::GET_DESCRIPTOR_LINKS;
  map_action.request.object.id = goal_.edge.id;
  map_action.request.interface_name = exit_angle_interface_name_;
  map_agent_.call(map_action);
  if (map_action.response.descriptor_links.empty())
  {
    ROS_WARN_STREAM("No exit angle descriptor for edge " << map_action.request.object.id <<
        " on interface " << exit_angle_interface_name_);
    return false;
  }
  if (map_action.response.descriptor_links.size() > 1)
  {
    ROS_WARN("More than one exit angle descriptor for edge %d, taking the first one",
        map_action.request.object.id);
  }
  lama_interfaces::GetDouble exit_angle_srv;
  exit_angle_srv.request.id = map_action.response.descriptor_links[0].descriptor_id;
  if (!exit_angle_getter_.call(exit_angle_srv))
  {
    ROS_ERROR_STREAM(jockey_name_ << ": failed to get exit_angle with id " <<
        exit_angle_srv.request.id << " and interface " << exit_angle_interface_name_ <<
        " (service " << exit_angle_getter_.getService() << ")");
    return false;
  }
  direction_ = exit_angle_srv.response.descriptor;
  return true;
}

/* GoToGoal behavior for pure rotation
 *
 * direction[in] direction the robot should have at the end (in odometry_ frame).
 * twist[out] set velocity.
 */
bool CrossingEscaper::turnToAngle(const double direction, geometry_msgs::Twist& twist) const
{
  const double yaw_now= tf::getYaw(odometry_.pose.pose.orientation);
  const double dtheta = angles::shortest_angular_distance(yaw_now, direction_);
  ROS_DEBUG("dtheta to goal: %f", dtheta);

  const double wz = kp_w_ * dtheta;

  twist.linear.x = 0;
  twist.angular.z = wz;
  return (std::abs(dtheta) < reach_angular_distance_);
}

/* GoToGoal behavior
 *
 * goal[in] relative goal.
 * twist[out] set velocity.
 */
bool CrossingEscaper::goToGoal(const geometry_msgs::Point& goal, geometry_msgs::Twist& twist) const
{
  const double dx = odometry_.pose.pose.position.x - start_position_.pose.pose.position.x;
  const double dy = odometry_.pose.pose.position.y - start_position_.pose.pose.position.y;
  const double traveled_distance = std::sqrt(dx * dx + dy * dy);
  if (traveled_distance > distance_to_escape_)
  {
    // Goal reached.
    twist = geometry_msgs::Twist();
    return true;
  }

  ROS_DEBUG("goal: (%.3f, %.3f)", goal.x, goal.y);
  double distance = std::sqrt(goal.x * goal.x + goal.y * goal.y);

  double dtheta = std::atan2(goal.y, goal.x);

  if (std::fabs(dtheta) > threshold_w_only_)
  {
    // Do not go forward because the goal is not well in front of the robot.
    distance = 0.0;
  }

  // TODO: add a parameter "allow_backward" so that we don't need to turn 180 deg.
  
  double vx = kp_v_ * distance; 
  double wz = kp_w_ * dtheta;

  // Dead-zone management.
  if ((vx < min_linear_velocity_) && (std::abs(distance) > 1e-10) && (std::abs(wz) < min_angular_velocity_))
  {
    vx = min_linear_velocity_;
  }
  if ((wz > 0) && (wz < min_angular_velocity_) && (vx <= min_linear_velocity_))
  {
    wz = min_angular_velocity_;
  }
  else if ((wz < 0) && (wz > -min_angular_velocity_) && (vx <= min_linear_velocity_))
  {
    wz = -min_angular_velocity_;
  }
  ROS_DEBUG("Distance to goal: %f, dtheta to goal: %f, vx: %f, wz: %f", distance, dtheta, vx, wz);

  twist.linear.x = vx;
  twist.angular.z = wz;

  return false;
}

/* Return the relative goal from current robot position and start position
 */
geometry_msgs::Point CrossingEscaper::goalFromOdometry()
{
  geometry_msgs::Point abs_goal;
  abs_goal.x = start_position_.pose.pose.position.x + distance_to_escape_ * std::cos(direction_);
  abs_goal.y = start_position_.pose.pose.position.y + distance_to_escape_ * std::sin(direction_);

  geometry_msgs::Point rel_goal_abs_frame;
  rel_goal_abs_frame.x = abs_goal.x - odometry_.pose.pose.position.x;
  rel_goal_abs_frame.y = abs_goal.y - odometry_.pose.pose.position.y;

  const double yaw = tf::getYaw(odometry_.pose.pose.orientation);
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  geometry_msgs::Point rel_goal;
  rel_goal.x = rel_goal_abs_frame.x * cos_yaw + rel_goal_abs_frame.y * sin_yaw;
  rel_goal.y = -rel_goal_abs_frame.x * sin_yaw + rel_goal_abs_frame.y * cos_yaw;

  return rel_goal;
}

} // namespace nj_escape_crossing
